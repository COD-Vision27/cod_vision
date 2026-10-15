//! ROS2 图像捕获实现

use crate::capture::{
    CameraFov, CaptureBundle, CaptureSource, ImageHandle, compute_camera_intrinsics,
    driver::{
        CaptureConfig, CaptureFrameId, CapturedFrame, CapturedFrameKind, GpuCaptureHandler,
        SnapshotAsync, SnapshotSync,
    },
    setup_capture_camera, setup_preview_window, sync_capture_camera,
};
use crate::components::{Controlled, Infantry, InfantryGimbal, InfantryLaunchOffset};
use crate::ros2::image::compress_image;
use crate::ros2::prelude::M_ALIGN_MAT3;
use crate::ros2::topic::SpFrameTfTopic;
use crate::ros2::topic::{CameraInfoTopic, ImageCompressedTopic, ImageRawTopic, TopicPublisher};
use crate::systems::GameplaySystems;
use bevy::ecs::world::DeferredWorld;
use bevy::prelude::*;
use bevy::render::{Extract, ExtractSchedule, RenderApp, RenderSystems};
use r2r::Clock;
use r2r::geometry_msgs::msg::{Quaternion, Transform as RosTransform, TransformStamped, Vector3};
use r2r::sensor_msgs::msg::{CameraInfo, RegionOfInterest};
use r2r::std_msgs::msg::Header;
use r2r::tf2_msgs::msg::TFMessage;
use std::cell::RefCell;
use std::sync::{Arc, Mutex};

struct RosSnapshotSync {
    stamp: RefCell<r2r::builtin_interfaces::msg::Time>,
    frame_tf: TFMessage,
}

impl SnapshotSync for RosSnapshotSync {
    fn captured(
        self: Box<Self>,
        world: &mut DeferredWorld,
        _config: &CaptureConfig,
    ) -> Box<dyn SnapshotAsync> {
        Box::new(RosSnapshot {
            stamp: self.stamp,
            frame_tf: self.frame_tf,
            ctx: world.resource::<RosCaptureContextShared>().0.clone(),
        })
    }
}

struct RosSnapshot {
    stamp: RefCell<r2r::builtin_interfaces::msg::Time>,
    frame_tf: TFMessage,
    ctx: Arc<RosCaptureContext>,
}

impl SnapshotAsync for RosSnapshot {
    fn captured(&mut self, frame: CapturedFrame<'_>) {
        if frame.kind != CapturedFrameKind::Rgb8 {
            return;
        }

        let optical_frame_hdr = Header {
            stamp: self.stamp.take(),
            frame_id: "camera_optical_frame".to_string(),
        };
        // Pose and image were extracted from the same rendered frame; GPU readback
        // may finish later. Never stamp these with the readback completion time.
        self.ctx.frame_tf.publish(self.frame_tf.clone());
        self.ctx.camera_info.publish(ros_camera_info(
            optical_frame_hdr.clone(),
            frame.width,
            frame.height,
            self.ctx.fov_y,
        ));
        if self.ctx.publish_compressed {
            self.ctx.image_compressed.publish(compress_image(
                optical_frame_hdr,
                frame.width,
                frame.height,
                frame.data,
            ));
        } else {
            self.ctx.image_raw.publish(raw_image(
                optical_frame_hdr,
                frame.width,
                frame.height,
                frame.data,
            ));
        }
    }
}

#[derive(Default)]
struct RosSnapshotCreator {}

impl GpuCaptureHandler for RosSnapshotCreator {
    fn captured(
        &self,
        world: &World,
        _frame_id: Option<CaptureFrameId>,
    ) -> Option<Box<dyn SnapshotSync>> {
        let pose = world.resource::<ExtractedSpFrame>();
        let frame_tf = pose.tf.clone()?;
        Some(Box::new(RosSnapshotSync {
            stamp: RefCell::new(pose.stamp.clone()),
            frame_tf,
        }))
    }
}

#[derive(Resource, Clone, Deref, DerefMut)]
pub struct RosCaptureContextShared(Arc<RosCaptureContext>);

#[derive(Resource, Clone)]
pub struct RosCaptureContext {
    pub clock: Arc<Mutex<Clock>>,
    pub fov_y: f32,
    pub publish_compressed: bool,
    pub camera_info: TopicPublisher<CameraInfoTopic>,
    pub image_raw: TopicPublisher<ImageRawTopic>,
    pub image_compressed: TopicPublisher<ImageCompressedTopic>,
    pub frame_tf: TopicPublisher<SpFrameTfTopic>,
}

pub struct RosCapturePlugin {
    pub config: CaptureConfig,
    pub context: RosCaptureContext,
}

impl Plugin for RosCapturePlugin {
    fn build(&self, app: &mut App) {
        let capture = CaptureBundle::color(
            app,
            self.config.clone(),
            vec![Box::new(RosSnapshotCreator::default())],
        );
        let render_target_handle = capture.color_target().unwrap().clone();

        app.add_plugins(capture)
            .insert_resource(ImageHandle(render_target_handle))
            .insert_resource(CameraFov(self.context.fov_y))
            .insert_resource(self.context.clone())
            .add_systems(Startup, setup_capture_camera)
            .add_systems(Startup, setup_preview_window)
            .add_systems(
                Update,
                sync_capture_camera
                    .after(GameplaySystems::Camera)
                    .before(RenderSystems::Render),
            );
        app.sub_app_mut(RenderApp)
            .insert_resource(RosCaptureContextShared(Arc::new(self.context.clone())))
            .insert_resource(self.context.clone())
            .init_resource::<ExtractedSpFrame>()
            .add_systems(ExtractSchedule, extract_sp_frame);
    }
}

#[derive(Resource, Default)]
struct ExtractedSpFrame {
    stamp: r2r::builtin_interfaces::msg::Time,
    tf: Option<TFMessage>,
}

fn ros_tf(
    stamp: &r2r::builtin_interfaces::msg::Time,
    parent: &str,
    child: &str,
    translation: Vec3,
    rotation: Quat,
) -> TransformStamped {
    TransformStamped {
        header: Header {
            stamp: stamp.clone(),
            frame_id: parent.into(),
        },
        child_frame_id: child.into(),
        transform: RosTransform {
            translation: Vector3 {
                x: translation.x as f64,
                y: translation.y as f64,
                z: translation.z as f64,
            },
            rotation: Quaternion {
                x: rotation.x as f64,
                y: rotation.y as f64,
                z: rotation.z as f64,
                w: rotation.w as f64,
            },
        },
    }
}

fn extract_sp_frame(
    mut frame: ResMut<ExtractedSpFrame>,
    context: Extract<Res<RosCaptureContext>>,
    camera: Extract<Query<&GlobalTransform, With<CaptureSource>>>,
    gimbal: Extract<Query<&GlobalTransform, (With<Controlled>, With<InfantryGimbal>)>>,
    muzzle: Extract<Query<&Transform, (With<Controlled>, With<InfantryLaunchOffset>)>>,
    robot: Extract<Query<&Transform, (With<Controlled>, With<Infantry>)>>,
) {
    frame.tf = None;
    let (Ok(camera), Ok(gimbal), Ok(muzzle), Ok(robot)) = (
        camera.single(),
        gimbal.single(),
        muzzle.single(),
        robot.single(),
    ) else {
        return;
    };
    let stamp = Clock::to_builtin_time(&context.clock.lock().unwrap().get_now().unwrap());
    let align = Quat::from_mat3(&M_ALIGN_MAT3);
    // Same origin and forward axis as projectile_launch. Camera follows this basis.
    let gun_basis =
        (gimbal.rotation() * muzzle.rotation * Quat::from_rotation_x(std::f32::consts::FRAC_PI_2))
            .normalize();
    let origin = robot.translation + gimbal.rotation() * muzzle.translation;
    let gun_ros = (align * gun_basis * align.inverse()).normalize();
    let camera_optical_ros =
        (align * camera.rotation() * Quat::from_rotation_x(std::f32::consts::PI)).normalize();
    let camera_rel_rotation = (gun_ros.inverse() * camera_optical_ros).normalize();
    let camera_rel_position =
        M_ALIGN_MAT3 * (gun_basis.inverse() * (camera.translation() - origin));
    frame.tf = Some(TFMessage {
        transforms: vec![
            ros_tf(&stamp, "map", "sp_gimbal", M_ALIGN_MAT3 * origin, gun_ros),
            ros_tf(
                &stamp,
                "sp_gimbal",
                "sp_camera_optical",
                camera_rel_position,
                camera_rel_rotation,
            ),
        ],
    });
    frame.stamp = stamp;
}

fn raw_image(hdr: Header, width: u32, height: u32, data: &[u8]) -> r2r::sensor_msgs::msg::Image {
    r2r::sensor_msgs::msg::Image {
        header: hdr,
        height,
        width,
        encoding: "rgb8".to_string(),
        is_bigendian: 0,
        step: width * 3,
        data: Vec::from(data),
    }
}

fn ros_camera_info(hdr: Header, width: u32, height: u32, fov_y: f32) -> CameraInfo {
    let intrinsics = compute_camera_intrinsics(width, height, fov_y);

    CameraInfo {
        header: hdr,
        height,
        width,
        distortion_model: "plumb_bob".to_string(),
        d: vec![0.000, 0.000, 0.000, 0.000, 0.000],
        k: vec![
            intrinsics.fx,
            0.0,
            intrinsics.cx,
            0.0,
            intrinsics.fy,
            intrinsics.cy,
            0.0,
            0.0,
            1.0,
        ],
        p: vec![
            intrinsics.fx,
            0.0,
            intrinsics.cx,
            0.0,
            0.0,
            intrinsics.fy,
            intrinsics.cy,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ],
        r: vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        binning_x: 0,
        binning_y: 0,
        roi: RegionOfInterest {
            x_offset: 0,
            y_offset: 0,
            height,
            width,
            do_rectify: true,
        },
    }
}
