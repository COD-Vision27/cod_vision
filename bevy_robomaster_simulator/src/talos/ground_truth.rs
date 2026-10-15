use crate::capture::{CaptureSource, IMAGE_HEIGHT, IMAGE_WIDTH};
use crate::components::{Controlled, Infantry};
use crate::metrics::{GroundTruthProjection, SimulationMetrics};
use crate::robomaster::prelude::{
    Activation, Armor, ArmorRoot, MechanismState, PowerRune, PowerRuneMechanism, PowerRuneRotation,
    RuneMode, Team,
};
use crate::talos::capture::{TalosCaptureContext, TalosFrameStamp};
use crate::talos::plugin::M_ALIGN_MAT3;
use avian3d::prelude::AngularVelocity;
use bevy::prelude::*;
use talos_ipc::*;

fn to_ros_vec3(v: Vec3) -> Vec3 {
    M_ALIGN_MAT3 * v
}

fn team_to_u8(team: &Team) -> u8 {
    match team {
        Team::Red => 0,
        Team::Blue => 1,
    }
}

fn activation_to_u8(a: &Activation) -> u8 {
    match a {
        Activation::Deactivated => 0,
        Activation::Activating => 1,
        Activation::Activated => 2,
        Activation::Completed => 3,
    }
}

fn mechanism_state_to_u8(s: &MechanismState) -> u8 {
    match s {
        MechanismState::Inactive { .. } => 0,
        MechanismState::Activating(_) => 1,
        MechanismState::Activated { .. } => 2,
        MechanismState::Failed { .. } => 3,
    }
}

fn rune_mode_to_u8(m: &RuneMode) -> u8 {
    match m {
        RuneMode::Small => 0,
        RuneMode::Large => 1,
    }
}

/// Compute yaw in the ROS reference frame from a Bevy GlobalTransform.
///
/// The alignment matrix maps Bevy (Y-up) → ROS (Z-up).
/// We convert the rotation quaternion through the alignment to extract the Z-up yaw.
fn ros_yaw(global_tf: &GlobalTransform) -> f32 {
    let align_quat = Quat::from_mat3(&M_ALIGN_MAT3);
    let ros_rot = align_quat * global_tf.rotation() * align_quat.inverse();
    let (yaw, _, _) = ros_rot.to_euler(EulerRot::ZYX);
    yaw
}

/// Project a world-space target through the same camera convention used by the
/// captured RGB stream.  ROS coordinates use +X forward, +Y left and +Z up.
fn project_target(
    camera: &GlobalTransform,
    world_position: Vec3,
    fov_y: f32,
) -> Option<GroundTruthProjection> {
    let camera_position_bevy =
        camera.rotation().inverse() * (world_position - camera.translation());
    let camera_position_ros = to_ros_vec3(camera_position_bevy);
    let forward = camera_position_ros.x;
    if !camera_position_ros.is_finite() || !fov_y.is_finite() || forward <= 1.0e-4 {
        return None;
    }

    let fov_y = fov_y.clamp(1.0e-3, std::f32::consts::PI - 1.0e-3);
    let fy = IMAGE_HEIGHT as f32 / (2.0 * (fov_y * 0.5).tan());
    let fx = fy;
    let cx = IMAGE_WIDTH as f32 * 0.5;
    let cy = IMAGE_HEIGHT as f32 * 0.5;
    let pixel_u = fx * camera_position_ros.y / forward + cx;
    let pixel_v = cy - fy * camera_position_ros.z / forward;
    let yaw_deg = camera_position_ros.y.atan2(forward).to_degrees();
    let pitch_deg = camera_position_ros
        .z
        .atan2(camera_position_ros.x.hypot(camera_position_ros.y))
        .to_degrees();
    Some(GroundTruthProjection {
        camera_position_bevy,
        camera_position_ros,
        yaw_deg,
        pitch_deg,
        pixel_u,
        pixel_v,
        visible: pixel_u >= 0.0
            && pixel_u < IMAGE_WIDTH as f32
            && pixel_v >= 0.0
            && pixel_v < IMAGE_HEIGHT as f32,
    })
}

pub fn publish_ground_truth_system(
    context: Option<Res<TalosCaptureContext>>,
    frame_stamp: Res<TalosFrameStamp>,
    mut metrics: ResMut<SimulationMetrics>,
    camera_query: Query<&GlobalTransform, With<CaptureSource>>,
    infantry_query: Query<(
        Entity,
        &GlobalTransform,
        Option<&AngularVelocity>,
        Option<&Controlled>,
        &Infantry,
    )>,
    armor_query: Query<(Entity, &GlobalTransform, &ArmorRoot, &Armor)>,
    controlled_query: Query<Entity, With<Controlled>>,
    child_of: Query<&ChildOf>,
    rune_query: Query<(
        &GlobalTransform,
        &Transform,
        &PowerRune,
        &PowerRuneMechanism,
        &PowerRuneRotation,
    )>,
) {
    let frame_seq = frame_stamp.frame_seq;
    let timestamp_ns = frame_stamp.timestamp_ns;
    metrics.set_talos_frame_stamp(frame_seq, timestamp_ns);

    let Some(ctx) = context else {
        return;
    };
    let camera = camera_query.single().ok();

    let mut batch = GroundTruthBatch::default();
    batch.frame_seq = frame_seq;
    batch.timestamp_ns = timestamp_ns;

    // Collect robot ground truth from all infantry robots
    for (entity, global_tf, ang_vel, controlled, infantry) in infantry_query.iter() {
        let pos_ros = to_ros_vec3(global_tf.translation());
        let pos_bevy = global_tf.translation();
        let team = &infantry.team;
        let config = infantry.config;

        let vyaw = ang_vel
            .map(|av| {
                let ros_ang = to_ros_vec3(av.0);
                ros_ang.z
            })
            .unwrap_or(0.0);

        let yaw = ros_yaw(global_tf);

        if (batch.target_count as usize) < GROUND_TRUTH_MAX_TARGETS {
            let idx = batch.target_count as usize;
            let team_id = team_to_u8(team);
            let label = config.armor.label() as u8;
            batch.targets[idx] = GroundTruthTarget {
                frame_seq,
                timestamp_ns,
                team: team_id,
                armor_label: label,
                is_outpost: 0,
                _pad1: 0,
                position: [pos_ros.x, pos_ros.y, pos_ros.z],
                vyaw,
                yaw,
                _pad: [0; 24],
            };
            metrics.record_ground_truth_target(
                frame_seq,
                timestamp_ns,
                idx as u32,
                entity.to_bits(),
                team_id,
                label,
                config.armor.armor_type() as u8,
                config.armor_count as u32,
                controlled.is_some(),
                pos_bevy,
                pos_ros,
                camera.and_then(|camera| project_target(camera, pos_bevy, ctx.fov_y)),
                vyaw,
                yaw,
            );
            batch.target_count += 1;
        }
    }

    // ArmorRoot marks the single logical entity for each visible armor.  Query
    // it directly so mesh children cannot duplicate image-level ground truth.
    for (entity, global_tf, armor_root, armor) in armor_query.iter() {
        let Ok(armor_id) = u32::try_from(armor_root.id.as_usize()) else {
            continue;
        };
        let pos_bevy = global_tf.translation();
        let pos_ros = to_ros_vec3(pos_bevy);
        let is_controlled = child_of
            .iter_ancestors(entity)
            .any(|ancestor| controlled_query.get(ancestor).is_ok());
        metrics.record_ground_truth_armor(
            frame_seq,
            timestamp_ns,
            entity.to_bits(),
            armor_id,
            team_to_u8(&armor.team),
            armor.label as u8,
            armor.spec.armor_type() as u8,
            is_controlled,
            pos_bevy,
            pos_ros,
            camera.and_then(|camera| project_target(camera, pos_bevy, ctx.fov_y)),
        );
    }

    // Collect rune ground truth
    for (global_tf, local_tf, power_rune, mechanism, rotation) in rune_query.iter() {
        if (batch.rune_count as usize) >= GROUND_TRUTH_MAX_RUNES {
            break;
        }

        let pos_ros = to_ros_vec3(global_tf.translation());

        // Extract current rotation angle around the actual rune axis (-1, 0, -1).
        // The rune rotates via `rotate_local_axis(direction, angle)`, so we must
        // project the quaternion back onto that axis — not extract an Euler X angle.
        let rune_axis = Dir3::from_xyz(-1.0, 0.0, -1.0).unwrap();
        let (axis, angle) = local_tf.rotation.to_axis_angle();
        let current_angle = angle * axis.dot(*rune_axis).signum();

        let controller = rotation.controller();
        let direction = if controller.is_clockwise() { 1 } else { -1 };

        let (sin_amplitude, sin_omega, relative_time, sin_offset) = controller
            .variable_params()
            .map(|(a, omega, t)| (a, omega, t, 2.090 - a))
            .unwrap_or((0.0, 0.0, 0.0, 0.0));

        let mut target_activations = [0u8; 5];
        for (i, a) in mechanism.state().target_states().iter().enumerate() {
            if i < 5 {
                target_activations[i] = activation_to_u8(a);
            }
        }

        let idx = batch.rune_count as usize;
        batch.runes[idx] = GroundTruthRune {
            frame_seq,
            timestamp_ns,
            team: team_to_u8(&power_rune.team()),
            rune_mode: rune_mode_to_u8(&power_rune.mode()),
            mechanism_state: mechanism_state_to_u8(mechanism.state()),
            _pad1: 0,
            r_center_odom: [pos_ros.x, pos_ros.y, pos_ros.z],
            radius: 0.0,
            current_angle,
            v_roll: 0.0,
            direction,
            sin_amplitude,
            sin_omega,
            sin_phase: 0.0,
            sin_offset,
            relative_time,
            blade_id: -1,
            target_activations,
            _pad: [0; 20],
        };
        batch.rune_count += 1;
    }

    if let Ok(mut publisher) = ctx.publisher.lock() {
        publisher.publish_ground_truth(&batch);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ros_yaw_reads_z_axis_from_zyx_decomposition() {
        let angle = 0.63;
        let align_quat = Quat::from_mat3(&M_ALIGN_MAT3);
        let bevy_rotation = align_quat.inverse() * Quat::from_rotation_z(angle) * align_quat;
        let transform = GlobalTransform::from_rotation(bevy_rotation);
        assert!((ros_yaw(&transform) - angle).abs() < 1.0e-5);
    }
}
