use bevy::prelude::*;

use crate::components::{Controlled, InfantryChassis, InfantryGimbal, InfantryLaunchOffset};
use crate::config::{GimbalAxisPidConfig, SimulationConfig};

/// Absolute muzzle-frame aim target produced by an external auto-aim solver.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GimbalAimTarget {
    pub yaw: f32,
    pub pitch: f32,
}

impl GimbalAimTarget {
    /// Solver commands arrive in degrees with pitch measured from the vertical axis.
    pub fn from_solver_degrees(yaw_deg: f32, pitch_deg: f32) -> Self {
        Self {
            yaw: yaw_deg.to_radians(),
            pitch: (pitch_deg - 90.0).to_radians(),
        }
    }

    fn rotation(self) -> Quat {
        Quat::from_euler(EulerRot::YXZ, self.yaw, self.pitch, 0.0)
    }
}

/// Check the physical +Y firing axis, using the pose the projectile launcher uses.
/// Image-associated feedback is delayed and cannot replace this final actuator check.
pub fn sp_fire_aligned(target: GimbalAimTarget, muzzle_world: Quat, tolerance: f32) -> bool {
    let expected = target.rotation() * Vec3::Y;
    let actual = muzzle_world * Vec3::Y;
    expected.is_finite() && actual.is_finite()
        && actual.normalize_or_zero().dot(expected.normalize_or_zero()) >= tolerance.cos()
}

/// One independent single-axis PID loop.
#[derive(Clone, Copy, Debug, Default)]
struct AxisPid {
    integral: f32,
    previous_error: f32,
}

impl AxisPid {
    /// Angular rate command (rad/s) for this axis given the current error.
    fn step(&mut self, error: f32, config: &GimbalAxisPidConfig, dt: f32) -> f32 {
        self.integral =
            (self.integral + error * dt).clamp(-config.integral_limit, config.integral_limit);
        let derivative = (error - self.previous_error) / dt;
        self.previous_error = error;

        let rate = error * config.kp + self.integral * config.ki + derivative * config.kd;
        rate.clamp(-config.max_rate, config.max_rate)
    }
}

/// Closed-loop state for tracking a solver target. Its presence on the gimbal *is*
/// the "auto-aim has a target" state: no tracker means no target and no PID output,
/// and a freshly inserted tracker starts from clean integrator/derivative state.
#[derive(Component, Clone, Copy, Debug)]
pub struct GimbalAimTracker {
    target: GimbalAimTarget,
    yaw: AxisPid,
    pitch: AxisPid,
    motion: Option<SpMotionPlan>,
}

/// SP trajectory derivatives in the simulator's absolute muzzle convention.
#[derive(Clone, Copy, Debug)]
pub struct SpMotionPlan {
    velocity: Vec2,
    acceleration: Vec2,
    max_acceleration: Vec2,
    valid_for: f32,
    elapsed: f32,
    previous_rate: Vec2,
}

impl SpMotionPlan {
    pub fn new(
        yaw_vel: f32,
        pitch_vel: f32,
        yaw_acc: f32,
        pitch_acc: f32,
        max_yaw_acc: f32,
        max_pitch_acc: f32,
        valid_for: f32,
    ) -> Self {
        Self {
            velocity: Vec2::new(yaw_vel, pitch_vel),
            acceleration: Vec2::new(yaw_acc, pitch_acc),
            max_acceleration: Vec2::new(max_yaw_acc, max_pitch_acc),
            valid_for,
            elapsed: 0.0,
            previous_rate: Vec2::ZERO,
        }
    }

    fn sample(&self, initial: GimbalAimTarget) -> GimbalAimTarget {
        let offset =
            self.velocity * self.elapsed + self.acceleration * (0.5 * self.elapsed * self.elapsed);
        GimbalAimTarget {
            yaw: initial.yaw + offset.x,
            pitch: initial.pitch + offset.y,
        }
    }
}

impl GimbalAimTracker {
    pub fn new(target: GimbalAimTarget) -> Self {
        Self {
            target,
            yaw: AxisPid::default(),
            pitch: AxisPid::default(),
            motion: None,
        }
    }

    pub fn target(&self) -> GimbalAimTarget {
        self.target
    }

    /// Update the setpoint while keeping the loop state, so a stream of commands
    /// drives one continuous controller instead of restarting every message.
    pub fn retarget(&mut self, target: GimbalAimTarget) {
        self.target = target;
        self.motion = None;
    }

    pub fn retarget_sp(&mut self, target: GimbalAimTarget, mut motion: SpMotionPlan) {
        if let Some(previous) = self.motion {
            motion.previous_rate = previous.previous_rate;
        }
        self.target = target;
        self.motion = Some(motion);
    }
}

fn wrap_angle(angle: f32) -> f32 {
    let wrapped = (angle + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU);
    wrapped - std::f32::consts::PI
}

/// Tracking error as `(yaw, pitch)` in the gimbal's own local frame, i.e. in the same
/// coordinates as `InfantryGimbal`. The solver target aims the *muzzle* in world space,
/// so the muzzle's fixed mount offset and the chassis rotation are divided out first —
/// measuring the error in world space instead flips the pitch sign for any mount whose
/// local axes disagree with the world axes.
fn tracking_error(
    target_rotation: Quat,
    gimbal_local: Quat,
    gimbal_world: Quat,
    muzzle_world: Quat,
) -> Vec2 {
    // World-space correction that would put the muzzle on target right now.
    let correction = target_rotation * muzzle_world.inverse();
    // Same correction expressed as the gimbal's desired local rotation.
    let desired_local = gimbal_local * gimbal_world.inverse() * correction * gimbal_world;

    let (desired_yaw, desired_pitch, _) = desired_local.to_euler(EulerRot::YXZ);
    let (current_yaw, current_pitch, _) = gimbal_local.to_euler(EulerRot::YXZ);

    Vec2::new(
        wrap_angle(desired_yaw - current_yaw),
        wrap_angle(desired_pitch - current_pitch),
    )
}

/// Replaces the direct pose snap that auto-aim used to apply: the solver target is
/// tracked by a rate-limited PID loop so the gimbal moves like an actuated axis.
pub fn gimbal_pid_controls(
    mut commands: Commands,
    time: Res<Time>,
    config: Res<SimulationConfig>,
    gimbal: Option<
        Single<
            (
                Entity,
                &mut Transform,
                &GlobalTransform,
                &mut InfantryGimbal,
                &mut GimbalAimTracker,
            ),
            (
                With<Controlled>,
                Without<InfantryChassis>,
                Without<InfantryLaunchOffset>,
            ),
        >,
    >,
    muzzle: Option<Single<&GlobalTransform, (With<InfantryLaunchOffset>, With<Controlled>)>>,
) {
    let (Some(gimbal), Some(muzzle)) = (gimbal, muzzle) else {
        return;
    };
    let dt = time.delta_secs();
    if dt <= 0.0 {
        return;
    }

    let (entity, mut gimbal_transform, gimbal_global, mut gimbal_data, mut tracker) =
        gimbal.into_inner();
    // Integrate the actuator at <=10 ms even when rendering/ROS readback stalls.
    // Large render deltas must not turn the discrete PID into a pose jump.
    let chassis_rotation = gimbal_global.rotation() * gimbal_transform.rotation.inverse();
    let mount_rotation = gimbal_global.rotation().inverse() * muzzle.rotation();
    let steps = if tracker.motion.is_some() { (dt / 0.01).ceil().max(1.0) as usize } else { 1 };
    let dt = dt / steps as f32;
    for _ in 0..steps {
        let world_rotation = chassis_rotation * gimbal_transform.rotation;
        let muzzle_rotation = world_rotation * mount_rotation;
        if tracker.motion.is_some_and(|m| m.elapsed >= m.valid_for) {
            commands.entity(entity).remove::<GimbalAimTracker>();
            break;
        }
        let sampled_target = tracker
            .motion
            .map_or(tracker.target(), |m| m.sample(tracker.target()));

        // Error is measured on the muzzle, so chassis motion shows up as tracking error
        // instead of being cancelled out by a one-shot correction.
        let error = tracking_error(
            sampled_target.rotation(),
            gimbal_transform.rotation,
            world_rotation,
            muzzle_rotation,
        );
        let (yaw_error, pitch_error) = (error.x, error.y);
        let pid = &config.vehicle.gimbal_pid;
        let mut yaw_rate = tracker.yaw.step(yaw_error, &pid.yaw, dt);
        let mut pitch_rate = tracker.pitch.step(pitch_error, &pid.pitch, dt);
        if let Some(mut motion) = tracker.motion {
            // Express world trajectory derivatives in local gimbal axes through the
            // same mount/chassis conversion as the position feedback loop.
            let velocity = motion.velocity + motion.acceleration * motion.elapsed;
            let epsilon = 0.001;
            let next = GimbalAimTarget {
                yaw: sampled_target.yaw + velocity.x * epsilon,
                pitch: sampled_target.pitch + velocity.y * epsilon,
            };
            let next_error = tracking_error(
                next.rotation(),
                gimbal_transform.rotation,
                world_rotation,
                muzzle_rotation,
            );
            let feedforward = Vec2::new(
                wrap_angle(next_error.x - error.x),
                wrap_angle(next_error.y - error.y),
            ) / epsilon;
            let requested = Vec2::new(yaw_rate + feedforward.x, pitch_rate + feedforward.y);
            let delta = (requested - motion.previous_rate)
                .clamp(-motion.max_acceleration * dt, motion.max_acceleration * dt);
            let rate = motion.previous_rate + delta;
            yaw_rate = rate.x.clamp(-pid.yaw.max_rate, pid.yaw.max_rate);
            pitch_rate = rate.y.clamp(-pid.pitch.max_rate, pid.pitch.max_rate);
            motion.previous_rate = Vec2::new(yaw_rate, pitch_rate);
            motion.elapsed += dt;
            tracker.motion = Some(motion);
        }

        (gimbal_data.local_yaw, gimbal_data.pitch, _) =
            gimbal_transform.rotation.to_euler(EulerRot::YXZ);
        gimbal_data.local_yaw += yaw_rate * dt;
        gimbal_data.pitch = (gimbal_data.pitch + pitch_rate * dt).clamp(
            -config.vehicle.gimbal_pitch_limit,
            config.vehicle.gimbal_pitch_limit,
        );

        gimbal_transform.rotation =
            Quat::from_euler(EulerRot::YXZ, gimbal_data.local_yaw, gimbal_data.pitch, 0.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn axis(kp: f32, max_rate: f32) -> GimbalAxisPidConfig {
        GimbalAxisPidConfig {
            kp,
            ki: 0.0,
            kd: 0.0,
            integral_limit: 1.0,
            max_rate,
        }
    }

    #[test]
    fn proportional_rate_follows_error_sign_and_magnitude() {
        let mut pid = AxisPid::default();

        assert!((pid.step(0.2, &axis(10.0, 20.0), 0.01) - 2.0).abs() < 1e-5);
        assert!((pid.step(-0.1, &axis(10.0, 20.0), 0.01) + 1.0).abs() < 1e-5);
    }

    #[test]
    fn each_axis_uses_its_own_gains_and_rate_limit() {
        let mut tracker = GimbalAimTracker::new(GimbalAimTarget {
            yaw: 0.0,
            pitch: 0.0,
        });
        let yaw_config = axis(10.0, 20.0);
        let pitch_config = axis(4.0, 1.0);

        let yaw_rate = tracker.yaw.step(0.5, &yaw_config, 0.01);
        let pitch_rate = tracker.pitch.step(0.5, &pitch_config, 0.01);

        assert!((yaw_rate - 5.0).abs() < 1e-5);
        assert_eq!(pitch_rate, 1.0);
    }

    #[test]
    fn axis_loop_state_is_not_shared_between_yaw_and_pitch() {
        let mut tracker = GimbalAimTracker::new(GimbalAimTarget {
            yaw: 0.0,
            pitch: 0.0,
        });
        let config = axis(0.0, 20.0);

        tracker.yaw.step(1.0, &config, 0.5);

        assert_eq!(tracker.yaw.integral, 0.5);
        assert_eq!(tracker.pitch.integral, 0.0);
        assert_eq!(tracker.pitch.previous_error, 0.0);
    }

    #[test]
    fn integral_term_is_bounded_against_windup() {
        let mut config = axis(0.0, 20.0);
        config.ki = 1.0;
        let mut pid = AxisPid::default();

        for _ in 0..1000 {
            pid.step(1.0, &config, 0.01);
        }

        assert_eq!(pid.integral, config.integral_limit);
    }

    #[test]
    fn retarget_keeps_loop_state_so_command_stream_is_continuous() {
        let mut tracker = GimbalAimTracker::new(GimbalAimTarget {
            yaw: 0.0,
            pitch: 0.0,
        });
        tracker.yaw.step(0.1, &axis(10.0, 20.0), 0.01);
        let integral = tracker.yaw.integral;

        tracker.retarget(GimbalAimTarget {
            yaw: 0.5,
            pitch: 0.1,
        });

        assert_eq!(tracker.yaw.integral, integral);
        assert_eq!(tracker.target().yaw, 0.5);
    }

    #[test]
    fn error_points_the_same_way_as_the_local_angles_on_a_bare_gimbal() {
        let target = Quat::from_euler(EulerRot::YXZ, 0.3, -0.2, 0.0);
        let error = tracking_error(target, Quat::IDENTITY, Quat::IDENTITY, Quat::IDENTITY);

        assert!((error.x - 0.3).abs() < 1e-5);
        assert!((error.y + 0.2).abs() < 1e-5);
    }

    /// Guards the pitch sign. The gimbal's pitch axis is its *local* X, which points
    /// against world X once the gimbal has yawed past 90 degrees; an error read off the
    /// world-space correction is inverted there and drives pitch away from the target.
    #[test]
    fn pitch_error_keeps_its_sign_when_the_gimbal_faces_backwards() {
        let local = Quat::from_rotation_y(std::f32::consts::PI);
        let target = local * Quat::from_rotation_x(0.1);
        let error = tracking_error(target, local, local, local);

        assert!(error.x.abs() < 1e-5, "yaw leaked {}", error.x);
        assert!((error.y - 0.1).abs() < 1e-5, "pitch error was {}", error.y);
    }

    #[test]
    fn closed_loop_converges_through_a_mount_offset_and_chassis_yaw() {
        let chassis = Quat::from_rotation_y(2.8);
        let mount = Quat::from_euler(EulerRot::YXZ, 0.4, 0.25, 0.15);
        let goal = Quat::from_euler(EulerRot::YXZ, 0.3, -0.2, 0.0);
        let target = chassis * goal * mount;

        let mut tracker = GimbalAimTracker::new(GimbalAimTarget {
            yaw: 0.0,
            pitch: 0.0,
        });
        let yaw_config = axis(10.0, 20.0);
        let pitch_config = axis(6.0, 20.0);
        let mut local = Quat::IDENTITY;
        let dt = 1.0 / 120.0;

        for _ in 0..600 {
            let gimbal_world = chassis * local;
            let error = tracking_error(target, local, gimbal_world, gimbal_world * mount);
            let (mut yaw, mut pitch, _) = local.to_euler(EulerRot::YXZ);
            yaw += tracker.yaw.step(error.x, &yaw_config, dt) * dt;
            pitch += tracker.pitch.step(error.y, &pitch_config, dt) * dt;
            local = Quat::from_euler(EulerRot::YXZ, yaw, pitch, 0.0);
        }

        let (yaw, pitch, _) = local.to_euler(EulerRot::YXZ);
        assert!((yaw - 0.3).abs() < 1e-3, "yaw settled at {yaw}");
        assert!((pitch + 0.2).abs() < 1e-3, "pitch settled at {pitch}");
    }

    #[test]
    fn wrap_angle_takes_the_short_way_around() {
        assert!((wrap_angle(std::f32::consts::TAU + 0.2) - 0.2).abs() < 1e-5);
        assert!((wrap_angle(std::f32::consts::PI + 0.1) + std::f32::consts::PI - 0.1).abs() < 1e-5);
    }

    #[test]
    fn sp_zero_pitch_points_the_projectile_horizontally() {
        let target = GimbalAimTarget {
            yaw: 0.0,
            pitch: -std::f32::consts::FRAC_PI_2,
        };
        assert!((target.rotation() * Vec3::Y - Vec3::NEG_Z).length() < 1e-6);
        let up = GimbalAimTarget {
            yaw: 0.0,
            pitch: -std::f32::consts::FRAC_PI_2 + 0.2,
        };
        assert!((up.rotation() * Vec3::Y).y > 0.0);
        let align = Mat3::from_cols(Vec3::new(0.0, -1.0, 0.0), Vec3::Z, Vec3::NEG_X);
        for yaw in [-2.8_f32, 0.0, 0.6] {
            for pitch in [-0.2_f32, 0.0, 0.2] {
                let target = GimbalAimTarget {
                    yaw,
                    pitch: -pitch - std::f32::consts::FRAC_PI_2,
                };
                let actual = align * (target.rotation() * Vec3::Y);
                let expected = Vec3::new(
                    yaw.cos() * pitch.cos(),
                    yaw.sin() * pitch.cos(),
                    -pitch.sin(),
                );
                assert!((actual - expected).length() < 1e-6);
            }
        }
    }

    #[test]
    fn fire_gate_uses_physical_axis_and_rejects_actuator_lag() {
        let target = GimbalAimTarget { yaw: 0.3, pitch: -std::f32::consts::FRAC_PI_2 + 0.1 };
        assert!(sp_fire_aligned(target, target.rotation(), 0.01));
        let lagged = GimbalAimTarget { yaw: target.yaw - 0.03, pitch: target.pitch };
        assert!(!sp_fire_aligned(target, lagged.rotation(), 0.01));
        // Roll about the physical projectile axis does not change where it goes.
        assert!(sp_fire_aligned(target, target.rotation() * Quat::from_rotation_y(1.0), 0.01));
        assert!(!sp_fire_aligned(target, Quat::from_xyzw(f32::NAN, 0.0, 0.0, 1.0), 0.01));
    }

    #[test]
    fn sp_motion_uses_both_derivatives_and_preserves_rate_on_retarget() {
        let mut motion = SpMotionPlan::new(1.0, -2.0, 2.0, 4.0, 50.0, 100.0, 0.25);
        motion.elapsed = 0.1;
        let sample = motion.sample(GimbalAimTarget {
            yaw: 0.0,
            pitch: 0.0,
        });
        assert!((sample.yaw - 0.11).abs() < 1e-6);
        assert!((sample.pitch + 0.18).abs() < 1e-6);
        motion.previous_rate = Vec2::new(2.0, 3.0);
        let mut tracker = GimbalAimTracker::new(sample);
        tracker.retarget_sp(sample, motion);
        tracker.retarget_sp(
            sample,
            SpMotionPlan::new(0.0, 0.0, 0.0, 0.0, 50.0, 100.0, 0.25),
        );
        assert_eq!(tracker.motion.unwrap().previous_rate, Vec2::new(2.0, 3.0));
        tracker.retarget(sample);
        assert!(tracker.motion.is_none());
    }

    #[test]
    fn sp_actuator_converges_with_acceleration_limits_and_stops_on_timeout() {
        use bevy::ecs::system::RunSystemOnce;
        let mut world = World::new();
        world.insert_resource(Time::<()>::default());
        world.insert_resource(SimulationConfig::default());
        let goal = GimbalAimTarget {
            yaw: 0.3,
            pitch: -std::f32::consts::FRAC_PI_2 + 0.1,
        };
        let gimbal = world
            .spawn((
                Controlled,
                Transform::default(),
                GlobalTransform::default(),
                InfantryGimbal::default(),
                GimbalAimTracker::new(goal),
            ))
            .id();
        let muzzle = world
            .spawn((Controlled, InfantryLaunchOffset, GlobalTransform::default()))
            .id();
        let mount = Quat::from_rotation_x(-65_f32.to_radians());
        let dt = 0.01_f32;
        let mut previous_angle = Vec2::ZERO;
        let mut previous_rate = Vec2::ZERO;
        for step in 0..600 {
            if step % 10 == 0 {
                world
                    .get_mut::<GimbalAimTracker>(gimbal)
                    .unwrap()
                    .retarget_sp(
                        goal,
                        SpMotionPlan::new(0.0, 0.0, 0.0, 0.0, 50.0, 100.0, 0.25),
                    );
            }
            let rotation = world.get::<Transform>(gimbal).unwrap().rotation;
            world
                .entity_mut(gimbal)
                .insert(GlobalTransform::from(Transform::from_rotation(rotation)));
            world
                .entity_mut(muzzle)
                .insert(GlobalTransform::from(Transform::from_rotation(
                    rotation * mount,
                )));
            world
                .resource_mut::<Time>()
                .advance_by(std::time::Duration::from_secs_f32(dt));
            world.run_system_once(gimbal_pid_controls).unwrap();
            let (yaw, pitch, _) = world
                .get::<Transform>(gimbal)
                .unwrap()
                .rotation
                .to_euler(EulerRot::YXZ);
            let angle = Vec2::new(yaw, pitch);
            let rate = (angle - previous_angle) / dt;
            let acceleration = (rate - previous_rate) / dt;
            assert!(acceleration.x.abs() < 50.1 && acceleration.y.abs() < 100.1);
            previous_angle = angle;
            previous_rate = rate;
        }
        let actual = world.get::<Transform>(gimbal).unwrap().rotation * mount * Vec3::Y;
        assert!((actual - goal.rotation() * Vec3::Y).length() < 0.01);
        for _ in 0..40 {
            world
                .resource_mut::<Time>()
                .advance_by(std::time::Duration::from_secs_f32(dt));
            world.run_system_once(gimbal_pid_controls).unwrap();
        }
        assert!(world.get::<GimbalAimTracker>(gimbal).is_none());
        let stopped = *world.get::<Transform>(gimbal).unwrap();
        world.run_system_once(gimbal_pid_controls).unwrap();
        assert_eq!(
            world.get::<Transform>(gimbal).unwrap().rotation,
            stopped.rotation
        );
    }
}
