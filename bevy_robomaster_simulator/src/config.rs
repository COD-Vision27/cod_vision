use avian3d::prelude::SubstepCount;
use bevy::prelude::*;
use crossbeam_channel::{Receiver, Sender, unbounded};
use notify::{Event, RecommendedWatcher, RecursiveMode, Watcher};
use serde::Deserialize;
use std::path::Path;

#[derive(Resource, Deserialize, Reflect, Clone)]
#[reflect(Resource)]
pub struct SimulationConfig {
    #[serde(default)]
    pub window: WindowConfig,
    #[serde(default)]
    pub debug: DebugConfig,
    #[serde(default)]
    pub preview: PreviewConfig,
    #[serde(default)]
    pub render: RenderConfig,
    #[serde(default)]
    pub capture: CapturePipelineConfig,
    #[serde(default)]
    pub livox_ros: LivoxRosConfig,
    #[serde(default)]
    pub telemetry: TelemetryConfig,
    pub physics: PhysicsConfig,
    pub vehicle: VehicleConfig,
    #[serde(default)]
    pub scenario: ScenarioConfig,
    #[serde(default)]
    pub mecanum: MecanumConfig,
    pub projectile: ProjectileConfig,
    pub camera: CameraConfig,
}

/// Repeatable auto-aim target motion; omitted in the original interactive config.
#[derive(Deserialize, Reflect, Clone, Copy, Debug, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ScenarioMode {
    Static,
    Spin,
    Move,
}

#[derive(Deserialize, Reflect, Clone, Debug)]
#[serde(default)]
pub struct ScenarioConfig {
    pub mode: Option<ScenarioMode>,
    /// Signed angular speed; omitted inherits vehicle.rotation_speed.
    pub spin_speed_rad_s: Option<f32>,
    pub target_distance_m: f32,
    pub target_height_m: f32,
    pub target_lateral_m: f32,
    pub initial_pitch_deg: f32,
    pub move_amplitude_m: f32,
    pub move_peak_speed_mps: f32,
    pub move_start_delay_s: f32,
    pub move_position_gain: f32,
    pub move_velocity_gain: f32,
    pub move_deadband: f32,
}

impl Default for ScenarioConfig {
    fn default() -> Self {
        Self {
            mode: None,
            spin_speed_rad_s: None,
            target_distance_m: 4.0,
            target_height_m: 1.0,
            target_lateral_m: 0.0,
            initial_pitch_deg: -9.0,
            move_amplitude_m: 0.8,
            move_peak_speed_mps: 0.64,
            move_start_delay_s: 4.0,
            move_position_gain: 3.0,
            move_velocity_gain: 2.0,
            move_deadband: 0.2,
        }
    }
}

impl ScenarioConfig {
    pub fn validate(&self) -> Result<(), &'static str> {
        if [
            self.target_distance_m,
            self.target_height_m,
            self.target_lateral_m,
            self.initial_pitch_deg,
            self.move_amplitude_m,
            self.move_peak_speed_mps,
            self.move_start_delay_s,
            self.move_position_gain,
            self.move_velocity_gain,
            self.move_deadband,
        ]
        .iter()
        .any(|v| !v.is_finite())
            || self.spin_speed_rad_s.is_some_and(|v| !v.is_finite())
        {
            return Err("scenario parameters must be finite");
        }
        if self.target_distance_m <= 0.0
            || self.target_height_m < 0.0
            || self.initial_pitch_deg.abs() > 89.0
            || self.move_amplitude_m < 0.0
            || self.move_peak_speed_mps < 0.0
            || self.move_start_delay_s < 0.0
            || self.move_position_gain < 0.0
            || self.move_velocity_gain < 0.0
            || self.move_deadband < 0.0
        {
            return Err("invalid scenario distance, pitch, motion or controller bounds");
        }
        Ok(())
    }

    pub fn move_reference(&self, elapsed: f32) -> (f32, f32) {
        if elapsed <= self.move_start_delay_s || self.move_amplitude_m == 0.0 {
            return (self.target_lateral_m, 0.0);
        }
        let omega = self.move_peak_speed_mps / self.move_amplitude_m;
        let phase = (elapsed - self.move_start_delay_s) * omega;
        (
            self.target_lateral_m + self.move_amplitude_m * phase.sin(),
            self.move_peak_speed_mps * phase.cos(),
        )
    }
}

impl SimulationConfig {
    pub fn scenario_mode(&self) -> Option<ScenarioMode> {
        match std::env::var("DAEDALUS_SP_SCENARIO").as_deref() {
            Ok("static") => Some(ScenarioMode::Static),
            Ok("spin") => Some(ScenarioMode::Spin),
            Ok("move") => Some(ScenarioMode::Move),
            _ => self.scenario.mode,
        }
    }
}

/// Optional JSONL output used to measure simulator shots without changing the
/// Talos shared-memory ABI.  Keep this disabled for normal interactive runs.
#[derive(Deserialize, Reflect, Clone)]
#[serde(default)]
pub struct TelemetryConfig {
    /// Write versioned per-shot events and a session summary.
    pub enabled: bool,
    /// Empty means `daedalus_metrics.jsonl` when `enabled` is true.
    pub output_path: String,
    /// Flush every event for crash-safe short experiments; disable for long runs.
    pub flush_each_event: bool,
}

impl Default for TelemetryConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            output_path: String::new(),
            flush_each_event: true,
        }
    }
}

#[derive(Deserialize, Reflect, Clone)]
pub struct WindowConfig {
    pub present_mode: String,
}

impl Default for WindowConfig {
    fn default() -> Self {
        Self {
            // Uncap rendering by default so off-screen capture (Talos/ROS2) can exceed 60Hz.
            present_mode: "auto_no_vsync".to_string(),
        }
    }
}

#[derive(Deserialize, Reflect, Clone)]
#[serde(default)]
pub struct DebugConfig {
    pub egui: bool,
    pub inspector: bool,
    pub diagnostics: bool,
}

impl Default for DebugConfig {
    fn default() -> Self {
        Self {
            egui: false,
            inspector: false,
            diagnostics: false,
        }
    }
}

#[derive(Deserialize, Reflect, Clone)]
pub struct PreviewConfig {
    pub enabled: bool,
}

impl Default for PreviewConfig {
    fn default() -> Self {
        Self { enabled: true }
    }
}

#[derive(Deserialize, Reflect, Clone)]
#[serde(default)]
pub struct RenderConfig {
    pub illuminance: f32,
    pub shadows: bool,
    pub main_camera_fxaa: bool,
    #[serde(alias = "main_camera_metalfx_temporal")]
    pub metalfx_temporal: bool,
    #[serde(alias = "main_camera_metalfx_frame_generation")]
    pub metalfx_frame_generation: bool,
    #[serde(alias = "main_camera_metalfx_scale")]
    pub metalfx_scale: f32,
}

impl Default for RenderConfig {
    fn default() -> Self {
        Self {
            illuminance: 50.0,
            shadows: false,
            main_camera_fxaa: false,
            metalfx_temporal: cfg!(target_os = "macos"),
            metalfx_frame_generation: false,
            metalfx_scale: 2.0,
        }
    }
}

#[derive(Deserialize, Reflect, Clone)]
#[serde(default)]
pub struct PhysicsConfig {
    pub substep_count: u32,
    pub fixed_hz: f64,
}

impl Default for PhysicsConfig {
    fn default() -> Self {
        Self {
            substep_count: 8,
            fixed_hz: 120.0,
        }
    }
}

#[derive(Deserialize, Reflect, Clone)]
#[serde(default)]
pub struct VehicleConfig {
    pub rotation_speed: f32,
    pub yaw_acceleration: f32,
    pub tilt_rotation_speed: f32,
    pub gimbal_rotation_speed: f32,
    pub gimbal_pitch_limit: f32,
    pub max_speed: f32,
    pub linear_acceleration: f32,
    pub acceleration_exponent: f32,
    #[serde(default)]
    pub gimbal_pid: GimbalPidConfig,
}

impl Default for VehicleConfig {
    fn default() -> Self {
        Self {
            rotation_speed: 3.0,
            yaw_acceleration: 24.0,
            tilt_rotation_speed: 3.0,
            gimbal_rotation_speed: 3.0,
            gimbal_pitch_limit: 0.785,
            max_speed: 4.0,
            linear_acceleration: 8.0,
            acceleration_exponent: 10.0,
            gimbal_pid: GimbalPidConfig::default(),
        }
    }
}

/// Gains for the closed-loop gimbal tracking used by every remote/auto-aim command.
/// The two axes carry different inertia and travel limits, so they are tuned apart.
#[derive(Deserialize, Reflect, Clone)]
#[serde(default)]
pub struct GimbalPidConfig {
    pub yaw: GimbalAxisPidConfig,
    pub pitch: GimbalAxisPidConfig,
}

impl Default for GimbalPidConfig {
    fn default() -> Self {
        Self {
            yaw: GimbalAxisPidConfig {
                kp: 12.0,
                ki: 0.5,
                kd: 0.35,
                integral_limit: 0.5,
                max_rate: 20.0,
            },
            pitch: GimbalAxisPidConfig {
                kp: 10.0,
                ki: 0.5,
                kd: 0.3,
                integral_limit: 0.5,
                max_rate: 12.0,
            },
        }
    }
}

#[derive(Deserialize, Reflect, Clone)]
#[serde(default)]
pub struct GimbalAxisPidConfig {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    /// Bound on the accumulated error term (rad·s), guards against windup.
    pub integral_limit: f32,
    /// Saturation of the commanded angular rate for this axis (rad/s).
    pub max_rate: f32,
}

impl Default for GimbalAxisPidConfig {
    fn default() -> Self {
        Self {
            kp: 12.0,
            ki: 0.5,
            kd: 0.35,
            integral_limit: 0.5,
            max_rate: 20.0,
        }
    }
}

#[derive(Deserialize, Reflect, Clone)]
#[serde(default)]
pub struct MecanumConfig {
    pub wheel_radius_m: f32,
    pub half_wheelbase_m: f32,
    pub half_trackwidth_m: f32,
}

impl Default for MecanumConfig {
    fn default() -> Self {
        Self {
            wheel_radius_m: 0.076,
            half_wheelbase_m: 0.18,
            half_trackwidth_m: 0.15,
        }
    }
}

#[derive(Deserialize, Reflect, Clone)]
pub struct ProjectileConfig {
    pub lifetime: f32,
    pub speed: f32,
    pub cooldown: f32,
    pub diameter: f32,
    pub uav_size: f32,
    pub uav_vel: f32,
    pub mass: f32,
    pub friction: f32,
    pub linear_damping: f32,
    #[serde(default)]
    pub aerodynamics: ProjectileAerodynamicsConfig,
}

#[derive(Deserialize, Reflect, Clone)]
pub struct ProjectileAerodynamicsConfig {
    pub enabled: bool,
    pub air_density: f32,
    pub drag_coefficient: f32,
    pub wind: [f32; 3],
}

impl Default for ProjectileAerodynamicsConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            // kg/m^3 - air density at sea level (15°C)
            air_density: 1.225,
            // Drag coefficient for a smooth sphere, typical Re for 17mm @ ~25m/s.
            drag_coefficient: 0.47,
            // m/s - wind velocity in world coordinates.
            wind: [0.0, 0.0, 0.0],
        }
    }
}

#[derive(Deserialize, Reflect, Clone)]
pub struct CameraConfig {
    pub fov: f32,
    pub free_move_speed: f32,
    pub follow_offset: [f32; 3],
    pub mouse_sensitivity: f32,
}

#[derive(Deserialize, Reflect, Clone)]
#[serde(default)]
pub struct CapturePipelineConfig {
    pub color: CaptureStreamConfig,
    pub depth: DepthCaptureConfig,
}

impl Default for CapturePipelineConfig {
    fn default() -> Self {
        Self {
            color: CaptureStreamConfig::default(),
            depth: DepthCaptureConfig::default(),
        }
    }
}

#[derive(Deserialize, Reflect, Clone)]
#[serde(default)]
pub struct CaptureStreamConfig {
    pub width: u32,
    pub height: u32,
}

impl Default for CaptureStreamConfig {
    fn default() -> Self {
        Self {
            width: 1440,
            height: 1080,
        }
    }
}

#[derive(Deserialize, Reflect, Clone)]
#[serde(default)]
pub struct DepthCaptureConfig {
    pub width: u32,
    pub height: u32,
    pub near: f32,
    pub far: f32,
}

impl Default for DepthCaptureConfig {
    fn default() -> Self {
        Self {
            width: 640,
            height: 480,
            near: 0.1,
            far: 80.0,
        }
    }
}

#[derive(Deserialize, Reflect, Clone)]
#[serde(default)]
pub struct LivoxRosConfig {
    pub enabled: bool,
    pub frame_id: String,
    pub publish_freq: f32,
    pub points_per_second: u32,
    pub line_num: u8,
    pub tag_default: u8,
    pub intensity_default: f32,
}

impl Default for LivoxRosConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            frame_id: "livox_frame".to_string(),
            publish_freq: 10.0,
            points_per_second: 100_000,
            line_num: 6,
            tag_default: 0,
            intensity_default: 100.0,
        }
    }
}

impl SimulationConfig {
    pub fn load() -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
        let content = std::fs::read_to_string("config.toml")?;
        let config: Self = toml::from_str(&content)?;
        config.scenario.validate()?;
        if !config.vehicle.rotation_speed.is_finite()
            || !config.vehicle.yaw_acceleration.is_finite()
            || config.vehicle.yaw_acceleration <= 0.0
        {
            return Err(
                "vehicle rotation_speed must be finite and yaw_acceleration must be positive"
                    .into(),
            );
        }
        Ok(config)
    }
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self::load().unwrap_or_else(|e| {
            if Path::new("config.toml").exists() {
                panic!("Invalid config.toml: {e}");
            }
            warn!("Failed to load config.toml: {}, using defaults", e);
            Self {
                window: WindowConfig::default(),
                debug: DebugConfig::default(),
                preview: PreviewConfig::default(),
                render: RenderConfig::default(),
                capture: CapturePipelineConfig::default(),
                livox_ros: LivoxRosConfig::default(),
                telemetry: TelemetryConfig::default(),
                physics: PhysicsConfig::default(),
                vehicle: VehicleConfig::default(),
                scenario: ScenarioConfig::default(),
                mecanum: MecanumConfig::default(),
                projectile: ProjectileConfig {
                    lifetime: 5.0,
                    speed: 25.0,
                    cooldown: 0.1,
                    diameter: 0.017,
                    mass: 0.017,
                    friction: 1.1,
                    linear_damping: 0.0,
                    aerodynamics: ProjectileAerodynamicsConfig::default(),
                    uav_size: 1.0,
                    uav_vel: 2.0,
                },
                camera: CameraConfig {
                    fov: 45.0,
                    free_move_speed: 8.0,
                    follow_offset: [0.0, 3.0, 2.0],
                    mouse_sensitivity: 0.003,
                },
            }
        })
    }
}

#[derive(Resource)]
pub struct ConfigWatcher {
    _watcher: RecommendedWatcher,
    receiver: Receiver<Result<Event, notify::Error>>,
}

pub struct ConfigPlugin;

impl Plugin for ConfigPlugin {
    fn build(&self, app: &mut App) {
        let config = SimulationConfig::default();

        // Set up file watcher using crossbeam-channel for thread safety
        let (tx, rx): (
            Sender<Result<Event, notify::Error>>,
            Receiver<Result<Event, notify::Error>>,
        ) = unbounded();
        let watcher_result = RecommendedWatcher::new(
            move |res| {
                let _ = tx.send(res);
            },
            notify::Config::default(),
        );

        match watcher_result {
            Ok(mut watcher) => {
                if let Err(e) = watcher.watch(Path::new("config.toml"), RecursiveMode::NonRecursive)
                {
                    warn!("Failed to watch config.toml: {}", e);
                } else {
                    info!("Config hot-reload enabled for config.toml");
                    app.insert_resource(ConfigWatcher {
                        _watcher: watcher,
                        receiver: rx,
                    });
                    app.add_systems(Update, config_hot_reload);
                }
            }
            Err(e) => {
                warn!("Failed to create config watcher: {}", e);
            }
        }

        app.insert_resource(config)
            .register_type::<SimulationConfig>();
    }
}

fn config_hot_reload(
    mut config: ResMut<SimulationConfig>,
    watcher: Option<Res<ConfigWatcher>>,
    mut substeps: Option<ResMut<SubstepCount>>,
    mut fixed_time: Option<ResMut<Time<Fixed>>>,
) {
    let Some(watcher) = watcher else {
        return;
    };

    // Non-blocking check for file changes
    while let Ok(Ok(event)) = watcher.receiver.try_recv() {
        if event.kind.is_modify() {
            match SimulationConfig::load() {
                Ok(new_config) => {
                    info!("Config reloaded successfully");
                    if let Some(substeps) = substeps.as_deref_mut() {
                        substeps.0 = new_config.physics.substep_count;
                    }
                    if let Some(fixed_time) = fixed_time.as_deref_mut() {
                        *fixed_time = Time::<Fixed>::from_hz(new_config.physics.fixed_hz.max(1.0));
                    }
                    *config = new_config;
                }
                Err(e) => {
                    warn!("Failed to reload config: {}", e);
                }
            }
        }
    }
}

#[cfg(test)]
mod scenario_tests {
    use super::*;

    #[test]
    fn omitted_scenario_keeps_original_interactive_scene() {
        let config: SimulationConfig = toml::from_str(include_str!("../config.toml")).unwrap();
        assert_eq!(config.scenario.mode, None);
        assert_eq!(config.scenario.spin_speed_rad_s, None);
    }

    #[test]
    fn custom_speeds_and_mode_deserialize() {
        let c: ScenarioConfig = toml::from_str("mode = 'spin'\nspin_speed_rad_s = -3.0").unwrap();
        assert_eq!(c.mode, Some(ScenarioMode::Spin));
        assert_eq!(c.spin_speed_rad_s, Some(-3.0));
        c.validate().unwrap();
        assert!(toml::from_str::<ScenarioConfig>("mode = 'typo'").is_err());
    }

    #[test]
    fn motion_reference_respects_delay_amplitude_and_peak_speed() {
        let c = ScenarioConfig {
            move_amplitude_m: 2.0,
            move_peak_speed_mps: 1.0,
            target_lateral_m: 0.3,
            ..Default::default()
        };
        assert_eq!(c.move_reference(3.0), (0.3, 0.0));
        let (x, v) = c.move_reference(4.0 + std::f32::consts::PI);
        assert!((x - 2.3).abs() < 1e-5);
        assert!(v.abs() < 1e-5);
        let (_, v) = c.move_reference(4.0 + 4.0 * std::f32::consts::PI);
        assert!((v - 1.0).abs() < 1e-5);
    }

    #[test]
    fn zero_amplitude_and_zero_speed_are_stationary() {
        for c in [
            ScenarioConfig {
                move_amplitude_m: 0.0,
                ..Default::default()
            },
            ScenarioConfig {
                move_peak_speed_mps: 0.0,
                ..Default::default()
            },
        ] {
            assert_eq!(c.move_reference(10.0), (c.target_lateral_m, 0.0));
        }
    }

    #[test]
    fn invalid_motion_parameters_are_rejected() {
        for c in [
            ScenarioConfig {
                move_amplitude_m: -1.0,
                ..Default::default()
            },
            ScenarioConfig {
                target_distance_m: 0.0,
                ..Default::default()
            },
            ScenarioConfig {
                spin_speed_rad_s: Some(f32::NAN),
                ..Default::default()
            },
        ] {
            assert!(c.validate().is_err());
        }
    }
}
