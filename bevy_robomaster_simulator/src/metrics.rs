//! Optional, versioned simulator metrics for closed-loop auto-aim evaluation.
//!
//! Metrics are deliberately kept outside the Talos shared-memory ABI.  This lets a
//! simulator run with an older C++ consumer while still producing a deterministic,
//! machine-readable shot log.  Set `DAEDALUS_METRICS_PATH` or enable `[telemetry]`
//! in `config.toml` to write JSONL; with no path the hot path only updates counters.

use bevy::prelude::*;
use serde_json::{Value, json};
use std::fs::{File, OpenOptions, create_dir_all};
use std::io::{BufWriter, Write};
use std::path::Path;

use crate::config::TelemetryConfig;

pub const METRICS_SCHEMA: &str = "daedalus.simulation_metrics";
pub const METRICS_VERSION: u32 = 1;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShotKind {
    Projectile,
    Dart,
}

impl ShotKind {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Projectile => "projectile",
            Self::Dart => "dart",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShotOutcome {
    Pending,
    ArmorHit,
    RuneHit,
    RuneMiss,
}

impl ShotOutcome {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Pending => "pending",
            Self::ArmorHit => "armor_hit",
            Self::RuneHit => "rune_hit",
            Self::RuneMiss => "rune_miss",
        }
    }
}

/// Metadata carried by each simulator projectile.  It is present even when file
/// metrics are disabled so collision accounting remains one-shot/one-outcome.
#[derive(Component, Clone, Copy, Debug)]
pub struct ShotTelemetry {
    pub id: u64,
    pub kind: ShotKind,
    pub outcome: ShotOutcome,
    pub launch_frame_seq: u64,
    pub launch_timestamp_ns: u64,
    pub launch_talos_frame_seq: Option<u64>,
    pub launch_talos_timestamp_ns: Option<u64>,
    pub origin: Vec3,
    pub direction: Vec3,
}

/// Camera-frame projection for one ground-truth target.  This is telemetry-only
/// and deliberately does not change the Talos shared-memory layout.
#[derive(Clone, Copy, Debug)]
pub struct GroundTruthProjection {
    pub camera_position_bevy: Vec3,
    pub camera_position_ros: Vec3,
    pub yaw_deg: f32,
    pub pitch_deg: f32,
    pub pixel_u: f32,
    pub pixel_v: f32,
    pub visible: bool,
}

#[derive(Clone, Debug)]
struct LastGimbalCommand {
    source: &'static str,
    received_frame_seq: u64,
    timestamp_ns: u64,
    talos_frame_seq: Option<u64>,
    talos_timestamp_ns: Option<u64>,
    yaw_deg: f32,
    pitch_deg: f32,
    distance_m: f32,
    fire_advice: bool,
}

#[derive(Default, Clone, Copy, Debug)]
struct MetricsSummary {
    shots: u64,
    projectile_shots: u64,
    dart_shots: u64,
    armor_hits: u64,
    projectile_armor_hits: u64,
    dart_armor_hits: u64,
    rune_hits: u64,
    rune_misses: u64,
    misses: u64,
}

/// Runtime metrics sink.  It is intentionally cheap when no output path is set.
#[derive(Resource)]
pub struct SimulationMetrics {
    writer: Option<BufWriter<File>>,
    flush_each_event: bool,
    next_shot_id: u64,
    frame_seq: u64,
    timestamp_ns: u64,
    talos_frame_seq: Option<u64>,
    talos_timestamp_ns: Option<u64>,
    summary: MetricsSummary,
    last_command: Option<LastGimbalCommand>,
}

impl SimulationMetrics {
    pub fn from_config(config: &TelemetryConfig) -> Self {
        let env_path = std::env::var_os("DAEDALUS_METRICS_PATH")
            .filter(|path| !path.is_empty())
            .map(std::path::PathBuf::from);
        let configured_path = if config.enabled {
            let path = config.output_path.trim();
            if path.is_empty() {
                Some(std::path::PathBuf::from("daedalus_metrics.jsonl"))
            } else {
                Some(std::path::PathBuf::from(path))
            }
        } else {
            None
        };
        let path = env_path.or(configured_path);
        let writer = path.and_then(|path| match open_metrics_file(&path) {
            Ok(file) => Some(BufWriter::new(file)),
            Err(error) => {
                eprintln!(
                    "daedalus metrics disabled: cannot open {}: {error}",
                    path.display()
                );
                None
            }
        });

        let mut metrics = Self {
            writer,
            flush_each_event: config.flush_each_event,
            next_shot_id: 1,
            frame_seq: 0,
            timestamp_ns: 0,
            talos_frame_seq: None,
            talos_timestamp_ns: None,
            summary: MetricsSummary::default(),
            last_command: None,
        };
        metrics.emit(json!({
            "schema": METRICS_SCHEMA,
            "version": METRICS_VERSION,
            "event": "session_start",
            "sim_timestamp_ns": metrics.timestamp_ns,
        }));
        metrics
    }

    pub fn advance_frame(&mut self, elapsed_secs: f64) {
        self.frame_seq = self.frame_seq.wrapping_add(1);
        self.timestamp_ns = if elapsed_secs.is_finite() && elapsed_secs >= 0.0 {
            (elapsed_secs * 1_000_000_000.0).min(u64::MAX as f64) as u64
        } else {
            0
        };
    }

    pub fn set_talos_frame_stamp(&mut self, frame_seq: u64, timestamp_ns: u64) {
        self.talos_frame_seq = Some(frame_seq);
        self.talos_timestamp_ns = Some(timestamp_ns);
    }

    pub fn record_gimbal_command(
        &mut self,
        source: &'static str,
        yaw_deg: f32,
        pitch_deg: f32,
        distance_m: f32,
        fire_advice: bool,
    ) {
        let command = LastGimbalCommand {
            source,
            received_frame_seq: self.frame_seq,
            timestamp_ns: self.timestamp_ns,
            talos_frame_seq: self.talos_frame_seq,
            talos_timestamp_ns: self.talos_timestamp_ns,
            yaw_deg,
            pitch_deg,
            distance_m,
            fire_advice,
        };
        if self.writer.is_some() {
            self.emit(json!({
                "schema": METRICS_SCHEMA,
                "version": METRICS_VERSION,
                "event": "gimbal_command",
                "source": source,
                "frame_seq": self.frame_seq,
                "sim_timestamp_ns": self.timestamp_ns,
                "talos_frame_seq": self.talos_frame_seq,
                "talos_timestamp_ns": self.talos_timestamp_ns,
                "yaw_deg": yaw_deg,
                "pitch_deg": pitch_deg,
                "simulator_pitch_deg": if source == "talos" { -pitch_deg } else { pitch_deg },
                "distance_m": distance_m,
                "fire_advice": fire_advice,
            }));
        }
        self.last_command = Some(command);
    }

    /// Mirror the Talos ground-truth target into JSONL for frame-aligned
    /// centerline error evaluation.  The call is a no-op unless output is on.
    pub fn record_ground_truth_target(
        &mut self,
        frame_seq: u64,
        timestamp_ns: u64,
        target_index: u32,
        target_id: u64,
        team: u8,
        armor_label: u8,
        armor_type: u8,
        armor_count: u32,
        is_controlled: bool,
        position_bevy: Vec3,
        position: Vec3,
        projection: Option<GroundTruthProjection>,
        vyaw: f32,
        yaw: f32,
    ) {
        if self.writer.is_none() {
            return;
        }
        let mut event = json!({
            "schema": METRICS_SCHEMA,
            "version": METRICS_VERSION,
            "event": "ground_truth_target",
            "frame_seq": frame_seq,
            "timestamp_ns": timestamp_ns,
            "target_index": target_index,
            "target_id": target_id,
            "team": team,
            "armor_label": armor_label,
            "armor_type": armor_type,
            "armor_count": armor_count,
            "is_controlled": is_controlled,
            "position_bevy": [position_bevy.x, position_bevy.y, position_bevy.z],
            "position_ros": [position.x, position.y, position.z],
            "position_frame": "ros_z_up",
            "position": [position.x, position.y, position.z],
            "vyaw": vyaw,
            "yaw": yaw,
        });
        if let Some(projection) = projection {
            event["camera_position_bevy"] = json!([
                projection.camera_position_bevy.x,
                projection.camera_position_bevy.y,
                projection.camera_position_bevy.z,
            ]);
            event["camera_position_ros"] = json!([
                projection.camera_position_ros.x,
                projection.camera_position_ros.y,
                projection.camera_position_ros.z,
            ]);
            event["camera_position_frame"] = json!("ros_z_up");
            event["yaw_deg"] = json!(projection.yaw_deg);
            event["pitch_deg"] = json!(projection.pitch_deg);
            event["pixel_u"] = json!(projection.pixel_u);
            event["pixel_v"] = json!(projection.pixel_v);
            event["visible"] = json!(projection.visible);
        }
        self.emit(event);
    }

    /// Record one deduplicated armor entity for image-level evaluation.  This
    /// event is JSONL-only; the Talos ground-truth ABI remains root-target based.
    pub fn record_ground_truth_armor(
        &mut self,
        frame_seq: u64,
        timestamp_ns: u64,
        entity_id: u64,
        armor_id: u32,
        team: u8,
        armor_label: u8,
        armor_type: u8,
        is_controlled: bool,
        position_bevy: Vec3,
        position_ros: Vec3,
        projection: Option<GroundTruthProjection>,
    ) {
        if self.writer.is_none() {
            return;
        }
        let mut event = json!({
            "schema": METRICS_SCHEMA,
            "version": METRICS_VERSION,
            "event": "ground_truth_armor",
            "frame_seq": frame_seq,
            "timestamp_ns": timestamp_ns,
            "target_id": entity_id,
            "armor_id": armor_id,
            "team": team,
            "armor_label": armor_label,
            "armor_type": armor_type,
            "is_controlled": is_controlled,
            "position_bevy": [position_bevy.x, position_bevy.y, position_bevy.z],
            "position_ros": [position_ros.x, position_ros.y, position_ros.z],
            "position_frame": "ros_z_up",
        });
        if let Some(projection) = projection {
            event["camera_position_bevy"] = json!([
                projection.camera_position_bevy.x,
                projection.camera_position_bevy.y,
                projection.camera_position_bevy.z,
            ]);
            event["camera_position_ros"] = json!([
                projection.camera_position_ros.x,
                projection.camera_position_ros.y,
                projection.camera_position_ros.z,
            ]);
            event["camera_position_frame"] = json!("ros_z_up");
            event["yaw_deg"] = json!(projection.yaw_deg);
            event["pitch_deg"] = json!(projection.pitch_deg);
            event["pixel_u"] = json!(projection.pixel_u);
            event["pixel_v"] = json!(projection.pixel_v);
            event["visible"] = json!(projection.visible);
        }
        self.emit(event);
    }

    pub fn begin_shot(&mut self, kind: ShotKind, origin: Vec3, direction: Vec3) -> ShotTelemetry {
        let shot = ShotTelemetry {
            id: self.next_shot_id,
            kind,
            outcome: ShotOutcome::Pending,
            launch_frame_seq: self.frame_seq,
            launch_timestamp_ns: self.timestamp_ns,
            launch_talos_frame_seq: self.talos_frame_seq,
            launch_talos_timestamp_ns: self.talos_timestamp_ns,
            origin,
            direction,
        };
        self.next_shot_id = self.next_shot_id.wrapping_add(1);
        self.summary.shots += 1;
        match kind {
            ShotKind::Projectile => self.summary.projectile_shots += 1,
            ShotKind::Dart => self.summary.dart_shots += 1,
        }

        if self.writer.is_some() {
            let mut event = json!({
                "schema": METRICS_SCHEMA,
                "version": METRICS_VERSION,
                "event": "shot_fired",
                "shot_id": shot.id,
                "kind": kind.as_str(),
                "launch_frame_seq": shot.launch_frame_seq,
                "launch_timestamp_ns": shot.launch_timestamp_ns,
                "talos_frame_seq": shot.launch_talos_frame_seq,
                "talos_timestamp_ns": shot.launch_talos_timestamp_ns,
                "origin": [origin.x, origin.y, origin.z],
                "direction": [direction.x, direction.y, direction.z],
            });
            if let Some(command) = &self.last_command {
                event["command"] = json!({
                    "source": command.source,
                    "received_frame_seq": command.received_frame_seq,
                    "timestamp_ns": command.timestamp_ns,
                    "talos_frame_seq": command.talos_frame_seq,
                    "talos_timestamp_ns": command.talos_timestamp_ns,
                    "yaw_deg": command.yaw_deg,
                    "pitch_deg": command.pitch_deg,
                    "simulator_pitch_deg": if command.source == "talos" {
                        -command.pitch_deg
                    } else {
                        command.pitch_deg
                    },
                    "distance_m": command.distance_m,
                    "fire_advice": command.fire_advice,
                });
            }
            self.emit(event);
        }
        shot
    }

    pub fn record_armor_hit(
        &mut self,
        shot: &ShotTelemetry,
        team: &'static str,
        armor_label: u8,
        hit_position: Vec3,
    ) {
        if self.writer.is_some() {
            self.emit(json!({
                "schema": METRICS_SCHEMA,
                "version": METRICS_VERSION,
                "event": "armor_hit",
                "shot_id": shot.id,
                "kind": shot.kind.as_str(),
                "hit_frame_seq": self.frame_seq,
                "hit_timestamp_ns": self.timestamp_ns,
                "team": team,
                "armor_label": armor_label,
                "hit_position": [hit_position.x, hit_position.y, hit_position.z],
            }));
        }
    }

    pub fn record_rune_hit(&mut self, shot: &ShotTelemetry, accurate: bool, hit_position: Vec3) {
        if self.writer.is_some() {
            self.emit(json!({
                "schema": METRICS_SCHEMA,
                "version": METRICS_VERSION,
                "event": "rune_hit",
                "shot_id": shot.id,
                "kind": shot.kind.as_str(),
                "hit_frame_seq": self.frame_seq,
                "hit_timestamp_ns": self.timestamp_ns,
                "accurate": accurate,
                "hit_position": [hit_position.x, hit_position.y, hit_position.z],
            }));
        }
    }

    pub fn finish_shot(&mut self, shot: &ShotTelemetry) {
        let outcome = shot.outcome;
        match outcome {
            ShotOutcome::ArmorHit => {
                self.summary.armor_hits += 1;
                match shot.kind {
                    ShotKind::Projectile => self.summary.projectile_armor_hits += 1,
                    ShotKind::Dart => self.summary.dart_armor_hits += 1,
                }
            }
            ShotOutcome::RuneHit => self.summary.rune_hits += 1,
            ShotOutcome::RuneMiss => self.summary.rune_misses += 1,
            ShotOutcome::Pending => self.summary.misses += 1,
        }
        if self.writer.is_some() {
            self.emit(json!({
                "schema": METRICS_SCHEMA,
                "version": METRICS_VERSION,
                "event": "shot_finished",
                "shot_id": shot.id,
                "kind": shot.kind.as_str(),
                "outcome": outcome.as_str(),
                "finish_frame_seq": self.frame_seq,
                "finish_timestamp_ns": self.timestamp_ns,
            }));
        }
    }

    fn emit(&mut self, value: Value) {
        let Some(writer) = self.writer.as_mut() else {
            return;
        };
        if serde_json::to_writer(&mut *writer, &value).is_err() || writer.write_all(b"\n").is_err()
        {
            self.writer = None;
            eprintln!("daedalus metrics disabled after write failure");
            return;
        }
        if self.flush_each_event && writer.flush().is_err() {
            self.writer = None;
            eprintln!("daedalus metrics disabled after flush failure");
        }
    }
}

impl Drop for SimulationMetrics {
    fn drop(&mut self) {
        let summary = self.summary;
        self.emit(json!({
            "schema": METRICS_SCHEMA,
            "version": METRICS_VERSION,
            "event": "session_summary",
            "shots": summary.shots,
            "projectile_shots": summary.projectile_shots,
            "dart_shots": summary.dart_shots,
            "armor_hits": summary.armor_hits,
            "projectile_armor_hits": summary.projectile_armor_hits,
            "dart_armor_hits": summary.dart_armor_hits,
            "rune_hits": summary.rune_hits,
            "rune_misses": summary.rune_misses,
            "misses": summary.misses,
            "projectile_armor_accuracy": if summary.projectile_shots == 0 {
                0.0
            } else {
                summary.projectile_armor_hits as f64 / summary.projectile_shots as f64
            },
        }));
        if let Some(writer) = self.writer.as_mut() {
            let _ = writer.flush();
        }
    }
}

fn open_metrics_file(path: &Path) -> std::io::Result<File> {
    if let Some(parent) = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        create_dir_all(parent)?;
    }
    OpenOptions::new()
        .create(true)
        .truncate(true)
        .write(true)
        .open(path)
}

pub fn advance_metrics_frame(mut metrics: ResMut<SimulationMetrics>, time: Res<Time>) {
    metrics.advance_frame(time.elapsed_secs_f64());
}
