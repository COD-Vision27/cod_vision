use crate::metrics::{ShotOutcome, ShotTelemetry, SimulationMetrics};
use crate::robomaster::power_rune::common::RuneHitOutcome;
use crate::robomaster::power_rune::rotation::PowerRuneRotation;
use crate::robomaster::power_rune::rune::PowerRuneMechanism;
use avian3d::prelude::{CollisionEnd, CollisionEventsEnabled};
use bevy::prelude::{
    ChildOf, Commands, Component, Entity, EntityEvent, GlobalTransform, On, Query, ResMut,
    Resource, Update, With,
};
use std::collections::HashSet;

#[derive(Component)]
#[require(CollisionEventsEnabled)]
pub struct Projectile;

#[derive(Resource, Default)]
struct ConsumedRuneProjectiles(HashSet<Entity>);

#[derive(Component, Debug, Copy, Clone)]
pub struct RuneIndex {
    pub target: usize,
    pub rune: Entity,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct HitResult {
    pub outcome: RuneHitOutcome,
}

impl HitResult {
    pub const fn accurate(self) -> bool {
        self.outcome.is_accurate()
    }
}

#[derive(EntityEvent)]
pub struct RuneActivated {
    #[event_target]
    pub rune: Entity,
}

#[derive(EntityEvent)]
pub struct RuneHit {
    #[event_target]
    pub rune: Entity,
    pub result: HitResult,
}

fn handle_rune_collision(
    event: On<CollisionEnd>,
    mut commands: Commands,
    mut consumed_projectiles: ResMut<ConsumedRuneProjectiles>,
    mut metrics: ResMut<SimulationMetrics>,
    mut runes: Query<(&mut PowerRuneMechanism, &mut PowerRuneRotation)>,
    targets: Query<&RuneIndex>,
    mut projectiles: Query<(&mut ShotTelemetry, &GlobalTransform), With<Projectile>>,
    child_of: Query<&ChildOf>,
) {
    let (projectile_entity, target_collider) = if let Some(body) = event.body1 {
        if projectiles.contains(body) {
            (body, event.collider2)
        } else if let Some(body) = event.body2 {
            if projectiles.contains(body) {
                (body, event.collider1)
            } else {
                return;
            }
        } else {
            return;
        }
    } else if let Some(body) = event.body2 {
        if projectiles.contains(body) {
            (body, event.collider1)
        } else {
            return;
        }
    } else {
        return;
    };

    let Some(target) = find_rune_target(target_collider, &targets, &child_of) else {
        return;
    };

    let Ok((mut mechanism, mut rotation)) = runes.get_mut(target.rune) else {
        return;
    };

    let Ok((mut shot, projectile_transform)) = projectiles.get_mut(projectile_entity) else {
        return;
    };
    if shot.outcome != ShotOutcome::Pending {
        return;
    }
    if !consumed_projectiles.0.insert(projectile_entity) {
        return;
    }

    commands
        .entity(projectile_entity)
        .remove::<CollisionEventsEnabled>();

    let mut rng = rand::rng();
    let outcome = mechanism.state_mut().hit(target.target, &mut rng);
    shot.outcome = if outcome.is_accurate() {
        ShotOutcome::RuneHit
    } else {
        ShotOutcome::RuneMiss
    };
    metrics.record_rune_hit(
        &shot,
        outcome.is_accurate(),
        projectile_transform.translation(),
    );
    rotation.sync_activation(
        mechanism.state().mode(),
        mechanism.state().is_activating(),
        &mut rng,
    );

    commands.trigger(RuneHit {
        rune: target.rune,
        result: HitResult { outcome },
    });

    if outcome.activates_rune() {
        commands.trigger(RuneActivated { rune: target.rune });
    }
}

fn cleanup_consumed_rune_projectiles(
    mut consumed_projectiles: ResMut<ConsumedRuneProjectiles>,
    projectiles: Query<(), With<Projectile>>,
) {
    consumed_projectiles
        .0
        .retain(|entity| projectiles.contains(*entity));
}

fn find_rune_target(
    entity: Entity,
    targets: &Query<&RuneIndex>,
    child_of: &Query<&ChildOf>,
) -> Option<RuneIndex> {
    if let Ok(target) = targets.get(entity) {
        return Some(*target);
    }

    child_of
        .iter_ancestors(entity)
        .find_map(|ancestor| targets.get(ancestor).ok().copied())
}

#[derive(Default)]
pub(super) struct PowerRuneCollisionPlugin;

impl bevy::app::Plugin for PowerRuneCollisionPlugin {
    fn build(&self, app: &mut bevy::app::App) {
        app.init_resource::<ConsumedRuneProjectiles>()
            .add_systems(Update, cleanup_consumed_rune_projectiles)
            .add_observer(handle_rune_collision);
    }
}
