use avian3d::prelude::{CollisionEnd, CollisionEventsEnabled};
use bevy::prelude::{ChildOf, Commands, Entity, GlobalTransform, On, Plugin, Query, ResMut, With};

use super::construct::Armor;
use crate::metrics::{ShotOutcome, ShotTelemetry, SimulationMetrics};
use crate::robomaster::power_rune::prelude::Projectile;
use crate::statistic::ProjectileStatistics;

fn handle_armor_collision(
    event: On<CollisionEnd>,
    mut commands: Commands,
    mut stats: ResMut<ProjectileStatistics>,
    mut metrics: ResMut<SimulationMetrics>,
    mut projectiles: Query<(&mut ShotTelemetry, &GlobalTransform), With<Projectile>>,
    armors: Query<(&Armor, &GlobalTransform)>,
    child_of: Query<&ChildOf>,
) {
    let (projectile_entity, other_collider) = if let Some(body) = event.body1 {
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

    let armor_entity = if armors.contains(other_collider) {
        Some(other_collider)
    } else {
        child_of
            .iter_ancestors(other_collider)
            .find(|ancestor| armors.contains(*ancestor))
    };
    let Some(armor_entity) = armor_entity else {
        return;
    };

    let Ok((armor, armor_transform)) = armors.get(armor_entity) else {
        return;
    };
    let Ok((mut shot, projectile_transform)) = projectiles.get_mut(projectile_entity) else {
        return;
    };
    if shot.outcome == ShotOutcome::Pending {
        // Disable collision events for this projectile so it only counts once
        commands
            .entity(projectile_entity)
            .remove::<CollisionEventsEnabled>();
        shot.outcome = ShotOutcome::ArmorHit;
        stats.increase_accurate();
        metrics.record_armor_hit(
            &shot,
            match armor.team {
                crate::robomaster::prelude::Team::Red => "red",
                crate::robomaster::prelude::Team::Blue => "blue",
            },
            armor.label as u8,
            projectile_transform
                .translation()
                .lerp(armor_transform.translation(), 0.5),
        );
    }
}

#[derive(Default)]
pub(super) struct ArmorCollisionPlugin;

impl Plugin for ArmorCollisionPlugin {
    fn build(&self, app: &mut bevy::app::App) {
        app.add_observer(handle_armor_collision);
    }
}
