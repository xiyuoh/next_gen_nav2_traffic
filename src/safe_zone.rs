use crate::{AgentName, NavigationTarget, RclrsNode, RosSubscription};
use bevy::prelude::*;
use rmf_prototype_msgs::msg::{Region, SafeZone};
use std::sync::Arc;

#[derive(Component)]
pub struct SafeZoneSubscription {
    pub subscriber: Arc<RosSubscription<SafeZone>>,
}

#[derive(Component, Debug, Clone, Default, Deref)]
pub struct CurrentSafeZone(pub Option<SafeZone>);

impl CurrentSafeZone {
    pub fn update(&mut self, value: SafeZone) {
        self.0 = Some(value);
    }

    pub fn clear(&mut self) {
        self.0 = None;
    }

    pub fn matches(&self, other: &SafeZone) -> bool {
        self.as_ref().is_some_and(|sz| sz.id == other.id)
    }
}

#[derive(Default)]
pub struct SafeZoneSubscriptionPlugin {}

impl Plugin for SafeZoneSubscriptionPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(PreUpdate, update_incremental_target)
            .add_observer(subscribe_on_new_agent);
    }
}

fn subscribe_on_new_agent(
    trigger: Trigger<OnAdd, AgentName>,
    mut commands: Commands,
    agent_names: Query<&AgentName>,
    node: Res<RclrsNode>,
) {
    let e = trigger.target();
    let Ok(agent_name) = agent_names.get(e).map(|agent| agent.0.clone()) else {
        return;
    };
    let topic = agent_name + "/plan/safe_zone";
    let subscription = Arc::new(RosSubscription::<SafeZone>::new(&node, topic.clone()));
    commands.entity(e).insert((
        SafeZoneSubscription {
            subscriber: Arc::clone(&subscription),
        },
        CurrentSafeZone::default(),
    ));
}

fn update_incremental_target(
    mut nav_target: EventWriter<NavigationTarget>,
    mut subscriptions: Query<(
        Entity,
        &SafeZoneSubscription,
        &mut CurrentSafeZone,
        &AgentName,
    )>,
) {
    for (e, safe_zone_sub, mut current_safe_zone, agent) in subscriptions.iter_mut() {
        let Some(safe_zone) = safe_zone_sub.subscriber.data_callback() else {
            continue;
        };
        if current_safe_zone.matches(&safe_zone) {
            continue;
        }
        let Some((target_x, target_y, target_yaw)) = next_target(&safe_zone) else {
            continue;
        };

        *current_safe_zone = CurrentSafeZone(Some(safe_zone.clone()));
        nav_target.write(NavigationTarget::new(
            e,
            safe_zone.id.plan_id,
            target_x as f64,
            target_y as f64,
            target_yaw as f64,
        ));
    }
}

fn next_target(safe_zone: &SafeZone) -> Option<(f32, f32, f32)> {
    // TODO(@xiyuoh) more sophisticated point selection taking into account
    // all factors (region hints, orientations, etc.)

    let constraints = &safe_zone.incremental_target;
    let mut xy: Option<(f32, f32)> = None;
    let mut yaw: Option<f32> = None;

    // Assume either regions or nodes will be populated, not both.
    for target_region in constraints.regions.iter() {
        let tolerance = target_region.tolerance;
        let region = &target_region.region;
        let points = &region.points;

        match region.hint {
            Region::HINT_POINT => {
                // There should only be exactly 2 elements forming (x, y)
                if points.len() != 2 {
                    continue;
                }
                // TODO(@xiyuoh)
                xy = Some((points[0], points[1]));
            }
            Region::HINT_AXIS_ALIGNED_RECTANGLE => {
                //
            }
            Region::HINT_RECTANGLE => {
                //
            }
            Region::HINT_CONVEX_POLYGON => {
                //
            }
            Region::HINT_POLYGON | Region::HINT_UNSPECIFIED => {
                //
            }
            _ => {
                //
            }
        }

        for target_ori in target_region.orientations.iter() {
            yaw = Some(target_ori.orientation_radians);
            // TODO(@xiyuoh) some processing using spread and tolerance
        }
    }
    for target_node in constraints.nodes.iter() {
        //
    }

    xy.zip(yaw).map(|((x, y), yaw)| (x, y, yaw))
}
