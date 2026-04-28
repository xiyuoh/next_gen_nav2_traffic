use crate::{NavigationTarget, RclrsNode, RosNamespace, RosSubscription};
use bevy::prelude::*;
use rmf_prototype_msgs::msg::{Region, SafeZone};
use std::sync::Arc;

#[derive(Resource)]
pub struct SafeZoneSubscription {
    pub subscriber: Arc<RosSubscription<SafeZone>>,
}

#[derive(Resource, Default, Deref, DerefMut)]
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
        let namespace = app.world().resource::<RosNamespace>().0.clone();
        let node = app.world().resource::<RclrsNode>();
        let subscription = Arc::new(RosSubscription::<SafeZone>::new(
            &node,
            namespace + "/plan/safe_zone",
        ));
        app.insert_resource(SafeZoneSubscription {
            subscriber: Arc::clone(&subscription),
        })
        .insert_resource(CurrentSafeZone::default())
        .add_systems(PreUpdate, update_incremental_target);
    }
}

fn update_incremental_target(
    mut current_safe_zone: ResMut<CurrentSafeZone>,
    mut nav_target: EventWriter<NavigationTarget>,
    safe_zone_sub: Res<SafeZoneSubscription>,
) {
    let Some(safe_zone) = safe_zone_sub.subscriber.data_callback() else {
        return;
    };
    if current_safe_zone.matches(&safe_zone) {
        return;
    }

    let Some((target_x, target_y, target_yaw)) = next_target(&safe_zone) else {
        return;
    };

    // TODO(@xiyuoh) publish cost via updateCosts()

    // TODO(@xiyuoh)
    current_safe_zone.update(safe_zone.clone());
    nav_target.write(NavigationTarget::new(
        safe_zone.id.plan_id.destination_session,
        target_x as f64,
        target_y as f64,
        target_yaw as f64,
    ));
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
