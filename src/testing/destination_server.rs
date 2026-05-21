use crate::{Nav2Agent, RclrsNode, RosSubscription};
use bevy::prelude::*;
use rmf_prototype_msgs::msg::DestinationGoal;
use std::sync::Arc;

#[derive(Component)]
pub struct DestinationGoalSubscription {
    pub subscriber: Arc<RosSubscription<DestinationGoal>>,
}

#[derive(Default)]
pub struct MockDestinationServerPlugin {}

impl Plugin for MockDestinationServerPlugin {
    fn build(&self, app: &mut App) {
        app.add_observer(create_destination_goal_subscription);
    }
}

fn create_destination_goal_subscription(
    trigger: Trigger<OnAdd, Nav2Agent>,
    mut commands: Commands,
    agents: Query<&Nav2Agent>,
    node: Res<RclrsNode>,
) {
    let e = trigger.target();
    let Ok(agent_name) = agents.get(e).map(|agent| agent.name.clone()) else {
        return;
    };
    let topic = agent_name + "/destination/goal";
    let subscriber = Arc::new(RosSubscription::<DestinationGoal>::new(
        &node,
        topic.clone(),
    ));
    commands.entity(e).insert(DestinationGoalSubscription {
        subscriber: Arc::clone(&subscriber),
    });
}
