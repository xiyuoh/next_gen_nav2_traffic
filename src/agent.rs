use crate::{RclrsNode, RosSubscription};
use bevy::prelude::*;
use mapf::negotiation::scenario::Agent;
use ros_env::{geometry_msgs::msg::PoseWithCovarianceStamped, rmf_prototype_msgs::msg::SafeZoneId};
use std::sync::Arc;

#[derive(Component, Clone, Debug)]
pub struct Nav2Agent {
    pub agent: Agent,
    pub id: i32,
    pub name: String,
    pub localized: bool,
    pub last_safe_zone_id: Option<SafeZoneId>,
}

impl Nav2Agent {
    pub fn new(name: String) -> Self {
        // TODO(@xiyuoh) review this - danger of duplicate IDs
        // Maybe have an accumulator mapping id to agent name as a resource
        let id = name
            .chars()
            .last()
            .and_then(|id| id.to_digit(10))
            .unwrap_or(0) as i32;

        Self {
            agent: Agent {
                start: [0, 0],
                yaw: 0.0,
                goal: [10, 10],
                radius: 0.5,
                speed: 1.0,
                spin: 1.0,
            },
            id,
            name,
            localized: false,
            last_safe_zone_id: None,
        }
    }
}

#[derive(Component)]
pub struct AmclPose(pub PoseWithCovarianceStamped);

#[derive(Component)]
pub struct AmclPoseSubscription {
    pub subscriber: Arc<RosSubscription<PoseWithCovarianceStamped>>,
}

#[derive(Default)]
pub struct Nav2AgentPlugin {}

impl Plugin for Nav2AgentPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(PreUpdate, update_amcl_pose)
            .add_observer(create_amcl_pose_subscriber);
    }
}

fn create_amcl_pose_subscriber(
    trigger: Trigger<OnAdd, Nav2Agent>,
    mut commands: Commands,
    agents: Query<&Nav2Agent>,
    node: Res<RclrsNode>,
) {
    let e = trigger.target();
    let Ok(agent_name) = agents.get(e).map(|agent| agent.name.clone()) else {
        return;
    };
    let topic = agent_name.clone() + "/inner/amcl_pose";
    let subscriber = Arc::new(RosSubscription::<PoseWithCovarianceStamped>::new(
        &node,
        topic.clone(),
    ));
    commands.entity(e).insert((
        AmclPoseSubscription {
            subscriber: Arc::clone(&subscriber),
        },
        AmclPose(PoseWithCovarianceStamped::default()),
    ));
}

fn update_amcl_pose(mut agents: Query<(&mut AmclPose, &AmclPoseSubscription)>) {
    for (mut amcl_pose, amcl_pose_sub) in agents.iter_mut() {
        let Some(amcl_pose_msg) = amcl_pose_sub.subscriber.data_callback() else {
            continue;
        };
        if amcl_pose.0 == amcl_pose_msg {
            continue;
        }
        amcl_pose.0 = amcl_pose_msg.clone();
    }
}
