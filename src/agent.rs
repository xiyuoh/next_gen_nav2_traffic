use crate::{RclrsNode, RosSubscription};
use bevy::prelude::*;
use geometry_msgs::msg::PoseWithCovarianceStamped;
use mapf::negotiation::scenario::Agent;
use std::sync::Arc;
use tf2_msgs::msg::TFMessage;

#[derive(Component, Clone, Debug)]
pub struct Nav2Agent {
    pub agent: Agent,
    pub name: String,
    pub localized: bool,
}

impl Nav2Agent {
    pub fn new(name: String) -> Self {
        Self {
            agent: Agent {
                start: [0, 0],
                yaw: 0.0,
                goal: [10, 10],
                radius: 0.5,
                speed: 1.0,
                spin: 1.0,
            },
            name,
            localized: false,
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
        app.add_observer(create_tf_subscriber);

        let agent_names = vec!["robot0".to_string(), "robot1".to_string()];
        // Spawn agents
        for name in agent_names {
            app.world_mut().spawn(Nav2Agent::new(name));
        }
    }
}

fn create_tf_subscriber(
    trigger: Trigger<OnAdd, Nav2Agent>,
    mut commands: Commands,
    agents: Query<&Nav2Agent>,
    node: Res<RclrsNode>,
) {
    let e = trigger.target();
    let Ok(agent_name) = agents.get(e).map(|agent| agent.name.clone()) else {
        return;
    };
    let topic = agent_name.clone() + "/amcl_pose";
    let subscriber = Arc::new(RosSubscription::<PoseWithCovarianceStamped>::new(
        &node,
        topic.clone(),
    ));
    commands.entity(e).insert((AmclPoseSubscription {
        subscriber: Arc::clone(&subscriber),
    },));
}

fn update_amcl_pose(mut agents: Query<(&mut AmclPose, &AmclPoseSubscription)>) {
    for (mut amcl_pose, amcl_pose_sub) in agents.iter_mut() {
        let Some(amcl_pose_msg) = amcl_pose_sub.subscriber.data_callback() else {
            continue;
        };
        amcl_pose.0 = amcl_pose_msg.clone();
    }
}

fn localize_agent(mut agents: Query<(&mut Nav2Agent, &AmclPose)>) {
    for (mut agent, amcl_pose) in agents.iter_mut() {
        if agent.localized {
            continue;
        }

        let pose = amcl_pose.0.pose.pose.clone();
        let cell_x = pose.position.x.round() as i64;
        let cell_y = pose.position.y.round() as i64;
        agent.agent.start = [cell_x, cell_y];
        agent.agent.yaw = pose.orientation.z.atan2(pose.orientation.w) * 2.0;
        agent.localized = true;

        // Set goal = start to prevent unwanted planning
        agent.agent.goal = agent.agent.start;
    }
}
