use crate::{AgentName, RclrsNode, RosActionServer};
use bevy::prelude::*;
use nav2_msgs::action::NavigateToPose;
use rclrs::*;
use std::{sync::Arc, time::Duration};

// TODO(@xiyuoh) set up an action server to receive goals over ~/navigate_to_pose,
// and publish them to ~/destination/goal.

#[derive(Component)]
pub struct NavigateToPoseServer {
    pub action_server: Arc<RosActionServer<NavigateToPose>>,
}

#[derive(Default)]
pub struct NavigateToPoseServerPlugin {}

impl Plugin for NavigateToPoseServerPlugin {
    fn build(&self, app: &mut App) {
        app.add_observer(initialize_new_navigation_server);
    }
}

fn initialize_new_navigation_server(
    trigger: Trigger<OnAdd, AgentName>,
    mut commands: Commands,
    agent_names: Query<&AgentName>,
    node: Res<RclrsNode>,
) {
    let e = trigger.target();
    let Ok(agent_name) = agent_names.get(e).map(|agent| agent.0.clone()) else {
        return;
    };
    let action_name = agent_name + "/navigate_to_pose";
    // Set up action server for incoming navigation goals
    let action_server = RosActionServer::<NavigateToPose>::new(&node, action_name, |handle| {
        nav_to_pose_action(handle, NavigateToPoseActionSettings::default())
    });

    commands.entity(e).insert(NavigateToPoseServer {
        action_server: Arc::new(action_server),
    });
}

// TODO(@xiyuoh) implement actual action server logic
async fn nav_to_pose_action(
    handle: RequestedGoal<NavigateToPose>,
    NavigateToPoseActionSettings {
        period,
        cancel_refusal_limit,
        continue_after_cancelling,
    }: NavigateToPoseActionSettings,
) -> TerminatedGoal {
    handle.reject()
}

// =============================================================================
// Taken from https://github.com/ros2-rust/ros2_rust/blob/2067da8998bd6ad78d7c6edae846d96880fd3e27/rclrs/src/action.rs#L613-L614
struct NavigateToPoseActionSettings {
    period: Duration,
    cancel_refusal_limit: usize,
    continue_after_cancelling: bool,
}

impl Default for NavigateToPoseActionSettings {
    fn default() -> Self {
        Self {
            period: Duration::from_micros(10),
            cancel_refusal_limit: 3,
            continue_after_cancelling: false,
        }
    }
}

impl NavigateToPoseActionSettings {
    fn slow() -> Self {
        NavigateToPoseActionSettings {
            period: Duration::from_secs(1),
            ..Default::default()
        }
    }

    fn cancel_refusal(mut self, limit: usize) -> Self {
        self.cancel_refusal_limit = limit;
        self
    }

    fn continue_after_cancelling(mut self) -> Self {
        self.continue_after_cancelling = true;
        self
    }
}
