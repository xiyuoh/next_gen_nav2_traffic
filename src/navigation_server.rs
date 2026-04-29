use crate::{AgentName, DestinationGoalPublisher, RclrsNode, RosActionServer, RosPublisher};
use bevy::prelude::*;
use geometry_msgs::msg::PoseStamped;
use nav2_msgs::action::{NavigateToPose, NavigateToPose_Feedback, NavigateToPose_Result};
use rclrs::*;
use rmf_prototype_msgs::msg::{
    DestinationConstraints, DestinationGoal, Region, TargetOrientation, TargetRegion,
};
use std::{sync::Arc, time::Duration};
use tokio::sync::mpsc::unbounded_channel;
use unique_identifier_msgs::msg::UUID as RosUuid;
use uuid::Uuid;

// TODO(@xiyuoh) set up an action server to receive goals over ~/navigate_to_pose,
// and publish them to ~/destination/goal.

#[derive(Component)]
pub struct NavigateToPoseServer {
    pub action_server: Arc<RosActionServer<NavigateToPose>>,
}

#[derive(Default)]
pub struct NavigationServerPlugin {}

impl Plugin for NavigationServerPlugin {
    fn build(&self, app: &mut App) {
        app.add_observer(create_navigation_server);
    }
}

fn create_navigation_server(
    trigger: Trigger<OnAdd, AgentName>,
    mut commands: Commands,
    agents: Query<(&AgentName, &DestinationGoalPublisher)>,
    node: Res<RclrsNode>,
) {
    let e = trigger.target();
    let Ok((agent_name, destination_publisher)) = agents
        .get(e)
        .map(|(agent, publisher)| (agent.0.clone(), publisher))
    else {
        return;
    };
    let (agent_name_for_move, publisher_for_move) =
        (agent_name.clone(), destination_publisher.publisher.clone());
    let action_name = agent_name + "/navigate_to_pose";
    // Set up action server for incoming navigation goals
    let action_server = RosActionServer::<NavigateToPose>::new(&node, action_name, move |handle| {
        nav_to_pose_action(
            handle,
            agent_name_for_move.clone(),
            publisher_for_move.clone(),
            NavigateToPoseActionSettings::default(),
        )
    });

    commands.entity(e).insert(NavigateToPoseServer {
        action_server: Arc::new(action_server),
    });
}

// TODO(@xiyuoh) implement actual action server logic
async fn nav_to_pose_action(
    handle: RequestedGoal<NavigateToPose>,
    agent_name: String,
    destination_publisher: Arc<RosPublisher<DestinationGoal>>,
    NavigateToPoseActionSettings {
        period,
        cancel_refusal_limit,
        continue_after_cancelling,
    }: NavigateToPoseActionSettings,
) -> TerminatedGoal {
    let goal_order = &handle.goal().pose;
    handle.reject()
}

fn pose_stamped_to_destination_goal(pose: &PoseStamped) -> DestinationGoal {
    let (x, y, q) = (
        pose.pose.position.x as f32,
        pose.pose.position.y as f32,
        pose.pose.orientation.clone(),
    );
    let quat = Quat::from_xyzw(q.x as f32, q.y as f32, q.z as f32, q.w as f32);
    let (yaw, _pitch, _roll) = quat.to_euler(EulerRot::ZXY);

    // Consider this a single-point region
    let region = Region {
        points: vec![x, y],
        hint: Region::HINT_POINT,
    };
    let target_orientation = TargetOrientation {
        orientation_radians: yaw,
        spread_radians: 0.0,
        tolerance_radians: 0.0,
    };
    let target_region = TargetRegion {
        tolerance: 0.5,
        region,
        orientations: vec![target_orientation],
    };

    // Populate either regions or nodes, not both
    let constraints = DestinationConstraints {
        regions: vec![target_region],
        nodes: vec![],
    };

    DestinationGoal {
        one_of: vec![constraints],
        cost_bias: vec![],
        session: {
            let random_uuid = Uuid::new_v4();
            let bytes: [u8; 16] = *random_uuid.as_bytes();
            RosUuid { uuid: bytes }
        },
    }
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
