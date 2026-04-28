use crate::{RclrsExecutorCommands, RclrsNode, RosActionClient, RosActionServer, RosNamespace};
use bevy::prelude::*;
use crossflow::{prelude::*, service::Service};
use futures::StreamExt;
use geometry_msgs::msg::{Point, PoseStamped, Quaternion};
use nalgebra::UnitQuaternion;
use nav2_msgs::{
    action::{NavigateToPose, NavigateToPose_Feedback, NavigateToPose_Goal, NavigateToPose_Result},
    msg::Costmap,
};
use rclrs::{vendor::builtin_interfaces::msg::Duration as RclDuration, *};
use rmf_prototype_msgs::msg::DestinationGoal;
use std::{future::Future, sync::Arc, time::Duration};
use thiserror::Error;
use tokio::sync::mpsc::unbounded_channel;
use unique_identifier_msgs::msg::UUID;

// Observer for new safe_zone -> inner/navigate_to_pose goals
#[derive(Clone, Debug, Event)]
pub struct NavigationTarget {
    id: UUID,
    x: f64,
    y: f64,
    yaw: f64,
}

impl NavigationTarget {
    pub fn new(id: UUID, x: f64, y: f64, yaw: f64) -> Self {
        Self { id, x, y, yaw }
    }
}

// TODO(@xiyuoh) set up an action server to receive goals over ~/navigate_to_pose,
// and publish them to ~/destination/goal.

#[derive(Resource)]
pub struct InnerNavigateToPoseClient {
    // Stores the action client object
    pub action_client: Arc<RosActionClient<NavigateToPose>>,
    // Whether there is an ongoing action goal
    pub goal_client: Option<GoalClient<NavigateToPose>>,
}

impl InnerNavigateToPoseClient {
    pub fn goal_client(&self) -> &Option<GoalClient<NavigateToPose>> {
        &self.goal_client
    }

    pub fn goal_client_mut(&mut self) -> &mut Option<GoalClient<NavigateToPose>> {
        &mut self.goal_client
    }

    pub fn reset_goal_client(&mut self) {
        self.goal_client = None;
    }
}

#[derive(Resource)]
pub struct NavigateToPoseServer {
    pub action_server: Arc<RosActionServer<NavigateToPose>>,
}

#[derive(Default)]
pub struct NavigateToPosePlugin {}

impl Plugin for NavigateToPosePlugin {
    fn build(&self, app: &mut App) {
        let namespace = app.world().resource::<RosNamespace>().0.clone();
        let node = app.world().resource::<RclrsNode>();
        let action_name = "navigate_to_pose".to_string();

        // Set up action server for incoming navigation goals
        let action_server =
            RosActionServer::<NavigateToPose>::new(&node, action_name.clone(), |handle| {
                nav_to_pose_action(handle, NavigateToPoseActionSettings::default())
            });
        // Set up client for the inner Nav2 action
        let inner_action_client =
            RosActionClient::<NavigateToPose>::new(&node, namespace + "/inner/" + &action_name);

        app.insert_resource(NavigateToPoseServer {
            action_server: Arc::new(action_server),
        })
        .insert_resource(InnerNavigateToPoseClient {
            action_client: Arc::new(inner_action_client),
            goal_client: None,
        });

        app.add_event::<NavigationTarget>();

        // Initialize Navigation services
        let navigation_services = NavigationServices::from_app(app);
        app.insert_resource(navigation_services);
        let await_and_send_goal = app
            .world()
            .resource::<NavigationServices>()
            .await_and_send_goal
            .clone();

        app.world_mut().command(|commands| {
            let _ = commands.request((), await_and_send_goal).detach();
        });
    }
}

#[derive(Clone)]
struct NavigationRequest {
    target_pose: PoseStamped,
    goal_client: GoalClient<NavigateToPose>,
}

// TODO(@xiyuoh) same definition as above but serve very different purposes:
// goal_client here refers to an ongoing goal and is meant for cancellation in
// favor of the new target_pose.
#[derive(Clone)]
struct NavigationCancelRequest {
    target_pose: PoseStamped,
    goal_client: GoalClient<NavigateToPose>,
}

#[derive(Clone)]
struct NavigationSuccess {
    pub request: NavigationRequest,
}

#[derive(Clone, Debug, Error)]
enum NavigationError {
    #[error("Error!")]
    DefaultError,
    #[error("Failed to cancel current goal!")]
    CancelGoalError,
    #[error("Failed to request new goal!")]
    RequestGoalError,
    #[error("Goal was aborted!")]
    GoalAbortedError,
    #[error("Unknown error!")]
    UnknownError,
}

type NavigationResult = Result<NavigationSuccess, NavigationError>;

#[derive(Resource)]
pub struct NavigationServices {
    await_and_send_goal: Service<(), (), ()>,
}

impl NavigationServices {
    pub fn from_app(app: &mut App) -> Self {
        let await_new_nav_requests_service =
            app.spawn_continuous_service(Update, await_new_nav_requests);
        let check_existing_goal_service = app.spawn_service(check_existing_goal);
        let async_cancel_goal_service = app.spawn_service(async_cancel_goal);
        let async_request_new_goal_service = app.spawn_service(async_request_new_goal);
        let update_goal_client_service = app.spawn_service(update_goal_client);
        let async_monitor_new_navigation_request_service =
            app.spawn_service(async_monitor_new_navigation_request);
        let cleanup_goal_client_service = app.spawn_service(cleanup_goal_client);

        let await_and_send_goal = app.world_mut().spawn_workflow(|scope, builder| {
            let await_requests_node = builder
                .chain(scope.start)
                .then_node(await_new_nav_requests_service);

            // Create nodes
            let check_existing_goal = builder.create_node(check_existing_goal_service);
            let async_cancel_goal = builder.create_node(async_cancel_goal_service);
            let async_request_new_goal = builder.create_node(async_request_new_goal_service);
            let update_goal_client = builder.create_node(update_goal_client_service);
            let async_monitor_new_navigation_request =
                builder.create_node(async_monitor_new_navigation_request_service);
            let cleanup_goal_client = builder.create_node(cleanup_goal_client_service);

            // Stream out new requests to downstream nodes
            builder.connect(await_requests_node.streams, check_existing_goal.input);

            // Check if there is an existing goal client; if so, connect to
            // cancellation node, else request new goal
            let (check_existing_fork_result_input, check_existing_fork_result) =
                builder.create_fork_result();
            builder.connect(check_existing_goal.output, check_existing_fork_result_input);
            builder.connect(check_existing_fork_result.ok, async_cancel_goal.input);
            builder.connect(check_existing_fork_result.err, async_request_new_goal.input);

            // Handles incoming inner/navigate_to_pose requests - cancels ongoing
            // action goals and sends new action goal.
            // Upon successfull cancellation, trim any downstream nodes.
            let (cancel_goal_fork_result_input, cancel_goal_fork_result) =
                builder.create_fork_result();
            builder.connect(async_cancel_goal.output, cancel_goal_fork_result_input);
            let trim = builder.create_trim::<PoseStamped>(Some(TrimBranch::downstream(
                async_request_new_goal.input,
            )));
            // If current goal successfully canceled, trim any downstream nodes
            builder.connect(cancel_goal_fork_result.ok, trim.input);
            builder.connect(trim.output, async_request_new_goal.input);
            let (new_goal_fork_result_input, new_goal_fork_result) = builder.create_fork_result();
            builder.connect(async_request_new_goal.output, new_goal_fork_result_input);

            // Handles and monitors ongoing inner/navigate_to_pose request
            builder.connect(new_goal_fork_result.ok, update_goal_client.input);
            builder.connect(
                update_goal_client.output,
                async_monitor_new_navigation_request.input,
            );

            // On completed navigation request, reset goal client
            builder.connect(
                async_monitor_new_navigation_request.output,
                cleanup_goal_client.input,
            );

            // Connect only await_requests output to terminate
            builder.connect(await_requests_node.output, scope.terminate);
        });

        Self {
            await_and_send_goal,
        }
    }
}

fn await_new_nav_requests(
    srv: ContinuousService<(), (), StreamOf<PoseStamped>>,
    mut orders: ContinuousQuery<(), (), StreamOf<PoseStamped>>,
    mut nav_target: EventReader<NavigationTarget>,
    node: Res<RclrsNode>,
) {
    let Some(mut orders) = orders.get_mut(&srv.key) else {
        return;
    };
    if orders.is_empty() {
        return;
    }
    if nav_target.is_empty() {
        return;
    }

    for target in nav_target.read() {
        let now = node.get_clock().now();
        let quat = UnitQuaternion::from_euler_angles(0.0, 0.0, target.yaw);

        let goal_pose = PoseStamped {
            header: std_msgs::msg::Header {
                stamp: builtin_interfaces::msg::Time {
                    sec: (now.nsec / 1_000_000_000) as i32,
                    nanosec: (now.nsec % 1_000_000_000) as u32,
                },
                frame_id: "map".to_string(),
            },
            pose: geometry_msgs::msg::Pose {
                position: Point {
                    x: target.x,
                    y: target.y,
                    z: 0.0,
                },
                orientation: Quaternion {
                    x: quat.coords.x,
                    y: quat.coords.y,
                    z: quat.coords.z,
                    w: quat.coords.w,
                },
            },
        };

        orders.for_each(|order| order.streams().send(goal_pose.clone()));
    }
}

fn check_existing_goal(
    Blocking {
        request: target_pose,
        ..
    }: Blocking<PoseStamped>,
    inner_nav_client: Res<InnerNavigateToPoseClient>,
) -> Result<NavigationCancelRequest, PoseStamped> {
    if let Some(existing_goal) = inner_nav_client.goal_client() {
        return Ok(NavigationCancelRequest {
            target_pose,
            goal_client: existing_goal.clone(),
        });
    }
    return Err(target_pose);
}

fn async_cancel_goal(
    Async { request, .. }: Async<NavigationCancelRequest>,
    executor_commands: Res<RclrsExecutorCommands>,
) -> impl Future<Output = Result<PoseStamped, NavigationError>> {
    executor_commands
        .run(async move {
            let cancellation = request.goal_client.cancellation.cancel().await;
            if cancellation.is_accepted() {
                return Ok(request.target_pose);
            }
            Err(NavigationError::CancelGoalError)
        })
        .then(|res| async move { res.unwrap_or(Err(NavigationError::CancelGoalError)) })
}

fn async_request_new_goal(
    Async {
        request: target_pose,
        ..
    }: Async<PoseStamped>,
    mut inner_nav_client: ResMut<InnerNavigateToPoseClient>,
    executor_commands: Res<RclrsExecutorCommands>,
) -> impl Future<Output = Result<NavigationRequest, NavigationError>> {
    // Reset goal client before submitting new request
    inner_nav_client.reset_goal_client();

    let nav_request = inner_nav_client
        .action_client
        .request_goal(NavigateToPose_Goal {
            pose: target_pose.clone(),
            ..default()
        });

    executor_commands
        .run(async move {
            match nav_request.await {
                Some(handle) => Ok(NavigationRequest {
                    target_pose,
                    goal_client: handle,
                }),
                None => Err(NavigationError::RequestGoalError),
            }
        })
        .then(|res| async move { res.unwrap_or(Err(NavigationError::RequestGoalError)) })
}

fn update_goal_client(
    Blocking { request, .. }: Blocking<NavigationRequest>,
    mut inner_nav_client: ResMut<InnerNavigateToPoseClient>,
) -> NavigationRequest {
    *inner_nav_client.goal_client_mut() = Some(request.goal_client.clone());
    request
}

fn async_monitor_new_navigation_request(
    Async { request, .. }: Async<NavigationRequest>,
    executor_commands: Res<RclrsExecutorCommands>,
) -> impl Future<Output = NavigationResult> {
    executor_commands
        .run(async move {
            let mut goal_client_stream = request.goal_client.clone().stream();
            // TODO(@xiyuoh) this gets stuck sometimes, find out why
            while let Some(event) = goal_client_stream.next().await {
                match event {
                    GoalEvent::Feedback(_feedback) => {
                        // Do nothing
                    }
                    GoalEvent::Status(s) => {
                        println!("[status] Received status: {:?}", s.code);
                    }
                    GoalEvent::Result((status, result)) => {
                        println!("[result] Received result: {:?}", result);
                        match status {
                            GoalStatusCode::Succeeded => return Ok(NavigationSuccess { request }),
                            GoalStatusCode::Aborted | GoalStatusCode::Cancelled => {
                                return Err(NavigationError::GoalAbortedError)
                            }
                            _ => {}
                        }
                    }
                }
            }
            Err(NavigationError::UnknownError)
        })
        .then(|res| async move { res.unwrap_or(Err(NavigationError::UnknownError)) })
}

fn cleanup_goal_client(
    Blocking { .. }: Blocking<NavigationResult>,
    mut inner_nav_client: ResMut<InnerNavigateToPoseClient>,
) {
    // Clear inner goal client
    inner_nav_client.reset_goal_client();
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
