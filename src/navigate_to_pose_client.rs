use crate::{AgentName, RclrsExecutorCommands, RclrsNode, RosActionClient};
use bevy::prelude::*;
use crossflow::{prelude::*, service::Service};
use futures::StreamExt;
use geometry_msgs::msg::{Point, PoseStamped, Quaternion};
use nalgebra::UnitQuaternion;
use nav2_msgs::action::{NavigateToPose, NavigateToPose_Goal};
use rclrs::*;
use rmf_prototype_msgs::msg::PlanId;
use std::{future::Future, sync::Arc};
use thiserror::Error;

#[derive(Clone, Debug, Event)]
pub struct NavigationTarget {
    agent: Entity,
    id: PlanId,
    x: f64,
    y: f64,
    yaw: f64,
}

impl NavigationTarget {
    pub fn new(agent: Entity, id: PlanId, x: f64, y: f64, yaw: f64) -> Self {
        Self {
            agent,
            id,
            x,
            y,
            yaw,
        }
    }
}

#[derive(Component, Clone)]
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

#[derive(Default)]
pub struct NavigateToPoseClientPlugin {}

impl Plugin for NavigateToPoseClientPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<NavigationTarget>()
            .add_observer(initialize_new_inner_navigation_client);

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

fn initialize_new_inner_navigation_client(
    trigger: Trigger<OnAdd, AgentName>,
    mut commands: Commands,
    agent_names: Query<&AgentName>,
    node: Res<RclrsNode>,
) {
    let e = trigger.target();
    let Ok(agent_name) = agent_names.get(e).map(|agent| agent.0.clone()) else {
        return;
    };
    let action_name = agent_name + "/inner/navigate_to_pose";
    // Set up client for the inner Nav2 action
    let inner_action_client = RosActionClient::<NavigateToPose>::new(&node, action_name);

    commands.entity(e).insert(InnerNavigateToPoseClient {
        action_client: Arc::new(inner_action_client),
        goal_client: None,
    });
}

#[derive(Clone)]
struct PendingNavigationRequest {
    agent: Entity,
    plan_id: PlanId,
    target_pose: PoseStamped,
}

#[derive(Clone)]
struct NavigationRequest {
    agent: Entity,
    plan_id: PlanId,
    target_pose: PoseStamped,
    goal_client: GoalClient<NavigateToPose>,
}

#[derive(Clone)]
struct NavigationCancelRequest {
    request: PendingNavigationRequest,
    cancel_client: GoalClient<NavigateToPose>,
}

#[derive(Clone)]
struct NavigationSuccess {
    pub request: NavigationRequest,
}

#[derive(Clone)]
struct NavigationError {
    pub request: Option<NavigationRequest>,
    pub kind: NavigationErrorKind,
}

#[derive(Clone, Debug, Error)]
enum NavigationErrorKind {
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
            let trim = builder.create_trim::<PendingNavigationRequest>(Some(
                TrimBranch::downstream(async_request_new_goal.input),
            ));
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
    srv: ContinuousService<(), (), StreamOf<PendingNavigationRequest>>,
    mut orders: ContinuousQuery<(), (), StreamOf<PendingNavigationRequest>>,
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
        let pending_request = PendingNavigationRequest {
            agent: target.agent,
            plan_id: target.id.clone(),
            target_pose: goal_pose,
        };

        orders.for_each(|order| order.streams().send(pending_request.clone()));
    }
}

fn check_existing_goal(
    Blocking { request, .. }: Blocking<PendingNavigationRequest>,
    inner_nav_clients: Query<&InnerNavigateToPoseClient>,
) -> Result<NavigationCancelRequest, PendingNavigationRequest> {
    if let Some(existing_goal) = inner_nav_clients
        .get(request.agent)
        .ok()
        .and_then(|inner_action_client| inner_action_client.goal_client.clone())
    {
        return Ok(NavigationCancelRequest {
            request,
            cancel_client: existing_goal.clone(),
        });
    }
    return Err(request);
}

fn async_cancel_goal(
    Async { request, .. }: Async<NavigationCancelRequest>,
    executor_commands: Res<RclrsExecutorCommands>,
) -> impl Future<Output = Result<PendingNavigationRequest, NavigationError>> {
    executor_commands
        .run(async move {
            let cancellation = request.cancel_client.cancellation.cancel().await;
            if cancellation.is_accepted() {
                return Ok(request.request);
            }
            Err(NavigationError {
                request: None,
                kind: NavigationErrorKind::CancelGoalError,
            })
        })
        .then(|res| async move {
            res.unwrap_or(Err(NavigationError {
                request: None,
                kind: NavigationErrorKind::CancelGoalError,
            }))
        })
}

fn async_request_new_goal(
    Async { request, .. }: Async<PendingNavigationRequest>,
    mut inner_nav_clients: Query<&mut InnerNavigateToPoseClient>,
    executor_commands: Res<RclrsExecutorCommands>,
) -> impl Future<Output = Result<NavigationRequest, NavigationError>> {
    let inner_nav_client_result = inner_nav_clients.get_mut(request.agent);
    if inner_nav_client_result.is_err() {
        return std::future::ready(Err(NavigationError {
            request: None,
            kind: NavigationErrorKind::RequestGoalError,
        }))
        .left_future();
    }
    let mut inner_nav_client = inner_nav_client_result.unwrap();

    // Reset goal client before submitting new request
    inner_nav_client.reset_goal_client();

    let nav_request = inner_nav_client
        .action_client
        .request_goal(NavigateToPose_Goal {
            pose: request.target_pose.clone(),
            ..default()
        });

    executor_commands
        .run(async move {
            match nav_request.await {
                Some(handle) => Ok(NavigationRequest {
                    agent: request.agent,
                    plan_id: request.plan_id.clone(),
                    target_pose: request.target_pose,
                    goal_client: handle,
                }),
                None => Err(NavigationError {
                    request: None,
                    kind: NavigationErrorKind::RequestGoalError,
                }),
            }
        })
        .then(|res| async move {
            res.unwrap_or(Err(NavigationError {
                request: None,
                kind: NavigationErrorKind::RequestGoalError,
            }))
        })
        .right_future()
}

fn update_goal_client(
    Blocking { request, .. }: Blocking<NavigationRequest>,
    mut inner_nav_clients: Query<&mut InnerNavigateToPoseClient>,
) -> NavigationRequest {
    if let Ok(mut inner_nav_client) = inner_nav_clients.get_mut(request.agent) {
        *inner_nav_client.goal_client_mut() = Some(request.goal_client.clone());
    } else {
        println!(
            "[warning] InnerNavigateToPoseClient not found for agent [{:?}]!",
            request.agent.index()
        );
    }
    request
}

fn async_monitor_new_navigation_request(
    Async { request, .. }: Async<NavigationRequest>,
    executor_commands: Res<RclrsExecutorCommands>,
) -> impl Future<Output = NavigationResult> {
    let nav_request = request.clone();
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
                                return Err(NavigationError {
                                    request: Some(request.clone()),
                                    kind: NavigationErrorKind::GoalAbortedError,
                                })
                            }
                            _ => {}
                        }
                    }
                }
            }
            Err(NavigationError {
                request: Some(request.clone()),
                kind: NavigationErrorKind::UnknownError,
            })
        })
        .then(|res| async move {
            res.unwrap_or(Err(NavigationError {
                request: Some(nav_request.clone()),
                kind: NavigationErrorKind::UnknownError,
            }))
        })
}

fn cleanup_goal_client(
    Blocking {
        request: result, ..
    }: Blocking<NavigationResult>,
    mut inner_nav_clients: Query<&mut InnerNavigateToPoseClient>,
) {
    let request = match result {
        Ok(res) => res.request,
        Err(err) => {
            let Some(req) = err.request else {
                return;
            };
            req
        }
    };

    // Clear inner goal client
    if let Ok(mut inner_nav_client) = inner_nav_clients.get_mut(request.agent) {
        inner_nav_client.reset_goal_client();
    } else {
        println!(
            "[warning] InnerNavigateToPoseClient not found for agent [{:?}]!",
            request.agent.index()
        );
    }
}
