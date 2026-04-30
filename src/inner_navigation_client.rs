use crate::{AgentName, RclrsExecutorCommands, RclrsNode, RosActionClient};
use bevy::prelude::*;
use crossflow::{prelude::*, service::Service};
use futures::StreamExt;
use geometry_msgs::msg::{Point, PoseStamped, Quaternion};
use nalgebra::UnitQuaternion;
use nav2_msgs::action::{NavigateToPose, NavigateToPose_Feedback, NavigateToPose_Goal};
use rclrs::*;
use rmf_prototype_msgs::msg::SafeZoneId;
use std::{future::Future, sync::Arc};
use thiserror::Error;

#[derive(Clone, Debug, Event)]
pub struct InnerNavigationTarget {
    agent: Entity,
    safe_zone_id: SafeZoneId,
    x: f64,
    y: f64,
    yaw: f64,
}

impl InnerNavigationTarget {
    pub fn new(agent: Entity, safe_zone_id: SafeZoneId, x: f64, y: f64, yaw: f64) -> Self {
        Self {
            agent,
            safe_zone_id,
            x,
            y,
            yaw,
        }
    }
}

#[derive(Clone)]
pub struct ActiveInnerGoal {
    pub goal_client: GoalClient<NavigateToPose>,
    pub safe_zone_id: SafeZoneId,
}

impl ActiveInnerGoal {
    pub fn new(goal_client: GoalClient<NavigateToPose>, safe_zone_id: SafeZoneId) -> Self {
        Self {
            goal_client,
            safe_zone_id,
        }
    }

    pub fn client(&self) -> &GoalClient<NavigateToPose> {
        &self.goal_client
    }

    pub fn client_mut(&mut self) -> &mut GoalClient<NavigateToPose> {
        &mut self.goal_client
    }

    pub fn id(&self) -> &SafeZoneId {
        &self.safe_zone_id
    }
}

#[derive(Component, Clone)]
pub struct InnerNavigationClient {
    // Stores the action client object
    pub action_client: Arc<RosActionClient<NavigateToPose>>,
    // The ongoing action goal (if any)
    pub active_goal: Option<ActiveInnerGoal>,
}

impl InnerNavigationClient {
    pub fn new(action_client: Arc<RosActionClient<NavigateToPose>>) -> Self {
        Self {
            action_client,
            active_goal: None,
        }
    }

    pub fn goal(&self) -> &Option<ActiveInnerGoal> {
        &self.active_goal
    }

    pub fn goal_mut(&mut self) -> &mut Option<ActiveInnerGoal> {
        &mut self.active_goal
    }

    pub fn reset_goal(&mut self) {
        self.active_goal = None;
    }
}

#[derive(Default)]
pub struct InnerNavigationClientPlugin {}

impl Plugin for InnerNavigationClientPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<InnerNavigationTarget>()
            .add_event::<InnerNavigationFeedback>()
            .add_observer(create_inner_navigation_client);

        // Initialize Navigation services
        let navigation_services = InnerNavigationServices::from_app(app);
        app.insert_resource(navigation_services);
        let await_and_send_goal = app
            .world()
            .resource::<InnerNavigationServices>()
            .await_and_send_goal
            .clone();

        app.world_mut().command(|commands| {
            let _ = commands.request((), await_and_send_goal).detach();
        });
    }
}

fn create_inner_navigation_client(
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

    commands
        .entity(e)
        .insert(InnerNavigationClient::new(Arc::new(inner_action_client)));
}

#[derive(Clone)]
struct InnerNavigationRequest {
    agent: Entity,
    safe_zone_id: SafeZoneId,
    target_pose: PoseStamped,
}

#[derive(Clone)]
struct CurrentInnerNavigationGoal {
    request: InnerNavigationRequest,
    goal_client: GoalClient<NavigateToPose>,
}

#[derive(Clone)]
struct CancelInnerNavigation {
    request: InnerNavigationRequest,
    cancel_client: GoalClient<NavigateToPose>,
}

#[derive(Clone)]
struct InnerNavigationSuccess {
    pub handle: CurrentInnerNavigationGoal,
}

#[derive(Clone)]
struct InnerNavigationError {
    pub handle: Option<CurrentInnerNavigationGoal>,
    pub kind: InnerNavigationErrorKind,
}

#[derive(Clone, Debug, Error)]
enum InnerNavigationErrorKind {
    #[error("Failed to cancel current goal!")]
    CancelGoalError,
    #[error("Failed to request new goal!")]
    RequestGoalError,
    #[error("Goal was aborted!")]
    GoalAbortedError,
    #[error("Unknown error!")]
    UnknownError,
}

type InnerNavigationResult = Result<InnerNavigationSuccess, InnerNavigationError>;

#[derive(Resource)]
pub struct InnerNavigationServices {
    await_and_send_goal: Service<(), (), ()>,
}

impl InnerNavigationServices {
    pub fn from_app(app: &mut App) -> Self {
        let await_new_requests_service = app.spawn_continuous_service(Update, await_new_requests);
        let check_existing_goal_service = app.spawn_service(check_existing_goal);
        let async_cancel_goal_service = app.spawn_service(async_cancel_goal);
        let async_request_new_goal_service = app.spawn_service(async_request_new_goal);
        let update_goal_client_service = app.spawn_service(update_goal_client);
        let async_monitor_ongoing_navigation_service =
            app.spawn_service(async_monitor_ongoing_navigation);
        let cleanup_goal_client_service = app.spawn_service(cleanup_goal_client);

        let await_and_send_goal = app.world_mut().spawn_workflow(|scope, builder| {
            let await_requests = builder
                .chain(scope.start)
                .then_node(await_new_requests_service);

            // Create nodes
            let check_existing_goal = builder.create_node(check_existing_goal_service);
            let async_cancel_goal = builder.create_node(async_cancel_goal_service);
            let async_request_new_goal = builder.create_node(async_request_new_goal_service);
            let update_goal_client = builder.create_node(update_goal_client_service);
            let async_monitor_new_navigation_request =
                builder.create_node(async_monitor_ongoing_navigation_service);
            let cleanup_goal_client = builder.create_node(cleanup_goal_client_service);

            // Stream out new requests to downstream nodes
            builder.connect(await_requests.streams, check_existing_goal.input);

            // Check if there is an existing goal client; if so, connect to
            // cancellation node, else request new goal
            let (check_existing_fork_result_input, check_existing_fork_result) =
                builder.create_fork_result();
            builder.connect(check_existing_goal.output, check_existing_fork_result_input);
            builder.connect(check_existing_fork_result.ok, async_cancel_goal.input);
            builder.connect(check_existing_fork_result.err, async_request_new_goal.input);

            // Handles incoming inner/navigate_to_pose requests - cancels ongoing
            // action goals and sends new action goal.
            // Upon successful cancellation, trim any downstream nodes.
            let (cancel_goal_fork_result_input, cancel_goal_fork_result) =
                builder.create_fork_result();
            builder.connect(async_cancel_goal.output, cancel_goal_fork_result_input);
            let trim = builder.create_trim::<InnerNavigationRequest>(Some(TrimBranch::downstream(
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
            builder.connect(await_requests.output, scope.terminate);
        });

        Self {
            await_and_send_goal,
        }
    }
}

fn await_new_requests(
    srv: ContinuousService<(), (), StreamOf<InnerNavigationRequest>>,
    mut orders: ContinuousQuery<(), (), StreamOf<InnerNavigationRequest>>,
    mut nav_target: EventReader<InnerNavigationTarget>,
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
        let pending_request = InnerNavigationRequest {
            agent: target.agent,
            safe_zone_id: target.safe_zone_id.clone(),
            target_pose: goal_pose,
        };

        orders.for_each(|order| order.streams().send(pending_request.clone()));
    }
}

fn check_existing_goal(
    Blocking { request, .. }: Blocking<InnerNavigationRequest>,
    inner_nav_clients: Query<&InnerNavigationClient>,
) -> Result<CancelInnerNavigation, InnerNavigationRequest> {
    if let Some(existing_goal) = inner_nav_clients
        .get(request.agent)
        .ok()
        .and_then(|inner_client| inner_client.goal().as_ref().map(|goal| goal.client()))
    {
        return Ok(CancelInnerNavigation {
            request,
            cancel_client: existing_goal.clone(),
        });
    }
    return Err(request);
}

fn async_cancel_goal(
    Async { request, .. }: Async<CancelInnerNavigation>,
    executor_commands: Res<RclrsExecutorCommands>,
) -> impl Future<Output = Result<InnerNavigationRequest, InnerNavigationError>> {
    executor_commands
        .run(async move {
            let cancellation = request.cancel_client.cancellation.cancel().await;
            if cancellation.is_accepted() {
                return Ok(request.request);
            }
            Err(InnerNavigationError {
                handle: None,
                kind: InnerNavigationErrorKind::CancelGoalError,
            })
        })
        .then(|res| async move {
            res.unwrap_or(Err(InnerNavigationError {
                handle: None,
                kind: InnerNavigationErrorKind::CancelGoalError,
            }))
        })
}

fn async_request_new_goal(
    Async { request, .. }: Async<InnerNavigationRequest>,
    mut inner_nav_clients: Query<&mut InnerNavigationClient>,
    executor_commands: Res<RclrsExecutorCommands>,
) -> impl Future<Output = Result<CurrentInnerNavigationGoal, InnerNavigationError>> {
    let inner_nav_client_result = inner_nav_clients.get_mut(request.agent);
    if inner_nav_client_result.is_err() {
        return std::future::ready(Err(InnerNavigationError {
            handle: None,
            kind: InnerNavigationErrorKind::RequestGoalError,
        }))
        .left_future();
    }
    let mut inner_nav_client = inner_nav_client_result.unwrap();

    // Reset goal client before submitting new request
    inner_nav_client.reset_goal();

    let nav_request = inner_nav_client
        .action_client
        .request_goal(NavigateToPose_Goal {
            pose: request.target_pose.clone(),
            ..default()
        });

    executor_commands
        .run(async move {
            match nav_request.await {
                Some(handle) => Ok(CurrentInnerNavigationGoal {
                    request,
                    goal_client: handle,
                }),
                None => Err(InnerNavigationError {
                    handle: None,
                    kind: InnerNavigationErrorKind::RequestGoalError,
                }),
            }
        })
        .then(|res| async move {
            res.unwrap_or(Err(InnerNavigationError {
                handle: None,
                kind: InnerNavigationErrorKind::RequestGoalError,
            }))
        })
        .right_future()
}

fn update_goal_client(
    Blocking {
        request: handle, ..
    }: Blocking<CurrentInnerNavigationGoal>,
    mut inner_nav_clients: Query<&mut InnerNavigationClient>,
) -> CurrentInnerNavigationGoal {
    if let Ok(mut inner_nav_client) = inner_nav_clients.get_mut(handle.request.agent) {
        *inner_nav_client.goal_mut() = Some(ActiveInnerGoal::new(
            handle.goal_client.clone(),
            handle.request.safe_zone_id.clone(),
        ));
    } else {
        warn!(
            "InnerNavigationClient not found for agent [{:?}]!",
            handle.request.agent.index()
        );
    }
    handle
}

#[derive(Clone, Event)]
pub struct InnerNavigationFeedback {
    pub agent: Entity,
    pub feedback: NavigateToPose_Feedback,
}

impl InnerNavigationFeedback {
    pub fn new(agent: Entity, feedback: NavigateToPose_Feedback) -> Self {
        Self { agent, feedback }
    }
}

fn async_monitor_ongoing_navigation(
    Async {
        request: handle,
        channel,
        ..
    }: Async<CurrentInnerNavigationGoal>,
    executor_commands: Res<RclrsExecutorCommands>,
) -> impl Future<Output = InnerNavigationResult> {
    let nav_handle = handle.clone();
    executor_commands
        .run(async move {
            let mut goal_client_stream = handle.goal_client.clone().stream();
            let agent = handle.request.agent.clone();
            // TODO(@xiyuoh) this gets stuck sometimes, find out why
            while let Some(event) = goal_client_stream.next().await {
                match event {
                    GoalEvent::Feedback(feedback) => {
                        // Publish feedback via observer triggers
                        channel.commands(move |cmds| {
                            cmds.trigger(InnerNavigationFeedback::new(agent, feedback.clone()));
                        });
                    }
                    GoalEvent::Status(s) => {
                        info!("[inner nav2pose] Status: {:?}", s.code);
                    }
                    GoalEvent::Result((status, result)) => {
                        info!("[inner nav2pose] Result: {:?}", result);
                        match status {
                            GoalStatusCode::Succeeded => {
                                return Ok(InnerNavigationSuccess { handle })
                            }
                            GoalStatusCode::Aborted | GoalStatusCode::Cancelled => {
                                return Err(InnerNavigationError {
                                    handle: Some(handle.clone()),
                                    kind: InnerNavigationErrorKind::GoalAbortedError,
                                })
                            }
                            _ => {}
                        }
                    }
                }
            }
            Err(InnerNavigationError {
                handle: Some(handle.clone()),
                kind: InnerNavigationErrorKind::UnknownError,
            })
        })
        .then(|res| async move {
            res.unwrap_or(Err(InnerNavigationError {
                handle: Some(nav_handle.clone()),
                kind: InnerNavigationErrorKind::UnknownError,
            }))
        })
}

fn cleanup_goal_client(
    Blocking {
        request: result, ..
    }: Blocking<InnerNavigationResult>,
    mut inner_nav_clients: Query<&mut InnerNavigationClient>,
) {
    let handle = match result {
        Ok(res) => res.handle,
        Err(err) => {
            let Some(req) = err.handle else {
                return;
            };
            req
        }
    };

    // Clear inner goal client
    if let Ok(mut inner_nav_client) = inner_nav_clients.get_mut(handle.request.agent) {
        inner_nav_client.reset_goal();
    } else {
        warn!(
            "InnerNavigationClient not found for agent [{:?}]!",
            handle.request.agent.index()
        );
    }
}
