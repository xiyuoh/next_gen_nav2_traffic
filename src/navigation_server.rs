use crate::{
    destination::DestinationGoalPublisher,
    inner_navigation_client::{InnerNavigationClient, InnerNavigationFeedback},
    AgentName, RclrsNode, RosActionServer, RosPublisher,
};
use bevy::prelude::*;
use crossbeam::channel::{unbounded, Receiver, Sender};
use crossflow::{prelude::*, service::Service};
use geometry_msgs::msg::PoseStamped;
use nav2_msgs::action::{NavigateToPose, NavigateToPose_Feedback, NavigateToPose_Result};
use rclrs::*;
use rmf_prototype_msgs::msg::{
    DestinationConstraints, DestinationGoal, PlanId, Region, TargetOrientation, TargetRegion,
};
use std::{sync::Arc, time::Duration};
use tokio::sync::mpsc::{unbounded_channel, UnboundedReceiver, UnboundedSender};
use unique_identifier_msgs::msg::UUID as RosUuid;
use uuid::Uuid;

#[derive(Component)]
pub struct NavigateToPoseServer {
    pub action_server: Arc<RosActionServer<NavigateToPose>>,
    pub request_receiver: Receiver<NavigationRequest>,
    pub request_sender: Sender<NavigationRequest>,
}

#[derive(Default)]
pub struct NavigationServerPlugin {}

impl Plugin for NavigationServerPlugin {
    fn build(&self, app: &mut App) {
        // TODO(@xiyuoh) have a systematic approach to setting schedules for systems
        app.add_systems(PreUpdate, request_publish_feedback_service)
            .add_observer(create_navigation_server)
            .add_observer(on_inner_navigation_feedback);

        let navigation_services = NavigationServices::from_app(app);
        app.insert_resource(navigation_services);
    }
}

fn create_navigation_server(
    trigger: Trigger<OnAdd, DestinationGoalPublisher>,
    mut commands: Commands,
    agents: Query<(&AgentName, &DestinationGoalPublisher)>,
    node: Res<RclrsNode>,
) {
    let agent_entity = trigger.target();
    let Ok((agent_name, destination_publisher)) = agents
        .get(agent_entity)
        .map(|(agent, publisher)| (agent.0.clone(), publisher))
    else {
        return;
    };
    let (tx, rx) = unbounded::<NavigationRequest>();
    let (tx_for_closure, publisher_for_closure) =
        (tx.clone(), destination_publisher.publisher.clone());
    let action_name = agent_name + "/navigate_to_pose";
    // Set up action server for incoming navigation goals
    let action_server = RosActionServer::<NavigateToPose>::new(&node, action_name, move |handle| {
        let (sender, receiver) = unbounded_channel::<NavigateToPose_Feedback>();
        let pose = &handle.goal().pose;
        info!("Received a new NavigationRequest to {:?}!", pose);

        // Throw sender into workflow
        let nav_request = NavigationRequest::new(agent_entity.clone(), sender, pose.clone());
        let _ = tx_for_closure.send(nav_request);

        nav_to_pose_action(
            handle,
            receiver,
            publisher_for_closure.clone(),
            NavigateToPoseActionSettings::default(),
        )
    });

    commands.entity(agent_entity).insert(NavigateToPoseServer {
        action_server: Arc::new(action_server),
        request_receiver: rx,
        request_sender: tx,
    });
}

fn request_publish_feedback_service(
    mut commands: Commands,
    mut navigation_servers: Query<&NavigateToPoseServer>,
    navigation_services: Res<NavigationServices>,
) {
    for server in navigation_servers.iter_mut() {
        while let Ok(request) = server.request_receiver.try_recv() {
            commands
                .request(request, navigation_services.publish_feedback)
                .detach();
        }
    }
}

#[derive(Resource)]
pub struct NavigationServices {
    publish_feedback: Service<NavigationRequest, (), ()>,
}

impl NavigationServices {
    pub fn from_app(app: &mut App) -> Self {
        let monitor_inner_navigation_clients_service =
            app.spawn_continuous_service(Update, monitor_inner_navigation_clients);
        let monitor_inner_navigation_feedback_service =
            app.spawn_continuous_service(Update, monitor_inner_navigation_feedback);
        let publish_navigation_feedback_service = app.spawn_service(publish_navigation_feedback);

        let publish_feedback = app.world_mut().spawn_workflow(|scope, builder| {
            // Create nodes
            let monitor_inner_navigation_clients =
                builder.create_node(monitor_inner_navigation_clients_service);
            let monitor_inner_navigation_feedback =
                builder.create_node(monitor_inner_navigation_feedback_service);
            let publish_navigation_feedback =
                builder.create_node(publish_navigation_feedback_service);

            let (fork_clone_input, fork_clone) = builder.create_fork_clone();

            // Continuous service to manage incoming navigation requests and
            // respond to orders when the entire nav is completed
            builder.connect(scope.start, fork_clone_input);
            let cloned = fork_clone.clone_output(builder);
            builder.connect(cloned, monitor_inner_navigation_clients.input);

            // Stream out inner navigation feedback buffer
            let cloned = fork_clone.clone_output(builder);
            builder.connect(cloned, monitor_inner_navigation_feedback.input);
            let feedback_buffer =
                builder.create_buffer::<InnerNavigationFeedback>(BufferSettings::keep_last(10));
            builder.connect(
                monitor_inner_navigation_feedback.streams,
                feedback_buffer.input_slot(),
            );

            // Publish feedback
            let access = builder
                .create_buffer_access::<NavigationRequest, Buffer<InnerNavigationFeedback>>(
                    feedback_buffer,
                );
            builder.connect(monitor_inner_navigation_clients.streams, access.input);
            builder.connect(access.output, publish_navigation_feedback.input);

            // Connect only monitor_inner_navigation_clients to terminate
            builder.connect(monitor_inner_navigation_clients.output, scope.terminate);
        });

        Self { publish_feedback }
    }
}

async fn nav_to_pose_action(
    handle: RequestedGoal<NavigateToPose>,
    mut receiver: UnboundedReceiver<NavigateToPose_Feedback>,
    destination_publisher: Arc<RosPublisher<DestinationGoal>>,
    NavigateToPoseActionSettings {
        period,
        cancel_refusal_limit,
        continue_after_cancelling,
    }: NavigateToPoseActionSettings,
) -> TerminatedGoal {
    let goal_order = &handle.goal().pose;
    let destination_goal = pose_stamped_to_destination_goal(goal_order);
    let Ok(_) = destination_publisher.publish(destination_goal) else {
        return handle.reject();
    };

    let result = NavigateToPose_Result::default();

    let executing = match handle.accept().begin() {
        BeginAcceptedGoal::Execute(executing) => executing,
        BeginAcceptedGoal::Cancel(cancelling) => return cancelling.cancelled_with(result),
    };

    let mut cancel_requests = 0;
    let cancelling = loop {
        match executing.unless_cancel_requested(receiver.recv()).await {
            Ok(Some(next)) => {
                executing.publish_feedback(next);
            }
            Ok(None) => {
                // Navigation for this PlanId completed
                info!("NavigationRequest completed!");
                return executing.succeeded_with(result);
            }
            Err(_) => {
                // Cancellation requested
                cancel_requests += 1;
                if cancel_requests > cancel_refusal_limit {
                    let cancelling = executing.begin_cancelling();
                    if !continue_after_cancelling {
                        return cancelling.cancelled_with(result);
                    }
                    break cancelling;
                }
            }
        }
    };

    // TODO(@xiyuoh) cleanup while cancelling?
    info!("NavigationRequest cancelled!");
    return cancelling.succeeded_with(result);
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
        session: new_uuid(),
    }
}

fn new_uuid() -> RosUuid {
    let random_uuid = Uuid::new_v4();
    let bytes: [u8; 16] = *random_uuid.as_bytes();
    RosUuid { uuid: bytes }
}

fn dist(a: (f64, f64), b: (f64, f64)) -> f64 {
    ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)).sqrt()
}

#[derive(Clone)]
pub struct NavigationRequest {
    pub agent: Entity,
    pub plan_id: PlanId,
    pub sender: UnboundedSender<NavigateToPose_Feedback>,
    pub target: PoseStamped,
    pub threshold: f64,
}

impl NavigationRequest {
    pub fn new(
        agent: Entity,
        sender: UnboundedSender<NavigateToPose_Feedback>,
        target: PoseStamped,
    ) -> Self {
        Self {
            agent,
            plan_id: PlanId {
                destination_session: new_uuid(),
                plan_version: 0,
            },
            sender,
            target,
            threshold: 0.5,
        }
    }

    pub fn threshold(&mut self, threshold: f64) -> &mut Self {
        self.threshold = threshold;
        self
    }

    pub fn plan_id(&mut self, plan_id: PlanId) -> &mut Self {
        self.plan_id = plan_id;
        self
    }

    pub fn destination_reached(&self, pose: &AgentPose) -> bool {
        let current_pos = &pose.pose().pose.position;
        let target_pos = &self.target.pose.position;
        if dist((current_pos.x, current_pos.y), (target_pos.x, target_pos.y)) < self.threshold {
            return true;
        }
        false
    }
}

#[derive(Clone, Debug, Component)]
pub struct AgentPose(pub PoseStamped);

impl AgentPose {
    pub fn pose(&self) -> &PoseStamped {
        &self.0
    }

    pub fn pose_mut(&mut self) -> &mut PoseStamped {
        &mut self.0
    }
}

fn on_inner_navigation_feedback(
    trigger: Trigger<InnerNavigationFeedback>,
    mut inner_nav_feedback: EventWriter<InnerNavigationFeedback>,
) {
    let event = trigger.event();
    inner_nav_feedback.write(InnerNavigationFeedback::new(
        event.agent,
        event.feedback.clone(),
    ));
}

fn monitor_inner_navigation_feedback(
    srv: ContinuousService<NavigationRequest, (), StreamOf<InnerNavigationFeedback>>,
    mut orders: ContinuousQuery<NavigationRequest, (), StreamOf<InnerNavigationFeedback>>,
    mut inner_nav_feedback: EventReader<InnerNavigationFeedback>,
) {
    let Some(mut orders) = orders.get_mut(&srv.key) else {
        return;
    };
    if orders.is_empty() {
        return;
    }

    for feedback in inner_nav_feedback.read() {
        orders.for_each(|order| {
            if feedback.agent == order.request().agent {
                order.streams().send(feedback.clone())
            }
        });
    }
}

fn monitor_inner_navigation_clients(
    srv: ContinuousService<NavigationRequest, (), StreamOf<NavigationRequest>>,
    mut orders: ContinuousQuery<NavigationRequest, (), StreamOf<NavigationRequest>>,
    agents: Query<(&InnerNavigationClient, &AgentPose)>,
) {
    let Some(mut orders) = orders.get_mut(&srv.key) else {
        return;
    };
    if orders.is_empty() {
        return;
    }

    orders.for_each(|order| {
        let request = order.request();
        order.streams().send(request.clone());

        // If reached destination, complete order
        if let Ok((client, pose)) = agents.get(request.agent) {
            if client.active_goal.is_none() && request.destination_reached(pose) {
                order.respond(());
            }
        }
    });
}

fn publish_navigation_feedback(
    Blocking {
        request: (nav_request, key),
        id,
        ..
    }: Blocking<(NavigationRequest, BufferKey<InnerNavigationFeedback>)>,
    mut commands: Commands,
    mut access: BufferAccessMut<InnerNavigationFeedback>,
) {
    let Ok(feedback_vec) = access
        .get_mut(id, &key)
        .map(|mut res| res.drain(..).collect::<Vec<InnerNavigationFeedback>>())
    else {
        return;
    };

    for feedback in feedback_vec.iter() {
        // Update AgentPose
        commands
            .entity(feedback.agent)
            .insert(AgentPose(feedback.feedback.current_pose.clone()));

        // TODO(@xiyuoh) Handle cases where action has been cancelled or ended,
        // should we drop the sender?
        let _ = nav_request.sender.send(feedback.feedback.clone());
    }
}

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
