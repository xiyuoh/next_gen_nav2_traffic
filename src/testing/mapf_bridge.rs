use crate::{testing::SafeZoneReceived, AmclPose, Nav2Agent, RclrsNode, RosSubscription};
use bevy::prelude::*;
use crossbeam::channel::{unbounded, Receiver};
use crossflow::{prelude::*, service::Service};
use nav2_msgs::msg::Costmap;
use rclrs::*;
use reqwest::blocking::Client;
use rmf_prototype_msgs::msg::{
    DestinationConstraints, PlanId, Region, SafeZone, SafeZoneId, TargetOrientation, TargetRegion,
};
use rosidl_runtime_rs::BoundedSequence;
use serde::{Deserialize, Serialize};
use std::{collections::HashSet, sync::Arc};
use std_srvs::srv::{Empty, Empty_Request, Empty_Response};
use unique_identifier_msgs::msg::UUID as RosUuid;
use uuid::Uuid;

#[derive(Event, Clone)]
pub struct RequestPlan {
    agent: Entity,
    request: AgentPoseRequest,
}

#[derive(Resource)]
pub struct RequestPlanService {
    receiver: Receiver<()>,
    service: Arc<ServiceState<Empty, Arc<NodeState>>>,
}

#[derive(Resource)]
pub struct MapfPostClient {
    client: Client,
    url: String,
}

// --- API Request/Response Structs from mapf_post ---
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AgentPoseRequest {
    pub agent_id: usize,
    pub x: f32,
    pub y: f32,
    pub angle: f32, // Added angle for Isometry2
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AgentAllocationResponse {
    pub agent_id: usize,
    pub cell_size: f32,
    pub allocated_free_space: Vec<(f32, f32)>,
    pub next_goal: (f32, f32),
    pub remaining_traj: Vec<(f32, f32)>,
}

#[derive(Component)]
pub struct CostmapSubscription {
    pub subscriber: Arc<RosSubscription<Costmap>>,
}

#[derive(Default)]
pub struct MapfBridgePlugin {}

impl Plugin for MapfBridgePlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(MapfPostClient {
            client: Client::new(),
            url: "http://127.0.0.1:3000/update_pose".into(),
        })
        .add_event::<RequestPlan>()
        .add_observer(create_costmap_subscriber);

        let allocation_services = MapfAllocationServices::from_app(app);
        app.insert_resource(allocation_services);

        let (tx, rx) = unbounded::<()>();
        let tx_for_closure = tx.clone();
        let node = app.world().resource::<RclrsNode>();
        let service_name = "/request_plan".to_string();
        let service = node
            .create_service::<Empty, _>(
                service_name.keep_all().transient_local(),
                move |_request: Empty_Request| {
                    info!("Received a new RequestPlan!");
                    let _ = tx_for_closure.send(());
                    Empty_Response {
                        structure_needs_at_least_one_member: 0,
                    }
                },
            )
            .unwrap();
        app.insert_resource(RequestPlanService {
            receiver: rx,
            service,
        })
        .add_systems(PreUpdate, listen_for_request);
    }
}

fn create_costmap_subscriber(
    trigger: Trigger<OnAdd, Nav2Agent>,
    mut commands: Commands,
    agents: Query<&Nav2Agent>,
    node: Res<RclrsNode>,
) {
    let e = trigger.target();
    let Ok(agent_name) = agents.get(e).map(|agent| agent.name.clone()) else {
        return;
    };
    let topic = agent_name + "/global_costmap/costmap_raw";
    let subscription = Arc::new(RosSubscription::<Costmap>::new(&node, topic.clone()));
    commands.entity(e).insert(CostmapSubscription {
        subscriber: Arc::clone(&subscription),
    });
}

#[derive(Resource)]
pub struct MapfAllocationServices {
    allocation: Service<RequestPlan, (), ()>,
}

impl MapfAllocationServices {
    pub fn from_app(app: &mut App) -> Self {
        let request_mapf_plan_service = app.spawn_service(request_mapf_plan);
        let convert_to_safe_zone_service = app.spawn_service(mapf_plan_to_safe_zone);
        let update_safe_zone_service = app.spawn_service(update_safe_zone);

        let allocation = app.world_mut().spawn_workflow(|scope, builder| {
            let request = builder
                .chain(scope.start)
                .then_node(request_mapf_plan_service);

            let convert_to_safe_zone = builder.create_node(convert_to_safe_zone_service);
            let update_safe_zone = builder.create_node(update_safe_zone_service);

            let (request_fork_result_input, request_fork_result) = builder.create_fork_result();
            builder.connect(request.output, request_fork_result_input);
            builder.connect(request_fork_result.ok, convert_to_safe_zone.input);
            builder.connect(request_fork_result.err, scope.terminate);

            let (convert_fork_result_input, convert_fork_result) = builder.create_fork_result();
            builder.connect(convert_to_safe_zone.output, convert_fork_result_input);
            builder.connect(convert_fork_result.ok, update_safe_zone.input);
            builder.connect(convert_fork_result.err, scope.terminate);

            builder.connect(update_safe_zone.output, scope.terminate);
        });

        Self { allocation }
    }
}

// NOTE(@xiyuoh) Make this blocking instead of async because we do not spawn
// an active tokio reactor
fn request_mapf_plan(
    Blocking { request, .. }: Blocking<RequestPlan>,
    mapf_post_client: Res<MapfPostClient>,
) -> Result<(RequestPlan, AgentAllocationResponse), ()> {
    let client = mapf_post_client.client.clone();
    let url = mapf_post_client.url.clone();
    info!(
        "Sending POST request payload to {}...",
        mapf_post_client.url
    );

    let response = match client.post(url).json(&request.request).send() {
        Ok(res) => res,
        Err(e) => {
            error!("Failed to send POST request: {:?}", e);
            return Err(());
        }
    };

    if response.status().is_success() {
        let Ok(server_response) = response.json::<AgentAllocationResponse>() else {
            return Err(());
        };
        info!("Response received from mapf_post: {:?}", server_response);
        return Ok((request, server_response));
    } else {
        info!("Server returned an error status: {}", response.status());
        if let Ok(error_text) = response.text() {
            error!("Error requesting mapf plan: {:?}", error_text);
        }
        return Err(());
    }
}

fn mapf_plan_to_safe_zone(
    Blocking { request, .. }: Blocking<(RequestPlan, AgentAllocationResponse)>,
    mut costmap_sub: Query<(&mut Nav2Agent, &CostmapSubscription)>,
) -> Result<(RequestPlan, SafeZone), ()> {
    let Ok((mut agent, subscription)) = costmap_sub.get_mut(request.0.agent) else {
        return Err(());
    };

    let Some(latest_costmap) = subscription.subscriber.data_callback() else {
        return Err(());
    };

    let allocation = request.1;
    if allocation.agent_id as i32 != agent.id {
        return Err(());
    }

    let mut costmap = latest_costmap.clone();
    let mut costmap_data = latest_costmap.data.clone();
    let mut safe_indices = HashSet::with_capacity(allocation.allocated_free_space.len());
    let metadata = latest_costmap.metadata;
    let costmap_origin_x = metadata.origin.position.x;
    let costmap_origin_y = metadata.origin.position.y;

    let costmap_res = metadata.resolution as f64;
    let half_cell = (allocation.cell_size / 2.0) as f64;

    // Assign allocated free space to the costmap with a bounding box
    for &(world_x, world_y) in &allocation.allocated_free_space {
        let world_min_x = world_x as f64 - half_cell;
        let world_max_x = world_x as f64 + half_cell;
        let world_min_y = world_y as f64 - half_cell;
        let world_max_y = world_y as f64 + half_cell;

        let start_cell_x = ((world_min_x - costmap_origin_x) / costmap_res).floor() as i32;
        let end_cell_x = ((world_max_x - costmap_origin_x) / costmap_res).floor() as i32;
        let start_cell_y = ((world_min_y - costmap_origin_y) / costmap_res).floor() as i32;
        let end_cell_y = ((world_max_y - costmap_origin_y) / costmap_res).floor() as i32;

        let min_x = start_cell_x.max(0).min(metadata.size_x as i32 - 1);
        let max_x = end_cell_x.max(0).min(metadata.size_x as i32 - 1);
        let min_y = start_cell_y.max(0).min(metadata.size_y as i32 - 1);
        let max_y = end_cell_y.max(0).min(metadata.size_y as i32 - 1);

        for cell_y in min_y..=max_y {
            for cell_x in min_x..=max_x {
                let index = (cell_y * metadata.size_x as i32 + cell_x) as usize;
                safe_indices.insert(index);
            }
        }
    }

    const LETHAL_OBSTACLE: u8 = 254;
    const FREE_SPACE: u8 = 0;
    // Mark any non-free space as LETHAL_OBSTACLE
    for index in 0..costmap_data.len() {
        if safe_indices.contains(&index) {
            costmap_data[index] = FREE_SPACE;
        } else {
            costmap_data[index] = LETHAL_OBSTACLE;
        }
    }
    costmap.data = costmap_data;

    // Update SafeZoneId
    let safe_zone_id = agent.last_safe_zone_id.get_or_insert(SafeZoneId {
        plan_id: PlanId {
            destination_session: new_uuid(),
            plan_version: 0,
        },
        safe_zone_version: 0,
    });
    safe_zone_id.safe_zone_version = safe_zone_id.safe_zone_version + 1;

    let safe_zone = SafeZone {
        incremental_target: DestinationConstraints {
            regions: vec![TargetRegion {
                tolerance: 0.0,
                region: Region {
                    points: vec![allocation.next_goal.0, allocation.next_goal.1],
                    hint: Region::HINT_POINT,
                },
                orientations: vec![TargetOrientation {
                    orientation_radians: 1.57,
                    ..default()
                }],
            }],
            nodes: vec![],
        },
        costmap,
        target_waypoint: BoundedSequence::<u64, 1>::new(1), // TODO(@xiyuoh)
        last_waypoint: 0,                                   // TODO(@xiyuoh)
        target_progress: 0.0,                               // TODO(@xiyuoh)
        id: safe_zone_id.clone(),
    };

    Ok((request.0, safe_zone))
}

fn update_safe_zone(
    Blocking { request, .. }: Blocking<(RequestPlan, SafeZone)>,
    mut commands: Commands,
) {
    // Trigger a SafeZoneReceived event instead of directly publishing it; we
    // allow all publishing to be consolidated in the observer
    commands.trigger(SafeZoneReceived {
        agent: request.0.agent,
        safe_zone: request.1,
    });
}

fn listen_for_request(
    mut commands: Commands,
    agent_poses: Query<(Entity, &Nav2Agent, &AmclPose)>,
    allocation_services: Res<MapfAllocationServices>,
    request_plan_service: Res<RequestPlanService>,
) {
    while let Ok(_) = request_plan_service.receiver.try_recv() {
        for (e, agent, amcl_pose) in agent_poses.iter() {
            let pose = amcl_pose.0.pose.pose.clone();
            let req = RequestPlan {
                agent: e,
                request: AgentPoseRequest {
                    agent_id: agent.id as usize,
                    x: pose.position.x as f32,
                    y: pose.position.y as f32,
                    angle: 0.0,
                },
            };
            let _ = commands
                .request(req, allocation_services.allocation.clone())
                .detach();
        }
    }
}

fn new_uuid() -> RosUuid {
    let random_uuid = Uuid::new_v4();
    let bytes: [u8; 16] = *random_uuid.as_bytes();
    RosUuid { uuid: bytes }
}
