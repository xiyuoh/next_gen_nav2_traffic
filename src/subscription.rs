use bevy::prelude::*;
use rclrs::*;
use std::sync::{Arc, Mutex};
use std_msgs::msg::String as StringMsg;

#[derive(Resource, Deref)]
pub struct RclrsExecutorCommands(Arc<ExecutorCommands>);

#[derive(Default)]
pub(crate) struct RclrsPlugin;

impl Plugin for RclrsPlugin {
    fn build(&self, app: &mut App) {
        let mut executor = Context::default_from_env().unwrap().create_basic_executor();
        app.insert_resource(RclrsExecutorCommands(Arc::clone(executor.commands())));

        std::thread::spawn(move || {
            let r = executor.spin(SpinOptions::default());
            for err in r {
                error!("An error occurred in rclrs: {err}");
            }
        });
    }
}

pub struct NavSubscriptionNode {
    node: Arc<Node>,
    _subscriber: Arc<Subscription<StringMsg>>,
    data: Arc<Mutex<Option<StringMsg>>>,
}

impl NavSubscriptionNode {
    pub fn new(context: &Context) -> Self {
        let node = context.create_node("nav_subscription_node").unwrap();
        let data = Arc::new(Mutex::new(None));
        let data_clone: Arc<Mutex<Option<StringMsg>>> = Arc::clone(&data);

        let subscriber = node
            .create_subscription("navigate_to_pose", QOS_PROFILE_DEFAULT, move |msg| {
                info!("Received a message: {:?}", msg);
                *data_clone.lock().unwrap() = Some(msg.clone());
            })
            .unwrap();

        Self {
            node: Arc::new(node),
            _subscriber: Arc::new(subscriber),
            data,
        }
    }

    fn data_callback(&self) -> Result<(), RclrsError> {
        if let Some(data) = self.data.lock().unwrap().as_ref() {
            println!("{}", data.data);
        } else {
            println!("No data received yet.");
        }
        Ok(())
    }
}

// pub struct Ros2SubscriptionPlugin;

// impl Plugin for Ros2SubscriptionPlugin {
//     fn build(&self, app: &mut App) {
//         let executor_commands = app.world().resource::<RclrsExecutorCommands>();
//         let node = executor_commands.create_node("nav2_traffic_node").unwrap();

//         // app.insert_resource(RosConfig::new(node.clone));

//         let nav_sub = node
//             .create_subscription("navigate_to_pose", QOSProfile::default(), move |msg| {
//                 info!("Received a message: {:?}", msg);
//             })
//             .unwrap();
//     }
// }
