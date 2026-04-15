use nav2_traffic::*;
use rclrs::*;
use std::{env, thread, time::Duration};

fn main() -> Result<(), RclrsError> {
    println!("Hello, world!");

    let context = Context::new(env::args()).unwrap();
    let subscription = Arc::new(NavSubscriptionNode::new(&context)).unwrap();
    let subscription_other_thread = Arc::clone(&subscription);
    thread::spawn(move || loop {
        thread::sleep(Duration::from_millis(1000));
        subscription_other_thread.data_callback().unwrap();
    });
    rclrs::spin(subscription.node.clone())
}
