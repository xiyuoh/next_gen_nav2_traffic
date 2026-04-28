use bevy::prelude::*;
use nav2_traffic::*;
use std::{env, thread, time::Duration};

fn main() {
    println!("Hello, world!");

    let mut app = App::new();
    app.add_plugins(DefaultPlugins);
    app.add_plugins(Nav2TrafficPlugin::default());
    app.run();
}
