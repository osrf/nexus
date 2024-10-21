/*
 * Copyright (C) 2024 Johnson & Johnson
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

use bevy::prelude::*;

use std::sync::Arc;
use std::time::Duration;

// The rclrs::context is a global variable and hence will be initialized as a
// bevy resource.
#[derive(Resource)]
pub struct RosContext {
    context: rclrs::Context,
    pub node: Arc<rclrs::Node>,
}

impl Default for RosContext {
    fn default() -> Self {
        let context = rclrs::Context::new(std::env::args()).unwrap_or_else(|err| {
            panic!("Unable to initialize the ROS Context: {err}");
        });
        let node = rclrs::create_node(&context, "nexus_workcell_editor").unwrap_or_else(|err| {
            panic!("Unable to initialize the ROS Node: {err}");
        });
        RosContext { context, node }
    }
}

fn spin_node(ros_context: Res<RosContext>) {
    if ros_context.context.ok() {
        let spin_node = Arc::clone(&ros_context.node);
        let _ = rclrs::spin_once(spin_node, Some(Duration::ZERO));
    }
}

pub struct RosContextPlugin;

impl Plugin for RosContextPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<RosContext>()
            .add_systems(Update, spin_node);
    }
}
