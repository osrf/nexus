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
use librmf_workcell_editor::bevy_egui::EguiContexts;

use crossbeam_channel::{Receiver, Sender};

use rmf_workcell_format::{workcell::*, Anchor};

use std::{collections::HashMap, sync::Arc};

use nexus_calibration_msgs::srv::CalibrateExtrinsics;
use nexus_calibration_msgs::srv::CalibrateExtrinsics_Request;
use nexus_calibration_msgs::srv::CalibrateExtrinsics_Response;

use crate::ros_context::*;

// This component will be attached to a workcell entity.
#[derive(Component)]
pub struct CalibrationClient {
    pub client: Arc<rclrs::Client<CalibrateExtrinsics>>,
}

type TransformData = HashMap<String, Transform>;
// Channels to send transform data from a system that makes the service reqeust
// to a system that will update anchor poses.
#[derive(Debug, Resource)]
pub struct CalibrationChannels {
    pub sender: Sender<TransformData>,
    pub receiver: Receiver<TransformData>,
}

impl Default for CalibrationChannels {
    fn default() -> Self {
        let (sender, receiver) = crossbeam_channel::unbounded();
        Self { sender, receiver }
    }
}

// A system to insert the CalibrationClient component into a workcell entity.
pub fn add_calibration_client(
    mut commands: Commands,
    workcells: Query<(Entity, &NameOfWorkcell), Changed<NameOfWorkcell>>,
    ros_context: Res<RosContext>,
) {
    for (e, component) in &workcells {
        let base_service_name: &str = "/calibrate_extrinsics";
        let service_name = component.0.clone() + base_service_name;
        let client = ros_context
            .node
            .create_client::<CalibrateExtrinsics>(service_name.as_str())
            .unwrap_or_else(|err| {
                panic!("Unable to create service client {err}");
            });
        // This will overwrite any previous value(s) of the same component type.
        commands.entity(e).insert(CalibrationClient { client });
    }
}

// A system to make the calibration service call and update anchors
fn handle_keyboard_input(
    keyboard_input: Res<Input<KeyCode>>,
    calib_channels: ResMut<CalibrationChannels>,
    mut egui_context: EguiContexts,
    workcells: Query<(Entity, &CalibrationClient, &NameOfWorkcell)>,
) {
    let egui_context = egui_context.ctx_mut();
    let ui_has_focus = egui_context.wants_pointer_input()
        || egui_context.wants_keyboard_input()
        || egui_context.is_pointer_over_area();

    if ui_has_focus {
        return;
    }

    if !keyboard_input.just_pressed(KeyCode::C) {
        return;
    }
    info!("Calibrating workcells...");
    for (_e, calibration, workcell) in workcells.iter() {
        // Make service call here and put frame names into a hash map.
        // As per workcell coordinate frame conventions, the workcell
        // datum link is named as <workcell_name>_workcell_link.
        let base_workcell_root_name: &str = "_workcell_link";
        let request = CalibrateExtrinsics_Request {
            frame_id: workcell.0.clone() + base_workcell_root_name,
        };
        dbg!(&request);

        let sender = calib_channels.sender.clone();
        calibration
            .client
            .async_send_request_with_callback(
                request,
                move |response: CalibrateExtrinsics_Response| {
                    if !response.success {
                        error!("Unsuccessful calibration results");
                        return;
                    }
                    info!("Successfully retrieved calibration results!");
                    let mut transforms = TransformData::new();
                    for r in response.results {
                        transforms.insert(
                            r.child_frame_id,
                            Transform {
                                translation: Vec3::new(
                                    r.transform.translation.x as f32,
                                    r.transform.translation.y as f32,
                                    r.transform.translation.z as f32,
                                ),
                                rotation: Quat::from_xyzw(
                                    r.transform.rotation.x as f32,
                                    r.transform.rotation.y as f32,
                                    r.transform.rotation.z as f32,
                                    r.transform.rotation.w as f32,
                                ),
                                scale: Vec3::ONE,
                            },
                        );
                    }
                    sender
                        .send(transforms)
                        .expect("Failed sending calibration transforms");
                },
            )
            .unwrap_or_else(|_err| {
                panic!("Unable to get calibration results");
            });
    }
}

fn update_anchor_poses(
    calib_channels: ResMut<CalibrationChannels>,
    mut anchors: Query<(&NameInWorkcell, &mut Anchor)>,
) {
    if let Ok(result) = calib_channels.receiver.try_recv() {
        for (name, mut anchor) in &mut anchors {
            match result.get(&name.0) {
                Some(t) => {
                    anchor.move_to(t);
                    info!(
                        "Moving anchor {} to [pos: {}, rot:{}]",
                        &name.0, &t.translation, &t.rotation
                    );
                }
                None => {
                    warn!("[warn] No calibration data received for link {}", &name.0);
                    continue;
                }
            }
        }
    }
}

pub struct WorkcellCalibrationPlugin;

impl Plugin for WorkcellCalibrationPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<CalibrationChannels>().add_systems(
            Update,
            (
                add_calibration_client,
                handle_keyboard_input,
                update_anchor_poses,
            ),
        );
    }
}
