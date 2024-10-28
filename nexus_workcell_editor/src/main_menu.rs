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

use bevy::{app::AppExit, prelude::*, tasks::Task, window::PrimaryWindow};
use bevy_egui::{egui, EguiContexts};
use librmf_site_editor::{
    workspace::{WorkspaceData, WorkspaceLoader},
    AppState,
};
use rmf_site_format;
use std::path::PathBuf;

#[derive(Resource)]
pub struct Autoload {
    pub filename: Option<PathBuf>,
    pub import: Option<PathBuf>,
    pub importing: Option<Task<Option<(Entity, rmf_site_format::Site)>>>,
}

impl Autoload {
    pub fn file(filename: PathBuf, import: Option<PathBuf>) -> Self {
        Autoload {
            filename: Some(filename),
            import,
            importing: None,
        }
    }
}

pub fn demo_workcell() -> Vec<u8> {
    return include_str!("../test/test.workcell.json")
        .as_bytes()
        .to_vec();
}

fn egui_ui(
    mut egui_context: EguiContexts,
    mut _exit: EventWriter<AppExit>,
    mut workspace_loader: WorkspaceLoader,
    mut _app_state: ResMut<State<AppState>>,
    autoload: Option<ResMut<Autoload>>,
    primary_windows: Query<Entity, With<PrimaryWindow>>,
) {
    if let Some(mut autoload) = autoload {
        #[cfg(not(target_arch = "wasm32"))]
        {
            if let Some(filename) = autoload.filename.clone() {
                workspace_loader.load_from_path(filename);
            }
            autoload.filename = None;
        }
        return;
    }

    let Some(ctx) = primary_windows
        .get_single()
        .ok()
        .and_then(|w| egui_context.try_ctx_for_window_mut(w))
    else {
        return;
    };
    egui::Window::new("Welcome!")
        .collapsible(false)
        .resizable(false)
        .title_bar(false)
        .anchor(egui::Align2::CENTER_CENTER, egui::vec2(0., 0.))
        .show(ctx, |ui| {
            ui.heading("Welcome to The NEXUS Workcell Editor!");
            ui.add_space(10.);

            ui.horizontal(|ui| {
                if ui.button("New workcell").clicked() {
                    workspace_loader.load_from_data(WorkspaceData::Workcell(
                        rmf_site_format::Workcell::default()
                            .to_string()
                            .unwrap()
                            .into(),
                    ));
                }

                if ui.button("Load workcell").clicked() {
                    workspace_loader.load_from_dialog();
                }

                if ui.button("Demo workcell").clicked() {
                    workspace_loader.load_from_data(WorkspaceData::Workcell(demo_workcell()));
                }
            });

            #[cfg(not(target_arch = "wasm32"))]
            {
                ui.add_space(20.);
                ui.horizontal(|ui| {
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.button("Exit").clicked() {
                            _exit.send(AppExit);
                        }
                    });
                });
            }
        });
}

pub struct MainMenuPlugin;

impl Plugin for MainMenuPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, egui_ui.run_if(in_state(AppState::MainMenu)));
    }
}
