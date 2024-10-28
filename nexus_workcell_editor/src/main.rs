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

use bevy::render::{
    render_resource::{AddressMode, SamplerDescriptor},
    settings::{WgpuFeatures, WgpuSettings},
    RenderPlugin,
};
use bevy::{log::LogPlugin, pbr::DirectionalLightShadowMap, prelude::*};
use bevy_egui::EguiPlugin;

use clap::Parser;

use librmf_site_editor::{
    aabb::AabbUpdatePlugin, animate::AnimationPlugin, asset_loaders::AssetLoadersPlugin,
    interaction::InteractionPlugin, issue::IssuePlugin, keyboard::*, log::LogHistoryPlugin,
    occupancy::OccupancyPlugin, site::SitePlugin, site_asset_io::SiteAssetIoPlugin,
    view_menu::ViewMenuPlugin, widgets::*, wireframe::SiteWireframePlugin,
    workcell::WorkcellEditorPlugin, workspace::*, AppState, CommandLineArgs,
};

pub mod main_menu;
use main_menu::{Autoload, MainMenuPlugin};

pub mod ros_context;
use ros_context::RosContextPlugin;

pub mod workcell_calibration;
use workcell_calibration::WorkcellCalibrationPlugin;

fn main() {
    info!("Starting nexus_workcell_editor");

    let mut app = App::new();

    #[cfg(not(target_arch = "wasm32"))]
    {
        let command_line_args: Vec<String> = std::env::args().collect();
        let command_line_args = CommandLineArgs::parse_from(command_line_args);
        if let Some(path) = command_line_args.filename {
            app.insert_resource(Autoload::file(
                path.into(),
                command_line_args.import.map(Into::into),
            ));
        }
    }

    let mut plugins = DefaultPlugins.build();
    plugins = plugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: "RMF Site Editor".to_owned(),
            #[cfg(not(target_arch = "wasm32"))]
            resolution: (1600., 900.).into(),
            #[cfg(target_arch = "wasm32")]
            canvas: Some(String::from("#rmf_site_editor_canvas")),
            #[cfg(target_arch = "wasm32")]
            fit_canvas_to_parent: true,
            ..default()
        }),
        ..default()
    });
    app.add_plugins((
        SiteAssetIoPlugin,
        plugins
            .disable::<LogPlugin>()
            .set(ImagePlugin {
                default_sampler: SamplerDescriptor {
                    address_mode_u: AddressMode::Repeat,
                    address_mode_v: AddressMode::Repeat,
                    address_mode_w: AddressMode::Repeat,
                    ..Default::default()
                }
                .into(),
            })
            .set(RenderPlugin {
                render_creation: WgpuSettings {
                    #[cfg(not(target_arch = "wasm32"))]
                    features: WgpuFeatures::POLYGON_MODE_LINE,
                    ..default()
                }
                .into(),
                ..default()
            }),
    ));

    app.insert_resource(DirectionalLightShadowMap { size: 2048 })
        .add_state::<AppState>()
        .add_plugins((
            AssetLoadersPlugin,
            LogHistoryPlugin,
            AabbUpdatePlugin,
            EguiPlugin,
            KeyboardInputPlugin,
            MainMenuPlugin,
            WorkcellEditorPlugin,
            SitePlugin,
            InteractionPlugin::default(),
            StandardUiPlugin::default(),
            AnimationPlugin,
            OccupancyPlugin,
            WorkspacePlugin,
            SiteWireframePlugin,
        ))
        .add_plugins((
            IssuePlugin,
            ViewMenuPlugin,
            RosContextPlugin,
            WorkcellCalibrationPlugin,
            bevy_impulse::ImpulsePlugin::default(),
        ))
        .run();
}
