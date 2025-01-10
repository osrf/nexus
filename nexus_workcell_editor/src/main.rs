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

use bevy::{
    log::LogPlugin,
    pbr::DirectionalLightShadowMap,
    prelude::*,
    render::{
        render_resource::{AddressMode, SamplerDescriptor},
        settings::{WgpuFeatures, WgpuSettings},
        RenderPlugin,
    },
};

use clap::Parser;

use librmf_workcell_editor::{
    bevy_egui::EguiPlugin, bevy_impulse, interaction::InteractionPlugin, keyboard::*,
    site_asset_io::SiteAssetIoPlugin, view_menu::ViewMenuPlugin, widgets::*,
    workcell::WorkcellEditorPlugin, workspace::*, AabbUpdatePlugin, AnimationPlugin, AppState,
    AssetLoadersPlugin, CommandLineArgs, DeletionPlugin, FuelPlugin, LogHistoryPlugin,
    ModelLoadingPlugin, SiteAssets, SiteWireframePlugin,
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
        .init_resource::<SiteAssets>()
        .add_state::<AppState>()
        .add_plugins((
            AssetLoadersPlugin,
            LogHistoryPlugin,
            DeletionPlugin,
            FuelPlugin::default(),
            AabbUpdatePlugin,
            EguiPlugin,
            KeyboardInputPlugin,
            MainMenuPlugin,
            WorkcellEditorPlugin,
            ModelLoadingPlugin::default(),
            InteractionPlugin::default(),
            StandardUiPlugin::default(),
            AnimationPlugin,
            WorkspacePlugin,
            SiteWireframePlugin,
        ))
        .add_plugins((
            ViewMenuPlugin,
            RosContextPlugin,
            WorkcellCalibrationPlugin,
            bevy_impulse::ImpulsePlugin::default(),
        ))
        .run();
}
