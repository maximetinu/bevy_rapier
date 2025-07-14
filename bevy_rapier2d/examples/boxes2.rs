use std::num::NonZeroUsize;

use bevy::{
    input::mouse::{MouseMotion, MouseWheel},
    prelude::*,
};
use bevy_rapier2d::prelude::*;

use rapier2d::prelude::IntegrationParameters;

#[cfg(target_arch = "wasm32")]
use web_sys::{window, Url, UrlSearchParams};

#[cfg(target_arch = "wasm32")]
#[macro_export]
macro_rules! mark {
    ($($arg:tt)*) => {
        mark(&format!($($arg)*))
    };
}

#[cfg(target_arch = "wasm32")]
#[macro_export]
macro_rules! mark_start {
    ($($arg:tt)*) => {
        mark_start(&format!($($arg)*))
    };
}

#[cfg(target_arch = "wasm32")]
#[macro_export]
macro_rules! mark_end {
    ($($arg:tt)*) => {
        mark_end(&format!($($arg)*))
    };
}

#[cfg(target_arch = "wasm32")]
#[macro_export]
macro_rules! measure {
    ($($arg:tt)*) => {
        measure(&format!($($arg)*))
    };
}

#[cfg(target_arch = "wasm32")]
fn mark(name: &str) {
    let Some(window) = window() else { return };
    let Some(performance) = window.performance() else {
        return;
    };
    let _ = performance.mark(name);
}

#[cfg(target_arch = "wasm32")]
fn mark_start(name: &str) {
    mark(&format!("{name} start"));
}

#[cfg(target_arch = "wasm32")]
fn mark_end(name: &str) {
    mark(&format!("{name} end"));
}

#[cfg(target_arch = "wasm32")]
fn measure(name: &str) {
    let Some(window) = window() else { return };
    let Some(performance) = window.performance() else {
        return;
    };
    let start_mark = format!("{name} start");
    let end_mark = format!("{name} end");
    let _ = performance.measure_with_start_mark_and_end_mark(name, &start_mark, &end_mark);
}

#[cfg(not(target_arch = "wasm32"))]
fn mark(_name: &str) {}

#[cfg(not(target_arch = "wasm32"))]
fn mark_start(_name: &str) {}

#[cfg(not(target_arch = "wasm32"))]
fn mark_end(_name: &str) {}

#[cfg(not(target_arch = "wasm32"))]
fn measure(_name: &str) {}

// worse for perf ?
pub const PLAYER: Group = Group::GROUP_1;
pub const DYNAMIC_CUBES: Group = Group::GROUP_2;
pub const FIXED_CUBES: Group = Group::GROUP_3;
pub const GROUND: Group = Group::GROUP_4;

// better for perf ?
// pub const PLAYER: Group = Group::NONE;
// pub const DYNAMIC_CUBES: Group = Group::NONE;
// pub const FIXED_CUBES: Group = Group::NONE;
// pub const GROUND: Group = Group::NONE;

// Default constants for random cube spawning
const DEFAULT_NUM_RANDOM_CUBES: usize = 5000;
const DEFAULT_SPAWN_RADIUS: f32 = 40000.0;

// bad performance:
// const NUM_RANDOM_CUBES: usize = 12000;
// const SPAWN_RADIUS: f32 = 50000.0;

// The float value is the player movement speed in 'pixels/second'.
#[derive(Component)]
pub struct Player(f32);

// Component to track camera state
#[derive(Component)]
pub struct CameraController {
    pub zoom_speed: f32,
    pub pan_speed: f32,
    pub min_zoom: f32,
    pub max_zoom: f32,
}

#[cfg(target_arch = "wasm32")]
fn get_url_params() -> (usize, f32) {
    let Some(window) = window() else {
        return (DEFAULT_NUM_RANDOM_CUBES, DEFAULT_SPAWN_RADIUS);
    };

    let location = window.location();
    let url_string = match location.href() {
        Ok(s) => s,
        Err(_) => String::new(),
    };

    let Ok(url) = Url::new(&url_string) else {
        return (DEFAULT_NUM_RANDOM_CUBES, DEFAULT_SPAWN_RADIUS);
    };

    let search_params = UrlSearchParams::new_with_str(&url.search())
        .unwrap_or_else(|_| UrlSearchParams::new().unwrap());

    let num_cubes = search_params
        .get("cubes")
        .and_then(|s| s.parse::<usize>().ok())
        .unwrap_or(DEFAULT_NUM_RANDOM_CUBES);

    let spawn_radius = search_params
        .get("radius")
        .and_then(|s| s.parse::<f32>().ok())
        .unwrap_or(DEFAULT_SPAWN_RADIUS);

    (num_cubes, spawn_radius)
}

#[cfg(not(target_arch = "wasm32"))]
fn get_url_params() -> (usize, f32) {
    (DEFAULT_NUM_RANDOM_CUBES, DEFAULT_SPAWN_RADIUS)
}

fn main() {
    let rapier = RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0);

    let rapier_schedule = rapier.schedule;

    App::new()
        .insert_resource(ClearColor(Color::srgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        // .insert_resource(WinitSettings {
        //     focused_mode: bevy::winit::UpdateMode::reactive_low_power(Duration::from_secs_f32(
        //         1.0 / 30.0, // 30 FPS
        //     )),
        //     unfocused_mode: bevy::winit::UpdateMode::reactive_low_power(Duration::from_secs(1)),
        // })
        .add_plugins((DefaultPlugins, rapier, RapierDebugRenderPlugin::default()))
        .add_systems(Startup, (setup_graphics, setup_physics, setup_phys_config))
        .add_systems(
            Update,
            (
                player_movement, //
                camera_controls,
                distance_based_activation,
            ),
        )
        .add_systems(
            rapier_schedule,
            (
                start_measure.before(PhysicsSet::StepSimulation),
                end_measure.after(PhysicsSet::StepSimulation),
            ),
        )
        .run();
}

pub fn start_measure() {
    #[cfg(target_arch = "wasm32")]
    mark_start!("physics step!");
}

pub fn end_measure() {
    #[cfg(target_arch = "wasm32")]
    {
        mark_end!("physics step!");
        measure!("physics step!");
    }
}

pub fn setup_phys_config(mut ctx: Query<&mut RapierContextSimulation>) {
    let mut ctx = ctx.single_mut().unwrap();
    ctx.integration_parameters = IntegrationParameters {
        dt: 1.0 / 60.0,
        min_ccd_dt: 1.0 / 60.0 / 10.0,
        contact_damping_ratio: 10.0, // Higher value = more compliant, less accurate but faster
        contact_natural_frequency: 5.0, // Lower value = slower penetration correction, less jitter
        joint_natural_frequency: 1.0e4, // Lower than default for performance
        joint_damping_ratio: 2.0,    // Higher value = more compliant joints
        warmstart_coefficient: 0.1,  // Reduced from 1.0 for less warmstarting overhead
        length_unit: 100.0,          // Keep default
        normalized_allowed_linear_error: 0.01, // Higher tolerance = less correction work
        normalized_max_corrective_velocity: 5.0, // Lower than default = less aggressive correction
        normalized_prediction_distance: 0.001, // Lower value = fewer predictive contacts
        num_solver_iterations: NonZeroUsize::new(1).unwrap(), // Reduced from 4 for performance
        num_additional_friction_iterations: 0, // Keep at 0 for performance
        num_internal_pgs_iterations: 1, // Keep at 1 for performance
        num_internal_stabilization_iterations: 1, // Reduced from 2 for performance
        min_island_size: 256,        // Increased for better SIMD parallelism
        max_ccd_substeps: 1,         // Keep at 1 for performance
    };
}

pub fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera2d,
        Transform::from_xyz(0.0, 20.0, 0.0),
        CameraController {
            zoom_speed: 0.1,
            pan_speed: 1.0,
            min_zoom: 0.01,
            max_zoom: 10000.0,
        },
    ));
}

pub fn setup_physics(mut commands: Commands, mut rapier_config: Query<&mut RapierConfiguration>) {
    // Set gravity to 0.0 for free movement
    if let Ok(mut config) = rapier_config.single_mut() {
        config.gravity = Vec2::ZERO;
    }

    /*
     * Ground
     */
    let ground_size = 500.0;
    let ground_height = 10.0;

    commands.spawn((
        Transform::from_xyz(0.0, 0.0 * -ground_height, 0.0),
        Collider::cuboid(ground_size, ground_height),
        CollisionGroups::new(GROUND, GROUND | DYNAMIC_CUBES | FIXED_CUBES | PLAYER),
    ));

    /*
     * Create the cubes
     */
    let num = 8;
    let rad = 10.0;

    let shift = rad * 2.0 + rad;
    let centerx = shift * (num / 2) as f32;
    let centery = shift / 2.0;

    let mut offset = -(num as f32) * (rad * 2.0 + rad) * 0.5;

    for j in 0usize..20 {
        for i in 0..num {
            let x = i as f32 * shift - centerx + offset;
            let y = j as f32 * shift + centery + 30.0;

            commands.spawn((
                Transform::from_xyz(x, y, 0.0),
                RigidBody::Dynamic,
                Collider::cuboid(rad * 0.5, rad * 0.5),
                CollisionGroups::new(DYNAMIC_CUBES, DYNAMIC_CUBES | FIXED_CUBES | GROUND | PLAYER),
            ));
        }

        offset -= 0.05 * rad * (num as f32 - 1.0);
    }

    /*
     * Create random fixed cubes
     */
    use oorandom::Rand32;
    let mut rng = Rand32::new(41); // Using a seed for reproducibility

    // Get parameters from URL or use defaults
    let (num_random_cubes, spawn_radius) = get_url_params();

    const SAFE_RADIUS: f32 = 1000.0;

    for _ in 0..num_random_cubes {
        // Generate random angle and distance for polar coordinates
        let angle = rng.rand_float() * std::f32::consts::TAU;
        let distance = rng.rand_float() * spawn_radius;

        if distance < SAFE_RADIUS {
            continue;
        }

        // Convert to cartesian coordinates
        let x = distance * angle.cos();
        let y = distance * angle.sin();

        // worse perf ?
        // Generate random sizes for x and y dimensions independently
        // let size_x = rad * rng.rand_float();
        // let size_y = rad * rng.rand_float();

        // even worse
        let size_x = rad * rad * rad * rng.rand_float() * 0.2;
        let size_y = rad * rad * rad * rng.rand_float() * 0.2;

        // better perf ?
        // let size_x = rad;
        // let size_y = rad;

        commands.spawn((
            Transform::from_xyz(x, y, 0.0),
            RigidBody::Fixed,
            Collider::cuboid(size_x, size_y),
            CollisionGroups::new(FIXED_CUBES, FIXED_CUBES | DYNAMIC_CUBES | GROUND | PLAYER),
        ));
    }

    /*
     * Create player ball
     */
    let player_size = 5.0;
    commands.spawn((
        Transform::from_xyz(50.0, 200.0, 0.0),
        RigidBody::Dynamic,
        Velocity::zero(),
        Collider::ball(player_size),
        Player(300.0), // Player movement speed
        CollisionGroups::new(PLAYER, PLAYER | DYNAMIC_CUBES | FIXED_CUBES | GROUND),
        Ccd::enabled(),
    ));
}

pub fn player_movement(
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut player_info: Query<(&Player, &mut Velocity)>,
) {
    for (player, mut rb_vels) in &mut player_info {
        let up = keyboard_input.any_pressed([KeyCode::KeyW, KeyCode::ArrowUp]);
        let down = keyboard_input.any_pressed([KeyCode::KeyS, KeyCode::ArrowDown]);
        let left = keyboard_input.any_pressed([KeyCode::KeyA, KeyCode::ArrowLeft]);
        let right = keyboard_input.any_pressed([KeyCode::KeyD, KeyCode::ArrowRight]);

        let x_axis = -(left as i8) + right as i8;
        let y_axis = -(down as i8) + up as i8;

        let mut move_delta = Vec2::new(x_axis as f32, y_axis as f32);
        if move_delta != Vec2::ZERO {
            move_delta /= move_delta.length();
        }

        // Update the velocity on the rigid_body_component,
        // the bevy_rapier plugin will update the transform.
        rb_vels.linvel = move_delta * player.0;
    }
}

pub fn camera_controls(
    mut camera_query: Query<(&mut Transform, &mut Projection, &CameraController)>,
    mut mouse_wheel_events: EventReader<MouseWheel>,
    mut mouse_motion_events: EventReader<MouseMotion>,
    mouse_button_input: Res<ButtonInput<MouseButton>>,
) {
    let Ok((mut transform, mut projection, camera_controller)) = camera_query.single_mut() else {
        return;
    };

    let orthographic_projection = match &mut *projection {
        Projection::Orthographic(orthographic_projection) => orthographic_projection,
        _ => return,
    };

    // Handle mouse wheel zoom
    for event in mouse_wheel_events.read() {
        let zoom_delta = event.y * camera_controller.zoom_speed;
        let new_scale = (orthographic_projection.scale - zoom_delta)
            .clamp(camera_controller.min_zoom, camera_controller.max_zoom);
        orthographic_projection.scale = new_scale;
    }

    // Handle mouse panning (hold left, middle or right mouse button)
    if mouse_button_input.any_pressed([MouseButton::Left, MouseButton::Middle, MouseButton::Right])
    {
        for event in mouse_motion_events.read() {
            // Make panning speed inversely proportional to zoom scale
            let adjusted_pan_speed = camera_controller.pan_speed * orthographic_projection.scale;
            let pan_delta = Vec2::new(-event.delta.x, event.delta.y) * adjusted_pan_speed;
            transform.translation += Vec3::new(pan_delta.x, pan_delta.y, 0.0);
        }
    }
}

/// System to disable rigid bodies and colliders that are far from the player and cursor
pub fn distance_based_activation(
    player_query: Query<&GlobalTransform, (With<Player>, Without<RigidBodyDisabled>)>,
    mut commands: Commands,
    // Query for entities that have both RigidBody and Collider components
    mut physics_entities: Query<
        (Entity, &GlobalTransform),
        (With<RigidBody>, With<Collider>, Without<Player>),
    >,
    // Query for entities that have only Collider (like the ground)
    mut standalone_colliders: Query<
        (Entity, &GlobalTransform),
        (With<Collider>, Without<RigidBody>, Without<Player>),
    >,
    // Camera and cursor info
    camera_query: Query<(&Camera, &GlobalTransform, &Projection)>,
    window_query: Query<&Window>,
) {
    #[cfg(target_arch = "wasm32")]
    mark_start!("distance_based_activation");
    const ACTIVATION_RADIUS: f32 = 200.0;

    // Get player position
    let Ok(player_transform) = player_query.single() else {
        return;
    };
    let player_pos = player_transform.translation().truncate();

    // Get cursor position in world space
    let cursor_world_pos = get_cursor_world_position(&camera_query, &window_query);

    // Check entities with both rigid bodies and colliders
    for (entity, transform) in physics_entities.iter_mut() {
        let entity_pos = transform.translation().truncate();
        let distance_to_player = player_pos.distance(entity_pos);

        // Check if close to either player or cursor
        let should_be_active = if let Some(cursor_pos) = cursor_world_pos {
            let distance_to_cursor = cursor_pos.distance(entity_pos);
            distance_to_player <= ACTIVATION_RADIUS || distance_to_cursor <= ACTIVATION_RADIUS
        } else {
            // If cursor is off screen, only check player distance
            distance_to_player <= ACTIVATION_RADIUS
        };

        if should_be_active {
            // Enable both if close enough to either player or cursor
            commands.entity(entity).remove::<RigidBodyDisabled>();
            commands.entity(entity).remove::<ColliderDisabled>();
        } else {
            // Disable both if too far from both
            commands.entity(entity).insert(RigidBodyDisabled);
            commands.entity(entity).insert(ColliderDisabled);
        }
    }

    // Check standalone colliders (like the ground)
    for (entity, transform) in standalone_colliders.iter_mut() {
        let entity_pos = transform.translation().truncate();
        let distance_to_player = player_pos.distance(entity_pos);

        // Check if close to either player or cursor
        let should_be_active = if let Some(cursor_pos) = cursor_world_pos {
            let distance_to_cursor = cursor_pos.distance(entity_pos);
            distance_to_player <= ACTIVATION_RADIUS || distance_to_cursor <= ACTIVATION_RADIUS
        } else {
            // If cursor is off screen, only check player distance
            distance_to_player <= ACTIVATION_RADIUS
        };

        if should_be_active {
            // Enable if close enough to either player or cursor
            commands.entity(entity).remove::<ColliderDisabled>();
        } else {
            // Disable if too far from both
            commands.entity(entity).insert(ColliderDisabled);
        }
    }

    #[cfg(target_arch = "wasm32")]
    {
        mark_end!("distance_based_activation");
        measure!("distance_based_activation");
    }
}

/// Helper function to get cursor position in world space
fn get_cursor_world_position(
    camera_query: &Query<(&Camera, &GlobalTransform, &Projection)>,
    window_query: &Query<&Window>,
) -> Option<Vec2> {
    let Ok((_camera, camera_transform, projection)) = camera_query.single() else {
        return None;
    };

    let Ok(window) = window_query.single() else {
        return None;
    };

    // Get cursor position in screen space
    let cursor_pos = window.cursor_position()?;

    // Convert screen position to world position
    let screen_size = Vec2::new(window.width(), window.height());
    let screen_pos = Vec2::new(cursor_pos.x, cursor_pos.y);

    // Convert to normalized device coordinates (-1 to 1)
    let ndc = (screen_pos / screen_size) * 2.0 - Vec2::ONE;
    let ndc = Vec3::new(ndc.x, -ndc.y, 0.0); // Flip Y and set Z to 0 for 2D

    // Convert to world space
    let world_pos = match projection {
        Projection::Orthographic(ortho) => {
            let view_matrix = camera_transform.compute_matrix();
            let proj_matrix = Mat4::orthographic_rh(
                -ortho.area.width() / 2.0,
                ortho.area.width() / 2.0,
                -ortho.area.height() / 2.0,
                ortho.area.height() / 2.0,
                ortho.near,
                ortho.far,
            );

            let world_matrix = view_matrix.inverse() * proj_matrix.inverse();
            world_matrix.project_point3(ndc).truncate()
        }
        Projection::Perspective(_) => {
            // For perspective cameras, we'd need more complex math
            // For now, just return None to avoid complexity
            return None;
        }
        Projection::Custom(_) => {
            // For custom projections, we'd need to handle them specifically
            // For now, just return None to avoid complexity
            return None;
        }
    };

    Some(world_pos)
}
