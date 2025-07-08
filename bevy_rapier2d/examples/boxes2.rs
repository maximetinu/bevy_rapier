use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

// Constants for random cube spawning
const NUM_RANDOM_CUBES: usize = 3000;
const SPAWN_RADIUS: f32 = 10000.0;

// The float value is the player movement speed in 'pixels/second'.
#[derive(Component)]
pub struct Player(f32);

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, (setup_graphics, setup_physics))
        .add_systems(Update, player_movement)
        .run();
}

pub fn setup_graphics(mut commands: Commands) {
    commands.spawn((Camera2d, Transform::from_xyz(0.0, 20.0, 0.0)));
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
                Collider::cuboid(rad, rad),
            ));
        }

        offset -= 0.05 * rad * (num as f32 - 1.0);
    }

    /*
     * Create random fixed cubes
     */
    use oorandom::Rand32;
    let mut rng = Rand32::new(42); // Using a seed for reproducibility

    for _ in 0..NUM_RANDOM_CUBES {
        // Generate random angle and distance for polar coordinates
        let angle = rng.rand_float() * std::f32::consts::TAU;
        let distance = rng.rand_float() * SPAWN_RADIUS;

        // Convert to cartesian coordinates
        let x = distance * angle.cos();
        let y = distance * angle.sin();

        commands.spawn((
            Transform::from_xyz(x, y, 0.0),
            RigidBody::Fixed,
            Collider::cuboid(rad, rad),
        ));
    }

    /*
     * Create player ball
     */
    let player_size = 20.0;
    commands.spawn((
        Transform::from_xyz(0.0, 0.0, 0.0),
        RigidBody::Dynamic,
        Velocity::zero(),
        Collider::ball(player_size),
        Player(300.0), // Player movement speed
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
