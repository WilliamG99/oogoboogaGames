#define GRAVITY 9.81
#define BOUNCE_DAMPING 0.9

#define SMOOTHING_RADIUS 10.0
#define PARTICLE_MASS 7.0

#define REST_DENSITY 0.5
#define PRESSURE_CONSTANT 10.0

typedef enum EntityArchetype {
	arch_nil = 0,
	arch_particle = 1,
	arch_boundary = 2,
} EntityArchetype;

typedef struct Entity {
	bool is_valid;
	EntityArchetype arch;
	
	float mass;
	Vector2 position;
	Vector2 velocity;
	Vector2 acceleration;
	float influence;
	float density;
	float pressure;
	Vector2 pressure_force;
	Vector2 pressure_acceleration;

	Vector2 boundaryPoint0;
	Vector2 boundaryPoint1;
	Vector2 boundaryPoint2;
	Vector2 boundaryPoint3;
} Entity;
#define MAX_ENTITY_COUNT 1024

typedef struct World {
	Entity entities[MAX_ENTITY_COUNT];
} World;
World* world = 0;

Entity* entity_create() {
	Entity* entity_found = 0;
	for (int i = 0; i < MAX_ENTITY_COUNT; i++) {
		Entity* existing_entity = &world->entities[i];
		if (!existing_entity->is_valid) {
			entity_found = existing_entity;
			break;
		}
	}
	assert(entity_found, "No more free entities!");
	entity_found->is_valid = true;
	return entity_found;
}

void entity_destroy(Entity* entity) {
	memset(entity, 0, sizeof(Entity));
}

void setup_particle(Entity* en) {
	en->arch = arch_particle;
	en->mass = PARTICLE_MASS;
	en->position = v2(0.0, 0.0);
	en->velocity = v2(0.0, 0.0);
}

void setup_boundary(Entity* en) {
	en->arch = arch_boundary;
}

int entry(int argc, char **argv) {
	window.title = STR("FluidSim");
	window.width = 1280;
	window.height = 720;
	window.x = 200;
	window.y = 200;
	window.clear_color = hex_to_rgba(0x2a2d3aff);

	world = alloc(get_heap_allocator(), sizeof(World));
	memset(world, 0, sizeof(World));

	Entity* particle_en = entity_create();
	setup_particle(particle_en);

    for (int i = 0; i < 100	; i++) {
        Entity* en = entity_create();
        setup_particle(en);
        en->position = v2(get_random_float32_in_range(-10.0, 10.0), get_random_float32_in_range(-10.0, 10.0));
    }

	Entity* boundary0_en = entity_create();
	setup_boundary(boundary0_en);
	boundary0_en->boundaryPoint0 = v2(-50, -50);
	boundary0_en->boundaryPoint1 = v2( 50, -50);
	boundary0_en->boundaryPoint2 = v2( 50,  50);
	boundary0_en->boundaryPoint3 = v2(-50,  50);

	float64 seconds_counter = 0.0;
	s32 frame_count = 0;

	// Smoothing Kernels and their Gradients.
	const float32 kernel = (4.0f / (PI32 * powf(SMOOTHING_RADIUS, 8.0f)));
	const float32 kernel_derivative = -10.0f / (PI32 * powf(SMOOTHING_RADIUS, 5.0f));

	float64 last_time = os_get_current_time_in_seconds();
	while (!window.should_close) {
		reset_temporary_storage();

		draw_frame.projection = m4_make_orthographic_projection(window.width * -0.5, window.width * 0.5, window.height * -0.5, window.height * 0.5, -1, 10);
		float zoom = 5.3;
		draw_frame.view = m4_make_scale(v3(1.0/zoom, 1.0/zoom, 1.0));

		float64 now = os_get_current_time_in_seconds();
		float64 delta_t = now - last_time;
		last_time = now;

		os_update();

		// :render
		for (int i = 0; i < MAX_ENTITY_COUNT; i++) {
			Entity* en = &world->entities[i];
			if (en->is_valid) {

				switch (en->arch) {

					case arch_particle:
						// Particle Updates
						// Velocity
						//en->velocity = v2_add(en->velocity, v2_mulf(v2(0.0, -GRAVITY), delta_t));
						//en->velocity = v2_add(en->velocity, en->pressure_acceleration);
						en->velocity = v2_add(en->velocity, v2_mulf(en->pressure_acceleration, delta_t));
						// Position
						//en->position = v2_add(en->position, v2_mulf(en->velocity, delta_t));

						// // Simple Collision w/ BoundaryFloor
						// if (en->position.y < -50.0) {
						// 	en->velocity.y = -(en->velocity.y * BOUNCE_DAMPING);
						// }

						// Particle Transform
						Vector2 size	= v2(1.0,1.0);
						Matrix4 xform	= m4_scalar(1.0);
						xform			= m4_translate(xform, v3(en->position.x, en->position.y, 0));
						xform			= m4_translate(xform, v3(size.x * -0.5, size.y * -0.5, 0));
						draw_circle_xform(xform, size, COLOR_WHITE);

						// Reset Particle Properties
						//log("Density: %f", en->density);
						en->density = 0.0f;
						en->pressure = 0.0f;
						en->pressure_force = v2(0.0f, 0.0f);

						// Search Neighbour Particles to Calculate Density
						for (int j = 0; j < MAX_ENTITY_COUNT; j++) {
							Entity* en_neighbour = &world->entities[j];
							if (en_neighbour->is_valid) {
								
								switch (en_neighbour->arch) {

									case arch_particle:

										// Calculate Density
										float32 distance = v2_length(v2_sub(en_neighbour->position, en->position));
										float32 sqrd_distance = square(distance);
										
										if (sqrd_distance < square(SMOOTHING_RADIUS)) {
											en->density += PARTICLE_MASS * kernel * powf(square(SMOOTHING_RADIUS)-sqrd_distance, 3.0f);
										}

										// log("Pos1: %f, %f", en_neighbour->position.x, en_neighbour->position.y);
										// log("Pos2: %f, %f", en->position.x, en->position.y);
										// log("Sub: %f, %f", sub.x, sub.y);
										// log("Distance: %f", distance);

										break;

									default:
										break;
								}
							}
						}

						// Calculate Pressure
						en->pressure = PRESSURE_CONSTANT * (en->density - REST_DENSITY);

						// Search Neighbour Particles to Calculate Forces
						for (int j = 0; j < MAX_ENTITY_COUNT; j++) {
							Entity* en_neighbour = &world->entities[j];
							if (en_neighbour->is_valid) {
								
								switch (en_neighbour->arch) {

									case arch_particle:

										if(en == en_neighbour){break;}

										float32 distance = v2_length(v2_sub(en_neighbour->position, en->position));
										Vector2 rij_normal = v2_normalize(v2_sub(en_neighbour->position, en->position));

										if (distance < SMOOTHING_RADIUS){
											// Compute pressure force contribution
											en->pressure_force = v2_add(en->pressure_force, v2_mulf(rij_normal, PARTICLE_MASS * (en->pressure + en_neighbour->pressure) / (2.0f * en_neighbour->density) * kernel_derivative * powf(SMOOTHING_RADIUS - distance, 3.0f)));
										}

										break;

									default:
										break;
								}
							}
						}

						// Apply Forces
						en->pressure_acceleration = v2_divf(en->pressure_force, en->density);

						break;

					case arch_boundary:
						draw_line(en->boundaryPoint0, en->boundaryPoint1, 0.5, COLOR_RED);
						draw_line(en->boundaryPoint1, en->boundaryPoint2, 0.5, COLOR_RED);
						draw_line(en->boundaryPoint2, en->boundaryPoint3, 0.5, COLOR_RED);
						draw_line(en->boundaryPoint3, en->boundaryPoint0, 0.5, COLOR_RED);
						break;

					default:
					{
						// Matrix4 xform = m4_scalar(1.0);
						// xform         = m4_translate(xform, v3(en->position.x, en->position.y, 0));
						// draw_circle_xform(xform, v2(1, 1), COLOR_WHITE);
						break;
					}
				}
			}
		}

		// Window Commands/Keys
		if (is_key_just_pressed(KEY_ESCAPE)) {
			window.should_close = true;
		}

		gfx_update();
		seconds_counter += delta_t;
		frame_count += 1;
		if (seconds_counter > 1.0) {
			log("delta_t: %f", delta_t);
			log("fps: %i\n", frame_count);
			log("Position: (%f, %f)", particle_en->position.x, particle_en->position.y);
			log("Velocity: (%f, %f)", particle_en->velocity.x, particle_en->velocity.y);
			log("Density: %f", particle_en->density);
			log("Pressure: %f", particle_en->pressure);
			log("Pressure Force: %f, %f", particle_en->pressure_force.x, particle_en->pressure_force.y);
			log("Pressure Accel: %f, %f\n", particle_en->pressure_acceleration.x, particle_en->pressure_acceleration.y);
			seconds_counter = 0.0;
			frame_count = 0;
		}
	}

	return 0;
}