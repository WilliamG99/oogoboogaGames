#define GRAVITY 9.81
#define BOUNCE_DAMPING 0.8

#define SMOOTING_RADIUS 1.0
#define PARTICLE_MASS 1.0

typedef enum EntityArchetype {
	arch_nil = 0,
	arch_particle = 1,
	arch_boundary = 2,
} EntityArchetype;

typedef struct Entity {
	bool is_valid;
	EntityArchetype arch;
	Vector2 position;
	Vector2 velocity;
	Vector2 acceleration;
	float mass;
	float density;
	float pressure;

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

    for (int i = 0; i < 10	; i++) {
        Entity* en = entity_create();
        setup_particle(en);
        en->position = v2(get_random_float32_in_range(-20.0, 20.0), get_random_float32_in_range(-50.0, 50.0));
		en->velocity = v2(0.0, 0.0);
    }

	Entity* boundary0_en = entity_create();
	setup_boundary(boundary0_en);
	boundary0_en->boundaryPoint0 = v2(-50, -50);
	boundary0_en->boundaryPoint1 = v2( 50, -50);
	boundary0_en->boundaryPoint2 = v2( 50,  50);
	boundary0_en->boundaryPoint3 = v2(-50,  50);

	float64 seconds_counter = 0.0;
	s32 frame_count = 0;

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
						en->velocity = v2_add(en->velocity, v2_mulf(v2(0.0, -GRAVITY), delta_t));
						// Position
						en->position = v2_add(en->position, v2_mulf(en->velocity, delta_t));

						// Simple Collision w/ BoundaryFloor
						if (en->position.y < -50.0) {
							en->velocity.y = -(en->velocity.y * BOUNCE_DAMPING);
						}

						// Particle Transform
						Vector2 size	= v2(1.0,1.0);
						Matrix4 xform	= m4_scalar(1.0);
						xform			= m4_translate(xform, v3(en->position.x, en->position.y, 0));
						xform			= m4_translate(xform, v3(size.x * -0.5, size.y * -0.5, 0));
						draw_circle_xform(xform, size, COLOR_WHITE);
						
						// Smoothing Kernel
						for (int i = 0; i < MAX_ENTITY_COUNT; i++) {
							Entity* en_neighbour = &world->entities[i];
							if (en_neighbour->is_valid) {

								switch (en_neighbour->arch) {

									case arch_particle:
										float32 dist = v2_length(v2_sub(en->position, en_neighbour->position));
										break;
									
									default:
										break;
								}
							}
						}

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
			log("fps: %i", frame_count);
			log("Position: (%f, %f)\n", particle_en->position.x, particle_en->position.y);
			log("Velocity: (%f, %f)\n", particle_en->velocity.x, particle_en->velocity.y);
			seconds_counter = 0.0;
			frame_count = 0;
		}
	}

	return 0;
}