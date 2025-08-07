#define GRAVITY 0.0
#define BOUNCE_DAMPING 0.8

#define SMOOTHING_RADIUS 3.0
#define PARTICLE_MASS 1.0

#define REST_DENSITY 2.75
#define PRESSURE_CONSTANT 25.0

#define PARTICLE_NUM 50

typedef enum EntityArchetype {
	arch_nil = 0,
	arch_particle = 1,
	arch_boundary = 2,
} EntityArchetype;

typedef struct Entity {
	bool is_valid;
	EntityArchetype arch;
	
	int index;
	Vector2 position;
	Vector2i cell_position;
	int32 hash;
	int16 cell_key;
	Vector2 velocity;
	Vector2 acceleration;
	float32 density;
	float32 pressure;
	Vector2 pressure_force;
	Vector2 pressure_acceleration;

	Vector2 boundaryPoint0;
	Vector2 boundaryPoint1;
	Vector2 boundaryPoint2;
	Vector2 boundaryPoint3;
} Entity;
#define MAX_ENTITY_COUNT 4096

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
	en->density = 0.0f;
	en->pressure = 0.0f;
	en->pressure_force = v2(0.0f, 0.0f);
}

void setup_boundary(Entity* en) {
	en->arch = arch_boundary;
}

// Main functions and calculations
// Smooting Kernel
float32 SmoothingKernel(float distance) {
	if (distance >= SMOOTHING_RADIUS) {return 0.0f;}

	float32 volume = (PI32 * powf(SMOOTHING_RADIUS, 4)) / 6.f;
	return (SMOOTHING_RADIUS - distance) * (SMOOTHING_RADIUS - distance) / volume;
}
// Smoothing Kernel Derivative
float32 SmoothingKernelDerivative(float32 distance) {
	if (distance >= SMOOTHING_RADIUS) {return 0.0f;}

	float32 scale = 12.f / (powf(SMOOTHING_RADIUS, 4) * PI32);
	return (distance - SMOOTHING_RADIUS) * scale;
}
// Calculate Pressure
float32 ConvertDensityToPressure(float32 density) {
	float32 density_error = density - REST_DENSITY;
	float32 pressure = density_error * PRESSURE_CONSTANT;
	return pressure;
}
// Calculate Shared Pressure
float32 CalculateSharedPressure(float32 pressureA, float32 pressureB) {
	return (pressureA + pressureB) / 2;
}

// Cell Linked List Optimisation
// Constants for Hashing
const int hash1 = 15823;
const int hash2 = 9737333;
// Get 2D Cell Co-ordinates from particle position
Vector2i Get2DCell(Vector2 position) {
	int cellX = (int)floorf(position.x / SMOOTHING_RADIUS);
	int cellY = (int)floorf(position.y / SMOOTHING_RADIUS);
	return v2i(cellX, cellY);
}
// Hash Cell Co-ordinates into a single Int
int GetHashFrom2DCell(Vector2i cell_coord) {
	int a = cell_coord.x * hash1;
	int b = cell_coord.y * hash2;
	return a+b;
}
// Hash to Key
int GetKeyFromHash(int hash) {
    int table_size = PARTICLE_NUM * PARTICLE_NUM;
    // This formula handles negative hashes
    return (hash % table_size + table_size) % table_size;
}
// Struct for Lookup Array Data
struct SpatialLookupStruct {
	int16 index;
	int16 key;
};
// Comparison Function for Sorting based on Cell Key
int CompareByKey(const void* a, const void* b) {
    int16 key_a = ((struct SpatialLookupStruct*)a)->key;
    int16 key_b = ((struct SpatialLookupStruct*)b)->key;
    return (key_a < key_b) ? -1 : (key_a > key_b);
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
	
	// Initialise An Array of Structs
	struct SpatialLookupStruct SpatialLookup[PARTICLE_NUM*PARTICLE_NUM];

	// Setup Particles
	int k = 0;
    for (int i = 0; i < PARTICLE_NUM; i++) {
		for (int j = 0; j < PARTICLE_NUM; j++) {
			Entity* en = entity_create();
			setup_particle(en);
			en->position = v2(1.5*i-3, 1.5*j-3 );
			
			en->index = k;
			SpatialLookup[k].index = k;
			k++;
		}
    }

	// Initialise Array for Inidicie Start
	int StartIndices[PARTICLE_NUM*PARTICLE_NUM];

	Entity* boundary0_en = entity_create();
	setup_boundary(boundary0_en);
	boundary0_en->boundaryPoint0 = v2(-120, -70);
	boundary0_en->boundaryPoint1 = v2( 120, -70);
	boundary0_en->boundaryPoint2 = v2( 120,  70);
	boundary0_en->boundaryPoint3 = v2(-120,  70);

	float64 seconds_counter = 0.0;
	s32 frame_count = 0;

	Vector2i grid_offsets[9] = {
		{-1, -1}, {0, -1}, {1, -1},
		{-1,  0}, {0,  0}, {1,  0},
		{-1,  1}, {0,  1}, {1,  1}
	};

	float64 last_time = os_get_current_time_in_seconds();
	while (!window.should_close) {
		reset_temporary_storage();

		draw_frame.projection = m4_make_orthographic_projection(window.width * -0.5, window.width * 0.5, window.height * -0.5, window.height * 0.5, -1, 10);
		float zoom = 5.0;
		draw_frame.view = m4_make_scale(v3(1.0/zoom, 1.0/zoom, 1.0));

		float64 now = os_get_current_time_in_seconds();
		float64 delta_t = now - last_time;
		last_time = now;

		os_update();

		// :render
		int valid_count = 0;

		// Phase 1: Build spatial lookup and reset particle properties
		for (int i = 0; i < MAX_ENTITY_COUNT; i++) {
			Entity* en = &world->entities[i];
			if (en->is_valid && en->arch == arch_particle) {
				en->cell_position = Get2DCell(en->position);
				en->hash = GetHashFrom2DCell(en->cell_position);
				en->cell_key = GetKeyFromHash(en->hash);

				SpatialLookup[valid_count].key = en->cell_key;
				SpatialLookup[valid_count].index = i;
				valid_count++;

				en->density = 0.0f;
				en->pressure = 0.0f;
				en->pressure_force = v2(0.0f, 0.0f);
			}
		}

		// Phase 2: Sort spatial lookup
		qsort(SpatialLookup, valid_count, sizeof(struct SpatialLookupStruct), CompareByKey);

		// Phase 2.5: Build StartIndices table
		for (int i = 0; i < PARTICLE_NUM * PARTICLE_NUM; i++) {
			StartIndices[i] = -1;
		}
		for (int i = 0; i < valid_count; i++) {
			int16 key = SpatialLookup[i].key;
			if (StartIndices[key] == -1) {
				StartIndices[key] = i;
			}
		}

		// Phase 3: Compute densities
		for (int i = 0; i < MAX_ENTITY_COUNT; i++) {
			Entity* en = &world->entities[i];
			if (!en->is_valid || en->arch != arch_particle) continue;

			// Add self-contribution to density
			en->density = PARTICLE_MASS * SmoothingKernel(0.0f);

			for (int k = 0; k < 9; k++) {
				Vector2i neighbor_cell = v2i_add(en->cell_position, grid_offsets[k]);
				uint64 neighbor_hash = GetHashFrom2DCell(neighbor_cell);
				int16 neighbor_key = GetKeyFromHash(neighbor_hash);
				if (neighbor_key < 0 || neighbor_key >= PARTICLE_NUM * PARTICLE_NUM) continue;
				
				int start = StartIndices[neighbor_key];
				if (start == -1) continue;
				if (start < 0 || start >= valid_count) {
					// invalid start index, skip or assert
					continue;
				}
				
				for (int m = start; m < valid_count && SpatialLookup[m].key == neighbor_key; m++) {
					Entity* en_n = &world->entities[SpatialLookup[m].index];
					if (!en_n->is_valid || en_n == en || en_n->arch != arch_particle) continue;

					float32 dist = v2_length(v2_sub(en_n->position, en->position));
					en->density += PARTICLE_MASS * SmoothingKernel(dist);
				}
			}
		}

		// Phase 4: Compute pressures
		for (int i = 0; i < MAX_ENTITY_COUNT; i++) {
			Entity* en = &world->entities[i];
			if (en->is_valid && en->arch == arch_particle) {
				en->pressure = ConvertDensityToPressure(en->density);
			}
		}

		// Phase 5: Compute pressure forces
		for (int i = 0; i < MAX_ENTITY_COUNT; i++) {
			Entity* en = &world->entities[i];
			if (!en->is_valid || en->arch != arch_particle) continue;

			for (int k = 0; k < 9; k++) {
				Vector2i neighbor_cell = v2i_add(en->cell_position, grid_offsets[k]);
				uint64 neighbor_hash = GetHashFrom2DCell(neighbor_cell);
				int16 neighbor_key = GetKeyFromHash(neighbor_hash);
				if (neighbor_key < 0 || neighbor_key >= PARTICLE_NUM * PARTICLE_NUM) continue;
				
				int start = StartIndices[neighbor_key]; 
				if (start == -1 || start >= valid_count) continue;
				for (int m = start; m < valid_count && SpatialLookup[m].key == neighbor_key; m++) {
					Entity* en_n = &world->entities[SpatialLookup[m].index];
					if (!en_n->is_valid || en_n == en || en_n->arch != arch_particle || en_n->density == 0.0f)
						continue;

					float32 dist = v2_length(v2_sub(en_n->position, en->position));
					if (dist == 0.0f) continue;

					Vector2 dir = v2_divf(v2_sub(en_n->position, en->position), dist);
					float32 slope = SmoothingKernelDerivative(dist);
					float32 sharedP = CalculateSharedPressure(en->pressure, en_n->pressure);

					en->pressure_force = v2_add(en->pressure_force,
						v2_mulf(dir, -sharedP * slope * PARTICLE_MASS / en_n->density));
				}
			}
		}

		// Phase 6: Integrate, handle collisions, render
		for (int i = 0; i < MAX_ENTITY_COUNT; i++) {
			Entity* en = &world->entities[i];
			if (!en->is_valid) continue;

			switch (en->arch) {
				case arch_particle: {
					if (en->density > 0.0f) {
						en->pressure_acceleration = v2_divf(en->pressure_force, en->density);
					} else {
						en->pressure_acceleration = v2(0.0f, 0.0f);
					}

					en->pressure_acceleration = v2_add(en->pressure_acceleration, v2(0.0f, -GRAVITY));

					en->velocity = v2_add(en->velocity, v2_mulf(en->pressure_acceleration, delta_t));
					en->position = v2_add(en->position, v2_mulf(en->velocity, delta_t));

					// Boundary collision
					if (en->position.y <= -70.0) {
						en->position.y = -69.9;
						en->velocity.y = -(en->velocity.y * BOUNCE_DAMPING);
					} else if (en->position.y >= 70.0) {
						en->position.y = 69.9;
						en->velocity.y = -(en->velocity.y * BOUNCE_DAMPING);
					} else if (en->position.x <= -120.0) {
						en->position.x = -119.9;
						en->velocity.x = -(en->velocity.x * BOUNCE_DAMPING);
					} else if (en->position.x >= 120.0) {
						en->position.x = 119.9;
						en->velocity.x = -(en->velocity.x * BOUNCE_DAMPING);
					}

					// Draw
					Vector2 size = v2(1.0, 1.0);
					Matrix4 xform = m4_scalar(1.0);
					xform = m4_translate(xform, v3(en->position.x, en->position.y, 0));
					xform = m4_translate(xform, v3(size.x * -0.5, size.y * -0.5, 0));
					draw_circle_xform(xform, size, COLOR_WHITE);
					break;
				}

				case arch_boundary:
					draw_line(en->boundaryPoint0, en->boundaryPoint1, 0.5, COLOR_RED);
					draw_line(en->boundaryPoint1, en->boundaryPoint2, 0.5, COLOR_RED);
					draw_line(en->boundaryPoint2, en->boundaryPoint3, 0.5, COLOR_RED);
					draw_line(en->boundaryPoint3, en->boundaryPoint0, 0.5, COLOR_RED);
					break;

				default:
					break;
			}
		}


		// Window Commands/Keys
		if (is_key_just_pressed(KEY_ESCAPE)) {
			window.should_close = true;
		}

		gfx_update();
		seconds_counter += delta_t;
		frame_count += 1;

		Entity* particle_en = &world->entities[0];
		Entity* particle_en2 = &world->entities[1];
		Entity* particle_en3 = &world->entities[2];
		Entity* particle_en4 = &world->entities[3];
		Entity* particle_en5 = &world->entities[4];
		Entity* particle_en6 = &world->entities[5];
		Entity* particle_en7 = &world->entities[6];
		Entity* particle_en8 = &world->entities[7];
		Entity* particle_en9 = &world->entities[8];


		
		if (seconds_counter > 1.0) {

			// log("Index: (%i)", particle_en->index);
			// log("Position: (%f, %f)", particle_en->position.x, particle_en->position.y);
			// log("Hash: (%i)", particle_en->hash);
			// log("Cell Key: (%i)", particle_en->cell_key);
			// log("Cell Position: (%i, %i) ", particle_en->cell_position.x, particle_en->cell_position.y);
			// log("Velocity: (%f, %f)", particle_en->velocity.x, particle_en->velocity.y);
			// log("\n");
			
			// log("Index: (%i)", particle_en2->index);
			// log("Position: (%f, %f)", particle_en2->position.x, particle_en2->position.y);
			// log("Hash: (%i)", particle_en2->hash);
			// log("Cell Key: (%i)", particle_en2->cell_key);
			// log("Cell Position: (%i, %i)", particle_en2->cell_position.x, particle_en2->cell_position.y);
			// log("Velocity: (%f, %f)", particle_en2->velocity.x, particle_en2->velocity.y);
			// log("\n");
		
			// log("Index: (%i)", particle_en3->index);
			// log("Position: (%f, %f)", particle_en3->position.x, particle_en3->position.y);
			// log("Hash: (%i)", particle_en3->hash);
			// log("Cell Key: (%i)", particle_en3->cell_key);
			// log("Cell Position: (%i, %i)", particle_en3->cell_position.x, particle_en3->cell_position.y);
			// log("Velocity: (%f, %f)", particle_en3->velocity.x, particle_en3 ->velocity.y);
			// log("\n");

			// log("Index: (%i)", particle_en4->index); 
			// log("Position: (%f, %f)", particle_en4->position.x, particle_en4->position.y);
			// log("Cell Key: (%i)", particle_en4->cell_key);
			// log("Cell Position: (%i, %i)", particle_en4->cell_position.x, particle_en4->cell_position.y);
			// log("Velocity: (%f, %f)", particle_en4->velocity.x, particle_en4->velocity.y);
			// log("\n");

			// log("Index: (%i)", particle_en5->index);
			// log("Position: (%f, %f)", particle_en5->position.x, particle_en5->position.y);
			// log("Cell Key: (%i)", particle_en5->cell_key);
			// log("Cell Position: (%i, %i)", particle_en5->cell_position.x, particle_en5->cell_position.y);
			// log("Velocity: (%f, %f)", particle_en5->velocity.x, particle_en5->velocity.y);
			// log("\n");

			// log("Index: (%i)", particle_en6->index);
			// log("Position: (%f, %f)", particle_en6->position.x, particle_en6->position.y);
			// log("Cell Key: (%i)", particle_en6->cell_key);
			// log("Cell Position: (%i, %i)", particle_en6->cell_position.x, particle_en6->cell_position.y);
			// log("Velocity: (%f, %f)", particle_en6->velocity.x, particle_en6->velocity.y);
			// log("\n");

			// log("Index: (%i)", particle_en7->index);
			// log("Position: (%f, %f)", particle_en7->position.x, particle_en7->position.y);
			// log("Cell Key: (%i)", particle_en7->cell_key);
			// log("Cell Position: (%i, %i)", particle_en7->cell_position.x, particle_en7->cell_position.y);
			// log("Velocity: (%f, %f)", particle_en7->velocity.x, particle_en7->velocity.y);
			// log("\n");

			// log("Index: (%i)", particle_en8->index);
			// log("Position: (%f, %f)", particle_en8->position.x, particle_en8->position.y);
			// log("Cell Key: (%i)", particle_en8->cell_key);
			// log("Cell Position: (%i, %i)", particle_en8->cell_position.x, particle_en8->cell_position.y);
			// log("Velocity: (%f, %f)", particle_en8->velocity.x, particle_en8->velocity.y);
			// log("\n");

			// log("Index: (%i)", particle_en9->index);
			// log("Position: (%f, %f)", particle_en9 ->position.x, particle_en9->position.y);
			// log("Cell Key: (%i)", particle_en9->cell_key);
			// log("Cell Position: (%i, %i)", particle_en9->cell_position.x, particle_en9->cell_position.y);
			// log("Velocity: (%f, %f)", particle_en9->velocity.x, particle_en9->velocity.y);
			// log("\n");

			log("delta_t: %f", delta_t);
			log("fps: %i\n", frame_count);
			log("Spatial Lookup: %i, %i", SpatialLookup[0].index, SpatialLookup[0].key);
			log("Index: (%i)", particle_en->index);
			log("Hash: (%i)", particle_en->hash);
			log("Cell Key: (%i)", particle_en->cell_key);
			log("Cell Position: (%i, %i)", particle_en->cell_position.x, particle_en->cell_position.y);
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