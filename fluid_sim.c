#define GRAVITY 9.81
#define BOUNCE_DAMPING 0.8

#define SMOOTHING_RADIUS 3.0
#define PARTICLE_MASS 1.0

#define REST_DENSITY 2.75
#define PRESSURE_CONSTANT 25.0

#define PARTICLE_NUM 12

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
	return hash % (PARTICLE_NUM * PARTICLE_NUM);
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
    for (int i = 0; i < PARTICLE_NUM	; i++) {
		for (int j = 0; j < PARTICLE_NUM	; j++) {
			Entity* en = entity_create();
			setup_particle(en);
			en->position = v2(1.5*i-3, 1.5*j-3 );
			
			en->index = k;
			SpatialLookup[k].index = k;
			k++;
		}
    }

	// Initialise Array for Inidicie Start
	int StartIndicies[PARTICLE_NUM*PARTICLE_NUM];

	Entity* boundary0_en = entity_create();
	setup_boundary(boundary0_en);
	boundary0_en->boundaryPoint0 = v2(-120, -70);
	boundary0_en->boundaryPoint1 = v2( 120, -70);
	boundary0_en->boundaryPoint2 = v2( 120,  70);
	boundary0_en->boundaryPoint3 = v2(-120,  70);

	float64 seconds_counter = 0.0;
	s32 frame_count = 0;

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
		for (int i = 0; i < MAX_ENTITY_COUNT; i++) {
			Entity* en = &world->entities[i];
			if (en->is_valid) {

				switch (en->arch) {

					case arch_particle:

						// Particle Updates
						// Velocity
						// en->velocity = v2_add(en->velocity, v2_mulf(v2(0.0, -GRAVITY), delta_t));
						// en->velocity = v2_add(en->velocity, en->pressure_acceleration);
						// en->velocity = en->pressure_acceleration;
						en->velocity = v2_add(en->velocity, v2_mulf(en->pressure_acceleration, delta_t));
						// Position
						en->position = v2_add(en->position, v2_mulf(en->velocity, delta_t));

						// Simple Collision w/ BoundaryFloor
						if (en->position.y <= -70.0) {
							en->position.y = -69.9;
							en->velocity.y = -(en->velocity.y * BOUNCE_DAMPING);
						}
						else if (en->position.y >= 70.0)
						{
							en->position.y = 69.9;
							en->velocity.y = -(en->velocity.y * BOUNCE_DAMPING);
						}
						else if (en->position.x <= -120.0)
						{
							en->position.x = -119.9;
							en->velocity.x = -(en->velocity.x * BOUNCE_DAMPING);
						}
						else if (en->position.x >= 120.0)
						{
							en->position.x = 119.9;
							en->velocity.x = -(en->velocity.x * BOUNCE_DAMPING);
						}
						
						// Particle Transform
						Vector2 size	= v2(1.0,1.0);
						Matrix4 xform	= m4_scalar(1.0);
						xform			= m4_translate(xform, v3(en->position.x, en->position.y, 0));
						xform			= m4_translate(xform, v3(size.x * -0.5, size.y * -0.5, 0));
						draw_circle_xform(xform, size, COLOR_WHITE);

						// Cell Linked List Optimisation
						// Calculate Cell Co-ordinates with Particel Position
						en->cell_position = Get2DCell(en->position);
						// Convert Co-ords into Hash
						en->hash = GetHashFrom2DCell(en->cell_position);
						
						// Hash to Key
						en->cell_key = GetKeyFromHash(en->hash);
						// Add Cell Keys to Array Struct
						SpatialLookup[i].key = en->cell_key;
						int16 n = sizeof(SpatialLookup) / sizeof(SpatialLookup[0]);
						qsort(SpatialLookup, n, sizeof(struct SpatialLookupStruct), CompareByKey);
 
						//log("%i %i", SpatialLookup[0].index, SpatialLookup[0].key);
						
						// Set Start Indicies of each Cell to an Array
						if (i > 0 && SpatialLookup[i].key != SpatialLookup[i-1].key) {
							StartIndicies[en->cell_key] = i;
							//log("%i", StartIndicies[en->cell_key]);
						}

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
										float32 influence = SmoothingKernel(distance);
										en->density += PARTICLE_MASS * influence;

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
						en->pressure = ConvertDensityToPressure(en->density);

						// Search Neighbour Particles to Calculate Forces
						for (int j = 0; j < MAX_ENTITY_COUNT; j++) {
							Entity* en_neighbour = &world->entities[j];
							if (en_neighbour->is_valid) {
								
								switch (en_neighbour->arch) {

									case arch_particle:

										if(en == en_neighbour || en_neighbour->density == 0.0f){break;}

										// Calculate Pressure Force
										float32 distance = v2_length(v2_sub(en_neighbour->position, en->position));
										Vector2 direction = v2_divf(v2_sub(en_neighbour->position, en->position), distance);
										float32 slope = SmoothingKernelDerivative(distance);
										//log("%f", slope);
										float32 sharedPressure = CalculateSharedPressure(en->pressure, en_neighbour->pressure);
										en->pressure_force = v2_add(v2_mulf(direction, -sharedPressure * slope * PARTICLE_MASS / en_neighbour->density), en->pressure_force);

										break;

									default:
										break;
								}
							}
						}

						// Calculate Pressure Force Acceleration
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

				// int16 n = sizeof(SpatialLookup) / sizeof(SpatialLookup[0]);
				// qsort(SpatialLookup, n, sizeof(struct SpatialLookupStruct), CompareByKey);
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

			log("Index: (%i)", particle_en->index);
			log("Position: (%f, %f)", particle_en->position.x, particle_en->position.y);
			log("Hash: (%i)", particle_en->hash);
			log("Cell Key: (%i)", particle_en->cell_key);
			log("Cell Position: (%i, %i) ", particle_en->cell_position.x, particle_en->cell_position.y);
			log("Velocity: (%f, %f)", particle_en->velocity.x, particle_en->velocity.y);
			log("\n");
			
			log("Index: (%i)", particle_en2->index);
			log("Position: (%f, %f)", particle_en2->position.x, particle_en2->position.y);
			log("Hash: (%i)", particle_en2->hash);
			log("Cell Key: (%i)", particle_en2->cell_key);
			log("Cell Position: (%i, %i)", particle_en2->cell_position.x, particle_en2->cell_position.y);
			log("Velocity: (%f, %f)", particle_en2->velocity.x, particle_en2->velocity.y);
			log("\n");
		
			log("Index: (%i)", particle_en3->index);
			log("Position: (%f, %f)", particle_en3->position.x, particle_en3->position.y);
			log("Hash: (%i)", particle_en3->hash);
			log("Cell Key: (%i)", particle_en3->cell_key);
			log("Cell Position: (%i, %i)", particle_en3->cell_position.x, particle_en3->cell_position.y);
			log("Velocity: (%f, %f)", particle_en3->velocity.x, particle_en3 ->velocity.y);
			log("\n");

			log("Index: (%i)", particle_en4->index); 
			log("Position: (%f, %f)", particle_en4->position.x, particle_en4->position.y);
			log("Cell Key: (%i)", particle_en4->cell_key);
			log("Cell Position: (%i, %i)", particle_en4->cell_position.x, particle_en4->cell_position.y);
			log("Velocity: (%f, %f)", particle_en4->velocity.x, particle_en4->velocity.y);
			log("\n");

			log("Index: (%i)", particle_en5->index);
			log("Position: (%f, %f)", particle_en5->position.x, particle_en5->position.y);
			log("Cell Key: (%i)", particle_en5->cell_key);
			log("Cell Position: (%i, %i)", particle_en5->cell_position.x, particle_en5->cell_position.y);
			log("Velocity: (%f, %f)", particle_en5->velocity.x, particle_en5->velocity.y);
			log("\n");

			log("Index: (%i)", particle_en6->index);
			log("Position: (%f, %f)", particle_en6->position.x, particle_en6->position.y);
			log("Cell Key: (%i)", particle_en6->cell_key);
			log("Cell Position: (%i, %i)", particle_en6->cell_position.x, particle_en6->cell_position.y);
			log("Velocity: (%f, %f)", particle_en6->velocity.x, particle_en6->velocity.y);
			log("\n");

			log("Index: (%i)", particle_en7->index);
			log("Position: (%f, %f)", particle_en7->position.x, particle_en7->position.y);
			log("Cell Key: (%i)", particle_en7->cell_key);
			log("Cell Position: (%i, %i)", particle_en7->cell_position.x, particle_en7->cell_position.y);
			log("Velocity: (%f, %f)", particle_en7->velocity.x, particle_en7->velocity.y);
			log("\n");

			log("Index: (%i)", particle_en8->index);
			log("Position: (%f, %f)", particle_en8->position.x, particle_en8->position.y);
			log("Cell Key: (%i)", particle_en8->cell_key);
			log("Cell Position: (%i, %i)", particle_en8->cell_position.x, particle_en8->cell_position.y);
			log("Velocity: (%f, %f)", particle_en8->velocity.x, particle_en8->velocity.y);
			log("\n");

			log("Index: (%i)", particle_en9->index);
			log("Position: (%f, %f)", particle_en9 ->position.x, particle_en9->position.y);
			log("Cell Key: (%i)", particle_en9->cell_key);
			log("Cell Position: (%i, %i)", particle_en9->cell_position.x, particle_en9->cell_position.y);
			log("Velocity: (%f, %f)", particle_en9->velocity.x, particle_en9->velocity.y);
			log("\n");

			// log("delta_t: %f", delta_t);
			// log("fps: %i\n", frame_count);
			// log("Spatial Lookup: %i, %i", SpatialLookup[0].index, SpatialLookup[0].key);
			// log("Index: (%i)", particle_en->index);
			// log("Hash: (%i)", particle_en->hash);
			// log("Cell Key: (%i)", particle_en->cell_key);
			// log("Cell Position: (%i, %i)", particle_en->cell_position.x, particle_en->cell_position.y);
			// log("Position: (%f, %f)", particle_en->position.x, particle_en->position.y);
			// log("Velocity: (%f, %f)", particle_en->velocity.x, particle_en->velocity.y);
			// log("Density: %f", particle_en->density);
			// log("Pressure: %f", particle_en->pressure);
			// log("Pressure Force: %f, %f", particle_en->pressure_force.x, particle_en->pressure_force.y);
			// log("Pressure Accel: %f, %f\n", particle_en->pressure_acceleration.x, particle_en->pressure_acceleration.y);
			seconds_counter = 0.0;
			frame_count = 0;
		}
	}

	return 0;
}