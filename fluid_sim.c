#define GRAVITY 9.81
#define BOUNCE_DAMPING 0.8

#define SMOOTHING_RADIUS 3
#define PARTICLE_MASS 3.0

#define REST_DENSITY 2.75
#define PRESSURE_CONSTANT 5

#define PARTICLE_NUM 20

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
	int hash;
	int cell_key;
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
	en->position = v2(-25.0, -25.0);
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
	if (distance > SMOOTHING_RADIUS) {return 0.0f;}

	float32 volume = (PI32 * powf(SMOOTHING_RADIUS, 4)) / 6.f;
	return (SMOOTHING_RADIUS - distance) * (SMOOTHING_RADIUS - distance) / volume;
}
// Smoothing Kernel Derivative
float32 SmoothingKernelDerivative(float32 distance) {
	if (distance > SMOOTHING_RADIUS) {return 0.0f;}

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
const int hash1 = 3079;
const int hash2 = 6151;
// Get 2D Cell Co-ordinates from particle position
Vector2i Get2DCell(Vector2 position) {
	float x = position.x;
	float y = position.y;
	int cellX = floorf((x + 120) / (int)SMOOTHING_RADIUS);
	int cellY = floorf((y + 70) / (int)SMOOTHING_RADIUS);
	return v2i(cellX, cellY);
}
// Hash Cell Co-ordinates into a single Int
int GetHash2DCell(Vector2i cell_coord) {
	int x = cell_coord.x;
	int y = cell_coord.y;
	int a = x * hash1;
	int b = y * hash2;
	return a+b;
}
// Hash to Key
int HashToKey(int hash) {
	return hash % (PARTICLE_NUM * PARTICLE_NUM);
}
// Struct for Lookup Array Data
struct SpatialLookupStruct {
	Entity* index;
	int key;
};
// Comparison Function for Sorting based on Cell Key
int CompareByKey(const void* a, const void* b) {
    int key_a = ((struct SpatialLookupStruct*)a)->index->cell_key;
    int key_b = ((struct SpatialLookupStruct*)b)->index->cell_key;
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
	struct SpatialLookupStruct SpatialLookupArray[PARTICLE_NUM*PARTICLE_NUM] = { 0 };
	// Initialise Array for Inidicie Start
	int StartIndicies[PARTICLE_NUM*PARTICLE_NUM] = { 0 };

	int k = 0;
    for (int i = 0; i < PARTICLE_NUM; i++) {
		for (int j = 0; j < PARTICLE_NUM; j++) {
			Entity* en = entity_create();
			setup_particle(en);
			en->position = v2(1.5*i-7.5, 1.5*j-7.5);
			en->index = k;
			SpatialLookupArray[k].index = &world->entities[k];
			//log("Index Address: %p", SpatialLookupArray[k].index);
			k++;
		}
    }

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

		// Cell Linked List
		for (int i = 0; i < MAX_ENTITY_COUNT; i++) {
			Entity* en = &world->entities[i];
			if (en->is_valid) {

				switch (en->arch) {

					case arch_particle:
						// Cell Linked List Optimisation
						en->cell_position = Get2DCell(en->position);
						en->hash = GetHash2DCell(en->cell_position);
						en->cell_key = HashToKey(en->hash);
						SpatialLookupArray[i].index->cell_key = en->cell_key;
						//int n = sizeof(SpatialLookupArray) / sizeof(SpatialLookupArray[0]);
						//qsort(SpatialLookupArray, n, sizeof(struct SpatialLookupStruct), CompareByKey);

						//log("%i %i", SpatialLookupArray[0].index, SpatialLookupArray[0].index->cell_key);

						break;
						
					default:
					{break;}
				}
			}
		}

		int n = sizeof(SpatialLookupArray) / sizeof(SpatialLookupArray[0]);
		qsort(SpatialLookupArray, n, sizeof(struct SpatialLookupStruct), CompareByKey);

		int StartIndicies[PARTICLE_NUM*PARTICLE_NUM] = { 0 };

		for (int i = 0; i < MAX_ENTITY_COUNT; i++) {
			Entity* en = &world->entities[i];
			if (en->is_valid) {

				switch (en->arch) {

					case arch_particle:
											
						// Set Start Indicies Array with change of Cell Key
						if (i > 0 && SpatialLookupArray[i].index->cell_key != SpatialLookupArray[i-1].index->cell_key) {
							StartIndicies[SpatialLookupArray[i].index->cell_key] = i;
							// log("%i", StartIndicies[en->cell_key]);
						}
						break;
		
					default:
					{break;}
				}
			}
		}

		for (int i = 0; i < MAX_ENTITY_COUNT; i++) {
			Entity* en = &world->entities[i];
			if (en->is_valid) {

				switch (en->arch) {

					case arch_particle:
						// Reset Particle Properties
						//log("Density: %f", en->density);
						en->density = 0.0f;
						en->pressure = 0.0f;
						en->pressure_force = v2(0.0f, 0.0f);
						
						// For Each Neighbour Particle Within Particle Radius
						// Loop through 3x3 cells around the centre cell
						for (int x = 0; x < 3; x++) {
							for (int y = 0; y < 3; y++) {
								Vector2i cell_position = v2i_sub(en->cell_position, v2i(x-1, y-1));

								if (cell_position.x <= -1 || cell_position.x >= 80){break;}
								else if (cell_position.y <= -1 || cell_position.y >= 47)
								{continue;}
								

								int cell_key = HashToKey(GetHash2DCell(cell_position));
								int cell_start_index = StartIndicies[cell_key];

								// With key of current cell, loop over all points with the same key
								for (int z = cell_start_index; z < square(PARTICLE_NUM); z++) {

									if (SpatialLookupArray[z].index->cell_key != cell_key) {break;}

									//log("Memory Address Index: %p", SpatialLookupArray[z].index);

									Entity* en_neighbour = SpatialLookupArray[z].index;

									float32 distance = v2_length(v2_sub(en_neighbour->position, en->position));

									if (distance <= SMOOTHING_RADIUS) {
										// Calculate Density;
										float32 influence = SmoothingKernel(distance);
										en->density += PARTICLE_MASS * influence;
									}
								}
							} 
						}
						
						// Calculate Pressure
						en->pressure = ConvertDensityToPressure(en->density);

						for (int x = 0; x < 3; x++) {
							for (int y = 0; y < 3; y++) {
								Vector2i cell_position = v2i_sub(en->cell_position, v2i(x-1, y-1));

								if (cell_position.x <= -1 || cell_position.x >= 80){break;}
								else if (cell_position.y <= -1 || cell_position.y >= 47)
								{continue;}

								int cell_key = HashToKey(GetHash2DCell(cell_position));
								int cell_start_index = StartIndicies[cell_key];

								// With key of current cell, loop over all points with the same key
								for (int z = cell_start_index; z < square(PARTICLE_NUM); z++) {

									if (SpatialLookupArray[z].index->cell_key != cell_key) {break;}

									//log("Memory Address Index: %p", SpatialLookupArray[z].index);

									Entity* en_neighbour = SpatialLookupArray[z].index;

									float32 distance = v2_length(v2_sub(en_neighbour->position, en->position));

									if (distance <= SMOOTHING_RADIUS) {
										if(en == en_neighbour || en_neighbour->density == 0.0f){continue;}

										// Calculate Pressure Force
										float32 distance = v2_length(v2_sub(en_neighbour->position, en->position));
										Vector2 direction = v2_divf(v2_sub(en_neighbour->position, en->position), distance);
										float32 slope = SmoothingKernelDerivative(distance);
										//log("%f", slope);
										float32 sharedPressure = CalculateSharedPressure(en->pressure, en_neighbour->pressure);
										en->pressure_force = v2_add(v2_mulf(direction, -sharedPressure * slope * PARTICLE_MASS / en_neighbour->density), en->pressure_force);

									}
								}
							} 
						}


						// Calculate Pressure Force Acceleration
						if (en->density > 0.0f) {
							en->pressure_acceleration = v2_divf(en->pressure_force, en->density);
						} else {
							en->pressure_acceleration = v2f32_zero;
						}

						break;
						
					default:
					{break;}
				}
			}
		}


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
							en->velocity.x = -(en->velocity.y * BOUNCE_DAMPING);
						}
						else if (en->position.x >= 120.0)
						{
							en->position.x = 119.9;
							en->velocity.x = -(en->velocity.y * BOUNCE_DAMPING);
						}
						
						// Particle Transform
						Vector2 size	= v2(1.0,1.0);
						Matrix4 xform	= m4_scalar(1.0);
						xform			= m4_translate(xform, v3(en->position.x, en->position.y, 0));
						xform			= m4_translate(xform, v3(size.x * -0.5, size.y * -0.5, 0));
						draw_circle_xform(xform, size, COLOR_WHITE);

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

		Entity* particle_en = &world->entities[0];

		gfx_update();
		seconds_counter += delta_t;
		frame_count += 1;
		if (seconds_counter > 1.0) {
			log("delta_t: %f", delta_t);
			log("fps: %i\n", frame_count);
			log("Spatial Lookup: %i, %i", SpatialLookupArray[0].index->index, SpatialLookupArray[0].index->cell_key);
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