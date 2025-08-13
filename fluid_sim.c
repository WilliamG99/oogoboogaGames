// --- Simulation Parameters ---

// #define GRAVITY 10.0
// #define BOUNCE_DAMPING 0.1

// #define SMOOTHING_RADIUS 5.0
// #define PARTICLE_MASS 1.0

// #define REST_DENSITY 0.15
// #define PRESSURE_CONSTANT 1000.0
// #define NEAR_PRESSURE_CONSTANT 3000.0
// #define VISCOSITY_CONSTANT 0.2

// #define INTERACTION_RADIUS 25.0
// #define INTERACTION_STRENGTH 35.0

////////////////////////////
// #define PARTICLE_NUM 25

// float GRAVITY = 10.0f;
// float BOUNCE_DAMPING = 0.1f;

// float SMOOTHING_RADIUS = 5.0;
// float PARTICLE_MASS = 1.0f;

// float REST_DENSITY = 0.2f;
// float PRESSURE_CONSTANT = 4000.0f;
// float NEAR_PRESSURE_CONSTANT = 1500.0f;
// float VISCOSITY_CONSTANT = 0.10f;

// float INTERACTION_RADIUS = 40.0f;
// float INTERACTION_STRENGTH = 25.0f;
////////////////////////////////

#define PARTICLE_NUM 25

float GRAVITY = 10.0f;
float BOUNCE_DAMPING = 0.1f;

float SMOOTHING_RADIUS = 5.0;
float PARTICLE_MASS = 1.0f;

float REST_DENSITY = 0.2f;
float PRESSURE_CONSTANT = 4000.0f;
float NEAR_PRESSURE_CONSTANT = 1500.0f;
float VISCOSITY_CONSTANT = 0.10f;

float INTERACTION_RADIUS = 40.0f;
float INTERACTION_STRENGTH = 25.0f;

Gfx_Font* g_font = 0;

// --- UI System ---

typedef struct {
    char label[32];
    string text_to_draw;
    char edit_buffer[32];
    float* target_variable;
    Vector2 position;
    bool is_active;
} UITextInput;

#define NUM_UI_INPUTS 9
typedef struct {
    bool is_open;
    Vector2 position;
    Vector2 size;
    int active_input_index;
    
    UITextInput inputs[NUM_UI_INPUTS];

    Gfx_Image* ui_texture;
    bool is_dirty;

} UIState;

UIState ui_state;

// --- End UI System ---

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
	float32 near_density;
	float32 pressure;
	float32 near_pressure;
	Vector2 pressure_force;
	Vector2 viscosity_force;
	Vector2 pressure_acceleration;

	Vector2 boundaryPoint0;
	Vector2 boundaryPoint1;
	Vector2 boundaryPoint2;
	Vector2 boundaryPoint3;
} Entity;
#define MAX_ENTITY_COUNT 4096

void UI_Initialize() {
    ui_state.is_open = true;
    ui_state.position = v2(1285, 465);
    ui_state.size = v2(250, 240);
    ui_state.active_input_index = -1;

    ui_state.inputs[0] = (UITextInput){ .label = "Gravity", .target_variable = &GRAVITY };
    ui_state.inputs[1] = (UITextInput){ .label = "Radius", .target_variable = &SMOOTHING_RADIUS };
    ui_state.inputs[2] = (UITextInput){ .label = "Mass", .target_variable = &PARTICLE_MASS };
    ui_state.inputs[3] = (UITextInput){ .label = "Rest Dens.", .target_variable = &REST_DENSITY };
    ui_state.inputs[4] = (UITextInput){ .label = "Pressure", .target_variable = &PRESSURE_CONSTANT };
    ui_state.inputs[5] = (UITextInput){ .label = "Near Press", .target_variable = &NEAR_PRESSURE_CONSTANT };
    ui_state.inputs[6] = (UITextInput){ .label = "Viscosity", .target_variable = &VISCOSITY_CONSTANT };
    ui_state.inputs[7] = (UITextInput){ .label = "Intrct Rad", .target_variable = &INTERACTION_RADIUS };
    ui_state.inputs[8] = (UITextInput){ .label = "Intrct Str", .target_variable = &INTERACTION_STRENGTH };

    // loop initializes the text for all inputs, regardless of order.
    for (int i = 0; i < NUM_UI_INPUTS; i++) {
        ui_state.inputs[i].text_to_draw = sprintf(get_heap_allocator(), "%.2f", *ui_state.inputs[i].target_variable);
        ui_state.inputs[i].is_active = false;
    }
}

void UI_Draw() {
    if (!ui_state.is_open) {
        return;
    }

    // Draw the main panel background
    Vector4 panel_color = hex_to_rgba(0x3c3f4c88);
    draw_rect(ui_state.position, ui_state.size, panel_color);

    // --- Draw Text Fields ---
    float padding = 10.0f;
    float line_height = 25.0f;
    float label_width = 80.0f;
    float input_box_width = 120.0f;
    float input_box_height = 20.0f;

    for (int i = 0; i < NUM_UI_INPUTS; i++) {
        UITextInput* input = &ui_state.inputs[i];
        
		Vector2 label_pos = v2(
			ui_state.position.x + padding,
			ui_state.position.y + ui_state.size.y - padding - (i + 1) * line_height
		);
		Vector2 input_box_pos = v2(label_pos.x + label_width, label_pos.y);

        // Draw the static label text
        draw_text(g_font, STR(input->label), 16, label_pos, v2(1, 1), COLOR_WHITE);

        // Draw the input box background
        Vector4 box_color = (ui_state.active_input_index == i) ? COLOR_BLACK : hex_to_rgba(0x2a2d3aFF);
        draw_rect(input_box_pos, v2(input_box_width, input_box_height), box_color);
        
        // Draw the pre-made string for the value
        Vector2 text_pos = v2(input_box_pos.x + 5, input_box_pos.y + 2);
        draw_text(g_font, input->text_to_draw, 16, text_pos, v2(1,1), COLOR_WHITE);
    }
}

void UI_Update() {
    if (!ui_state.is_open) {
        return;
    }

    // --- MOUSE CLICK LOGIC ---
    if (is_key_down(MOUSE_BUTTON_LEFT)) {
        Vector2 mouse_pos = v2(input_frame.mouse_x, input_frame.mouse_y);
        
        float padding = 10.0f;
        float line_height = 25.0f;
        float label_width = 80.0f;
        float input_box_width = 120.0f;
        float input_box_height = 20.0f;

        bool an_input_was_clicked = false;
        for (int i = 0; i < NUM_UI_INPUTS; i++) {
			Vector2 input_box_pos = v2(
				ui_state.position.x + padding + label_width,
				ui_state.position.y + ui_state.size.y - padding - (i + 1) * line_height
			);

            if (mouse_pos.x >= input_box_pos.x && mouse_pos.x <= input_box_pos.x + input_box_width &&
                mouse_pos.y >= input_box_pos.y && mouse_pos.y <= input_box_pos.y + input_box_height)
            {
                ui_state.active_input_index = i;
                an_input_was_clicked = true;
                break;
            }
        }
        
        // If the user clicked outside of any input box, deactivate them all.
        if (!an_input_was_clicked) {
            ui_state.active_input_index = -1;
        }
    }

    // --- KEYBOARD INPUT LOGIC ---
    // This part handles typing into the selected box.
    if (ui_state.active_input_index != -1) {
        UITextInput* active_input = &ui_state.inputs[ui_state.active_input_index];
        bool text_was_changed = false;

        // When an input is first clicked, copy its text to a temporary buffer for editing.
        if (active_input->is_active == false) {
            active_input->is_active = true;
            memcpy(active_input->edit_buffer, active_input->text_to_draw.data, active_input->text_to_draw.count);
            active_input->edit_buffer[active_input->text_to_draw.count] = '\0';
        }

        // Loop through all input events for the frame.
        for (u64 i = 0; i < input_frame.number_of_events; i++) {
            Input_Event e = input_frame.events[i];
            
            // Handle character typing.
            if (e.kind == INPUT_EVENT_TEXT) {
                int buffer_len = strlen(active_input->edit_buffer);
                char typed_char = e.ascii;

                if ((typed_char >= '0' && typed_char <= '9') || typed_char == '.' || typed_char == '-') {
                    if (buffer_len < sizeof(active_input->edit_buffer) - 1) {
                        active_input->edit_buffer[buffer_len] = typed_char;
                        active_input->edit_buffer[buffer_len + 1] = '\0';
                        text_was_changed = true;
                    }
                }
            }
            // Handle key presses like Backspace.
            else if (e.kind == INPUT_EVENT_KEY && (e.key_state & INPUT_STATE_JUST_PRESSED || e.key_state & INPUT_STATE_REPEAT)) {
                if (e.key_code == KEY_BACKSPACE) {
                    int buffer_len = strlen(active_input->edit_buffer);
                    if (buffer_len > 0) {
                        active_input->edit_buffer[buffer_len - 1] = '\0';
                        text_was_changed = true;
                    }
                }
            }
        }

        // If the text was changed by typing or backspace, update the visible string.
        if (text_was_changed) {
            active_input->text_to_draw = sprintf(get_heap_allocator(), "%s", active_input->edit_buffer);
        }
        
        // Handle pressing Enter to confirm the new value.
        if (is_key_just_pressed(KEY_ENTER)) {
            float new_value = atof(active_input->edit_buffer);
            *active_input->target_variable = new_value;
            active_input->text_to_draw = sprintf(get_heap_allocator(), "%.2f", new_value);
            ui_state.active_input_index = -1; // De-select the box.
        }
    }

    // If no input box is selected, make sure all are marked as inactive.
    if (ui_state.active_input_index == -1) {
        for (int i = 0; i < NUM_UI_INPUTS; i++) {
            ui_state.inputs[i].is_active = false;
        }
    }
}


Vector2 screen_to_world() {
    float mouse_x = input_frame.mouse_x;
    float mouse_y = input_frame.mouse_y;
    Matrix4 proj = draw_frame.projection;
    Matrix4 view = draw_frame.camera_xform;
    float window_w = window.width;
    float window_h = window.height;

    float ndcX = (mouse_x / (window_w * 0.5f)) - 1.0f;
    float ndcY = 1.0f - (mouse_y / (window_h * 0.5f));

	// Transform to world coordinates
	Vector4 world_pos = v4(ndcX, ndcY, 0, 1);
	world_pos = m4_transform(m4_inverse(proj), world_pos);
	world_pos = m4_transform(view, world_pos);

	return (Vector2){ world_pos.x, -world_pos.y };
}

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
	en->near_density = 0.0f;
	en->pressure = 0.0f;
	en->near_pressure = 0.0f;
	en->pressure_force = v2(0.0f, 0.0f);
}

void setup_boundary(Entity* en) {
	en->arch = arch_boundary;
}

// Main functions and calculations
float32 SpikyKernelPow2(float distance) {
	if (distance >= SMOOTHING_RADIUS) {return 0.0f;}

	float32 scale = 6.0f / (PI32 * powf(SMOOTHING_RADIUS, 4));
	float32 distance_from_edge = SMOOTHING_RADIUS - distance;

	return distance_from_edge * distance_from_edge * scale;
}

float32 SpikyKernelPow2Derivative(float32 distance) {
	if (distance >= SMOOTHING_RADIUS) {return 0.0f;}

	float32 scale = 12.0f / (PI32 * powf(SMOOTHING_RADIUS, 4));
    float32 distance_from_edge = SMOOTHING_RADIUS - distance;

    return -(distance_from_edge * scale);
}

float32 SpikyKernelPow3(float distance) {
    if (distance >= SMOOTHING_RADIUS) { return 0.0f; }

    float32 scale = 10.0f / (PI32 * powf(SMOOTHING_RADIUS, 5));
    float32 distance_from_edge = SMOOTHING_RADIUS - distance;

    return distance_from_edge * distance_from_edge * distance_from_edge * scale;
}

float32 SpikyKernelPow3Derivative(float distance) {
    if (distance >= SMOOTHING_RADIUS) { return 0.0f; }

    float32 scale = 30.0f / (PI32 * powf(SMOOTHING_RADIUS, 5));
    float32 distance_from_edge = SMOOTHING_RADIUS - distance;

    return -(distance_from_edge * distance_from_edge * scale);
}

// For Viscosity
float Poly6Kernel(float distance) {
    if (distance >= SMOOTHING_RADIUS) { return 0.0f; }

    float scale = 4.0f / (PI32 * powf(SMOOTHING_RADIUS, 8));
    float v = SMOOTHING_RADIUS * SMOOTHING_RADIUS - distance * distance;

    return v * v * v * scale;
}

float32 ConvertDensityToPressure(float32 density) {
	float32 density_error = density - REST_DENSITY;
	float32 pressure = density_error * PRESSURE_CONSTANT;
    return pressure;
}

float32 CalculateSharedPressure(float32 pressureA, float32 pressureB) {
	return (pressureA + pressureB) / 2;
}

// Cell Linked List Optimisation
const int hash1 = 15823;
const int hash2 = 9737333;

Vector2i Get2DCell(Vector2 position) {
	int cellX = (int)floorf(position.x / SMOOTHING_RADIUS);
	int cellY = (int)floorf(position.y / SMOOTHING_RADIUS);
	return v2i(cellX, cellY);
}

int GetHashFrom2DCell(Vector2i cell_coord) {
	int a = cell_coord.x * hash1;
	int b = cell_coord.y * hash2;
	return a+b;
}

int GetKeyFromHash(int hash) {
    int table_size = PARTICLE_NUM * PARTICLE_NUM;
    return (hash % table_size + table_size) % table_size;
}

struct SpatialLookupStruct {
	int16 index;
	int16 key;
};

int CompareByKey(const void* a, const void* b) {
    int16 key_a = ((struct SpatialLookupStruct*)a)->key;
    int16 key_b = ((struct SpatialLookupStruct*)b)->key;
    return (key_a < key_b) ? -1 : (key_a > key_b);
}

int entry(int argc, char **argv) {
	window.title = STR("FluidSim");
	window.width = 1580;
	window.height = 720;
	window.x = 200;
	window.y = 200;
	window.clear_color = hex_to_rgba(0x2a2d3aff);
	
	g_font = load_font_from_disk(STR("C:/windows/fonts/arial.ttf"), get_heap_allocator());
	assert(g_font, "Failed to load font!");
	
	world = alloc(get_heap_allocator(), sizeof(World));
	memset(world, 0, sizeof(World));

	UI_Initialize();
	
	struct SpatialLookupStruct SpatialLookup[PARTICLE_NUM*PARTICLE_NUM];

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

	{
    // Pick a particle in the center of the grid for an average reading
    int sample_particle_index = (PARTICLE_NUM * PARTICLE_NUM) / 2 + PARTICLE_NUM / 2;
    Entity* sample_particle = &world->entities[sample_particle_index];
    
    float32 total_density = 0.0f;
    float32 total_near_density = 0.0f;

    // Calculate its density based on the initial positions of all other particles
    for (int i = 0; i < PARTICLE_NUM * PARTICLE_NUM; i++) {
        Entity* other_particle = &world->entities[i];
        float32 dist = v2_length(v2_sub(sample_particle->position, other_particle->position));
        total_density += PARTICLE_MASS * SpikyKernelPow2(dist);
        total_near_density += PARTICLE_MASS * SpikyKernelPow3(dist);
    }
    
    log("\n================================================");
    log("Calculated Initial Density: %f", total_density);
    log("Use this value for your REST_DENSITY define.");
    log("================================================\n");
}

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

    Vector2 mouse_pos_world = v2(0.0f, 0.0f);

	float64 last_time = os_get_elapsed_seconds();
	while (!window.should_close) {
		reset_temporary_storage();

		float64 now = os_get_elapsed_seconds();
		float64 delta_t = now - last_time;
		last_time = now;
		
		os_update();
		UI_Update();
		
		draw_frame.projection = m4_make_orthographic_projection(window.width * -0.5, window.width * 0.5, window.height * -0.5, window.height * 0.5, -1, 10);
		float zoom = 5.0;
		draw_frame.camera_xform = m4_make_scale(v3(1.0, 1.0, 1.0));
		draw_frame.camera_xform = m4_mul(draw_frame.camera_xform, m4_make_translation(v3(30, 0, 0)));
		draw_frame.camera_xform = m4_mul(draw_frame.camera_xform, m4_make_scale(v3(1.0/zoom, 1.0/zoom, 1.0)));

        if (is_key_down(MOUSE_BUTTON_LEFT) || is_key_down(MOUSE_BUTTON_RIGHT)) {
            mouse_pos_world = screen_to_world();
        }

		int valid_count = 0;

		// // --- LOGGING: START OF FRAME ---
		// Entity* p0 = &world->entities[0];
		// log("\n--- FRAME START: Particle 0 ---");
		// log("Position: (%f, %f), Velocity: (%f, %f)", p0->position.x, p0->position.y, p0->velocity.x, p0->velocity.y);


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
				en->near_density = 0.0f;
				en->pressure = 0.0f;
				en->near_pressure = 0.0f;
				en->pressure_force = v2(0.0f, 0.0f);
				en->viscosity_force = v2(0.0f, 0.0f);
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

			// Self-contribution for density
			en->density = PARTICLE_MASS * SpikyKernelPow2(0.0f);
			en->near_density = PARTICLE_MASS * SpikyKernelPow3(0.0f); 

			for (int k = 0; k < 9; k++) {
				Vector2i neighbor_cell = v2i_add(en->cell_position, grid_offsets[k]);
				uint64 neighbor_hash = GetHashFrom2DCell(neighbor_cell);
				int16 neighbor_key = GetKeyFromHash(neighbor_hash);
				if (neighbor_key < 0 || neighbor_key >= PARTICLE_NUM * PARTICLE_NUM) continue;
				
				int start = StartIndices[neighbor_key];
				if (start == -1) continue;
				if (start < 0 || start >= valid_count) continue;
				
				for (int m = start; m < valid_count && SpatialLookup[m].key == neighbor_key; m++) {
					Entity* en_n = &world->entities[SpatialLookup[m].index];
					if (!en_n->is_valid || en_n == en || en_n->arch != arch_particle) continue;

					float32 dist = v2_length(v2_sub(en_n->position, en->position));
					en->density += PARTICLE_MASS * SpikyKernelPow2(dist);
					en->near_density += PARTICLE_MASS * SpikyKernelPow3(dist);
				}
			}
		}
		// // --- LOGGING: AFTER PHASE 3 ---
		// log("PHASE 3: Density = %f, Near Density = %f", p0->density, p0->near_density);


		// Phase 4: Compute pressures
		for (int i = 0; i < MAX_ENTITY_COUNT; i++) {
			Entity* en = &world->entities[i];
			if (en->is_valid && en->arch == arch_particle) {
				en->pressure = ConvertDensityToPressure(en->density);
				en->near_pressure = NEAR_PRESSURE_CONSTANT * en->near_density;
			}
		}
		// --- LOGGING: AFTER PHASE 4 ---
		// log("PHASE 4: Pressure = %f, Near Pressure = %f", p0->pressure, p0->near_pressure);


		// Phase 5: Compute pressure and viscosity forces
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

					float32 dist = v2_length(v2_sub(en->position, en_n->position));
					if (dist == 0.0f) continue;
					Vector2 dir = v2_divf(v2_sub(en_n->position, en->position), dist);

					// // --- LOGGING: INSIDE PHASE 5 NEIGHBOR LOOP (for particle 0) ---
					// if (en->index == 0) {
					// 	log("  - Neighbor %d at dist %f:", en_n->index, dist);
					// }

					// Standard Pressure
					float32 slope = SpikyKernelPow2Derivative(dist);
					float32 sharedP = CalculateSharedPressure(en->pressure, en_n->pressure);
					Vector2 pressure_impulse = v2_mulf(dir, sharedP * slope * PARTICLE_MASS / en_n->density);

					// if (en->index == 0) {
					// 	log("    Std Pressure: sharedP=%f, slope=%f -> impulse=(%f, %f)", sharedP, slope, pressure_impulse.x, pressure_impulse.y);
					// }

					// Near-pressure
					float32 near_slope = SpikyKernelPow3Derivative(dist);
					float32 shared_nearP = (en->near_pressure + en_n->near_pressure) / 2.0f;
					Vector2 near_pressure_impulse = v2_mulf(dir, shared_nearP * near_slope * PARTICLE_MASS / en_n->density);
					
					// if (en->index == 0) {
					// 	log("    Near Pressure: shared_nearP=%f, near_slope=%f -> impulse=(%f, %f)", shared_nearP, near_slope, near_pressure_impulse.x, near_pressure_impulse.y);
					// }

					// Combine Forces
					en->pressure_force = v2_add(en->pressure_force, v2_add(pressure_impulse, near_pressure_impulse));

					// Viscosity
					Vector2 velocity_difference = v2_sub(en_n->velocity, en->velocity);
					float influence = Poly6Kernel(dist); 
					Vector2 viscosity_impulse = v2_mulf(velocity_difference, VISCOSITY_CONSTANT * influence * PARTICLE_MASS / en_n->density);
					en->viscosity_force = v2_add(en->viscosity_force, viscosity_impulse);
				}
			}
		}
		// // --- LOGGING: AFTER PHASE 5 ---
		// log("PHASE 5: Total Pressure Force = (%f, %f), Total Viscosity Force = (%f, %f)", p0->pressure_force.x, p0->pressure_force.y, p0->viscosity_force.x, p0->viscosity_force.y);


		// Phase 6: Integrate, handle collisions, render
		for (int i = 0; i < MAX_ENTITY_COUNT; i++) {
			Entity* en = &world->entities[i];
			if (!en->is_valid) continue;

			switch (en->arch) {
				case arch_particle: {
					if (en->density > 0.0f) {
						Vector2 total_force = v2_add(en->pressure_force, en->viscosity_force);
						en->acceleration = v2_divf(total_force, en->density);
					} else {
						en->acceleration = v2(0.0f, 0.0f);
					}

					// if (i == 0) log("PHASE 6.1: Initial Accel = (%f, %f)", en->acceleration.x, en->acceleration.y);


					if (is_key_down(MOUSE_BUTTON_LEFT)) {
						Vector2 offset = v2_sub(mouse_pos_world, en->position);
						float sqrDst = v2_dot(offset, offset);

						if (sqrDst < INTERACTION_RADIUS * INTERACTION_RADIUS) {
							float dist = sqrtf(sqrDst);
							Vector2 dir_to_mouse = v2(0.0f, 0.0f);
							if (dist > 0.0001f) { 
								dir_to_mouse = v2_divf(offset, dist);
							}
							float centreT = 1.0f - dist / INTERACTION_RADIUS;
							Vector2 interaction_force = v2_sub(v2_mulf(dir_to_mouse, INTERACTION_STRENGTH), en->velocity);
							interaction_force = v2_mulf(interaction_force, centreT);

							if (en->density > 0.0f)
							{
								en->acceleration = v2_add(en->acceleration, v2_divf(interaction_force, en->density));
							}
						}
					}

					if (is_key_down(MOUSE_BUTTON_RIGHT)) {
						// Get the vector from the mouse to the particle
						Vector2 offset = v2_sub(en->position, mouse_pos_world);
						float sqrDst = v2_dot(offset, offset);

						// Check if the particle is within the interaction radius
						if (sqrDst < INTERACTION_RADIUS * INTERACTION_RADIUS) {
							float dist = sqrtf(sqrDst);

							// This is the direction pointing AWAY from the mouse
							Vector2 dir_from_mouse = v2(0.0f, 0.0f);
							if (dist > 0.0001f) {
								dir_from_mouse = v2_divf(offset, dist);
							}

							// The force is strongest at the center and fades to zero at the edge
							float falloff = 1.0f - dist / INTERACTION_RADIUS;
							
							// Calculate the simple outward push force based on the interaction strength
							Vector2 push_force = v2_mulf(dir_from_mouse, INTERACTION_STRENGTH * falloff);

							// Apply the force to the particle's acceleration
							if (en->density > 0.0f) {
								en->acceleration = v2_add(en->acceleration, v2_divf(push_force, en->density));
							}
						}
					}

				en->acceleration = v2_add(en->acceleration, v2(0.0f, -GRAVITY));
				// if (i == 0) log("PHASE 6.2: Final Accel (w/ gravity & mouse) = (%f, %f)", en->acceleration.x, en->acceleration.y);


				en->velocity = v2_add(en->velocity, v2_mulf(en->acceleration, delta_t));
				en->position = v2_add(en->position, v2_mulf(en->velocity, delta_t));
				// if (i == 0) log("PHASE 6.3: Final Velocity = (%f, %f), Final Position = (%f, %f)", en->velocity.x, en->velocity.y, en->position.x, en->position.y);


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

		// Store the original world projection and view
		Matrix4 world_projection = draw_frame.projection;
		Matrix4 world_view = draw_frame.camera_xform;

		// Set up a new projection for screen-space UI (top-left 0,0)
		draw_frame.projection = m4_make_orthographic_projection(0, window.width, 0, window.height, -1, 1);
		// Reset the view matrix for screen-space drawing
		draw_frame.camera_xform = m4_identity();

		// --- TEST 1: Comment out the line below ---
		UI_Draw();

		// Restore the original projection and view for the next frame
		draw_frame.projection = world_projection;
		draw_frame.camera_xform = world_view;

		gfx_update();
		
		// --- LOGGING: END OF FRAME ---
		// This log is outside the main timer to avoid clutter. 
		// You can move it inside the 'seconds_counter' block if you prefer.
		//log("--- FRAME END ---");

		seconds_counter += delta_t;
		frame_count += 1;
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
			
			log("delta_t: %f", delta_t);
			log("fps: %i\n", frame_count);

			if (is_key_down(MOUSE_BUTTON_LEFT)) {
				log("lmb IS PRESSED");
				log("mouse pos: %f, %f", screen_to_world());
			};

			seconds_counter = 0.0;
			frame_count = 0;
		}
	}

	return 0;
}
