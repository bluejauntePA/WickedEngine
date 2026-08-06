#pragma once

class Translator
{
private:
	bool dragging = false;
	bool dragStarted = false;
	bool dragEnded = false;
	XMFLOAT3 intersection_start = XMFLOAT3(0, 0, 0);
	XMFLOAT3 axis = XMFLOAT3(1, 0, 0);
	XMFLOAT4 drag_plane = XMFLOAT4(0, 0, 0, 0);
	float angle = 0;
	float angle_start = 0;
	bool has_selected_transform = false;
	wi::vector<uint64_t> temp_filters;
	wi::vector<wi::ecs::Entity> selectedWithHierarchy; // all the selected entities and their descendants
public:

	void Update(const wi::scene::CameraComponent& camera, const XMFLOAT4& currentMouse, const wi::Canvas& canvas);
	void Draw(const wi::scene::CameraComponent& camera, const XMFLOAT4& currentMouse, wi::graphics::CommandList cmd) const;

	// Attach selection to translator temporarily
	void PreTranslate();
	// Apply translator to selection
	void PostTranslate() const;

	wi::scene::Scene* scene = nullptr;
	wi::scene::TransformComponent transform;
	wi::vector<wi::scene::PickResult> selected; // all the selected picks
	wi::unordered_set<wi::ecs::Entity> selectedEntitiesLookup; // fast lookup for selected entities
	wi::vector<wi::ecs::Entity> selectedEntitiesNonRecursive; // selected entities that don't contain entities that would be included in recursive iterations

	float scale_snap = 1;
	float rotate_snap = XM_PIDIV4;
	float translate_snap = 1;
	float tool_opacity = 1;
	float tool_darken_negative_axes = 1;
	float tool_scale = 1.0f;
	float tool_thickness = 1.0f;
	bool tool_mirror_axes_to_camera = true;
	bool tool_axis_text_in_local_space = false;
	XMFLOAT3 tool_world_axis_sign = XMFLOAT3(1, 1, 1);
	bool tool_axis_x_enabled = true;
	bool tool_axis_y_enabled = true;
	bool tool_axis_z_enabled = true;
	// Keep visual transforms current while another UI surface owns input.
	bool tool_interaction_enabled = true;
	// Optional Blender-style view trackball, picked from a compact center target.
	bool tool_trackball_enabled = false;
	float tool_trackball_radius = 0.65f;
	float tool_trackball_sensitivity = 0.01f;
	// Optional projected plane-handle picking. This keeps thin, edge-on plane
	// handles selectable and only lets them override the origin where visible.
	bool tool_use_screen_space_plane_picking = false;
	// Optional rotation-ring presentation for compact posing tools. The front
	// hemisphere test is camera-relative, and tube sides give edge-on rings a
	// real silhouette instead of allowing a flat annulus to collapse to a line.
	bool tool_rotation_front_facing_only = false;
	uint32_t tool_rotation_ring_tube_sides = 0;
	// Optional independent world-space rotation axes. This is intentionally
	// rotator-only so the default translator/scalator behavior is unchanged.
	bool tool_use_independent_rotation_axes = false;
	XMFLOAT3 tool_rotation_axis_x = XMFLOAT3(1, 0, 0);
	XMFLOAT3 tool_rotation_axis_y = XMFLOAT3(0, 1, 0);
	XMFLOAT3 tool_rotation_axis_z = XMFLOAT3(0, 0, 1);

	enum TRANSLATOR_STATE
	{
		TRANSLATOR_IDLE,
		TRANSLATOR_X,
		TRANSLATOR_Y,
		TRANSLATOR_Z,
		TRANSLATOR_XY,
		TRANSLATOR_XZ,
		TRANSLATOR_YZ,
		TRANSLATOR_TRACKBALL,
		TRANSLATOR_XYZ,
	} state = TRANSLATOR_IDLE;

	// Current Blender-style trackball values, exposed for application feedback
	// and for consumers that solve rotations outside this helper.
	XMFLOAT2 trackball_delta_radians = XMFLOAT2(0, 0);
	XMFLOAT3 trackball_axis = XMFLOAT3(1, 0, 0);
	float trackball_angle = 0.0f;

	XMMATRIX GetMirrorMatrix(TRANSLATOR_STATE state, const wi::scene::CameraComponent& camera) const;
	void WriteAxisText(TRANSLATOR_STATE axis, const wi::scene::CameraComponent& camera, char* text) const;

	float dist = 1;
	XMFLOAT2 trackball_mouse_start = XMFLOAT2(0, 0);

	bool isTranslator = true;
	bool isScalator = false;
	bool isRotator = false;
	bool isLocalSpace = false;
	bool is2D = false;
	bool IsEnabled() const { return isTranslator || isRotator || isScalator; }
	XMMATRIX GetLocalRotation() const;
	void SetEnabled(bool value)
	{
		if (value && !IsEnabled())
		{
			isTranslator = true;
		}
		else if (!value && IsEnabled())
		{
			isTranslator = false;
			isScalator = false;
			isRotator = false;
		}
	}


	// Check if the drag started in this exact frame
	bool IsDragStarted() const { return dragStarted; };
	// Check if the drag ended in this exact frame
	bool IsDragEnded() const { return dragEnded; };

	bool IsInteracting() const { return state != TRANSLATOR_IDLE; }

	wi::scene::TransformComponent transform_start;
	wi::vector<XMFLOAT4X4> matrices_start;
	wi::vector<XMFLOAT4X4> matrices_current;
};

