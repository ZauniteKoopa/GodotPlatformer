extends PlatformerPackage3D

# Get the gravity from the project settings to be synced with RigidBody nodes.
var gravity = ProjectSettings.get_setting("physics/3d/default_gravity")


func _process(delta):
	# Get the input direction and handle the movement/deceleration.
	# As good practice, you should replace UI actions with custom gameplay actions.
	var input_dir = Input.get_vector(
		"move_left",
		"move_right",
		"move_backwards",
		"move_forward"
	)
	
	relative_run(input_dir, delta);
	
	if (Input.is_action_just_pressed("Jump")):
		start_jump()
	if (Input.is_action_just_released("Jump")):
		cancel_jump()
