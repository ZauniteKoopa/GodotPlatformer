extends PlatformerPackage3D

# Get the gravity from the project settings to be synced with RigidBody nodes.
var gravity = ProjectSettings.get_setting("physics/3d/default_gravity")

# Get animation tree
@onready var animation_tree: AnimationTree = $CollisionShape3D/zephAnimated/AnimationTree

func _process(delta):
	process_timers(delta)
		
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
		
	if (Input.is_action_just_pressed("dash")):
		dash()
		
	update_animation_parameters()
	pass
		

func update_animation_parameters():
	var isMoving = get_current_horizontal_speed() > 0.01
	var isJumpingUp = get_current_vertical_speed() > 0
	var doingOverridingAction = is_ledge_grabbing() or is_wall_grabbing() or is_dashing()
	var skidDashing = is_skidding() && is_dashing() && is_grounded()
	
	animation_tree["parameters/conditions/idle"] = is_grounded() and !isMoving and !doingOverridingAction
	animation_tree["parameters/conditions/moving"] = is_grounded() and isMoving and !is_skidding() and !doingOverridingAction
	animation_tree["parameters/conditions/skidding"] = is_grounded() and isMoving and is_skidding() and (!doingOverridingAction or skidDashing)
	animation_tree["parameters/conditions/jumping"] = !is_grounded() and isJumpingUp and !doingOverridingAction
	animation_tree["parameters/conditions/falling"] = !is_grounded() and !isJumpingUp and !doingOverridingAction
	animation_tree["parameters/conditions/wallGrabbing"] = is_wall_grabbing()
	animation_tree["parameters/conditions/ledgeGrabbing"] = is_ledge_grabbing()
	animation_tree["parameters/conditions/dashing"] = is_dashing() and !skidDashing
	pass


func _on_death():
	respawn()
	pass # Replace with function body.
