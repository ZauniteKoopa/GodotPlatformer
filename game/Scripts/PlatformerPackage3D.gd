extends PlatformerPackage3D

# Get animation tree
@export var mainPlayerUi: Control
@export var deathBlackoutTransitionTime: float
@export var deathBlackoutDurationTime: float
@export var reviveScreenTransitionTime: float
@onready var inputEnabled: bool = true
@onready var animation_tree: AnimationTree = $CollisionShape3D/zephAnimated/AnimationTree
@onready var animation_player: AnimationPlayer = $CollisionShape3D/zephAnimated/AnimationPlayer

func _process(delta):
	process_timers(delta)
	
	# Only go in here for inputEnabled
	if inputEnabled == true:
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
		pass
		
	update_animation_parameters(delta)
	pass
		

func update_animation_parameters(delta: float):
	var isMoving = get_current_horizontal_speed() > 0.01
	var isJumpingUp = get_current_vertical_speed() > 0
	var doingOverridingAction = is_ledge_grabbing() or is_wall_grabbing() or is_dashing()
	var skidDashing = is_skidding() && is_dashing() && is_grounded()
	
	# Set animation transition variables
	animation_tree["parameters/conditions/idle"] = is_grounded() and !isMoving and !doingOverridingAction
	animation_tree["parameters/conditions/moving"] = is_grounded() and isMoving and !is_skidding() and !doingOverridingAction
	animation_tree["parameters/conditions/skidding"] = is_grounded() and isMoving and is_skidding() and (!doingOverridingAction or skidDashing)
	animation_tree["parameters/conditions/jumping"] = !is_grounded() and isJumpingUp and !doingOverridingAction
	animation_tree["parameters/conditions/falling"] = !is_grounded() and !isJumpingUp and !doingOverridingAction
	animation_tree["parameters/conditions/wallGrabbing"] = is_wall_grabbing()
	animation_tree["parameters/conditions/ledgeGrabbing"] = is_ledge_grabbing()
	animation_tree["parameters/conditions/dashing"] = is_dashing() and !skidDashing
	
	# Set animation speed
	animation_tree["parameters/WalkTree/TimeScale/scale"] = get_current_walking_animation_speed()
	pass


func _on_death():
	# black out immediately
	inputEnabled = false
	mainPlayerUi._transition_color_screen(Color.BLACK, deathBlackoutTransitionTime)
	await get_tree().create_timer(deathBlackoutTransitionTime).timeout
	
	# Teleport to spawn point during blackout and wait out blackout duration
	respawn()
	await get_tree().create_timer(deathBlackoutDurationTime).timeout
	
	# wake up
	mainPlayerUi._transition_color_screen(Color(0, 0, 0, 0), deathBlackoutTransitionTime)
	await get_tree().create_timer(deathBlackoutTransitionTime).timeout
	inputEnabled = true
	
	pass # Replace with function body.
