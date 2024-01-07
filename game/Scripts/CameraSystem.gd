extends Node3D

const ROTATION_SPEED = 2;

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	var cameraRotationAxis = Input.get_axis(
		"rotate_camera_left",
		"rotated_camera_right"
	)
	
	rotate(Vector3.UP, ROTATION_SPEED * cameraRotationAxis * delta)
	pass
