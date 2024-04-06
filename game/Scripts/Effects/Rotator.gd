extends StaticBody3D
@export var angular_rotate_speed = 3;


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	rotate_y(angular_rotate_speed * delta)
	pass
