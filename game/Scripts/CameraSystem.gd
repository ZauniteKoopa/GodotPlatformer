extends Node3D

const MOUSE_SENSITIVITY = 0.003;
const X_ROTATION_LIMIT = 45.0;

# On game ready
func _ready():
	Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)

# main input function to handle mouse events
func _input(event):
	if event is InputEventMouseMotion:
		var mouseDelta = event.relative
		
		# horizontal rotation
		var yEulerAngle = transform.basis.get_euler().y + (MOUSE_SENSITIVITY * -mouseDelta.x)
		
		# vertical rotation
		var xEulerAngle = transform.basis.get_euler().x + (MOUSE_SENSITIVITY * -mouseDelta.y)
		xEulerAngle = clamp(xEulerAngle, deg_to_rad(-X_ROTATION_LIMIT), deg_to_rad(X_ROTATION_LIMIT))
		
		# apply rotation
		transform.basis = transform.basis.from_euler(Vector3(xEulerAngle, yEulerAngle, 0))
		pass
	pass
