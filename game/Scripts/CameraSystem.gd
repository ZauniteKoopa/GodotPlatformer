extends DynamicCameraPivot

const MOUSE_SENSITIVITY = 0.003;
const X_ROTATION_LIMIT = 45.0;

# On game ready
func _ready():
	Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)
	pass

# main input function to handle mouse events
func _input(event):
	if event is InputEventMouseMotion:
		var mouseDelta = event.relative
		
		# horizontal rotation
		var deltaY = (MOUSE_SENSITIVITY * -mouseDelta.x)
	
		# vertical rotation
		var deltaX = (MOUSE_SENSITIVITY * -mouseDelta.y)
		
		rotate_camera(deltaX, deltaY);
		pass
	pass
