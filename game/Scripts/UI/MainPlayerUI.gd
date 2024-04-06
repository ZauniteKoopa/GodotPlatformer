extends Control

@export var colorScreen: ColorRect

# Called when running black screen functionality
func _transition_color_screen(targetColor: Color, transitionDuration: float):
	var timer: float = 0
	var srcColor: Color = colorScreen.color
	
	while timer < transitionDuration:
		await get_tree().process_frame
		timer += get_process_delta_time()
		colorScreen.color = srcColor.lerp(targetColor, timer / transitionDuration)
		pass
		
	colorScreen.color = targetColor
	pass
