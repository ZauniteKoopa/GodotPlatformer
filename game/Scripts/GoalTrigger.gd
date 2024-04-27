extends Area3D
@export var winMenu: Control
@export var victorySound: AudioStreamPlayer

# Main event handler function for when player enters zone and you entered a new checkpoint
func _on_body_entered(body: Node3D):
	if body is PlatformerPackage3D:
		victorySound.play()
		Input.set_mouse_mode(Input.MOUSE_MODE_VISIBLE)
		get_tree().paused = true
		winMenu.show()
	pass

# Event handler for when button was pressed
func _on_button_pressed():
	Input.set_mouse_mode(Input.MOUSE_MODE_VISIBLE)
	get_tree().paused = false
	get_tree().change_scene_to_file("res://MainLevels/MainMenu.tscn")
	pass
	
