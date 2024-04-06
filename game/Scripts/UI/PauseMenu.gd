extends Control

var isPaused = false

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	if (Input.is_action_just_pressed("pause")):
		if(isPaused):
			unpause()
			pass
		elif (!get_tree().paused):
			pause()
			pass
		pass
	pass


func pause():
	isPaused = true
	show()
	get_tree().paused = true
	Input.set_mouse_mode(Input.MOUSE_MODE_VISIBLE)


func unpause():
	isPaused = false
	hide()
	get_tree().paused = false
	Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)


func _on_back_to_game_pressed():
	unpause()
	pass # Replace with function body.


func _on_main_menu_pressed():
	get_tree().paused = false
	get_tree().change_scene_to_file("res://MainLevels/MainMenu.tscn")
	pass # Replace with function body.
