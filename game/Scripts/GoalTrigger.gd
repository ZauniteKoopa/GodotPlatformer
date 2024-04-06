extends Area3D



# Main event handler function for when player enters zone and you entered a new checkpoint
func _on_body_entered(body: Node3D):
	if body is PlatformerPackage3D:
		Input.set_mouse_mode(Input.MOUSE_MODE_VISIBLE)
		get_tree().change_scene_to_file("res://MainLevels/MainMenu.tscn")
	pass
