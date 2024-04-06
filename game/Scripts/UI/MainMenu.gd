extends Control

@onready var howToPlayScreen: Node = get_node("MainMenuBackground/HowToPlayScreen")


func _on_play_pressed():
	get_tree().change_scene_to_file("res://MainLevels/TestLab.tscn")
	pass # Replace with function body.


func _on_how_to_play_pressed():
	howToPlayScreen.show()
	pass # Replace with function body.


func _on_exit_pressed():
	get_tree().quit()
	pass # Replace with function body.


func _on_how_to_play_back_pressed():
	howToPlayScreen.hide()
	pass # Replace with function body.
