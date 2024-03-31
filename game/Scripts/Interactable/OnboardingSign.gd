extends Interactable

@export var signUI: Control
var isActive: bool

func _process(delta):
	if (isActive and Input.is_action_just_pressed("interact")):
		interact_end();
		pass
	
	pass

func on_interaction_sequence_start():
	get_tree().paused = true
	signUI.show()
	await get_tree().process_frame
	
	isActive = true;
	pass


func interact_end():
	isActive = false;
	signUI.hide()
	get_tree().paused = false
	
	await get_tree().process_frame
	emit_signal("interaction_sequence_end")
	
