extends Interactable

	
func interact_behavior():
	print("INTERACT")
	
	# when destroying an object, MUST wait a frame before truly deleting object
	emit_signal("destroyed", self)
	await get_tree().process_frame
	queue_free()
	pass

