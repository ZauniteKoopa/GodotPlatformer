extends Area3D


# Main event handler function for when player enters zone and you entered a new checkpoint
func _on_body_entered(body: Node3D):
	if body is PlatformerPackage3D:
		body.kill()
	pass
