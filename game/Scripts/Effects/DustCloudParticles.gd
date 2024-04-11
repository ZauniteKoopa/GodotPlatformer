extends GPUParticles3D

@export var playerPlatformer: PlatformerPackage3D

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	emitting = playerPlatformer.is_wall_grabbing()
	pass
