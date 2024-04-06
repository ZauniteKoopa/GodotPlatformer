extends GPUParticles3D

@export var platformerPackage: PlatformerPackage3D;


func on_dashed():
	if (!platformerPackage.is_grounded()):
		emitting = true
		pass
	pass
	
func on_landed():
	if (emitting):
		emitting = false
		pass
	pass
