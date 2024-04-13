extends AudioStreamPlayer3D

@export var playerPackage: PlatformerPackage3D
@export var basePitch: float = 0.75
@export var pitchScaling: float = 0.5

func _on_jump_started():
	pitch_scale = basePitch + (playerPackage.get_current_ground_jump_number() * pitchScaling)
	play()
	pass
