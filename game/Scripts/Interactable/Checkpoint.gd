extends Area3D

static var main_checkpoint = null
var mainPlayer: PlatformerPackage3D
@export var flagMesh: MeshInstance3D
@export var activeColor: Color
@export var inactiveColor: Color

signal checkpoint_obtained


# Called when the node enters the scene tree for the first time: disable color
func _ready():
	flagMesh.get_active_material(0).albedo_color = inactiveColor
	pass 
	
# Main event handler function for when player enters zone and you entered a new checkpoint
func _on_body_entered(body: Node3D):
	if body is PlatformerPackage3D and main_checkpoint != self:
		# if this isn't your first checkpoint, deactivate the previous main checkpoint
		if main_checkpoint != null:
			main_checkpoint._deactivate()
		
		# activate
		_activate(body)
	pass

# Main function to activate checkpoint
func _activate(player: PlatformerPackage3D):
	# set flag to active and listen to player 
	mainPlayer = player
	mainPlayer.respawned.connect(_respawn)
	flagMesh.get_surface_override_material(0).albedo_color = activeColor
	main_checkpoint = self
	
	emit_signal("checkpoint_obtained")
	pass

# Main function to deactivate checkpoint
func _deactivate():
	# set flag to active and listen to player 
	flagMesh.get_surface_override_material(0).albedo_color = inactiveColor
	if mainPlayer != null:
		mainPlayer.respawned.disconnect(_respawn)
	
	main_checkpoint = null
	pass

# Main function to respawn
func _respawn():
	mainPlayer.global_position = global_position + (1.5 * Vector3.UP)
	pass
