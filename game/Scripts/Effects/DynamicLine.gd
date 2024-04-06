extends MeshInstance3D
@export var spotShadow: SpotShadow

# Called when the node enters the scene tree for the first time.
func _ready():
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	var curSpotShadowDistance = spotShadow.get_hit_length()
	position = curSpotShadowDistance / 2 * Vector3.BACK
	
	var cylinderMesh = mesh as CylinderMesh
	cylinderMesh.height = curSpotShadowDistance
	pass
