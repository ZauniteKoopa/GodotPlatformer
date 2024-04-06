extends TextureRect

@export var startingScale: float = 0.75
@export var endingScale: float = 4
@export var platformerPackage: PlatformerPackage3D


#Called when user dashes
func on_dash_started():
	# set up timer and show
	var timer: float = 0
	var scalingDuration: float = platformerPackage.get_dash_distance() / platformerPackage.get_dash_speed()
	scale = Vector2(startingScale, startingScale)
	show()
	
	while (timer < scalingDuration):
		await get_tree().process_frame
		timer += get_process_delta_time()
		
		var scaleValue: float = lerpf(startingScale, endingScale, timer / scalingDuration)
		scale = Vector2(scaleValue, scaleValue)
		
		pass
	
	hide()
	
	
