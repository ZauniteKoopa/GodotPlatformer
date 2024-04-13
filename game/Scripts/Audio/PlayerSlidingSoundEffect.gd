extends AudioStreamPlayer

@export var playerPlatformer: PlatformerPackage3D
var prevSpeakerActivated = false


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	# check player variables
	var isPlayerWallSliding = playerPlatformer.is_wall_grabbing()
	var isPlayerSkidding = playerPlatformer.is_grounded() && playerPlatformer.is_skidding()
	var shouldSpeakerBeActive = isPlayerWallSliding or isPlayerSkidding
	var speakerStateChange = shouldSpeakerBeActive != prevSpeakerActivated
	
	# If indicated speaker state change, play or stop the sound effect appropriately
	if (speakerStateChange):
		if (shouldSpeakerBeActive):
			play()
		else:
			stop()
		pass
		
	# Update prevSpeakerActivated
	prevSpeakerActivated = shouldSpeakerBeActive
	pass
