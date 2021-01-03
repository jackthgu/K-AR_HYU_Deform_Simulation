return {
		skeleton="../Resource/motion/muaythai1.wrl",
		motion="../Resource/motion/muaythai1.dof",
		toes={'LEFTANKLE', 'RIGHTANKLE'},
		toes_lpos={ vector3(0,-0.09,0.13), vector3(0,-0.09,0.13)},
		ankles={'LEFTANKLE', 'RIGHTANKLE'},
		ankles_lpos={vector3(0,-0.10,0), vector3(0,-0.10,0)},
		filter_size=5,
		thr_speed=0.125, -- speed limit
		toelimit={ 0.002, 0.002}, -- toe height threshhold, (L, R) the lower limit, more frames are unconstrained
		anklelimit = {0.002, 0.002},
	}
