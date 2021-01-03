matchPose=true
performIK=true
useGraph=false
--background="terrain"
background="plane"
showTargetMesh=false
	

--[[
   아래 부분의 코드가 구현에 해당한다. 윗부분을 바꿔서 다른 시나리오를 수행할 수 있다.
]]--
scenario="original_scale"

--option="explicit"
option="implicit"
--option="semi-implicit"


--RE.viewpoint():setFOVy(44.999999)

--rootnode =RE.ogreRootSceneNode()
--bgnode=RE.createChildSceneNode(rootnode , "BackgroundNode")

if scenario=="original_scale" then
	scale_factor=1

	-- 500kg, 80 cm

	--RE.viewpoint():setClipDistances(0.5, 1000)
	--RE.viewpoint():setFOVy(45.000002)
	--RE.viewpoint().vpos:assign({54.725145, 3.286307, 74.032226})
	--RE.viewpoint().vat:assign({1.288031, 0.837722, 0.142416})
	--RE.viewpoint():update()


	-- 1000cm/s^2 = -10 m/s^2
	gravity=-1000
	mass=5000
	bendingConstraints1=4

	stiffness=5000
	bendingStiffness=stiffness*0.8
	integrateMethod=1
	simulationFrameRate=240
	springDamp=100
	kDF=200	-- dynamic friction coefficient
	kDFvelThr=0.01	
	if background=="terrain" then
		initialPosition=vector3(0,-30,0)
	else
		initialPosition=vector3(0,60,0)
		--print(initialPosition)	
		initialPosition2=Vector3(1,1,1)
		--print(initialPosition2)
		initialPosition2:assign({11,11,11})
		--print(initialPosition2)
		--print(initialPosition2.x)
	end

	if background=="terrain" then
		win:createTerrain("../Resource/crowdEditingScripts/terrain/heightmap1_256_256_2bytes.raw", vector3(-200,-50, -200), 256, 256, 800, 800, 30)
	else
		win:createFloor(-15, vector3(200, 15, 200))
	end

	
	if useGraph == true then
		win:setTRCwinTerrain();
		local w=trcwin:findWidget("Synthesis")
		w:checkButtonValue(1)
		trcwin:callCallbackFunction(w)

		if showTargetMesh == false then
			win:setSkeletonVisible(false)
		else
			win:setSkeletonVisible(true)
		end

	end

	
elseif scenario=="reduced_scale" then 
	scale_factor=0.1

	--단위: dm=10cm

	-- 500kg, 8 dm

	RE.viewpoint():setClipDistances(0.5, 1000)
	RE.viewpoint().vpos:assign(vector3(-5.3399359, 0.6480593, 3.6277348))
	RE.viewpoint().vat:assign(vector3(0, 0.6480593, 0))
	RE.viewpoint():setScale(30)
	RE.viewpoint():update()

	-- 100dm/s^2 = -10 m/s^2
	gravity=-1000
	mass=500
	bendingConstraints1=3

	if option=="implicit" then
		stiffness=10000	
		bendingStiffness=stiffness*0.8
		integrateMethod=1
		simulationFrameRate=240
		springDamp=100
	elseif option=="semi-implicit" then
		gravity=-100
		stiffness=10000
		bendingStiffness=8000
		integrateMethod=2
		simulationFrameRate=2400
		springDamp=40
	elseif option=="explicit" then
		stiffness=10000	
		bendingStiffness=8000
		integrateMethod=0
		simulationFrameRate=2400
		springDamp=30
	end

	win:createFloor(-15, vector3(200, 15, 200))
	--win:createTerrain("../Resource/crowdEditingScripts/terrain/heightmap1_256_256_2bytes.raw", vector3(-200,-20, -200), 256, 256, 400, 400, 50)
end
