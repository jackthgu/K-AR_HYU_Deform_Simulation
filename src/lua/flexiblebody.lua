RE.viewpoint():setFOVy(44.999999)

rootnode =RE.ogreRootSceneNode()
bgnode=RE.createChildSceneNode(rootnode , "BackgroundNode")


gravity=-10

option=2
if option==1 then
	-- 5kg, 80cm
	RE.viewpoint():setClipDistances(0.5, 1000)
	RE.viewpoint().vpos:assign(vector3(-1.105151, 1.513821, 0.219561))
	RE.viewpoint().vat:assign(vector3(0, -1.6480593, 0))
	RE.viewpoint():setScale(30)
	RE.viewpoint():update()

	mass=10
	scale_factor=0.01 
	kDF=50
	bendingConstraints2=0.0005
	simulationFrameRate=9000
	win:createFloor(-2, vector3(200,1.5,200))
	air_density=120
else
	-- 50kg, 8m

	RE.viewpoint():setClipDistances(0.5, 1000)
	RE.viewpoint().vpos:assign(vector3(-5.3399359, 0.6480593, 3.6277348))
	RE.viewpoint().vat:assign(vector3(0, -10.6480593, 0))
	RE.viewpoint():setScale(30)
	RE.viewpoint():update()

	gravity=-10
	mass=500
	scale_factor=0.1
	kDF=0.5
	bendingConstraints2=0.5
	simulationFrameRate=60
	air_density=1.2

	if win then
		--win:createFloor(-15, vector3(200, 15, 200))
		--usage: createTerrain(filename, translation, image_sizeX, image_sizeY, world_sizeX, world_sizeY, heightMax)
		win:createTerrain("../Resource/crowdEditingScripts/terrain/heightmap1_256_256_2bytes.raw", vector3(-200,-20, -200), 256, 256, 400, 400, 50)
	end
end
bendingConstraints1=2
iterations=10
