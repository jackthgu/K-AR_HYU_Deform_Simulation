   
require("config")
require("module")
require("common")
require("cameraTracker")
package.path=package.path..";../Samples/FlexibleBody/lua/?.lua" --;"..package.path
function ctor()
	--dbg.draw('Sphere', vector3(0,0,0), "origin", "red", 30)
	this:create("Button", "Draw terrain", "Draw terrain")
	this:updateLayout()

	RE.viewpoint():setClipDistances(0.5, 1000)
	RE.viewpoint():setFOVy(45.000006)
	RE.viewpoint().vpos:assign(vector3(93.776154, 64.218978, 124.498157))
	RE.viewpoint().vat:assign(vector3(0.220461, 20.121055, -5.625024))
	RE.viewpoint():update()

	rootnode =RE.ogreRootSceneNode()
	bgnode=RE.createChildSceneNode(rootnode , "BackgroundNode")

	win:loadIguanaMotion()
	scale_factor=1
	initialPosition=vector3(0,20,0)
	win:createTargetMesh(0, scale_factor, initialPosition);

	win.mTargetMesh:calculateVertexNormal()

	pnode=RE.createChildSceneNode(rootnode, '___')
	meshToEntity=MeshToEntity(win.mTargetMesh, "target_mesh");
	meshToEntity:updatePositionsAndNormals()
	pnode:attachObject(meshToEntity:createEntity("target_mesh_entity"));

	terrain=btRigidBody.createTerrain("../Resource/crowdEditingScripts/terrain/heightmap1_256_256_2bytes.raw", vector3(-200,-50, -200), 256, 256, 800, 800, 50)
	--terrain=btRigidBody.createTerrain("../Resource/crowdEditingScripts/terrain/heightmap1_256_256_2bytes.raw", vector3(0,-50, 0), 256, 256, 800, 800, 50)
	--mIKsolver=IguanaIKSolver(win.mMotion:skeleton(), terrain)
	mIKsolver=IguanaIKSolverLUA(win.mMotion:skeleton(), terrain)
	noParam=vector3N ()
	mOriginalPose=win.mMotion:pose(0)
	local pose=Pose()
	pose:assign(mOriginalPose)
	--mIKsolver:IKsolve(pose, noParam)

	win.mMeshAnimation:setPose(pose)
	win.mMeshAnimation:retrieveAnimatedMesh(win.mTargetMesh);
	meshToEntity:updatePositions()


end

function frameMove(fElapsedTime)
	return 1;
end
function onCallback(w, userData)
	if w:id()=="Draw terrain" then
		local sizeX=100
		local sizeY=100
		local width=800
		local height=800
		local goal=matrixn(sizeX*sizeY,3)
		for i=0, sizeY-1 do
			for j=0, sizeX-1 do
				local x=sop.map(i, 0, sizeX-1, 0, width)
				local z=sop.map(j, 0, sizeY-1, 0, height)
				local out=vector3()
				local normal=vector3()

				terrain:getTerrainHeight( vector3(x,0,z), out, normal)
				goal:row(i*sizeY+j):setVec3(0, out)
			end
		end
		dbg.namedDraw('Traj', goal, 'goal2', 'blueCircle', thickness, 'QuadListZ')
	end
end
function dtor()
	dbg.finalize()
end


function handleRendererEvent(ev, button, x, y)
		--print(ev, x,y,button)
	if ev=="PUSH" then
		local xPos=RE.FltkRenderer():screenToWorldXZPlane(x, y)
		--print("PUSH")
		return 1
	elseif ev=="DRAG" then
		local xPos=RE.FltkRenderer():screenToWorldXZPlane(x, y)
		--print(x,y,button)
		return 1
	elseif ev=="RELEASE" then
		return 1
	elseif ev=="MOVE" then
		local xPos=RE.FltkRenderer():screenToWorldXZPlane(x, y)
		dbg.draw('Sphere', xPos, "cursor", "red", 3)

		local out=vector3()
		local normal=vector3()
		terrain:getTerrainHeight( xPos, out, normal)
		dbg.draw('Sphere', out, "cursor_on_terrain", "bullet",5)
		--print(x,y,button)
		local pose=Pose()
		pose:assign(mOriginalPose)
		pose.translations(0).x=out.x
		pose.translations(0).z=out.z
		mIKsolver:IKsolve(pose, noParam)

		win.mMeshAnimation:setPose(pose)
		win.mMeshAnimation:retrieveAnimatedMesh(win.mTargetMesh);
		meshToEntity:updatePositions()
		return 1
	end
	return 0
end
if true then
function btRigidBody:getTerrainHeight2(pos)
	local out=vector3()
	local normal=vector3()
	self:getTerrainHeight(pos, out, normal)
	return out.y
end
end
require("test/IguanaIKSolverLua")
