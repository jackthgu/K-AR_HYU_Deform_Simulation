   
require("config")
require("module")
require("common")
require("cameraTracker")
function ctor()
	this:create("Button", "applyForce", "applyForce")
	this:updateLayout()

	usePenaltyMethod=true;
	renderingFrameRate=30
	skipFrame=20
	simulationFrameRate=600

	if usePenaltyMethod then
		simulationFrameRate=2000
	end

	local step=1.0/simulationFrameRate;
	win:startWorld(step)
	background="terrain"
	--background="plane"

	RE.viewpoint():setFOVy(44.999999)

	rootnode =RE.ogreRootSceneNode()
	bgnode=RE.createChildSceneNode(rootnode , "BackgroundNode")

	scale_factor=0.01 -- mesh is in cm scale, but simulation is done using meter unit.

	RE.viewpoint():setFOVy(45.000004)
	RE.viewpoint():setZoom(1.000000)
	RE.viewpoint().vpos:set(118.003034, 103.180620, 262.343986)
	RE.viewpoint().vat:set(-17.388127, -40.321265, 54.982566)
	RE.viewpoint():update()


	gravity=-9.8
	stiffness=4000
	springDamp=40
	initialHeight=0.2
	RE.motionPanel():motionWin():playFrom(0)
	if true then
		-- much softer
		initialHeight=0.2
		stiffness=100
		springDamp=4
		world:setParameter("drawSimulatedMeshOffset",0.005) -- move upward so that the mesh does not penetrate the terrain.
		--simulationFrameRate=1200
	end

	world:setUsePenaltyMethod(usePenaltyMethod)
	world:setPenaltyStiffness(70000)

	dynamicFrictionCoef=1
	--dynamicFrictionCoef=10.3
	world:setParameter("mass", 4)
	world:setParameter("bendingConstraints1", 4)
	world:setParameter("stiffness", stiffness)
	world:setParameter("bendingStiffness", stiffness*0.5)

	-- INTEGRATE_EXPLICIT	:		0
	-- INTEGRATE_IMPLICIT	:		1
	-- INTEGRATE_SEMI_IMPLICIT:		2
	world:setParameter("integrateMethod",1) -- implicit
	world:setParameter("springDamp",springDamp)
	world:setParameter("kDF", dynamicFrictionCoef)	-- dynamic friction coefficient
	world:setParameter("kDFvelThr", 0.01)
	world:setParameter("debugDrawWorld",0)
	world:setParameter("drawSimulatedMesh",1)
	world:setGVector(vector3(0, gravity, 0))

	if background=="terrain" then
		-- before scaling, so the division is necessary.
		initialPosition=vector3(0,initialHeight/scale_factor,0)
	else
		initialPosition=vector3(0,3/scale_factor,0)
	end

	win:createTargetMesh(0, 1, initialPosition);

	function scaleTargetMesh(scale_factor)
		local min=vector3(1000,1000,1000)
		local max=vector3(-1000,-1000,-1000)
		for i=0, win.mTargetMesh:numVertex()-1 do
			local v=win.mTargetMesh:getVertex(i)
			v:assign(v*scale_factor)

			if v.x<min.x then
				min.x=v.x
			end
			if v.y<min.y then
				min.y=v.y
			end
			if v.z<min.z then
				min.z=v.z
			end
			if v.x>max.x then
				max.x=v.x
			end
			if v.y>max.y then
				max.y=v.y
			end
			if v.z>max.z then
				max.z=v.z
			end
		end
		return min, max
	end
	local m,M=scaleTargetMesh(scale_factor)

	if background=="terrain" then
		world:createTerrain("../Resource/crowdEditingScripts/terrain/heightmap1_256_256_2bytes.raw", vector3(-2,0, -2), 256, 256, 8, 8, 0.3)
		--world:createTerrain("../Resource/crowdEditingScripts/terrain/heightmap1_256_256_2bytes.raw", vector3(-2,0, -2), 256, 256, 8, 8, 0) -- flat terrain
	else
		world:createFloor(-15, vector3(400, 30, 400))
	end
	if false then
		-- default
		world:createSoftBody(win.mTargetMesh)
	else

		local mesh=win.mTargetMesh
		mEdges=EdgeConnectivity(mesh)
		local springs=intmatrixn()
		springs:resize(mEdges:numEdges(), 2)
		for i=0,mEdges:numEdges()-1 do
			local si=mEdges:source(i)
			local ti=mEdges:target(i)
			springs:set(i, 0, si)
			springs:set(i, 1, ti)
		end

		local thr=13
		for i=0, mesh:numVertex()-1 do
			for j=i+1, mesh:numVertex()-1 do
				if mesh:getVertex(i):distance(mesh:getVertex(j))<thr then
					if not mEdges:isConnected(i,j) then
						springs:pushBack(CT.ivec(i,j))
					end
				end
			end
		end
		local vstiff=vectorn(springs:rows())
		vstiff:range(0, mEdges:numEdges()):setAllValue(stiffness)
		vstiff:range(mEdges:numEdges(), vstiff:size()):setAllValue(stiffness*0.5)
		world:createSoftBody(win.mTargetMesh, springs, vstiff)
	end

	--	debug.debug()


	local param=vectorn ()
	param:setValues(0.5,0.5)
	world:registerAllCollisionCheckPairs(0, 1, param)

	world:init(1.0/simulationFrameRate, Physics.DynamicsSimulator.EULER)

	world:setParameter("drawContactForce", 1)
	world:setParameter("contactMethod",0)
	world:setParameter("penaltyStiffness", 100000)
	world:setParameter("penaltyDampness", 1000)
	world:setParameter("penaltyMuScale", 2)
	--world:setParameter("contactMethod",SimulatorImplicit.LCP_METHOD_FRICTIONLESS)
	--world:setParameter("contactMethod",SimulatorImplicit.LCP_METHOD)
	--world:setParameter("contactMethod",SimulatorImplicit.BARAFF98)

	if true then
		local resourcePath="../src/lua/"
		world:softbody(1):adjustSoftBodyParam("dynamicFrictionCoef", resourcePath.."iguanaAll.selection", 0)
		--world:softbody(0):adjustSoftBodyParam("dynamicFrictionCoef", resourcePath.."iguanaTail.selection", 0)
		world:softbody(1):adjustSoftBodyParam("dynamicFrictionCoef", resourcePath.."leftFoot.selection", dynamicFrictionCoef)
		world:softbody(1):adjustSoftBodyParam("dynamicFrictionCoef", resourcePath.."rightFoot.selection", dynamicFrictionCoef)
		world:softbody(1):adjustSoftBodyParam("dynamicFrictionCoef", resourcePath.."leftHand.selection", dynamicFrictionCoef)
		world:softbody(1):adjustSoftBodyParam("dynamicFrictionCoef", resourcePath.."rightHand.selection", dynamicFrictionCoef)

		world:softbody(1):adjustSoftBodyParam("stiffness", resourcePath.."leftFoot.selection", stiffness*2)
		world:softbody(1):adjustSoftBodyParam("stiffness", resourcePath.."rightFoot.selection", stiffness*2)
		world:softbody(1):adjustSoftBodyParam("stiffness", resourcePath.."leftHand.selection", stiffness*2)
		world:softbody(1):adjustSoftBodyParam("stiffness", resourcePath.."rightHand.selection", stiffness*2)
	end
end

function frameMove(fElapsedTime)

	local mSimulator=world
	if true then

		local nSubStep=math.round(simulationFrameRate/renderingFrameRate);
		local step=1.0/simulationFrameRate;

		if(win.mTargetMesh:numVertex()) then
			local curFrame=RE.motionPanel():motionWin():getCurrFrame();
			win.mMeshAnimation:setPose(win.mMotion:pose(curFrame));
			win.mMeshAnimation:retrieveAnimatedMesh(win.mTargetMesh);
			scaleTargetMesh(scale_factor)

			mSimulator:softbody(1):changeSoftBodyRestLength(win.mTargetMesh)
			for i=0,nSubStep-1 do 
				if g_forces then
					world:softbody(1):addExternalForce(g_forces[1], g_forces[2])
					g_forces[3]=g_forces[3]+1
				end
				mSimulator:stepSimulation();
				if g_forces then 

					local mSimulatedMesh=world:softbody(1).mSimulatedMesh;
					local pos=mSimulatedMesh:getVertex(g_forces[1](0))*100

					local f=g_forces[2](0)*0.05
					dbg.draw('Arrow',  pos-f, pos, "force", 4)
					if g_forces[3]>simulationFrameRate*0.1 then
						-- 0.2s
						g_forces=nil
					end
				else
					dbg.erase('Arrow', 'force')
				end
			end
		end
	end
	mSimulator:renderme(); 

	return 1;
end
function onCallback(w, userData)

	if w:id()=='applyForce' then
		g_forces={
			CT.ivec(5,20,4,1,7),
			--CT.ivec(76), -- vertex 76
			vector3N(5), 
			0}
		local f=vector3(-20,0, -20)
		g_forces[2](0):assign(f)
		g_forces[2](1):assign(f*0.7)
		g_forces[2](2):assign(f*0.7)
		g_forces[2](3):assign(f*0.7)
		g_forces[2](4):assign(f*0.7)
	end
end
function dtor()
	dbg.finalize()
end
