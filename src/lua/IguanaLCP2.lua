require("config")
require("module")
require("common")
require("cameraTracker")
package.path=package.path..";../Samples/FlexibleBody/lua/?.lua" --;"..package.path
require("test/IguanaIKSolverLua")
require("RigidBodyWin/subRoutines/CollisionChecker")
background="terrain"
--background="plane"
showTargetMesh=false
matchPose=true
performIK=false -- buggy. Use penaltyWin instead for testing this.

--[[
   아래 부분의 코드가 구현에 해당한다. 윗부분을 바꿔서 다른 시나리오를 수행할 수 있다.
]]--
scale_factor=0.01 -- mesh is in cm scale, but simulation is done using meter unit.


--option="explicit"
option="implicit"
--option="semi-implicit"
useEventRecoder=false
if useEventRecoder then
	startMode="save" -- "load" or "save" 
end

Timeline=LUAclass(LuaAnimationObject)
function Timeline:__init(label, totalTime)
	self.totalTime=totalTime
	RE.renderer():addFrameMoveObject(self)
	self:attachTimer(1/30, totalTime)		
	RE.motionPanel():motionWin():addSkin(self)
end

function start()
	if not mMotion then
		mMotion=Motion()
		mMotion:initSkeleton(RE.motionLoader("iguana.skl"));
		mMotion:skeleton():readJointIndex("../Resource/motion/trc/iguana_from248.ee");
		mMotion:concatFromFile( "trc/iguana_motion_set.mot");
		mMotion:setIdentifier("iguana");
	end
	mTargetMesh=Mesh()
	world=TRL.DynamicsSimulator_TRL_massSpring()
	RE.viewpoint():setFOVy(44.999999)

	local mSimulator=world

	if false and world:numSkeleton()~=0 then 
		if win:isTRCwin_synthesizing() then
			win.mTRCwin:findWidget("Synthesis"):checkButtonValue(false)
			win.mTRCwin:callCallbackFunction(w)
		end
	else

		prevDrawn=0
		prevExactTime=0

	end
	rootnode =RE.ogreRootSceneNode()
	bgnode=RE.createChildSceneNode(rootnode , "BackgroundNode")

	if true then

		-- 50kg, 80 mm  (메시가 크고 코드 전반적으로 스케일을 고치기 워려워서..)
		if useEventRecoder then
			RE.viewpoint():setFOVy(45.000004)
			RE.viewpoint():setZoom(1.000001)
			RE.viewpoint().vpos:set(400.811377, 193.491147, 485.733366)
			RE.viewpoint().vat:set(61.462009, -103.110592, 22.623585)
			RE.viewpoint():update()
		else
			RE.viewpoint():setClipDistances(0.5, 1000)
			RE.viewpoint():setFOVy(45.000002)
			RE.viewpoint().vpos:assign(vector3(54.725145, 3.286307, 74.032226))
			RE.viewpoint().vat:assign(vector3(1.288031, 0.837722, 0.142416))
			RE.viewpoint():update()
		end

		-- 10000mm/s^2 = -10 m/s^2
		gravity=-9.8
		-- 120
		win.simulationFrameRate=120
		win.renderingFrameRate=30

		stiffness=800
		legStiffness=stiffness*2
		springDamp=40

		if false then
			-- much softer
			stiffness=400
			springDamp=4
		elseif false then
			-- slightly softer
			stiffness=4000
			springDamp=11
		end

		dynamicFrictionCoef=0.5
		--dynamicFrictionCoef=10 -- this doesn't seem correct but..
		world:setParameter("mass", 15)
		world:setParameter("bendingConstraints1", 4)
		world:setParameter("stiffness", stiffness)
		world:setParameter("bendingStiffness", stiffness*0.5)

		integrateM=1

		--#define INTEGRATE_EXPLICIT			0
		--#define INTEGRATE_IMPLICIT			1
		--#define INTEGRATE_SEMI_IMPLICIT		2
		world:setParameter("integrateMethod",integrateM)
		world:setParameter("springDamp",springDamp)
		world:setParameter("kDF", dynamicFrictionCoef)	-- dynamic friction coefficient
		world:setParameter("kDFvelThr", 0.01)
		world:setParameter("drawContactForce", 1)

		world:setParameter("debugDrawWorld",0)
		world:setParameter("drawSimulatedMesh",1)
		world:setGVector(vector3(0, gravity, 0))

		local function createTerrain(filename, pos, sizeX, sizeY, width, height, heightMax)
			world:createTerrain(filename, pos*scale_factor, sizeX, sizeY, width*scale_factor, height*scale_factor, heightMax*scale_factor)
			--win:setTRCwinTerrain(filename, pos, sizeX, sizeY, width, height, heightMax)
		end
		createTerrain("../Resource/crowdEditingScripts/terrain/heightmap1_256_256_2bytes.raw", vector3(-200,0, -200), 256, 256, 800, 800, 20)


		--	debug.debug()
		if background=="terrain" then
			--initialPosition=vector3(0,30,0)
			initialPosition=vector3(0,10,0)
		else
			initialPosition=vector3(0,30,0)
		end

		win:createTargetMesh(0, 1.0, initialPosition);

		mScaledTargetMesh=win:targetMesh():copy()

		
		function scaleTargetMesh(scale_factor)

			local mTargetMesh=win:targetMesh()
			for i=0, mTargetMesh:numVertex()-1 do
				local v=mTargetMesh:getVertex(i)
				mScaledTargetMesh:getVertex(i):assign(v*scale_factor)
			end
		end

		local maxV=vector3(-1e5, -1e5, -1e5)
		local minV=vector3(1e5, 1e5, 1e5)
		for i=0, win:targetMesh():numVertex()-1 do
			local v= win:targetMesh():getVertex(i)
			maxV.x=math.max(v.x, maxV.x)
			maxV.y=math.max(v.y, maxV.y)
			maxV.z=math.max(v.z, maxV.z)
			minV.x=math.min(v.x, minV.x)
			minV.y=math.min(v.y, minV.y)
			minV.z=math.min(v.z, minV.z)
		end
		print(maxV-minV)
		scaleTargetMesh(scale_factor)
		if false then
			-- default
			world:createSoftBody(mScaledTargetMesh)
		else
			-- manual
			local mesh=win:targetMesh()
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
			world:createSoftBody(mScaledTargetMesh, springs, vstiff)
		end

		--if integrateM~=1 then
		local param=vectorn ()
		param:setValues(0.5,0.5)
		world:registerAllCollisionCheckPairs(0, 1, param)

		world:init(1.0/win.simulationFrameRate, Physics.DynamicsSimulator.EULER)


		local resourcePath="../src/lua/"

		world:softbody(1):adjustSoftBodyParam("dynamicFrictionCoef", resourcePath.."iguanaAll.selection", dynamicFrictionCoef*0)
		world:softbody(1):adjustSoftBodyParam("dynamicFrictionCoef", resourcePath.."leftFoot.selection", dynamicFrictionCoef)
		world:softbody(1):adjustSoftBodyParam("dynamicFrictionCoef", resourcePath.."rightFoot.selection", dynamicFrictionCoef)
		world:softbody(1):adjustSoftBodyParam("dynamicFrictionCoef", resourcePath.."leftHand.selection", dynamicFrictionCoef)
		world:softbody(1):adjustSoftBodyParam("dynamicFrictionCoef", resourcePath.."rightHand.selection", dynamicFrictionCoef)

		world:softbody(1):adjustSoftBodyParam("stiffness", resourcePath.."leftFoot.selection", legStiffness)
		world:softbody(1):adjustSoftBodyParam("stiffness", resourcePath.."rightFoot.selection", legStiffness)
		world:softbody(1):adjustSoftBodyParam("stiffness", resourcePath.."leftHand.selection", legStiffness)
		world:softbody(1):adjustSoftBodyParam("stiffness", resourcePath.."rightHand.selection", legStiffness)
		--mTerrain=TerrainWrapper ()
		--mIKsolver=IguanaIKSolverLUA(win:trcWin_motion():skeleton(), mTerrain)
	end
end
IguanaHelper=LUAclass()

function IguanaHelper:__init()

end
function IguanaHelper:targetMesh()
	return mTargetMesh
end
function IguanaHelper:isTRCwin_synthesizing() 
	return false
end
function IguanaHelper:createTargetMesh()


	local mMeshAnimation=SkinnedMeshLoader("iguana_physics.mesh");
	mMeshAnimation:reorderVertices()
	mMeshAnimation:sortBones(mMotion:skeleton());
	local bindpose=Pose()
	RE.loadPose(bindpose, "../mesh/iguana_bind.pose");
	mMeshAnimation:setPose(bindpose);
	mMeshAnimation:setCurPoseAsBindPose();
	mMeshAnimation:loadMesh("../mesh/iguana_bound.tri");	-- 내가 인덱싱만 바꾼 메쉬를 읽는다. -  volume mesh 와 호환되도록 바꾸었다.

	mTargetMesh:assignMesh(mMeshAnimation:getCurrMesh());

	for i=0,mTargetMesh:numVertex()-1 do
		mTargetMesh:getVertex(i):radd(initialPosition);
	end
end
function ctor()
	win=IguanaHelper()

	this:create("Button", "Start", "Start");
	this:widget(0):buttonShortcut('FL_ALT+s');	
	this:updateLayout()


	--mTimeline=Timeline("Timeline", 100000)
	--RE.motionPanel():motionWin():playFrom(0)
	if useEventRecoder then
		--mEventReceiver=EVR() -- also rename EVRecoder:_onFrameChanged to EVR:_onFrameChanged
		mEventReceiver=EVRecoder("none", "_testCollAvoid.dat") 
	end
end

function onCallback(w, userData)
	--win:_onCallback(w, userData)
	if w:id()=='Start' then
		start()
	end
end
if not useEventRecoder then
	EVRecoder={}
end
function EVRecoder:_frameMove(fElapsedTime)
	if false then
		return win:_frameMove(fElapsedTime)
	end

	if not world then return end

	local mSimulator=world

	local nSubStep=math.round(win.simulationFrameRate/win.renderingFrameRate)
	local step=1.0/win.simulationFrameRate

	local mTargetMesh=win:targetMesh()
	if mTargetMesh:numVertex()>0 then
		if win:isTRCwin_synthesizing() then
			win:trcWin_singleFrame();
			for i=0, nSubStep-1 do
				local exactTime=sop.clampMap(i,0,nSubStep, prevDrawn, win:trcWin_lastDrawn());
				--print(exactTime);
				if(win:trcWin_hasTerrain()) then
					updateTargetMeshTRCwin(win:trcWin_terrainMotion(), prevExactTime, exactTime);
				else
					updateTargetMeshTRCwin(win:trcWin_motion(), prevExactTime, exactTime);
				end
				win:stepSimulation(step);
				win:stepSimulation(step); -- make iguana twice slower
				prevExactTime=exactTime;
			end
			prevDrawn=win:trcWin_lastDrawn();
		else
			world:stepSimulation();
		end

	end

	world:renderme()

end
function dtor()
	dbg.finalize()
end
function updateTargetMeshTRCwin(mot, prevTime, time)
	local prevPose=Pose()
	local pose=Pose();
	mot:samplePose(prevPose, prevTime);
	mot:samplePose(pose, time);
	-- retrieve example mesh to mTargetMesh.

	local mMeshAnimation=win.mMeshAnimation

	local mTargetMesh=win:targetMesh()
	local mPrevTargetMesh=win:prevTargetMesh()
	mMeshAnimation:setPose(prevPose);
	mMeshAnimation:retrieveAnimatedMesh(mPrevTargetMesh);
	mMeshAnimation:setPose(pose);
	mMeshAnimation:retrieveAnimatedMesh(mTargetMesh);

	if(matchPose) then
		-- 타겟메쉬의 이전 프레임과 simulatedMesh의 마지막 프레임을 맞추어준다.

		local metric=math.KovarMetric ()

		if not mScaledSimulatedMesh then
			mScaledSimulatedMesh=world:softbody(1).mSimulatedMesh:copy()
		end

		function scaleSimulatedMesh(scale_factor)
			local simMesh=world:softbody(1).mSimulatedMesh;
			for i=0, simMesh:numVertex()-1 do
				local v=simMesh:getVertex(i)
				mScaledSimulatedMesh:getVertex(i):assign(v*(1.0/scale_factor))
			end
		end
		scaleSimulatedMesh(scale_factor)
		local mSimulatedMesh=mScaledSimulatedMesh

		local nv=mSimulatedMesh:numVertex();
		local simulated=vectorn(nv*3)
		local captured=vectorn(nv*3)
		for i=0, nv-1 do
			simulated:setVec3(i*3, mSimulatedMesh:getVertex(i));
		end

		for i=0, nv-1 do
			captured:setVec3(i*3, win:prevTargetMesh():getVertex(i));
		end
		
		metric:calcDistance(simulated, captured);

		--dbg.namedDraw('Traj', simulated:matView(3), 'curr', 'blueCircle', 3, 'QuadListZ')
		--dbg.namedDraw('Traj', captured:matView(3), 'goal', 'redCircle', 3, 'QuadListZ')

		win:trcWin_transformSynthesized(metric.transfB);

		if(performIK) then

			pose:setRootTransformation(pose:rootTransformation()*transf(metric.transfB));

			local DEBUG_DRAW=false
			if DEBUG_DRAW then
				-- debug skin
				if not SKIN then
					SKIN=RE.createSkin(win:trcWin_motion():skeleton())
				end
			end
			local noParam=vector3N ()
			mIKsolver:IKsolve(pose, noParam)
			if DEBUG_DRAW then
				SKIN:setPose(pose, win:trcWin_motion():skeleton())
			end
			-- update targetMesh to reveal IK result.
			mMeshAnimation:setPose(pose);
			mMeshAnimation:retrieveAnimatedMesh(mTargetMesh);
		end
	end
	scaleTargetMesh(scale_factor)
	world:softbody(1):changeSoftBodyRestLength(mScaledTargetMesh);
end

function EVRecoder:_onFrameChanged(win, iframe)
end
function EVRecoder:_handleRendererEvent(ev, button, x,y) 
	print(ev, button)
	if ev=="KEYDOWN" or ev=="KEYUP" then
		local ascii=tonumber(button)
		if not ascii then
			ascii=string.byte(button)
		end
		--print(ascii)
		return win:trcWin_handleRendererEvent(ev, ascii, -1, -1);
	else
		return win:trcWin_handleRendererEvent(ev, button,x, y);
	end
	--return 0
end

TerrainWrapper = LUAclass()
function TerrainWrapper :__init() 
end
function TerrainWrapper:getTerrainHeight2(pos)
	return win:getTerrainHeight(pos)
end

if not useEventRecoder then
	function handleRendererEvent(ev, button, x,y)
		return EVRecoder:_handleRendererEvent(ev, button, x, y)
	end
	function frameMove(fElapsedTime)
		return EVRecoder:_frameMove(fElapsedTime)
	end
end
