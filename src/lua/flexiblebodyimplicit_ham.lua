require("config")
require("module")
require("common")
require("cameraTracker")
package.path=package.path..";../Samples/FlexibleBody/lua/?.lua" --;"..package.path
require("test/IguanaIKSolverLua")
require("RigidBodyWin/subRoutines/CollisionChecker")
useGraph=true
background="terrain"
--background="plane"
showHighMesh=false	
showTargetMesh=false
matchPose=true
performIK=true
--drawSphere=true
drawSphere=false

--[[
   아래 부분의 코드가 구현에 해당한다. 윗부분을 바꿔서 다른 시나리오를 수행할 수 있다.
]]--
scenario="original_scale"

--option="explicit"
option="implicit"
--option="semi-implicit"
usePenaltyMethod=true
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
	rootnode =RE.ogreRootSceneNode()
	bgnode=RE.createChildSceneNode(rootnode , "BackgroundNode")

	if scenario=="original_scale" then
		scale_factor=1

		-- 50kg, 80 mm  (메시가 크고 코드 전반적으로 스케일을 고치기 워려워서..)
		if useEventRecoder then
			RE.viewpoint():setFOVy(45.000004)
			RE.viewpoint():setZoom(1.000001)
			RE.viewpoint().vpos:set(400.811377, 193.491147, 485.733366)
			RE.viewpoint().vat:set(61.462009, -103.110592, 22.623585)
			RE.viewpoint():update()
			if not drawSphere then
				win:trcWin_setDrawSphere(false)
			end
		else
			RE.viewpoint():setClipDistances(0.5, 1000)
			RE.viewpoint():setFOVy(45.000002)
			RE.viewpoint().vpos:assign(vector3(54.725145, 3.286307, 74.032226))
			RE.viewpoint().vat:assign(vector3(1.288031, 0.837722, 0.142416))
			RE.viewpoint():update()
		end

		if not drawSphere then
			win:trcWin_setDrawSphere(false)
		end

		-- 10000mm/s^2 = -10 m/s^2
		gravity=-10000
		-- 120
		if usePenaltyMethod then
			win.simulationFrameRate=1200 -- 밑에 penalty gains를 두배 늘리고 simulationFrameRate를 두배 늘리면 품질이 좀더 나아지지만, 너무 느려서 일단 이렇게.
		else
			win.simulationFrameRate=120
		end
		win.renderingFrameRate=60
		stiffness=100000
		legStiffness=stiffness*2
		springDamp=1000	

		if false then
			stiffness=10000
			legStiffness=stiffness*2
			springDamp=100	
		end
		dynamicFrictionCoef=0.5
		penaltyStiffness=1000000
		penaltyDampness=10000
		--dynamicFrictionCoef=10 -- this doesn't seem correct but..
		world:setParameter("mass", 50)
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
		world:setParameter("drawSimulatedMesh",0)
--		world:setParameter("drawSimulatedMesh",1)
		world:setGravity(vector3(0, gravity, 0))

		if background=="terrain" then
			initialPosition=vector3(0,30,0)
		else
			initialPosition=vector3(0,30,0)
		end

		--to change background height, use last parameter of createTerrain function
		--to change background texture, see Ogre_ShapeDrawer.cpp createOgreRigidBody - setMaterialName
		if background=="terrain" then
			world:createTerrain("../Resource/crowdEditingScripts/terrain/heightmap1_256_256_2bytes.raw", vector3(-200,0, -200), 256, 256, 800, 800, 30)
--			world:createTerrain("../Resource/crowdEditingScripts/terrain/heightmap1_256_256_2bytes.raw", vector3(-200,0, -200), 256, 256, 800, 800, 0)
		else
			world:createFloor(-15, vector3(400, 30, 400))
		end


		if useGraph == true then
			win:setTRCwinTerrain();
			local w=win.mTRCwin:findWidget("Synthesis")
			w:checkButtonValue(1)
			win.mTRCwin:callCallbackFunction(w)

			if showTargetMesh == false then
				win:setSkeletonVisible(false)
			else
				win:setSkeletonVisible(true)
			end

		end

		--	debug.debug()

		win:createTargetMesh(0, scale_factor, initialPosition);

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
		world:createSoftBody(win:targetMesh())
		--if integrateM~=1 then
		--world:setParameter("contactMethod",SimulatorImplicit.PENALTY_METHOD)
		world:setParameter("contactMethod",SimulatorImplicit.LCP_METHOD)

		if(usePenaltyMethod) then
			world:setParameter("contactMethod",SimulatorImplicit.PENALTY_METHOD)
			world:setParameter("penaltyStiffness", penaltyStiffness)
			world:setParameter("penaltyDampness", penaltyDampness)
			world:setParameter("penaltyMuScale", 2)
		end
		--else
		--world:setParameter("contactMethod",SimulatorImplicit.LCP_METHOD_FRICTIONLESS)
		--world:setParameter("contactMethod",SimulatorImplicit.LCP_METHOD)
		--	world:setParameter("contactMethod",SimulatorImplicit.BARAFF98)
		--	world:setParameter("penaltyStiffness", 10000)
		--end

		local resourcePath="../src/lua/"

		world:adjustSoftBodyParam("dynamicFrictionCoef", resourcePath.."iguanaAll.selection", dynamicFrictionCoef*0)
		--world:adjustSoftBodyParam("dynamicFrictionCoef", resourcePath.."iguanaTail.selection", 0)
		world:adjustSoftBodyParam("dynamicFrictionCoef", resourcePath.."leftFoot.selection", dynamicFrictionCoef)
		world:adjustSoftBodyParam("dynamicFrictionCoef", resourcePath.."rightFoot.selection", dynamicFrictionCoef)
		world:adjustSoftBodyParam("dynamicFrictionCoef", resourcePath.."leftHand.selection", dynamicFrictionCoef)
		world:adjustSoftBodyParam("dynamicFrictionCoef", resourcePath.."rightHand.selection", dynamicFrictionCoef)

		world:adjustSoftBodyParam("stiffness", resourcePath.."leftFoot.selection", legStiffness)
		world:adjustSoftBodyParam("stiffness", resourcePath.."rightFoot.selection", legStiffness)
		world:adjustSoftBodyParam("stiffness", resourcePath.."leftHand.selection", legStiffness)
		world:adjustSoftBodyParam("stiffness", resourcePath.."rightHand.selection", legStiffness)
		mTerrain=TerrainWrapper ()
		mIKsolver=IguanaIKSolverLUA(win:trcWin_motion():skeleton(), mTerrain)
		noParam=vector3N ()
	end
end
function ctor()
	win:_ctor()
	--mTimeline=Timeline("Timeline", 100000)
	--RE.motionPanel():motionWin():playFrom(0)
	if useEventRecoder then
		--mEventReceiver=EVR() -- also rename EVRecoder:_onFrameChanged to EVR:_onFrameChanged
		mEventReceiver=EVRecoder("none", "_testCollAvoid.dat") 
		if not drawSphere then
			mEventReceiver.drawCursor=true
		end
	end
end

function callStart()
	RE.viewpoint():setFOVy(44.999999)
	if(world:isWorldValid()) then
		if win:isTRCwin_synthesizing() then
			win.mTRCwin:findWidget("Synthesis"):checkButtonValue(false)
			win.mTRCwin:callCallbackFunction(w)
		end
	else
		win:start()
		prevDrawn=0
		prevExactTime=0
		start()

		if useEventRecoder and mEventReceiver.mode=="none" then
			mEventReceiver:changeMode(startMode)
		end
	end
end

function onCallback(w, userData)
	--win:_onCallback(w, userData)
	--can see button create part in FlexibleBodyImplicitWin.cpp
	if w:id()=='Start' then
		callStart()
		local viewMatrix = matrix4()
		viewMatrix:setValue(
 -0.684961, 0.000000, -0.728580, 195.517271,
 -0.252231, 0.938163, 0.237130, 2.081572,
 0.683526, 0.346195, -0.642605, -157.120731,
 0.000000, 0.000000, 0.000000, 1.000000
		)
		
		RE.viewpoint():SetViewMatrix(viewMatrix)
		win:trcWin_setDestination(vector3(235,0,222))
		--win:trcWin_setDestination(vector3(278,0,231))
--		win:trcWin_setDestination(vector3(225,0,249))

	elseif w:id()=='setDest' then
		win:trcWin_setDestination(vector3(225,0,249))
	elseif w:id()=='setView' then
		local viewMatrix = matrix4()
		viewMatrix:setValue(
		-0.581654, 0.000000, -0.813436, 236.667422,
		-0.481537, 0.805953, 0.344327, 38.769528,
		0.655592, 0.591979, -0.468786, -236.511823,
		0.000000, 0.000000, 0.000000, 1.000000
		)
		RE.viewpoint():SetViewMatrix(viewMatrix)
--		RE.saveViewpoint("flexiblebodyimplicit_view.lua")
	elseif w:id()=='setView2' then
		local viewMatrix = matrix4()
		viewMatrix:setValue(
		-0.048476, 0.000000, -0.998824, 180.585711,
		 -0.998824, 0.000017, 0.048476, 136.644448,
		 0.000017, 1.000000, -0.000001, -287.990082,
		 0.000000, 0.000000, 0.000000, 1.000000
		)
		RE.viewpoint():SetViewMatrix(viewMatrix)
	elseif w:id()=='saveView' then
		RE.saveViewpoint("flexiblebodyimplicit_view.lua")
	end

end
if not useEventRecoder then
	EVRecoder={}
end
function EVRecoder:_frameMove(fElapsedTime)
	if false then
		return win:_frameMove(fElapsedTime)
	end

	local mSimulator=world
	if not mSimulator:isWorldValid() then return 0 end

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
				prevExactTime=exactTime;
			end
			prevDrawn=win:trcWin_lastDrawn();
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
		local mSimulatedMesh=world.mSimulatedMesh;

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

			local DEBUG_DRAW=true
			if DEBUG_DRAW then
				-- debug skin
				if not SKIN then
					SKIN=RE.createSkin(win:trcWin_motion():skeleton())
--					SKIN:setTranslation(0,50,0)
					SKIN2=RE.createSkin(win:trcWin_motion():skeleton())
--					SKIN2:setTranslation(0,50,0)
				end
			end
			mIKsolver:IKsolve(pose, noParam)
			if DEBUG_DRAW then
--				SKIN:setPose(pose, win:trcWin_motion():skeleton())
				SKIN2:setPose(mIKsolver.PFpose,win:trcWin_motion():skeleton())
--				SKIN:setPose(mIKsolver.PFpose,win:trcWin_motion():skeleton())
			end
			-- update targetMesh to reveal IK result.
			mMeshAnimation:setPose(pose);
			mMeshAnimation:retrieveAnimatedMesh(mTargetMesh);
		end
	end
	world:changeSoftBodyRestLength(mTargetMesh);
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
