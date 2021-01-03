require("config")

package.projectPath='../Samples/QP_controller/'
package.resourcePath='../Resource/motion/'
package.path=package.path..";../Samples/QP_controller/lua/?.lua" --;"..package.path
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path
package.path=package.path..";../Resource/classification/lua/?.lua" --;"..package.path
require("module")

Timeline=LUAclass(LuaAnimationObject)
function Timeline:__init(label, totalTime)
	self.totalTime=totalTime
	RE.renderer():addFrameMoveObject(self)
	self:attachTimer(1/30, totalTime)		
	RE.motionPanel():motionWin():addSkin(self)
end

model_files={}
model_files.default={}
-- Default PD-servo settings
model_files.default.k_p_PD=500 -- Nm/rad
model_files.default.k_d_PD=5 --  Nms/rad. 
model_files.default.k_p_slide=2000-- 
model_files.default.k_d_slide=50 --   
model_files.default.timestep=1/8000
model_files.default.rendering_step=1/30
--model_files.default.rendering_step=1/60 -- 0.5x real-time speed
model_files.default.frame_rate=120 -- mocap frame rate
model_files.default.initialHeight=0.01 -- meters
model_files.default.penaltyDepthMax={0.0005}
model_files.default.penaltyForceStiffness=30000
model_files.default.penaltyForceDamp=3000
model_files.default.start=0

model_files.gymnist=deepCopyTable(model_files.default)
model_files.gymnist.file_name=package.resourcePath.."gymnist/gymnist.wrl"
model_files.gymnist.mot_file=package.resourcePath.."gymnist/gymnist.dof"
--model_files.gymnist.initialHeight=0.07
model_files.gymnist.initialHeight=0.7
model_files.gymnist.timestep=1/8200

model_files.lowerbody=deepCopyTable(model_files.default)
model_files.lowerbody.file_name=package.resourcePath.."lowerbody.wrl"
model_files.lowerbody.initialHeight=1.5 -- meters

model_files.chain=deepCopyTable(model_files.default)
model_files.chain.file_name="../Resource/mesh/tree_complex3.wrl"
model_files.chain.initialHeight=1.2 -- meters
model_files.chain.initialAngle=math.rad(10) 

model_files.chain2=deepCopyTable(model_files.default)
model_files.chain2.file_name="../Resource/mesh/tree_slide_3.wrl"
model_files.chain2.initialHeight=1.2 -- meters
model_files.chain2.initialAngle=math.rad(10) 
model_files.chain2.k_p_PD=50 -- Nm/rad
model_files.chain2.k_d_PD=1 --  Nms/rad. 

model_files.car=deepCopyTable(model_files.default)
model_files.car.file_name="../Samples/QP_controller/Resource/car_bmw7_original_doublewheel.wrl" -- output from car_generator.lua
model_files.car.initialHeight=1.0 -- meters
model_files.car.timestep=1/800
model_files.car.numSolverIterations=200
model_files.car.k_p_slide=100000-- 
model_files.car.k_d_slide=10000 --   
model_files.car.clampTorque=8000 -- default value is much smaller, but a car is so much heavier than a human.
model_files.car.clampForce=80000
model=model_files.gymnist
--model=model_files.lowerbody
--model=model_files.chain
--model=model_files.chain2
--model=model_files.car

ServoMethods={PD=1, HD=2, QP=3}
--servoMethod=ServoMethods.QP
servoMethod=ServoMethods.PD

function createSimulator()
	--return Physics.DynamicsSimulator_AIST("bullet") 
	--return Physics.DynamicsSimulator_AIST("libccd") 
	--return Physics.DynamicsSimulator_AIST("libccd") 
	--return Physics.DynamicsSimulator_TRL_LCP("libccd")  -- now works well.
	return TRL.DynamicsSimulator_TRL_massSpring()  -- now works well.
	--local sim=TRL.DynamicsSimulator_TRL_massSpring()  -- now works well.
	--sim:createTerrain("../Resource/crowdEditingScripts/terrain/heightmap1_256_256_2bytes.raw", vector3(-200,0, -200), 256, 256, 800, 800, 30)
	--hreturn sim
	--return Physics.DynamicsSimulator_gmbs_penalty() 
	--servoMethod=ServoMethods.QP return Physics.DynamicsSimulator_gmbs()   -- use Samples/QP_controller
	--return Physics.DynamicsSimulator_UT("bullet") 
	--return Physics.DynamicsSimulator_UT("opcode") 
    --return Physics.DynamicsSimulator_UT_penalty() 
	--return Physics.DynamicsSimulator_UT("simple")  -- doesn't work
	--Physics.DynamicsSimulator_AIST("simple")  --  doesn't work
end



function init_globals()

	simulatorParam={
		timestep=model.timestep,
		integrator=Physics.DynamicsSimulator.EULER,
		debugContactParam={10, 0, 0.01, 0, 0}, -- size, tx, ty, tz, tfront
	}
	
	--[[ implementations ]]--
	if servoMethod==ServoMethods.HD then
		simulatorParam.useInverseDynamics=true
		simulatorParam.useQPsolver=false
	elseif servoMethod==ServoMethods.QP then
		simulatorParam.useInverseDynamics=false
		simulatorParam.useQPsolver=true
	elseif servoMethod==ServoMethods.PD then
	end

	k_p=model.k_p_PD	-- Nm/rad
	k_d=model.k_d_PD --  Nms/rad. worked in range [0, 1]

end

EVR=LUAclass(EventReceiver)
function EVR:__init()
end
function EVR:onFrameChanged(win, iframe)
	if mLoader~=nill and this:findWidget("simulation"):checkButtonValue() then
		local niter=math.floor(model.rendering_step/model.timestep+0.5)
		mRagdoll:frameMove(niter)
	end
end
function ctor()
	mEventReceiver=EVR()
   --	this:create("Button", "Start", "Start")
   --	this:widget(0):buttonShortcut("FL_ALT+s")

   this:create("Check_Button", "simulation", "simulation", 0, 2,0)
   this:widget(0):checkButtonValue(1) -- 1 for imediate start
   this:widget(0):buttonShortcut("FL_ALT+s")
   
   this:create("Button", "single step", "single step", 2, 3,0)
   
   this:create("Check_Button", "draw skeleton", "draw skeleton", 0, 3,0)
   this:widget(0):checkButtonValue(0)

   this:create("Button", "debug LoaderToTree class", "debug LoaderToTree class")
   this:create("Button", "debug get/setlink", "debug get/setlink")
   this:create("Button", "debug get/setlink2", "debug get/setlink2")

   this:updateLayout()
   this:redraw()
   
   RE.viewpoint().vpos:assign(vector3(330.411743, 69.357635, 0.490963))
   RE.viewpoint().vat:assign(vector3(-0.554537, 108.757057, 0.477768))
   RE.viewpoint():update()
   RE.viewpoint():TurnRight(math.rad(viewRotate or 0))
   _start()
end

function dtor()
   -- remove objects that are owned by C++
   if mSkin~=nill then
      RE.remove(mSkin)
      mSkin=nil
   end
   if mSkin2~=nill then
      RE.remove(mSkin2)
      mSkin2=nil
   end
   if mTimeline then
	   RE.motionPanel():motionWin():detachSkin(mTimeline)
	   mTimeline=nil
   end
   -- remove objects that are owned by LUA
   collectgarbage()
end

function _start()
	dtor()
	mTimeline=Timeline("Timeline", 1000000)
	RE.motionPanel():motionWin():playFrom(0)
	print("start")
	mLoader=MainLib.VRMLloader(model.file_name)
	mLoader:printHierarchy()
	if model.mot_file~=nill then
		local container=MotionDOFcontainer(mLoader.dofInfo, model.mot_file)
		mMotionDOF=container.mot
	else
		local loader=mLoader
		local container=MotionDOFcontainer(loader.dofInfo)
		--local container=MotionDOFcontainer(mLoader.dofInfo, model.mot_file)

		local nf=100
		container:resize(nf)
		for i=0, nf-1 do
			container.mot:row(i):setAllValue(0)
			container.mot:row(i):set(3, 1) -- assuming quaternion (free root joint)
			
			--container.mot:row(i):assign(container.mot:row(0))
		end
		mMotionDOF=container.mot
	end

	mFloor=MainLib.VRMLloader("../Resource/mesh/floor_y.wrl")

	drawSkeleton=this:findWidget("draw skeleton"):checkButtonValue()

	local simulator=createSimulator()
	if useBullet and model.numSolverIterations then
		simulator:setNumSolverIterations(model.numSolverIterations )
	end
	init_globals()


	mRagdoll= RagdollSim(mLoader, drawSkeleton, mMotionDOF, simulator, simulatorParam)
	mRagdoll.drawDebugInformation=true

	mSkin2=RE.createVRMLskin(mFloor, false)
	local s=model.skinScale or 100
	mSkin2:scale(s,s,s)
end

function onCallback(w, userData)
	if w:id()=="Start" then
		_start()
	elseif w:id()=="debug get/setlink" then
		-- just for testing and debugging.
		local theta=vectorn()
		local dtheta=vectorn()
		local sim=mRagdoll.simulator
		sim:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, theta);
		sim:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dtheta);
		sim:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, theta);
		sim:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dtheta);
		sim:initSimulation()
		local theta2=vectorn()
		local dtheta2=vectorn()
		sim:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, theta2);
		sim:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dtheta2);
		local delta=dtheta-dtheta2
		assert(delta:maximum()<0.0001)
		assert(delta:minimum()>-0.0001)
		print('delta1:', delta)
		sim:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, theta2);
		sim:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dtheta2);
		sim:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, theta);
		sim:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dtheta);
		local delta=dtheta-dtheta2
		assert(delta:maximum()<0.0001)
		assert(delta:minimum()>-0.0001)
		print('test passed.')
	elseif w:id()=="debug LoaderToTree class" then
		-- just for testing and debugging.
		local theta=vectorn()
		local dtheta=vectorn()
		local theta2=vectorn()
		local dtheta2=vectorn()
		local sim=mRagdoll.simulator
		sim:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, theta);
		sim:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dtheta);

		sim:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, theta);
		sim:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dtheta);

		sim:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, theta2);
		sim:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dtheta2);

		sim:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, theta2);
		sim:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dtheta2);

		local e=MotionUtil.Effectors()
		local c=MotionUtil.Constraints()
		local tree=MotionUtil.LoaderToTree(mLoader, e,c, false,false)
		tree:setPoseDOF(mLoader.dofInfo, theta)
		print('hihi')
		print(theta)
		print(dtheta)
		tree:setVelocity(mLoader.dofInfo, dtheta)
		tree:Print()

		local testPassed=true
		for i=1, mLoader:numBone()-1 do
			local wldst=sim:getWorldState(0):globalFrame(i)
			local wldst2=tree:getLastNode(i)._global
			if (wldst.translation- wldst2.translation):length()>0.0001 then
				testPassed=false
				print('test1 failed')
				dbg.console()
			end
		end
		for i=1, mLoader:numBone()-1 do
			local wldst=sim:getWorldState(0):globalFrame(i)
			local wldst2=tree:getLastNode(i)._global
			local angVel=sim:getWorldAngVel(0, mLoader:VRMLbone(i))
			local linVel=sim:getWorldVelocity(0, mLoader:VRMLbone(i), vector3(0,0,0))
			local bangVel=tree:getLastNode(i):bodyAngVel():copy()
			local blinVel=tree:getLastNode(i):bodyLinVel():copy()
			angVel:rotate(wldst2.rotation:inverse())
			linVel:rotate(wldst2.rotation:inverse())
			print(i)
			print(wldst)
			print(wldst2)
			print(angVel, bangVel)
			print(linVel, blinVel)
			if( angVel- bangVel):length()>0.0001 then
				testPassed=false
				dbg.console()
			end
			if( linVel- blinVel):length()>0.0001 then
				testPassed=false
				dbg.console()
			end
		end
		local dtheta2=vectorn(dtheta:size())
		tree:getVelocity(mLoader.dofInfo, dtheta2)
		local delta=dtheta-dtheta2
		assert(delta:maximum()<0.0001)
		assert(delta:minimum()>-0.0001)
		if testPassed then
			print("Test passed")
		end
	elseif w:id()=='debug get/setlink2' then
		testGetSetLink()
	end
end

function frameMove(fElapsedTime)
end

RagdollSim=LUAclass ()

function RagdollSim:__init(loader, drawSkeleton, motdof, simulator, simulatorParam)
	if drawSkeleton==nil then drawSkeleton = true end
	self.simulator=simulator
	self.skin=RE.createVRMLskin(loader, drawSkeleton)
	self.skin:setThickness(0.03)
	local s=model.skinScale or 100
	self.skin:scale(s,s,s)

	self.simulator:registerCharacter(loader)
	-- do not use local here.
	floor=mFloor or VRMLloader("../Resource/mesh/floor_y.wrl")
	self.simulator:registerCharacter(floor)
	registerContactPairAll(model, loader, floor, self.simulator)   
	if simulatorParam.useInverseDynamics then
		self.simulator:registerContactQueryBone(0, loader:getBoneByVoca(MotionLoader.LEFTANKLE))
		self.simulator:registerContactQueryBone(1, loader:getBoneByVoca(MotionLoader.RIGHTANKLE))
	end

	self.simulator:init(simulatorParam.timestep, simulatorParam.integrator)

	self.simulator:setSimulatorParam("debugContact", simulatorParam.debugContactParam) 
	self.simulator:setSimulatorParam("contactForceVis", {0.001,0.001,0.001})
	self.simulator:setSimulatorParam("penaltyDepthMax", {0.0005})
	-- adjust initial positions

	self.motionDOF=motdof
	self.simulationParam=simulatorParam

	self.controlforce=vectorn(loader.dofInfo:numDOF())
	if self.motionDOF then
		for i=0, self.motionDOF:numFrames()-1 do
			self.motionDOF:row(i):set(1,self.motionDOF:row(i):get(1)+(model.initialHeight or 0) )

			if false then
				-- test get, setlinkdata
				self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE,self.motionDOF:row(i))
				local out=vectorn()
				self.simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE,out);
				local errorv=out-self.motionDOF:row(i)
				if errorv:maximum()>0.00001 or errorv:minimum()<-0.00001 then
					print(i, errorv)
					dbg.console()
				end
			end
		end
	else
		assert(false)
	end
	if model.initialAngle then
		local delta= quater(model.initialAngle, vector3(1,0,0) )
		for i=0, self.motionDOF:numFrames()-1 do
			self.motionDOF:row(i):setQuater(3, self.motionDOF:row(i):toQuater(3)*delta)
		end
	end

	self.DMotionDOF=calcDerivative(self.motionDOF)

	self.DDMotionDOF=self.DMotionDOF:derivative(120)

	require("RigidBodyWin/subRoutines/PDservo")
	--require("RigidBodyWin/subRoutines/nonlinearPDservo") PDservo=nonlinearPDservo
	self.pdservo=PDservo(loader.dofInfo)
	self.pdservo:initPDservo(model.start, self.motionDOF:numFrames(),
	self.motionDOF, self.DMotionDOF)



	if self.motionDOF then
		model.start=math.min(model.start, self.motionDOF:numFrames()-1)
		initialState=vectorn()
		initialState:assign(self.motionDOF:row(model.start))

		print("initialState=",initialState)
		self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, initialState)

		if self.DMotionDOF then
			local initialVel=self.DMotionDOF:row(model.start):copy()
			self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, initialVel)
		end
		self.simulator:initSimulation()
	else
		local wldst=self.simulator:getWorldState(0)
		wldst:localFrame(loader:getBoneByTreeIndex(1)).rotation:identity()
		wldst:localFrame(loader:getBoneByTreeIndex(1)).translation:radd(vector3(0, model.initialHeight, 0))
		wldst:forwardKinematics()
		self.simulator:setWorldState(0)
	end
	--	debug.debug()
	self.simulator.setPose(self.skin,self.simulator,0)

	self.skin:setMaterial("lightgrey_transparent")

	--self.simulator.setGVector(vector3(0,0,9.8))
	self.simulator:setGVector(model.GVector or vector3(0,9.8,0))
	self.simulator:initSimulation()
	self.loader=loader
	self.floor=floor -- have to be a member to prevent garbage collection
end
function RagdollSim:setFrame(iframe, initialHeight)
	self.pdservo.startFrame=iframe
	self.pdservo:rewindTargetMotion(self.simulator)
	local initialState=vectorn()
	initialState:assign(self.motionDOF:row(iframe))
	-- set global position
	initialState:set(1,initialState:get(1)+initialHeight)

	self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, initialState)

	if self.DMotionDOF then
		local initialVel=self.DMotionDOF:row(iframe):copy()
		self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, initialVel)
	end
	self.simulator:initSimulation()
end
function RagdollSim:__finalize()
	-- remove objects that are owned by C++
	if self.skin~=nill then
		RE.remove(self.skin)
		self.skin=nil
	end
	self.simulator=nil

end
function RagdollSim:frameMove(niter)
	--assert(math.floor(niter)==niter)
	--		debug.debug()
	temp=vectorn()
	self.controlforce:zero()
	self.simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, temp)

	for iter=1,niter do
		do
			local maxForce=9.8*80
			if not self.pdservo:generateTorque(self.simulator, maxForce) then
				self.pdservo:rewindTargetMotion(self.simulator)
				self.pdservo:generateTorque(self.simulator, maxForce)
			end

			local controlforce=self.pdservo.controlforce

			if self.pdservo.stepSimul then
				self.pdservo:stepSimul(self.simulator)
			else
				--testGetSetLink()
				self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_TORQUE, controlforce)
				if self.drawDebugInformation then
					self.simulator:drawDebugInformation()
				end
				self.simulator:stepSimulation()
			end
			self.controlforce:radd(controlforce)
		end
		self.simulator.setPose(self.skin, self.simulator, 0)				
	end

	self.controlforce:rdiv(niter)
end

function testGetSetLink()
	local self=mRagdoll
	
	local v1=vectorn()
	local v2=vectorn()
	self.simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, v1)
	self.simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, v2)
	local zero=vectorn(v1:size())
	zero:setAllValue(0)
	zero:set(3,1)
	self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, zero)
	self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, zero)
	self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, v1)
	--v2:range(3,v2:size()):setAllValue(0)
	self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, v2)
end
function registerContactPairAll(model, loader, floor, simulator)
   param=vectorn ()
   param:setValues(0.5,0.5, model.penaltyForceStiffness, model.penaltyForceDamp)
   for i=1,loader:numBone()-1 do

      local bone_i=loader:VRMLbone(i)
      simulator:registerCollisionCheckPair(loader:name(),bone_i.NameId, floor:name(), floor:bone(1):name(), param)
   end
end
function calcDerivative(motionDOF)
	assert(motionDOF~=nil)
	local dmotionDOF=matrixn()

	dmotionDOF:setSize(motionDOF:numFrames(), motionDOF:numDOF())

	for i=1, motionDOF:rows()-2 do
		calcDerivative_row(i,dmotionDOF, motionDOF)
	end

	-- fill in empty rows
	dmotionDOF:row(0):assign(dmotionDOF:row(1))
	dmotionDOF:row(dmotionDOF:rows()-1):assign(dmotionDOF:row(dmotionDOF:rows()-2))
	return dmotionDOF
end

function calcDerivative_row(i, dmotionDOF, motionDOF)
   local dmotionDOF_i=dmotionDOF:row(i);
   dmotionDOF_i:sub(motionDOF:row(i+1), motionDOF:row(i)) -- forward difference

   local frameRate=120
   if model then frameRate=model.frame_rate end
   dmotionDOF_i:rmult(frameRate)
   
   assert(motionDOF.dofInfo:numSphericalJoint()==1) 
   -- otherwise following code is incorrect
   local T=MotionDOF.rootTransformation(motionDOF:row(i))
   local V=T:twist( MotionDOF.rootTransformation(motionDOF:row(i+1)), 1/frameRate)
   dmotionDOF_i:setVec3(0, V.v)
   dmotionDOF_i:setVec3(4, V.w)
end

