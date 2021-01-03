

IguanaIKSolverLUA=LUAclass()
function IguanaIKSolverLUA:__init(skel, terrain)
	self.terrain=terrain
	self.ee=MotionUtil.Effectors()
	local ee=self.ee
	self.numEE=5

	ee:resize(self.numEE)
	ee(0):init(skel:getBoneByVoca(MotionLoader.LEFTANKLE):childHead(), vector3(0,0,0));
	ee(1):init(skel:getBoneByVoca(MotionLoader.RIGHTANKLE):childHead(), vector3(0,0,0));
	ee(2):init(skel:getBoneByVoca(MotionLoader.LEFTWRIST):childHead(), vector3(0,0,0));
	ee(3):init(skel:getBoneByVoca(MotionLoader.RIGHTWRIST):childHead(), vector3(0,0,0));
	ee(4):init(skel:getBoneByVoca(MotionLoader.CHEST):childHead(), vector3(0,0,0));

	--for i=0, 4 do
	--	print(ee(i).bone:treeIndex())
	--end

	local tail=ee(4).bone;
	self.tails={
		--tail,
		tail:parent(),
		tail:parent():parent(),
		tail:parent():parent():parent(),
	}
	if false then
		self.ik=MotionUtil.createFullbodyIk_LimbIK(skel, ee);
	else
		self.g_con=MotionUtil.Constraints() -- std::vector<MotionUtil::RelativeConstraint>
		self.g_ee=MotionUtil.Effectors()
		self.ik=MotionUtil.createFullbodyIk_MotionDOF_MultiTarget_lbfgs(skel.dofInfo,self.g_ee,self.g_con)
		local ik=self.ik
		ik:_changeNumEffectors(self.numEE-1) -- excluding tail
		
		for i=0,self.numEE-2 do
			local bone=ee(i).bone
			if not bone:childHead() and bone:rotJointIndex()==-1 then
				ik:_setEffector(i, bone:parent(), bone:getOffset()+ee(i).localpos)
			else
				ik:_setEffector(i, ee(i).bone, ee(i).localpos)
			end
		end
		ik:_effectorUpdated()

		function IguanaIKSolverLUA:IKsolve_sub(pose, con, con_tail)
			self.ik:_changeNumConstraints(con_tail:size()) 
			for i=0, con_tail:size()-1 do
				local plane=Plane(vector3(0,-1,0), con_tail(i))
				self.ik:_setHalfSpaceConstraint(i, self.tails[i+1], vector3(0,0,0), plane.normal, plane.d)
				--self.ik:_setPlaneDistanceConstraint(i, self.tails[i+1], vector3(0,0,0), plane.normal, plane.d)
			end
			self.ik:_effectorUpdated()

			self.skel:setPose(pose)
			local v=vectorn()
			self.skel:getPoseDOF(v)
			local vo=v:copy()
			self.ik:IKsolve(v, con)
			self.skel:setPoseDOF(v)
			self.skel:getPose(pose)
		end
	end
	self.skel=skel
end
function IguanaIKSolverLUA:IKsolve(input_pose)  
	local pose=Pose()
	pose:assign(input_pose)

	--pose:identity() do input_pose:assign(pose) return end


	-- 4다리의 예재 동작에서의 높이 구하기
	local numEE=self.numEE
	local desired_height=vectorn (); desired_height:setSize(numEE);
	local desired_tail_height=vectorn (); desired_tail_height:setSize(#self.tails);
	local curHeight=vectorn (); curHeight:setSize(numEE);
	local floorHeight=vectorn (); floorHeight:setSize(numEE);

	local skel=self.skel
	skel:setPose(pose);

	local ee=self.ee
	for i=0,numEE-1 do
		desired_height:set(i,ee(i).bone:getTranslation().y);
		--RE.output2("ccon"..i, desired_height(i))
		--dbg.draw('Sphere', ee(i).bone:getTranslation(), "ccon".. i, "green", 3.0);
	end
	for i=0, #self.tails-1 do
		desired_tail_height:set(i, self.tails[i+1]:getTranslation().y)
	end
	local con=vector3N (numEE);
	local projectedCon=vector3N (numEE);

	local mTerrain=self.terrain
	local DEBUG_DRAW=false
	for i=0,numEE-1 do
		con(i):assign(ee(i).bone:getTranslation());
		projectedCon(i):assign(con(i));
		projectedCon(i).y=mTerrain:getTerrainHeight2(con(i))+desired_height(i);
		if DEBUG_DRAW then
			dbg.draw('Sphere', con(i), "ccon".. i, "green", 1.5);
			dbg.draw('Sphere', projectedCon(i), "ppcon".. i, "blue", 1.5);
		end
	end


	local numEE=4;
	projectedCon:resize(numEE);
	con:resize(numEE);

	local function addVertAxis(pos)
		local p1=pos:toVector3(0);
		local p2=pos:toVector3(3);
		local p3=pos:toVector3(6);
		local p4=pos:toVector3(9);

		local len=0;
		local dir1=p2-p1; len=len+dir1:length(); dir1:normalize();
		local dir2=p4-p3; len=len+dir2:length(); dir2:normalize();
		local dir3=p3-p1; len=len+dir3:length(); dir3:normalize();
		local dir4=p4-p2; len=len+dir4:length(); dir4:normalize();
		len=len/4.0;

		up=vector3 ();
		up:cross( (dir1+dir2)*0.5, (dir3+dir4)*0.5);

		pos:resize(pos:size()+3);
		pos:setVec3(pos:size()-3, (p1+p2+p3+p4)*0.25+up*len);
	end
	local metric=math.PointCloudMetric ();
	local pa=projectedCon:vecView():copy();
	addVertAxis(pa);
	local pb=con:vecView():copy();
	addVertAxis(pb);
	metric:calcDistance(pa, pb);

	local newRoot=transf ();
	newRoot:mult(transf(metric.transfB), pose:rootTransformation());

	local ROTATE_ROOT=true
	if ROTATE_ROOT then
		--ROTATE_ROOT
		-- root돌려준후 다시 4다리의 현재 동작에서 높이 구하기.
		pose:setRootTransformation(newRoot);
		skel:setPose(pose);
	end

	self.PFpose=Pose()
	self.PFpose:assign(pose)

	local onlyPlaneFitting =false
	if onlyPlaneFitting then
		input_pose:assign(pose)
		return 0
	end

	---------------------------------

	numEE=5;	-- 꼬리 추가됨.
	projectedCon:resize(numEE);
	con:resize(numEE);

	local DEBUG_DRAW2 = false
	for i=0,numEE-1 do
		con(i):assign(ee(i).bone:getTranslation());
		if DEBUG_DRAW2 then 
			dbg.draw('Sphere',con(i), "rcon".. i, "red", 3.0);
		end
		con(i).y=mTerrain:getTerrainHeight2( con(i))+desired_height(i);
		if DEBUG_DRAW2 then
			dbg.draw('Sphere',con(i), "bcon".. i, "blue", 3.0);
		end
	end

	local tails=self.tails
	local con_tail=vector3N (#tails);
	local DEBUG_DRAW3 = false
	for i=0, con_tail:size()-1 do
		con_tail(i):assign(tails[i+1]:getTranslation())
		local p=con_tail(i)
		p.y=math.max(p.y,mTerrain:getTerrainHeight2( p)+desired_tail_height(i))
		if DEBUG_DRAW3 then
			dbg.draw('Sphere',p, "tail".. i, "red", 3.0);
		end
	end

	local ik=self.ik

	--ik:IKsolve(pose, con, rot_joint_index, delta_rot);
	if self.IKsolve_sub then
		self:IKsolve_sub(pose, con, con_tail)

		--if not SKIN then
		--	SKIN=RE.createSkin(self.skel)
		--end
		--SKIN:setPose(pose, self.skel)

		--skel:setPose(pose)
		--local tails=self.tails
		--for i=1,#tails do
		--	local p= tails[i]:getTranslation():copy();
		--	p.y=math.max(p.y,mTerrain:getTerrainHeight2( p)+desired_tail_height(i-1))
		--end
	else
		ik:IKsolve(pose, con)
	end
	input_pose:assign(pose)
end
