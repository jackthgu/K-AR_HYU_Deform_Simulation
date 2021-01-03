
require("config")
package.projectPath='../Samples/classification/'
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path
require("common")
require("module")

require("RigidBodyWin/subRoutines/Constraints")

package.path=package.path..";../../taesooLib/Samples/scripts/?.lua" --;"..package.path
package.path=package.path..";../../taesooLib/Samples/scripts/RigidBodyWin/?.lua" --;"..package.path
package.path=package.path..";../../taesooLib/Samples/classification/lua/?.lua" --;"..package.path

require("subRoutines/AnimOgreEntity")
PoseTransfer2=require("subRoutines/PoseTransfer2")
require("Kinematics/meshTools")

skinScale=100 -- meter / cm
entityScale=1
skelScale=1

function getVertex(mesh, vidx, baryCoeffs)
	--local f=mesh:getFace(fIdx)
	local v1=mesh:getVertex(vidx.x)
	local v2=mesh:getVertex(vidx.y)
	local v3=mesh:getVertex(vidx.z)
	local out2=v1*baryCoeffs.x+v2*baryCoeffs.y +v3*baryCoeffs.z
	return out2
end
function ctor()
	mEventReceiver=EVR()

	local sourceFile
	assert(RE.getOgreVersionMinor()>=12)
	-- initial selection. you can add or remove constraints using UI

	--sourceFile='sp_fbx.mesh' skelScale=100
	sourceFile='mmSurface0_hor_fbx.mesh' skelScale=100

	-- create mOrigLoaderu
	mOrigLoader=SkinnedMeshLoader(sourceFile, false, false)
	mOrigLoader:printHierarchy()
	mOrigLoader:gotoBindPose()

	
	-- copy the current skeleton pose to the skin
	local newPose=Pose()
	mOrigLoader:getPose(newPose)


	-- step 1. copy the bad skeleton
	
	if dbg.lunaType(mOrigLoader)~='MainLib.VRMLloader' then
		mLoader=mOrigLoader:toVRMLloader()
	else
		mLoader=mOrigLoader:copy()  
	end

	mLoader:removeAllRedundantBones() -- clean up the skeleton
	mLoader:setPose(newPose)
	mLoader:setCurPoseAsInitialPose() -- adjust skeleton so that the current pose becomes the identity pose (all local joint orientations are identity quaternions)
	mOrigOffset=mLoader:bone(1):getOffsetTransform().translation
	mLoader:bone(1):getOffsetTransform().translation:zero()
	MotionUtil.removeSlidingJoints(mLoader)


	-- you can further simplify mLoader as long as all the joint positions are kept.
	-- for example, you can change rotational channels so that knee cannot twist, and so on. (see skeletonEditor_GUI.lua)
	-- after that, create a mapping (PT) between the two skeletons 

	PT=PoseTransfer2(mLoader, mOrigLoader)
	mSkinLoader=mOrigLoader
	for i=1, mSkinLoader:numBone()-1 do
		print(i, mSkinLoader:getDerivedScale(i), mSkinLoader:getBindingPoseInverseScale(i))
	end

	local numVertex=mSkinLoader:getCurrMesh():numVertex()
	
	mSkinningInfo=SkinnedMeshFromVertexInfo(mSkinLoader)
	for i=0, numVertex-1 do
		local treeIndices=mSkinningInfo:treeIndices(i)
		local localpos=mSkinningInfo:localPos(i)
		
		-- localpos needs to be converted from mSkinLoader space to mLoader space.
		for j=0, localpos:size()-1 do
			local frameA=mSkinLoader:bone(treeIndices(j)):getFrame()
			local frameB=mLoader:bone(treeIndices(j)):getFrame()
			localpos(j):assign(frameB:inverse()*((frameA*localpos(j))*(entityScale/skinScale)))
		end
	end

	-- -- manually selected vertices (IK handles)
	local vertices={ {0}, {1}, {2},{3}, {4}, {numVertex-2}, {numVertex-1}}

	for ii, vertex in ipairs(vertices) do
		vertex[2]=mSkinningInfo:treeIndices(vertex[1])
		vertex[3]=mSkinningInfo:localPos(vertex[1])
		vertex[4]=mSkinningInfo:weights(vertex[1])
	end

	g_vertices=vertices


	mPose0=Pose()
	mLoader:getPose(mPose0)
	-- now let's attach an IK solver and IK handles (empty)
	mSolverInfo=createQuatIKsolver(mLoader, {}) -- changes the pose.

	mLoader:setPose(mPose0)
	mMesh=mSkinLoader:getCurrMesh():copy() -- scale이 달라서 vertex 위치 업데이트 필요.
	mSkinningInfo:calcVertexPositions(mLoader, mMesh);
	redrawRedBox(mMesh, skinScale)


	local vertexPositions=vector3N(#g_vertices)
	for ii, vertex in ipairs(g_vertices) do
		local vpos=vector3(0,0,0)
		local _, treeIndices, localpos, weights=unpack(vertex)
		for j=0, weights:size()-1 do
			vpos:radd((mLoader:bone(treeIndices(j)):getFrame()*localpos(j))*weights(j))
			print(mSkinLoader:bone(treeIndices(j)):getFrame(), mLoader:bone(treeIndices(j)):getFrame())
		end
		vertexPositions(ii-1):assign(vpos + vector3(0, 0, 0))
		--dbg.draw('Sphere', vpos*skinScale, 'vertex'..ii, 'red', 10)
	end

	mCON=Constraints(vertexPositions*skinScale)
	mCON:connect(eventFunction)


end

function redrawRedBox(mMesh, node_scale)
	mMeshEntity =mMesh:drawMesh('lightgrey_transparent', 'redBox')
	local node=RE.getSceneNode('redBox')
	RE.moveEntity(node, quater(1,0,0,0), vector3(0,0,0))
	node:setScale(node_scale,node_scale,node_scale)
end
function eventFunction()
	limbik()
end
function limbik()
	local mIK=mSolverInfo.solver
	local mEffectors=mSolverInfo.effectors
	local mPose=vectorn()
	mLoader:setPose(mPose0);
	mLoader:getPoseDOF(mPose)
	mIK:_changeNumEffectors(0)
	mIK:_changeNumConstraints(#g_vertices) 

	for ii, vertex in ipairs(g_vertices) do
		local _, treeIndices, localpos, weights=unpack(vertex)
		local desired_pos=mCON.conPos(ii-1)/skinScale

		--dbg.draw('Sphere', desired_pos*entityScale, 'vertex'..ii, 'red', 10)

		mIK:_setSkinningConstraint(ii-1, treeIndices, localpos, weights, desired_pos) 
	end

	mIK:_effectorUpdated()
	local footPos=vector3N()
	mIK:IKsolve(mPose, footPos)
	assert(not mPose:isnan())

	mLoader:setPoseDOF(mPose)
	mSkinningInfo:calcVertexPositions(mLoader, mMesh);
	redrawRedBox(mMesh, skinScale)
	if true then
		-- 실제 IK handle의 위치 그리기.
		for ii, vertex in ipairs(g_vertices) do
			local vpos=vector3(0,0,0)
			local _, treeIndices, localpos, weights=unpack(vertex)
			for j=0, weights:size()-1 do
				vpos:radd((mLoader:bone(treeIndices(j)):getFrame()*localpos(j))*weights(j))
			end
			dbg.draw('Sphere', vpos*skinScale, 'vertex'..ii, 'red', 3)
		end
	end
	--dbg.draw("Sphere", mOrigLoader:bone(1):getFrame().translation*skinScale, 'root', 'gree', 15)


end

function createQuatIKsolver(loader, config)
	local out={}
	local mEffectors=MotionUtil.Effectors()
	local numCon=#config
	mEffectors:resize(numCon);
	out.effectors=mEffectors
	out.numCon=numCon

	for i=0, numCon-1 do
		local conInfo=config[i+1]
		mEffectors(i):init(loader:getBoneByName(conInfo[1]), conInfo[2])
	end
	out.solver=MotionUtil.createFullbodyIk_MotionDOF_MultiTarget_lbfgs(loader.dofInfo);

	out.solver:setParam("damping_weight", 0,0.01)
	return out
end
function dtor()
end

function handleRendererEvent(ev, button, x, y)
	if mCON then
		local res=mCON:handleRendererEvent(ev, button, x,y)
		print(ev, res)

		if res==0 then
			if ev=="MOVE" or ev=="PUSH" then

				local out=vector3()
				-- use proper ray pick
				local ray=Ray()
				RE.FltkRenderer():screenToWorldRay(x, y,ray)

				ray:scale(1/skinScale) -- change to meter unit.
				local mesh=mMesh

				local out=vector3()
				local baryCoeffs=vector3()
				local ti=ray:pickBarycentric(mesh, baryCoeffs, out)

				print(ti)
				if ti>=0 then
					-- 픽 성공, 위치는 out에 저장됨.

					-- 어떤 triangle이고 bary centric coordinate는 어디인지는 ti와 baryCoeffs에 저장됨.
					local f=mesh:getFace(ti)
					local v1=mesh:getVertex(f:vertexIndex(0))
					local v2=mesh:getVertex(f:vertexIndex(1))
					local v3=mesh:getVertex(f:vertexIndex(2))

					local out2=v1*baryCoeffs.x+v2*baryCoeffs.y +v3*baryCoeffs.z
					-- 그려보면 bary centric interpolation결과(out2)와 out이 동일함
					dbg.draw('Sphere', out2*skinScale, "cursor", "green",5)
					if ev=="PUSH" then
						mCON:addCON(out2*skinScale)
						--mCON:addCON(v1*skinScale)

						if true then
							-- use c++ implementation
							local info={ -1, intvectorn(), vector3N(), vectorn()}
							mSkinningInfo:getVertexInfo( f:vertexIndex(0), f:vertexIndex(1), f:vertexIndex(2),baryCoeffs, info[2], info[3], info[4]);
							table.insert(g_vertices, info)
						else
							function SkinnedMeshFromVertexInfo:getSkinningInfo(vertexIndex)
								local info={}
								info[1]=vertexIndex -- unused
								info[2]=self:treeIndices(vertexIndex)
								info[3]=self:localPos(vertexIndex)
								info[4]=self:weights(vertexIndex)
								return info
							end
							local info1=mSkinningInfo:getSkinningInfo(f:vertexIndex(0))
							local info2=mSkinningInfo:getSkinningInfo(f:vertexIndex(1))
							local info3=mSkinningInfo:getSkinningInfo(f:vertexIndex(2))

							-- now merge these three infos into one.
							--
							-- basic idea
							-- (M*lpos+b)*w1+(M*lpos2+b)*w2
							-- = M*(lpos*w1+lpos2*w2)+b*(w1+w2)
							-- = (M*(lpos*w1+lpos2*w)/(w1+w2) + b)*(w1+w2)

							local info={
								-1,
								info1[2]:copy(), -- ti
								info1[3]:copy(), -- lpos
								info1[4]*baryCoeffs.x, -- weights
							}

							local ti=info[2]
							for i=0, ti:size()-1 do
								info[3](i):scale(info[4](i)) -- lpos*w1
							end

							local others={ info2, info3}
							local bc={ baryCoeffs.y, baryCoeffs.z}

							local temp=info[4]:copy()
							for i, oinfo  in ipairs(others) do
								local tio=info[2]
								local ti=oinfo[2]
								local b=bc[i]
								for i=0, ti:size()-1 do
									local fi=tio:findFirstIndex(ti(i))
									if fi==-1 then
										info[2]:pushBack(ti(i))
										local w=oinfo[4](i)*b
										info[3]:pushBack(oinfo[3](i)*w)
										info[4]:pushBack(w)
									else
										local w=oinfo[4](i)*b
										info[3](fi):radd(oinfo[3](i)*w)
										info[4]:set(fi, info[4](fi)+w)
									end
								end
							end

							local ti=info[2]
							for i=0, ti:size()-1 do
								info[3](i):scale(1/info[4](i)) -- (lpos*w1 +lpos2*w2)/(w1+w2)
							end
							table.insert(g_vertices, info)
						end
					end
				end
				return 1
			end

		else
			dbg.erase('Sphere', "cursor")
		end
	end
	return 0
end

function onCallback(w, userData)
end

if EventReceiver then
	EVR=LUAclass(EventReceiver)
	function EVR:__init(graph)
		self.currFrame=0
		self.cameraInfo={}
	end
end

function EVR:onFrameChanged(win, iframe)
	self.currFrame=iframe

	RE.output("iframe", iframe)

end

function frameMove(fElapsedTime)
end
