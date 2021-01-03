
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
entityScale=0.5

function getVertex(mesh, vidx, baryCoeffs)
	--local f=mesh:getFace(fIdx)
	local v1=mesh:getVertex(vidx.x)
	local v2=mesh:getVertex(vidx.y)
	local v3=mesh:getVertex(vidx.z)
	local out2=v1*baryCoeffs.x+v2*baryCoeffs.y +v3*baryCoeffs.z
	return out2
end

Timeline=LUAclass(LuaAnimationObject)
function Timeline:__init(label, totalTime)
	self.totalTime=totalTime
	RE.renderer():addFrameMoveObject(self)
	self:attachTimer(1/30, totalTime)		
	RE.motionPanel():motionWin():addSkin(self)
end
EVR=LUAclass(EventReceiver)
function EVR:__init()
end
function EVR:onFrameChanged(w,i)
	print(i)
	drawFrame(i)
end
function drawFrame(i)
	local points=g_frames
	if i<#points -2 then
		local p1=points[i+1]
		local goal=matrixn(#p1,3)
		local goal2=matrixn(#p1,3)
		local lines=matrixn(#p1*2,3)
		for i,v in ipairs(p1) do
			goal:row(i-1):setVec3(0, v.a)
			-- v.fIdx, v.coeffs
			--local v2=getVertex(mMesh, v.vidx, v.coeffs)
			local v2=getVertex(mMesh, 
			vector3( g_vmap(v.vidx.x), g_vmap(v.vidx.y), g_vmap(v.vidx.z)), 
			v.coeffs
			)*100
			goal2:row(i-1):setVec3(0, v2)

			lines:row((i-1)*2):setVec3(0, v.a)
			lines:row((i-1)*2+1):setVec3(0, v2)

		end
		dbg.namedDraw('Traj', goal, 'goal2', 'blueCircle', thickness, 'QuadListV')
		--dbg.namedDraw('Traj', goal2, 'goal3', 'redCircle', thickness, 'QuadListV')
		dbg.namedDraw('Traj', lines, 'll', 'solidred', thickness, 'LineList')

		if this:findWidget("solve IK"):checkButtonValue() then

			mSC:clearConstraints()
			for i,v in ipairs(p1) do
				print('fidx', v.fIdx)
				assert(v.fIdx>=0)
				--mSC:addConstraints(v.vidx.x, v.vidx.y, v.vidx.z, v.coeffs)
				mSC:addConstraints(  g_vmap(v.vidx.x), g_vmap(v.vidx.y), g_vmap(v.vidx.z), v.coeffs)
			end

			local conPos=vector3N(#p1)
			for i,v in ipairs(p1) do
				conPos(i-1):assign(v.a/skinScale)
			end

			mSC:solveIK(conPos)
			redrawRedBox(mMesh, skinScale)
			if false then
				-- 실제 IK handle의 위치 그리기.
				for ii=0, mSC:numCon()-1 do
					local vpos=mSC:getConPos(ii)
					dbg.draw('Sphere', vpos*skinScale, 'vertex'..ii, 'red', 3)
				end
			end
		end
	end
end


function ctor()
	local osm=RE.ogreSceneManager()
	osm:setFogNone() -- turn off fog 

	-- turn off the checkboard floor
	local bgnode=RE.ogreSceneManager():getSceneNode("BackgroundNode")
	bgnode:flipVisibility()

	local sourceFile
	assert(RE.getOgreVersionMinor()>=12)
	-- initial selection. you can add or remove constraints using UI

	--sourceFile='sp_fbx.mesh' sourcePos=vector3(0,0.25,-2.5)
	sourceFile='mmSurface0_hor_fbx.mesh' sourcePos=vector3(0,0.25,-2)

	mSC=SurfaceControl(sourceFile, entityScale/skinScale, sourcePos)


	mMesh=mSC:getCurrMesh()
	redrawRedBox(mMesh, skinScale)




	local points, errmsg=loadfile('points.lua')
	assert(errmsg==nil)
	points=points()

	if true then
		--local mesh="../mesh/tri1_deformed.obj"
		--local mesh="../mesh/tri1.obj"
		local mesh="../data/mmSurface0.obj"
		local mesh2=Geometry()
		mesh2:loadOBJ(mesh)
		--mMeshEntity2 =mesh2:drawMesh('red_transparent', 'box_orig', true)

		-- vertex 개수부터 다름.
		-- 에라 모르겠다. 그냥 가장 가까운 vertex로 매핑하자.
		local vmap=intvectorn(mesh2:numVertex())

		for i=0, mesh2:numVertex()-1 do

			local argMin=-1
			local minDist=10000
			for j=0, mMesh:numVertex()-1 do

				local d=(mMesh:getVertex(j)*skinScale):distance(mesh2:getVertex(i))
				if d<minDist then
					minDist=d
					argMin=j
				end
			end
			assert(argMin~=-1)
			assert(minDist<1) -- less than 1cm
			print(minDist)
			vmap:set(i, argMin)
		end

		g_vmap=vmap
	end


	mTimeline=Timeline("Timeline", #points)
	mEventReceiver=EVR() -- also rename EVRecoder:_onFrameChanged to EVR:_onFrameChanged
	this:create("Check_Button", "solve IK", "solve IK")
	this:updateLayout()


	g_frames=points
	drawFrame(0)
end

function redrawRedBox(mMesh, node_scale)
	mMeshEntity =mMesh:drawMesh('lightgrey_transparent', 'redBox')
	local node=RE.getSceneNode('redBox')
	RE.moveEntity(node, quater(1,0,0,0), vector3(0,0,0))
	node:setScale(node_scale,node_scale,node_scale)
end

function dtor()
end


function onCallback(w, userData)
end


function frameMove(fElapsedTime)
end
