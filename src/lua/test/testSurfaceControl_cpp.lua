
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
function ctor()
	mEventReceiver=EVR()

	local sourceFile
	assert(RE.getOgreVersionMinor()>=12)
	-- initial selection. you can add or remove constraints using UI

	sourceFile='sp_fbx.mesh' sourcePos=vector3(0,0.25,-2.5)
	--sourceFile='mmSurface0_hor_fbx.mesh' sourcePos=vector3(0,0.25,-2)


	mSC=SurfaceControl(sourceFile, entityScale/skinScale, sourcePos)

	local numVertex=mSC:getCurrMesh():numVertex()
	-- -- manually selected vertices (IK handles)
	local vertices={ {0}, {648}, {219}, {229}, {numVertex-4}, {numVertex-3}, {numVertex-2}, {numVertex-1}}

	for ii, vertex in ipairs(vertices) do
		mSC:addConstraints(vertex[1])
	end

	g_vertices=vertices

	mMesh=mSC:getCurrMesh()
	redrawRedBox(mMesh, skinScale)


	local vertexPositions=vector3N(#g_vertices)
	for ii, vertex in ipairs(g_vertices) do
		local vpos=mSC:getConPos(ii-1)
		vertexPositions(ii-1):assign(vpos + vector3(0, 0, 0))
	end

	mCON=Constraints(vertexPositions*skinScale)
	mCON:connect(eventFunction)

	if true then
		local mesh="../data/mmSurface0.obj"
		local mesh2=Geometry()
		mesh2:loadOBJ(mesh)
		mMeshEntity2 =mesh2:drawMesh('red_transparent', 'box_orig', true)
	end
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
	mSC:solveIK(mCON.conPos*(1.0/skinScale))
	redrawRedBox(mMesh, skinScale)
	if true then
		-- 실제 IK handle의 위치 그리기.
		for ii=0, mCON.conPos:size()-1 do
			local vpos=mSC:getConPos(ii)
			dbg.draw('Sphere', vpos*skinScale, 'vertex'..ii, 'red', 3)
		end
	end
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
						mSC:addConstraints(f:vertexIndex(0), f:vertexIndex(1), f:vertexIndex(2),baryCoeffs);
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
