
require("config")

package.projectPath='../Samples/classification/'
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path

require("module")

function ctor()
	-- see testdrawing.lua also
	this:updateLayout()

	do
		-- draw mesh
		local mesh=Geometry()
		local numSegX=1
		local numSegZ=1

		mesh:loadMesh("../mesh/iguana_bound.tri",false)
		local meshToEntity=MeshToEntity(mesh, 'meshName')
		meshToEntity:updatePositionsAndNormals()
		local entity=meshToEntity:createEntity('entityName' )
		entity:setMaterialName("lightgrey_transparent")
		local node=RE.createChildSceneNode(RE.ogreRootSceneNode(), "nodeName")
		node:attachObject(entity)
		--node:translate(0,60,0)

		-- visualization scale
		local s=1
		node:scale(s,s,s)

		mMesh=mesh

		mEdges=EdgeConnectivity(mesh)
		local edges=intmatrixn()
		edges:resize(mEdges:numEdges(), 2)
		for i=0,mEdges:numEdges()-1 do
			local si=mEdges:source(i)
			local ti=mEdges:target(i)
			edges:set(i, 0, si)
			edges:set(i, 1, ti)
		end

		local function drawEdges(edges, title, mat)
			local goal=matrixn(edges:rows()*2, 3)
			for i=0, edges:rows()-1 do
				local si=edges(i,0)
				local ti=edges(i,1)
				goal:row(i*2):setVec3(0, mMesh:getVertex(si))
				goal:row(i*2+1):setVec3(0, mMesh:getVertex(ti))
			end
			dbg.namedDraw('Traj', goal*s, title, mat, 0, 'LineList')
		end
		drawEdges(edges, 'edges','solidred')


		local thr=13
		--local thr=7
		local springs=edges:copy()
		for i=0, mMesh:numVertex()-1 do
			for j=i+1, mMesh:numVertex()-1 do
				if mMesh:getVertex(i):distance(mMesh:getVertex(j))<thr then
					if not mEdges:isConnected(i,j) then
						springs:pushBack(CT.ivec(i,j))
					end
				end
			end
		end
		drawEdges(springs, 'springs','solidgreen')
	end

end

function dtor()
end

function onCallback(w, userData)
	if w:id()=="set desired speed" then
		mSpeed=w:sliderValue()
	elseif w:id()=="set desired position" then
		mPos=w:sliderValue()
	else
		this:updateFloatOptions(w)
	end
   
end

			
			
function frameMove(fElapsedTime)
end
