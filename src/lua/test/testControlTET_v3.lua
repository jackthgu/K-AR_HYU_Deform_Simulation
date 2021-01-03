
   
require("config")
require("module")
require("common")
require("cameraTracker")
require("RigidBodyWin/subRoutines/Constraints")
--g_method='InvDeform'
g_method='FwdDeform'

function getVertices(mesh)
	local o=vector3N(mesh:numVertex())
	for i=0, mesh:numVertex()-1 do
		o(i):assign(mesh:getVertex(i))
	end
	return o
end
function setVertices(mesh, o)
	for i=0, mesh:numVertex()-1 do
		mesh:getVertex(i):assign(o(i))
	end
end

function ctor()
	this:updateLayout()
	local bgnode=RE.ogreSceneManager():getSceneNode("BackgroundNode") 
	bgnode:setVisible(false)
	local meshname='../mesh/tri0'
	--local meshname='../mesh/testgroot'
	-- draw mesh
	local mesh=Mesh()
	mesh:loadOBJ(meshname..".surface.obj")
	local meshToEntity=MeshToEntity(mesh, 'meshName', false,true)
	meshToEntity:updatePositions()
	local entity=meshToEntity:createEntity('entityName' ,'red_transparent_ccw')

	local node=RE.createChildSceneNode(RE.ogreRootSceneNode(), "mesh_node")
	node:attachObject(entity)

	local meshHigh=Mesh()
	meshHigh:loadOBJ(meshname..".obj")
	v=getVertices(meshHigh)
	meshHighDeformed=Mesh()
	meshHighDeformed:loadOBJ(meshname.."_deformed.obj")


	g_mesh=mesh
	g_meshOrig=mesh:copy()
	g_meshViewer=meshToEntity

	if g_method=='invDeform' then
		--g_deformer=LinearDeformer.createKNNIDWDeformer(30,2.0)
		g_deformer=LinearDeformer.createMeanValueDeformer()
		g_deformer:calculateCorrespondence(g_meshOrig, v)
		g_invDeformer=InverseDeformer(g_deformer, v)
	else
		g_deformer=LinearDeformer.createKNNIDWDeformer(10,2.0, 0.01)
		g_featureMesh=Mesh()
		g_featureMesh:resize(v:size(), 0)
		setVertices(g_featureMesh, v)
		g_vertices=getVertices(g_meshOrig)
		g_vertices_orig=g_vertices:copy()
		g_deformer:calculateCorrespondence(g_featureMesh,g_vertices )
	end
	eventFunction()
end

function eventFunction()
	if g_method=='invDeform' then
		g_invDeformer:invDeform(mCON.conPos, g_meshOrig, g_mesh)
	else
		local temp=getVertices(meshHighDeformed)
		local thickness=1
		-- too many high-mesh vertices, so let's not draw it
		--dbg.namedDraw('Traj', temp:matView(), 'goal2', 'blueCircle', thickness, 'QuadListZ')
		if false then
			local ll=vector3N()
			for i=0, v:size()-1 do
				ll:pushBack(v(i))
				ll:pushBack(temp(i))
			end
			dbg.draw('Traj',ll:matView(), 'lines')
		end
		setVertices(g_featureMesh, temp)
		g_deformer:transfer(g_featureMesh,1.0)
		dbg.namedDraw('Traj', g_vertices:matView(), 'goal3', 'greenCircle', thickness, 'QuadListZ')
		dbg.namedDraw('Traj', g_vertices_orig:matView(), 'goal4', 'redCircle', thickness, 'QuadListZ')
		setVertices(g_mesh, g_vertices)
	end
	g_meshViewer:updatePositions()
end

function frameMove(fElapsedTime)
end
function onCallback(w, userData)
end
function dtor()
end
function handleRendererEvent(ev, button, x,y) 
	return 0
end
