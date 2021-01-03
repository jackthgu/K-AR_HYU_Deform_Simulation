
   
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

	-- draw mesh
	local mesh=Mesh()
	mesh:loadOBJ("../mesh/testgroot.surface.obj")
	local meshToEntity=MeshToEntity(mesh, 'meshName', false,true)
	meshToEntity:updatePositions()
	local entity=meshToEntity:createEntity('entityName' ,'red_transparent_ccw')

	local node=RE.createChildSceneNode(RE.ogreRootSceneNode(), "mesh_node")
	node:attachObject(entity)

	local f=util.BinaryFile()
	f:openRead("featurepoints.dat")
	local v=vector3N()
	f:unpack(v)
	f:close()
	mCON=Constraints(v)
	mCON:connect(eventFunction)
	mCON.size=2
	mCON:drawConstraints()

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
		g_deformer:calculateCorrespondence(g_featureMesh,g_vertices )
	end
end

function eventFunction()
	if g_method=='invDeform' then
		g_invDeformer:invDeform(mCON.conPos, g_meshOrig, g_mesh)
	else
		setVertices(g_featureMesh, mCON.conPos)
		g_deformer:transfer(g_featureMesh,1.0)
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
	if mCON then
		return mCON:handleRendererEvent(ev, button, x,y)
	end
	return 0
end
