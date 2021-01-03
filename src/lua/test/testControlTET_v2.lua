
   
require("config")
require("module")
require("common")
require("cameraTracker")
require("RigidBodyWin/subRoutines/Constraints")
require("test/Delaunay")
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

	local tf=matrix4()
	tf:identity()
	tf:setRotation(quater(math.rad(130), vector3(0,1,0)))

	local featureMesh, v=featureToMesh("featurepoints.dat", tf)
	featureMesh:transform(tf:inverse())
	v:leftMult(tf:inverse())

	if false then
		local mc=featureMesh:calcMeshCenter()
		dbg.draw('Sphere', mc, 'mesh_center')

		tf:identity()
		tf:leftMultTranslation(-mc)
		local s=1.2
		tf:leftMultScaling(s,s,s)
		tf:leftMultTranslation(mc)

		featureMesh:transform(tf)
	end


	mCON=Constraints(v)
	mCON:connect(eventFunction)
	mCON.size=2
	mCON:drawConstraints()

	g_mesh=mesh
	g_meshOrig=mesh:copy()
	g_meshViewer=meshToEntity

	g_meshViewerF=drawMesh(featureMesh, 'meshV', false)

	if true then
		g_deformer=LinearDeformer.createKNNIDWDeformer(10,2.0, 0.01)
		g_featureMesh=Mesh()
		g_featureMesh:resize(v:size(), 0)
		setVertices(g_featureMesh, v)
		g_vertices=getVertices(g_meshOrig)
		g_deformer:calculateCorrespondence(g_featureMesh,g_vertices )
	else
		g_featureMesh=featureMesh
		--g_deformer=LinearDeformer.createMeanValueDeformer()
		g_deformer=LinearDeformer.createMeanValueDeformer()
		--g_deformer=LinearDeformer.createHarmonicDeformer()
		g_vertices=getVertices(g_meshOrig)
		g_deformer:calculateCorrespondence(g_featureMesh,g_vertices )
	end
end

function eventFunction()
	do
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
