

   
require("config")
require("module")
require("common")
require("cameraTracker")
require("RigidBodyWin/subRoutines/Constraints")
require("test/Delaunay")
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

	local meshA=Mesh()
	meshA:loadOBJ('../mesh/tri0.obj')
	local meshB=Mesh()
	meshB:loadOBJ('../mesh/tri0_deformed.obj')

	dbg.draw('Mesh', meshA, 'meshA', 'red_transparent')
	--dbg.draw('Mesh', meshB, 'meshB', 'green_transparent')

	g_meshViewer=drawMesh(meshB, 'meshB')


	local va=getVertices(meshA)
	local vb=getVertices(meshB)
	drawCorrespondences(va,vb, tl.arange(va:size()), tl.arange(va:size()))

end
function eventFunction(ev, i, arg)
	RE.output2('event', ev, i)
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
