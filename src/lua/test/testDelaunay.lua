

   
require("config")
require("module")
require("common")
require("cameraTracker")
require("RigidBodyWin/subRoutines/Constraints")
require("test/Delaunay")



function ctor()
	this:updateLayout()

	local tf=matrix4()
	tf:identity()
	tf:setRotation(quater(math.rad(130), vector3(0,1,0)))

	local mesh, v=featureToMesh("featurepoints.dat", tf, true)

	dbg.draw('Mesh', mesh, 'mesh', 'red_transparent')

	g_meshViewer=drawMesh(mesh, 'meshV')

	mCON=Constraints(v)
	mCON:connect(eventFunction)
	mCON.size=2
	mCON:drawConstraints()

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
	if mCON then
		return mCON:handleRendererEvent(ev, button, x,y)
	end
	return 0
end
