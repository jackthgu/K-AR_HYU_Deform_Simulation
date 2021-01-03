
   
require("config")
require("module")
require("common")
require("cameraTracker")
package.path=package.path..';../scripts/RigidBodyWin/subRoutines/?.lua'
require("Timeline")
function ctor()
	mEventReceiver=EVR()
	mTimeline=Timeline("Timeline", 10000)
	this:updateLayout()
	mSoftWorld=IntegratedWorld(1.0/180.0, 0.99) -- timestep, damping
	mSoftWorld:addBody("muscle_params.xml")
	mSoftWorld:initSimulation()
	redraw()
end
local currTime=0
function frameMove(fElapsedTime)
end

function redraw()
	local cst=vector3N()
	mSoftWorld:getAttachmentCst(cst)
	dbg.draw('Traj', cst:matView()*100, 'a_cst', 'blueCircle', 3, 'QuadListV')

	local elts=vector3N()
	mSoftWorld:getCorotateFEMCst(elts)
	dbg.draw('Traj', elts:matView()*100,'elts')
	RE.output("elts", elts(0), elts(1), elts(elts:size()-1))
end
function onCallback(w, userData)
end
function dtor()
	mTimeline:dtor()
end

function EVR:onFrameChanged(win, iframe)
	for i=1,3 do
		-- timeline runs at 60hz
		-- 180 == 60*3 
		mSoftWorld:TimeStepping()
	end
	redraw()
end
