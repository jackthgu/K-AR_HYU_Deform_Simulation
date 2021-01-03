require("config")
require("module")
require("common")
require("cameraTracker")
package.path=package.path..';../scripts/RigidBodyWin/subRoutines/?.lua'
require("Timeline")
require("cmaesOptimizer")

function ctor()
	mEventReceiver=EVR()
	mTimeline=Timeline("Timeline", 10000)
	this:create('Value_Slider', 'activation', 'activation')

	this:widget(0):sliderRange(0,1);
	this:widget(0):sliderValue(0)

	this:updateLayout()
	mSoftWorld=IntegratedWorld(1.0/60.0, 0.95) -- timestep, damping
	mMesh=TetraMeshLoader.TetraMesh()

	mMeshCompare = tetraMeshLoader.TetraMesh()
	mMeshCompare.loadMesh("../mesh/iamgroot101_surface.1by10.tet")
	--mMesh:loadMesh("../mesh/iguana.tet")
	--mMesh:loadMesh("../mesh/fandisk_1.tet")
	mMesh:loadMesh("../mesh/iamgroot101_surface.1by10.tet")
	local s=matrix4()
	s:setScaling(0.1,0.1,0.1)
	--s:setTranslation(vector(0,10,0))
	mMesh:transform(s)
	mSoftWorld:addBody('iguana', mMesh, 30)
	local stiffness=1E5
	local nodeIndex=20
	--mSoftWorld:addAttachmentConstraint('c0', stiffness, nodeIndex, mMesh:nodePos(nodeIndex))
	mSoftWorld:addAttachmentConstraint('c0', stiffness, nodeIndex, mMesh:nodePos(nodeIndex) )

	for i=0, mMesh:getNumTetra()-1 do
		local muscleStiffness=4E5
		local fiber_direction=vector3(1,1,1)
		local imuscle=mSoftWorld:addLinearMuscleConstraint('m'..tostring(i), muscleStiffness, i, mMesh, fiber_direction)
		assert(imuscle==i)
	end

	mSoftWorld:initSimulation()
	redraw()
end
local currTime=0
function frameMove(fElapsedTime)
end

function redraw()
	local cst=vector3N()
	mSoftWorld:getAttachmentCst(cst)
	dbg.draw('Traj', cst:matView()*100, 'a_cst', 'blueCircle', 10, 'QuadListV')


	local elts=vector3N()
	mSoftWorld:getCorotateFEMCst(elts)
	--dbg.draw('Traj', elts:matView()*100,'elts')
	RE.output("elts", elts(0), elts(1), elts(elts:size()-1))
end
function onCallback(w, userData)
	if w:id()=='activation' then
		local v=w:sliderValue()
		--for im=0, mMesh:getNumTetra()-1 do
		for im=10, 30 do
			mSoftWorld:setActivationLevel(im, v)
		end
	end
end
function dtor()
	mTimeline:dtor()
end

function EVR:onFrameChanged(win, iframe)
	mSoftWorld:TimeStepping()
	redraw()
end
