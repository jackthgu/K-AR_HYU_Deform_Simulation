require("config")

package.path=package.path..";../Samples/FlexibleBody/lua/?.lua" --;"..package.path
package.path=package.path..";../Resource/classification/lua/?.lua" --;"..package.path
package.path=package.path..";../Samples/QP_controller/lua/?.lua" --;"..package.path
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path
require("module")

background="terrain"
--background="plane"
if background=="terrain" then
	initialPosition=vector3(100,10,100)
else
	initialPosition=vector3(0,30,0)
end
gravity=-1000
scale=10
simulationFrameRate=120
renderingFrameRate=30
stiffness=20000
springDamp=30
dynamicFrictionCoef=0.3

--dbg.startTrace()
--dbg.startCount(110)
function ctor()
	this:create("Button", "start", "start")
	this:widget(0):buttonShortcut("FL_ALT+s")
	this:updateLayout()
end

function dtor()
	if mSimulator then
		mSimulator:exitWorld()
	end
end

function onCallback(w, userData)
	if w:id()=="start" then
		print("started")
		mObjectList=Ogre.ObjectList()
		mMesh=Mesh()
		mMeshBackup=Mesh()
		mMesh:loadOBJ("../src/lua/bunny_simplified1.obj")
		--mMesh:loadOBJ("../src/lua/bunnyobj_simplified2.obj")

		mSimulator=SimulatorImplicit()
		mSimulator:startWorld()
		local world=mSimulator
		RE.viewpoint():setFOVy(44.999999)


		rootnode =RE.ogreRootSceneNode()
		bgnode=RE.createChildSceneNode(rootnode , "BackgroundNode")

		RE.viewpoint():setClipDistances(0.5, 20)	-- decide where bunny falls from
		RE.viewpoint():setFOVy(45.000006)
		RE.viewpoint().vpos:assign(vector3(200, 100, 260))
		RE.viewpoint().vat:assign(vector3(0.220461, 20.121055, -5.625024))
		RE.viewpoint():update()

		-- 500kg, 80 cm
		-- 1000cm/s^2 = -10 m/s^2
		--dynamicFrictionCoef=10.3
		world:setParameter("mass", 50)
		world:setParameter("bendingConstraints1", 4)
		world:setParameter("stiffness", stiffness)
		world:setParameter("bendingStiffness", stiffness*0.5)

		-- INTEGRATE_EXPLICIT	:		0
		-- INTEGRATE_IMPLICIT	:		1
		-- INTEGRATE_SEMI_IMPLICIT:		2
		world:setParameter("integrateMethod", 1) -- implicit
		world:setParameter("springDamp", springDamp)
		world:setParameter("kDF", dynamicFrictionCoef)	-- dynamic friction coefficient
		world:setParameter("kDFvelThr", 0.01)
		world:setParameter("debugDrawWorld", 0)
		world:setParameter("drawSimulatedMesh", 1)
		world:setGravity(vector3(0, gravity, 0))


		if background=="terrain" then
			world:createTerrain("../Resource/crowdEditingScripts/terrain/heightmap1_256_256_2bytes.raw", vector3(-200,0, -200), 256, 256, 800, 800, 30)
		else
			world:createFloor(-15, vector3(400, 30, 400))
		end

		local m=matrix4()
		m:setScaling(scale,scale,scale)
		m:leftMultTranslation(initialPosition)
		mMesh:transform(m)

		mMeshBackup:assign(mMesh)

		mObjectList:registerMesh(mMesh, 'mMesh')
		mObjectList:findNode('mMesh'):setPosition(initialPosition)

		world:createSoftBody(mMesh)

		world:setParameter("drawContactForce", 1)
		world:setParameter("contactMethod",SimulatorImplicit.LCP_METHOD)
		world:setParameter("penaltyStiffness", 10000)

		--world.mSimulatedOgreMesh:getLastCreatedEntity():setMaterialName('white')

	end
	g_time=0
end

function renderOneFrame()
	noFrameMove=true
	RE.renderOneFrame(true)
	noFrameMove=nil
end

function onFrameChanged(currFrame)
end

function frameMove(fElapsedTime)
	if mSimulator and mSimulator:isWorldValid() then

		local nsubstep=simulationFrameRate/renderingFrameRate
		for i=1,nsubstep do
			local dt=1.0/simulationFrameRate

--input

			mMesh:assign(mMeshBackup)
			local m=matrix4()
			m:identity()
			local s=1+(math.sin(g_time)+1)*0.5
			print(s)
			m:setScaling(s,s,s)
			mMesh:transform(m)
		
			mObjectList:registerMesh(mMesh, 'mMesh')
			mObjectList:findNode('mMesh'):setPosition(initialPosition)

			mSimulator:changeSoftBodyRestLength(mMesh)
			mSimulator:stepSimulation(dt)
			g_time=g_time+dt
		end
		mSimulator:renderme()
	end
end
