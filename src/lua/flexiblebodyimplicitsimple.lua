background="terrain"
--background="plane"

RE.viewpoint():setFOVy(44.999999)

rootnode =RE.ogreRootSceneNode()
bgnode=RE.createChildSceneNode(rootnode , "BackgroundNode")

scale_factor=1

RE.viewpoint():setClipDistances(0.5, 1000)
RE.viewpoint():setFOVy(45.000006)
RE.viewpoint().vpos:assign(vector3(93.776154, 64.218978, 124.498157))
RE.viewpoint().vat:assign(vector3(0.220461, 20.121055, -5.625024))
RE.viewpoint():update()

RE.viewpoint():update()

-- 500kg, 80 cm
-- 1000cm/s^2 = -10 m/s^2
gravity=-1000
simulationFrameRate=240
stiffness=20000
springDamp=10
dynamicFrictionCoef=1
--dynamicFrictionCoef=10.3
world:setParameter("mass", 50)
world:setParameter("bendingConstraints1", 4)
world:setParameter("stiffness", stiffness)
world:setParameter("bendingStiffness", stiffness*0.5)

	-- INTEGRATE_EXPLICIT	:		0
	-- INTEGRATE_IMPLICIT	:		1
	-- INTEGRATE_SEMI_IMPLICIT:		2
world:setParameter("integrateMethod",1) -- implicit
world:setParameter("springDamp",springDamp)
world:setParameter("kDF", dynamicFrictionCoef)	-- dynamic friction coefficient
world:setParameter("kDFvelThr", 0.01)
world:setParameter("debugDrawWorld",0)
world:setParameter("drawSimulatedMesh",1)
world:setGravity(vector3(0, gravity, 0))

if background=="terrain" then
	initialPosition=vector3(0,20,0)
else
	initialPosition=vector3(0,30,0)
end

if background=="terrain" then
	world:createTerrain("../Resource/crowdEditingScripts/terrain/heightmap1_256_256_2bytes.raw", vector3(-200,0, -200), 256, 256, 800, 800, 30)
else
	world:createFloor(-15, vector3(400, 30, 400))
end

--	debug.debug()

win:createTargetMesh(0, scale_factor, initialPosition);
world:createSoftBody(win:targetMesh())

world:setParameter("drawContactForce", 1)
--world:setParameter("contactMethod",SimulatorImplicit.PENALTY_METHOD)
--world:setParameter("contactMethod",SimulatorImplicit.LCP_METHOD_FRICTIONLESS)
world:setParameter("contactMethod",SimulatorImplicit.LCP_METHOD)
--world:setParameter("contactMethod",SimulatorImplicit.BARAFF98)
world:setParameter("penaltyStiffness", 10000)

local resourcePath="../src/lua/"
world:adjustSoftBodyParam("dynamicFrictionCoef", resourcePath.."iguanaAll.selection", 0)
--world:adjustSoftBodyParam("dynamicFrictionCoef", resourcePath.."iguanaTail.selection", 0)
world:adjustSoftBodyParam("dynamicFrictionCoef", resourcePath.."leftFoot.selection", dynamicFrictionCoef)
world:adjustSoftBodyParam("dynamicFrictionCoef", resourcePath.."rightFoot.selection", dynamicFrictionCoef)
world:adjustSoftBodyParam("dynamicFrictionCoef", resourcePath.."leftHand.selection", dynamicFrictionCoef)
world:adjustSoftBodyParam("dynamicFrictionCoef", resourcePath.."rightHand.selection", dynamicFrictionCoef)

world:adjustSoftBodyParam("stiffness", resourcePath.."leftFoot.selection", stiffness*2)
world:adjustSoftBodyParam("stiffness", resourcePath.."rightFoot.selection", stiffness*2)
world:adjustSoftBodyParam("stiffness", resourcePath.."leftHand.selection", stiffness*2)
world:adjustSoftBodyParam("stiffness", resourcePath.."rightHand.selection", stiffness*2)
