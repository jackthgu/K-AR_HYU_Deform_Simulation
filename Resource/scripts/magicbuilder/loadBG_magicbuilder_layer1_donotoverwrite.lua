RE.viewpoint():setFOVy(45.000002)
RE.viewpoint().vpos:assign({5023.647316, 2305.585893, -1863.907754})
RE.viewpoint().vat:assign({1020.866260, -723.543574, 364.613501})
RE.viewpoint():update()

rootnode =RE.ogreRootSceneNode()
bgnode=RE.createChildSceneNode(rootnode , "Floor_layer1")

pNode=RE.createChildSceneNode(bgnode, "l1_entity2")
l1_entity=RE.ogreSceneManager():createEntity("_l1_entity_l1_entity2", "mountin.mesh")
pNode:setPosition(-950.000000, -200.000000, -1400.000000)
pNode:setOrientation(1.000000, 0.000000, 0.000000, 0.000000)
pNode:setScale(1.000000, 1.000000, 1.000000)
l1_entity:setNormaliseNormals(true)
pNode:attachObject(l1_entity)

pNode=RE.createChildSceneNode(bgnode, "l1_h6622")
l1_entity=RE.ogreSceneManager():createEntity("_l1_entity_h6622", "h66.mesh")
pNode:setPosition(-50.000000, 50.000000, 750.000000)
pNode:setOrientation(1.000000, 0.000000, 0.000000, 0.000000)
pNode:setScale(1.000000, 1.000000, 1.000000)
l1_entity:setNormaliseNormals(true)
pNode:attachObject(l1_entity)

pNode=RE.createChildSceneNode(bgnode, "l1_h6632")
l1_entity=RE.ogreSceneManager():createEntity("_l1_entity_h6632", "h66.mesh")
pNode:setPosition(50.000000, 50.000000, 750.000000)
pNode:setOrientation(1.000000, 0.000000, 0.000000, 0.000000)
pNode:setScale(1.000000, 1.000000, 1.000000)
l1_entity:setNormaliseNormals(true)
pNode:attachObject(l1_entity)

pNode=RE.createChildSceneNode(bgnode, "l1_h552")
l1_entity=RE.ogreSceneManager():createEntity("_l1_entity_h552", "h55.mesh")
pNode:setPosition(900.000000, 150.000000, 700.000000)
pNode:setOrientation(1.000000, 0.000000, 0.000000, 0.000000)
pNode:setScale(1.000000, 1.000000, 1.000000)
l1_entity:setNormaliseNormals(true)
pNode:attachObject(l1_entity)

pNode=RE.createChildSceneNode(bgnode, "l1_h36")
l1_entity=RE.ogreSceneManager():createEntity("_l1_entity_h36", "h36.mesh")
pNode:setPosition(1200.000000, 50.000000, 700.000000)
pNode:setOrientation(1.000000, 0.000000, 0.000000, 0.000000)
pNode:setScale(1.000000, 1.000000, 1.000000)
l1_entity:setNormaliseNormals(true)
pNode:attachObject(l1_entity)

pNode=RE.createChildSceneNode(bgnode, "l1_h475")
l1_entity=RE.ogreSceneManager():createEntity("_l1_entity_h475", "h47.mesh")
pNode:setPosition(0.000000, 100.000000, 600.000000)
pNode:setOrientation(0.707107, 0.000000, -0.707107, 0.000000)
pNode:setScale(1.000000, 1.000000, 1.000000)
l1_entity:setNormaliseNormals(true)
pNode:attachObject(l1_entity)

pNode=RE.createChildSceneNode(bgnode, "l1_h4722")
l1_entity=RE.ogreSceneManager():createEntity("_l1_entity_h4722", "h47.mesh")
pNode:setPosition(0.000000, 100.000000, 700.000000)
pNode:setOrientation(0.707107, 0.000000, -0.707107, 0.000000)
pNode:setScale(1.000000, 1.000000, 1.000000)
l1_entity:setNormaliseNormals(true)
pNode:attachObject(l1_entity)

pNode=RE.createChildSceneNode(bgnode, "l1_h52222")
l1_entity=RE.ogreSceneManager():createEntity("_l1_entity_h52222", "h52.mesh")
pNode:setPosition(0.000000, 250.000000, 700.000000)
pNode:setOrientation(0.707107, 0.000000, -0.707107, 0.000000)
pNode:setScale(1.000000, 1.000000, 1.000000)
l1_entity:setNormaliseNormals(true)
pNode:attachObject(l1_entity)

pNode=RE.createChildSceneNode(bgnode, "l1_h523")
l1_entity=RE.ogreSceneManager():createEntity("_l1_entity_h523", "h52.mesh")
pNode:setPosition(0.000000, 250.000000, 600.000000)
pNode:setOrientation(0.707107, 0.000000, 0.707107, 0.000000)
pNode:setScale(1.000000, 1.000000, 1.000000)
l1_entity:setNormaliseNormals(true)
pNode:attachObject(l1_entity)

pNode=RE.createChildSceneNode(bgnode, "l1_h454")
l1_entity=RE.ogreSceneManager():createEntity("_l1_entity_h454", "h45.mesh")
pNode:setPosition(900.000000, 50.000000, 700.000000)
pNode:setOrientation(1.000000, 0.000000, 0.000000, 0.000000)
pNode:setScale(1.000000, 1.000000, 1.000000)
l1_entity:setNormaliseNormals(true)
pNode:attachObject(l1_entity)

pNode=RE.createChildSceneNode(bgnode, "l1_h513")
l1_entity=RE.ogreSceneManager():createEntity("_l1_entity_h513", "h51.mesh")
pNode:setPosition(250.000000, 250.000000, -350.000000)
pNode:setOrientation(1.000000, 0.000000, 0.000000, 0.000000)
pNode:setScale(1.000000, 1.000000, 1.000000)
l1_entity:setNormaliseNormals(true)
pNode:attachObject(l1_entity)

pNode=RE.createChildSceneNode(bgnode, "l1_h532")
l1_entity=RE.ogreSceneManager():createEntity("_l1_entity_h532", "h53.mesh")
pNode:setPosition(0.000000, 250.000000, -350.000000)
pNode:setOrientation(0.000000, 0.000000, 1.000000, 0.000000)
pNode:setScale(1.000000, 1.000000, 1.000000)
l1_entity:setNormaliseNormals(true)
pNode:attachObject(l1_entity)

pNode=RE.createChildSceneNode(bgnode, "l1_h433")
l1_entity=RE.ogreSceneManager():createEntity("_l1_entity_h433", "h43.mesh")
pNode:setPosition(-50.000000, 50.000000, -400.000000)
pNode:setOrientation(0.000000, 0.000000, 1.000000, 0.000000)
pNode:setScale(1.000000, 1.000000, 1.000000)
l1_entity:setNormaliseNormals(true)
pNode:attachObject(l1_entity)

pNode=RE.createChildSceneNode(bgnode, "l1_h5224")
l1_entity=RE.ogreSceneManager():createEntity("_l1_entity_h5224", "h52.mesh")
pNode:setPosition(350.000000, 250.000000, -350.000000)
pNode:setOrientation(1.000000, 0.000000, 0.000000, 0.000000)
pNode:setScale(1.000000, 1.000000, 1.000000)
l1_entity:setNormaliseNormals(true)
pNode:attachObject(l1_entity)

pNode=RE.createChildSceneNode(bgnode, "l1_h4734")
l1_entity=RE.ogreSceneManager():createEntity("_l1_entity_h4734", "h47.mesh")
pNode:setPosition(200.000000, 100.000000, -300.000000)
pNode:setOrientation(0.707107, 0.000000, -0.707107, 0.000000)
pNode:setScale(1.000000, 1.000000, 1.000000)
l1_entity:setNormaliseNormals(true)
pNode:attachObject(l1_entity)

pNode=RE.createChildSceneNode(bgnode, "l1_h4122")
l1_entity=RE.ogreSceneManager():createEntity("_l1_entity_h4122", "h41.mesh")
pNode:setPosition(350.000000, 50.000000, -300.000000)
pNode:setOrientation(1.000000, 0.000000, 0.000000, 0.000000)
pNode:setScale(1.000000, 1.000000, 1.000000)
l1_entity:setNormaliseNormals(true)
pNode:attachObject(l1_entity)

pNode=RE.createChildSceneNode(bgnode, "l1_h443")
l1_entity=RE.ogreSceneManager():createEntity("_l1_entity_h443", "h44.mesh")
pNode:setPosition(350.000000, 50.000000, -300.000000)
pNode:setOrientation(1.000000, 0.000000, 0.000000, 0.000000)
pNode:setScale(1.000000, 1.000000, 1.000000)
l1_entity:setNormaliseNormals(true)
pNode:attachObject(l1_entity)

pNode=RE.createChildSceneNode(bgnode, "l1_h4422")
l1_entity=RE.ogreSceneManager():createEntity("_l1_entity_h4422", "h44.mesh")
pNode:setPosition(-50.000000, 150.000000, -400.000000)
pNode:setOrientation(0.000000, 0.000000, 1.000000, 0.000000)
pNode:setScale(1.000000, 1.000000, 1.000000)
l1_entity:setNormaliseNormals(true)
pNode:attachObject(l1_entity)

pNode=RE.createChildSceneNode(bgnode, "l1_h47322")
l1_entity=RE.ogreSceneManager():createEntity("_l1_entity_h47322", "h47.mesh")
pNode:setPosition(-50.000000, 100.000000, -250.000000)
pNode:setOrientation(0.000000, 0.000000, 1.000000, 0.000000)
pNode:setScale(1.000000, 1.000000, 1.000000)
l1_entity:setNormaliseNormals(true)
pNode:attachObject(l1_entity)

pNode=RE.createChildSceneNode(bgnode, "l1_h47332")
l1_entity=RE.ogreSceneManager():createEntity("_l1_entity_h47332", "h47.mesh")
pNode:setPosition(50.000000, 100.000000, -250.000000)
pNode:setOrientation(0.000000, 0.000000, 1.000000, 0.000000)
pNode:setScale(1.000000, 1.000000, 1.000000)
l1_entity:setNormaliseNormals(true)
pNode:attachObject(l1_entity)

pNode=RE.createChildSceneNode(bgnode, "l1_h47423")
l1_entity=RE.ogreSceneManager():createEntity("_l1_entity_h47423", "h47.mesh")
pNode:setPosition(300.000000, 100.000000, -400.000000)
pNode:setOrientation(0.707107, 0.000000, 0.707107, 0.000000)
pNode:setScale(1.000000, 1.000000, 1.000000)
l1_entity:setNormaliseNormals(true)
pNode:attachObject(l1_entity)

pNode=RE.createChildSceneNode(bgnode, "l1_h4142")
l1_entity=RE.ogreSceneManager():createEntity("_l1_entity_h4142", "h41.mesh")
pNode:setPosition(350.000000, 150.000000, -300.000000)
pNode:setOrientation(0.707107, 0.000000, 0.707107, 0.000000)
pNode:setScale(1.000000, 1.000000, 1.000000)
l1_entity:setNormaliseNormals(true)
pNode:attachObject(l1_entity)

pNode=RE.createChildSceneNode(bgnode, "l1_h5122")
l1_entity=RE.ogreSceneManager():createEntity("_l1_entity_h5122", "h51.mesh")
pNode:setPosition(150.000000, 250.000000, -350.000000)
pNode:setOrientation(1.000000, 0.000000, 0.000000, 0.000000)
pNode:setScale(1.000000, 1.000000, 1.000000)
l1_entity:setNormaliseNormals(true)
pNode:attachObject(l1_entity)

pNode=RE.createChildSceneNode(bgnode, "l1_h52232")
l1_entity=RE.ogreSceneManager():createEntity("_l1_entity_h52232", "h52.mesh")
pNode:setPosition(0.000000, 250.000000, -200.000000)
pNode:setOrientation(0.707107, 0.000000, -0.707107, 0.000000)
pNode:setScale(1.000000, 1.000000, 1.000000)
l1_entity:setNormaliseNormals(true)
pNode:attachObject(l1_entity)

pNode=RE.createChildSceneNode(bgnode, "l1_h474222")
l1_entity=RE.ogreSceneManager():createEntity("_l1_entity_h474222", "h47.mesh")
pNode:setPosition(100.000000, 100.000000, -400.000000)
pNode:setOrientation(0.707107, 0.000000, 0.707107, 0.000000)
pNode:setScale(1.000000, 1.000000, 1.000000)
l1_entity:setNormaliseNormals(true)
pNode:attachObject(l1_entity)

