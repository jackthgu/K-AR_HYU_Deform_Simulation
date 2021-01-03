
-- lua implemetation of Dynamic Time Warpping (DTW)
-- See http://en.wikipedia.org/wiki/Dynamic_time_warping
--
-- usage:
--[[
function dist(x,y)
	return math.abs(x-y)
end

local a=DTW({1,3,5,7,9,11}, {3,10,11.5}, dist)
print( a:calculate())
printTable(a:get_path())
	]]

DTW=LUAclass()
MAP2D=LUAclass()
function MAP2D:__init()
	self._map={}
end
function MAP2D:set(i1, i2, v)
	local a=self._map[i1]
	if a then
		a[i2]=v
	else
		self._map[i1]={[i2]=v}
	end
end
function MAP2D:get(i1, i2)
	local a=self._map[i1]
	if a then
		return a[i2]
	else
		return nil
	end
end
function DTW. __init(self, seq1, seq2, distance_func)
	--[[
	seq1, seq2 are two lists,
	distance_func is a function for calculating
	the local distance between two elements.
	]]
	if type(seq1)~='table' then
		local _seq1={}
		for i=0, seq1:size()-1 do
			table.insert(_seq1, seq1(i))
		end
		local _seq2={}
		for i=0, seq2:size()-1 do
			table.insert(_seq2, seq2(i))
		end
		self._seq1 = _seq1
		self._seq2 = _seq2
	else
		self._seq1 = seq1
		self._seq2 = seq2
	end
	self.inf=1e10
	self._distance_func = distance_func

	if distance_func==nil then
		self._distance_func=function (x,y)
			return x:distance(y)
		end
	end
	self._map = MAP2D()
	self._map:set(-1,-1,0.0)
	self._distance_matrix = MAP2D()
	self._path = {}
end

function DTW. get_distance(self, i1, i2)
	local ret = self._distance_matrix:get(i1, i2)
	if not ret then
		ret = self._distance_func(self._seq1[i1+1], self._seq2[i2+1])
		self._distance_matrix:set(i1, i2, ret)
	end
	return ret
end

function DTW.min(self, a,b,c, comp)
	if comp(self, a)< comp(self,b) then
		if comp(self,a)<comp(self,c) then
			return a
		else
			return c
		end
	else
		if comp(self,b)<comp(self,c) then
			return b
		else
			return c
		end
	end
end
function DTW. calculate_backward(self, i1, i2)
	assert(i1)
	assert(i2)
	--[[
	Calculate the dtw distance between
	seq1[:i1 + 1] and seq2[:i2 + 1]
	]]
	if self._map:get(i1, i2) then
		return self._map:get(i1, i2)
	end

	if i1 == -1 or i2 == -1 then
		self._map:set(i1, i2, self.inf)
		return self.inf
	end

	local out = self:min({i1 - 1, i2}, {i1, i2 - 1}, {i1 - 1, i2 - 1}, 
						function (self, x) return self:calculate_backward(x[1], x[2]) end)
	local min_i1, min_i2=unpack(out)

	self._map:set(i1, i2, self:get_distance(i1, i2) + self:calculate_backward(min_i1, min_i2))

	return self._map:get(i1, i2)
end

function DTW. get_path(self)
	--[[
	Calculate the path mapping.
	Must be called after calculate()
	]]
	local i1, i2 = #self._seq1 - 1, #self._seq2 - 1
	while not (i1==-1 and i2==-1) do
		table.insert(self._path,{i1, i2})
		local out = self:min({i1 - 1, i2}, {i1, i2 - 1}, {i1 - 1, i2 - 1},
						function (self, x) return self._map:get(x[1], x[2]) end)
		i1, i2 = unpack(out)
	end
	local out=intmatrixn(#self._path, 2)
	for i=0, out:rows()-1 do
		out:set(i, 0, self._path[out:rows()-i][1])
		out:set(i, 1, self._path[out:rows()-i][2])
	end
	return out
end

function DTW. calculate(self)
	return self:calculate_backward(#self._seq1 - 1, #self._seq2 - 1)
end

function cross(v1,v2)
	local v=vector3()
	v:cross(v1,v2)
	return v
end
function vector3N:leftMult(o)
	for i=0, self:rows()-1 do
		self(i):assign(o*self(i))
	end
end
function createDelaunayMesh(v, normalPrior)
	local mesh=Mesh()
	mesh:resize(v:size(),0)
	for i=0,mesh:numVertex()-1 do
		mesh:getVertex(i):assign(v(i))
	end

	mesh:delaunayTriangulation()
	if true then
		-- align normals
		for i=0,mesh:numFace()-1 do
			local f=mesh:getFace(i)
			local vi1=f:vertexIndex(1)
			local vi2=f:vertexIndex(2)
			local v0=mesh:getVertex(f:vertexIndex(0))
			local v1=mesh:getVertex(vi1)
			local v2=mesh:getVertex(vi2)
			local normal=cross(v1-v0, v2-v0)
			normal:normalize()
			if normal:dotProduct(normalPrior)<0 then
				f:setVertexIndex(1,vi2)
				f:setVertexIndex(2,vi1)
			end
		end
	end
	return mesh
end
function drawMesh(mesh, name, drawNormals)
	local out={}
	out.mesh=mesh
	-- draw mesh
	if false then
		local meshToEntity=MeshToEntity(mesh, name, false,true)
		meshToEntity:updatePositions()
		local entity=meshToEntity:createEntity(name..'entityName' ,'red_transparent_ccw')

		local node=RE.createChildSceneNode(RE.ogreRootSceneNode(), name.."mesh_node")
		node:attachObject(entity)
		out.viewer=meshToEntity
	end
	if true then
		-- draw mesh edges
		pNode=RE.ogreRootSceneNode():createChildSceneNode(name.."meshNode");
		--pMeshDeformed=MeshLineStripReduced();
		pMeshDeformed=MeshLineStrip();
		--pMeshDeformed:setMaterial("solidred");

		pMeshDeformed:assignMesh(mesh)
		pNode:attachObject(pMeshDeformed);
		pMeshDeformed:firstInit();
	end
	if drawNormals then

		local normals=vector3N()

		-- draw normals
		for i=0,mesh:numFace()-1 do
			local f=mesh:getFace(i)
			local v0=mesh:getVertex(f:vertexIndex(0))
			local v1=mesh:getVertex(f:vertexIndex(1))
			local v2=mesh:getVertex(f:vertexIndex(2))
			local center=(v0+v1+v2)/3.0
			local normal=cross(v1-v0, v2-v0)
			normal:normalize()
			normals:pushBack(center)
			normals:pushBack(center+normal*10)
		end

		dbg.namedDraw('Traj', normals:matView(), name..'normals')
	end
	return out
end
function getBoundaryEdges(mesh, drawBoundaryVertices)
	local ec=EdgeConnectivity(mesh)
	local mc=OBJloader.MeshConnectivity(mesh, ec)

	if drawBoundaryVertices then
		-- draw boundary vertices
		for i=0, mesh:numVertex()-1 do
			if mesh.isBoundaryVertex(i) then
				dbg.draw("Sphere", mesh:getVertex(i), "bd"..i, 'blue', 3)
				local fe=mc:vertexToFace(i)
				print('vertex '..i, fe:source(mesh), fe:target(mesh),  fe:cross(mesh))
				assert(mc:prev(fe):isNull())
			end
		end
	end
	-- draw boundary edges
	local fBV -- the first boundary vertex
	for i=0, mesh:numVertex()-1 do
		if mesh.isBoundaryVertex(i) then
			fBV=i
			break
		end
	end
	assert(fBV)
	local fe=mc:vertexToFace(fBV)
	local edges=vector3N()
	local edgesi=intvectorn()
	repeat
		print(fe:source(mesh))
		assert(mesh.isBoundaryVertex(fe:source(mesh)))
		fe=mc:vertexToFace(fe:target(mesh))

		edges:pushBack(mesh:getVertex(fe:source(mesh)))
		edges:pushBack(mesh:getVertex(fe:target(mesh)))
		edgesi:pushBack(fe:source(mesh))
	until fe:source(mesh)==fBV

	return edges,edgesi
end
