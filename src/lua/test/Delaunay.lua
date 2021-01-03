require("test/DTW")

package.path=package.path..";../../taesooLib/Samples/scripts/?.lua"
require("tl")

function drawCorrespondences(v, v2, vi, vi2)
	local corr=vector3N()
	assert(vi:size()==vi2:size())
	for ii=0, vi:size()-1 do
		local i=vi(ii)
		local j=vi2(ii)
		if j>=0 then
			corr:pushBack(v(i))
			corr:pushBack(v2(j))
		end
	end
	dbg.namedDraw('Traj', corr:matView(), 'corr_edges', 'solidblue', thickness, 'BillboardLineList' )
end

function featureToMesh(filename, tf, debugDraw)

	local v=vector3N()
	local v2=vector3N()
	local frontToBack=intvectorn()
	if true then
		local frontToFeature=intvectorn()
		local fidx=intvectorn()
		local f=util.BinaryFile()
		f:openRead("featurepoints.dat")
		f:unpack(v)
		f:unpack(fidx)
		f:close()
		local v_o=v
		v=vector3N()
		v2=vector3N()

		for i=0, fidx:size()-1 do
			if fidx(i)>=0 then
				v:pushBack(v_o(i))
				frontToBack:pushBack(-1)
				frontToFeature:pushBack(i)
			else
				local j=i-1 -- corresponding front face
				assert( i>0)
				local dist=v_o(i):distance(v_o(j))
				if dist<1 then
					-- skip adding this problematic vertices
				else
					assert(frontToFeature(frontToFeature:size()-1)==j)
					frontToBack:set(frontToFeature:size()-1,v2:size())
					v2:pushBack(v_o(i))
				end
			end
		end

		v:leftMult(tf)
		v2:leftMult(tf)
		--drawCorrespondences(v, v2, tl.arange(v:size()), frontToBack)
	else
		assert(false)
	end
	local meshF=createDelaunayMesh(v,vector3(0,0,1))
	local meshB=createDelaunayMesh(v2,vector3(0,0,-1))

	local thickness=5
	local edges, edgesi=getBoundaryEdges(meshF)
	local edges2, edgesi2=getBoundaryEdges(meshB)

	local fi=edgesi2:findFirstIndex(frontToBack(edgesi(0)))
	assert(fi>=0 and fi<edgesi2:size())
	assert(edgesi2(fi)==frontToBack(edgesi(0)))
	edgesi2=edgesi2:slice(fi,0)..edgesi2:slice(0,fi)
	edgesi2=tl.reversed(edgesi2)

	local dist=function (x,y)
		local xx=x:copy()
		xx.z=0
		local yy=y:copy()
		yy.z=0
		return xx:distance(yy)
	end
	--local dtw=DTW(v:extract(edgesi), v2:extract(edgesi2), dist)
	local dtw=DTW(v:extract(edgesi), v2:extract(edgesi2))
	local out=dtw:calculate()
	local path=dtw:get_path()
	local i1=edgesi:extract(path:column(0))
	local i2=edgesi2:extract(path:column(1))

	if debugDraw then
		drawCorrespondences(v, v2, i1, i2)
		dbg.namedDraw('Traj', edges:matView(), 'boundary_edges', 'solidwhiteTrailZTest', thickness, 'BillboardLineList' )
		dbg.namedDraw('Traj', edges2:matView(), 'boundary_edgesB', 'solidwhiteTrailZTest', thickness, 'BillboardLineList' )
	end

	local mesh=Mesh()
	mesh:mergeMesh(meshF, meshB)

	local faces={}
	for ii=0, i1:size()-1 do
		local j1=i1(ii)
		local k1=i2(ii)+v:size()
		local j2,k2
		if ii==i1:size()-1 then
			-- loop
			j2=i1(0)
			k2=i2(0)+v:size()
		else
			j2=i1(ii+1)
			k2=i2(ii+1)+v:size()
		end

		if j1==j2 then
			assert(k1~=k2)
			table.insert(faces,{j1,k1,k2})
		elseif k1==k2 then
			assert(j1~=j2)
			table.insert(faces,{j1,k1,j2})
		else
			-- quad
			table.insert(faces,{j1,k1,j2})
			table.insert(faces,{j2,k1,k2})
		end
	end
	-- add connecting triangles
	local iface=mesh:numFace()
	mesh:resize(mesh:numVertex(), mesh:numFace()+#faces)
	for i, v in ipairs(faces) do
		mesh:getFace(iface+i-1):setIndex(v[1], v[2], v[3])
	end
	return mesh, v
end
