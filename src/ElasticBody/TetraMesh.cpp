#include "stdafx.h"
#include "TetraMesh.h"
#include "Mesh_old.h"
#include "sgeom.h"
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include "MainLib/OgreFltk/RE.h"
#include "MainLib/OgreFltk/pldprimskin.h"

using namespace TetraMeshLoader;
//=============================================================
//                 TetraMesh
//=============================================================

void TetraMesh::compareMesh(TetraMesh dest)
{
    std::vector<int> result_diff;
    for (int i=0 ; i < dest.getNumTetra(); i++)
    {
	result_diff.push_back(this->nodePos(i).distance(dest.nodePos(i)));
    }

}


bool TetraMesh::loadMesh(const char *file_name_)
{
	int n, m;
	vector3 x_tmp;
	std::ifstream fin;

	// ---------------- read mesh information from file -------------------

	fin.open(file_name_);

	if ( !fin.is_open() ) return false;

	// get the number of nodes and mesh elements
	fin >> n >> m;	

	_vmesh_vertex.resize(n);
	_vmesh_index.resize(m);

	// get initial node positions from file
	for (int i=0; i<n; i++) {
		// node position in {mesh file reference frame}
		fin >> x_tmp.x >> x_tmp.y >> x_tmp.z;

		_vmesh_vertex[i].pos=x_tmp;
	}

	// get mesh indexes from file
	for (int i=0; i<m; i++) {
		fin >> _vmesh_index[i][0] >> _vmesh_index[i][1] >> _vmesh_index[i][2] >> _vmesh_index[i][3];
	}

	fin.close();

	normalizeIndex();

/*
	// ---------------- find neighbors of each node -------------------

	// scan neighbors for each node
	std::vector< std::vector<Node*> > list_neighbors;
	list_neighbors.resize(n);
	for (int i=0; i<m; i++) {
		for (int j=0; j<4; j++) {
			for (int k=0; k<4; k++) {

				std::vector<Node*> & l=list_neighbors[_vmesh_index[i][j]];

				if ( std::find(l.begin(), l.end(), &(_vmesh_vertex[_vmesh_index[i][k]])) ==l.end() ) {
					list_neighbors[_vmesh_index[i][j]].push_back(&(_vmesh_vertex[_vmesh_index[i][k]]));
				}
			}
		}
	}

	// set neighbors
	for (int i=0; i<n; i++) {
		_vmesh_vertex[i].setNeighbors(list_neighbors[i]);
	}
*/
	return true;
}

bool TetraMesh::saveMesh(const char *file_name_)
{
	int n, m;
	vector3 x_tmp;
	std::ofstream fout;

	// ---------------- read mesh information from file -------------------

	fout.open(file_name_);

	if ( !fout.is_open() ) return false;

	// get the number of nodes and mesh elements

	n=_vmesh_vertex.size();
	m=_vmesh_index.size();
	fout << n <<" "<< m<<std::endl;	

	// get initial node positions from file
	for (int i=0; i<n; i++) {
		x_tmp=_vmesh_vertex[i].pos;

		// node position in {mesh file reference frame}
		fout << x_tmp.x <<" "<< x_tmp.y <<" "<< x_tmp.z<<std::endl;

	}

	// get mesh indexes from file
	for (int i=0; i<m; i++) {
		fout << _vmesh_index[i][0] <<" "<< _vmesh_index[i][1] <<" "<< _vmesh_index[i][2] <<" "<< _vmesh_index[i][3]<<std::endl;
	}

	fout.close();
}

vector3& TetraMesh::getTetraCorner(int iTetra, int icorner) 
{
	return nodePos(getTetra(iTetra)[icorner]);
}

_tvector<int,3> TetraMesh::getTetraFace(int i, int iface) const
{
	_tvector<int,3> vi;
	switch(iface)
	{
	case 0:
		vi[0]=_vmesh_index[i][0];
		vi[1]=_vmesh_index[i][1];
		vi[2]=_vmesh_index[i][2];
		break;
	case 1:
		vi[0]=_vmesh_index[i][0];
		vi[1]=_vmesh_index[i][2];
		vi[2]=_vmesh_index[i][3];
		break;
	case 2:
		vi[0]=_vmesh_index[i][0];
		vi[1]=_vmesh_index[i][3];
		vi[2]=_vmesh_index[i][1];
		break;
	case 3:
		vi[0]=_vmesh_index[i][3];
		vi[1]=_vmesh_index[i][2];
		vi[2]=_vmesh_index[i][1];
		break;
	}
	return vi;
}

vector3 TetraMesh::calcTetraCenter(int i) const
{
	vector3 const& v1=nodePos(getTetra(i)(0));
	vector3 const& v2=nodePos(getTetra(i)(1));
	vector3 const& v3=nodePos(getTetra(i)(2));
	vector3 const& v4=nodePos(getTetra(i)(3));

	return (v1+v2+v3+v4)/4.0;
}

void TetraMesh::normalizeIndex(int start, int end)
{
	if(end>getNumTetra()) end=getNumTetra();

	for(int i=start; i<end; i++)
	{		
		std::vector<_quadi> shuffle;
				
		shuffle.push_back(_quadi(0,1,2,3));
		shuffle.push_back(_quadi(0,1,3,2));
		shuffle.push_back(_quadi(0,2,1,3));
		shuffle.push_back(_quadi(0,2,3,1));
		shuffle.push_back(_quadi(0,3,1,2));
		shuffle.push_back(_quadi(0,3,2,1));
		shuffle.push_back(_quadi(1,0,2,3));
		shuffle.push_back(_quadi(1,0,3,2));
		shuffle.push_back(_quadi(1,2,0,3));
		shuffle.push_back(_quadi(1,2,3,0));
		shuffle.push_back(_quadi(1,3,0,2));
		shuffle.push_back(_quadi(1,3,2,0));
		shuffle.push_back(_quadi(2,0,1,3));
		shuffle.push_back(_quadi(2,0,3,1));
		shuffle.push_back(_quadi(2,1,0,3));
		shuffle.push_back(_quadi(2,1,3,0));
		shuffle.push_back(_quadi(2,3,0,1));
		shuffle.push_back(_quadi(2,3,1,0));
		shuffle.push_back(_quadi(3,0,1,2));
		shuffle.push_back(_quadi(3,0,2,1));
		shuffle.push_back(_quadi(3,1,0,2));
		shuffle.push_back(_quadi(3,1,2,0));
		shuffle.push_back(_quadi(3,2,0,1));
		shuffle.push_back(_quadi(3,2,1,0));

		int iter;
		for(iter=0; iter<shuffle.size(); iter++)
		{
			// tetrahedron.
			//  0
			// /|\
			// 123

			// normalize tetra index so that faces (012) (023) (031) and (321) face outside.
			vector3 p0=_vmesh_vertex[_vmesh_index[i][shuffle[iter][0]]].pos;
			vector3 p1=_vmesh_vertex[_vmesh_index[i][shuffle[iter][1]]].pos;
			vector3 p2=_vmesh_vertex[_vmesh_index[i][shuffle[iter][2]]].pos;
			vector3 p3=_vmesh_vertex[_vmesh_index[i][shuffle[iter][3]]].pos;

			vector3 n0 = Cross(p1-p0, p2-p0); n0.normalize();
			vector3 n1 = Cross(p2-p0, p3-p0); n1.normalize();
			vector3 n2 = Cross(p3-p0, p1-p0); n2.normalize();
			vector3 n3 = Cross(p3-p1, p2-p1); n3.normalize();

			if( Inner(n0, p3-p0) < 0 && 
				Inner(n1, p1-p0) < 0 &&
				Inner(n2, p2-p0) < 0 &&
				Inner(n3, p0-p1) < 0 )
				break;
		}

		ASSERT(iter!=shuffle.size());

		_quadi newIndex;
		newIndex[0]=_vmesh_index[i][shuffle[iter][0]];
		newIndex[1]=_vmesh_index[i][shuffle[iter][1]];
		newIndex[2]=_vmesh_index[i][shuffle[iter][2]];
		newIndex[3]=_vmesh_index[i][shuffle[iter][3]];

		_vmesh_index[i]=newIndex;
		
	}
}


void TetraMesh::transform(matrix4 const& b)
{
	quater q;
	q.setRotation(b);

	for(int i=0; i<_vmesh_vertex.size(); i++)
	{
		_vmesh_vertex[i].pos.leftMult(b);
	}
}

using namespace Ogre;

#include <OgreMeshManager.h>
#include <OgreSubMesh.h>
#include <OgreRoot.h>
// Ogre::Mesh와 MeshLoader::Mesh간의 자료교환을 담당하는 클래스.


TetraMeshToEntity::TetraMeshToEntity(const TetraMesh& mesh, const char* meshId, Option option)
:mInputMesh(mesh)
{
	mSavedOption=option;
	
	if(MeshManager::getSingleton().resourceExists(meshId))
	{
		Ogre::MeshPtr pmesh=MeshManager::getSingleton().getByName(meshId);
		mMesh=(Ogre::Mesh*)pmesh.get();

		Ogre::ResourcePtr ptr=MeshManager::getSingleton().getByName(meshId);

		if(mMesh->sharedVertexData->vertexCount != mesh.getNumTetra()*12)
		{
			MeshManager::getSingleton().remove(ptr);
		}
		else
			return;
	}
	mMesh= (Ogre::Mesh*)MeshManager::getSingleton().createManual(meshId, ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME).get();
	SubMesh *pMeshVertex = mMesh->createSubMesh();

	mMesh->sharedVertexData = new VertexData();
	VertexData* vertexData = mMesh->sharedVertexData;

	// define the vertex format
	VertexDeclaration* vertexDecl = vertexData->vertexDeclaration;
	size_t currOffset = 0;
	// positions
	vertexDecl->addElement(0, currOffset, VET_FLOAT3, VES_POSITION);
	currOffset += VertexElement::getTypeSize(VET_FLOAT3);

	if(option.useNormal)
	{
		// normals
		vertexDecl->addElement(0, currOffset, VET_FLOAT3, VES_NORMAL);
		currOffset += VertexElement::getTypeSize(VET_FLOAT3);
	}

	if(option.useRandomColor)
	{
		// two dimensional texture coordinates
		vertexDecl->addElement(0, currOffset, VET_COLOUR, VES_DIFFUSE);
		currOffset += VertexElement::getTypeSize(VET_COLOUR);
	}
		
	HardwareVertexBuffer::Usage usage=HardwareBuffer::HBU_STATIC_WRITE_ONLY;
	HardwareVertexBuffer::LockOptions lockOption=HardwareBuffer::HBL_DISCARD;
	if(option.dynamicUpdate)
	{
		usage=HardwareBuffer::HBU_DYNAMIC ;
		lockOption=HardwareBuffer::HBL_NORMAL ;
	}

	// allocate the vertex buffer
	vertexData->vertexCount = mesh.getNumTetra()*12;
	HardwareVertexBufferSharedPtr vBuf = HardwareBufferManager::getSingleton().createVertexBuffer(vertexDecl->getVertexSize(0), vertexData->vertexCount, usage, false);
	VertexBufferBinding* binding = vertexData->vertexBufferBinding;
	binding->setBinding(0, vBuf);

	unsigned char* vertex=static_cast<unsigned char*>(vBuf->lock(lockOption));
	//float* prPos = static_cast<float*>(vBuf->lock(lockOption));

	// allocate index buffer
	pMeshVertex->indexData->indexCount = mesh.getNumTetra()*4*3;
	pMeshVertex->indexData->indexBuffer = HardwareBufferManager::getSingleton().createIndexBuffer(HardwareIndexBuffer::IT_16BIT, pMeshVertex->indexData->indexCount, usage, false);
	HardwareIndexBufferSharedPtr iBuf = pMeshVertex->indexData->indexBuffer;
	unsigned short* pIndices = static_cast<unsigned short*>(iBuf->lock(lockOption));

	
	// Drawing stuff 
	Ogre::AxisAlignedBox box;

	const VertexDeclaration::VertexElementList& elemList = vertexDecl->getElements();

	RenderSystem* rs = Root::getSingleton().getRenderSystem();
	ASSERT(rs);

	int current_vertex_index=0;

	srand(0);
	for(int i=0; i<mesh.getNumTetra(); i++)
	{
		CPixelRGBA c;	// random tetra color
		c.R=rand()%255;
		c.G=rand()%255;
		c.B=rand()%255;
		c.A=255;

		for(int f=0; f<4; f++)
		{
			_tvector<int,3> vi=mesh.getTetraFace(i, f);

			vector3 normal;
			vector3 v0=mesh._vmesh_vertex[vi[0]].pos;
			vector3 v1=mesh._vmesh_vertex[vi[1]].pos;
			vector3 v2=mesh._vmesh_vertex[vi[2]].pos;

			normal.cross(v1-v0,v2-v0);
			normal.normalize();

			for(int v=0; v<3; v++)
			{

				for (VertexDeclaration::VertexElementList::const_iterator elem = elemList.begin();
					elem != elemList.end(); ++elem)
				{
					float* pFloat=0;
					RGBA* pRGBA=0;
					switch(elem->getType())
					{
					case VET_FLOAT1:
					case VET_FLOAT2:
					case VET_FLOAT3:
						elem->baseVertexPointerToElement(vertex, &pFloat);
						break;
					case VET_COLOUR:
					case VET_COLOUR_ABGR:
					case VET_COLOUR_ARGB:
						elem->baseVertexPointerToElement(vertex, &pRGBA);
						break;
					default:
						// nop ?
						break;
					};

					switch(elem->getSemantic())
					{
					case VES_POSITION:
						{
							vector3 const& pp=mesh._vmesh_vertex[vi[v]].pos;
							*pFloat++ = pp.x;
							*pFloat++ = pp.y;
							*pFloat++ = pp.z;
							box.merge(Ogre::Vector3(pp.x, pp.y, pp.z));
						}
						break;
					case VES_NORMAL:
						ASSERT(option.useNormal);
						*pFloat++ = normal.x;
						*pFloat++ = normal.y;
						*pFloat++ = normal.z;
						break;
					case VES_DIFFUSE:
						ASSERT(option.useRandomColor);
						{
							Ogre::ColourValue colour(
								float(c.R)/float(255), 
								float(c.G)/float(255), 
								float(c.B)/float(255), 
								float(c.A)/float(255));
							rs->convertColourValue(colour, pRGBA++);
						}
						break;
					default:
						// nop ?
						break;
					};
				}

				vertex += vBuf->getVertexSize();
				*pIndices++=current_vertex_index++;
			}
		}
	}

	ASSERT(current_vertex_index==mesh.getNumTetra()*12);

	vBuf->unlock(); 
	iBuf->unlock();

	// Generate face list
	pMeshVertex ->useSharedVertices=true;

	//mMesh->_setBounds(Ogre::AxisAlignedBox(ToOgre(min), ToOgre(max)), false);
	//mMesh->_setBoundingSphereRadius(min.distance(max)*2);
	
	mMesh->_setBounds(box);
	mMesh->_setBoundingSphereRadius(box.getMinimum().distance(box.getMaximum()));

	mMesh->load();
	
	if(option.buildEdgeList)
	{
		mMesh->buildEdgeList();
	}
}

void TetraMeshToEntity::updatePositions()
{
	/*
	Option& option=mSavedOption;

	const VertexElement* posElem = mMesh->sharedVertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
	HardwareVertexBufferSharedPtr vBuf = mMesh->sharedVertexData->vertexBufferBinding->getBuffer(posElem->getSource());

	unsigned char* vertex=static_cast<unsigned char*>(vBuf->lock(HardwareBuffer::HBL_NORMAL ));

	const MeshLoader::Mesh& mesh=mInputMesh;

	float* prPos;
	// Drawing stuff 
	vector3 min, max;

	min.x=min.y=min.z=FLT_MAX;
	max.x=max.y=max.z=-FLT_MAX;

	Ogre::AxisAlignedBox box;

	for(int i=0; i<mesh.numVertex(); i++)
	{
		vector3 const& pp=mesh.m_arrayVertex[i].pos;
		posElem->baseVertexPointerToElement(vertex, &prPos);

		*prPos++ = pp.x; 
		*prPos++ = pp.y; 
		*prPos++ = pp.z; 
		
		box.merge(Ogre::Vector3(pp.x, pp.y, pp.z));

		vertex += vBuf->getVertexSize();
	}
	vBuf->unlock();

	mMesh->_setBounds(box);
	mMesh->_setBoundingSphereRadius(box.getMinimum().distance(box.getMaximum()));

	if(option.buildEdgeList)
	{
		mMesh->freeEdgeList();
		mMesh->buildEdgeList();
	}
*/

}


void TetraMeshToEntity::updatePositionsAndNormals()
{
	/*
	Option& option=mSavedOption;

	updatePositions();

	if(option.useNormal)
	{

		const VertexElement* normalElem = mMesh->sharedVertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_NORMAL);
		HardwareVertexBufferSharedPtr vBuf = mMesh->sharedVertexData->vertexBufferBinding->getBuffer(normalElem->getSource());

		unsigned char* vertex=static_cast<unsigned char*>(vBuf->lock(HardwareBuffer::HBL_NORMAL ));

		const MeshLoader::Mesh& mesh=mInputMesh;

		float* prnormal;
		for(int i=0; i<mesh.numVertex(); i++)
		{
			normalElem->baseVertexPointerToElement(vertex, &prnormal);

			*prnormal++= mesh.m_arrayVertex[i].normal.x;
			*prnormal++= mesh.m_arrayVertex[i].normal.y;
			*prnormal++= mesh.m_arrayVertex[i].normal.z;

			vertex += vBuf->getVertexSize();
		}
		vBuf->unlock();
	}

	if(option.buildEdgeList)
	{
		mMesh->freeEdgeList();
		mMesh->buildEdgeList();
	}*/
}

Ogre::Entity* TetraMeshToEntity::createEntity(const char* entityName, const char* materialName)
{
	if(RE::ogreSceneManager()->hasEntity(entityName))
		RE::ogreSceneManager()->destroyEntity(entityName);

	Ogre::Entity* entity=RE::ogreSceneManager()->createEntity(entityName, mMesh->getName());
	mEntity=entity;
	entity->setMaterialName(materialName);
	return entity;
}


Entity* TetraMeshLoader::createEntityFromTetraMesh(TetraMesh const& mesh, const char* entityName, const char* materialName,TetraMeshToEntity::Option opt)
{
	TString meshId=RE::generateUniqueName();
		
	TetraMeshToEntity mc(mesh, meshId, opt);
	Ogre::Entity* pPlaneEnt = mc.createEntity(entityName, materialName);

	//Ogre::Entity* pPlaneEnt = RE::ogreSceneManager()->createEntity( entityName, meshId.ptr() );

	pPlaneEnt->setCastShadows(false);
	return pPlaneEnt;
}
