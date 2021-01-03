
#include "stdafx.h"
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include "softbody2/Physics.h"
#include "DynamicsSimulator_TRL_massSpring.h"
#include "MainLib/OgreFltk/RE.h"
#include "MainLib/OgreFltk/pldprimskin.h"
#include "SoftBodyLCP.h"
#include "PhysicsLib/TRL/TRL_common.h"
#include "PhysicsLib/TRL/ForwardDynamicsABM.h"
#include "PhysicsLib/TRL/eigenSupport.h"

const int maxProxies = 32766;
const int maxOverlap = 65535;
const int maxNumObjects = 32760;

static const bool SKIP_REDUNDANT_ACCEL_CALC = false; // true로 할꺼면  SoftBodyLCP의 SKIP_REDUNDANT_ACCEL_CALC 부분 추가로 포팅해야함.

bool TRL::GArticulatedBody::_isStatic() { return body->isStatic();}
void TRL::GArticulatedBody::clearExternalForces()
{
	body->clearExternalForces();
}
class TRL::GArticulatedBodyLink :public TRL::GLink {
	friend class GArticulatedBody;
		::vector3	dvo;
		::vector3	dw;
		::vector3	pf0;
		::vector3	ptau0;
		double  uu;
		double  uu0;
		double  ddq;
		int     numberToCheckAccelCalcSkip;
		int     parentIndex;
		TRL::Link*   link;
		
	public:
		GArticulatedBodyLink(TRL::GBody* _body, TRL::Link* _link): GLink(_body, _link->index), link(_link)
		{
			parentIndex=link->parent?link->parent->index:-1;
		}
		virtual ~GArticulatedBodyLink()
		{
		}
		virtual bool isStatic(){
			return (link->jointType == TRL::Link::FIXED_JOINT);
		}
		virtual void addConstraintForce(vector3 const& contactPoint, int nodeIndex, vector3 const& f)
		{
			link->fext += f;
			link->tauext+=cross(contactPoint, f);
		}
		virtual vector3 getVelocity(vector3 const& contactPoint, int contactPointIndex){
			vector3 v;
			if(link->jointType == TRL::Link::FIXED_JOINT){
				v = 0.0;
			} else {
				v = link->vo + cross(link->w, contactPoint);
			}
			return v;
		}
		virtual vector3 getAcceleration(vector3 const& contactPoint, int contactPointIndex){
			return dvo - cross(contactPoint, dw) + cross(link->w, vector3(link->vo + cross(link->w, contactPoint)));
		}
		virtual void addTestForce(vector3 const& f, vector3 const& constraintpoint, int index)
		{
			vector3 tau(cross(constraintpoint, f));

			vector3 dpf  (-f);
			vector3 dptau(-tau);

			TRL::Link* _link = this -> link;
			GArticulatedBody* _body=((GArticulatedBody*)body);
			while(_link->parent){
				auto& data = _body->getlinksData(_link->index);
				double duu = -(dot(_link->sv, dpf) + dot(_link->sw, dptau));
				data.uu += duu;
				double duudd = duu / _link->dd;
				dpf   += duudd * _link->hhv;
				dptau += duudd * _link->hhw;
				_link = _link->parent;
			}

			_body->dpf   += dpf;
			_body->dptau += dptau;
		}
};

TRL::GArticulatedBody::GArticulatedBody(int ichara, TRL::Body* _body):TRL::GBody(), body(_body)
{
	linksData.resize(body->numLinks());
	// initialize link data
	const TRL::Body& traverse=*body;
	for(int j=0; j < traverse.numLinks(); ++j){
		linksData[j]=new GArticulatedBodyLink(this, traverse[j]);
		assert(j==traverse[j]->index);
	}
}
TRL::GArticulatedBody::~GArticulatedBody()
{
}


void TRL::GArticulatedBody::calcAccelsWithoutExtForce(){

	auto& bodyData=*this;
	bodyData.dpf   = 0;
	bodyData.dptau = 0;

	auto& linksData = bodyData.linksData;
	//const LinkTraverse& traverse = bodyData.body->linkTraverse();
	const TRL::Body& traverse=*bodyData.body;
	int n = traverse.numLinks();

	for(int i = n-1; i >= 0; --i){
		TRL::Link* link = traverse[i];
		auto& data = getlinksData(i);

		data.pf0   = link->pf;
		data.ptau0 = link->ptau;

		for(TRL::Link* child = link->child; child; child = child->sibling){

			auto& childData = getlinksData(child->index);

			data.pf0   += childData.pf0;
			data.ptau0 += childData.ptau0;

			double uu_dd = childData.uu0 / child->dd;
			data.pf0    += uu_dd * child->hhv;
			data.ptau0  += uu_dd * child->hhw;
		}

		if(i > 0){
			data.uu0  = link->uu + link->u - (dot(link->sv, data.pf0) + dot(link->sw, data.ptau0));
			data.uu = data.uu0;
		}
	}
	calcAccels();
}
void TRL::GArticulatedBody::initialze(){
	GBody::initialize();
	auto& bodyData=*this;

	auto& linksData = bodyData.linksData;

	if(bodyData.isStatic ){
		int n = linksData.size();
		for(int j=0; j < n; ++j){
			assert(linksData[j]);
			GArticulatedBodyLink& linkData = (GArticulatedBodyLink&)(*linksData[j]);
			linkData.dw  = 0.0;
			linkData.dvo = 0.0;
		}
	}


	// initialize link connection
	TRL::Body::LinkConnectionArray& connections = body->linkConnections;
	for(size_t j=0; j < connections.size(); ++j){
		Msg::verify(false, "connections has not been implemented yet");
		/*
		   connectedLinkPairs.push_back(LinkPair());
		   LinkPair& linkPair = connectedLinkPairs.back();

		   Body::LinkConnection& connection = connections[j];
		   linkPair.connection = &connection;
		   linkPair.isSameBodyPair = true;
		   linkPair.constraintPoints.resize(connection.numConstraintAxes);

		   for(int k=0; k < connection.numConstraintAxes; ++k){
		   ConstraintPoint& constraint = linkPair.constraintPoints[k];
		   constraint.globalFrictionIndex = numeric_limits<int>::max();
		   }

		   for(int k=0; k < 2; ++k){
		   linkPair.bodyIndex[k] = bodyIndex;
		   linkPair.bodyData[k] = &bodiesData[bodyIndex];
		   Link* link = connection.link[k];
		   linkPair.link[k] = link;
		   linkPair.linkData[k] = &(bodyData.linksData[link->index]);
		   }
		   */
	}
}
void TRL::GArticulatedBody::calcAccels()
{
	auto&bodyData=*this;

	GArticulatedBodyLink& rootData = (GArticulatedBodyLink&)(*linksData[0]);
	TRL::Link* rootLink = rootData.link;

	if(rootLink->jointType == TRL::Link::FREE_JOINT){

		vector3 pf  (rootData.pf0   + bodyData.dpf);
		vector3 ptau(rootData.ptau0 + bodyData.dptau);

		CMatrix66 Ia;
		setMatrix33(rootLink->Ivv, Ia, 0, 0);
		setTransMatrix33(rootLink->Iwv, Ia, 0, 3);
		setMatrix33(rootLink->Iwv, Ia, 3, 0);
		setMatrix33(rootLink->Iww, Ia, 3, 3);

		CVector6 p;
		setVector3(pf,   p, 0);
		setVector3(ptau, p, 3);
		p *= -1.0;

		Eigen::Matrix<double, 6,1> x=Ia.ldlt().solve(p);

		getVector3(rootData.dvo, x, 0);
		getVector3(rootData.dw,  x, 3);

	} else {
		rootData.dw  = 0.0;
		rootData.dvo = 0.0;
	}

	// reset
	bodyData.dpf   = 0;
	bodyData.dptau = 0;

	int skipCheckNumber =  (std::numeric_limits<int>::max() - 1);
	int n = linksData.size();
	for(int linkIndex = 1; linkIndex < n; ++linkIndex){
		auto& linkData = getlinksData(linkIndex);

		if(!SKIP_REDUNDANT_ACCEL_CALC || linkData.numberToCheckAccelCalcSkip <= skipCheckNumber){
			TRL::Link* link = linkData.link;
			auto& parentData = getlinksData(linkData.parentIndex);

			linkData.ddq = (linkData.uu - (dot(link->hhv, parentData.dvo) + dot(link->hhw, parentData.dw))) / link->dd;
			linkData.dvo = parentData.dvo + link->cv + link->sv * linkData.ddq;
			linkData.dw  = parentData.dw  + link->cw + link->sw * linkData.ddq;

			// reset
			linkData.uu   = linkData.uu0;
		}
	}
}

void TRL::DynamicsSimulator_TRL_massSpring::_registerCollisionCheckPair(int bodyIndex1, int bodyIndex2, int treeIndex1, int treeIndex2, vectorn const& param)
{
	double staticFriction=param[0];
	double slipFriction=param[1];

	const double epsilon = 0.0;
	if(bodyIndex1 >= 0 && bodyIndex2 >= 0){
		VRMLloader* skel1=&skeleton(bodyIndex1);
		VRMLloader* skel2=&skeleton(bodyIndex2);
		int links1=((VRMLTransform&)skel1->bone(treeIndex1)).lastHRPjointIndex();
		int links2=((VRMLTransform&)skel2->bone(treeIndex2)).lastHRPjointIndex();

		GLink* link1= body(bodyIndex1)->linksData[links1];
		GLink* link2= body(bodyIndex2)->linksData[links2];

		if(link1 && link2 && link1 != link2){
			bool ok = _contactForceSolver->addCollisionCheckLinkPair
				(bodyIndex1, link1, bodyIndex2, link2, staticFriction, slipFriction, epsilon);
			if(ok){
				collisionDetector->addCollisionPair(skel1, treeIndex1, skel2, treeIndex2);
				printf("added\n");
			}
		}
	}
}
void TRL::DynamicsSimulator_TRL_massSpring::registerAllCollisionCheckPairs(int bodyIndex1, int bodyIndex2, vectorn const& param)
{
	VRMLloader* skel1=&skeleton(bodyIndex1);
	VRMLloader* skel2=&skeleton(bodyIndex2);
	if (skel1!=skel2)
	{
		for(int i=1; i<skel1->numBone(); i++)
			for(int j=1; j<skel2->numBone(); j++)
				_registerCollisionCheckPair(bodyIndex1, bodyIndex2, i, j, param);
	}
	else
	{
		Msg::error("use registerCollisionCheckPair instead!!!");
	}
}
void TRL::DynamicsSimulator_TRL_massSpring::registerCollisionCheckPair
(
 const char *charName1,
 const char *linkName1,
 const char *charName2,
 const char *linkName2,
 vectorn const& param
 )
{
	int bodyIndex1 = world.bodyIndex(charName1);
	int bodyIndex2 = world.bodyIndex(charName2);

	if(bodyIndex1 >= 0 && bodyIndex2 >= 0){

		int treeIndex1=skeleton(bodyIndex1).getTreeIndexByName(linkName1);
		int treeIndex2=skeleton(bodyIndex2).getTreeIndexByName(linkName2);
		_registerCollisionCheckPair(bodyIndex1, bodyIndex2, treeIndex1, treeIndex2, param);
	}
}
void TRL::DynamicsSimulator_TRL_massSpring::_registerCharacter(const char *name, OpenHRP::CharacterInfo const& cinfo)
{
	OpenHRP::DynamicsSimulator_TRL_penalty::_registerCharacter(name, cinfo);

	int ichara=_characters.size()-1;
	TRL::Body* chara=((DynamicsSimulator_TRL_penalty*)this)->world.body(ichara);
	_addBody(new GArticulatedBody(ichara, chara));
	int ibody=_bodies.size()-1;
	assert(ibody==ichara);
}









TRL::GTerrainBody::GTerrainBody(TRL::DynamicsSimulator_TRL_massSpring& sim, const char* filename, vector3 const& pos, int sizeX, int sizeY, m_real width, m_real height, m_real heightMax)
{
	linksData.resize(1);
	linksData[0]=new GLink(this, 0);

	MeshLoader::Terrain* terrain=new MeshLoader::Terrain(filename, sizeX, sizeY, width, height, heightMax, 1,1);
	mTerrain=terrain;
	mTerrainPos=pos;
	Ogre::Entity* entity=MeshLoader::createMeshEntity(*mTerrain, RE::generateUniqueName());

	Ogre::SceneNode* terrainNode=RE::createSceneNode( "terrain");
	double skinScale=100;
	terrainNode->setScale(skinScale, skinScale, skinScale);
	terrainNode->setPosition(mTerrainPos.x*skinScale, mTerrainPos.y*skinScale, mTerrainPos.z*skinScale);
	entity->setMaterialName("CrowdEdit/Terrain1");
	entity->setCastShadows(false);
	terrainNode-> attachObject(entity);
}
double TRL::GTerrainBody::getTerrainHeight(vector3 const& pos)
{
	vector3 localX=pos-mTerrainPos;
	vector3 normal;
	double y=mTerrain->height(vector2(localX.x, localX.z), normal);
	return y+mTerrainPos.y;
	//localX.y=y;
	//vector3 floor=localX+mTerrainPos;
	//return floor.y;
}
TRL::GTerrainBody::~GTerrainBody(){}
static inline double	AreaOf(const vector3& x0, const vector3& x1, const vector3& x2)
{
	const vector3	a=x1-x0;
	const vector3	b=x2-x0;
	vector3	cr;
	cr.cross(a,b);
	const double	area=cr.length();
	return(area);
}
TRL::GSoftBody::GSoftBody(TRL::DynamicsSimulator_TRL_massSpring& sim, OBJloader::Mesh const& targetMesh):
	GLink(NULL, 0),
	_sim(sim)
{
	linksData.resize(1);
	linksData[0]=this;
	GLink::body=this;

	mSimulatedMesh=targetMesh;
	//OBJloader::GetMeshAnimation::_create(mSkin, mSimulatedMesh,0);
	mSimulatedMesh.calculateVertexNormal();
	OBJloader::MeshToEntity::Option option;
	option.buildEdgeList=true;
	option.useTexCoord=false;
	mSimulatedOgreMesh=new OBJloader::MeshToEntity(mSimulatedMesh, "simulatedMesh", option);
	if(!mNode)
	{
		mNode.reset(RE::createSceneNode(RE::generateUniqueName()), (void(*)(Ogre::SceneNode* node))RE::removeEntity);			
	}
	//mNode->attachObject(mSimulatedOgreMesh->createEntity("simulatedMeshEntity", "white"));
	mNode->attachObject(mSimulatedOgreMesh->createEntity("simulatedMeshEntity", "lightgrey_transparent"));
	double skinScale=100;
	mNode->setScale(skinScale, skinScale, skinScale);

	mSimulatedOgreMesh->getLastCreatedEntity()->setVisible(sim._drawSimulatedMesh);
	//#define _USE_BUNNY
#ifdef _USE_BUNNY
	btSoftBody2*	psb=btSoftBody2Helpers::CreateFromTriMesh(gVerticesBunny,
			&gIndicesBunny[0][0],
			BUNNY_NUM_TRIANGLES);
#else
	OBJloader::EdgeConnectivity edges(targetMesh);

	int node_count=targetMesh.numVertex();
	m_pSystem=new Physics_ParticleSystem(node_count);

	Physics_ParticleSystem* system=m_pSystem;
	system->m_iInverseIterations = 10;
	system->m_iIntegrationMethod = sim._integrateMethod;
	_CreateFromTriMesh(targetMesh, edges, sim._stiffness, sim._springDamp, sim._bendingConstraints1, sim._bendingStiffness);
#endif


	setTotalMass(targetMesh, sim._mass,true);

	m_pSystem->SetupMatrices();

	vector3 g;
	sim.getGVector(g);
	Physics_GravityForce *pGravity = new Physics_GravityForce( g);

	m_pSystem->AddForce(*pGravity);

	//psb->createSystem();
	//psb->m_pSystem->m_cfg.mu=FlexibleBodyImplicit::kDF;
}
TRL::GSoftBody::GSoftBody(TRL::DynamicsSimulator_TRL_massSpring& sim, OBJloader::Mesh const& targetMesh, intmatrixn const& springs, vectorn const& stiffness )
:
	GLink(NULL, 0),
	_sim(sim)
{
	linksData.resize(1);
	linksData[0]=this;
	GLink::body=this;

	mSimulatedMesh=targetMesh;
	//OBJloader::GetMeshAnimation::_create(mSkin, mSimulatedMesh,0);
	mSimulatedMesh.calculateVertexNormal();
	OBJloader::MeshToEntity::Option option;
	option.buildEdgeList=true;
	option.useTexCoord=false;
	mSimulatedOgreMesh=new OBJloader::MeshToEntity(mSimulatedMesh, "simulatedMesh", option);
	if(!mNode)
	{
		mNode.reset(RE::createSceneNode(RE::generateUniqueName()), (void(*)(Ogre::SceneNode* node))RE::removeEntity);			
	}
	//mNode->attachObject(mSimulatedOgreMesh->createEntity("simulatedMeshEntity", "white"));
	mNode->attachObject(mSimulatedOgreMesh->createEntity("simulatedMeshEntity", "lightgrey_transparent"));
	double skinScale=100;
	mNode->setScale(skinScale, skinScale, skinScale);

	mSimulatedOgreMesh->getLastCreatedEntity()->setVisible(sim._drawSimulatedMesh);
	OBJloader::EdgeConnectivity edges(targetMesh);

	int node_count=targetMesh.numVertex();
	m_pSystem=new Physics_ParticleSystem(node_count);

	Physics_ParticleSystem* system=m_pSystem;
	system->m_iInverseIterations = 10;
	system->m_iIntegrationMethod = sim._integrateMethod;

	int nv=targetMesh.numVertex();
	for(int i=0;i<nv; i++)
	{
		nodePos(i)=targetMesh.getVertex(i);
		nodeVel(i)=vector3(0,0,0);
	}

	for(int i=0; i<springs.rows(); i++)
	{
		appendLink(targetMesh, springs(i,0), springs(i,1), stiffness(i), sim._springDamp, Structural);
	}

	setTotalMass(targetMesh, sim._mass,true);

	m_pSystem->SetupMatrices();

	vector3 g;
	sim.getGVector(g);
	Physics_GravityForce *pGravity = new Physics_GravityForce( g);

	m_pSystem->AddForce(*pGravity);

	//psb->createSystem();
	//psb->m_pSystem->m_cfg.mu=FlexibleBodyImplicit::kDF;
}
vector3& TRL::GSoftBody::nodePos(int i) { return m_pSystem->Position(i);}
vector3& TRL::GSoftBody::nodeVel(int i) { return m_pSystem->Velocity(i);}

static intvectorn indices;
static vector3N forces;

void TRL::GSoftBody::addExternalForce(intvectorn const& _indices, vector3N const& _forces)
{
	indices=_indices;
	forces=_forces;
}
void TRL::GSoftBody::clearExternalForces()
{
	m_pSystem->m_TotalForces_ext.zero();
	for(int i=0; i<indices.size(); i++)
		m_pSystem->m_TotalForces_ext[indices[i]]=forces[i];
}
static double _drawSimulatedMeshOffset=0.0;
void TRL::GSoftBody::renderme()
{
	for(int i=0;i<m_pSystem->m_iParticles;++i)
		mSimulatedMesh.getVertex(i)=nodePos(i)+vector3(_drawSimulatedMeshOffset);
	mSimulatedMesh.calculateVertexNormal();
	mSimulatedOgreMesh->updatePositionsAndNormals();

	for(int i=0;i<m_pSystem->m_iParticles;++i)
		mSimulatedMesh.getVertex(i)=nodePos(i);
	if(_sim._drawContactForce)
	{
		for(int i=0; i<m_pSystem->m_iParticles; i++)
		{
			vector3 contactForce(0,0,0);
			//contactForce=m_pSystem->m_vContactForce[i];

			//printf("%f %f %f\n", contactForce.x() , contactForce.y(), contactForce.z());
			if(contactForce.length()>0.1)
			{	
				vector3 from=nodePos(i);
				vector3 to=from+contactForce*10;
				//idraw->drawLine(from,to,vector3(1,1,1));
			}
		}
	}
	mNode->_updateBounds();
}
void TRL::GSoftBody::setTotalMass(OBJloader::Mesh const& mesh, double mass,bool fromfaces)
{
	int numNodes=m_pSystem->m_iParticles;
	if(fromfaces)
	{
		vectorn masses(numNodes);
		masses.setAllValue(0.0);

		for(int i=0;i<mesh.numFace(); ++i)
		{
			const OBJloader::Face&		f=mesh.getFace(i);
			const double	twicearea=AreaOf(	
					nodePos(f.vi(0)),
					nodePos(f.vi(1)),
					nodePos(f.vi(2)));
			for(int j=0;j<3;++j)
				masses(f.vi(j))+=twicearea;
		}

		masses*=mass/masses.sum();

		for(int i=0;i<numNodes;++i)
		{
			m_pSystem->SetMass(i, masses[i]);
		}
	}
	else
	{
		double itm=mass/((double)numNodes);
		for(int i=0;i<numNodes;++i)
			m_pSystem->SetMass(i, itm);
	}
}
void TRL::GSoftBody::setRestLength(vectorn const& length)
{
	/* Links		*/ 
	for(int i=0,ni=_links.rows();i<ni;++i)
	{
		intvectornView l=_links.row(i);
		Physics_SpringForce& sp=(Physics_SpringForce&)m_pSystem->Force(l(LINK_FORCE));
		sp.m_RestDistance	=	length[i];
		//l.m_c0	=	l.m_n[0]->im()+l.m_n[1]->im();
		//l.m_c1	=	l.m_pSpring->m_RestDistance*l.m_pSpring->m_RestDistance;
		sp.m_MaxDistance=sp.m_RestDistance*1.1;
	}
}
void TRL::GSoftBody::changeSoftBodyRestLength(OBJloader::Mesh const& targetMesh)
{
	/* Links		*/ 
	for(int i=0,ni=_links.rows();i<ni;++i)
	{
		intvectornView l=_links.row(i);
		Physics_SpringForce& sp=(Physics_SpringForce&)m_pSystem->Force(l(LINK_FORCE));
		sp.m_RestDistance	=	targetMesh.getVertex(l(LINK_N0)).distance(targetMesh.getVertex(l(LINK_N1)));
		//l.m_c0	=	l.m_n[0]->im()+l.m_n[1]->im();
		//l.m_c1	=	l.m_pSpring->m_RestDistance*l.m_pSpring->m_RestDistance;
		sp.m_MaxDistance=sp.m_RestDistance*1.1;
	}
}
void TRL::GSoftBody::extractLinkLength(OBJloader::Mesh const& targetMesh, vectorn& length)
{
	int ni=_links.rows();
	length.setSize(ni);

	/* Links		*/ 
	for(int i=0;i<ni;++i)
	{
		intvectornView l=_links.row(i);
		length[i]	=	targetMesh.getVertex(l(LINK_N0)).distance(targetMesh.getVertex(l(LINK_N1)));
	}
}

void TRL::GSoftBody::appendLink(OBJloader::Mesh const& mesh, int index1, int index2, double stiffness, double _springDamp, int linkType)
{
	Physics_SpringForce* pSpring = new Physics_SpringForce();
	pSpring->m_iParticle[0] = index1;
	pSpring->m_iParticle[1] = index2;
	pSpring->m_RestDistance = (mesh.getVertex(index1)-mesh.getVertex(index2)).length();
	pSpring->m_MaxDistance = pSpring->m_RestDistance* 1.1;
	pSpring->m_bFixup = true;
	pSpring->m_kSpring = stiffness;
	pSpring->m_kSpringDamp = _springDamp;
	m_pSystem->AddForce( *pSpring );
	intvectorn temp(4);
	temp[LINK_FORCE]=m_pSystem->numForces()-1;
	temp[LINK_N0]=index1;
	temp[LINK_N1]=index2;
	temp[LINK_TYPE]=linkType;
	_links.pushBack(temp);
}
void TRL::GSoftBody::_CreateFromTriMesh( OBJloader::Mesh const& mesh, OBJloader::EdgeConnectivity const& edges, double stiffness, double damp, int con2, double stiff2)
{

	int nv=mesh.numVertex();
	for(int i=0;i<nv; i++)
	{
		nodePos(i)=mesh.getVertex(i);
		nodeVel(i)=vector3(0,0,0);
	}
	// _springDamp, _integrateMethod;

	OBJloader::EdgeConnectivity::edgeT e;
	TUGL_for_all_edges(e, edges.mMeshGraph)
	{
		appendLink(mesh, e.v1().index(), e.v2().index(), stiffness, damp, Structural);
	}
	for(int i=0; i<mesh.numFace(); i++)
	{	
		//appendFace(mesh.getFace(i).vi(0), mesh.getFace(i).vi(1), mesh.getFace(i).vi(2));
	}

	generateBendingConstraints(mesh, con2, stiff2, damp);

}


void TRL::GSoftBody::adjustSoftBodyParam(const char* _adjustWhat, const char* selectionFileName, double value)
{

	bitvectorn selectedVertices, selectedEdges;
	BinaryFile bf(false, selectionFileName);
	bf.unpack(selectedVertices);
	bf.unpack(selectedEdges);
	bf.close();
	adjustSoftBodyVertexParam( _adjustWhat,  selectedVertices, value);
}

void TRL::GSoftBody::adjustSoftBodyEdgeParam(const char* _adjustWhat, boolN const& selectedEdges, double value)
{
	TString adjustWhat(_adjustWhat);
}

void TRL::GSoftBody::getCustomFrictionCoef(vector3 const& contactPoint, int nodeIndex, bool isSlipping, double& mu)
{
	mu=m_pSystem->m_cfg.m_dynamicFrictionCoef[nodeIndex];		

}

void TRL::GSoftBody::adjustSoftBodyVertexParam(const char* _adjustWhat, boolN const& selectedVertices, double value)
{
	TString adjustWhat(_adjustWhat);
	if(adjustWhat=="dynamicFrictionCoef")
	{		
		for(int i=0; i<m_pSystem->m_cfg.m_dynamicFrictionCoef.size(); i++)
		{
			if(selectedVertices[i])
				m_pSystem->m_cfg.m_dynamicFrictionCoef[i]=value;
		}		
	}
	else if(adjustWhat=="stiffness")
	{
		for(int i=0,ni=_links.rows();i<ni;++i)
		{
			intvectornView l=_links.row(i);
			Physics_SpringForce& sp=(Physics_SpringForce&)m_pSystem->Force(l(LINK_FORCE));
			int index1=l(LINK_N0);
			int index2=l(LINK_N1);
			if(selectedVertices[index1] && selectedVertices[index2])	
			{
				sp.m_kSpring=value;
			}
		}
	}
}
int	TRL::GSoftBody::generateBendingConstraints(OBJloader::Mesh const& mesh, int distance, double stiffness, double damp)
{
	if(distance>1)
	{
		/* Build graph	*/ 
		const int		n=mesh.numVertex();
		const unsigned	inf=(~(unsigned)0)>>1;
		unsigned*		adj=new unsigned[n*n];
#define IDX(_x_,_y_)	((_y_)*n+(_x_))
		for(int j=0;j<n;++j)
		{
			for(int i=0;i<n;++i)
			{
				if(i!=j)	adj[IDX(i,j)]=adj[IDX(j,i)]=inf;
				else
					adj[IDX(i,j)]=adj[IDX(j,i)]=0;
			}
		}
		for(int i=0;i<_links.rows();++i)
		{
			if(_links(i, LINK_TYPE)==Structural)
			{
				const int	ia=_links(i,LINK_N0);
				const int	ib=_links(i,LINK_N1);
				adj[IDX(ia,ib)]=1;
				adj[IDX(ib,ia)]=1;
			}
		}
		for(int k=0;k<n;++k)
		{
			for(int j=0;j<n;++j)
			{
				for(int i=j+1;i<n;++i)
				{
					const unsigned	sum=adj[IDX(i,k)]+adj[IDX(k,j)];
					if(adj[IDX(i,j)]>sum)
					{
						adj[IDX(i,j)]=adj[IDX(j,i)]=sum;
					}
				}
			}
		}
		/* Build links	*/ 
		int	nlinks=0;
		for(int j=0;j<n;++j)
		{
			for(int i=j+1;i<n;++i)
			{
				if(adj[IDX(i,j)]==(unsigned)distance)
				{
					appendLink(mesh, i,j,stiffness,damp, Bending);
					++nlinks;
				}
			}
		}
		delete[] adj;
		return(nlinks);
	}
	return(0);
}


void TRL::GSoftBody::calcAccelsWithoutExtForce()
{
	m_pSystem->m_TotalForces_ext.zero();
	//m_pSystem->UpdateDV_Implicit(_sim.getTimestep());
	m_pSystem->UpdateDV_Implicit_part1(_sim.getTimestep());
	m_b_f=m_pSystem->m_vTemp1;
	m_pSystem->UpdateDV_Implicit_part2(_sim.getTimestep());
	m_internalForces=m_pSystem->m_TotalForces_int;
}

void TRL::GSoftBody::initialze()
{
	GBody::initialize();
}
void TRL::GSoftBody::calcAccels()
{
	//m_pSystem->UpdateDV_Implicit(_sim.getTimestep());
	m_pSystem->m_TotalForces_int=m_internalForces;
	m_pSystem->m_vTemp1=m_b_f;
	m_pSystem->UpdateDV_Implicit_part2(_sim.getTimestep());
}

void TRL::GSoftBody::addTestForce(vector3 const& f, vector3 const& contactPoint,int nodeIndex)
{
	m_pSystem->m_TotalForces_ext.zero();
	m_pSystem->m_TotalForces_ext[nodeIndex]=f;
	//m_pSystem->UpdateDV_Implicit(_sim.getTimestep());
	//std::cout << "testforce"<< f<< " dv" <<m_pSystem->m_dv[nodeIndex]<<std::endl;


}
vector3 TRL::GSoftBody::getVelocity(vector3 const& contactPoint, int nodeIndex)
{
	return m_pSystem->Velocity(nodeIndex);
}
vector3 TRL::GSoftBody::getAcceleration(vector3 const& contactPoint, int nodeIndex)
{
	// acceleration*dt = dv
	// -> accelerations = dv/dt
	return m_pSystem->m_dv[nodeIndex]/_sim.getTimestep();
}

void TRL::GSoftBody::addConstraintForce(vector3 const& contactPoint, int nodeIndex, vector3 const& f)
{
	m_pSystem->m_TotalForces_ext[nodeIndex]=f;
	//printf("cf %d %s\n", nodeIndex, f.output().ptr());
}

TRL::GSoftBodyFastContact::GSoftBodyFastContact(DynamicsSimulator_TRL_massSpring& sim, OBJloader::Mesh const& targetMesh)
:TRL::GSoftBody(sim,targetMesh)
{
}
TRL::GSoftBodyFastContact::GSoftBodyFastContact(DynamicsSimulator_TRL_massSpring& sim, OBJloader::Mesh const& targetMesh, intmatrixn const& springs, vectorn const& stiffness )
:TRL::GSoftBody(sim,targetMesh, springs, stiffness)
{
}
void TRL::GSoftBodyFastContact::addTestForce(vector3 const& f, vector3 const& contactPoint,int contactSetIndex)
{
	m_pSystem->m_TotalForces_ext.zero();
	ContactSet& cs=contactSets[contactSetIndex];
	for(int i=0,ni=cs.nodeWeights.size(); i<ni; i++)
	{
		m_pSystem->m_TotalForces_ext[cs.nodeIndices[i]]+=f*cs.nodeWeights[i];
	}
}

vector3 TRL::GSoftBodyFastContact::getVelocity(vector3 const& contactPoint, int contactSetIndex)
{
	ContactSet& cs=contactSets[contactSetIndex];
	return TRL::GSoftBody::getVelocity(contactPoint, cs.iTargetNode);
}
vector3 TRL::GSoftBodyFastContact::getAcceleration(vector3 const& contactPoint, int contactSetIndex)
{
	ContactSet& cs=contactSets[contactSetIndex];
	return TRL::GSoftBody::getAcceleration(contactPoint, cs.iTargetNode);
}
void TRL::GSoftBodyFastContact::addConstraintForce(vector3 const& contactPoint, int contactSetIndex, vector3 const& f)
{
	ContactSet& cs=contactSets[contactSetIndex];
	for(int i=0,ni=cs.nodeWeights.size(); i<ni; i++)
	{
		m_pSystem->m_TotalForces_ext[cs.nodeIndices[i]]+=f*cs.nodeWeights[i];
	}
}

void TRL::DynamicsSimulator_TRL_massSpring::_addBody(GBody* body)
{
	_bodies.resize(_bodies.size()+1);
	_bodies.back()=body;
	_bodies.back()->initialize();
}
void makeCharacterInfo(VRMLloader const& input, OpenHRP::CharacterInfo& output);
void TRL::DynamicsSimulator_TRL_massSpring::_addEmptyCharacter(const char* name)
{
	// add empty character first
	OpenHRP::CharacterInfo cinfo;
	VRMLloader* l=new VRMLloader("../Resource/mesh/empty.wrl");
	l->name=name;
	makeCharacterInfo(*l, cinfo);
	OpenHRP::DynamicsSimulator_TRL_penalty::_registerCharacter(cinfo.name, cinfo);
}
void TRL::DynamicsSimulator_TRL_massSpring::createSoftBody(OBJloader::Mesh const& targetMesh)
{
	TString name;
	name.format("softmesh_%d", _characters.size());
	_addEmptyCharacter(name.ptr());

	_addBody(new GSoftBody(*this, targetMesh));
	int ichara=_characters.size()-1;
	int ibody=_bodies.size()-1;
	assert(ichara==ibody);
	_softbodyIndices.pushBack(ibody);
}
void TRL::DynamicsSimulator_TRL_massSpring::createSoftBody(OBJloader::Mesh const& targetMesh, intmatrixn const& springs, vectorn const& stiffness )
{
	TString name;
	name.format("softmesh_%d", _characters.size());
	_addEmptyCharacter(name.ptr());

	//_addBody(new GSoftBodyFastContact(*this, targetMesh, springs, stiffness));
	_addBody(new GSoftBody(*this, targetMesh, springs, stiffness));
	int ichara=_characters.size()-1;
	int ibody=_bodies.size()-1;
	assert(ichara==ibody);
	_softbodyIndices.pushBack(ibody);
}
TRL::GSoftBody* TRL::DynamicsSimulator_TRL_massSpring::softbody(int bodyIndex)
{ 
	if(body(bodyIndex)->getType()==GBody::SoftBody || body(bodyIndex)->getType()==GBody::SoftBodyFastContact) 
		return (GSoftBody*)_bodies[bodyIndex]; 
	return NULL;
}

void TRL::DynamicsSimulator_TRL_massSpring::createTerrain(const char* filename, vector3 const& pos, int sizeX, int sizeY, m_real width, m_real height, m_real heightMax)
{
	TString name;
	name.format("terrain_%d", _characters.size());
	_addEmptyCharacter(name.ptr());

	_addBody(new GTerrainBody(*this, filename, pos, sizeX, sizeY, width, height, heightMax));
	int ibody=_bodies.size()-1;
	_hasTerrain=true;
}

void	TRL::DynamicsSimulator_TRL_massSpring::renderme()
{
	for(int i=0; i<numBodies(); i++)
		body(i)->renderme();

	//mNode->showBoundingBox(true);
	//mSimulatedOgreMesh->updatePositions();
	//mSimulatedOgreMesh->mTargetMesh->buildEdgeList();

}








TRL::DynamicsSimulator_TRL_massSpring::DynamicsSimulator_TRL_massSpring()
	:
		DynamicsSimulator_TRL_penalty("libccd"),
		_drawContactForce(true),
		_debugDrawWorld(false),
		_drawSimulatedMesh(true),
		_hasTerrain(false)
{
	_MA=0;
	_contactForceSolver=new SoftBodyLCP(*this);
	_usePenaltyMethod=false;
	_penaltyStiffness=10000.0;
}
TRL::DynamicsSimulator_TRL_massSpring::~DynamicsSimulator_TRL_massSpring()
{
	delete _contactForceSolver;
}

void TRL::DynamicsSimulator_TRL_massSpring::initSimulation()
{
	//world.initialize(); // moved to init (which is less frequently called)
	int n = _characters.size();
	for(int i=0; i<n; i++){
		TRL::ForwardDynamicsABM* fd=world.forwardDynamics(i);
		fd->calcPositionAndVelocityFK();
	}
	_updateCharacterPose();
}
void TRL::DynamicsSimulator_TRL_massSpring::init(double timeStep, OpenHRP::DynamicsSimulator::IntegrateMethod integrateOpt)
{
	DynamicsSimulator_TRL_penalty::init(timeStep, integrateOpt);

	_contactForceSolver->initialize();
	if(_contactForceSolver->collisionCheckLinkPairs.size()>0)
	{
		for(int i=0; i<_contactForceSolver->collisionCheckLinkPairs.size(); i++)
		{
			assert(_contactForceSolver->collisionCheckLinkPairs[i].bodyData[0]);
			assert(_contactForceSolver->collisionCheckLinkPairs[i].bodyData[1]);
		}
	}
}


void TRL::DynamicsSimulator_TRL_massSpring::setParam_Epsilon_Kappa(double eps, double kap)
{
	_contactForceSolver->epsilon=eps;
	_contactForceSolver->kappa=kap;
}
void TRL::DynamicsSimulator_TRL_massSpring::setParam_R_B_MA(double r, double b, double ma)
{
	_contactForceSolver->_R=r;
}

//

TRL::GTerrainBody* TRL::DynamicsSimulator_TRL_massSpring::getTerrain() const 
{
	for(int i=0; i<_bodies.size(); i++)
	{
		if(_bodies[i] && _bodies[i]->getType()==GBody::Terrain)
		{
			GTerrainBody*terrain=(GTerrainBody*)_bodies[i];
			return terrain;
		}
	}
	return NULL;
}
TRL::GSoftBody* TRL::DynamicsSimulator_TRL_massSpring::getSoftBody() const 
{
	for(int i=0; i<_bodies.size(); i++)
	{
		if(_bodies[i] && _bodies[i]->getType()==GBody::SoftBody || _bodies[i]->getType()==GBody::SoftBodyFastContact)
		{
			GSoftBody*sb=(GSoftBody*)_bodies[i];
			return sb;
		}
	}
	return NULL;
}
bool TRL::DynamicsSimulator_TRL_massSpring::stepSimulation()
{

	// collision check rigid-rigid pairs
	collisionDetector->queryContactDeterminationForDefinedPairs(_characters, *collisions);
	// collision check soft-terrain pairs
	{
		for(int ipair=0, np=_contactForceSolver->collisionCheckLinkPairs.size(); ipair<np; ipair++)
		{

			SoftBodyLCP::LinkPair& lp=_contactForceSolver->collisionCheckLinkPairs[ipair];
			int sbIndex=-1;
			if(lp.bodyData[0]->getType()==GBody::SoftBody || lp.bodyData[0]->getType()==GBody::SoftBodyFastContact )
				sbIndex=0;
			else if(lp.bodyData[1]->getType()==GBody::SoftBody || lp.bodyData[1]->getType()==GBody::SoftBodyFastContact )
				sbIndex=1;

			if(sbIndex!=-1)
			{
				int obIndex=(sbIndex+1)%2;
				if(lp.bodyData[obIndex]->getType()==GBody::Terrain)
				{
					double step=getTimestep();
					double margin=0.0025;
					Physics_ParticleSystem* m_pSystem=((GSoftBody*)lp.bodyData[sbIndex])->m_pSystem;
					GTerrainBody*terrain=(GTerrainBody*)lp.bodyData[obIndex];
					MeshLoader::Terrain* mTerrain=terrain->mTerrain;

					int icon=0;
					// check collisions
					int numNodes=m_pSystem->m_iParticles;
					// softbody, terrain pairs
					OpenHRP::CollisionPointSequence& points=(*collisions)[ipair].points;
					points.resize(0);
					for(int inode=0; inode<numNodes; inode++)
					{
						double marginOffset=0;
						if(m_pSystem->mContacts.contactStatus[inode]!=Physics_Contacts::no_contact)
							marginOffset=margin;

						vector3 x=m_pSystem->Position(inode);
							double skinScale=100;
						//if (inode <10) RE::moveEntity(RE::createEntity(TString("pos", inode) ,"sphere1010.mesh"),vector3(1,1,1),x*skinScale);
						vector3 localX=x-terrain->mTerrainPos;
						vector2 proj_localX(localX.x, localX.z);
						m_real maxHeight=mTerrain->maxHeight(proj_localX);

						if(localX.y>maxHeight+margin)
							continue;

						vector3 normal;
						double dst=localX.y-(mTerrain->height(proj_localX, normal)+margin);

						const vector3		va=vector3(0,0,0);
						const vector3		vb=m_pSystem->Velocity(inode);
						const vector3		vr=vb-va;
						if(dst-marginOffset<0)
						{
							double dp2=dst-marginOffset;
							//m_pSystem->mContacts_peneltyMethod.add(va, vb, normal, dp2, inode);
							//
							points.resize(points.size()+1);
							OpenHRP::CollisionPoint& point =points.back();
							if(sbIndex==0)
								point.normal=-normal;
							else
								point.normal=normal;
							point.idepth=-dp2;
							point.position=x-point.normal*(point.idepth*0.5); // center pos.
							//point.position=x;// bottom pos.
							//point.position=x-point.normal*point.idepth;// top pos.
							point.inode=inode;
							//printf("detected\n");

							//RE::moveEntity(RE::createEntity(TString("rmetric", inode) ,"sphere1010.mesh"),vector3(1,1,1),point.position *skinScale);

						}
					}


					// single contact point method.
					if(lp.bodyData[sbIndex]->getType()==GBody::SoftBodyFastContact && points.size()>0)
					{
						// choose COP
						double maxDepth=-1000;
						int argMax=-1;

						GSoftBodyFastContact* sb=(GSoftBodyFastContact*)lp.bodyData[sbIndex];
						sb->contactSets.resize(1);
						sb->contactSets[0].nodeWeights.setSize(points.size());
						sb->contactSets[0].nodeIndices.setSize(points.size());
						sb->contactSets[0].nodeWeights.setAllValue(1.0/((double)points.size()));

						//std::cout<<sb->contactSets[0].nodeWeights<<std::endl;
						intvectorn& ni=sb->contactSets[0].nodeIndices;

						for(int i=0; i<points.size(); i++)
						{
							OpenHRP::CollisionPoint& point =points[i];
							if (point.idepth>maxDepth)
							{
								argMax=i;
								maxDepth=point.idepth;
							}
							ni(i)=point.inode;
						}
						
						Msg::verify(argMax!=-1,"???");
						points[0]=points[argMax];
						points.resize(1);
						sb->contactSets[0].iTargetNode=points[0].inode;
						points[0].inode=0; // 0-th ContactSet
					}
				}
				else
				{
					printf("warning! collisions between softbody and non-terrain objects are not supported yet!\n");
				}

			}
		}
	}
	for(int i=0; i<world.numBodies(); i++)
	{
		TRL::Body* cinfo=world.body(i);
		TRL::ForwardDynamicsABM* fd=world.forwardDynamics(i);
		// position, velocity fk
		fd->calcABMPhase1();
		// update inertia and velocity related terms
		fd->calcABMPhase2Part1();
	}


	// set external forces
	if(!_usePenaltyMethod)
		_contactForceSolver->solve(*collisions);
	else
	{
		// penalty mode 
		// almost identical to DynamicsSimulator::_calcContactForce
		_contactForceSolver->initialize(); // calls GBody::initialize, GBody::clearExternalForces
		// By default, this function uses penalty method. Each simulator implementations needs to override this function.
		_updateContactInfo( *collisions);
		for(size_t i=0; i < collisionDetector->getCollisionPairs().size(); ++i)
		{
			SoftBodyLCP::LinkPair& linkPair=_contactForceSolver->collisionCheckLinkPairs[i];
			OpenHRP::CollisionPointSequence& points = (*collisions)[i].points;

			int n_point = points.size();
			if(n_point == 0) continue;

			double k,d; k=_penaltyStiffness; d=_penaltyStiffness*0.1;
			for(int j=0; j<n_point; j++)
			{
				const vector3& normal=points[j].normal;	
				const vector3& pos=points[j].position;
				double depth=points[j].idepth;

				GLink* link1 = linkPair.linkData[0];
				GLink* link2 = linkPair.linkData[1];

				vector3 vel1, vel2;
				int inode=points[j].inode; // todo : inode needs to be inode1, inode2
				vel1=link1->getVelocity(pos, inode);
				vel2=link2->getVelocity(pos, inode);

				vector3 relvel;
				relvel.sub(vel1, vel2);

				double vn=normal%relvel;	// normal rel vel
				double _depthMax=0.01;	// 1cm

				double f=depth*k ;
				double f2= vn*d*sop::clampMap(depth, 0, _depthMax);

				f=f+f2;
				if(f<0.0) continue;

				::vector3 normalForce=-1*normal*f;
				::vector3 fv=relvel-normal*vn;

				double tiny = 1e-8;
				double mu=1.0;
				for(int k=0; k<2; ++k){
					GLink* link = linkPair.linkData[k];
					if(link->hasCustomFricCoef())
					{
						link->getCustomFrictionCoef(pos, inode, true, mu);
					}
				}

				::vector3 force=normalForce;
				if(fv.length()>tiny)	// slip friction force
				{
					// calc frictionForce
					::vector3 dir;
					dir.normalize(fv);

					force-=dir*f*mu;
				}
				else
				{
					//printf("static friction needed\n");
				}
				force/=n_point;

				double sign=1.0;
				for(int ipair=0; ipair<2; ++ipair){
					if(ipair==1) sign=-1.0;

					GLink* link=linkPair.linkData[ipair];
					if(!link->isStatic())
						link->addConstraintForce(pos, points[j].inode, force*sign); 
				}
			}
		}
	}
	// rigidbody: collect forces, calculate accelerations, and then integrate them
	world.calcNextState();	 // ABMPhase2-2, 3, integrate, ABMPhase1, 2-1

	// softbody:
	for(int i=0, ni=_softbodyIndices.size(); i<ni; i++)
	{
		GSoftBody* sb=softbody(_softbodyIndices[i]);
		//for(int ii=0; ii<100; ii++) // speed test
		//sb->m_pSystem->m_TotalForces_ext.zero();
		//sb->m_pSystem->m_TotalForces_ext[1]=vector3(1,1000,0);
		/*sb->m_pSystem->m_TotalForces_int=sb->m_internalForces;
		sb->m_pSystem->m_vTemp1=sb->m_b_f;
		sb->m_pSystem->UpdateDV_Implicit_part2(getTimestep());
		sb->m_pSystem->IntegrateDV(getTimestep());
		*/
		//sb->m_pSystem->m_TotalForces_ext.zero();

		//sb->m_pSystem->Update(getTimestep());
		sb->m_pSystem->Update_Implicit(getTimestep());
		//sb->m_pSystem->Update_SemiImplicit(getTimestep());
		//sb->m_pSystem->Update_Explicit(getTimestep());

	}

	_updateCharacterPose();

	for(int i=0; i<world.numBodies(); i++)
		world.body(i)->clearExternalForces();
	indices.setSize(0);

	return true;
}

void TRL::DynamicsSimulator_TRL_massSpring::setParameter(const char* _what, double value)
{
	TString what(_what);

	if(what=="mass")
		_mass=value;
	else if(what=="stiffness")
		_stiffness=value;
	else if(what=="springDamp")
		_springDamp=value;
	else if(what=="kDF")
		_kDF=value;
	else if(what=="kDFvelThr")
		_kDFvelThr=value;
	else if(what=="bendingConstraints1")
		_bendingConstraints1=value;
	else if(what=="bendingStiffness")
		_bendingStiffness=value;
	else if(what=="integrateMethod")
		_integrateMethod=value;
	else if(what=="debugDrawWorld")
		_debugDrawWorld=value!=0.0;
	else if(what=="drawContactForce")
		_drawContactForce=value!=0.0;
	else if(what=="drawSimulatedMesh")
		_drawSimulatedMesh=value!=0.0;
	else if(what=="drawSimulatedMeshOffset")
		_drawSimulatedMeshOffset=value;
	else 
	{
		/*
		ASSERT(m_pSystem);
		if(what=="penaltyStiffness")
		{
			m_pSystem->m_cfg.penaltyStiffness=value;
		}
		else if(what=="penaltyDampness")
		{
			m_pSystem->m_cfg.penaltyDampness=value;
		}
		else if(what=="usePenaltyMethod")
		{
			if(value==1.0)
				m_pSystem->m_cfg.contactMethod=Physics_ParticleSystem  ::Config::PENALTY_METHOD;
		}
		else if(what=="contactMethod")
		{
			m_pSystem->m_cfg.contactMethod=(int)value;
		}
		else if(what=="penaltyMuScale")
		{
			m_pSystem->m_cfg.penaltyMuScale=value;
		}
		else
			Msg::error("unknown parameter %s", _what);
			*/
	}
}
