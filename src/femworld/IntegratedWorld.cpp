
#include "stdafx.h"
#include "IntegratedWorld.h"
//#include <tinyxml.h>
#include <algorithm>
#include <PhysicsLib/TRL/eigenSupport.h>
using namespace FEM;


Muscle::Muscle()
{

}
std::shared_ptr<Muscle>
Muscle::Create()
{
	auto nm = new Muscle();

	return std::shared_ptr<Muscle>(nm);
}
Eigen::Vector3d
GetPoint(const AnchorPoint& ap)
{
	//return ap.first->getTransform()*ap.second;
	return ap.second;
}
static Bone* findBone(IntegratedWorld& ms, const char* name)
{
	return NULL;
}

void
Muscle::TransferForce(Eigen::Vector3d& f_origin,Eigen::Vector3d& f_insertion)
{
	int no = origin_way_points.size();
	int ni = insertion_way_points.size();

	if(no>1)
	{
		Eigen::Vector3d u = (GetPoint(insertion_way_points[0])-GetPoint(origin_way_points[0])).normalized();
		Eigen::Vector3d v = (GetPoint(origin_way_points[no-2])-GetPoint(origin_way_points[no-1])).normalized();
		double angle = acos(u.dot(v));
		Eigen::Vector3d axis = u.cross(v);
		axis.normalize();
		Eigen::AngleAxisd aa(angle,axis);
		f_origin = aa.toRotationMatrix()*f_origin;
	}

	if(ni>1)
	{
		Eigen::Vector3d u = (GetPoint(origin_way_points[0])-GetPoint(insertion_way_points[0])).normalized();
		Eigen::Vector3d v = (GetPoint(insertion_way_points[ni-2])-GetPoint(insertion_way_points[ni-1])).normalized();
		double angle = acos(u.dot(v));
		Eigen::Vector3d axis = u.cross(v);
		axis.normalize();
		Eigen::AngleAxisd aa(angle,axis);
		f_insertion = aa.toRotationMatrix()*f_insertion;
	}
}
void
Muscle::SetActivationLevel(double a)
{
	for(auto& lmc : muscle_csts)
		lmc->SetActivationLevel(a);

	activation_level = a;
}
void
IntegratedWorld::TransformAttachmentPoints()
{
	for(auto& muscle : mMuscles)
	{
		auto& origin_way_points = muscle->origin_way_points;
		auto& insertion_way_points = muscle->insertion_way_points;

		Eigen::Vector3d po = GetPoint(origin_way_points[0]);
		Eigen::Vector3d pi = GetPoint(insertion_way_points[0]);

		muscle->origin->SetP(po);
		muscle->insertion->SetP(pi);
	}
}

void IntegratedWorld::getAttachmentCst(vector3N& points)
{
	auto& world=mSoftWorld;
	const Eigen::VectorXd& X = world->GetPositions();
	const auto& cs = world->GetConstraints();

	points.setSize(0);
	for(const auto& c : cs)
	{
		if(dynamic_cast<AttachmentCst*>(c.get()) != nullptr)
		{
			AttachmentCst* ac = dynamic_cast<AttachmentCst*>(c.get());	
			int i0 = ac->GetI0();
			const Eigen::Vector3d& p = ac->GetP();
			points.pushBack(toBase(p));
		}
	}
}
vector3 IntegratedWorld::getVertex(int inode) const
{
	auto& world=mSoftWorld;
	// p0-p1, p2-p3, ... six lines per a tet.
	const Eigen::VectorXd& x = world->GetPositions();
	const Eigen::Vector3d& p0 = x.block<3,1>(inode*3,0);
	return toBase(p0);
}

void IntegratedWorld::getCorotateFEMCst(vector3N& lines)
{
	auto& world=mSoftWorld;
	// p0-p1, p2-p3, ... six lines per a tet.
	const Eigen::VectorXd& x = world->GetPositions();
	const auto& cs = world->GetConstraints();

	lines.setSize(0);
	for(const auto& c : cs)
	{
		if(dynamic_cast<CorotateFEMCst*>(c.get()) != nullptr)
		{
			CorotateFEMCst* cc = dynamic_cast<CorotateFEMCst*>(c.get());
			int i0 = cc->GetI0();
			int i1 = cc->GetI1();
			int i2 = cc->GetI2();
			int i3 = cc->GetI3();

			const Eigen::Vector3d& p0 = x.block<3,1>(i0*3,0);
			const Eigen::Vector3d& p1 = x.block<3,1>(i1*3,0);
			const Eigen::Vector3d& p2 = x.block<3,1>(i2*3,0);
			const Eigen::Vector3d& p3 = x.block<3,1>(i3*3,0);

			lines.pushBack(toBase(p0)); lines.pushBack(toBase(p1));
			lines.pushBack(toBase(p2)); lines.pushBack(toBase(p3));
			lines.pushBack(toBase(p0)); lines.pushBack(toBase(p2));
			lines.pushBack(toBase(p0)); lines.pushBack(toBase(p3));
			lines.pushBack(toBase(p1)); lines.pushBack(toBase(p2));
			lines.pushBack(toBase(p1)); lines.pushBack(toBase(p3));
		}
	}
}	

void IntegratedWorld::getLinearMuscleCst(vector3N& lines,  vectorn& activation)
{
	auto& world=mSoftWorld;
	const Eigen::VectorXd& x = world->GetPositions();
	const auto& cs = world->GetConstraints();

	lines.setSize(0);
	activation.setSize(0);
	for(const auto& c : cs)
	{
		LinearMuscleCst* cc = dynamic_cast<LinearMuscleCst*>(c.get());
		int i0 = cc->GetI0();
		int i1 = cc->GetI1();
		int i2 = cc->GetI2();
		int i3 = cc->GetI3();
		double a = cc->GetActivationLevel();
		const Eigen::Vector3d& p0 = x.block<3,1>(i0*3,0);
		const Eigen::Vector3d& p1 = x.block<3,1>(i1*3,0);
		const Eigen::Vector3d& p2 = x.block<3,1>(i2*3,0);
		const Eigen::Vector3d& p3 = x.block<3,1>(i3*3,0);

		activation.pushBack(a);
		lines.pushBack(toBase(p0));
		lines.pushBack(toBase(p1));
		lines.pushBack(toBase(p2));
		lines.pushBack(toBase(p3));
	}
}
void
IntegratedWorld::AddMuscle(
	const std::string& name,
	const std::vector<AnchorPoint>& origin,
	const std::vector<AnchorPoint>& insertion,
	int origin_index,int insertion_index,
	const Eigen::Vector3d& fiber_direction,
	const MeshPtr& mesh)
{
	mMuscles.push_back(Muscle::Create());
	auto& muscle = mMuscles.back();

	muscle->name = name;
	muscle->mesh = mesh;
	muscle->origin_way_points = origin;
	muscle->insertion_way_points = insertion;

	muscle->origin = AttachmentCst::Create(name+"_origin",mTendonStiffness,origin_index,GetPoint(origin[0]));
	muscle->insertion = AttachmentCst::Create(name+"_insertion",mTendonStiffness,insertion_index,GetPoint(insertion[0]));
	muscle->activation_level = 0.0;

	const auto& tetrahedrons = muscle->mesh->GetTetrahedrons();
	const auto& vertices = muscle->mesh->GetVertices();
	int tet_index = 0;
	for(const auto& tet: tetrahedrons)
	{
		int i0,i1,i2,i3;
		Eigen::Vector3d p0,p1,p2,p3;

		i0 = tet[0];
		i1 = tet[1];
		i2 = tet[2];
		i3 = tet[3];

		p0 = vertices[i0];
		p1 = vertices[i1];
		p2 = vertices[i2];
		p3 = vertices[i3];

		Eigen::Matrix3d Dm;

		Dm.block<3,1>(0,0) = p1 -p0;
		Dm.block<3,1>(0,1) = p2 -p0;
		Dm.block<3,1>(0,2) = p3 -p0;

		//Peventing inversion
		if(Dm.determinant()<0)
		{
			i2 = tet[3];
			i3 = tet[2];
			
			p2 = vertices[i2];
			p3 = vertices[i3];

			Dm.block<3,1>(0,1) = p2-p0;
			Dm.block<3,1>(0,2) = p3-p0;
		}

		muscle->csts.push_back(CorotateFEMCst::Create(name+"_element_"+std::to_string(tet_index),
			mYoungsModulus,
			mPoissonRatio,
			i0,i1,i2,i3,1.0/6.0*Dm.determinant(),Dm.inverse()));

		muscle->muscle_csts.push_back(LinearMuscleCst::Create(name+"_muscle_"+std::to_string(tet_index),
			mMuscleStiffness,
			i0,i1,i2,i3,1.0/6.0*Dm.determinant(),Dm.inverse(),
			fiber_direction));
		mAllMuscleConstraints.push_back(muscle->muscle_csts.back());
		tet_index++;
	}
	for(auto c: muscle->muscle_csts) muscle->csts.push_back(c);
	muscle->csts.push_back(muscle->origin);
	muscle->csts.push_back(muscle->insertion);
}
void IntegratedWorld::SetActivationLevels(const Eigen::VectorXd& a)
{
	mActivationLevels = a;
	for(int i =0;i<mMuscles.size();i++)
		mMuscles[i]->SetActivationLevel(a[i]);
}
bool
IntegratedWorld::TimeStepping()
{

	// After SetActivationLevels(...)
	//TransformAttachmentPoints();
	mSoftWorld->TimeStepping();

	return true;
}

void IntegratedWorld::MakeMuscles(const std::string& path)
{
	ASSERT(false);/*
	TiXmlDocument doc;
    if(!doc.LoadFile(path))
    {
        std::cout<<"Cant open XML file : "<<path<<std::endl;
        return;
    }

    TiXmlElement* muscles = doc.FirstChildElement("Muscles");

    for(TiXmlElement* unit = muscles->FirstChildElement("unit");unit!=nullptr;unit = unit->NextSiblingElement("unit"))
    {
        TiXmlElement* ori = unit->FirstChildElement("origin");
        std::string name = (unit->Attribute("name"));
        std::vector<AnchorPoint> p_ori,p_ins;
       
        for(TiXmlElement* anc = ori->FirstChildElement("anchor");anc!=nullptr;anc = anc->NextSiblingElement("anchor"))   
        {
            std::string body_name = anc->Attribute("body");
            double x = std::stod(anc->Attribute("x"));
            double y = std::stod(anc->Attribute("y"));
            double z = std::stod(anc->Attribute("z"));

            //auto T = skel->getBodyNode(body_name.c_str())->getShapeNodesWith<VisualAspect>()[0]->getRelativeTransform();
            //auto T1 = skel->getBodyNode(body_name.c_str())->getTransform();
            Eigen::Vector3d body_coord(x,y,z);
            body_coord*=0.01;
            //body_coord = T* body_coord;

            p_ori.push_back(AnchorPoint(findBone(*this, body_name.c_str()),body_coord));
        }
        std::reverse(p_ori.begin(),p_ori.end());
        TiXmlElement* ins = unit->FirstChildElement("insertion");
        for(TiXmlElement* anc = ins->FirstChildElement("anchor");anc!=nullptr;anc = anc->NextSiblingElement("anchor"))   
        {
            std::string body_name = anc->Attribute("body");
            double x = std::stod(anc->Attribute("x"));
            double y = std::stod(anc->Attribute("y"));
            double z = std::stod(anc->Attribute("z"));

            //auto T = skel->getBodyNode(body_name.c_str())->getShapeNodesWith<VisualAspect>()[0]->getRelativeTransform();
            //auto T1 = skel->getBodyNode(body_name.c_str())->getTransform();
            Eigen::Vector3d body_coord(x,y,z);
            body_coord*=0.01;
            //body_coord = T* body_coord;

            p_ins.push_back(AnchorPoint(findBone(*this, body_name.c_str()),body_coord));
        }

        Eigen::Vector3d muscle_start,muscle_end;

        muscle_start = GetPoint(p_ori[0]);
        muscle_end = GetPoint(p_ins[0]);

        double len = (muscle_start - muscle_end).norm();
        Eigen::Vector3d unit_dir = (muscle_start - muscle_end).normalized();

        Eigen::Vector3d axis = Eigen::Vector3d::UnitX().cross(unit_dir);
        double cos_angle = unit_dir[0];
        double sin_angle = axis.norm();

        double angle = atan2(sin_angle,cos_angle);
        TiXmlElement* mesh_element = unit->FirstChildElement("mesh");

        Eigen::Affine3d T = Eigen::Affine3d::Identity();
        T.translation() = 0.5*(muscle_start + muscle_end);
        T.linear() = len*(Eigen::AngleAxisd(angle,axis.normalized()).matrix());
        int nx = std::stoi(mesh_element->Attribute("nx"));
        int ny = std::stoi(mesh_element->Attribute("ny"));
        double ratio = std::stod(mesh_element->Attribute("ratio"));
        auto dm = DiamondMesh::Create(1.0,(double)ny/(double)nx*ratio,(double)ny/(double)nx*ratio,nx,ny,ny,T);
        int i_ori = dm->GetEndingPointIndex();
        int i_ins = dm->GetStartingPointIndex();

        AddMuscle(name,p_ori,p_ins,i_ori,i_ins,unit_dir,dm);
        
        
    }
	*/
}

IntegratedWorld::IntegratedWorld(double timestep, double damping)
	:mTendonStiffness(1E5),mMuscleStiffness(1E6),mYoungsModulus(1E6),mPoissonRatio(0.3)
{
	mSoftWorld = FEM::World::Create(
		//FEM::IntegrationMethod::PROJECTIVE_QUASI_STATIC,	//Integration Method
		//FEM::IntegrationMethod::PROJECTIVE_QUASI_STATIC,
		 FEM::IntegrationMethod::PROJECTIVE_DYNAMICS,	//Integration Method
	//FEM::IntegrationMethod::NEWTON_METHOD, // slow
		timestep,							//Time Step
		10,								//Max Iteration
//		Eigen::Vector3d(0,-9.81,0),					//Gravity
		 Eigen::Vector3d(0,0,0),					//Gravity
		 
		damping
		);
}
IntegratedWorld::~IntegratedWorld()
{
}

int IntegratedWorld::addAttachmentConstraint(const char* name, double stiffness, int index, vector3 const& point)
{
	FEM::AttachmentCstPtr cst=AttachmentCst::Create(name,stiffness,index,toEigen(point));
	mSoftWorld->AddConstraint(cst);
	// for later use.
	mAttachmentPoints.push_back(cst);
	return mAttachmentPoints.size()-1;
}
int IntegratedWorld::addLinearMuscleConstraint(const char* name, double stiffness, int itet, TetraMeshLoader::TetraMesh const& mesh, vector3 const& fiber_direction)
{
	int i0,i1,i2,i3;
	Eigen::Vector3d p0,p1,p2,p3;
	_quadi const& tet=mesh.getTetra(itet);

	i0 = tet[0];
	i1 = tet[1];
	i2 = tet[2];
	i3 = tet[3];

	p0 = toEigen(mesh.nodePos(i0));
	p1 = toEigen(mesh.nodePos(i1));
	p2 = toEigen(mesh.nodePos(i2));
	p3 = toEigen(mesh.nodePos(i3));

	Eigen::Matrix3d Dm;

	Dm.block<3,1>(0,0) = p1 -p0;
	Dm.block<3,1>(0,1) = p2 -p0;
	Dm.block<3,1>(0,2) = p3 -p0;

	//Peventing inversion
	if(Dm.determinant()<0)
	{
		i2 = tet[3];
		i3 = tet[2];

		p2 = toEigen(mesh.nodePos(i2));
		p3 = toEigen(mesh.nodePos(i3));

		Dm.block<3,1>(0,1) = p2-p0;
		Dm.block<3,1>(0,2) = p3-p0;
	}

	std::shared_ptr<FEM::LinearMuscleCst> cst=LinearMuscleCst::Create(name, stiffness,  
			i0,i1,i2,i3,1.0/6.0*Dm.determinant(),Dm.inverse(),
			toEigen(fiber_direction));

	mSoftWorld->AddConstraint(cst);

	// for later use.
	mLinearMuscles.push_back(cst);
	return mLinearMuscles.size()-1;
}
void IntegratedWorld::addBody(const char* name, TetraMeshLoader::TetraMesh const& mesh, double m)
{
	int numNode=mesh.getNumNode();
	Eigen::VectorXd v(numNode*3);
	for(int i=0; i<numNode;i++)
		vecView(v).setVec3(i*3, mesh.nodePos(i));

	int numTetra=mesh.getNumTetra();
	std::vector<std::shared_ptr<FEM::Cst> > constraints;
	for(int i=0; i<numTetra;i++)
	{
		_quadi const & tet=mesh.getTetra(i);
		int tet_index=i;

		int i0,i1,i2,i3;
		Eigen::Vector3d p0,p1,p2,p3;

		i0 = tet[0];
		i1 = tet[1];
		i2 = tet[2];
		i3 = tet[3];

		p0 = toEigen(mesh.nodePos(i0));
		p1 = toEigen(mesh.nodePos(i1));
		p2 = toEigen(mesh.nodePos(i2));
		p3 = toEigen(mesh.nodePos(i3));

		Eigen::Matrix3d Dm;

		Dm.block<3,1>(0,0) = p1 -p0;
		Dm.block<3,1>(0,1) = p2 -p0;
		Dm.block<3,1>(0,2) = p3 -p0;

		//Peventing inversion
		if(Dm.determinant()<0)
		{
			i2 = tet[3];
			i3 = tet[2];
			
			p2 = toEigen(mesh.nodePos(i2));
			p3 = toEigen(mesh.nodePos(i3));

			Dm.block<3,1>(0,1) = p2-p0;
			Dm.block<3,1>(0,2) = p3-p0;
		}
		constraints.push_back(CorotateFEMCst::Create(std::string(name)+"_element_"+std::to_string(tet_index),
			mYoungsModulus,
			mPoissonRatio,
			i0,i1,i2,i3,1.0/6.0*Dm.determinant(),Dm.inverse()));

	}

	mSoftWorld->AddBody(v, constraints, m);
}
void IntegratedWorld::addBody(const char* muscle_param)
{
	MakeMuscles(muscle_param);
	{
		// mMusculoSkeletalSystem->Initialize(mSoftWorld,mRigidWorld);
		const std::shared_ptr<FEM::World>& soft_world=mSoftWorld;
		for(int i =0;i<mMuscles.size();i++)
		{
			int offset = soft_world->GetNumVertices();
			auto& muscle = mMuscles[i];

			const auto& vertices = muscle->mesh->GetVertices();

			for(auto& c: muscle->csts)
				c->AddOffset(offset);
			Eigen::VectorXd v(vertices.size()*3);
			for(int i =0;i<vertices.size();i++)
				v.block<3,1>(i*3,0) = vertices[i];

			soft_world->AddBody(v,muscle->csts,1.0);
		}

		mActivationLevels.resize(mMuscles.size());
		mActivationLevels.setZero();
	}
}
