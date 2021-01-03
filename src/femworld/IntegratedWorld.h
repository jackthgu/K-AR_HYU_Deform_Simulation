#ifndef __INTEGRATED_WORLD_H__
#define __INTEGRATED_WORLD_H__

#include "fem/fem.h"
#include "src/ElasticBody/TetraMesh.h"
#include <BaseLib/motion/VRMLloader.h>

typedef std::pair<Bone*,Eigen::Vector3d> AnchorPoint;
Eigen::Vector3d GetPoint(const AnchorPoint& ap);

struct Muscle
{
	void TransferForce(Eigen::Vector3d& f_origin,Eigen::Vector3d& f_insertion);
	void SetActivationLevel(double a);
	std::string 							name;
	FEM::MeshPtr							mesh;
	std::vector<AnchorPoint>				origin_way_points,insertion_way_points;
	double				activation_level;
	Eigen::Vector3d		origin_force,insertion_force;
	

	FEM::AttachmentCstPtr								origin,insertion;
	std::vector<FEM::LinearMuscleCstPtr>				muscle_csts;
	std::vector<FEM::CstPtr>							csts;
	
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Muscle(const Muscle& other) = delete;
	Muscle& operator=(const Muscle& other) = delete;
	static std::shared_ptr<Muscle> Create();
private:
	Muscle();

};

class IntegratedWorld
{
public:
	IntegratedWorld(double timestep, double damping);
	virtual ~IntegratedWorld();


	void addBody(const char* name, TetraMeshLoader::TetraMesh const& mesh, double m);
	void addBody(const char* muscle_param); // xml filename decribing the muscles modeled using soft-bodies.
	// returns index of the attachment constraint
	int addAttachmentConstraint(const char* name, double stiffness, int index, vector3 const& point);

	void setAttachmentPoint(int iCon, vector3 const& p)  { mAttachmentPoints[iCon]->SetP(toEigen(p));}
	
	// returns index of the linear muscle
	int addLinearMuscleConstraint(const char* name, double stiffness, int itet, TetraMeshLoader::TetraMesh const& mesh, vector3 const& fiber_direction);


	void setActivationLevel(int iMuscle, double a)  { mLinearMuscles[iMuscle]->SetActivationLevel(a);}
	double getActivationLevel(int iMuscle)  { return mLinearMuscles[iMuscle]->GetActivationLevel();}

	void initSimulation() { mSoftWorld->Initialize();}
	bool TimeStepping();

	const FEM::WorldPtr& GetSoftWorld(){return mSoftWorld;};	

	// for drawing
	void getAttachmentCst(vector3N& points);
	void getCorotateFEMCst(vector3N& lines); // p0-p1, p2-p3, ... six lines per a tet.
	void getLinearMuscleCst(vector3N& lines,  vectorn& activation);
	vector3 getVertex(int inode) const;

private:
	std::vector<FEM::LinearMuscleCstPtr > mLinearMuscles;
	std::vector<FEM::AttachmentCstPtr > mAttachmentPoints;
	void AddMuscle( const std::string& name, const std::vector<AnchorPoint>& origin, const std::vector<AnchorPoint>& insertion, int origin_index,int insertion_index, const Eigen::Vector3d& fiber_direction, const FEM::MeshPtr& mesh);
	void MakeMuscles(const std::string& path);

	FEM::WorldPtr mSoftWorld;
	//
	//Muscles 
	std::vector<std::shared_ptr<Muscle>>	mMuscles;

	//Material Properties
	double	mTendonStiffness;
	double	mMuscleStiffness;
	double	mYoungsModulus;
	double	mPoissonRatio;

	Eigen::VectorXd							mActivationLevels;
	std::vector<FEM::LinearMuscleCstPtr>	mAllMuscleConstraints;
	void TransformAttachmentPoints();
	void SetActivationLevels(const Eigen::VectorXd& a);
};


#endif
