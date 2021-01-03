#ifndef IguanaIKSolver_H_
#define IguanaIKSolver_H_

class IguanaIKSolver : public MotionUtil::FullbodyIK
{
	scoped_ptr<MotionUtil::FullbodyIK> ik;	
	MotionLoader& skel;
	btRigidBody* mTerrain;
	std::vector<MotionUtil::Effector> ee;
	int numEE;
public:

	IguanaIKSolver (MotionLoader& skel, btRigidBody* terrain);
	virtual~ IguanaIKSolver ();
	virtual void getAffectedDOF(intvectorn & rot_joint_index, intvectorn & trans_joint_index);
	virtual void IKsolve(Posture const& input_pose, vector3N const& constraintPositions, intvectorn & rot_joint_index, quaterN& delta_rot, intvectorn & trans_joint_index, vector3N& delta_trans);
};
#endif
