#include "stdafx.h"

#include "gymnist_def.h"
#include "MotionSynthesisLib/RigidBody/SDFAST_implementation/DynamicsSimulator_SDFAST.h"
#include "../MainLib/OgreFltk/VRMLloader.h"
static OpenHRP::DynamicsSimulator_SDFAST* gymnistsimulator=NULL;
void gymnistuforce(double t, double q[], double u[])
{
	// user provides this
	OpenHRP::DynamicsSimulator_SDFAST& sim=*gymnistsimulator;

	sim.uforce(t,q,u);
}

void gymnistmassmat2(matrixn & mat)
{
	double mmat[54][54];
	gymnistmassmat(mmat);
	mat.setSize(54,54);
	for (int i=0; i<54; i++){
		double * mrow=&mmat[i][0];
		double * row=&mat[i][0];
		
		for (int j=0; j<54; j++)
			row[j]=mrow[j];
	}
}

void connect_gymnist(OpenHRP::DynamicsSimulator_SDFAST& sim)
{

	sim._SDinit=&gymnistinit;
	sim._SDprinterr=&gymnistprinterr;
	sim._SDerror=&gymnisterror;
	sim._SDclearerr=&gymnistclearerr;
	sim._SDstatic=&gymniststatic;
	sim._SDpointf =&gymnistpointf;
	sim._SDtrans=&gymnisttrans;
	sim._SDhinget=&gymnisthinget;
	sim._SDbodyt=&gymnistbodyt;
	sim._SDangvel=&gymnistangvel;
	sim._SDgetbtj=&gymnistgetbtj;
	sim._SDgetitj=&gymnistgetitj;
	sim._SDvel=&gymnistvel;
	sim._SDpos=&gymnistpos;
	sim._SDorient=&gymnistorient;
	sim._SDmotion=&gymnistmotion;
	sim._SDfmotion=&gymnistfmotion;
	sim._SDreac=&gymnistreac;
	sim._SDacc=&gymnistacc;
	sim._SDgetgrav =&gymnistgetgrav;
	sim._SDgetmass=&gymnistgetmass;
	sim._SDinfo=&gymnistinfo;
	sim._SDindx=&gymnistindx;
	sim._SDuforce =&gymnistuforce;
	sim._SDstate=&gymniststate;
	sim._var=&gymnistsimulator;
	sim._SDmassmat=&gymnistmassmat2;

	gymnistsimulator=&sim;
	sim._SDbtj=NULL;
	sim._SDitj=NULL;
	sim._SDmass=NULL;
	sim._SDiner=NULL;
}

	// copy following sentences to DynammicsSImulator_SDFAST.cpp

	/*
//////////////////////////////////////////////////////
	// -> just before void DynamicsSimulator_SDFAST::init
	//////////////////////////////////////////////////////
	void connect_gymnist(OpenHRP::DynamicsSimulator_SDFAST& sim);

	//////////////////////////////////////////////////////
	// -> inside the init() function.
	//////////////////////////////////////////////////////
	else if(_characters[0]->skeleton->url.right(11).toUpper()=="GYMNIST.WRL")
		connect_gymnist(*this);
*/

