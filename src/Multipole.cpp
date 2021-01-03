#include "stdafx.h"
#include "Multipole.h"
#include <OgreRoot.h>
#ifdef _DEBUG
#pragma comment(lib, "../dependencies/RLtoolbox/external_library/Torch3.lib")
#pragma comment(lib, "../dependencies/RLtoolbox/RL_Toolbox_Windows_Source_20b/Debug/RL Toolbox Debug.lib")
#else
#pragma comment(lib, "../dependencies/RLtoolbox/external_library/Torch3.lib")
#pragma comment(lib, "../dependencies/RLtoolbox/RL_Toolbox_Windows_Source_20b/Release/RL Toolbox Release.lib")
#endif


#include <time.h>
#include "ril_debug.h"
//#include "cmultipolemodel.h"
#include "ctdlearner.h"
#include "cpolicies.h"
#include "cagent.h"
#include "clinearfafeaturecalculator.h"
#include "cqFunction.h"
#include "cqetraces.h"
#include "cactorcritic.h"
#include "cvFunction.h"
#include "cvfunctionlearner.h"

#define one_degree 0.0174532	/* 2pi/360 */
#define six_degrees 0.1047192
#define twelve_degrees 0.2094384
#define fifty_degrees 0.87266

FlLayout* createMultipoleWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer)
{
	return new MultipoleWin(x,y,w,h,mp, renderer);
}



MultipoleWin ::MultipoleWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer)
:FlLayout(x,y,w,h)
,m_motionPanel(mp)
,mRenderer(renderer)
{
	create("Button", "Start", "Start");
	updateLayout();	
}

MultipoleWin ::~MultipoleWin (void)
{

}

class CMultiPoleAction2 : public CPrimitiveAction
{
public:
	rlt_real force;
	CMultiPoleAction2(rlt_real _force):CPrimitiveAction(), force(_force){}
	rlt_real getForce()	{return force;}
};

class CMultiPoleModel2 : public CEnvironmentModel, public CRewardFunction
{
protected:
	/// internal state variables
	rlt_real x, x_dot, theta, theta_dot; 
	/// calculate the next state based on the action
	virtual void doNextState(CPrimitiveAction *action); 

public:
	CMultiPoleModel2();
	~CMultiPoleModel2();

	///returns the reward for the transition, implements the CRewardFunction interface
	virtual rlt_real getReward(CStateCollection *oldState, CAction *action, CStateCollection *newState); 
	///fetches the internal state and stores it in the state object
	virtual void getState(CState *state);
	///resets the model
	virtual void doResetModel();
};


#define GRAVITY 9.8
#define MASSCART 1.0
#define MASSPOLE 0.1
#define TOTAL_MASS (MASSPOLE + MASSCART)
#define LENGTH 0.5		  /* actually half the pole's length */
#define POLEMASS_LENGTH (MASSPOLE * LENGTH)
#define FORCE_MAG 10.0
#define TAU 0.02		  /* seconds between state updates */
#define FOURTHIRDS 1.3333333333333



CMultiPoleModel2::CMultiPoleModel2() : CEnvironmentModel(4, 0)
{
	x= x_dot = theta = theta_dot = 0;

	properties->setMinValue(0, -2.4 * 1.1);
	properties->setMaxValue(0, 2.4 * 1.1);

	properties->setMinValue(1, -2);
	properties->setMaxValue(1, 2);

	properties->setMinValue(2, -twelve_degrees * 1.1);
	properties->setMaxValue(2, twelve_degrees * 1.1);

	properties->setMinValue(3, -fifty_degrees * 1.5);
	properties->setMaxValue(3, fifty_degrees * 1.5);
}

CMultiPoleModel2::~CMultiPoleModel2() {
}

// Store the model state to the given state object
void CMultiPoleModel2::getState(CState *state)
{
	///resets the state object
	CEnvironmentModel::getState(state);

	// Set the 4 internal state variables to the 
	// continuous state variables of the model state
	state->setContinuousState(0, x);
	state->setContinuousState(1, x_dot);
	state->setContinuousState(2, theta);
	state->setContinuousState(3, theta_dot);
}

void CMultiPoleModel2::doNextState(CPrimitiveAction *act)
{
	rlt_real xacc,thetaacc,force,costheta,sintheta,temp;
	// cast the action to CMultiPoleAction2
	CMultiPoleAction2* action = (CMultiPoleAction2*)(act);
	// determine the force    
	force = action->getForce();

	// calculate the new state
	costheta = cos(theta);
	sintheta = sin(theta);
	temp = (force + POLEMASS_LENGTH * theta_dot * theta_dot * sintheta) / TOTAL_MASS;
	thetaacc = (GRAVITY * sintheta - costheta* temp) / (LENGTH * (FOURTHIRDS - MASSPOLE * costheta * costheta / TOTAL_MASS));
	xacc  = temp - POLEMASS_LENGTH * thetaacc* costheta / TOTAL_MASS;
	/*** Update the four state variables, using Euler's method. ***/
	x  += TAU * x_dot;
	x_dot += TAU * xacc;
	theta += TAU * theta_dot;
	theta_dot += TAU * thetaacc;

	// determine wether the episode has failed
	if (x < -2.4 ||
		x > 2.4  ||
		theta < -twelve_degrees ||
		theta > twelve_degrees) {
			reset = true;
			failed = true;
	}
	// indicate that a new episode has begun
	if (reset)
	{
		printf("Failed State: x = %f; theta = %f\n", x, theta);
	}

	// taesoo drawing code
	Ogre::SceneNode* pNode=RE::createEntity("cart_pole", "cube.mesh");
	vector3 v1, v2;
	quater q;
	q.setRotation(vector3(0,0,1), theta);
	v1.setValue(x*10,0,0);
	v2.rotate(q,vector3(0,20,0));
	v2+=v1;

	float thick=4;
	pNode->resetToInitialState();
	pNode->scale(0.01*thick, 0.01*(v2-v1).length(), 0.01*thick);
	pNode->rotate(ToOgre(q));
	pNode->translate(ToOgre((v1+v2)/2));

	RE::renderer().mRoot->renderOneFrame();
//	printf("doNextState\n");
}

void CMultiPoleModel2::doResetModel()
{
	/// Reset internal state variables
	x = x_dot = theta = theta_dot = 0;
//	printf("reset model\n");
}

rlt_real CMultiPoleModel2::getReward(CStateCollection *oldStatecol, CAction *action, CStateCollection *newStateCol) {
	rlt_real rew;
	CState *newState = newStateCol->getState(getStateProperties());

	rlt_real x = newState->getContinuousState(0);
	rlt_real theta = newState->getContinuousState(2);

	// calculate the reward:
	// -1: for failed
	// 0 : else
	if (x < -2.4 ||
		x > 2.4  ||
		theta < -twelve_degrees ||
		theta > twelve_degrees)
	{
		rew = - 1.0;
	}
	else rew = 0.0;

	return rew;
}


enum { CONTINUOUS, DISCRETE, DISCRETE_ACTOR_CRITIC};
void MultipoleWin ::onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData)
{
	if(w.mId=="Start")
	{
		/*****************************************************************************************************
		Tutorial for the Reinforcement Learning Toolbox. In this example the Pole Balancing task is learned with
		a Q-Learning algorithm. For discretization build-in single state discretizer are used. 
		The task is learned until the agent manages to pole the cart for over 100000 steps or for 500 episodes.
		******************************************************************************************************/

		// variable declaration
		
		
		CAbstractStateDiscretizer *discState = NULL;
			
		// create the model
		CMultiPoleModel2 *model = new CMultiPoleModel2();
		// initialize the reward function
		/* Our environment model also implements the reward function interface */
		CRewardFunction *rewardFunction = model;

		// create the agent
		CAgent *agent = new CAgent(model); 

		CAgentController *policy =NULL;
		CTDLearner *learner =NULL;
		CFeatureQFunction *qTable =NULL;

		// create the 2 actions for accelerating the cart and add them to the agent's action set
		CPrimitiveAction *primAction1 = new CMultiPoleAction2(10.0);
		CPrimitiveAction *primAction2 = new CMultiPoleAction2(-10.0);

		agent->addAction(primAction1);
		agent->addAction(primAction2);

		int option=DISCRETE_ACTOR_CRITIC;
		switch(option)
		{
		case CONTINUOUS:
			{
				unsigned int dimensions[] = {0, 1, 2, 3};
				unsigned int partitions[] = {5, 5, 10, 10};
				double offsets[] = {0.0, 0.0, 0.0, 0.0};
				double sigma[] = {0.1, 0.5, 0.05, 0.05};

				CFeatureCalculator *rbfCalc = new CRBFFeatureCalculator(4, dimensions, partitions, offsets, sigma);
				CFeatureCalculator *tilingCalc = new CTilingFeatureCalculator(4, dimensions, partitions, offsets);

				// add the discrete state to the agent's state modifier
				// discState must not be modified (e.g. with a State-Substitution) by now
				agent->addStateModifier(rbfCalc);
				agent->addStateModifier(tilingCalc);

				// Create the learner and the Q-Function
				qTable= new CFeatureQFunction(agent->getActions(), rbfCalc);
				learner= new CQLearner(rewardFunction, qTable);
				// initialise the learning algorithm parameters
				learner->setParameter("QLearningRate", 0.3);
				learner->setParameter("DiscountFactor", 0.95);
				learner->getETraces()->setReplacingETraces(false);
				learner->getETraces()->setLambda(0.95);
				learner->setParameter("ETraceMaxListSize", 400);
				learner->setParameter("ETraceTreshold", 0.0001);

				// add the Q-Learner to the listener list
				agent->addSemiMDPListener(learner);
				// Create the learners controller from the Q-Function, we use a SoftMaxPolicy
				policy= new CQStochasticPolicy(agent->getActions(), new CSoftMaxDistribution(10000.0), qTable);

				// set the policy as controller of the agent
				agent->setController(policy);
			}
			break;

		case DISCRETE:
		case DISCRETE_ACTOR_CRITIC:
			{
				// create the discretizer with the build in classes
				// create the partition arrays
				double partitions1[] = { -0.8, 0.8}; // partition for x
				double partitions2[] = {-0.5, 0.5}; // partition for x_dot
				double partitions3[] = {-six_degrees, -one_degree, 0, one_degree, six_degrees}; // partition for theta
				double partitions4[] = {-fifty_degrees, fifty_degrees}; // partition for theta_dot

				// Create the discretizer for the state variables
				CAbstractStateDiscretizer *disc1 = new CSingleStateDiscretizer(0, 2, partitions1);
				CAbstractStateDiscretizer *disc2 = new CSingleStateDiscretizer(1, 2, partitions2);
				CAbstractStateDiscretizer *disc3 = new CSingleStateDiscretizer(2, 5, partitions3);
				CAbstractStateDiscretizer *disc4 = new CSingleStateDiscretizer(3, 2, partitions4);

				// Merge the 4 discretizer
				CDiscreteStateOperatorAnd *andCalculator = new CDiscreteStateOperatorAnd();

				andCalculator->addStateModifier(disc1);
				andCalculator->addStateModifier(disc2);
				andCalculator->addStateModifier(disc3);
				andCalculator->addStateModifier(disc4);

				discState = andCalculator;

				// add the discrete state to the agent's state modifier
				// discState must not be modified (e.g. with a State-Substitution) by now
				agent->addStateModifier(discState);

				if(option==DISCRETE)
				{
					// Create the learner and the Q-Function
					CFeatureQFunction *qTable = new CFeatureQFunction(agent->getActions(), discState);

					CTDLearner *learner = new CQLearner(rewardFunction, qTable);
					// initialise the learning algorithm parameters
					learner->setParameter("QLearningRate", 0.1);
					learner->setParameter("DiscountFactor", 0.99);
					learner->setParameter("ReplacingETraces", 0.0);
					learner->setParameter("Lambda", 1.0);

					// Set the minimum value of a etrace, we need very small values
					learner->setParameter("ETraceTreshold", 0.00001);
					// Set the maximum size of the etrace list, standard is 100
					learner->setParameter("ETraceMaxListSize", 163);

					// add the Q-Learner to the listener list
					agent->addSemiMDPListener(learner);

					// Create the learners controller from the Q-Function, we use a SoftMaxPolicy
					policy = new CQStochasticPolicy(agent->getActions(), new CEpsilonGreedyDistribution(0.1), qTable);

					// set the policy as controller of the agent
					agent->setController(policy);
				}
				else
				{
					// Create the learner and the Q-Function
					CFeatureVFunction *vTableActor = new CFeatureVFunction(discState);
					CFeatureVFunction *vTableCritic = new CFeatureVFunction(discState);

					CVFunctionLearner *vLearner = new CVFunctionLearner(rewardFunction, vTableCritic);
					CActorFromActionValue *actor = new CActorFromActionValue(vTableActor, primAction1, primAction2);

					vLearner->addErrorListener(actor);

					vLearner->setParameter("VLearningRate", 0.1);
					vLearner->setParameter("Lambda", 0.85);
					vLearner->setParameter("ReplacingETraces", 0.0);

					actor->setParameter("ActorLearningRate", 100.0);
					actor->setParameter("ReplacingETraces", 0.0);
					actor->setParameter("Lambda", 0.95);

					agent->addSemiMDPListener(vLearner);

					// set the policy as controller of the agent
					agent->setController(actor);
				}
			}
			break;
		
		}

		// disable automatic logging of the current episode from the agent
		agent->setLogEpisode(false);


		int steps = 0;

		int max_Steps = 100000;
		// Learn for 500 Episodes
		for (int i = 0; i < 500; i++)
		{
			if(option==DISCRETE)
				policy->setParameter("EpsilonGreedy", 0.1 / (i + 1));

			// Do one training trial, with max max_Steps steps
			steps = agent->doControllerEpisode(1, max_Steps);

			printf("Episode %d %s with %d steps\n", i, model->isFailed() ? "failed" : "succeded", steps);

			if (steps >= max_Steps)
			{
				printf("Learned to balance the pole after %d Episodes\n", i);
				break;
			}
		}


		delete policy;
		delete learner;
		delete agent;
		delete qTable;
		delete model;

	}
}

// FrameMoveObject
int MultipoleWin ::FrameMove(float fElapsedTime)
{
	return 1;
}

void MultipoleWin ::show()
{
	__super::show();
}

void MultipoleWin ::hide()
{
	__super::hide();
}
