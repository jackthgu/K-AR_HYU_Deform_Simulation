// MotionLoader.cpp: implementation of the MotionLoader class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "MotionLoader.h"
#include "../BaseLib/utility/configtable.h"
#include "version.h"
#include "MotionDOF.h"
#include "MotionWrap.h"

	transf const& BoneForwardKinematics::global(const Bone& bone) const	{ return m_global[bone.treeIndex()];}
	transf const& BoneForwardKinematics::local(const Bone& bone) const	{ return m_local[bone.treeIndex()];}
	transf & BoneForwardKinematics::_global(const Bone& bone) 	{ return m_global[bone.treeIndex()];}
	transf & BoneForwardKinematics::_local(const Bone& bone) 	{ return m_local[bone.treeIndex()];}


BoneForwardKinematics::BoneForwardKinematics(MotionLoader* pskel)
:m_skeleton(pskel)
{	
}

void BoneForwardKinematics::forwardKinematics()
{
	// this is thread unsafe so.. 
	// NodeStack  & stack=m_skeleton->m_TreeStack;
	NodeStack  stack;
	stack.Initiate();

	Node *src=m_skeleton->m_pTreeRoot->m_pChildHead;	// dummy노드는 사용안함.
	int index=-1;

	while(TRUE) 
	{
		while(src)
		{
			index++;
			ASSERT(src->NodeType==BONE);
			Bone* pBone=(Bone*)src;
			int treeindex=pBone->treeIndex();
			if(stack.GetTop())
				_global(treeindex).mult(
				global(*((Bone*)stack.GetTop())), local(treeindex));
			else
				_global(treeindex)=local(treeindex);

			stack.Push(src);

			src=src->m_pChildHead;
		}
		stack.Pop(&src);
		if(!src) break;
		src=src->m_pSibling;
	}
}



void BoneForwardKinematics::init()
{
	m_local.resize(m_skeleton->numBone());
	m_global.resize(m_skeleton->numBone());
	m_local[0].identity();
	m_global[0].identity();

	for(int i=1, ni=m_skeleton->numBone(); i<ni; i++)
		m_local[i]=m_skeleton->bone(i).getOffsetTransform();

	forwardKinematics();
}

void BoneForwardKinematics::operator=(BoneForwardKinematics const& other)
{
	ASSERT(getSkeleton().numBone()==other.getSkeleton().numBone());

	for(int i=0,ni=getSkeleton().numBone(); i<ni; i++)
	{
		_local(i)=other.local(i);
		_global(i)=other.global(i);	
	}
}
void BoneForwardKinematics::setPose(const Posture& pose)
{
	if(m_local.size()==0)
		init();
	// update root position and rotations
	for(int ijoint=0, nj=m_skeleton->numRotJoint(); ijoint<nj; ijoint++)
	{
		// update rotations
		int target=m_skeleton->getTreeIndexByRotJointIndex(ijoint);
		m_local[target].rotation=pose.m_aRotations[ijoint];
	}

	for(int ijoint=0, nj=MIN(m_skeleton->numTransJoint(), pose.m_aTranslations.size()); ijoint<nj; ijoint++)
	{
		// update translations
		int target=m_skeleton->getTreeIndexByTransJointIndex(ijoint);
		m_local[target].translation=pose.m_aTranslations[ijoint];
	}

	forwardKinematics();
}

void BoneForwardKinematics::setPoseDOF(const vectorn& dof)
{
	// thread unsafe equivalent: setPose(m_skeleton->dofInfo.setDOF(poseDOF));	
	int start=0;
	MotionDOFinfo& dofInfo=m_skeleton->dofInfo;
	for(int i=1; i<m_skeleton->numBone(); i++)
	{
		Bone& bone=m_skeleton->bone(i);
		if(bone.transJointIndex()!=-1)
		{
			vector3& trans=m_local[bone.treeIndex()].translation;
			int nc=bone.getLocalTrans(trans, &dof[start]);
			start+=nc;
		}

		int ri=bone.rotJointIndex();
		if(ri==-1) continue;

		quater& rotation=m_local[bone.treeIndex()].rotation;
		if(dofInfo.hasQuaternion(i))
		{
			rotation=dof.toQuater(start);
			start+=4;
		}

		if(dofInfo.hasAngles(i))
		{
			int nc=bone.getLocalOri(rotation, &dof[start]);
			start+=nc;
		}
	}
	assert(start==dof.size());
	forwardKinematics();
}

int Bone::getLocalTrans(vector3& trans, const double* dof)
{
	trans.setValue(0,0,0);
	int nc=getTranslationalChannels().length(); 
	for(int c=0; c<nc; c++)
	{
		switch(getTranslationalChannels()[c])
		{
			case 'X':
				trans.x=dof[c];
				break;
			case 'Y':
				trans.y=dof[c];
				break;
			case 'Z':
				trans.z=dof[c];
				break;
		}
	}
	return nc;
}
int Bone::getLocalOri(quater& qRot, const double* dof)
{
	int nc=getRotationalChannels().length();
	if (getRotationalChannels().findChar(0,'A') != -1)
	{
		for(int i=0;i<nc;i++)
		{
			vector3 axis;
			quater temp;
			qRot.setValue(1, 0,0,0);

			int numChannels=strlen(getRotationalChannels());
			for(int i=0; i<numChannels; i++)
			{
				axis = getArbitraryAxis(i);
				temp.setRotation(axis, dof[i]);

				if(0)//(bRightToLeft)
					qRot.leftMult(temp);
				else
				{
					quater copy(qRot); qRot.mult(copy, temp);			
				}

			}

		}
	}
	else
	{
		m_real aValue[3];

		for(int c=0; c<nc; c++)
			aValue[c]=dof[c];

		qRot.setRotation(getRotationalChannels(), aValue);
	}
	return nc;
}

void BoneForwardKinematics::getPoseFromLocal(Posture& pose) const
{
	pose.Init(m_skeleton->numRotJoint(), m_skeleton->numTransJoint());

	// update root position and rotations
	for(int ijoint=0, nj=m_skeleton->numRotJoint(); ijoint<nj; ijoint++)
	{
		// update rotations
		int target=m_skeleton->getTreeIndexByRotJointIndex(ijoint);
		pose.m_aRotations[ijoint]=m_local[target].rotation;
	}

	for(int ijoint=0, nj=m_skeleton->numTransJoint(); ijoint<nj; ijoint++)
	{
		// update translations
		int target=m_skeleton->getTreeIndexByTransJointIndex(ijoint);
		pose.m_aTranslations[ijoint]=m_local[target].translation;
	}
}

void BoneForwardKinematics::getPoseFromGlobal(Posture& pose) const
{
	pose.Init(m_skeleton->numRotJoint(), m_skeleton->numTransJoint());

	// update root position and rotations
	for(int ijoint=0, nj=m_skeleton->numRotJoint(); ijoint<nj; ijoint++)
	{
		// update rotations
        Bone& target=m_skeleton->getBoneByRotJointIndex(ijoint);
		pose.m_aRotations[ijoint]=global(*target.parent()).toLocalRot(global(target).rotation);
	}
	for(int ijoint=0, nj=m_skeleton->numTransJoint(); ijoint<nj; ijoint++)
	{
		// update translations
        Bone& target=m_skeleton->getBoneByTransJointIndex(ijoint);
		pose.m_aTranslations[ijoint]=global(*target.parent()).toLocalPos(global(target).translation);
	}
}
void BoneForwardKinematics::updateBoneLength(MotionLoader const& loader)
{
	for(int i=2; i<loader.numBone()-1 ; i++)
		m_local[i].translation=loader.bone(i).getOffsetTransform().translation;

	forwardKinematics();
}

void BoneForwardKinematics::getPoseDOFfromGlobal(vectorn& poseDOF) const
{
	Posture p;
	getPoseFromGlobal(p);
	m_skeleton->dofInfo.getDOF(p, poseDOF);
}
void BoneForwardKinematics::getPoseDOFfromLocal(vectorn& poseDOF) const
{
	Posture p;
	getPoseFromLocal(p);
	m_skeleton->dofInfo.getDOF(p, poseDOF);
}
void BoneForwardKinematics::inverseKinematics()
{
	Posture pose;
	getPoseFromGlobal(pose);
	setPose(pose);
}

void BoneForwardKinematics::setChain(const Posture& pose, const Bone & bone)
{
	// update rotate chains
	vector3 offset;
	const Bone* pBone=&bone;
	while(pBone->child())
		pBone=pBone->child();
	do
	{
		if(pBone->rotJointIndex()!=-1)
			_local(*pBone).rotation=pose.m_aRotations[pBone->rotJointIndex()];

		if(pBone->transJointIndex()!=-1)
			_local(*pBone).translation=pose.m_aTranslations[pBone->transJointIndex()];

		pBone=pBone->parent();
	}
	while(pBone);

	setChain(bone);
}

void BoneForwardKinematics::setChain(const Bone & bone)
{

	int chain[m_local.size()]; // stack
	int chain_size=0;

	const Bone* ibone=&bone;
	while(ibone->child())
		ibone=ibone->child();

	while(1)
	{
		chain[chain_size++]=ibone->treeIndex();
		Msg::verify(chain_size<20,"bone_FK stack overflow");

		Bone* parent=ibone->parent();
		if(!parent) break;
		ibone=parent;
	}


	ASSERT(&m_skeleton->bone(chain[chain_size-1])==(Bone*)m_skeleton->m_pTreeRoot);	// dummy node
	ASSERT(&m_skeleton->bone(chain[chain_size-2])==((Bone*)m_skeleton->m_pTreeRoot)->child());

	_global(chain[chain_size-2])=local(chain[chain_size-2]);
	for(int i=chain_size-3; i>=0; i--)
		_global(chain[i]).mult(global(chain[i+1]), local(chain[i]));
}

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
Bone::Bone()
: Node()
{
	NodeType=BONE;
	m_transfOrig.identity();
	m_rotJointIndex=-1;
	m_transJointIndex=-1;
	m_voca=-1;
	m_skeleton=NULL;
}

vector3 Bone::axis(int ichannel)
{
	switch(getRotationalChannels() [ichannel])
	{
	case 'X':
		return vector3(1,0,0);
	case 'Y':
		return vector3(0,1,0);
	}
	return vector3(0,0,1);
}


Bone::~Bone()
{
}

TString Bone::name() const
{
	return TString(NameId);
}

bool Bone::isDescendent(const Bone * parent) const
{
	const Bone * child=this;
   while (child!=NULL)
   {
      if (child==parent )
		return true;
	  else
		child=child->parent();		
   }
   return false;
}
void Bone::pack(BinaryFile& bf, int version) const
{
	Node::pack(bf, version);

	bf.pack(m_transChannels);
	bf.pack(m_rotChannels);

	bf.pack(matrix4(m_transfOrig));
	bf.pack(matrix4());		//!< translation후 rotation하는 matrix, MotionLoader::SetPose에서 사용된다.
	bf.pack(matrix4());	//!< MotionLoader::SetPose에서 사용된다. global rotation하고 translation이 저장되어 있다.
}

void Bone::setChannels(const char* tx, const char* rx)
{
	m_transChannels=tx;
	m_rotChannels=rx;
}

void Bone::setArbitraryAxes(vector3* axisValue)
{
	m_rotAxes = axisValue;
}

vector3 Bone::getArbitraryAxis(int i)
{
	return m_rotAxes[i];
}



TString const& Bone::getTranslationalChannels() const
{
	return m_transChannels;
}


TString const& Bone::getRotationalChannels() const
{
	return m_rotChannels;
}

#include "BVHLoader.h"	// for ChannelParser.
void Bone::unpack(BinaryFile& bf, int version)
{
	Node::unpack(bf, version);

	if(version<=4)
	{
		int unused=bf.unpackInt();
		ChannelParser c;
		c.m_numChannel=bf.unpackInt();
		c.m_aeChannels=new int[c.m_numChannel];
		for(int i=0; i<c.m_numChannel; i++)
			c.m_aeChannels[i]=bf.unpackInt();

		setChannels(c.makeTransChannels(), c.makeRotChannels());
	}
	else
	{
		bf.unpack(m_transChannels);
		bf.unpack(m_rotChannels);
	}

	matrix4 _matRotOrig, _unused;
	bf.unpack(_matRotOrig);
	bf.unpack(_unused);		//!< translation후 rotation하는 matrix, MotionLoader::SetPose에서 사용된다.
	bf.unpack(_unused);	//!< MotionLoader::SetPose에서 사용된다. global rotation하고 translation이 저장
	m_transfOrig=_matRotOrig;
}

void printNewline(FILE* file, int level)
{
	fprintf(file, "\n");
	for(int i=0; i<level; i++)
		fprintf(file, "  ");
}

void Bone::packBVH(FILE* file, int level, MotionLoader* pLoader)
{
	if(level==0)
		fprintf(file, "HIERARCHY\nROOT ");
	else
	{
		printNewline(file, level);
		if(!m_pChildHead)
			fprintf(file, "End ");
		else
			fprintf(file, "JOINT ");
	}

/*
	TString NameId;
	if(!m_pChildHead)
		NameId="Site";
	else
	{
		int jointIndex=pLoader->getVocaByRotJointIndex(pLoader->getRotJointIndexByTreeIndex(pLoader->GetIndex(this)));
		if(jointIndex==-1)
			NameId="?";
		else
			NameId=pLoader->m_translationTable[jointIndex];
	}
*/
	if(!m_pChildHead)
		fprintf(file, "Site");
	else
		fprintf(file, "%s", NameId);

	printNewline(file, level);

	vector3 offset;
	getOffset(offset);
	fprintf(file, "{");
	printNewline(file, level+1);
	fprintf(file, "OFFSET %f %f %f", offset.x, offset.y, offset.z);

	if(!m_pChildHead)
	{
		printNewline(file, level);
		fprintf(file, "}");
	}
	else
	{
		printNewline(file, level+1);


		const bool bFullDOF=false;



		if(bFullDOF)
		{
			TString rc, tc;
			tc=getTranslationalChannels();
			rc=getRotationalChannels();

			int numChannels=tc.length()?3:0;
			numChannels+=rc.length()?3:0;
			fprintf(file, "CHANNELS %d ", numChannels);

			if(tc.length())
			{
				for(int i=0; i<tc.length(); i++)
				{
					switch(tc[i])
					{
					case 'X':
						fprintf(file, "Xposition ");
						break;
					case 'Y':
						fprintf(file, "Yposition ");
						break;
					case 'Z':
						fprintf(file, "Zposition ");
						break;
					}
				}
			}

			if(rc.length())
			{
				char aChannel[3];
				for(int i=0; i<rc.length(); i++)
					aChannel[i]=rc[i];

				//길이가 3보다 작으면 나머지 채널을 채워준다.
				for(int i=rc.length(); i<3; i++)
				{
					aChannel[i]='X';

					for(int k=0; k<2; k++)
					{
						for(int j=0; j<i; j++)
							if(aChannel[i]==aChannel[j]) aChannel[i]++;
					}
				}

				for(int i=0; i<3; i++)
				{
					switch(aChannel[i])
					{
						case 'Z':
						fprintf(file, "Zrotation ");
						break;
						case 'X':
						fprintf(file, "Xrotation ");
						break;
						case 'Y':
						fprintf(file, "Yrotation ");
						break;
					}
				}
			}
		}
		else
		{
			TString rc, tc;
			tc=getTranslationalChannels();
			rc=getRotationalChannels();

			int numChannels=tc.length()+rc.length();
			fprintf(file, "CHANNELS %d ", numChannels);

			if(tc.length())
			{
				for(int i=0; i<tc.length(); i++)
				{
					switch(tc[i])
					{
					case 'X':
						fprintf(file, "Xposition ");
						break;
					case 'Y':
						fprintf(file, "Yposition ");
						break;
					case 'Z':
						fprintf(file, "Zposition ");
						break;
					}
				}
			}

			if(rc.length())
			{
				for(int i=0; i<rc.length(); i++)
				{
					switch(rc[i])
					{
						case 'Z':
						fprintf(file, "Zrotation ");
						break;
						case 'X':
						fprintf(file, "Xrotation ");
						break;
						case 'Y':
						fprintf(file, "Yrotation ");
						break;
					}
				}
			}
		}

		for(Bone* node=(Bone*)m_pChildHead; node!=NULL; node=(Bone*)(node->m_pSibling))
		{
			node->packBVH(file, level+1, pLoader);
		}

		printNewline(file, level);
		fprintf(file, "}");
	}
}



m_real Bone::length() const
{
	return m_transfOrig.translation.length();
}

void Bone::getOffset(vector3& trans) const
{
	// mat rot original의 translation term만 뽑는다.
	trans=m_transfOrig.translation;
}

transf const& Bone::getOffsetTransform() const
{
	return m_transfOrig;
}

transf & Bone::_getOffsetTransform()
{
	return m_transfOrig;
}

void Bone::getTranslation(vector3& trans) const
{
	// mat combined의 translation term만 뽑는다.
	trans=m_skeleton->m_defaultFK.global(treeIndex()).translation;
}

vector3 const& Bone::getTranslation() const
{
	return m_skeleton->m_defaultFK.global(treeIndex()).translation;
}

void Bone::getRotation(quater& rot) const
{
	rot=m_skeleton->m_defaultFK.global(treeIndex()).rotation;
}

quater const& Bone::getRotation() const
{
	return m_skeleton->m_defaultFK.global(treeIndex()).rotation;
}

transf const& Bone::getFrame() const
{
	return m_skeleton->m_defaultFK.global(treeIndex());
}

transf & Bone::_getFrame() const
{
	return m_skeleton->m_defaultFK._global(treeIndex());
}

transf const& Bone::getLocalFrame() const
{
	return m_skeleton->m_defaultFK.local(treeIndex());
}

transf & Bone::_getLocalFrame() const
{
	return m_skeleton->m_defaultFK._local(treeIndex());
}

void MotionLoader::_updateTreeIndex()
{
	int numChannel;
	MakeBoneArrayFromTree(numChannel);
}

void MotionLoader::setCurPoseAsInitialPose()
{
	exportSkeleton("__temp.skl");
	MotionLoader backup("__temp.skl");
	// The above lines should become backup=*this later.

	Posture curpose;
	m_defaultFK.getPoseFromGlobal(curpose);	// assuming that at least global rotations are valid.

	m_defaultFK.setPose(curpose);	// to make sure all of the global and local transformations are valid.

	// Now, make curpose as initial pose (=identity rotations.)
	backup.setPose(curpose);

	for(int i=1; i<numBone(); i++)
	{
		bone(i)._getOffsetTransform().rotation.identity();
		bone(i)._getOffsetTransform().translation.difference(bone(i).parent()->getTranslation(), bone(i).getTranslation());
	}
	UpdateInitialBone();

	// Transfer motion data.
	PoseTransfer pt(&backup, this, NULL, true);

	for(int i=0; i<m_cPostureIP.numFrames(); i++)
	{
		// read pose
		pt.setTargetSkeleton(m_cPostureIP.pose(i));

		// overwrite pose
		getPose(m_cPostureIP.pose(i));
	}
}




void _initTranslationTable(utility::NameTable &m_translationTable)
{
	// 순서 중요. header와 같은 순으로 할 것.
	m_translationTable.Insert("HIPS");
	m_translationTable.Insert("LEFTHIP");
	m_translationTable.Insert("LEFTKNEE");
	m_translationTable.Insert("LEFTANKLE");
	m_translationTable.Insert("LEFTTOES");
	m_translationTable.Insert("RIGHTHIP");
	m_translationTable.Insert("RIGHTKNEE");
	m_translationTable.Insert("RIGHTANKLE");
	m_translationTable.Insert("RIGHTTOES");
	m_translationTable.Insert("CHEST");
	m_translationTable.Insert("CHEST2");
	m_translationTable.Insert("LEFTCOLLAR");
	m_translationTable.Insert("LEFTSHOULDER");
	m_translationTable.Insert("LEFTELBOW");
	m_translationTable.Insert("LEFTWRIST");
	m_translationTable.Insert("RIGHTCOLLAR");
	m_translationTable.Insert("RIGHTSHOULDER");
	m_translationTable.Insert("RIGHTELBOW");
	m_translationTable.Insert("RIGHTWRIST");
	m_translationTable.Insert("NECK");
	m_translationTable.Insert("HEAD");
	m_translationTable.Insert("LThWrist");
	m_translationTable.Insert("LThMetac");
	m_translationTable.Insert("LThIntra1");
	m_translationTable.Insert("LF1Metac");
	m_translationTable.Insert("LF1Intra1");
	m_translationTable.Insert("LF1Intra2");
	m_translationTable.Insert("LF2Metac");
	m_translationTable.Insert("LF2Intra1");
	m_translationTable.Insert("LF2Intra2");
	m_translationTable.Insert("LF3Metac");
	m_translationTable.Insert("LF3Intra1");
	m_translationTable.Insert("LF3Intra2");
	m_translationTable.Insert("LF4Metac");
	m_translationTable.Insert("LF4Intra1");
	m_translationTable.Insert("LF4Intra2");
	// left fingers (see retarget_common.lua)
	m_translationTable.Insert("RThWrist");
	m_translationTable.Insert("RThMetac");
	m_translationTable.Insert("RThIntra1");
	m_translationTable.Insert("RF1Metac");
	m_translationTable.Insert("RF1Intra1");
	m_translationTable.Insert("RF1Intra2");
	m_translationTable.Insert("RF2Metac");
	m_translationTable.Insert("RF2Intra1");
	m_translationTable.Insert("RF2Intra2");
	m_translationTable.Insert("RF3Metac");
	m_translationTable.Insert("RF3Intra1");
	m_translationTable.Insert("RF3Intra2");
	m_translationTable.Insert("RF4Metac");
	m_translationTable.Insert("RF4Intra1");
	m_translationTable.Insert("RF4Intra2");
	ASSERT(m_translationTable.Size()==MotionLoader::NUM_JOINT_VOCA);
}
MotionLoader::MotionLoader()
:ModelLoader()
,m_defaultFK(this)
{
	_initTranslationTable(m_translationTable);
	m_pFactory=new TDefaultFactory<Posture>();
	dofInfo._sharedinfo	=NULL;
}

MotionLoader::MotionLoader(const char* filename, const char* option)
:ModelLoader()
,m_defaultFK(this)
{
	_initTranslationTable(m_translationTable);
	m_pFactory=new TDefaultFactory<Posture>();

	BinaryFile bf(false, filename);

	int version=unpack(bf);


	int type=bf.unpackInt();

	if(type==POSTUREIP)
	{
		if(!option || TString("loadSkeletonOnly")!=option)
		{
			m_cPostureIP.InitSkeleton(this);
			m_cPostureIP._unpack(bf, version);
		}
	}

	bf.close();
}

void MotionLoader::changeFactory(TFactory<Posture>* pF)
{
	delete m_pFactory;
	m_pFactory=pF;
}

const TFactory<Posture>* MotionLoader::factory() const
{
	return m_pFactory;
}

void MotionLoader::exportSkeleton(const char* filename) const
{
	BinaryFile bf(true, filename);

	pack(bf, MOT_RECENT_VERSION);
	bf.packInt(-1);
	bf.close();
}

void MotionLoader::loadAnimation(Motion& mot, const char* filename) const
{
	TString fn=filename;
	if(fn.right(4).toUpper()==".MOT")
	{
		BinaryFile bf(false, filename);

		MotionLoader temp;
		int version=temp.unpack(bf);

		int type=bf.unpackInt();

		ASSERT(type==POSTUREIP);
		ASSERT(temp.numBone()==numBone());
		ASSERT(temp.numRotJoint()==numRotJoint());
		ASSERT(temp.numTransJoint()==numTransJoint());

		mot.InitSkeleton((MotionLoader*)this);
		mot._unpack(bf, version);

		bf.close();
	}
	else if(fn.right(4).toUpper()==".AMC")
	{
		Msg::error("this is not an instance of ASFLoader");
	}
	else if(fn.right(4).toUpper()==".BVH")
	{
		Msg::error("this is not an instance of BVHLoader");
	}
	else	// ANIM
	{
		Msg::verify(fn.right(4).toUpper()=="ANIM","unknown file format %s (need to rename it to .ANIM?)", fn.ptr());

		BinaryFile bf(false, filename);
		int version=bf.unpackInt();
		int type=bf.unpackInt();
		Msg::verify(type==POSTUREIP, "loadAnimationError");

		mot.InitSkeleton((MotionLoader*)this);
		mot._unpack(bf, version);
		bf.close();
	}
}


MotionLoader::~MotionLoader()
{
	delete dofInfo._sharedinfo;
	dofInfo._sharedinfo=NULL;
}

int MotionLoader::getVocaByTreeIndex(int treeIndex) const
{
	for(int i=0; i<NUM_JOINT_VOCA; i++)
	{
		if(treeIndex==m_aVoca2TreeIndex[i])
			return i;
	}
	return -1;
}

int MotionLoader::getVocaByRotJointIndex(int jointIndex) const
{
	int treeIndex=getTreeIndexByRotJointIndex(jointIndex);
	return getVocaByTreeIndex(treeIndex);
}

int MotionLoader::getRotJointIndexByTreeIndex(int treeIndex) const
{
	//return m_aTree2RotJointIndex[treeIndex];
	return getBoneByTreeIndex(treeIndex).rotJointIndex();
}

int MotionLoader::getTransJointIndexByTreeIndex(int treeIndex) const
{
	//return m_aTree2TransJointIndex[treeIndex];
	return getBoneByTreeIndex(treeIndex).transJointIndex();
}
void MotionLoader::UpdateBone()
{
	m_defaultFK.forwardKinematics();
}



void MotionLoader::UpdateInitialBone()
{
	m_defaultFK.init();
	for(int i=0; i<numBone(); i++)	// set Bone::m_skeleton so that Bone::getFrame() can be used.
		bone(i).m_skeleton=this;
}


void MotionLoader::_getTree2RotJointIndexArray(intvectorn& aTree2RotJointIndex) const
{
	aTree2RotJointIndex.resize(GetNumTreeNode());
	for(int i=0; i<GetNumTreeNode(); i++)
		aTree2RotJointIndex[i]=getBoneByTreeIndex(i).m_rotJointIndex;
}

void MotionLoader::_getTree2TransJointIndexArray(intvectorn& aTree2TransJointIndex) const
{
	aTree2TransJointIndex.resize(GetNumTreeNode());
	for(int i=0; i<GetNumTreeNode(); i++)
		aTree2TransJointIndex[i]=getBoneByTreeIndex(i).m_transJointIndex;
}

void MotionLoader::_setTree2RotJointIndexArray(intvectorn& aTree2RotJointIndex)
{
	for(int i=0; i<GetNumTreeNode(); i++)
		getBoneByTreeIndex(i).m_rotJointIndex=aTree2RotJointIndex[i];
}

void MotionLoader::_setTree2TransJointIndexArray(intvectorn& aTree2TransJointIndex)
{
	for(int i=0; i<GetNumTreeNode(); i++)
		getBoneByTreeIndex(i).m_transJointIndex=aTree2TransJointIndex[i];
}

int Bone::numChannels() const
{
	return getRotationalChannels().length()+getTranslationalChannels().length();
}

void MotionLoader::MakeBoneArrayFromTree(int& numChannel)
{
	m_nNumTreeNode=CountTreeNode();

	m_TreeStack.Initiate();
	m_apNode.resize(m_nNumTreeNode);
	m_aRotJoint2TreeIndex.setSize(m_nNumTreeNode);	// NumRotJoint()만큼의 array면 되지만, 아직 개수를 모르는 만큼, upper bound로 넉넉하게 준다.
	m_aTransJoint2TreeIndex.setSize(m_nNumTreeNode);  // NumTransJoint()만큼의 array면 되지만, 아직 개수를 모르는 만큼, upper bound로 넉넉하게 준다.

	intvectorn aTree2RotJointIndex;
	intvectorn aTree2TransJointIndex;
	aTree2RotJointIndex.setSize(m_nNumTreeNode);
	aTree2RotJointIndex.setAllValue(-1);
	aTree2TransJointIndex.setSize(m_nNumTreeNode);
	aTree2TransJointIndex.setAllValue(-1);
	Node *src=m_pTreeRoot;
	int inode=-1;
	numChannel=0;
	m_nNumEndNode=0;
	m_nNumRotNode=0;
	m_nNumTransNode=0;

	while(TRUE)
	{
		for(;src;src=src->m_pChildHead)
		{
			m_TreeStack.Push(src);
			// Do somthing for node
			inode++;
			Bone* pBone;
			m_apNode[inode]=src;
			src->m_nIndex=inode;

			pBone=(Bone*)src;
			pBone->m_skeleton=this;
			if(pBone->numChannels())
			{
				TString trans=pBone->getTranslationalChannels();
				TString rot=pBone->getRotationalChannels();

				if(trans.length())
				{
					m_aTransJoint2TreeIndex[m_nNumTransNode]=inode;
					aTree2TransJointIndex[inode]=m_nNumTransNode;
					m_nNumTransNode++;
				}

				if(rot.length())
				{
					m_aRotJoint2TreeIndex[m_nNumRotNode]=inode;
					aTree2RotJointIndex[inode]=m_nNumRotNode;
					m_nNumRotNode++;
				}

				numChannel+=pBone->numChannels();
			}
			else
				m_nNumEndNode++;
		}
		m_TreeStack.Pop(&src);
		if(!src) break;
		src=src->m_pSibling;
	}
	//ASSERT(irotjoint==numRotJoint());
	m_aRotJoint2TreeIndex.resize(numRotJoint());

	_setTree2RotJointIndexArray(aTree2RotJointIndex);
	_setTree2TransJointIndexArray(aTree2TransJointIndex);

	_updateVoca2TreeIndex();

	UpdateInitialBone();
	// make parent index array
#ifdef _DEBUG
	Node* parent;
	for(int i=0; i<GetNumTreeNode(); i++)
	{
		parent=GetNode(i);
		for(Node* child=GetFirstChild(parent); child!=NULL; child=GetNextSibling(child))
			ASSERT(child->m_pParent==parent);
	}
#endif
}

int dep_GetParentJoint(MotionLoader const& ml, int rotjointIndex)
{
	int parentIndex=ml.getBoneByRotJointIndex(rotjointIndex).m_pParent->GetIndex();
	while(parentIndex>=0)
	{
		int pj=ml.getRotJointIndexByTreeIndex(parentIndex);
		if(pj!=-1) return pj;
		Node* parent=ml.getBoneByTreeIndex(parentIndex).m_pParent;
		if(!parent) return -1;
		parentIndex=parent->GetIndex();
	}

	return -1;
}

int MotionLoader::getTreeIndexByName(const char* name) const
{
	return GetIndex(name);
}

Bone& MotionLoader::getBoneByName(const char* name) const
{
	if(!name) Msg::error("getBoneByName %s", name);
	int index=GetIndex(name);
	if(index==-1) Msg::error("getBoneByName %s", name);
	return *((Bone*)GetNode(index));
}


int MotionLoader::getTransJointIndexByName(const char* nameID)
{
	int index=GetIndex(nameID);

	if(index==-1) return -1;
	return getBoneByTreeIndex(index).transJointIndex();
}

int MotionLoader::getRotJointIndexByName(const char* nameid) const
{
	int index=GetIndex(nameid);

	if(index==-1) return -1;
	return getBoneByTreeIndex(index).rotJointIndex();
}

int MotionLoader::numRotJoint() const	//!< KeyFrame되어 있는 조인트의 개수를 구한다.
{
	return m_nNumRotNode;
}

int MotionLoader::numTransJoint() const	//!< KeyFrame되어 있는 조인트의 개수를 구한다.
{
	return m_nNumTransNode;
}

//joint 개수 + end노드 포함
int MotionLoader::numBone() const
{
	//joint 개수 end노드 포함, Dummy 도 포함.
	return m_nNumTreeNode;
}

void MotionLoader::createDummyRootBone()
{
	m_pTreeRoot=new Bone();
	m_pTreeRoot->SetNameId("HumanoidBody");
	int numChannel;
	MakeBoneArrayFromTree(numChannel);
}
void MotionLoader::insertSiteBones()
{

	while(true){
		int count=0;		
		for (int i=1; i<numBone(); i++){
			Bone& b=bone(i);
			if ( b.child()==NULL && b.numChannels()!=0){
				printf("adding %s's children: SITE\n", b.name().ptr());
				insertChildBone(b, "SITE");
				count++;
				break;
			}
		}
		if(count==0) break;
	}
}
void MotionLoader::_initDOFinfo()
{
	if(!dofInfo._sharedinfo)
		dofInfo._sharedinfo=new MotionDOFinfo::SharedInfo;
	dofInfo._sharedinfo->init(*this);
}
/// child의 transform은 identity가 되고, parent로 옮겨진다.
void MotionLoader::insertChildBone(Bone& parent, const char* nameId, bool bMoveChildren)
{
	Bone* newChild=new Bone();

	newChild->SetNameId(nameId);

	// initialize matrix
	newChild->m_transfOrig.identity();

	std::list<Node*> children;
	if(bMoveChildren)
		parent.detachAllChildren(children);

	parent.AddChild(newChild);

	if(bMoveChildren)
		newChild->addChildren(children);

	int numChannel;
	MakeBoneArrayFromTree(numChannel);
}

static void _getBoneByRotJoint(MotionLoader const& skel, std::vector<Bone*>& _rotJoint2bone)
{
	_rotJoint2bone.resize(skel.numRotJoint());

	for(int i=0; i<skel.numRotJoint(); i++)
		_rotJoint2bone[i]=&skel.getBoneByRotJointIndex(i);
}


static void _getBoneByTransJoint(MotionLoader const& skel, std::vector<Bone*>& _TransJoint2bone)
{
	_TransJoint2bone.resize(skel.numTransJoint());

	for(int i=0; i<skel.numTransJoint(); i++)
		_TransJoint2bone[i]=&skel.getBoneByTransJointIndex(i);
}

void MotionLoader::removeBone(Bone& target)
{
	Msg::verify(target.getRotationalChannels()==0,"removeBone1");
	Msg::verify(target.getTranslationalChannels()==0,"removeBone2");
	Msg::verify(target.parent(),"removeBone3");

	Bone& parent=*target.parent();

	std::list<Node*> children;
	target.detachAllChildren(children);

	std::vector<Bone*> _rotJoint2bone;
	std::vector<Bone*> _transJoint2bone;

	_getBoneByRotJoint(*this, _rotJoint2bone);
	_getBoneByTransJoint(*this, _transJoint2bone);


	// ta*ra*tb*rb*tc*rc
	// 
	// assuming bone:a is removed

	// rb' -> ra*rb
	// tb' =?

	// tb'*rb'= ta*ra*tb*rb
	// tb'  = ta*ra*tb*rb*(ra* rb)'
	//		= ta* ra * tb * ra'

	// (I tb') (X)  = (I ta) (ra 0) (I tb) (ra' 0)
	// (0  1 ) (1)    (0  1) (0  1) (0  1) (0   1)

	//              = (I ta) (ra 0) (ra' tb)
	//                (0  1) (0  1) (0    1)

	//              =  ...    ( I  ra*tb)
	//                        ( 0    1)

	//              = (I  ra*tb+ta)
	//
	   
	for(std::list<Node*> ::iterator i=children.begin(); i!=children.end(); ++i)
	{
		Bone*bone=(Bone*)(*i);

		bone->_getOffsetTransform().leftMult(target.getOffsetTransform());

		if(bone->rotJointIndex()!=-1)
		{
			for(int j=0; j<m_cPostureIP.numFrames(); j++)
				m_cPostureIP.pose(j).m_aRotations[bone->rotJointIndex()].leftMult(target.getOffsetTransform().rotation);
		}

		if(bone->transJointIndex()!=-1)
		{
			for(int j=0; j<m_cPostureIP.numFrames(); j++)
			{
				vector3& tb=m_cPostureIP.pose(j).m_aTranslations[bone->transJointIndex()];
				vector3 ta=target.getOffsetTransform().translation;
				quater ra=target.getOffsetTransform().rotation;
				tb.rotate(ra);
				tb+=ta;
			}
		}
	}

	parent.RemoveChild(&target);
	parent.addChildren(children);
	int numChannel;
	MakeBoneArrayFromTree(numChannel);

	bool bNeedToRearrange=false;
	for(int i=0; i<_rotJoint2bone.size(); i++)
	{
		if(_rotJoint2bone[i]!=&getBoneByRotJointIndex(i))
		{
			bNeedToRearrange=true;
			break;
		}
	}

	for(int i=0; i<_transJoint2bone.size(); i++)
	{
		if(_transJoint2bone[i]!=&getBoneByTransJointIndex(i))
		{
			bNeedToRearrange=true;
			break;
		}
	}

	if(bNeedToRearrange)
	{
		Posture temp;
		for(int i=0; i<_rotJoint2bone.size(); i++)
		{
			for(int j=0; j<m_cPostureIP.numFrames(); j++)
			{
				temp=m_cPostureIP.pose(j);
				for(int k=0; k<numRotJoint(); k++)
				{
					m_cPostureIP.pose(j).m_aRotations[_rotJoint2bone[k]->rotJointIndex()]=
						temp.m_aRotations[k];
				}

				for(int k=0; k<numTransJoint(); k++)
				{
					m_cPostureIP.pose(j).m_aTranslations[_transJoint2bone[k]->transJointIndex()]=
						temp.m_aTranslations[k];
				}
			}
		}
	}
}


void MotionLoader::removeAllRedundantBones()
{
	bool bChanged;
	do
	{
		bChanged=false;
		for(int i=2; i<numBone() ; i++)
		{
			Bone& target=bone(i);
			if(target.getRotationalChannels()==0
				&& target.getTranslationalChannels()==0
				&& target.numChildren())
			{
				printf("Removing redundant bone %s\n", target.NameId);
				removeBone(target);
				bChanged=true;
				break;
			}
		}
	}
	while(bChanged);
}

/// type="T" (translational) or "R" (rotational) or "RT" (both)
void MotionLoader::insertJoint(Bone& target, const char* type, std::vector<Motion*> dependentMotions)
{
		if(type[0]=='R')
	{
		int nchannel=3;
		if(type[1]=='T')
		{
			nchannel+=3;
			if(target.numChannels()!=0)
			{
				Msg::print("Bone %s already has keys\n", target.NameId);
				return;
			}
		}

		if(target.getRotationalChannels().length())
		{
			Msg::print("Bone %s already has rotational keys\n", target.NameId);
			return;
		}

		///////////////////////
		// make channels
		if(nchannel==6)
			target.setChannels("XYZ", "ZXY");
		else
			target.setChannels("","ZXY");

		// Backup indexes
		intvectorn prevTree2RotJointIndex;
		intvectorn prevTree2TransJointIndex;
		_getTree2RotJointIndexArray(prevTree2RotJointIndex);
		_getTree2TransJointIndexArray(prevTree2TransJointIndex);;

		// Update indexes
		int numChannel;
		MakeBoneArrayFromTree(numChannel);

		intvectorn newTree2RotJointIndex;
		intvectorn newTree2TransJointIndex;
		_getTree2RotJointIndexArray(newTree2RotJointIndex);
		_getTree2TransJointIndexArray(newTree2TransJointIndex);

		// index의 변화를 보고, dependentMotions를 고쳐준다.
		if(dependentMotions.size()==0 || dependentMotions[0]->numFrames()==0)
			return;
		
		// rotational joint
		if(!(prevTree2RotJointIndex==newTree2RotJointIndex))
		{
			for(int i=0; i<prevTree2RotJointIndex.size(); i++)
			{
				if(prevTree2RotJointIndex[i]!=-1)
				{
					int prevJoint=prevTree2RotJointIndex[i];
					int newjoint=newTree2RotJointIndex[i];

					if(prevJoint==newjoint) continue;
					assert(newjoint==prevJoint+1);

					// prevJoint이후가 prevJoint+1로 옮겨지면 된다.
					quater key;
					key=target.m_transfOrig.rotation;

					int prevNumRotJoint=dependentMotions[0]->pose(0).numRotJoint();

					for(int i=0; i<dependentMotions.size(); i++)
					{
						Motion& mot=*dependentMotions[i];
						for(int i=0; i<mot.numFrames(); i++)
						{
							mot.pose(i).m_aRotations.resize(prevNumRotJoint+1);
							for(int j=prevNumRotJoint-1; j>=prevJoint; j--)
								mot.pose(i).m_aRotations[j+1]=mot.pose(i).m_aRotations[j];
							mot.pose(i).m_aRotations[prevJoint]=key;
						}
					}
					break;
				}
			}
		}

		// translational joint
		if(!(prevTree2TransJointIndex==newTree2TransJointIndex))
		{
			vector3 key;
			key=target.m_transfOrig.translation;

			Posture temp;

			int prevNumTransJoint=dependentMotions[0]->pose(0).numTransJoint();
			for(int imot=0; imot<dependentMotions.size(); imot++)
			{
				Motion& mot=*dependentMotions[imot];
				for(int i=0; i<mot.numFrames(); i++)
				{
					temp=mot.pose(i);

					mot.pose(i).m_aTranslations.resize(prevNumTransJoint+1);

					for(int j=0; j<prevNumTransJoint+1; j++)
					{
						int treeIndex=getTreeIndexByTransJointIndex(j);

						if(prevTree2TransJointIndex[treeIndex]==-1)
							mot.pose(i).m_aTranslations[j]=key;
						else
							mot.pose(i).m_aTranslations[j]=temp.m_aTranslations[prevTree2TransJointIndex[treeIndex]];
					}
				}
			}
		}
	}
}

void MotionLoader::setNewRoot(Bone& newRoot)
{

	int ti=newRoot.transJointIndex();
	int ri=newRoot.rotJointIndex();

	int nf=m_cPostureIP.numFrames();
	quaterN newRootR(nf);
	vector3N newRootT(nf);
	for(int i=0; i<nf; i++)
	{
		setChain(m_cPostureIP.pose(i), newRoot);
		newRootR(i)=newRoot.getFrame().rotation;
		newRootT(i)=newRoot.getFrame().translation;
	}

	if(ti==-1 || ri==-1)
	{
		insertJoint(newRoot, "RT");
		ti=newRoot.transJointIndex();
		ri=newRoot.rotJointIndex();
	}

	Msg::verify(ti!=-1, "setNewRoot1");
	Msg::verify(ri!=-1, "setNewRoot2");

	std::list<Node*> children;
		newRoot.detachAllChildren(children);

	std::string name=newRoot.NameId;
	TString rc=newRoot.getRotationalChannels();
	TString tc=newRoot.getTranslationalChannels();

	Bone& parent=*newRoot.parent();
	//parent.RemoveChild(&newRoot);

	Bone& pparent=*parent.parent();
	pparent.RemoveChild(&parent);
	insertChildBone(pparent, name.c_str());
	bone(1).setChannels(tc.ptr(), rc.ptr());
	ASSERT(numBone()==3);
	//printf("children %d\n", children.size());
	bone(1).addChildren(children);
	
	// Update indexes
	int numChannel;
	MakeBoneArrayFromTree(numChannel);
	_initDOFinfo();


	// now recover motion
	for(int i=0; i<m_cPostureIP.numFrames(); i++)
	{
		Posture& p=m_cPostureIP.pose(i);
		p.m_aTranslations(0)=newRootT(i);
		p.m_aRotations(0)=newRootR(i);

		for(int j=1; j<numTransJoint(); j++) 
			p.m_aTranslations(j)=
			p.m_aTranslations(j+ti);
		for(int j=1; j<numRotJoint(); j++) 
			p.m_aRotations(j)=
			p.m_aRotations(j+ri);

		p.m_aRotations.resize(numRotJoint());
		p.m_aTranslations.resize(numTransJoint());
	}
}
void MotionLoader::insertJoint(Bone& target, const char* type)
{
	std::vector<Motion*> dependentMotions(1);
	dependentMotions[0]=&m_cPostureIP;
	insertJoint(target, type, dependentMotions);
}

void MotionLoader::readJointIndex(const char* filename)
{
	ConfigTable jointVoca(filename);

	TString name;
	m_aVoca2TreeIndex.setSize(NUM_JOINT_VOCA);
	m_aVoca2TreeIndex.setAllValue(-1);
	for(int i=0; i<NUM_JOINT_VOCA; i++)
	{
		name=jointVoca.Find(m_translationTable[i]);
		if(name.length()==0)
		{
			Msg::print("warning! %s is not exist in %s\n", m_translationTable[i], filename);
			continue;
		}
		name.trimLeft(" ");
		name.trimRight("\r");
		name.trimRight(" ");
		Bone& bone=getBoneByName(name);
		bone.m_voca=i;
		m_aVoca2TreeIndex[i]=bone.treeIndex();
	}
}

void MotionLoader::_updateVoca2TreeIndex()
{
	// initialize only
	m_aVoca2TreeIndex.setSize(NUM_JOINT_VOCA);
	m_aVoca2TreeIndex.setAllValue(-1);
}

void MotionLoader::Scale(float fScale)
{
	scale(fScale, m_cPostureIP);
}

// almost private
void MotionLoader::_Scale(float fScale)
{
	m_TreeStack.Initiate();

	Node *src=m_pTreeRoot;
	int index=-1;

	while(TRUE)
	{
		for(;src;src=src->m_pChildHead)
		{
			m_TreeStack.Push(src);
			index++;
			// do something for src
			// src의 index는 현재 index이다.

			Bone *pBone=(Bone*)src;
			vector3 offset;
			pBone->getOffset(offset);
			pBone->m_transfOrig.translation=(offset*fScale);
		}
		m_TreeStack.Pop(&src);
		if(!src) break;
		src=src->m_pSibling;
	}

	/* do not do this here. this function only scales the bones.
	for(int i=0; i<m_cPostureIP.numFrames(); i++)
	{
		for(int j=0; j<m_cPostureIP.numTransJoints(); j++)
		{
			m_cPostureIP.pose(i).m_aTranslations[j]*=fScale;
		}
	}
	m_cPostureIP.CalcInterFrameDifference();
	*/
	UpdateInitialBone();
}

void MotionLoader::scale(float fScale, Motion& mot)
{
	_Scale(fScale);

	for(int i=0; i<mot.numFrames(); i++)
	{
		for(int j=0; j<mot.numTransJoints(); j++)
		{
			mot.pose(i).m_aTranslations[j]*=fScale;
		}
	}
	mot.CalcInterFrameDifference();
}


void MotionLoader::setPose(PoseWrap pose) const
{
	if(pose._src->hasMotionDOF())
		setPose(dofInfo.setDOF(pose._src->motdof().row(pose.iframe)));
	else
		setPose(pose._src->mot().pose(pose.iframe));
}
void MotionLoader::setPose(const Posture& pose) const
{
	m_defaultFK.setPose(pose);
}

void MotionLoader::setPoseDOF(const vectorn& poseDOF) const
{
	
	setPose(dofInfo.setDOF(poseDOF));
}

void MotionLoader::getPose(Posture& pose) const
{
	m_defaultFK.getPoseFromGlobal(pose);
}


void MotionLoader::getPoseDOF(vectorn& poseDOF) const
{
	m_defaultFK.getPoseDOFfromGlobal(poseDOF);
}


Bone& MotionLoader::getBoneByTreeIndex(int index)	const
{return *((Bone*)GetNode(index));}

Bone& MotionLoader::bone(int index) const
{return *((Bone*)GetNode(index));}

	Bone& MotionLoader::getBoneByRotJointIndex(int iRotJoint)	const			{ return *((Bone*)GetNode(getTreeIndexByRotJointIndex(iRotJoint)));}
	Bone& MotionLoader::getBoneByTransJointIndex(int iTransJoint)	const		{ return *((Bone*)GetNode(getTreeIndexByTransJointIndex(iTransJoint)));}
	Bone& MotionLoader::getBoneByVoca(int jointVoca)	const	{ return *((Bone*)GetNode(getTreeIndexByVoca(jointVoca)));}

void MotionLoader::setChain(const Posture& pose, Bone& bone) const
{
	m_defaultFK.setChain(pose, bone);
}

void MotionLoader::setChain(const Posture& pose, int iTargetJoint) const
{
	m_defaultFK.setChain(pose, getBoneByRotJointIndex(iTargetJoint));
}


Bone& dep_GetBoneFromCon(MotionLoader const& ml, int constraint)
{
	Bone* bone;
	bool bSite=true;
	switch(constraint)
	{
	case CONSTRAINT_LEFT_HEEL:
		bSite=false;
	case CONSTRAINT_LEFT_TOE:
	case CONSTRAINT_LEFT_FOOT:
		bone=&ml.getBoneByVoca(MotionLoader::LEFTANKLE);
		break;

	case CONSTRAINT_RIGHT_HEEL:
		bSite=false;
	case CONSTRAINT_RIGHT_TOE:
	case CONSTRAINT_RIGHT_FOOT:
		bone=&ml.getBoneByVoca(MotionLoader::RIGHTANKLE);
		break;

	case CONSTRAINT_LEFT_HAND:
		bSite=false;
	case CONSTRAINT_LEFT_FINGERTIP:
		bone=&ml.getBoneByVoca(MotionLoader::LEFTWRIST);
		break;

	case CONSTRAINT_RIGHT_HAND:
		bSite=false;
	case CONSTRAINT_RIGHT_FINGERTIP:
		bone=&ml.getBoneByVoca(MotionLoader::RIGHTWRIST);
		break;
	}

	if(bSite)
		return *bone->child();
	else
		return *bone;
}

Bone& dep_GetSiteBone(MotionLoader const& ml, int ijoint)
{
	ASSERT(ml.getBoneByRotJointIndex(ijoint).m_pChildHead);
	Node* curr;
	// go down to leaf
	for(curr=&ml.getBoneByRotJointIndex(ijoint); curr->m_pChildHead!=NULL; curr=curr->m_pChildHead);
	return *((Bone*)curr);
}
#include "ConstraintMarking.h"


void MotionLoader::pack(BinaryFile & bf, int version) const
{
	bf.packInt(version);// version 1.
	bf.packInt(m_nNumTreeNode);
	bf.pack(MOT_VERSION_STRING[version]);
	bf.packInt(m_nNumRotNode);
	bf.packInt(m_nNumTransNode);

	for(int i=0; i<GetNumTreeNode(); i++)
		GetNode(i)->pack(bf, version);

	bf.pack(m_aRotJoint2TreeIndex);	//!< m_aRotJoint2TreeIndex[joint index]=TreeNode index

	intvectorn aTree2RotJointIndex;
	_getTree2RotJointIndexArray(aTree2RotJointIndex);
	bf.pack(aTree2RotJointIndex);	//!< m_aTree2RotJointIndex[tree node index]=joint index

	bf.pack(m_aTransJoint2TreeIndex);

	intvectorn aTree2TransJointIndex;
	_getTree2TransJointIndexArray(aTree2TransJointIndex);
	bf.pack(aTree2TransJointIndex);	//!< m_aTree2TransJointIndex[tree node index]=joint index

	intvectorn aParentIndex;
	aParentIndex.setSize(GetNumTreeNode());
	aParentIndex[0]=-1;
	for(int i=1; i<GetNumTreeNode(); i++)
		aParentIndex[i]=getBoneByTreeIndex(i).m_pParent->GetIndex();

	bf.pack(aParentIndex);

	bf.pack(m_aVoca2TreeIndex);

	bf.packInt(m_nNumEndNode);	//!< skeleton의 EndNode에는 KeyFrame이 들어가지 않으므로 중요.
}

int MotionLoader::unpack(BinaryFile & bf)
{
	int version=bf.unpackInt();
	Msg::verify(version>=1, "version is too old");
	bf.unpackInt(m_nNumTreeNode);

	if(version>=3)
	{
		TString str;
		bf.unpack(str);
		bf.unpackInt(m_nNumRotNode);
		bf.unpackInt(m_nNumTransNode);
	}
	m_apNode.resize(m_nNumTreeNode);

	for(int i=0; i<GetNumTreeNode(); i++)
	{
		Msg::verify(bf.unpackInt()==BONE, "nodetype!=BONE");

		m_apNode[i]=new Bone();
		m_apNode[i]->m_nIndex=i;
		m_apNode[i]->unpack(bf, version);
	}

	bf.unpack(m_aRotJoint2TreeIndex);	//!< m_aRotJoint2TreeIndex[joint index]=TreeNode index

	intvectorn aTree2RotJointIndex;
	bf.unpack(aTree2RotJointIndex);	//!< m_aTree2RotJointIndex[tree node index]=joint index

	intvectorn aTree2TransJointIndex;
	if(version>=3)
	{
		bf.unpack(m_aTransJoint2TreeIndex);
		bf.unpack(aTree2TransJointIndex);
	}
	else
	{
		m_aTransJoint2TreeIndex.setSize(1);
		m_aTransJoint2TreeIndex[0]=1;
		aTree2TransJointIndex.setSize(GetNumTreeNode());
		aTree2TransJointIndex.setAllValue(-1);
		aTree2TransJointIndex[1]=0;
	}

	intvectorn aParentIndex;
	bf.unpack(aParentIndex);

	if(version>=4)
		bf.unpack(m_aVoca2TreeIndex);
	else
	{
		intvectorn aVoca2RotJointIndex;
		bf.unpack(aVoca2RotJointIndex);

		m_aVoca2TreeIndex.setSize(aVoca2RotJointIndex.size());
		for(int i=0; i<m_aVoca2TreeIndex.size(); i++)
		{
			int rotJoint=aVoca2RotJointIndex[i];
			if(rotJoint!=-1)
				m_aVoca2TreeIndex[i]=m_aRotJoint2TreeIndex[rotJoint];
			else
				m_aVoca2TreeIndex[i]=-1;
		}
	}

	for(int voca=0; voca<m_aVoca2TreeIndex.size(); voca++)
	{
		if(m_aVoca2TreeIndex[voca]!=-1)
			getBoneByTreeIndex(m_aVoca2TreeIndex[voca]).m_voca=voca;
	}

	bf.unpackInt(m_nNumEndNode);	//!< skeleton의 EndNode에는 KeyFrame이 들어가지 않으므로 중요.

	_setTree2RotJointIndexArray(aTree2RotJointIndex) ;
	_setTree2TransJointIndexArray(aTree2TransJointIndex) ;

	if(version<3)
	{
		m_nNumRotNode=m_nNumTreeNode-m_nNumEndNode-1;
		m_nNumTransNode=1;
	}

	for(int i=0; i<GetNumTreeNode(); i++)
	{
		if(aParentIndex[i]!=-1)
			m_apNode[aParentIndex[i]]->AddChild(m_apNode[i]);
	}

	m_pTreeRoot=m_apNode[0];

	UpdateInitialBone();

	if(!dofInfo._sharedinfo)
		dofInfo._sharedinfo=new MotionDOFinfo::SharedInfo;
	dofInfo._sharedinfo->init(*this);
	return version;
}


Bone& dep_GetSiteBoneVoca(MotionLoader const& ml, int jointVoca) { return dep_GetSiteBone(ml, ml.getRotJointIndexByVoca(jointVoca));}




int MotionLoader::getTreeIndexByRotJointIndex(int rotjointIndex) const
{
	return m_aRotJoint2TreeIndex[rotjointIndex];
}

int MotionLoader::getTreeIndexByTransJointIndex(int transjointIndex) const
{
	return m_aTransJoint2TreeIndex[transjointIndex];
}

int MotionLoader::getRotJointIndexByVoca(int jointVoca) const
{
	Msg::verify(m_aVoca2TreeIndex.size()!=0, "call readJointIndex before using Voca");
	return getBoneByVoca(jointVoca).rotJointIndex();
}

TString MotionLoader::getName() const 
{
	return "MotionLoader";
}
int MotionLoader::getTreeIndexByVoca(int jointVoca) const
{
	
	return m_aVoca2TreeIndex[jointVoca];
}

void MotionLoader::_changeVoca(int jointVoca, Bone & bone)
{
	bone.m_voca=jointVoca;
	m_aVoca2TreeIndex[jointVoca]=bone.treeIndex();
}

bool _MotionLoader_sortBones_compare(Node* first, Node* second)
{
	if(first->GetIndex()<second->GetIndex())
		return true;
	return false;
}


void _MotionLoader_sortBones(Bone& parent1, Bone& parent2)
{
	std::list<Node*> children1;
	std::list<Node*> children2;

	Msg::verify((!parent1.m_pChildHead && !parent2.m_pChildHead) ||
		(TString(parent1.GetNameId()).toUpper()==TString(parent2.GetNameId()).toUpper()), "name different");

	parent1.setChannels(parent2.getTranslationalChannels() , parent2.getRotationalChannels());

	parent1.detachAllChildren(children1);
	parent2.getAllChildren(children2);

	if(children1.size()!=children2.size())
	{
		parent1.addChildren(children1);
		Msg::error("num children");
	}

	children1.sort(_MotionLoader_sortBones_compare);
	parent1.addChildren(children1);

	std::list<Node*>::iterator i;
	std::list<Node*>::iterator i2=children2.begin();
	for(i=children1.begin();i!=children1.end(); ++i)
	{
		Msg::verify((*i)->GetIndex()==(*i2)->GetIndex(), "hierarchy different???");

		_MotionLoader_sortBones((Bone&)(**i), (Bone&)(**i2));
		++i2;
	}
}

void MotionLoader::sortBones(MotionLoader const& referenceSkeleton)
{
	if(m_cPostureIP.numFrames())
		Msg::print("Warning! please make sure that m_cPostureIP is manually modified");

	if(numBone()!= referenceSkeleton.numBone())
		Msg::error("sortBones: Numbone check");

	// joint정보는 reference skeleton에서 카피 해올것임.
	//if(numRotJoint()!=referenceSkeleton.numRotJoint())
	//	Msg::error("sortBones: numrotjoint check");

	//if(numTransJoint()!=referenceSkeleton.numTransJoint())
	//	Msg::error("sortBones: numtransjoint check");


	intvectorn targetIndex;
	targetIndex.setSize(numBone());
	for(int i=1; i<numBone(); i++)
	{
		Bone& bone1=getBoneByTreeIndex(i);
		int numChildren=bone1.numChildren();

		if(numChildren==0) continue;

		Bone& bone2=referenceSkeleton.getBoneByName(bone1.NameId);

		if(numChildren==1)
		{
			ASSERT(bone2.numChildren()==1);
			// Site본의 이름은 다를 수 있다.
			targetIndex[bone1.child()->treeIndex()]=
				bone2.child()->treeIndex();
		}
		targetIndex[i]=bone2.GetIndex();
	}

	for(int i=1; i<numBone(); i++)
		getBoneByTreeIndex(i).m_nIndex=targetIndex[i];

	_MotionLoader_sortBones(getBoneByTreeIndex(1), referenceSkeleton.getBoneByTreeIndex(1));

	m_pTreeRoot->printHierarchy();
	int numChannel;
	MakeBoneArrayFromTree(numChannel);
}

void Bone::printHierarchy(int depth)
{
	for(int ii=0; ii<depth; ii++) printf(" ");

	if (treeIndex()>=1 && getSkeleton().dofInfo._sharedinfo)
	{
		printf("%s : %d, R: %d, T: %d %s:%s ", NameId, treeIndex(), rotJointIndex(), transJointIndex(), getRotationalChannels().ptr(), getTranslationalChannels().ptr() );
		printf("startT: %d endR: %d\n",getSkeleton().dofInfo.startT(treeIndex()), getSkeleton().dofInfo.endR(treeIndex()));
	}

	for(Node *i=m_pChildHead; i!=NULL; i=i->m_pSibling)
		i->printHierarchy(depth+1);

}


PoseTransfer::PoseTransfer(MotionLoader* pSrcSkel, MotionLoader* pTgtSkel, const char* convfilename, bool bCurrPoseAsBindPose)
{
	_ctor(pSrcSkel, pTgtSkel, convfilename, bCurrPoseAsBindPose);
}
PoseTransfer::PoseTransfer(MotionLoader* pSrcSkel, MotionLoader* pTgtSkel, bool bCurrPoseAsBindPose)
{
	_ctor(pSrcSkel, pTgtSkel, NULL, bCurrPoseAsBindPose);
}
void PoseTransfer::_ctor(MotionLoader* pSrcSkel, MotionLoader* pTgtSkel, const char* convfilename, bool bCurrPoseAsBindPose)
{
	mpSrcSkel=pSrcSkel;
	mpTgtSkel=pTgtSkel;
	TStrings convTable;
	convTable.resize(pSrcSkel->numBone());

	if(convfilename && convfilename[0]!=0)
	{
		CTextFile file;

		Msg::verify(file.OpenReadFile(convfilename), "file open error %s", convfilename);

		char *token;
		while(token=file.GetToken())
		{
			char* jointName=token;

			int iindex=pSrcSkel->GetIndex(jointName);

			if(iindex==-1)
			{
				printf("warning %s not exist\n", jointName);
			}
			else
			{
				int ijoint=pSrcSkel->getRotJointIndexByTreeIndex(iindex);
				ASSERT(ijoint<pSrcSkel->numRotJoint());
				//convTable[ijoint]=file.GetToken();
				convTable[iindex]=file.GetToken();
			}
		}

		file.CloseFile();
	}
	else
	{
		// use joints name as ogre::bone name.
		//for(int ijoint=0; ijoint<pSrcSkel->numRotJoint(); ijoint++)
		for(int ibone=1; ibone<pSrcSkel->numBone(); ibone++)
		{
			convTable[ibone]=pSrcSkel->GetName(ibone);
		}
	}

	m_aTargetIndex.setSize(pSrcSkel->numRotJoint());
	m_aTargetIndexByTransJoint.setSize(pSrcSkel->numTransJoint());
	m_aRotOrigComb.setSize(pSrcSkel->numRotJoint());
	m_aInvRotOrigComb.setSize(pSrcSkel->numRotJoint());
	m_aLocalRotOrig.setSize(pSrcSkel->numRotJoint());
	m_aBindPose.setSize(pSrcSkel->numRotJoint()+1);

	for(int i=0; i<pSrcSkel->numTransJoint(); i++)
	{
		int ibone=pSrcSkel->getTreeIndexByTransJointIndex(i);
		if(convTable[ibone].length())
		{
			TString const& jointName=convTable[ibone];
			int iindex=mpTgtSkel->GetIndex(jointName);
			if(iindex==-1)
				printf("warning %s not exist\n", jointName.ptr());
			m_aTargetIndexByTransJoint[i]=iindex;
		}
		else
			m_aTargetIndexByTransJoint[i]=-1;
	}

	for(int i=0; i<pSrcSkel->numRotJoint(); i++)
	{
		int ibone=pSrcSkel->getTreeIndexByRotJointIndex(i);
		if(convTable[ibone].length())
		{
			TString const& jointName=convTable[ibone];
			int iindex=mpTgtSkel->GetIndex(jointName);
			m_aTargetIndex[i]=iindex;
			if(iindex==-1)
				printf("warning %s not exist\n", jointName.ptr());
			else
			{
				Bone& bone=mpTgtSkel->bone(iindex);

		//	ASSERT(isSimilar(bone.getRotation(), quater(1,0,0,0)));
		//	ASSERT(isSimilar(m_aLocalRotOrig[i], quater(1,0,0,0)));

				if(bCurrPoseAsBindPose)
				{
					m_aRotOrigComb[i]=bone.getRotation();
					m_aInvRotOrigComb[i].inverse(bone.getRotation());
					m_aLocalRotOrig[i]=bone.getLocalFrame().rotation;
					pSrcSkel->getBoneByRotJointIndex(i).getRotation(m_aBindPose[i]);
				}
				else
				{
					m_aRotOrigComb[i].identity();
					m_aInvRotOrigComb[i].identity();
					m_aLocalRotOrig[i].identity();
					m_aBindPose[i].identity();
				}
			}

		}
		else
			m_aTargetIndex[i]=-1;
	}

	if(bCurrPoseAsBindPose)
	{
		// bind root position저장.
		vector3 trans;
		pSrcSkel->getBoneByRotJointIndex(0).getTranslation(trans);
		vector3 trans2;
		pTgtSkel->getBoneByRotJointIndex(0).getTranslation(trans2);
		trans-=trans2;
		m_aBindPose[pSrcSkel->numRotJoint()].setValue(0, trans.x, trans.y, trans.z);
	}
	else
		m_aBindPose[pSrcSkel->numRotJoint()].setValue(0, 0,0,0);



	parentIdx.setSize(pSrcSkel->numRotJoint());
	parentIdx.setAllValue(-1);

	for(int i=1; i<pSrcSkel->numRotJoint(); i++)
	{
		// missing joint에 대한 고려를 해주어야 한다. 따라서 실제 사용된 조인트 만으로 hierarchy를 다시 만든다.
		int j=i;
		do
		{
			j=dep_GetParentJoint(*pSrcSkel,j);
			if(j==-1) break;
		}
		while(m_aTargetIndex[j]==-1);

		parentIdx[i]=j;
	}
}

void PoseTransfer::setTargetSkeletonBothRotAndTrans(const Posture& srcposture)
{
	setTargetSkeleton(srcposture);

	const MotionLoader& srcSkel=*mpSrcSkel;
	MotionLoader& tgtSkel=*mpTgtSkel;
	for(int i=0; i<srcposture.numTransJoint(); i++)
	{
		Bone& srcbone=srcSkel.getBoneByTransJointIndex(i);
		vector3 srctrans=srcbone.getTranslation();


		Bone& tgtbone=tgtSkel.bone(m_aTargetIndexByTransJoint[i]);
		
		tgtbone._getFrame().translation=srctrans;

		if(srcbone.parent()->treeIndex()!=0)
		{
			tgtbone.parent()->_getFrame().translation=
				srcbone.parent()->getTranslation();
		}

	}
}

void PoseTransfer::setTargetSkeleton(const Posture & posture)
{
	const MotionLoader& skeleton=*mpSrcSkel;
	quaterN aRotations(posture.numRotJoint());

	((MotionLoader&)skeleton).setPose(posture);

	// calc global rotations.
	for(int i=0; i<posture.numRotJoint(); i++)
		skeleton.getBoneByRotJointIndex(i).getRotation(aRotations[i]);

	// root or root2 should not be missing. (for positioning)
	int curbone=0;

	if(m_aTargetIndex[0]!=-1)
	{
		Bone* pBone=&mpTgtSkel->getBoneByTreeIndex(m_aTargetIndex[0]);



		//= L2 * C2.inv * C1BB * C1B.inv * C2B * C2BB.inv * C2 ---------------2

		pBone->_getLocalFrame().rotation=(m_aLocalRotOrig[0]* m_aInvRotOrigComb[0] * posture.m_aRotations[0]*m_aBindPose[0].inverse()* m_aRotOrigComb[0] );
	//	mpTgtSkel->getRootBone()->setPosition(0,0,0);
	//	pBone->setPosition(0,0,0);
		quater& qbindTrans=m_aBindPose[m_aBindPose.size()-1];
		vector3 bindTrans(qbindTrans.x, qbindTrans.y, qbindTrans.z);
		pBone->_getLocalFrame().translation=(posture.m_aTranslations[0]-bindTrans);

		curbone++;
	}
	else
	{
		assert(m_aTargetIndex[1]!=-1 && posture.numTransJoint()>1);

		Bone* pBone=&mpTgtSkel->getBoneByTreeIndex(m_aTargetIndex[1]);

		pBone->_getLocalFrame().rotation=(m_aLocalRotOrig[1]* m_aInvRotOrigComb[1] * aRotations[1]*m_aBindPose[1].inverse()* m_aRotOrigComb[1] );
	//	mpTgtSkel->getRootBone()->setPosition(0,0,0);
	//	pBone->setPosition(0,0,0);
		quater& qbindTrans=m_aBindPose[m_aBindPose.size()-1];
		vector3 bindTrans(qbindTrans.x, qbindTrans.y, qbindTrans.z);
		vector3 v;
		skeleton.getBoneByRotJointIndex(1).getTranslation(v);
		ASSERT(0);	// 아래줄을 어떻게 바꿔야하는지 생각하기 귀찮음.
		//m_pSceneNode->setPosition(ToOgre(m_vTrans+v-bindTrans));

		curbone=2;
	}


	for(int i=curbone; i<posture.numRotJoint(); i++)
	{
		if(m_aTargetIndex[i]!=-1)
		{
			Bone* pBone=&mpTgtSkel->getBoneByTreeIndex(m_aTargetIndex[i]);

			/////////////////////////////////////////////////////////////////////////////////////////
			// Bind pose는 임의의 포즈가 될수 있다. (MaxExporter특성상, 주로 frame0를 사용한다.)

			// 현재 오우거 노드가 부모로부터 3번째 노드라고 하면.
			//  노드의 global orientation CN2=LN0*LN1*LN2
			//  C2가 모델의 binding pose에서의 global orientation이라 하자.

			// 목표:
			// binding pose를 SetPose함수에 입력한 경우, CN2가 C2가 되도록 해야한다.
			// C2BB 가 동작 데이타 바인딩 포즈의 combined mat이라 하자.
			// CN2 = C2B * C2BB.inv * C2 가 되면 된다.
			// (즉 C2BB와 C2B(SetPose함수의 입력자세)가 동일한 경우, CN2==C2)

			// CN2=CN1*LN2 이므로
			// -> LN2=CN1.inv * CN2
			//       =CN1.inv * C2B * C2BB.inv * C2
			//       = ( C1B * C1BB.inv* C1).inv * C2B * C2BB.inv * C2 ---------------1
			//		 = C1.inv* C1BB * C1B.inv * C2B * C2BB.inv * C2
			//		 = L2 * C2.inv * C1BB * C1B.inv * C2B * C2BB.inv * C2 ---------------2

			// 검산
			//  CN3= LN0 * lN1* LN2
			//  = (L0B * L0BB.inv * L0)*(L0.inv * L0BB * L0B.inv * C1B * C1BB.inv * C1)*(C1.inv* C1BB * C1B.inv * C2B * C2BB.inv * C2)
			//  =  C2B * C2BB.inv * C2


			pBone->_getLocalFrame().rotation=(
				m_aLocalRotOrig[i]*m_aInvRotOrigComb[i]			// C1.inv
				* m_aBindPose[parentIdx[i]]				// C1BB
				* aRotations[parentIdx[i]].inverse() * aRotations[i]	// C1B.inv * C2B
				* m_aBindPose[i].inverse()				// C2BB.inv
				* m_aRotOrigComb[i]);					// C2

			// maya에서 export한경우 아래 한줄이면 동작함.
			//pBone->m_matRot.setRotationWhilePreservingTranslation(posture.m_aRotations[i]);


			// 고려사항 1. 양쪽의 link길이가 다른경우. 예를 들어 Ogre쪽은 link가 4개(0,1,@,2)이지만, 포즈쪽은 link가 3개(0,1,2)라 하자
			// 위식 1 대로 계산하면
			// LN2 = L2 * C2.inv * C1BB * C1B.inv * C2B * C2BB.inv * C2;
			// LN@ = L@;
			// LN1 = L1 * C1.inv * C0BB * C0B.inv * C1B * C1BB.inv * C1
			// LN0 = C0B * C0BB.inv * L0;

			// 다곱하면.
			// CN2= C2B * C2BB.inv * C2; -> 문제 없음

			// 고려사항 2. 링크 구조가 다른경우. - 확실히 문제 생기는듯함. 수정요.
		}
	}

	mpTgtSkel->UpdateBone();
#ifdef _DEBUG_MAT


	for(int i=1; i<posture.numRotJoint(); i++)
	{
		// 검산. (본 구조가 다르면 틀릴때가 있는거 같음. 수정할 필요)
		if(m_aTargetIndex[i]!=-1)
		{
			Ogre::Bone* pBone=mpTgtSkel->getBone(unsigned short(m_aTargetIndex[i]));
			quater CNi=ToBase(pBone->_getDerivedOrientation());
			quater Ci=m_aRotOrigComb[i];
			//CN2 = C2B * C2BB.inv * C2
			quater CNi2= aRotations[i]* m_aBindPose[i].inverse() * m_aRotOrigComb[i];

			RE::output(TString(sz0::format("c%d", i)), "%s = %s =0 %s", CNi.output().ptr(),CNi2.output().ptr(),Ci.output().ptr());
			//

		}
	}
#endif
}
#include <iostream>
using namespace std;
void MotionLoader::updateBoneLengthFromGlobal()
{

	for (int i=2;i< numBone();i++){
		auto& b=bone(i);
		auto& g1=b.parent()->getFrame();
		auto& g2=b.getFrame();

		// g2=g1*l1
		transf l1=g1.inverse()*g2;
		b._getOffsetTransform().translation=l1.translation;
	}
	return;
	NodeStack  stack;
	stack.Initiate();

	Node *src=m_pTreeRoot->m_pChildHead;	// dummy노드는 사용안함.
	int index=-1;

	auto& fk=fkSolver();

	while(TRUE) 
	{
		while(src)
		{
			index++;
			ASSERT(src->NodeType==BONE);
			Bone* pBone=(Bone*)src;
			int treeindex=pBone->treeIndex();
			if(stack.GetTop())
			{
				//	_global(treeindex)= global(*((Bone*)stack.GetTop()))* local(treeindex);
				fk._local(treeindex)=fk._global(*((Bone*)stack.GetTop())).inverse()*fk._global(treeindex);
		//cout<<treeindex<<" :" <<"->"<< bone(treeindex)._getOffsetTransform().translation<< fk._local(treeindex).translation <<			endl;
				bone(treeindex)._getOffsetTransform().translation=fk.local(treeindex).translation;
			}
			else
			{
				fk._local(treeindex)=fk._global(treeindex);
				//bone(treeindex)._getOffsetTransform().translation=fk.local(treeindex).translation;
			}

			stack.Push(src);

			src=src->m_pChildHead;
		}
		stack.Pop(&src);
		if(!src) break;
		src=src->m_pSibling;
	}
}
