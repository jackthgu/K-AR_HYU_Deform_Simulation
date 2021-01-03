#include "stdafx.h"
#include "TetraMesh.h"
#include "ModelingWin.h"
#include "UI_mode.h"
#include "Tool.h"
#include "BaseLib/math/Operator.h"
#include "BaseLib/math/Operator_NR.h"
#include "MainLib/OgreFltk/MovableText.h"
#include "AsRigidAsPossible.h"
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include "MainLib/OgreFltk/FlChoice.h"
#include "MainLib/OgreFltk/framemoveobject.h"
#include "MainLib/OgreFltk/timesensor.h"
#include "MainLib/OgreFltk/AnimationObject.h"
#include "MainLib/OgreFltk/FltkRenderer.h"
#include "MainLib/OgreFltk/FltkAddon.h"
#include "MainLib/OgreFltk/MotionPanel.h"
#include "MainLib/OgreFltk/Mesh.h"
using namespace modelingWin;

void MeanValueWeights(OBJloader::Mesh const& m, vectorn & w, vector3 const& x);


DirectManipulation::DirectManipulation(ModelingWin& win):ModelingWin_Tool (win)
			,mSelectVertex((SelectVertex&)(*_win.m_modes[ModelingWin::SELECT_VERTEX]))
{
	create("Choice", "Mode", "Mode",1);
	menu(0)->size(4);
	menu(0)->item(0, "select", 'z');
	menu(0)->item(1, "translate", 'x');
	menu(0)->item(2, "rotate", 'c');
	menu(0)->item(3, "scale", 'v');
	menu(0)->value(0);
	m_currMode=0;

	create("Choice", "operations", 0, 0);
	menu(0)->size(11);
	int curop=0;
	menu(0)->item(curop++,"operations");
	menu(0)->item(curop++,"drawNormals");
	menu(0)->item(curop++,"drawFaceIndex");
	menu(0)->item(curop++,"drawVertexIndex");
	menu(0)->item(curop++,"mesh laplacian");
	menu(0)->item(curop++,"mean value weights");
	menu(0)->item(curop++,"save selection");
	menu(0)->item(curop++,"load selection");
	menu(0)->item(curop++,"save mesh");
	menu(0)->item(curop++,"load mesh");
	menu(0)->item(curop++, "flip edge", 'f');

	updateLayout();
	mCurrGizmo="none";

}
		
bool DirectManipulation::onLDown(int x, int y)
{
	if(Fl::event_alt()) return false;

	if(m_currMode!=SELECT && mCurrGizmo!="none")
	{
		Ogre::RaySceneQuery* query=RE::FltkRenderer().createRayQuery(x,y);
		Ogre::RaySceneQueryResult& result=query->execute();

		Ogre::RaySceneQueryResult::iterator itr;


		mAxis.setValue(0,0,0);
		//return true;
		for (itr = result.begin(); itr != result.end(); ++itr)
		{
			TString movableId=(*itr).movable->getName().c_str();
			if(movableId=="_entity_x_axis")
			{
				mAxis.setValue(1,0,0);
				break;
			}
			if(movableId=="_entity_y_axis")
			{
				mAxis.setValue(0,1,0);
				break;
			}
			if(movableId=="_entity_z_axis")
			{
				mAxis.setValue(0,0,1);
				break;
			}
		}
		if(itr==result.end())
		{
			RE::ogreSceneManager()->destroyQuery(query);
			return false;
		}
		RE::ogreSceneManager()->destroyQuery(query);
		return true;
	}
	return false;	
}
		
void DirectManipulation::transformSelectedVertices(matrix4 const& mat)
{
	for(int i=0; i<_win.numVertex(); i++)
	{
		if(mSelectVertex.mSelectedVertices[i])
			_win.mesh()->getVertex(i).leftMult(mat);
	}

}

		// translate only gizmo
void DirectManipulation::onLDrag(int pushed_x, int pushed_y, int x, int y)
{
	if(mAxis.length()<0.5) return;

	if(mCurrGizmo!="none")
	{
		float x1, y1;
		float x2, y2;

		RE::FltkRenderer().worldToScreen(mGizmoCenter, x1, y1);
		RE::FltkRenderer().worldToScreen(mGizmoCenter+mAxis, x2, y2);

		vector3 nor(x2-x1,y2-y1,0);
		m_real len=nor.length();
		nor/=len;
		vector3 dif(x-pushed_x, y-pushed_y,0);


		matrix4 mat;

		if(mCurrGizmo=="translate")
		{
			mat.setTranslation(mGizmoOffset*-1, false);
			transformSelectedVertices(mat);
			mGizmoOffset=mAxis*((nor%dif)/len);
			mat.setTranslation(mGizmoOffset, false);
		}
		else if(mCurrGizmo=="rotate")
		{
			quater q;
			q.setRotation(mGizmoOffset);
			mat.setTranslation(mGizmoCenter*-1, false);
			mat.leftMultRotation(q.inverse());
			q.setRotation(mAxis,(nor%dif)/300.0);
			mat.leftMultRotation(q);
			mat.leftMultTranslation(mGizmoCenter);
			mGizmoOffset=q.rotationVector();
		}
		else if(mCurrGizmo=="scale")
		{
			m_real s=mGizmoOffset.x+1.0;
			vector3 sv=mAxis*(s-1.0)+vector3(1,1,1);
			mat.setTranslation(mGizmoCenter*-1, false);
			mat.leftMultScaling(1.0/sv.x, 1.0/sv.y, 1.0/sv.z);
			s=(nor%dif)/300.0+1.0;
			sv=mAxis*(s-1.0)+vector3(1,1,1);
			mat.leftMultScaling(sv.x,sv.y,sv.z);
			mat.leftMultTranslation(mGizmoCenter);
			mGizmoOffset.x=s-1.0;
		}
		transformSelectedVertices(mat);

		_win.notify(UI_mode::MESH_VERTEX_MODIFIED);
	}
}

void DirectManipulation::notify(int msg)
{
	switch(msg)
	{
	case UI_mode::MESH_VERTEX_MODIFIED:
		redraw();
		break;
	case UI_mode::MESH_CHANGED:
		redraw();
		break;
	}
}

void DirectManipulation::onLUpNoDrag(int x, int y)
{
	redraw();
}

void DirectManipulation::onLUpDrag(int pushed_x, int pushed_y, int x, int y)
{
	if(mAxis.length()<0.5) return;

	if(mCurrGizmo=="translate")
	{
		//matrix4 mat;
		//mat.setTranslation(mGizmoOffset, false);
		//transformSelectedVertices(mat);

		_win.mesh()->calculateVertexNormal();
		_win.notify(UI_mode::MESH_VERTEX_MODIFIED);
		updateGizmo();
	}
}

void DirectManipulation::redraw()
{
	mObjectList.clear();

	if(mCurrGizmo!="none")
	{
		m_real s=1.0;
		vector3 center=mGizmoCenter+mGizmoOffset;
		if(mCurrGizmo!="translate")
			center-=mGizmoOffset;

		RE::moveEntity(mObjectList.registerEntity("x_axis", "x_axis.mesh"),vector3(s,s,s), center);
		RE::moveEntity(mObjectList.registerEntity("y_axis", "y_axis.mesh"),vector3(s,s,s), center);
		RE::moveEntity(mObjectList.registerEntity("z_axis", "z_axis.mesh"),vector3(s,s,s), center);
	}

}

// redraw is not called automatically.
void DirectManipulation::updateGizmo()
{
	if(m_currMode!=SELECT &&mSelectVertex.mSelectedVertices.count())
	{
		if(m_currMode==TRANSLATE)
			mCurrGizmo="translate";
		else if(m_currMode==ROTATE)
			mCurrGizmo="rotate";
		else if(m_currMode==SCALE)
			mCurrGizmo="scale";

		vector3 center(0,0,0);
		int count=0;
		for(int i=0; i<mSelectVertex.mSelectedVertices.size(); i++)
		{
			if(mSelectVertex.mSelectedVertices[i])
			{
				center+=_win.mesh()->getVertex(i);
				count++;
			}
		}

		center/=count;
		mGizmoCenter=center;
		mGizmoOffset.setValue(0,0,0);
	}
	else
	{
		mCurrGizmo="none";
	}
}
void DirectManipulation::onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData)
{
	if(w.mId=="Mode")
	{
		TString op=w.menu()->text();
		if(w.menu()->value()!=m_currMode)
		{
			if(getUIState()!=READY)
			{
				w.menu()->value(m_currMode);
				w.menu()->redraw();
			}
			else
				m_currMode=w.menu()->value();
		}
		updateGizmo();
		redraw();
	}
	else if(w.mId=="operations")
	{
		OBJloader::EdgeConnectivity* edges=_win.edges();
		OBJloader::MeshConnectivity* faces=_win.faces();

		TString op=w.menu()->text();

		if(op=="flip edge")
		{
			SelectEdge& se=(SelectEdge&)(*_win.m_modes[ModelingWin::SELECT_EDGE]);

			if(se.mSelectedEdges.count()==1)
			{
				for(int i=0; i<se.mSelectedEdges.size(); i++)
				{
					if(se.mSelectedEdges[i])
					{
						if(edges->getEdge(i).numFaceEdge==2)
						{
							int f1=edges->getEdge(i).faceEdge[0].faceIndex;
							int f2=edges->getEdge(i).faceEdge[1].faceIndex;

							int v1=edges->getEdge(i).faceEdge[0].source(*_win.mesh());
							int v2=edges->getEdge(i).faceEdge[0].target(*_win.mesh());
							int v3=edges->getEdge(i).faceEdge[0].cross(*_win.mesh());
							int v4=edges->getEdge(i).faceEdge[1].cross(*_win.mesh());

							_win.mesh()->getFace(f1).setIndex(v2, v3, v4);
							_win.mesh()->getFace(f2).setIndex(v4, v3, v1);

							_win.mesh()->saveMesh("__backup.tri");
							se.mSelectedEdges.clearAll();
							se.redraw();
							_win.notify(UI_mode::MESH_CHANGED);
							break;
						}
						else
							Msg::msgBox("Cannot flip");
					}
				}
			}
			else
			{
				Msg::msgBox("First select 1 edge. (%d selected)", se.mSelectedEdges.count());
			}
		}
		else if(op=="mean value weights")
		{
			SelectVertex& sv=(SelectVertex&)(*_win.m_modes[ModelingWin::SELECT_VERTEX]);

			if(sv.mSelectedVertices.count())
			{
				vector3 mean(0,0,0);
				for(int i=0; i<sv.mSelectedVertices.size(); i++)
					if(sv.mSelectedVertices[i]) mean+=_win.mesh()->getVertex(i);

				mean/=(double)sv.mSelectedVertices.count();
			
				RE::moveEntity(RE::createEntity("x_pos", "sphere1010.mesh"), quater(1,0,0,0), mean);
				
				vectorn w;
				MeanValueWeights(*_win.mesh(), w, mean);

				OBJloader::Mesh mesh;
				mesh=*_win.mesh();

				{
					for(int i=0; i<mesh.numVertex(); i++)
					{
						float c=w[i];
						mesh.getColor(i)=vector4(c,c,c,1.0);
					}
					
					TString nodeName="node_mean value weights";
					Ogre::SceneNode* pnode=RE::createSceneNode(nodeName);
					OBJloader::MeshToEntity::Option o;
					o.useColor=true;
					pnode->attachObject(OBJloader::createMeshEntity(mesh, nodeName+"entity", "use_vertexcolor_only", o));
					pnode->translate(0,40,0);
				}

			}
			else
			{
				Msg::msgBox("select vertices first");
			}

			
		}
		else if(op=="mesh laplacian")
		{
			int num_vertex=_win.numVertex();

			intvectorn degrees(num_vertex);

			for(int i=0; i<num_vertex; i++)
				degrees[i]=edges->numAdjEdges(i);

			matrixn L(num_vertex, num_vertex);
			L.setAllValue(0.0);


			const bool bUseKarniLaplacian=true;

			if(bUseKarniLaplacian)
			{
				L.setDiagonal(1.0);
				for(int i=0; i<num_vertex; i++)
					for(int j=i+1; j<num_vertex; j++)
					{
						if(edges->isConnected(i,j))
						{
							L[i][j]=-1.0/(double)(degrees[i]);
							L[j][i]=-1.0/(double)(degrees[j]);
						}
					}
			}
			else
			{
				matrixn w(num_vertex, num_vertex);
				w.setAllValue(0);
				for(int i=0; i<num_vertex; i++)
					for(int j=0; j<num_vertex; j++)
					{
						if(edges->isConnected(i,j))
					//		w[i][j]=1.0;
					w[i][j]=1.0/_win.mesh()->getVertex(i).distance(_win.mesh()->getVertex(j));
					}



				for(int i=0; i<num_vertex; i++)
					for(int j=0; j<num_vertex; j++)
					{
						if(i==j)
							L[i][j]=w.row(i).sum();
						else
							L[i][j]=-1.0*w[i][j];
					}
			}

			vectorn eigenValues;
			matrixn eigenVectors;
			m::eigenDecomposition(L, eigenValues, eigenVectors, 0);

			OBJloader::Mesh mesh;
			mesh=*_win.mesh();

			RE::output("eigen values", eigenValues.output().ptr());
			for(int mode=0; mode<eigenValues.size(); mode++)
			//for(int mode=0; mode<5; mode++)
			{
				vectornView ev=eigenVectors.column(eigenValues.size()-mode-1);

				ev/=ev.maximum();
				//printf("ev%d %s\n", mode, ev.output().ptr());
				for(int i=0; i<num_vertex; i++)
				{
					float c=ev[i];
					mesh.getColor(i)=vector4(c,c,c,1.0);
				}
				
				TString nodeName;
				nodeName.format("node_mesh_laplacian%d", mode);
				Ogre::SceneNode* pnode=RE::createSceneNode(nodeName);
				OBJloader::MeshToEntity::Option o;
				o.useColor=true;
				pnode->attachObject(OBJloader::createMeshEntity(mesh, nodeName+"entity", "use_vertexcolor_only", o));
				pnode->translate(0,mode*40+40,0);
			}
		}
		else if(op=="save mesh")
		{
			TString fn=FlChooseFile("save", "../Resource/mesh", "*.tri", true);
			if(fn.length())
			{
				bool bUnmerge=false;
				if(_win.unmergeInfo())
				{
					if(Msg::confirm("Do you want to export unmerged vertices?"))
					{
						bUnmerge=true;
						_win.skeleton()->mMesh.unmergeDuplicateVertices(_win.unmergeInfo());
					}
				}
				_win.mesh()->saveMesh(fn);
				_win.mesh()->saveObj(fn.left(-3)+"obj", _win.skeleton()->mMesh.option.useNormal, _win.skeleton()->mMesh.option.useTexCoord);

				if(bUnmerge)
				{
					_win.setUnmergeInfo(_win.skeleton()->mMesh.mergeDuplicateVertices(true));
				}
			}
		}
		else if(op=="load mesh")
		{
			TString fn=FlChooseFile("save", "../Resource/mesh", "*.{tri,obj}.");
			if(fn.length())
			{
				OBJloader::Mesh temp;

				if(fn.right(3).toUpper()=="OBJ")
					temp.loadObj(fn);
				else
					temp.loadMesh(fn);
				if(_win.mesh()->numVertex()==temp.numVertex())
				{
					(*_win.mesh())=temp;
					_win.notify(MESH_CHANGED);
				}
				else
					Msg::msgBox("Error! incompatible mesh.");
			}
		}
		else if(op=="save selection")
		{
			
			TString fn=FlChooseFile("input file name to save", "../Resource/scripts/ui/", "*.selection", true);
			if(fn.length()==0)
				return;
			BinaryFile bf(true, fn);
			bf.pack(((SelectVertex*)_win.m_modes[ModelingWin::SELECT_VERTEX].get())->mSelectedVertices);
			bf.pack(((SelectEdge*)_win.m_modes[ModelingWin::SELECT_EDGE].get())->mSelectedEdges);
			bf.close();
		}
		else if(op=="load selection")
		{
			TString fn=FlChooseFile("input file name to save", "../Resource/scripts/ui/", "*.selection");
			if(fn.length()==0)
				return;
			BinaryFile bf(false, fn);
			bf.unpack(((SelectVertex*)_win.m_modes[ModelingWin::SELECT_VERTEX].get())->mSelectedVertices);
			bf.unpack(((SelectEdge*)_win.m_modes[ModelingWin::SELECT_EDGE].get())->mSelectedEdges);
			bf.close();
			updateGizmo();
			_win.notify(UI_mode::SELECTION_CHANGED);
		}
		else if(op=="drawNormals")
		{
	#ifdef USE_NO_SHARED_VERTEX
			OBJloader::Mesh& mesh=mMeshNoSharedVertex;
	#else
			OBJloader::Mesh& mesh=*_win.mesh();
	#endif

			LineList* line=new LineList();
			line->begin(mesh.numVertex());

			for(int i=0, ni=mesh.numVertex(); i<ni; i++)
			{
				line->line(i, mesh.getVertex(i), mesh.getVertex(i)+mesh.getNormal(i));
			}
			line->end();
			_win.sceneNode()->attachObject(line);

		}
		else if(op=="drawFaceIndex")
		{

			for(int i=0; i<_win.mesh()->numFace(); i++)
			{
				vector3 pos=_win.mesh()->calcFaceCenter(i)+_win.mesh()->calcFaceNormal(i)*0.8;

				Ogre::SceneNode* child=_win.sceneNode()->createChildSceneNode(ToOgre(pos));

				Ogre::MovableText* text=new Ogre::MovableText(RE::generateUniqueName().ptr(), sz1::format("f%d", i).ptr(), "BlueHighway", 1, Ogre::ColourValue::Black);
				//text->showOnTop(true);
				child->attachObject(text);
			}
		}
		else if(op=="drawVertexIndex")
		{

			for(int i=0; i<_win.numVertex(); i++)
			{
				vector3 pos=_win.mesh()->getVertex(i)+_win.mesh()->getNormal(i)*0.8;

				Ogre::SceneNode* child=_win.sceneNode()->createChildSceneNode(ToOgre(pos));

				Ogre::MovableText* text=new Ogre::MovableText(RE::generateUniqueName().ptr(), sz1::format("v%d", i).ptr(), "BlueHighway", 1, Ogre::ColourValue::Black);
				//text->showOnTop(true);
				child->attachObject(text);
			}
		}
	}
}

///-----------------

LaplacianEditing::LaplacianEditing(ModelingWin& win):ModelingWin_Tool (win)
,mSelectVertex((SelectVertex&)(*_win.m_modes[ModelingWin::SELECT_VERTEX]))
{
	create("Choice", "Mode", "Mode",1);
	menu(0)->size(5);
	menu(0)->item(0, "select", 'z');
	menu(0)->item(1, "select control vertices", 'x');
	menu(0)->item(2, "translate", 'c');
	menu(0)->item(3, "rotate", 'v');
	menu(0)->item(4, "scale", 'b');
	menu(0)->value(0);
	m_currMode=0;

	create("Choice", "operations", 0, 0);
	menu(0)->size(7);
	int curop=0;
	menu(0)->item(curop++,"operations");
	menu(0)->item(curop++,"add to control vertices", 'n');
	menu(0)->item(curop++,"remove from control vertices",'m');
	menu(0)->item(curop++, "initialize deformer (as-rigid-as possible)");
	menu(0)->item(curop++, "initialize deformer (simple)");
	menu(0)->item(curop++, "create a guide mesh");
	menu(0)->item(curop++, "clear guide mesh");

	updateLayout();
	mLaplacian=NULL;

	mCurrGizmo="none";
}
void LaplacianEditing::updateControlVertices()
{
	if(mControlVertices.size()!=_win.numVertex())
	{
		mControlVertices.resize(_win.numVertex());
		mControlVertices.clearAll();
	}

	mControlVertexPos.resize(mControlVertices.size());
	for(int i=0; i<mControlVertices.size(); i++)
	{
		mControlVertexPos[i]=_win.mesh()->getVertex(i);
	}

}

void LaplacianEditing::notify(int msg)
{
	switch(msg)
	{
	case UI_mode::MESH_CHANGED:
	case UI_mode::MESH_VERTEX_MODIFIED:
		updateControlVertices();
		redraw();
		break;
	}
}

bool LaplacianEditing::onLDown(int x, int y)
{
	if(Fl::event_alt()) return false;

	if(m_currMode==SELECT_CONTROL_VERTICES)
	{
		return mSelectVertex.onLDown(x,y);
	}
	if(m_currMode!=SELECT && mCurrGizmo!="none")
	{
		Ogre::RaySceneQuery* query=RE::FltkRenderer().createRayQuery(x,y);
		Ogre::RaySceneQueryResult& result=query->execute();

		Ogre::RaySceneQueryResult::iterator itr;

		mAxis.setValue(0,0,0);
		//return true;
		for (itr = result.begin(); itr != result.end(); ++itr)
		{
			TString movableId=(*itr).movable->getName().c_str();
			if(movableId=="_entity_x_axis")
			{
				mAxis.setValue(1,0,0);
				break;
			}
			if(movableId=="_entity_y_axis")
			{
				mAxis.setValue(0,1,0);
				break;
			}
			if(movableId=="_entity_z_axis")
			{
				mAxis.setValue(0,0,1);
				break;
			}
		}
		if(itr==result.end())
		{
			RE::ogreSceneManager()->destroyQuery(query);
			return false;
		}
		RE::ogreSceneManager()->destroyQuery(query);
		return true;
	}
	return false;
}

void LaplacianEditing::transformSelectedVertices(matrix4 const& mat)
{
	for(int i=0; i<_win.numVertex(); i++)
	{
		if(mSelectVertex.mSelectedVertices[i] && mControlVertices[i])
			mControlVertexPos[i].leftMult(mat);
	}
}

// translate only gizmo
void LaplacianEditing::onLDrag(int pushed_x, int pushed_y, int x, int y)
{
	if(m_currMode==SELECT_CONTROL_VERTICES)
	{
		mSelectVertex.onLDrag(pushed_x, pushed_y, x,y);
		return ;
	}

	if(mCurrGizmo!="none")
	{
		float x1, y1;
		float x2, y2;

		RE::FltkRenderer().worldToScreen(mGizmoCenter, x1, y1);
		RE::FltkRenderer().worldToScreen(mGizmoCenter+mAxis, x2, y2);

		vector3 nor(x2-x1,y2-y1,0);
		m_real len=nor.length();
		nor/=len;
		vector3 dif(x-pushed_x, y-pushed_y,0);

		matrix4 mat;

		if(mCurrGizmo=="translate")
		{
			mat.setTranslation(mGizmoOffset*-1, false);
			transformSelectedVertices(mat);
			mGizmoOffset=mAxis*((nor%dif)/len);
			mat.setTranslation(mGizmoOffset, false);
		}
		else if(mCurrGizmo=="rotate")
		{
			quater q;
			q.setRotation(mGizmoOffset);
			mat.setTranslation(mGizmoCenter*-1, false);
			mat.leftMultRotation(q.inverse());
			q.setRotation(mAxis,(nor%dif)/300.0);
			mat.leftMultRotation(q);
			mat.leftMultTranslation(mGizmoCenter);
			mGizmoOffset=q.rotationVector();
		}
		else if(mCurrGizmo=="scale")
		{
			m_real s=mGizmoOffset.x+1.0;
			vector3 sv=mAxis*(s-1.0)+vector3(1,1,1);
			mat.setTranslation(mGizmoCenter*-1, false);
			mat.leftMultScaling(1.0/sv.x, 1.0/sv.y, 1.0/sv.z);
			s=(nor%dif)/300.0+1.0;
			sv=mAxis*(s-1.0)+vector3(1,1,1);
			mat.leftMultScaling(sv.x,sv.y,sv.z);
			mat.leftMultTranslation(mGizmoCenter);
			mGizmoOffset.x=s-1.0;
		}
		transformSelectedVertices(mat);

		redraw();
	}
}

void LaplacianEditing::onLUpNoDrag(int x, int y)
{
	redraw();
}

void LaplacianEditing::onLUpDrag(int pushed_x, int pushed_y, int x, int y)
{
	if(m_currMode==SELECT_CONTROL_VERTICES)
	{
		mSelectVertex.onLUpDrag(pushed_x, pushed_y, x,y);

		for(int i=0; i<mSelectVertex.mSelectedVertices.size(); i++)
		{
			if(mSelectVertex.mSelectedVertices[i] && !mControlVertices[i])
				mSelectVertex.mSelectedVertices.clearAt(i);
		}
		mSelectVertex.redraw();
		return ;
	}

	if(mCurrGizmo!="none")
	{
		//matrix4 mat;
		//mat.setTranslation(mGizmoOffset, false);
		//transformSelectedVertices(mat);

		intvectorn conIndex;
		vector3N conPos;

		for(int i=0; i<_win.numVertex(); i++)
		{
			if(mControlVertices[i])
			{
				conIndex.push_back(i);
				conPos.pushBack(mControlVertexPos[i]);
			}
		}
		if(mLaplacian)
		{
			matrixn deformed;
			mLaplacian->solve(conIndex, conPos, deformed);

			for(int i=0; i<_win.numVertex(); i++)
			{
				_win.mesh()->getVertex(i)=deformed.row3(i);
			}
		}

		updateGizmo();
		updateControlVertices();
		_win.mesh()->calculateVertexNormal();
		_win.notify(MESH_VERTEX_MODIFIED);
	}
}

void LaplacianEditing::redraw()
{
	mObjectList.clear();

	if(mCurrGizmo!="none")
	{
		m_real s=1.0;
		vector3 center=mGizmoCenter+mGizmoOffset;
		if(mCurrGizmo!="translate")
			center-=mGizmoOffset;
		RE::moveEntity(mObjectList.registerEntity("x_axis", "x_axis.mesh"),vector3(s,s,s), center);
		RE::moveEntity(mObjectList.registerEntity("y_axis", "y_axis.mesh"),vector3(s,s,s), center);
		RE::moveEntity(mObjectList.registerEntity("z_axis", "z_axis.mesh"),vector3(s,s,s), center);
	}

	mObjectVertices.clear();
	
	if(mControlVertices.count())
	{
		/*LineList* box=new LineList();

		for(int i=0; i<_win.numVertex(); i++)
		{
			if(mControlVertices[i])
			{

				vector3 d(0.5,0.5,0.5);
				vector3 center(mControlVertexPos[i]);
				box->addAxisAlignedBox(center-d,center+d);
			}
		}
		box->end();*/

		QuadList* box=new QuadList(vector3(0,1,0), 0.6);
		box->setCastShadows(false);
		box->setMaterial("redCircle");

		box->begin(mControlVertices.count());
		int c=0;
		for(int i=0; i<_win.numVertex(); i++)
		{
			if(mControlVertices[i])
				box->quad(c++, mControlVertexPos[i]);
		}
		box->end();

		mObjectVertices.registerObject(RE::generateUniqueName(), box);
	}
}

// redraw is not called automatically.
void LaplacianEditing::updateGizmo()
{
	if(m_currMode!=SELECT &&(mSelectVertex.mSelectedVertices&mControlVertices).count())
	{
		if(m_currMode==TRANSLATE)
			mCurrGizmo="translate";
		else if(m_currMode==ROTATE)
			mCurrGizmo="rotate";
		else if(m_currMode==SCALE)
			mCurrGizmo="scale";

		vector3 center(0,0,0);
		int count=0;
		for(int i=0; i<mSelectVertex.mSelectedVertices.size() ; i++)
		{
			if(mSelectVertex.mSelectedVertices[i] && mControlVertices[i])
			{
				center+=_win.mesh()->getVertex(i);
				count++;
			}
		}

		center/=count;
		mGizmoCenter=center;
		mGizmoOffset.setValue(0,0,0);
	}
	else
	{
		mCurrGizmo="none";
	}
}
void LaplacianEditing::onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData)
{
	if(w.mId=="Mode")
	{
		TString op=w.menu()->text();
		if(w.menu()->value()!=m_currMode)
		{
			if(getUIState()!=READY)
			{
				w.menu()->value(m_currMode);
				w.menu()->redraw();
			}
			else
				m_currMode=w.menu()->value();
		}
		updateGizmo();
		redraw();
	}
	else if(w.mId=="operations")
	{
		OBJloader::Mesh& mMesh=*_win.mesh();
		OBJloader::EdgeConnectivity* mEdges=_win.edges();
		OBJloader::MeshConnectivity* mFaces=_win.faces();

		TString op=w.menu()->text();
		bool bAdd;
		if((bAdd=(op=="add to control vertices")) ||op=="remove from control vertices")
		{
			updateControlVertices();

			for(int i=0; i<mSelectVertex.mSelectedVertices.size(); i++)
			{
				if(mSelectVertex.mSelectedVertices[i])
				{
					if(bAdd)
						mControlVertices.setAt(i);
					else
						mControlVertices.clearAt(i);
				}
			}
			redraw();
		}
		else if(op=="create a guide mesh")
		{
			TString fn=FlChooseFile("file", "../Resource/mesh", "*.{tri,obj}");
			if(fn.length())
			{
				OBJloader::Mesh mesh;
				if(fn.right(3).toUpper()=="OBJ")
					mesh.loadObj(fn);
				else
					mesh.loadMesh(fn);
				Ogre::Entity* pent=OBJloader::createMeshEntity(mesh,RE::generateUniqueName());
				mGuideMeshList.registerObject(RE::generateUniqueName(),pent);
			}
		}
		else if(op=="clear guide mesh")
		{
			mGuideMeshList.clear();
		}
		else if(op.left(19)== "initialize deformer")
		{
			if(_win.numVertex() && mControlVertices.count()>3)
			{
				delete mLaplacian;

				if(op.right(8)=="(simple)")
					mLaplacian=createSimpleDeformer(*_win.mesh());
				else
					mLaplacian=createAsRigidAsPossibleDeformer(*_win.mesh());

				mLaplacian->calcLocalCoords();
				updateControlVertices();
			}
			else
				Msg::msgBox("create control vertices first");
		}
	}
}
///////
void SelectVertexFromTetra::onLUpNoDrag(int x, int y)
{
	Ray ray;
	RE::FltkRenderer().screenToWorldRay(x,y,ray);

	std::vector<_tvector<int,3> > colTri;
	vectorn colZ;
	TetraMeshLoader::TetraMesh& m=mesh;

	for(int i=0; i<m.getNumTetra(); i++)
	{

		for(int f=0; f<4; f++)
		{
			_tvector<int,3> tetraFace=m.getTetraFace(i,f);

			std::pair<bool, m_real> res=ray.intersects(
					m.nodePos(tetraFace(0)),
					m.nodePos(tetraFace(1)),
					m.nodePos(tetraFace(2)));

			if(res.first)
			{
				colTri.push_back(tetraFace);
				colZ.pushBack(res.second);
			}
		}
	}
	clearSelection();

	if(colZ.size())
	{
		_tvector<int,3> selTri=colTri[colZ.argMin()];

		sv.mSelectedVertices.setAt(selTri(0));
		sv.mSelectedVertices.setAt(selTri(1));
		sv.mSelectedVertices.setAt(selTri(2));

		sv.drawSelection();
	}
}

TetraMeshGenerator::TetraMeshGenerator(ModelingWin& win):ModelingWin_Tool (win),
	sv(mTetraMesh, ((SelectVertex&)(*win.m_modes[ModelingWin::SELECT_VERTEX])))
{
	create("Box", "Box", "choose tool-specific mode");
	create("Box", "Box", "to select vertices");
	create("Box", "Box", "by picking a tetrahedron");

	create("Choice", "Option");
	menu(0)->size(2);
	menu(0)->item(0, "Option", 0);
	menu(0)->item(1, "Show tetra mesh", 0, Hash("Show tetra mesh"), FL_MENU_TOGGLE);
	menu(0)->item(1).set();
	menu(0)->value(0);

	create("Choice", "operations", 0, 0);
	int curop=0;
	menu(0)->size(5);
	menu(0)->item(curop++, "operations");
	menu(0)->item(curop++,"load tetra mesh",'z');
	menu(0)->item(curop++,"add a tetrahedron", 'x');
	menu(0)->item(curop++,"add tetras from vertex correspondence", 'c');
	menu(0)->item(curop++,"delete the last added tetrahedron", 'v');

	updateLayout();
	

}

void TetraMeshGenerator::redrawTetraMesh()
{
	if(mpTetraMeshToEntity)
		_win.sceneNode()->detachObject(mpTetraMeshToEntity->getLastCreatedEntity());
	mpTetraMeshToEntity.reset(new TetraMeshLoader::TetraMeshToEntity(mTetraMesh, "__tetraMesh0"));
	mpTetraMeshToEntity->createEntity("__tetraMeshEntity0", "use_vertexcolor");
	_win.sceneNode()->attachObject(mpTetraMeshToEntity->getLastCreatedEntity());
}

void TetraMeshGenerator::onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData)
{
	if(w.mId=="Option")
	{
		if(userData==Hash("Show tetra mesh"))
		{
			Fl_Choice* p=w.menu();
			if(p->menu()[p->value()].value())
			{
				if(mpTetraMeshToEntity)
					mpTetraMeshToEntity->getLastCreatedEntity()->setVisible(true);
			}
			else
			{				
				if(mpTetraMeshToEntity)
					mpTetraMeshToEntity->getLastCreatedEntity()->setVisible(false);
			}
		}
	}
	else if(w.mId=="operations")
	{
		TString op=w.menu()->text();
		if(op=="delete the last added tetrahedron")
		{
			mTetraMesh._vmesh_index.resize(mTetraMesh._vmesh_index.size()-1);
			redrawTetraMesh();
		}
		else if(op=="load tetra mesh")
		{
			TString fn=FlChooseFile("input file name to load", "../Resource/mesh/", "*.tet");
			mTetraMesh.loadMesh(fn);
			redrawTetraMesh();

		}
		else if(op=="create empty tetra mesh")
		{
			mTetraMesh._vmesh_vertex.resize(_win.numVertex());
			for(int i=0; i<_win.numVertex(); i++)
			{
				mTetraMesh._vmesh_vertex[i].pos=
					_win.mesh()->getVertex(i);
			}
		}
		else if(op=="add tetras from vertex correspondence")
		{
			VertexCorrespondence& vc=(VertexCorrespondence& )(*_win.m_modes[ModelingWin::VERTEX_CORRESPONDENCE]);

			if(vc.source.size()>0 && vc.source.size()==vc.target.size())
			{
				intvectorn sourceToTarget(_win.numVertex());

				sourceToTarget.setAllValue(-1);

				bool bOkToProceed=true;
				for(int i=0; i<vc.source.size(); i++)
				{
					if(sourceToTarget[vc.source[i]]==-1)
						sourceToTarget[vc.source[i]]=vc.target[i];
					else
						bOkToProceed=false;
				}

				if(bOkToProceed)
				{
					int numTetra=mTetraMesh.getNumTetra();
					_quadi tet, newtet;

					for(int i=0; i<numTetra; i++)
					{
						tet=mTetraMesh.getTetra(i);

						if(sourceToTarget(tet(0))!=-1 &&
							sourceToTarget(tet(1))!=-1 &&
							sourceToTarget(tet(2))!=-1 &&
							sourceToTarget(tet(3))!=-1 )
						{
							newtet(0)=sourceToTarget(tet(0));
							newtet(1)=sourceToTarget(tet(1));
							newtet(2)=sourceToTarget(tet(2));
							newtet(3)=sourceToTarget(tet(3));
							
							mTetraMesh._vmesh_index.push_back(newtet);
						}
					}

					mTetraMesh.normalizeIndex(numTetra-1);
					mTetraMesh.saveMesh("__backup.tet");
					redrawTetraMesh();
				}
				else
					Msg::msgBox("Please assign vertex correspondences again");

			}
			else
			{
				Msg::msgBox("Please assign vertex correspondences first.");
			}

		}
		else if(op=="add a tetrahedron")
		{
			SelectVertex& sv=(SelectVertex&)(*_win.m_modes[ModelingWin::SELECT_VERTEX]);

			if(sv.mSelectedVertices.count()==4)
			{
				_quadi index;

				int ii=0;
				for(int i=0; i<sv.mSelectedVertices.size(); i++)
					if(sv.mSelectedVertices[i])
					{
						index[ii++]=i;
					}
				ASSERT(ii==4);

				mTetraMesh._vmesh_index.push_back(index);
				mTetraMesh.normalizeIndex(mTetraMesh.getNumTetra()-1);

				mTetraMesh.saveMesh("__backup.tet");

				redrawTetraMesh();
			}
			else
			{
				Msg::msgBox("First select 4 vertices. (%d selected)", sv.mSelectedVertices.count());
			}
		}
	}
}
