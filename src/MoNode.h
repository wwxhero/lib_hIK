#pragma once
#include <vector>
#include <map>
#include "ArtiBody.h"
#include "Logger.h"
typedef CArtiBodyNode CJoint; //CJoint is yet to be developed

class CMoNode : public TreeNode<CMoNode>
{
public:
	enum TM_TYPE { homo = 0, cross, identity };
	static const char* TM_TYPE_STR[];
private:
	struct JointPair
	{
		CJoint* j_from;
		CJoint* j_to;
		CTransform from2to;
		CTransform to2from;
	};

	inline void InitJointPair(JointPair* pair, const CArtiBodyNode* artiPair[2], TM_TYPE tm_type)
	{
		pair->j_from = (CJoint *)artiPair[0]; // todo: need a real joint
		pair->j_to = (CJoint *)artiPair[1];
		if (cross == tm_type)
		{
			CTransform from2world, world2to;
			artiPair[0]->GetTransformLocal2World(from2world);
			artiPair[1]->GetTransformWorld2Local(world2to);
			pair->from2to = world2to * from2world;
			pair->to2from = pair->from2to.inverse();
		}
		else if(homo == tm_type)
		{
			CTransform bound_from, bound_to_inv;
			artiPair[0]->GetTransformLocal2Parent(bound_from);
			artiPair[1]->GetTransformParent2Local(bound_to_inv);
			pair->from2to = bound_to_inv * bound_from;
			// pair->to2from = pair->from2to.inverse(); to2from is not used for homo transformation
		}
	}
	inline void UnInitJointPair(JointPair* pair)
	{
		pair->j_from = NULL;
		pair->j_to = NULL;
	}
public:
	CMoNode(CArtiBodyNode* body)
		: m_tmType(identity)
		, m_hostee(body)
	{

	}

	~CMoNode()
	{
		for (auto jointPair : m_jointPairs)
		{
			UnInitJointPair(jointPair);
			delete jointPair;
		}
	}

	bool MoCNN_Initialize(TM_TYPE tm_type);
	bool MoCNN_Initialize(TM_TYPE tm_type, const char* pairs[][2], int n_pairs);

	void Motion_sync()
	{
		for (JointPair* pair : m_jointPairs)
		{
			CJoint* j_from = pair->j_from;
			CJoint* j_to = pair->j_to;
			CTransform delta_from;
			j_from->GetJointTransform(delta_from);
			CTransform delta_to;
			switch (m_tmType)
			{
			 	case cross:
			 		delta_to = pair->from2to * delta_from * pair->to2from;
			 		break;
			 	case homo:
			 		delta_to = pair->from2to * delta_from;
			 		break;
			}
			j_to->SetJointTransform(delta_to);
#ifdef _DEBUG
			std::stringstream logInfo;
			logInfo << TM_TYPE_STR[m_tmType] << j_from->GetName_c() <<":" << delta_from.ToString().c_str() << "\n"
					<< "\t" << TM_TYPE_STR[m_tmType] << j_to->GetName_c() << ":" <<  delta_to.ToString().c_str() << "\n";
			g_logger.Out(logInfo.str());
#endif
		}
		CArtiBodyTree::FK_Update(m_hostee);
	}
private:
	std::vector<JointPair*> m_jointPairs;
	TM_TYPE m_tmType;
	CArtiBodyNode* m_hostee;

};

class CMoTree : public Tree<CMoNode>
{
public:
	static bool Connect(CMoNode* parent, CMoNode* child, CNN cnn_type, CMoNode::TM_TYPE tm_type, const char* pairs[][2], int n_pairs);
	static bool Connect(CMoNode* parent, CMoNode* child, CNN cnn_type, CMoNode::TM_TYPE tm_type);
	static void Motion_sync(CMoNode* root);
};