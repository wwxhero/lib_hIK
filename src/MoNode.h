#pragma once
#include "ArtiBody.h"
class CMoNode : public TreeNode<CMoNode>
{
public:
	enum TM_TYPE { homo = 0, cross, identity };
	CMoNode(CArtiBodyNode* body)
	{

	}

	~CMoNode()
	{

	}

	bool MoCNN_Initialize(TM_TYPE tm_type, const char* pairs[][2], int n_pairs)
	{
		return false;
	}

	void Motion_sync()
	{
		// for (EdgePtr pipe : n_this->out())
		// {
		// 	assert(pipe->first == n_this);
		// 	for (JointPair* pair : pipe->Map())
		// 	{
		// 		// Joint* j_from = pair->first;
		// 		// Joint* j_to = pair->second;
		// 		// const Transform& delta_from = j_from->GetTransform();
		// 		// Transform delta_to;
		// 		// switch (pipe->type)
		// 		// {
		// 		// 	case EDGE::CROSS:
		// 		// 		delta_to = pair->from2to() * delta_from * pair->to2from();
		// 		// 		break;
		// 		// 	case EDGE::HOMO:
		// 		// 		delta_to = pair->from2to() * delta_from;
		// 		// 		break;
		// 		// }
		// 		// j_to->SetTransform(delta_to);
		// 		pair->Sync(pipe->type);
		// 	}
		// 	CArtiBodyTree::FK_Update(pipe->second->Host());
		// }
	}

};

class CMoTree : public Tree<CMoNode>
{
public:
	static bool Connect(CMoNode* parent, CMoNode* child, CNN cnn_type, CMoNode::TM_TYPE tm_type, const char* pairs[][2], int n_pairs);
	static void Motion_sync(CMoNode* root);
};