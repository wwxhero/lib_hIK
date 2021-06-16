#include "pch.h"
#include "MoNode.h"

bool CMoTree::Connect(CMoNode* parent, CMoNode* child, CNN cnn_type, CMoNode::TM_TYPE tm_type, const char* pairs[][2], int n_pairs)
{
	Tree<CMoNode>::Connect(parent, child, cnn_type);
	return child->MoCNN_Initialize(tm_type, pairs, n_pairs);
}

void CMoTree::Motion_sync(CMoNode* root)
{
	auto onEnterNode = [] (CMoNode* n_this)
					{
						for (CMoNode* n_child = n_this->GetFirstChild()
							; NULL != n_child
							; n_child = n_child->GetNextSibling())
						{
							n_child->Motion_sync();
						}

					};
	auto onLeaveNode = [] (CMoNode* n_this)
					{

					};
	TraverseDFS_botree_nonrecur(root, onEnterNode, onLeaveNode);
}