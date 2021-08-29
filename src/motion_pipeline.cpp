#include "pch.h"
#include "motion_pipeline.h"
#include "MoNode.h"
#include "handle_helper.hpp"

void init_mopipe(MotionPipe* mopipe)
{
	mopipe->bodies[0] = H_INVALID;
	mopipe->bodies[1] = H_INVALID;
	mopipe->mo_nodes[0] = H_INVALID;
	mopipe->mo_nodes[1] = H_INVALID;
}

void unload_mopipe(MotionPipe* mopipe)
{
	for (int i_retar = 0; i_retar < 2; i_retar ++)
	{
		auto& moNode_i = mopipe->mo_nodes[i_retar];
		if (VALID_HANDLE(moNode_i))
			destroy_tree_motion_node(moNode_i);
		moNode_i = H_INVALID;

		auto& body_i = mopipe->bodies[i_retar];
		if (VALID_HANDLE(body_i))
			destroy_tree_body(body_i);
		body_i = H_INVALID;
	}
}

HMOTIONNODE	create_tree_motion_node(HBODY mo_src)
{
	//todo: create a tree node that represents motions of the articulated body mo_src
	CArtiBodyNode* body = CAST_2PBODY(mo_src);
	CMoNode* mo = new CMoNode(body);
	return CAST_2HMONODE(mo);
}

void destroy_tree_motion_node(HMOTIONNODE node)
{
	//todo: destroy motion node
	CMoNode* mo = CAST_2PMONODE(node);
	delete mo;
}

void destroy_tree_motion(HMOTIONNODE tree)
{
	CMoNode* mo = CAST_2PMONODE(tree);
	CMoTree::Destroy(mo);
}

bool motion_sync_cnn_homo(HMOTIONNODE from, HMOTIONNODE to, CNN type)
{
	//todo: connection to motion nodes as a homo-space map
	CMoNode* mo_from = CAST_2PMONODE(from);
	CMoNode* mo_to = CAST_2PMONODE(to);
	return CMoTree::Connect_homo(mo_from, mo_to, type);
}

bool motion_sync_cnn_cross_c(HMOTIONNODE from
							, HMOTIONNODE to
							, CNN type
							, const char* pairs[][2]
							, int n_pairs
							, const Real p2c_w[3][3])
{
	//todo: connection to motion nodes as a cross-space map
	CMoNode* mo_from = CAST_2PMONODE(from);
	CMoNode* mo_to = CAST_2PMONODE(to);
	return CMoTree::Connect_cross(mo_from, mo_to, type, pairs, n_pairs, p2c_w);
}

bool motion_sync_cnn_cross_w(HMOTIONNODE from
							, HMOTIONNODE to
							, CNN type
							, const wchar_t* pairs[][2]
							, int n_pairs
							, const Real p2c_w[3][3])
{
	//todo: connection to motion nodes as a cross-space map
	CMoNode* mo_from = CAST_2PMONODE(from);
	CMoNode* mo_to = CAST_2PMONODE(to);
	return CMoTree::Connect_cross(mo_from, mo_to, type, pairs, n_pairs, p2c_w);
}

void motion_sync(HMOTIONNODE root)
{
	//todo: sync motions from tree root to leaf
	CMoNode* mo_root = CAST_2PMONODE(root);
	CMoTree::Motion_sync(mo_root);
}

HMOTIONNODE	get_first_child_mo_node(HMOTIONNODE node)
{
	CMoNode* mo_node = CAST_2PMONODE(node);
	return CAST_2HMONODE(mo_node->GetFirstChild());
}

HMOTIONNODE	get_next_sibling_mo_node(HMOTIONNODE node)
{
	CMoNode* mo_node = CAST_2PMONODE(node);
	return CAST_2HMONODE(mo_node->GetNextSibling());
}
