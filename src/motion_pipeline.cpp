#include "pch.h"
#include "motion_pipeline.h"
#include "MoNode.h"
#include "handle_helper.hpp"

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

bool motion_sync_cnn_homo(HMOTIONNODE parent, HMOTIONNODE child, CNN type)
{
	//todo: connection to motion nodes as a homo-space map
	CMoNode* mo_parent = CAST_2PMONODE(parent);
	CMoNode* mo_child = CAST_2PMONODE(child);
	return CMoTree::Connect(mo_parent, mo_child, type, CMoNode::homo);
}

bool motion_sync_cnn_cross(HMOTIONNODE parent, HMOTIONNODE child, CNN type, const char* pairs[][2], int n_pairs)
{
	//todo: connection to motion nodes as a cross-space map
	CMoNode* mo_parent = CAST_2PMONODE(parent);
	CMoNode* mo_child = CAST_2PMONODE(child);
	return CMoTree::Connect(mo_parent, mo_child, type, CMoNode::cross, pairs, n_pairs);
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