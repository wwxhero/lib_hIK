#include "pch.h"
#include "motion_pipeline.h"


HMOTIONNODE	create_tree_motion_node(HBODY mo_src)
{
	//todo: create a tree node that represents motions that could read from articulated body mo_src
	return H_INVALID;
}

bool motion_sync_cnn_homo(HMOTIONNODE parent, HMOTIONNODE child)
{
	//todo: connection to motion nodes as a homo-space map
	return false;
}

bool motion_sync_cnn_cross(HMOTIONNODE parent, HMOTIONNODE child, const char* pairs[][2], int n_pairs)
{
	//todo: connection to motion nodes as a cross-space map
	return false;
}

void motion_sync(HMOTIONNODE root)
{
	//todo: sync motions from tree root to leaf
}

void destroy_tree_motion_node(HMOTIONNODE node)
{
	//todo: destroy motion node
}

HMOTIONNODE	get_first_child_mo_node(HMOTIONNODE node)
{
	return H_INVALID;
}

HMOTIONNODE	get_next_sibling_mo_node(HMOTIONNODE node)
{
	return H_INVALID;
}