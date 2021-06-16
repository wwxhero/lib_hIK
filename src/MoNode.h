#pragma once
#include "ArtiBody.h"
class CMoNode
{
public:
	CMoNode(CArtiBodyNode* body)
	{

	}

	~CMoNode()
	{

	}

	CMoNode* GetFirstChild() const
	{
		return NULL;
	}

	CMoNode* GetNextSibling() const
	{
		return NULL;
	}
};

class CMoTree
{
public:
	enum EDGE { homo = 0, cross };
	static bool Connect(CMoNode* parent, CMoNode* child, CNN cnn_type, EDGE e_type, const char* pairs[][2], int n_pairs);
	static void Motion_sync(CMoNode* root);
};