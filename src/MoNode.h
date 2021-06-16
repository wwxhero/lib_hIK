#pragma once
#include "ArtiBody.h"
class CMoNode
{
public:
	enum CNN_TYPE {homo, cross};
	CMoNode(CArtiBody* body)
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

	static bool Connect(CMoNode* parent, CMoNode* child, CNN_TYPE type, const char* pairs[][2], int n_pairs);
	static void Motion_sync(CMoNode* root);
};