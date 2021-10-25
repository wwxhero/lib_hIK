#include "pch.h"
#include "PostureGraph.hpp"

IPostureGraph* CPostureGraphGen::LoadTransitions(const char* filePath)
{
	CFile2PostureGraphClose* pg = new CFile2PostureGraphClose();
	pg->LoadTransitions(filePath);
	return pg;
}



