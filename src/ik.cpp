#include "pch.h"
#include "ik.h"
#include <string>
#include <sstream>
#include <Windows.h>
float IKAPI ik_test(float theta)
{
	//std::stringstream info;
	//info << theta;
	//MessageBoxA(NULL, info.str().c_str(), NULL, MB_OK);
	return ++ theta;
}