#include "pch.h"
#include "ErrorTB.hpp"

class NullErrorTB : public IErrorTB
{

};


IErrorTB* IErrorTB::Factory::CreateHOMO(int n_theta)
{
	return NULL;
}