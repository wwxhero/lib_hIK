#pragma once
#include "posture_graph.h"

class IErrorTB
{
public:
	class Factory
	{
	public:
		static IErrorTB* CreateHOMO(int n_theta);
	};
	virtual void Set(int i_theta, int j_theta, Real err_ij) = 0;
	virtual Real Get(int i_theta, int j_theta) const = 0;
	virtual int N_Theta() const = 0;
	virtual void CopyTo(_ERROR_TB* etb) = 0;
};