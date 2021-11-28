#pragma once
#include <list>
#include "posture_graph.h"

class CPGTheta;

class IErrorTB
{
public:
	class Factory
	{
	public:
		static IErrorTB* CreateHOMO(const CPGTheta& theta, const std::list<std::string>& joints);
		static IErrorTB* CreateX(const CPGTheta& theta, const std::list<std::string>& joints, int n_theta_0, int n_theta_1);
		static void Release(IErrorTB* etb);
	};
	virtual void Set(int i_theta, int j_theta, Real err_ij) = 0;
	virtual Real Get(int i_theta, int j_theta) const = 0;
	virtual int N_Theta() const = 0;
	virtual void Alloc(_ERROR_TB* etb) = 0;
	static void Free(_ERROR_TB* etb);
};