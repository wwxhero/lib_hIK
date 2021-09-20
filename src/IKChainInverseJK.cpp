#include "pch.h"
#include "IKChainInverseJK.hpp"

CIKChainInverseJK_DLS::CIKChainInverseJK_DLS(Real weight_p, Real weight_r, int n_iter)
	: CIKChainInverseJK<IK_QJacobianDLS>(CIKChain::DLS, weight_p, weight_r, n_iter)
{

}

CIKChainInverseJK_DLS::~CIKChainInverseJK_DLS()
{

}


CIKChainInverseJK_SDLS::CIKChainInverseJK_SDLS(Real weight_p, Real weight_r, int n_iter)
	: CIKChainInverseJK<IK_QJacobianSDLS>(CIKChain::SDLS, weight_p, weight_r, n_iter)
{

}

CIKChainInverseJK_SDLS::~CIKChainInverseJK_SDLS()
{

}