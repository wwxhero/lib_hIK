#include "pch.h"
#include "IKChainInverseJK.hpp"

CIKChainInverseJK_DLS::CIKChainInverseJK_DLS(Real weight_p, Real weight_r, int n_iter, Real tol_p, Real tol_r)
	: IKChainInverseJK<IK_QJacobianDLS>(CIKChain::DLS, weight_p, weight_r, n_iter, tol_p, tol_r)
{

}

CIKChainInverseJK_DLS::~CIKChainInverseJK_DLS()
{

}


CIKChainInverseJK_SDLS::CIKChainInverseJK_SDLS(Real weight_p, Real weight_r, int n_iter, Real tol_p, Real tol_r)
	: IKChainInverseJK<IK_QJacobianSDLS>(CIKChain::SDLS, weight_p, weight_r, n_iter, tol_p, tol_r)
{

}

CIKChainInverseJK_SDLS::~CIKChainInverseJK_SDLS()
{

}