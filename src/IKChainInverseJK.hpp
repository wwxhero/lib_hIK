#include "IKChain.hpp"
#include "IK_QJacobian.h"

template<typename IK_QJacobianX>
class CIKChainInverseJK : public CIKChain
{
public:
	CIKChainInverseJK()
		: CIKChain(IK_QJacobianX::s_Algor)
	{
	}
};

typedef CIKChainInverseJK<IK_QJacobianDLS> CIKChainInverseJK_DLS;
typedef CIKChainInverseJK<IK_QJacobianSDLS> CIKChainInverseJK_SDLS;