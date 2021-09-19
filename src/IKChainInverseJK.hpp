#include "IKChain.hpp"
#include "IK_QJacobian.h"

template<typename IK_QJacobianX>
class CIKChainInverseJK : public CIKChain
{
public:
	CIKChainInverseJK(Real weight_p, Real weight_r, int n_iter)
		: CIKChain(IK_QJacobianX::s_Algor, n_iter)
	{
	}

	// virtual bool Init(const CArtiBodyNode* eef, int len) override;
	virtual void Dump(std::stringstream& info) const override
	{
		info << from_Algor(c_algor);
		CIKChain::Dump(info);
	}
	// virtual void UpdateNext(int step) override;
	// this is a quick IK update solution
	// virtual void UpdateAll() override;

};

typedef CIKChainInverseJK<IK_QJacobianDLS> CIKChainInverseJK_DLS;
typedef CIKChainInverseJK<IK_QJacobianSDLS> CIKChainInverseJK_SDLS;