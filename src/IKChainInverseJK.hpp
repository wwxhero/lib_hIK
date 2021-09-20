#include "IKChain.hpp"
#include "IK_QJacobian.h"
#include "IK_QSegment.hpp"

template<typename IK_QJacobianX>
class CIKChainInverseJK : public CIKChain
{
public:
	CIKChainInverseJK(Real weight_p, Real weight_r, int n_iter)
		: CIKChain(IK_QJacobianX::s_Algor, n_iter)
	{
	}

	virtual bool Init(const CArtiBodyNode* eef, int len, const std::vector<CONF::CJointConf>& joint_confs) override
	{
		bool initialized = CIKChain::Init(eef, len, joint_confs);
		return initialized;
	}

	virtual void Dump(std::stringstream& info) const override
	{
		info << from_Algor(c_algor) << " : ";
		CIKChain::Dump(info);
	}
	// virtual void UpdateNext(int step) override;
	// this is a quick IK update solution
	// virtual void UpdateAll() override;

private:
	IK_QJacobianX m_jacobian;
	IK_QJacobianX m_jacobian_sub;

	bool m_secondary_enabled;

	std::vector<IK_QSegment*> m_segments; //the corresponds to CIKChain::m_segments

};

typedef CIKChainInverseJK<IK_QJacobianDLS> CIKChainInverseJK_DLS;
typedef CIKChainInverseJK<IK_QJacobianSDLS> CIKChainInverseJK_SDLS;