#include "IKChain.hpp"
#include "IK_QSegment.hpp"

class CIKChainNumerical : public CIKChain
{
	typedef CIKChain Super;
public:
	CIKChainNumerical(CIKChain::Algor algor, int n_iters)
		: CIKChain(algor, n_iters)
	{
	}
	virtual ~CIKChainNumerical()
	{
		for (auto seg : m_segments)
			delete seg;
	}

	virtual bool Init(const CArtiBodyNode* eef, int len, const std::vector<CONF::CJointConf>& joint_confs) override;
protected:
	std::vector<IK_QSegment*> m_segments; //the corresponds to CIKChain::m_segments
};