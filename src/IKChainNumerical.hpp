#include "IKChain.hpp"
#include "IK_QSegment.hpp"

class CIKChainNumerical : public CIKChain
{
	typedef CIKChain Super;
public:
	CIKChainNumerical(CIKChain::Algor algor, int n_iters, Real tw_t, Real tw_r)
		: CIKChain(algor, n_iters)
		, c_taskW_t(tw_t)
		, c_taskW_r(tw_r)
	{
	}
	virtual ~CIKChainNumerical()
	{
		for (auto seg : m_segments)
			delete seg;
	}

	virtual bool Init(const CArtiBodyNode* eef, int len, const std::vector<CONF::CJointConf>& joint_confs) override;
	virtual bool Update() = 0;
	virtual Real Error() const;
protected:
	Real ErrorCCD() const;
	std::vector<IK_QSegment*> m_segments; //the corresponds to CIKChain::m_segments
	const Real c_taskW_r, c_taskW_t;
};