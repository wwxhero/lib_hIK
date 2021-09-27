#pragma once
#include "Transform.hpp"
#include "articulated_body.h"
#include "ik_logger.h"

class CArtiBodyNode;

class IJoint
{
public:
	virtual const char* GetName_c() const = 0;
	virtual const wchar_t* GetName_w() const = 0;
	virtual const Transform* GetTransformLocal2World() const = 0;
	virtual const Transform* GetTransform() const = 0;
	virtual Transform* GetTransform() = 0;
	virtual void SetLinear(const Eigen::Matrix3r& rotm) = 0;
	virtual void SetRotation(const Eigen::Quaternionr& rotq) = 0;
	virtual void SetRotation_w(const Eigen::Quaternionr& rotq) = 0;
	virtual void SetTranslation(const Eigen::Vector3r& tt) = 0;
};

template<typename TTransform>
class IJointImpl : public IJoint
{
	template<typename TJoint> friend class TArtiBodyNode_sim;
	friend class CArtiBodyNode_anim;
public:
	IJointImpl(CArtiBodyNode* host)
		: m_host(host)
	{
	}

	virtual const char* GetName_c() const
	{
		return m_host->GetName_c();
	}

	virtual const wchar_t* GetName_w() const
	{
		return m_host->GetName_w();
	}

	virtual const Transform* GetTransformLocal2World() const
	{
		return m_host->GetTransformLocal2World();
	}

	virtual const Transform* GetTransform() const
	{
		return &m_tm;
	}

	virtual Transform* GetTransform()
	{
		return &m_tm;
	}

	virtual void SetLinear(const Eigen::Matrix3r& rotm)
	{
		m_tm.setLinear(rotm);
	}

	virtual void SetRotation(const Eigen::Quaternionr& rotq)
	{
		m_tm.setRotation(rotq);
	}

	virtual void SetRotation_w(const Eigen::Quaternionr& rotq)
	{
		CArtiBodyNode* parent = m_host->GetParent();
		if (NULL == parent)
		{
			m_tm.setRotation(rotq);
		}
		else
		{
			Eigen::Quaternionr rot_0_p2l = Transform::getRotation_q(m_host->GetTransformParent2Local0());
			Eigen::Quaternionr rot_w2p = Transform::getRotation_q(parent->GetTransformWorld2Local());
			Eigen::Quaternionr rotq_l = rot_0_p2l * rot_w2p * rotq;
			m_tm.setRotation(rotq_l);
		}


	}

	virtual void SetTranslation(const Eigen::Vector3r& tt)
	{
		m_tm.setTranslation(tt);
	}

private:
	TTransform m_tm;
	CArtiBodyNode* m_host;
};

typedef IJointImpl<Transform_TRS> CJointAnim;	//t_trs
typedef IJointImpl<Transform_TR> CJointSim_tr;	//t_tr
typedef IJointImpl<Transform_R> CJointSim_r;	//t_r


