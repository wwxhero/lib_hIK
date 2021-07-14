#pragma once
#include <string>
#include <list>
#include "TreeBase.h"
#include "Joint.h"
#include "Transform.h"

class CArtiBodyNode : public TreeNode<CArtiBodyNode>
{
	friend class CArtiBodyTree;
	friend class CMoNode;		//for accessing m_kinalst
public:
	CArtiBodyNode(const wchar_t *name, NODETYPE type, TM_TYPE jtmflag);
	CArtiBodyNode(const char *name, NODETYPE type, TM_TYPE jtmflag);
	virtual ~CArtiBodyNode();

	const wchar_t* GetName_w() const
	{
		return m_namew.c_str();
	}

	const char* GetName_c() const
	{
		return m_namec.c_str();
	}

	virtual const Transform* GetTransformLocal2Parent() const = 0;
	virtual const Transform* GetTransformParent2Local() const = 0;
	virtual const Transform* GetTransformLocal2World() const = 0;
	virtual const Transform* GetTransformWorld2Local() const = 0;
	virtual IJoint* GetJoint() = 0;
	virtual const IJoint* GetJoint() const = 0;

protected:
	std::list<CArtiBodyNode*> m_kinalst;
private:
	std::string m_namec;
	std::wstring m_namew;
public:
	const NODETYPE c_type;
	const TM_TYPE c_jtmflag;
};

template<typename TTransform, typename TJoint>
class TArtiBodyNode : public CArtiBodyNode
{
	template<typename TJoint> friend class TArtiBodyNode_sim;
public:
	// fixme: jtm (joint transformation type) is a parameter for a joint type
	//		Joint<jtm> would create the joint class during compiling time
	TArtiBodyNode(const wchar_t *name, const _TRANSFORM* tm_rest_l2p, NODETYPE type, TM_TYPE jtmflag)
		: CArtiBodyNode(name, type, jtmflag)
		, m_local2parent0(*tm_rest_l2p)
		, m_joint(this)
	{
	}

	TArtiBodyNode(const char *name, const _TRANSFORM* tm_rest_l2p, NODETYPE type, TM_TYPE jtmflag)
		: CArtiBodyNode(name, type, jtmflag)
		, m_local2parent0(*tm_rest_l2p)
		, m_joint(this)
	{
	}

	virtual ~TArtiBodyNode()
	{
	}

	virtual const Transform* GetTransformLocal2Parent() const
	{
		return &m_local2parent_cached;
	}

	virtual const Transform* GetTransformParent2Local() const
	{
		return &m_parent2local_cached;
	}

	virtual const Transform* GetTransformLocal2World() const
	{
		return &m_local2world_cached;
	}

	virtual const Transform* GetTransformWorld2Local() const
	{
		return &m_world2local_cached;
	}

	virtual IJoint* GetJoint()
	{
		return &m_joint;
	}

	virtual const IJoint* GetJoint() const
	{
		return &m_joint;
	}

protected:
	TTransform m_local2parent0;
	TJoint m_joint;

protected:
	TTransform m_local2parent_cached;
	TTransform m_parent2local_cached;
	TTransform m_local2world_cached;
	TTransform m_world2local_cached;
};

class CArtiBodyNode_anim : public TArtiBodyNode<Transform_TRS, CJointAnim>
{
	friend class CArtiBodyTree;
	typedef TArtiBodyNode<Transform_TRS, CJointAnim> Super;
public:
	CArtiBodyNode_anim(const wchar_t *name, const _TRANSFORM* tm)
		: Super(name, tm, anim, t_trs)
	{
	}
	CArtiBodyNode_anim(const char *name, const _TRANSFORM* tm)
		: Super(name, tm, anim, t_trs)
	{
	}
	virtual ~CArtiBodyNode_anim()
	{
	}
private:
	inline void FK_UpdateNode()
	{
		m_local2parent_cached = m_local2parent0 * m_joint.m_tm;
		m_parent2local_cached = m_local2parent_cached.inverse();

		bool is_root = (NULL == m_parent);

		if (is_root)
		{
			m_local2world_cached = m_local2parent_cached;
			m_world2local_cached = m_parent2local_cached;
		}
		else
		{
			m_local2world_cached = static_cast<CArtiBodyNode_anim*>(m_parent)->m_local2world_cached * m_local2parent_cached;
			m_world2local_cached = m_parent2local_cached * static_cast<CArtiBodyNode_anim*>(m_parent)->m_world2local_cached;
		}

	}



};

typedef TArtiBodyNode_sim<CJointSim_tr> CArtiBodyNode_sim_tr;
typedef TArtiBodyNode_sim<CJointSim_r> CArtiBodyNode_sim_r;

template<typename TJoint>
class TArtiBodyNode_sim : public TArtiBodyNode<Transform_TR, TJoint>
{
	friend class CArtiBodyTree;
	typedef TArtiBodyNode<Transform_TR, TJoint> Super;
public:
	TArtiBodyNode_sim(const wchar_t *name, const _TRANSFORM* tm, TM_TYPE jtmflag)
		: Super(name, tm, sim, jtmflag)
	{
	}
	TArtiBodyNode_sim(const char *name, const _TRANSFORM* tm, TM_TYPE jtmflag)
		: Super(name, tm, sim, jtmflag)
	{
	}
	virtual ~TArtiBodyNode_sim()
	{
	}
private:

	inline void FK_UpdateNode()
	{
		// this function is performance sensitive
		this->m_local2parent_cached.Update(this->m_local2parent0, this->m_joint.m_tm);
		this->m_parent2local_cached = this->m_local2parent_cached.inverse();

		bool is_root = (NULL == this->m_parent);
		if (is_root)
		{
			this->m_local2world_cached = this->m_local2parent_cached;
			this->m_world2local_cached = this->m_parent2local_cached;
		}
		else
		{
			// this is ugly, but in perspective of performance, it is better than using virtual functions
			Transform_TR* world2parent = NULL;
			Transform_TR* parent2world = NULL;
			switch (this->m_parent->c_jtmflag)
			{
				case t_r:
				{
					CArtiBodyNode_sim_r* parent = static_cast<CArtiBodyNode_sim_r*>(this->m_parent);
					world2parent = &(parent->m_world2local_cached);
					parent2world = &(parent->m_local2world_cached);
					break;
				}
				case t_tr:
				{
					CArtiBodyNode_sim_tr* parent = static_cast<CArtiBodyNode_sim_tr*>(this->m_parent);
					world2parent = &(parent->m_world2local_cached);
					parent2world = &(parent->m_local2world_cached);
					break;
				}
			}
			this->m_local2world_cached.Update(*parent2world, this->m_local2parent_cached);
			this->m_world2local_cached.Update(this->m_parent2local_cached, *world2parent);
		}
	}
};

class CArtiBodyTree : public Tree<CArtiBodyNode>
{
	typedef Tree<CArtiBodyNode> Super;
private:
	template<typename TName>
	static CArtiBodyNode* CreateAnimNodeInternal(const TName* name, const _TRANSFORM* tm)
	{
		CArtiBodyNode* ret = new CArtiBodyNode_anim(name, tm);
		return ret;
	}

	template<typename TName>
	static CArtiBodyNode* CreateSimNodeInternal(const TName* name, const _TRANSFORM* tm, TM_TYPE jtm)
	{
		CArtiBodyNode* ret = NULL;
		switch (jtm)
		{
			case t_r:
				ret = new CArtiBodyNode_sim_r(name, tm, jtm);
				break;
			case t_tr:
				ret = new CArtiBodyNode_sim_tr(name, tm, jtm);
				break;
		}
		assert(NULL != ret);
		return ret;
	}

public:
	static CArtiBodyNode* CreateAnimNode(const wchar_t* name, const _TRANSFORM* tm);
	static CArtiBodyNode* CreateAnimNode(const char* name, const _TRANSFORM* tm);
	static CArtiBodyNode* CreateSimNode(const wchar_t* name, const _TRANSFORM* tm, TM_TYPE jtm);
	static CArtiBodyNode* CreateSimNode(const char* name, const _TRANSFORM* tm, TM_TYPE jtm);
	static void KINA_Initialize(CArtiBodyNode* root);
	static void FK_Update(CArtiBodyNode* root);
	static void Destroy(CArtiBodyNode* root);
#ifdef _DEBUG
	static void Connect(CArtiBodyNode* from, CArtiBodyNode* to, CNN type);
#endif
};