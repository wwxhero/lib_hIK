#pragma once
#include <string>
#include <list>
#include "TreeBase.hpp"
#include "Joint.hpp"
#include "Transform.hpp"
#include "ik_logger.h"

class CArtiBodyNode : public TreeNode<CArtiBodyNode>
{
	friend class CArtiBodyTree;
	friend class CMoNode;		//for accessing m_kinalst
public:
	CArtiBodyNode(const wchar_t *name, BODY_TYPE type, TM_TYPE jtmflag);
	CArtiBodyNode(const char *name, BODY_TYPE type, TM_TYPE jtmflag);
	virtual ~CArtiBodyNode();

	const wchar_t* GetName_w() const
	{
		return m_namew.c_str();
	}

	const char* GetName_c() const
	{
		return m_namec.c_str();
	}

	bool UpdateGoal(const Transform_TRS& tm_w)
	{
		_TRANSFORM goal_w;
		tm_w.CopyTo(goal_w);
		return UpdateGoal(goal_w);
	}

	bool UpdateGoal(const _TRANSFORM& tm_w)
	{
		const _TRANSFORM& tm_w_current_tm = m_goal;
		bool update = !Equal(tm_w, tm_w_current_tm);
		if (update)
			m_goal = tm_w;
		return update;
	}

	void GetGoal(_TRANSFORM& tm_w) const
	{
		tm_w = m_goal;
	}

	virtual void OnKINA_Initialize() = 0;
	virtual const Transform* GetTransformLocal2Parent0() const = 0;
	virtual const Transform* GetTransformParent2Local0() const = 0;
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
	_TRANSFORM m_goal;
public:
	const BODY_TYPE c_type;
	const TM_TYPE c_jtmflag;
};

template<typename TTransform, typename TJoint>
class TArtiBodyNode : public CArtiBodyNode
{
	template<typename TJoint> friend class TArtiBodyNode_sim;
public:
	// fixme: jtm (joint transformation type) is a parameter for a joint type
	//		Joint<jtm> would create the joint class during compiling time
	TArtiBodyNode(const wchar_t *name, const _TRANSFORM* tm_rest_l2p, BODY_TYPE type, TM_TYPE jtmflag)
		: CArtiBodyNode(name, type, jtmflag)
		, m_local2parent0(*tm_rest_l2p)
		, m_parent2local0(m_local2parent0.inverse())
		, m_joint(this)
	{
	}

	TArtiBodyNode(const char *name, const _TRANSFORM* tm_rest_l2p, BODY_TYPE type, TM_TYPE jtmflag)
		: CArtiBodyNode(name, type, jtmflag)
		, m_local2parent0(*tm_rest_l2p)
		, m_parent2local0(m_local2parent0.inverse())
		, m_joint(this)
	{
	}

	virtual ~TArtiBodyNode()
	{
	}

	virtual void OnKINA_Initialize()
	{
		//for the artibody node created with a local binding transform, it does nothing
	}

	virtual const Transform* GetTransformLocal2Parent0() const final
	{
		return &m_local2parent0;
	}

	virtual const Transform* GetTransformParent2Local0() const final
	{
		return &m_parent2local0;
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
	const TTransform m_local2parent0;
	const TTransform m_parent2local0;
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
		: Super(name, tm, fbx, t_trs)
	{
	}
	CArtiBodyNode_anim(const char *name, const _TRANSFORM* tm)
		: Super(name, tm, fbx, t_trs)
	{
	}
	virtual ~CArtiBodyNode_anim()
	{
	}
private:
	template<bool G_ROOT>
	inline void FK_UpdateNode()
	{
		m_local2parent_cached = m_local2parent0 * m_joint.m_tm;
		m_parent2local_cached = m_local2parent_cached.inverse();
		CArtiBodyNode_anim* parent = static_cast<CArtiBodyNode_anim*>(GetParent());
		bool is_root = (G_ROOT || NULL == parent);

		if (is_root)
		{
			m_local2world_cached = m_local2parent_cached;
			m_world2local_cached = m_parent2local_cached;
		}
		else
		{
			m_local2world_cached = parent->m_local2world_cached * m_local2parent_cached;
			m_world2local_cached = m_parent2local_cached * parent->m_world2local_cached;
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
	typedef Transform_TR Transform_Type;
public:
	TArtiBodyNode_sim(const wchar_t *name, const _TRANSFORM* tm, BODY_TYPE type, TM_TYPE jtmflag)
		: Super(name, tm, type, jtmflag)
	{
	}
	TArtiBodyNode_sim(const char *name, const _TRANSFORM* tm, BODY_TYPE type, TM_TYPE jtmflag)
		: Super(name, tm, type, jtmflag)
	{
	}
	virtual ~TArtiBodyNode_sim()
	{
	}
private:
	template<bool G_ROOT>
	inline void FK_UpdateNode()
	{
		// this function is performance sensitive
		this->m_local2parent_cached.Update(this->m_local2parent0, this->m_joint.m_tm);
		this->m_parent2local_cached = this->m_local2parent_cached.inverse();
		CArtiBodyNode* parent0 = this->GetParent();
		bool is_root = (G_ROOT || NULL == parent0);
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
			switch (parent0->c_jtmflag)
			{
				case t_r:
				{
					CArtiBodyNode_sim_r* parent = static_cast<CArtiBodyNode_sim_r*>(parent0);
					world2parent = &(parent->m_world2local_cached);
					parent2world = &(parent->m_local2world_cached);
					break;
				}
				case t_tr:
				{
					CArtiBodyNode_sim_tr* parent = static_cast<CArtiBodyNode_sim_tr*>(parent0);
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



template<typename ArtiBodyNode, typename TTransform>
class TArtiBodyNode_world : public ArtiBodyNode
{
	typedef ArtiBodyNode Super;
	typedef TArtiBodyNode_world<ArtiBodyNode, TTransform> This;
public:
	TArtiBodyNode_world(const char* name_c, const _TRANSFORM* tm, BODY_TYPE type, TM_TYPE jtm)
		: Super(name_c, tm, type, jtm)
		, m_local2world0(*tm)
		, m_world2local0(m_local2world0.inverse())
	{
		assert(m_local2world0.Valid()
			&& m_world2local0.Valid());
	}
	TArtiBodyNode_world(const wchar_t* name_w, const _TRANSFORM* tm, BODY_TYPE type, TM_TYPE jtm)
		: Super(name_w, tm, type, jtm)
		, m_local2world0(*tm)
		, m_world2local0(m_local2world0.inverse())
	{
		assert(m_local2world0.Valid()
			&& m_world2local0.Valid());
	}
// #ifdef _DEBUG
//  	virtual const Transform* GetTransformLocal2World() const override
//  	{
//  		return &m_local2world0;
//  	}

//  	virtual const Transform* GetTransformWorld2Local() const override
//  	{
//  		return &m_world2local0;
//  	}
// #endif
	virtual void OnKINA_Initialize() override
	{
		CArtiBodyNode* parent = this->GetParent();
		bool is_root = (NULL == parent);
		TTransform* local2parent0 = const_cast<TTransform*>(&this->m_local2parent0);
		TTransform* parent2local0 = const_cast<TTransform*>(&this->m_parent2local0);
		assert(m_local2world0.Valid()
			&& m_world2local0.Valid());
		if (is_root)
		{
			_TRANSFORM tm_l2p;
			m_local2world0.CopyTo(tm_l2p);
			// tm_l2p.tt.x = tm_l2p.tt.y = tm_l2p.tt.z = 0;
			local2parent0->CopyFrom(tm_l2p);

			_TRANSFORM tm_p2l;
			m_world2local0.CopyTo(tm_p2l);
			// tm_p2l.tt.x = tm_p2l.tt.y = tm_p2l.tt.z = 0;
			parent2local0->CopyFrom(tm_p2l);
		}
		else
		{
			const TTransform* local2world0_m = static_cast<const TTransform*>(parent->GetTransformLocal2Parent0());
			const TTransform* world2local0_m = static_cast<const TTransform*>(parent->GetTransformParent2Local0());
			// loca2world0_m * local2parent0 = local2world0
			//	=> local2parent0 = (local2world0_m^-1) * local2world0
			assert(local2world0_m->Valid()
				&& world2local0_m->Valid());
			local2parent0->Update(*world2local0_m, m_local2world0);
			parent2local0->Update(m_world2local0, *local2world0_m);
		}
	}
public:
	const TTransform m_local2world0;
	const TTransform m_world2local0;
};

typedef TArtiBodyNode_world<CArtiBodyNode_sim_tr, CArtiBodyNode_sim_tr::Transform_Type> CArtiBodyNode_sim_tr_world;
typedef TArtiBodyNode_world<CArtiBodyNode_sim_r, CArtiBodyNode_sim_r::Transform_Type> CArtiBodyNode_sim_r_world;

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
	static CArtiBodyNode* CreateSimNodeInternal(const TName* name, const _TRANSFORM* tm, BODY_TYPE type, TM_TYPE jtm, bool local = true)
	{
		CArtiBodyNode* ret = NULL;
		switch (jtm)
		{
			case t_r:
				if (local)
					ret = new CArtiBodyNode_sim_r(name, tm, type, jtm);
				else
					ret = new CArtiBodyNode_sim_r_world(name, tm, type, jtm);
				break;
			case t_tr:
				if (local)
					ret = new CArtiBodyNode_sim_tr(name, tm, type, jtm);
				else
					ret = new CArtiBodyNode_sim_tr_world(name, tm, type, jtm);
				break;
		}
		assert(NULL != ret);
		return ret;
	}

public:
	static bool CloneNode_fbx(const CArtiBodyNode* src, CArtiBodyNode** dst, const wchar_t* name_dst_opt = NULL);
	static bool CloneNode_bvh(const CArtiBodyNode* src, CArtiBodyNode** dst, const wchar_t* name_dst_opt = NULL);
	static bool CloneNode_htr(const CArtiBodyNode* src, CArtiBodyNode** dst, const Eigen::Matrix3r& src2dst_w, const wchar_t* name_dst_opt = NULL);

	static CArtiBodyNode* CreateAnimNode(const wchar_t* name, const _TRANSFORM* tm);
	static CArtiBodyNode* CreateAnimNode(const char* name, const _TRANSFORM* tm);
	static CArtiBodyNode* CreateSimNode(const wchar_t* name, const _TRANSFORM* tm, BODY_TYPE type, TM_TYPE jtm, bool local = true);
	static CArtiBodyNode* CreateSimNode(const char* name, const _TRANSFORM* tm, BODY_TYPE type, TM_TYPE jtm, bool local = true);

	static bool Clone_htr(const CArtiBodyNode* src, CArtiBodyNode** dst, const wchar_t* (*matches)[2], int n_matches, bool src_on_match0, const Eigen::Matrix3r& src2dst_w);

	template<typename CloneNode_x>
	static bool Clone(const CArtiBodyNode* src, CArtiBodyNode** dst, CloneNode_x CloneNode)
	{
		auto ConstructNode = [CloneNode](const CArtiBodyNode* src, CArtiBodyNode** dst) -> bool
		{
			bool ret = CloneNode(src, dst, NULL);
			return ret;
		};

		bool cloned = ConstructBFS(src, dst, ConstructNode);

		if (cloned)
		{
			KINA_Initialize(*dst);
			FK_Update<false>(*dst);
		}
		else
		{
			IKAssert(NULL == *dst);
		}

		return cloned;

	}

	static bool Clone(const CArtiBodyNode* src, CArtiBodyNode** dst)
	{
		switch(src->c_type)
		{
			case BODY_TYPE::fbx:
				return Clone(src, dst, CloneNode_fbx);
			case BODY_TYPE::bvh:
				return Clone(src, dst, CloneNode_bvh);
			case BODY_TYPE::htr:
			{
				auto CloneNode = [](const CArtiBodyNode* src, CArtiBodyNode** dst, const wchar_t* name_dst_opt) -> bool
				{
					return CArtiBodyTree::CloneNode_htr(src, dst, Eigen::Matrix3r::Identity(), name_dst_opt);
				};
				return Clone(src, dst, CloneNode);
			}
			default:
				IKAssert(0);
				*dst = NULL;
				return false;
		}
	}

	static void KINA_Initialize(CArtiBodyNode* root);

	template<bool G_SPACE>
	static void FK_Update(CArtiBodyNode* root)
	{
START_PROFILER_AUTOFRAME(root->GetName_c(), 1)
		bool is_an_anim = (root->c_type&anim);
		if (is_an_anim)
		{
			auto it = root->m_kinalst.begin();
			auto it_end = root->m_kinalst.end();
			if (it != it_end)
			{
				const bool G_ROOT = G_SPACE;
				static_cast<CArtiBodyNode_anim*>(*it)->FK_UpdateNode<G_ROOT>();
				for (it ++; it != it_end; it ++)
					static_cast<CArtiBodyNode_anim*>(*it)->FK_UpdateNode<false>();
			}
		}
		else // not an anim
		{
			auto it = root->m_kinalst.begin();
			auto it_end = root->m_kinalst.end();
			if (it != it_end)
			{
				const bool G_ROOT = G_SPACE;
				switch ((*it)->c_jtmflag)
				{
					case t_r:
						static_cast<CArtiBodyNode_sim_r*>(*it)->FK_UpdateNode<G_ROOT>();
						break;
					case t_tr:
						static_cast<CArtiBodyNode_sim_tr*>(*it)->FK_UpdateNode<G_ROOT>();
						break;
				}
				for (it ++; it != it_end; it ++)
				{
					switch ((*it)->c_jtmflag)
					{
						case t_r:
							static_cast<CArtiBodyNode_sim_r*>((*it))->FK_UpdateNode<false>();
							break;
						case t_tr:
							static_cast<CArtiBodyNode_sim_tr*>((*it))->FK_UpdateNode<false>();
							break;
					}
				}
			}
		}
STOP_PROFILER
	}

	template<bool IS_SAVE>
	static void Serialize(CArtiBodyNode* root, TransformArchive& tms)
	{
		if (IS_SAVE)
			tms.Resize((int)root->m_kinalst.size());
		
		int i_tm = 0;
		auto onEnterBody = [&i_tm, &tms](CArtiBodyNode* body)
			{
				_TRANSFORM& tm_i = tms[i_tm ++];
				if (IS_SAVE)
				{
					Transform* tm_joint_i = body->GetJoint()->GetTransform();
					tm_joint_i->CopyTo(tm_i);
				}
				else // is restore
				{
					Transform* tm_joint_i = body->GetJoint()->GetTransform();
					tm_joint_i->CopyFrom(tm_i);
				}
			};
		auto onLeaveBody = [](CArtiBodyNode* body)
			{

			};
		CArtiBodyTree::TraverseDFS(root, onEnterBody, onLeaveBody);
	}

	static void Destroy(CArtiBodyNode* node);

//the following code for building posture graph, not for real-time usage
	static void Body_T_Test(const CArtiBodyNode* body
					, const Eigen::Vector3r& dir_up
					, const Eigen::Vector3r& dir_forward
					, const std::vector<std::string>& pts_interest
					, int part_idx_range[parts_total][2]
					, Real err[]);
	static void Body_EQ_Test(const CArtiBodyNode* body_s
					, const CArtiBodyNode* body_d
					, const std::vector<std::string>& pts_interest
					, Real err[]);
	static int GetBodies(const CArtiBodyNode* root
						, const std::list<std::string>& names
						, std::list<const CArtiBodyNode*>& nodes);

#ifdef _DEBUG
	static void Connect(CArtiBodyNode* from, CArtiBodyNode* to, CNN type);
#endif



};