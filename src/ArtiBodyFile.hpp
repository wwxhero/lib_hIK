#pragma once
#include <queue>
#include "bvh11_helper.hpp"
#include "loggerfast.h"
#include "ik_logger.h"
#include "Joint.hpp"


class CArtiBodyRef2File : public bvh11::BvhObject
{
public:
	CArtiBodyRef2File(const CArtiBodyNode* root_src, int n_frames);
	void UpdateMotion(int i_frame);

	static void OutputHeader(CArtiBodyRef2File& bf, LoggerFast &logger);
	static void OutputMotion(CArtiBodyRef2File& bf, int i_frame, LoggerFast& logger);
private:
	void SetJointChannel(const CArtiBodyNode* body, std::shared_ptr<bvh11::Joint> joint);
	typedef std::shared_ptr<const bvh11::Joint> Joint_bvh_ptr;
	typedef std::pair<Joint_bvh_ptr, const CArtiBodyNode*> Bound;

	template<typename LAMaccessEnter, typename LAMaccessLeave>
	inline void TraverseBFS_boundtree_norecur(Bound root, LAMaccessEnter OnEnterBound, LAMaccessLeave OnLeaveBound)
	{
		std::queue<Bound> queBFS;
		queBFS.push(root);
		OnEnterBound(root);
		while (!queBFS.empty())
		{
			auto b_this = queBFS.front();
			auto joint_bvh = b_this.first;
			const CArtiBodyNode* body_hik = b_this.second;
			auto& children_bvh = joint_bvh->children();
			auto it_bvh_child = children_bvh.begin();
			auto body_child = body_hik->GetFirstChild();
			for (
				; it_bvh_child != children_bvh.end()
				&& nullptr != body_child
				; it_bvh_child++,
				body_child = body_child->GetNextSibling())
			{
				auto b_child = std::make_pair(*it_bvh_child, body_child);
				queBFS.push(b_child);
				OnEnterBound(b_child);
			}
			queBFS.pop();
			OnLeaveBound(b_this);
		}
	}
private:
	const CArtiBodyNode* m_bodyRoot;
};

class CArtiBodyFile : public bvh11::BvhObject
{
public:
	CArtiBodyFile(const char* path); //throw ...
	CArtiBodyFile(const std::string& path); //throw ...
	CArtiBodyFile(const CArtiBodyFile& src); //throw ...

public:
	static BODY_TYPE toType(const std::string& path);
	CArtiBodyNode* CreateBody(BODY_TYPE type) const;
	CArtiBodyNode* CreateBodyBVH() const;
	CArtiBodyNode* CreateBodyHTR() const;
	typedef std::shared_ptr<const bvh11::Joint> Joint_bvh_ptr;
	typedef std::pair<const Joint_bvh_ptr, CArtiBodyNode*> Bound;

	template<typename LAMaccessEnter, typename LAMaccessLeave>
	inline void TraverseBFS_boundtree_norecur(Bound root, LAMaccessEnter OnEnterBound, LAMaccessLeave OnLeaveBound) const
	{
		std::queue<Bound> queBFS;
		queBFS.push(root);
		OnEnterBound(root);
		while (!queBFS.empty())
		{
			auto b_this = queBFS.front();
			queBFS.pop();
			IKAssert(b_this.first->name()
					== b_this.second->GetName_c());
			auto joint_bvh = b_this.first;
			const CArtiBodyNode* body_hik = b_this.second;
			auto& children_bvh = joint_bvh->children();
			auto it_bvh_child = children_bvh.begin();
			auto body_child = body_hik->GetFirstChild();
			for (
				; it_bvh_child != children_bvh.end()
				&& nullptr != body_child
				; it_bvh_child++,
				body_child = body_child->GetNextSibling())
			{
				auto b_child = std::make_pair(*it_bvh_child, body_child);
				queBFS.push(b_child);
				OnEnterBound(b_child);
			}
			OnLeaveBound(b_this);
		}
	}
protected:
	std::vector<TransformArchive> m_motions;
};


class CThetaArtiBody
{
public:
	CThetaArtiBody(const char* path);
	CThetaArtiBody(const std::string& path);
	virtual ~CThetaArtiBody();
public:
	template<bool G_SPACE>
	void PoseBody(int i_frame) const
	{
		PoseBody<G_SPACE>(i_frame, m_rootBody);
	}

	void ETB_Setup(Eigen::MatrixXr& err_out, const std::list<std::string>& joints);

	int frames() const {return (int)m_motions.size();}

	const CArtiBodyNode* GetBody() const { return m_rootBody; }
	CArtiBodyNode* GetBody() { return m_rootBody;  }

	bool Merge(const CThetaArtiBody& f2b);
protected:
	template<bool G_SPACE>
	void PoseBody(int i_frame, CArtiBodyNode* body) const
	{
		IKAssert(i_frame < (int)m_motions.size());
		TransformArchive& tms_i = const_cast<TransformArchive&>(m_motions[i_frame]);
		CArtiBodyTree::Serialize<false>(body, tms_i);
		CArtiBodyTree::FK_Update<G_SPACE>(body);
	}

private:
	void Initialize(const std::string& path);
private:
	CArtiBodyNode* m_rootBody;
	std::vector<TransformArchive> m_motions;
};


class CBodyLogger
{
public:
	CBodyLogger(const CArtiBodyNode* root, const char* path) throw(...);
	~CBodyLogger();
	void LogHeader();
	void LogMotion();
private:
	CArtiBodyRef2File m_bodyFile;
	LoggerFast m_logger;
	unsigned int m_nMotions;
};