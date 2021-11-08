#pragma once
#include <queue>
#include <opencv2/opencv.hpp>
#include "bvh11_helper.hpp"
#include "loggerfast.h"
#include "ik_logger.h"
#include "Joint.hpp"

class CArtiBody2File : public bvh11::BvhObject
{
public:
	CArtiBody2File(const CArtiBodyNode* root_src, int n_frames);
	void UpdateMotion(int i_frame);

	static void OutputHeader(CArtiBody2File& bf, LoggerFast &logger);
	static void OutputMotion(CArtiBody2File& bf, int i_frame, LoggerFast& logger);
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

class CFile2ArtiBody : public bvh11::BvhObject
{
public:
	CFile2ArtiBody(const char* path);
	CFile2ArtiBody(const std::string& path);
	CArtiBodyNode* CreateBody(BODY_TYPE type) const;
	
	template<bool G_SPACE>
	void PoseBody(int i_frame, CArtiBodyNode* body) const
	{
		// IKAssert(i_frame < (int)m_motions.size());
		// TransformArchive& tms_i = const_cast<TransformArchive&>(m_motions[i_frame]);
		// CArtiBodyTree::Serialize<false>(body, tms_i);
		// CArtiBodyTree::FK_Update<G_SPACE>(body);
		auto onEnterBound_pose = [&src = *this, i_frame](Bound b_this)
		{
			IKAssert(b_this.first->name() == b_this.second->GetName_c());
			LOGIKVar(LogInfoCharPtr, b_this.first->name().c_str());
			LOGIKVar(LogInfoCharPtr, b_this.second->GetName_c());
			const Joint_bvh_ptr joint_bvh = b_this.first;
			CArtiBodyNode* body_hik = b_this.second;
			Eigen::Affine3d delta_l = src.GetLocalDeltaTM(joint_bvh, i_frame);
			Eigen::Quaterniond r(delta_l.linear());
			Eigen::Vector3d tt(delta_l.translation());
			IJoint* body_joint = body_hik->GetJoint();
			body_joint->SetRotation(Eigen::Quaternionr((Real)r.w(), (Real)r.x(), (Real)r.y(), (Real)r.z()));
			body_joint->SetTranslation(Eigen::Vector3r((Real)tt.x(), (Real)tt.y(), (Real)tt.z()));
		};
		auto onLeaveBound_pose = [](Bound b_this) {};

		Bound root = std::make_pair(root_joint(), body);
		TraverseBFS_boundtree_norecur(root, onEnterBound_pose, onLeaveBound_pose);
		CArtiBodyTree::FK_Update<G_SPACE>(body);
	}

	void ETB_Setup(Eigen::MatrixXr& err_out, const std::list<std::string>& joints);
private:
	void Initialize();

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
	CArtiBody2File m_bodyFile;
	LoggerFast m_logger;
	unsigned int m_nMotions;
};