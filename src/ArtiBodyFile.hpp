#pragma once
#include <queue>
#include <opencv2/opencv.hpp>
#include "bvh11_helper.hpp"
#include "loggerfast.h"
#include "ik_logger.h"
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
	CArtiBodyNode* CreateBody(BODY_TYPE type);
	void UpdateMotion(int i_frame, CArtiBodyNode* body);
	void ETB_Setup(Eigen::MatrixXr& err_out, const std::list<std::string>& joints);
private:
	CArtiBodyNode* CreateBodyBVH();
	CArtiBodyNode* CreateBodyHTR();
	typedef std::shared_ptr<const bvh11::Joint> Joint_bvh_ptr;
	typedef std::pair<const Joint_bvh_ptr, CArtiBodyNode*> Bound;

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