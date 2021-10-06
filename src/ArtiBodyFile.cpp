#include "pch.h"
#include "ArtiBodyFile.hpp"
#include "ArtiBody.hpp"

#define _FRAME_TIME_ 0.0083333

using namespace bvh11;

CArtiBodyFile::CArtiBodyFile(const CArtiBodyNode* root_src, int n_frames) // throw(...)
	: m_bodyRoot(root_src)
{
	frames_ = n_frames;
	frame_time_ = _FRAME_TIME_;
	auto Offset = [](const CArtiBodyNode* body) -> Eigen::Vector3d
		{
			const CArtiBodyNode* body_p = body->GetParent();
			if (nullptr == body_p)
				return Eigen::Vector3d::Zero();
			else
			{
				Eigen::Vector3r offset_r = body->GetTransformLocal2World()->getTranslation()
											- body_p->GetTransformLocal2World()->getTranslation();
				return Eigen::Vector3d(offset_r.x(), offset_r.y(), offset_r.z());
			}
		};

	auto root_dst = std::shared_ptr<Joint>(new Joint(root_src->GetName_c(), nullptr));
	root_joint_ = root_dst;
	struct Bound
	{
		const CArtiBodyNode* src;
		std::shared_ptr<Joint> dst;
	};
	Bound rootBnd = { root_src, root_dst };
	std::queue<Bound> bfs_que;
	bfs_que.push(rootBnd);
	while (!bfs_que.empty())
	{
		Bound node_b = bfs_que.front();
		auto body_src = node_b.src;
		auto joint_dst = node_b.dst;
		joint_dst->offset() = Offset(body_src);
		const CArtiBodyNode* body_src_child = body_src->GetFirstChild();
		if (nullptr == body_src_child)
		{
			joint_dst->has_end_site() = true;
			joint_dst->end_site() = Eigen::Vector3d::Zero();
		}
		for (
			; nullptr != body_src_child
			; body_src_child = body_src_child->GetNextSibling())
		{
			auto joint_dst_child = std::shared_ptr<Joint>(new Joint(body_src_child->GetName_c(), joint_dst));
			Bound node_b_child = { body_src_child, joint_dst_child };
			bfs_que.push(node_b_child);
			joint_dst->AddChild(joint_dst_child);
		}
		bfs_que.pop();
	}

	SetJointChannel(root_dst);

	motion_.resize(frames_, channels_.size());
}

void CArtiBodyFile::SetJointChannel(std::shared_ptr<Joint> joint)
{
	const std::vector<Channel::Type> channels_root = { Channel::Type::x_position
													, Channel::Type::y_position
													, Channel::Type::z_position
													, Channel::Type::z_rotation
													, Channel::Type::y_rotation
													, Channel::Type::x_rotation };
	const std::vector<Channel::Type> channels_leaf = { Channel::Type::z_rotation
													, Channel::Type::y_rotation
													, Channel::Type::x_rotation };
	bool is_root = (nullptr == joint->parent());
	const auto & channels_joint = (is_root ? channels_root : channels_leaf);

	for (auto channel : channels_joint)
	{
		joint->AssociateChannel((int)channels_.size());
		channels_.push_back({ channel, joint });
	}

	for (auto joint_child : joint->children())
		SetJointChannel(joint_child);
}


void CArtiBodyFile::SetMotion(int i_frame)
{
	auto& bvh = *this;
	auto onEnterBound_pose = [&bvh, i_frame](Bound b_this)
	{
		Joint_bvh_ptr joint_bvh = b_this.first;
		const IJoint* joint_hik = b_this.second->GetJoint();
		_TRANSFORM delta_l_tm = {0};
		joint_hik->GetTransform()->CopyTo(delta_l_tm);

		Eigen::Affine3d delta_l;
		Eigen::Vector3d tt(delta_l_tm.tt.x,
			delta_l_tm.tt.y,
			delta_l_tm.tt.z);
		Eigen::Quaterniond r(delta_l_tm.r.w,
			delta_l_tm.r.x,
			delta_l_tm.r.y,
			delta_l_tm.r.z);
		Eigen::Vector3d s(delta_l_tm.s.x,
			delta_l_tm.s.y,
			delta_l_tm.s.z);
		delta_l.fromPositionOrientationScale(tt, r, s);
		bvh.UpdateMotion(joint_bvh, delta_l, i_frame);

	};
	auto onLeaveBound_pose = [](Bound b_this) {};

	Bound root_b = std::make_pair(root_joint_, m_bodyRoot);
	TraverseBFS_boundtree_norecur(root_b, onEnterBound_pose, onLeaveBound_pose);
}