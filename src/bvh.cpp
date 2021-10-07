#include "pch.h"
#include <iostream>
#include <stack>
#include <queue>
#include "bvh.h"
#include "bvh11_helper.hpp"
#include "articulated_body.h"
#include "motion_pipeline.h"
#include "fk_joint.h"
#include "handle_helper.hpp"
#include "ik_logger.h"
#include "ArtiBodyFile.hpp"
#include "Math.hpp"
#include "loggerfast.h"


#define ZERO_ENTITY_TT_HOMO
#define ZERO_ENTITY_TT_CROSS

const double epsilon = 1e-4;

#pragma warning( push )
#pragma warning( disable : 4838 )
#pragma warning( disable : 4244 )

typedef std::shared_ptr<const bvh11::Joint> Joint_bvh_ptr;
typedef std::pair<Joint_bvh_ptr, HBODY> Bound;


HBODY CreateBVHArticulatedBody(bvh11::BvhObject& bvh)
{
	HBODY (*create_arti_body)(bvh11::BvhObject& bvh, Joint_bvh_ptr j_bvh)
	 = [] (bvh11::BvhObject& bvh, Joint_bvh_ptr j_bvh) -> HBODY
		{
			auto name = j_bvh->name().c_str();
			auto tm_bvh = bvh.GetTransformationRelativeToParent(j_bvh, -1);
			Eigen::Quaterniond rq(tm_bvh.linear());
			Eigen::Vector3d tt;
			TM_TYPE jtm;
			bool is_root = (nullptr == j_bvh->parent());
			if (is_root)
			{
				tt = Eigen::Vector3d::Zero();
				jtm = t_tr; //translation and rotation joint
			}
			else
			{
				tt = tm_bvh.translation();
				jtm = t_r; //rotation joint
			}

			_TRANSFORM tm_hik = {
				{1, 1, 1},
				{rq.w(), rq.x(), rq.y(), rq.z()},
				{tt.x(), tt.y(), tt.z()}
			};
			auto b_hik = create_bvh_body_node_c(name, &tm_hik, jtm);

			assert(nullptr != j_bvh->parent()
				|| tt.norm() < epsilon);

			return b_hik;
		};

	//traverse the bvh herachical structure
	//	to create an articulated body with the given posture
	std::queue<Bound> queBFS;
	auto root_j_bvh = bvh.root_joint();
	auto root_b_hik = create_arti_body(bvh, root_j_bvh);
	Bound root = std::make_pair(
		root_j_bvh,
		root_b_hik
	);
	queBFS.push(root);
	while (!queBFS.empty())
	{
		auto pair = queBFS.front();
		auto j_bvh = pair.first;
		auto b_hik = pair.second;
		CNN cnn = FIRSTCHD;
		auto& rb_this = b_hik;
		for (auto j_bvh_child : j_bvh->children())
		{
			auto b_hik_child = create_arti_body(bvh, j_bvh_child);
			Bound child = std::make_pair(
				j_bvh_child,
				b_hik_child
			);
			queBFS.push(child);
			auto& rb_next = b_hik_child;
			cnn_arti_body(rb_this, rb_next, cnn);
			cnn = NEXTSIB;
			rb_this = rb_next;
		}
		queBFS.pop();
	}
	initialize_kina(root_b_hik);
	update_fk(root_b_hik);
	return root_b_hik;
}




inline void printArtName(const char* name, int n_indent)
{
	std::string item;
	for (int i_indent = 0
		; i_indent < n_indent
		; i_indent++)
		item += "\t";
	item += name;
	std::cout << item.c_str() << std::endl;
}

inline void printBoundName(Bound bnd, int n_indent)
{
	std::string item;
	for (int i_indent = 0
		; i_indent < n_indent
		; i_indent++)
		item += "\t";
	std::string itemBvh(item);
	itemBvh += bnd.first->name();
	std::string itemBody(item);
	itemBody += body_name_c(bnd.second);
	std::cout << itemBvh.c_str() << std::endl;
	std::cout << itemBody.c_str() << std::endl;
}

inline bool TransformEq(const _TRANSFORM& tm_hik, const Eigen::Affine3d& tm_bvh)
{
	Eigen::Matrix3d linear_tm_bvh = tm_bvh.linear();
	Eigen::Vector3d tt_tm_bvh = tm_bvh.translation();

	Eigen::Vector3d s(tm_hik.s.x, tm_hik.s.y, tm_hik.s.z);
	Eigen::Matrix3d r(Eigen::Quaterniond(tm_hik.r.w, tm_hik.r.x, tm_hik.r.y, tm_hik.r.z));
	Eigen::Matrix3d linear_tm_hik = r * s.asDiagonal();
	Eigen::Vector3d tt_tm_hik(tm_hik.tt.x, tm_hik.tt.y, tm_hik.tt.z);

	Eigen::Matrix3d diff_linear = linear_tm_bvh.inverse() * linear_tm_bvh;
	double abs_diff_linear = (diff_linear - Eigen::Matrix3d::Identity()).norm();
	bool linear_eq = (-epsilon < abs_diff_linear
		&& abs_diff_linear < epsilon);
	double diff_tt = (tt_tm_bvh - tt_tm_hik).norm();
	bool tt_eq = (-epsilon < diff_tt
		&& diff_tt < epsilon);
	return linear_eq && tt_eq;
}

inline bool BoundEQ(Bound bnd, const bvh11::BvhObject& bvh, int frame)
{
	auto& joint_bvh = bnd.first;
	auto& body_hik = bnd.second;
	auto& name_bvh = joint_bvh->name();
	auto name_body = body_name_c(body_hik);
	bool name_eq = (name_bvh == name_body);
	Eigen::Affine3d tm_bvh = bvh.GetTransformation(joint_bvh, frame);

	_TRANSFORM tm_hik;
	get_body_transform_l2w(body_hik, &tm_hik);

	return name_eq && TransformEq(tm_hik, tm_bvh);
}

inline bool BoundResetAsBVHRest(Bound bnd, const bvh11::BvhObject& bvh, int frame)
{
	auto& joint_bvh = bnd.first;
	auto& body_hik = bnd.second;
	auto& name_bvh = joint_bvh->name();
	auto name_body = body_name_c(body_hik);
	bool name_eq = (name_bvh == name_body);
	Eigen::Affine3d tm_bvh = bvh.GetTransformation(joint_bvh, frame);
	auto joint_parent_bvh = joint_bvh->parent();
	Eigen::Affine3d tm_parent_bvh;
	if (nullptr == joint_parent_bvh)
	{
		tm_parent_bvh.linear() = Eigen::Matrix3d::Identity();
		tm_parent_bvh.translation() = tm_bvh.translation();
	}
	else
		tm_parent_bvh = bvh.GetTransformation(joint_parent_bvh, frame);
	Eigen::Vector3d tt_tm_bvh_l = tm_bvh.translation() - tm_parent_bvh.translation();


	_TRANSFORM tm_hik_l;
	get_body_transform_l2p(body_hik, &tm_hik_l);
	Eigen::Vector3d s(tm_hik_l.s.x, tm_hik_l.s.y, tm_hik_l.s.z);
	Eigen::Vector3d tt_tm_hik_l(tm_hik_l.tt.x, tm_hik_l.tt.y, tm_hik_l.tt.z);

	Real diff_s = (s - Eigen::Vector3d::Ones()).norm();
	Real diff_r = Eigen::Vector4d(tm_hik_l.r.w - 1, tm_hik_l.r.x, tm_hik_l.r.y, tm_hik_l.r.z).norm();
	Real diff_tt = (tt_tm_bvh_l - tt_tm_hik_l).norm();
	bool linear_id = (-epsilon < diff_s
		&&	diff_s < epsilon
		&&-epsilon < diff_r
		&&  diff_r < epsilon);
	bool tt_eq = (-epsilon < diff_tt
		&& diff_tt < epsilon);

	return name_eq && linear_id && tt_eq;
}

template<typename LAMaccessEnter, typename LAMaccessLeave>
inline void TraverseDFS_boundtree_recur(Bound bound_this, LAMaccessEnter OnEnterBound, LAMaccessLeave OnLeaveBound)
{
	OnEnterBound(bound_this);
	auto bvh_this = bound_this.first;
	auto body_this = bound_this.second;
	const auto& children_bvh_this = bvh_this->children();
	auto it_bvh_next = children_bvh_this.begin();
	auto body_next = get_first_child_body(body_this);
	bool proceed = (it_bvh_next != children_bvh_this.end());
	assert((proceed)
		== VALID_HANDLE(body_next));
	while (proceed)
	{
		Bound bound_next = std::make_pair(*it_bvh_next, body_next);
		TraverseDFS_boundtree_recur(bound_next, OnEnterBound, OnLeaveBound);
		it_bvh_next++;
		body_next = get_next_sibling_body(body_next);
		proceed = (it_bvh_next != children_bvh_this.end());
		assert((proceed)
			== VALID_HANDLE(body_next));
	}
	OnLeaveBound(bound_this);
}

void updateHeader(bvh11::BvhObject& bvh, HBODY body)
{
	//update the header part of the BVH file
	//	, the given body is in BVH rest posture
	auto lam_onEnter = [&bvh = std::as_const(bvh)](Bound b_this)
	{
		Joint_bvh_ptr joint_this = b_this.first;
		HBODY body_this = b_this.second;
		HBODY body_parent = get_parent_body(body_this);
		if (VALID_HANDLE(body_parent))
		{
			_TRANSFORM tm_l2w;
			get_body_transform_l2w(body_this, &tm_l2w);
			_TRANSFORM tm_p2w;
			get_body_transform_l2w(body_parent, &tm_p2w);
			const_cast<Eigen::Vector3d&>(joint_this->offset()) = Eigen::Vector3d( tm_l2w.tt.x - tm_p2w.tt.x
																				, tm_l2w.tt.y - tm_p2w.tt.y
																				, tm_l2w.tt.z - tm_p2w.tt.z );
		}
		else
		{
			const_cast<Eigen::Vector3d&>(joint_this->offset()) = Eigen::Vector3d::Zero();
		}
	};
	auto lam_onLeave = [](Bound b_this)
	{
	};
	Bound root = std::make_pair(bvh.root_joint(), body);
	TraverseDFS_boundtree_recur(root, lam_onEnter, lam_onLeave);
}

inline bool verify_bound(Bound b_this, bool enter, const bvh11::BvhObject& bvh, int i_frame)
{
	Joint_bvh_ptr joint_bvh = b_this.first;
	HBODY body_hik = b_this.second;
	const std::string& name_bvh = joint_bvh->name();
	std::string name_hik(body_name_c(body_hik));

	auto tm_l2w_bvh = bvh.GetTransformation(joint_bvh, i_frame);
	_TRANSFORM tm_l2w_hik = { 0 };
	get_body_transform_l2w(body_hik, &tm_l2w_hik);

	auto tm_l2p_bvh = bvh.GetTransformationRelativeToParent(joint_bvh, i_frame);
	_TRANSFORM tm_l2p_hik = { 0 };
	get_body_transform_l2p(body_hik, &tm_l2p_hik);

	auto tm_l_bvh = bvh.GetLocalDeltaTM(joint_bvh, i_frame);
	_TRANSFORM tm_l_hik = { 0 };
	get_joint_transform(body_hik, &tm_l_hik);

	bool delta_l_eq = TransformEq(tm_l_hik, tm_l_bvh);
	bool l2p_eq = TransformEq(tm_l2p_hik, tm_l2p_bvh);
	bool l2w_eq = TransformEq(tm_l2w_hik, tm_l2w_bvh);
	bool name_eq = (name_bvh == name_hik);

	bool verified = (name_eq && delta_l_eq && l2p_eq && l2w_eq);
	if (!verified)
	{
		std::cout << "Enter" << enter
			<< "\t" << name_hik.c_str()
			<< ":\t delta_l_eq = " << delta_l_eq
			<< "\t l2p_eq = " << l2p_eq
			<< "\t l2w_eq = " << l2w_eq
			<< "\t name_eq = " << name_eq
			<< "\t frame = " << i_frame
			<< std::endl;
	}
	return verified;
}


template<typename LAMaccessEnter, typename LAMaccessLeave>
inline void TraverseBFS_boundtree_norecur(Bound root, LAMaccessEnter OnEnterBound, LAMaccessLeave OnLeaveBound)
{
	std::queue<Bound> queBFS;
	queBFS.push(root);
	OnEnterBound(root);
	while (!queBFS.empty())
	{
		Bound b_this = queBFS.front();
		Joint_bvh_ptr joint_bvh = b_this.first;
		HBODY body_hik = b_this.second;
		auto& children_bvh = joint_bvh->children();
		auto it_bvh_child = children_bvh.begin();
		auto body_child = get_first_child_body(body_hik);
		for (
			; it_bvh_child != children_bvh.end()
			&& VALID_HANDLE(body_child)
			; it_bvh_child++,
			body_child = get_next_sibling_body(body_child))
		{
			Bound b_child = std::make_pair(*it_bvh_child, body_child);
			queBFS.push(b_child);
			OnEnterBound(b_child);
		}
		queBFS.pop();
		OnLeaveBound(b_this);
	}
}
// pose articulated body with the bvh and the frame
void pose_nonrecur(HBODY body_root, const bvh11::BvhObject& bvh, int i_frame)
{
	auto onEnterBound_pose = [&bvh, i_frame](Bound b_this)
	{
		Joint_bvh_ptr joint_bvh = b_this.first;
		HBODY body_hik = b_this.second;
		Eigen::Affine3d delta_l = bvh.GetLocalDeltaTM(joint_bvh, i_frame);
		Eigen::Quaterniond r(delta_l.linear());
		Eigen::Vector3d tt(delta_l.translation());
		_TRANSFORM delta_l_tm = {
			{1, 1, 1}, //scale is trivial
			{r.w(), r.x(), r.y(), r.z()}, //rotation
			{tt.x(), tt.y(), tt.z()}, //trivial
		};
		set_joint_transform(body_hik, &delta_l_tm);
	};
	auto onLeaveBound_pose = [](Bound b_this) {};

	Bound root = std::make_pair(bvh.root_joint(), body_root);
	TraverseBFS_boundtree_norecur(root, onEnterBound_pose, onLeaveBound_pose);

	update_fk(body_root);
}

void updateBVHAnim(HBODY body_root, bvh11::BvhObject& bvh, int i_frame, bool verify)
{
	auto onEnterBound_pose = [&bvh, i_frame](Bound b_this)
	{
		Joint_bvh_ptr joint_bvh = b_this.first;
		HBODY body_hik = b_this.second;
		_TRANSFORM delta_l_tm;
		get_joint_transform(body_hik, &delta_l_tm);

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

	Bound root = std::make_pair(bvh.root_joint(), body_root);
	TraverseBFS_boundtree_norecur(root, onEnterBound_pose, onLeaveBound_pose);

	update_fk(body_root);

	if (verify)
	{
		auto onEnterBound_verify = [](Bound b_this)
		{
		};

		auto onLeaveBound_verify = [&bvh, i_frame](Bound b_this)
		{
			verify_bound(b_this, true, bvh, i_frame);
		};

		TraverseBFS_boundtree_norecur(root, onEnterBound_verify, onLeaveBound_verify);
	}

}



HBODY create_tree_body_bvh_file(const wchar_t* path_src)
{
	std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
	auto path_src_c = converter.to_bytes(path_src);
	bvh11::BvhObject bvh(path_src_c);
	HBODY h_root = CreateBVHArticulatedBody(bvh);
	return h_root;
}

HBODY create_tree_body_bvh(HBVH hBvh)
{
	bvh11::BvhObject* bvh = CAST_2PBVH(hBvh);
	HBODY h_root = CreateBVHArticulatedBody(*bvh);
	return h_root;
}



HBVH load_bvh_c(const char* path_src)
{
	bvh11::BvhObject* bvh = NULL;
	try
	{
		bvh = new bvh11::BvhObject(path_src);
	}
	catch (const std::string& info)
	{
		LOGIK(info.c_str());
		return H_INVALID;
	}
	catch (...)
	{
		LOGIK("Unknown expection");
		return H_INVALID;
	}
	return CAST_2HBVH( bvh);
}

HBVH load_bvh_w(const wchar_t* path_src)
{
	std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
	auto path_src_c = converter.to_bytes(path_src);
	return load_bvh_c(path_src_c.c_str());
}

HBVH copy_bvh(HBVH src)
{
	bvh11::BvhObject* bvh_src = CAST_2PBVH(src);
	bvh11::BvhObject* bvh_dup = NULL;
	try
	{
		bvh_dup = new bvh11::BvhObject(*bvh_src);
	}
	catch (const std::string& info)
	{
		LOGIK(info.c_str());
		return H_INVALID;
	}
	catch (...)
	{
		LOGIK("Unknown expection");
		return H_INVALID;
	}
	return CAST_2HBVH(bvh_dup);
}

void unload_bvh(HBVH hBvh)
{
	bvh11::BvhObject* bvh = CAST_2PBVH(hBvh);
	delete bvh;
}

unsigned int get_n_frames(HBVH hBvh)
{
	bvh11::BvhObject* bvh = CAST_2PBVH(hBvh);
	return bvh->frames();
}

bool ResetRestPose(const char* path_src, int frame, const char* path_dst, double scale)
{
	try
	{
		bvh11::BvhObject bvh_src(path_src, scale);
		int frame_bvh11 = frame - 1;

		int n_frames = bvh_src.frames();
		bool in_range = (-1 < frame_bvh11
						&& frame_bvh11 < n_frames);
		if (!in_range)
			return false;
		HBODY h_driver = H_INVALID;
		HBODY h_driveeProxy = H_INVALID;
		HBODY h_drivee = H_INVALID;
		bool resetted = true;
		h_driver = CreateBVHArticulatedBody(bvh_src);
		resetted = VALID_HANDLE(h_driver);
#if defined _DEBUG
		{
			std::cout << "Bounds:" << std::endl;
			int n_indent = 1;
			auto lam_onEnter = [&n_indent, &bvh_src = std::as_const(bvh_src)](Bound b_this)
			{
				printBoundName(b_this, n_indent++);
				assert(BoundEQ(b_this, bvh_src, -1));
			};
			auto lam_onLeave = [&n_indent](Bound b_this)
			{
				n_indent--;
			};
			Bound root = std::make_pair(bvh_src.root_joint(), h_driver);
			TraverseDFS_boundtree_recur(root, lam_onEnter, lam_onLeave);
		}
#endif
		if (resetted)
		{
			pose_nonrecur(h_driver, bvh_src, frame_bvh11);
			resetted = clone_body_fbx(h_driver, &h_driveeProxy); // reset = false
		}

#if 0 //defined _DEBUG
		{
			std::cout << "Bounds:" << std::endl;
			int n_indent = 1;
			auto lam_onEnter = [&n_indent, &bvh_src = std::as_const(bvh_src), frame_bvh11](Bound b_this)
			{
				printBoundName(b_this, n_indent++);
				// BoundEQ(b_this, bvh_src, frame_bvh11);
				assert(BoundEQ(b_this, bvh_src, frame_bvh11));
			};
			auto lam_onLeave = [&n_indent](Bound b_this)
			{
				n_indent--;
			};
			Bound root = std::make_pair(bvh_src.root_joint(), h_driveeProxy);
			TraverseDFS_boundtree_recur(root, lam_onEnter, lam_onLeave);
		}
#endif
		if (resetted)
			resetted = clone_body_bvh(h_driveeProxy, &h_drivee);

#if defined _DEBUG
		if (resetted)
		{
			std::cout << "Bounds:" << std::endl;
			int n_indent = 1;
			auto lam_onEnter = [&n_indent, &bvh_src = std::as_const(bvh_src), frame_bvh11](Bound b_this)
			{
				printBoundName(b_this, n_indent++);
				assert(BoundResetAsBVHRest(b_this, bvh_src, frame_bvh11));
			};
			auto lam_onLeave = [&n_indent](Bound b_this)
			{
				n_indent--;
			};
			Bound root = std::make_pair(bvh_src.root_joint(), h_drivee);
			TraverseDFS_boundtree_recur(root, lam_onEnter, lam_onLeave);
		}
#endif
		if (resetted)
		{
			HMOTIONNODE h_motion_driver = create_tree_motion_node(h_driver);
			HMOTIONNODE h_motion_driveeProxy = create_tree_motion_node(h_driveeProxy);
			bool sync_created = motion_sync_cnn_homo(h_motion_driver, h_motion_driveeProxy, FIRSTCHD);
			assert(sync_created && "homo sync should be created for 2 same-header-BVHs");

			HMOTIONNODE h_motion_drivee = create_tree_motion_node(h_drivee);
			sync_created = motion_sync_cnn_cross_c(h_motion_driveeProxy, h_motion_drivee, FIRSTCHD, NULL, 0, NULL);
			assert(sync_created && "cross sync should be created for 2 algined postures");

			bool pre_reset_header = true;

			if (pre_reset_header)
			{
				CArtiBodyNode* drivee_root = CAST_2PBODY(h_drivee);
				auto bvh_reset = new CArtiBodyFile(drivee_root, n_frames);
				// LoggerFast logger((std::string(path_dst)+"_dup").c_str());
				// bvh_reset->OutputHeader(logger);
				for (int i_frame = 0
					; i_frame < n_frames
					; i_frame++)
				{
					PROFILE_FRAME(i_frame);
					pose_nonrecur(h_driver, bvh_src, i_frame);
					motion_sync(h_motion_driver);
					bvh_reset->SetMotion(i_frame);
					// bvh_reset->OutputMotion(i_frame, logger);
				}
				bvh_reset->WriteBvhFile(path_dst);
				delete bvh_reset;
			}

			HMOTIONNODE h_motions[] = {h_motion_driver, h_motion_driveeProxy, h_motion_drivee};
			const int n_motions = sizeof(h_motions)/sizeof(HMOTIONNODE);
			for (int i_motion = 0; i_motion < n_motions; i_motion++)
				destroy_tree_motion_node(h_motions[i_motion]);
		}

		HBODY bodies[] = {h_driver, h_driveeProxy, h_drivee};
		const int n_body = sizeof(bodies)/sizeof(HBODY);
		for (int i_body = 0; i_body < n_body; i_body ++)
		{
			if (VALID_HANDLE(bodies[i_body]))
				destroy_tree_body(bodies[i_body]);
		}
		return true;
	}
	catch (std::string& exp)
	{
		LOGIK(exp.c_str());
		LOGIKFlush();
		return false;
	}
}

void pose_body(HBVH bvh, HBODY body, int i_frame)
{
	auto pBvh = CAST_2PBVH(bvh);
	pose_nonrecur(body, *pBvh, i_frame);
}

unsigned int channels(HBVH hBvh)
{
	bvh11::BvhObject* pBVH = CAST_2PBVH(hBvh);
	return (unsigned int)pBVH->channels().size();
}



double frame_time(HBVH hBvh)
{
	bvh11::BvhObject* pBVH = CAST_2PBVH(hBvh);
	return pBVH->frame_time();
}

void PrintJointHierarchy(HBVH hBvh)
{
	bvh11::BvhObject* pBVH = CAST_2PBVH(hBvh);
	return pBVH->PrintJointHierarchy();
}

void WriteBvhFile(HBVH hBvh, const char* path_dst)
{
	auto* pBVH = CAST_2PBVH(hBvh);
	pBVH->WriteBvhFile(path_dst);
}

#pragma warning( pop )