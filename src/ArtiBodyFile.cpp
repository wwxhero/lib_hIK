#include "pch.h"
#include "ArtiBodyFile.hpp"
#include "ArtiBody.hpp"
#include "filesystem_helper.hpp"

#define _FRAME_TIME_ 0.0083333

using namespace bvh11;

CArtiBodyRef2File::CArtiBodyRef2File(const CArtiBodyNode* root_src, int n_frames) // throw(...)
	: m_bodyRoot(root_src)
{
	frames_ = n_frames;
	frame_time_ = _FRAME_TIME_;
	auto Offset = [](const CArtiBodyNode* body) -> Eigen::Vector3d
		{
			const CArtiBodyNode* body_p = body->GetParent();
			Eigen::Vector3r offset_r;
			if (nullptr == body_p)
				offset_r = body->GetTransformLocal2World()->getTranslation();
			else
			{
				offset_r = body->GetTransformLocal2World()->getTranslation()
							- body_p->GetTransformLocal2World()->getTranslation();
			}
			return Eigen::Vector3d(offset_r.x(), offset_r.y(), offset_r.z());
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

	SetJointChannel(root_src, root_dst);

	motion_.resize(frames_, channels_.size());
}

void CArtiBodyRef2File::SetJointChannel(const CArtiBodyNode* body, std::shared_ptr<Joint> joint)
{
	struct Entry
	{
		TM_TYPE tm_type;
		Channel::Type ch_type;
	} entries [] = {
		{t_tt, Channel::Xposition},
		{t_tt, Channel::Yposition},
		{t_tt, Channel::Zposition},
		{t_rz, Channel::Zrotation},
		{t_ry, Channel::Yrotation},
		{t_rx, Channel::Xrotation},
	};

	auto tm_type_body = body->GetJoint()->GetTransform()->Type();
	std::vector<Channel::Type> channels_joint;
	for (auto entry : entries)
	{
		if (tm_type_body&entry.tm_type)
			channels_joint.push_back(entry.ch_type);
	}

	for (auto channel : channels_joint)
	{
		joint->AssociateChannel((int)channels_.size());
		channels_.push_back({ channel, joint });
	}

	// for (auto joint_child : joint->children())
	auto joint_child = joint->children().begin();
	auto body_child = body->GetFirstChild();
	for (
		; joint->children().end() != joint_child
			&& nullptr != body_child
		; joint_child ++, body_child = body_child->GetNextSibling())
		SetJointChannel(body_child, *joint_child);
}


void CArtiBodyRef2File::UpdateMotion(int i_frame)
{
	bvh11::BvhObject& bvh = *this;
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

void CArtiBodyRef2File::OutputHeader(CArtiBodyRef2File& bf, LoggerFast &logger)
{
	std::stringstream st;
	st << "HIERARCHY" << "\n";
	bf.WriteJointSubHierarchy<std::stringstream>(st, bf.root_joint_, 0);
	// Motion
	st << "MOTION" << "\n";
	st << "Frames: " << bf.frames_ << "\n";
	st << "Frame Time: " << bf.frame_time_ << "\n";
	logger.Out(st.str().c_str());
}

void CArtiBodyRef2File::OutputMotion(CArtiBodyRef2File& bf, int i_frame, LoggerFast& logger)
{
	const Eigen::IOFormat motion_format(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", "", "", "\n", "", "");
	auto row_i = bf.motion_.row(i_frame);
	std::stringstream st;
	st << row_i.format(motion_format);
	logger.Out(st.str().c_str());
}


BODY_TYPE CArtiBodyFile::toType(const std::string& path)
{
	std::string ext = fs::path(path).extension().u8string();
	struct ExtType
	{
		std::string ext;
		BODY_TYPE type;
	} ext_type[] = {
		{".htr", htr},
		{".bvh", bvh}
	};

	BODY_TYPE type = undef;
	for (int i_ext2type = 0
		; i_ext2type < sizeof(ext_type)/sizeof(ExtType)
			&& undef == type
		; i_ext2type ++)
	{
		if (ext == ext_type[i_ext2type].ext)
			type = ext_type[i_ext2type].type;
	}
	IKAssert(undef != type);
	return type;
}

CArtiBodyFile::CArtiBodyFile(const char* path)
	: bvh11::BvhObject(path)
{
}

CArtiBodyFile::CArtiBodyFile(const std::string& path)
	: bvh11::BvhObject(path)
{
}


CArtiBodyNode* CArtiBodyFile::CreateBody(BODY_TYPE type) const
{
	switch(type)
	{
		case bvh:
			return CreateBodyBVH();
		case htr:
			return CreateBodyHTR();
		default:
			IKAssert(0);
			return NULL;
	}
}

CArtiBodyNode* CArtiBodyFile::CreateBodyBVH() const
{
	// CArtiBodyNode* (*create_arti_body)(const bvh11::BvhObject&, Joint_bvh_ptr)
	auto create_arti_body = [&] (const bvh11::BvhObject& bvh_src, Joint_bvh_ptr j_bvh) -> CArtiBodyNode*
			{
				auto name = j_bvh->name().c_str();
				auto tm_bvh = bvh_src.GetTransformationRelativeToParent(j_bvh, -1);
				Eigen::Quaterniond rq(tm_bvh.linear());
				Eigen::Vector3d tt(tm_bvh.translation());
				TM_TYPE jtm = t_none;
				bool exists_tt = false;
				bool exists_r = false;
				for (auto i_channel : j_bvh->associated_channels_indices())
				{
					exists_tt = ( exists_tt
								|| Channel::Xposition == channels_[i_channel].type
								|| Channel::Yposition == channels_[i_channel].type
								|| Channel::Zposition == channels_[i_channel].type );
					exists_r  = (  exists_r
								|| Channel::Xrotation == channels_[i_channel].type
								|| Channel::Yrotation == channels_[i_channel].type
								|| Channel::Zrotation == channels_[i_channel].type );
					if (exists_tt && exists_r)
						break;
				}
				IKAssert(exists_tt || exists_r);
				if (exists_tt)
					jtm = (TM_TYPE)(jtm | t_tt);
				if (exists_r)
					jtm = (TM_TYPE)(jtm | t_r);
				_TRANSFORM tm_hik = {
					{1, 1, 1},
					{(Real)rq.w(), (Real)rq.x(), (Real)rq.y(), (Real)rq.z()},
					{(Real)tt.x(), (Real)tt.y(), (Real)tt.z()}
				};
				auto b_hik = CArtiBodyTree::CreateSimNode(name, &tm_hik, BODY_TYPE::bvh, jtm);
				//assert(nullptr != j_bvh->parent()
				//	|| tt.norm() < c_epsilon);

				return b_hik;
			};

	//traverse the bvh herachical structure
	//	to create an articulated body with the given posture
	std::queue<Bound> queBFS;
	auto root_j_bvh = root_joint();
	CArtiBodyNode* root_b_hik = create_arti_body(*this, root_j_bvh);
	Bound root = std::make_pair(
		root_j_bvh,
		root_b_hik
	);
	queBFS.push(root);
	while (!queBFS.empty())
	{
		auto pair = queBFS.front();
		auto j_bvh = pair.first;
		CArtiBodyNode* b_hik = pair.second;
		CNN cnn = FIRSTCHD;
		CArtiBodyNode*& rb_this = b_hik;
		for (auto j_bvh_child : j_bvh->children())
		{
			CArtiBodyNode* b_hik_child = create_arti_body(*this, j_bvh_child);
			Bound child = std::make_pair(
				j_bvh_child,
				b_hik_child
			);
			queBFS.push(child);
			auto& rb_next = b_hik_child;
			CArtiBodyTree::Connect(rb_this, rb_next, cnn);
			cnn = NEXTSIB;
			rb_this = rb_next;
		}
		queBFS.pop();
	}
	CArtiBodyTree::KINA_Initialize(root_b_hik);
	CArtiBodyTree::FK_Update<false>(root_b_hik);
	return root_b_hik;
}

CArtiBodyNode* CArtiBodyFile::CreateBodyHTR() const
{
	CArtiBodyNode* body_bvh = NULL;
	CArtiBodyNode* body_htr = NULL;
	auto CloneNode = [](const CArtiBodyNode* src, CArtiBodyNode** dst, const wchar_t* name_dst_opt) -> bool
		{
			return CArtiBodyTree::CloneNode_htr(src, dst, Eigen::Matrix3r::Identity(), name_dst_opt);
		};
	bool created = (NULL != (body_bvh = CreateBodyBVH())
					&& CArtiBodyTree::Clone(body_bvh, &body_htr, CloneNode));
	if (body_bvh)
		CArtiBodyTree::Destroy(body_bvh);
	if (created)
		return body_htr;
	else
	{
		if (body_htr)
			CArtiBodyTree::Destroy(body_htr);
		return NULL;
	}
}

CFile2ArtiBody::CFile2ArtiBody(const char* path)
	: CArtiBodyFile(std::string(path))
	, m_rootBody(NULL)
{
	m_rootBody = CreateBody(CArtiBodyFile::toType(path));
	Initialize();
}

CFile2ArtiBody::CFile2ArtiBody(const std::string& path)
	: CArtiBodyFile(path)
	, m_rootBody(NULL)
{
	m_rootBody = CreateBody(CArtiBodyFile::toType(path));
	Initialize();
}

CFile2ArtiBody::~CFile2ArtiBody()
{
	CArtiBodyTree::Destroy(m_rootBody);
}

void CFile2ArtiBody::Initialize()
{
	int n_frames = frames();
	m_motions.resize(n_frames);

	int i_frame = 0;

	auto onEnterBound_pose = [&src = *this, &i_frame](Bound b_this)
	{
		IKAssert(b_this.first->name() == b_this.second->GetName_c());
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

	Bound root = std::make_pair(root_joint(), m_rootBody);

	for (i_frame = 0; i_frame < n_frames; i_frame ++)
	{
		TraverseBFS_boundtree_norecur(root, onEnterBound_pose, onLeaveBound_pose); //to pose body
		TransformArchive& tms_i = m_motions[i_frame];
		CArtiBodyTree::Serialize<true>(m_rootBody, tms_i);
	}
}

void CFile2ArtiBody::ETB_Setup(Eigen::MatrixXr& err_out, const std::list<std::string>& joints)
{
	unsigned int n_frames = frames();
	err_out.resize(n_frames, n_frames);
	// err_out.create(n_frames, n_frames, CV_16U);
	CArtiBodyNode* body_i = CreateBody(BODY_TYPE::htr);
	std::list<const CArtiBodyNode*> interest_bodies_i;
	int n_bodies_i = CArtiBodyTree::GetBodies(body_i, joints, interest_bodies_i);
	TransformArchive tm_data_i(n_bodies_i);

	CArtiBodyNode* body_j = CreateBody(BODY_TYPE::htr);
	std::list<const CArtiBodyNode*> interest_bodies_j;
	int n_bodies_j = CArtiBodyTree::GetBodies(body_j, joints, interest_bodies_j);
	TransformArchive tm_data_j(n_bodies_j);

	bool ok = (n_bodies_i == n_bodies_j);
	IKAssert(ok);

	auto UpdateTransforms = [] (std::list<const CArtiBodyNode*>& interest_bodies, TransformArchive& tm_data)
		{
			int i_tm = 0;
			for (auto body : interest_bodies)
			{
				_TRANSFORM& tm_i = tm_data[i_tm ++];
				body->GetJoint()->GetTransform()->CopyTo(tm_i);
			}
		};


	for (unsigned int i_frame = 0; i_frame < n_frames; i_frame++)
	{
		PoseBody<false>(i_frame, body_i);
		// CArtiBodyTree::Serialize<true>(body_i, tm_data_i);
		UpdateTransforms(interest_bodies_i, tm_data_i);
		for (unsigned int j_frame = 0; j_frame < n_frames; j_frame++)
		{
			PoseBody<false>(j_frame, body_j);
			// CArtiBodyTree::Serialize<true>(body_j, tm_data_j);
			UpdateTransforms(interest_bodies_j, tm_data_j);
			// auto& vis_scale_ij = err_out.at<unsigned short>(i_frame, j_frame);
			auto& vis_scale_ij = err_out(i_frame, j_frame);
			auto err_ij = TransformArchive::Error_q(tm_data_i, tm_data_j);
			// vis_scale_ij = (unsigned short)(err_ij * USHRT_MAX);
			vis_scale_ij = err_ij;
		}
	}
	CArtiBodyTree::Destroy(body_i);
	CArtiBodyTree::Destroy(body_j);
}

CFile2ArtiBodyRef::CFile2ArtiBodyRef(const char* path, CArtiBodyNode* body_ref)
	: CArtiBodyFile(path)
{
	m_rootRef = body_ref;
	Initialize(body_ref);
}

CFile2ArtiBodyRef::CFile2ArtiBodyRef(const std::string& path, CArtiBodyNode* body_ref)
	: CArtiBodyFile(path)
{
	m_rootRef = body_ref;
	Initialize(body_ref);
}

// the standard body might not be compatible with the file,
// the name of the body/joint is used for creating the map between standard and the file
// the motions is created for the standard body
void CFile2ArtiBodyRef::Initialize(CArtiBodyNode* root_ref)
{
	std::string exp("the standard body is not compatible with the bvh/htr file");
	int n_frames = frames();
	m_motions.resize(n_frames);
	m_jointsRef.clear();

	struct CHANNEL
	{
		const char* name;
		const CArtiBodyNode* body_file;
		CArtiBodyNode* body_ref;
	};

	CArtiBodyNode* root_file = CreateBodyHTR(); //the body type is trivial, since only the body joint is used which is indepedant of the body type

	std::unique_ptr<CArtiBodyNode, void(*)(CArtiBodyNode*)> root_file_gc(
																root_file
																, [](CArtiBodyNode* ptr)
        															{
        															    CArtiBodyTree::Destroy(ptr);
        															});

	std::vector<CHANNEL> channels;

	auto OnEnterBody_file = [&channels](CArtiBodyNode* body)
		{
			CHANNEL c = {body->GetName_c(), body, NULL};
			channels.push_back(c);
		};

	auto OnLeaveBody_file = [](CArtiBodyNode* body)
		{

		};

	CArtiBodyTree::TraverseDFS(root_file, OnEnterBody_file, OnLeaveBody_file);

	std::map<std::string, std::size_t> name2channel_i;
	for (std::size_t i_channel = 0; i_channel < channels.size(); i_channel++)
	{
		name2channel_i[channels[i_channel].name] = i_channel;
	}


	auto OnEnterBody_ref = [&channels, &name2channel_i = std::as_const(name2channel_i), exp](CArtiBodyNode* body)
		{
			auto it_c = name2channel_i.find(body->GetName_c());
			if (name2channel_i.end() != it_c)
			{
				channels[it_c->second].body_ref = body;
			}
		};

	auto OnLeaveBody_ref = [](CArtiBodyNode* body)
		{
		};

	CArtiBodyTree::TraverseDFS(root_ref, OnEnterBody_ref, OnLeaveBody_ref);

	for (auto channel : channels)
	{
		bool valid_ch_i = (NULL != channel.body_ref
						&& NULL != channel.body_file);
		if (valid_ch_i)
		{
			m_jointsRef.push_back(channel.body_ref->GetJoint());
			IKAssert(std::string(channel.body_ref->GetName_c())
					== std::string(channel.body_file->GetName_c()));
		}
	}

	int i_frame = 0;

	auto onEnterBound_pose = [&src = *this, &i_frame](Bound b_this)
	{
		IKAssert(b_this.first->name() == b_this.second->GetName_c());
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

	Bound root_file_bnd = std::make_pair(root_joint(), root_file);

	int n_tms = (int)m_jointsRef.size();
	TransformArchive tms_bk(n_tms);
	for (int i_tm = 0; i_tm < n_tms; i_tm++)
	{
		_TRANSFORM& tm_i = tms_bk[i_tm];
		m_jointsRef[i_tm]->GetTransform()->CopyTo(tm_i);
	}


	for (i_frame = 0; i_frame < n_frames; i_frame ++)
	{
		TraverseBFS_boundtree_norecur(root_file_bnd, onEnterBound_pose, onLeaveBound_pose); //to pose body
		for (auto channel : channels)
		{
			bool through = (NULL != channel.body_ref
							&& NULL != channel.body_file);
			if (!through)
				continue;
			IJoint* joint_ref_i = channel.body_ref->GetJoint();
			const IJoint* joint_file_i = channel.body_file->GetJoint();
			const Transform* tm_file_i = joint_file_i->GetTransform();
			joint_ref_i->SetRotation(Transform::getRotation_q(tm_file_i));
			joint_ref_i->SetTranslation(tm_file_i->getTranslation());
		}

		TransformArchive& motion_i = m_motions[i_frame];
		motion_i.Resize(n_tms);
		for (int j_tm = 0; j_tm < n_tms; j_tm ++)
		{
			_TRANSFORM& tm_ij = motion_i[j_tm];
			IJoint* joint_j = m_jointsRef[j_tm];
			joint_j->GetTransform()->CopyTo(tm_ij);
		}
	}

	for (int i_tm = 0; i_tm < n_tms; i_tm ++)
	{
		const _TRANSFORM& tm_i = tms_bk[i_tm];
		m_jointsRef[i_tm]->GetTransform()->CopyFrom(tm_i);
	}
}

CBodyLogger::CBodyLogger(const CArtiBodyNode* root, const char* path) throw (...)
	: m_bodyFile(root, 1)
	, m_logger(path)
	, m_nMotions(0)
{
}

CBodyLogger::~CBodyLogger()
{
	m_logger << "Frames: " << m_nMotions;
	m_logger.Flush();
}

void CBodyLogger::LogHeader()
{
	CArtiBodyRef2File::OutputHeader(m_bodyFile, m_logger);
	// m_logger.Flush();
}

void CBodyLogger::LogMotion()
{
	m_bodyFile.UpdateMotion(0);
	CArtiBodyRef2File::OutputMotion(m_bodyFile, 0, m_logger);
	m_nMotions ++;
}
