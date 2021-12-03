#include "pch.h"
#include "PostureGraph.hpp"
#include "PostureGraph_helper.hpp"

#define MAX_N_THETA_HOMO 51200
#define MAX_N_THETA_X MAX_N_THETA_HOMO

#define MED_N_THETA_X_ETB 1024000

#define MAX_N_THETA_HOMO_ETB 20480
#define MAX_N_THETA_X_ETB ((uint64_t)MAX_N_THETA_HOMO_ETB*(uint64_t)MAX_N_THETA_HOMO_ETB)

CPGThetaRuntime::CPGThetaRuntime(const char* path, CArtiBodyNode* body_ref)
{
	m_rootRef = body_ref;
	Initialize(path, body_ref);
}

CPGThetaRuntime::CPGThetaRuntime(const std::string& path, CArtiBodyNode* body_ref)
{
	m_rootRef = body_ref;
	Initialize(path, body_ref);
}

// the standard body might not be compatible with the file,
// the name of the body/joint is used for creating the map between standard and the file
// the motions is created for the standard body
void CPGThetaRuntime::Initialize(const std::string& path, CArtiBodyNode* root_ref)
{
	std::string exp("the standard body is not compatible with the bvh/htr file");
	CArtiBodyFile abfile(path);
	IKAssert(abfile.c_type == m_rootRef->c_type);
	int n_frames = abfile.frames();
	m_motions.resize(n_frames);
	m_jointsRef.clear();

	struct CHANNEL
	{
		const char* name;
		const CArtiBodyNode* body_file;
		CArtiBodyNode* body_ref;
	};

	CArtiBodyNode* root_file = abfile.CreateBodyHTR(); //the body type is trivial, since only the body joint is used which is indepedant of the body type

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

	auto onEnterBound_pose = [&src = abfile, &i_frame](CArtiBodyFile::Bound b_this)
	{
		IKAssert(b_this.first->name() == b_this.second->GetName_c());
		const CArtiBodyFile::Joint_bvh_ptr joint_bvh = b_this.first;
		CArtiBodyNode* body_hik = b_this.second;
		Eigen::Affine3d delta_l = src.GetLocalDeltaTM(joint_bvh, i_frame);
		Eigen::Quaterniond r(delta_l.linear());
		Eigen::Vector3d tt(delta_l.translation());
		IJoint* body_joint = body_hik->GetJoint();
		body_joint->SetRotation(Eigen::Quaternionr((Real)r.w(), (Real)r.x(), (Real)r.y(), (Real)r.z()));
		body_joint->SetTranslation(Eigen::Vector3r((Real)tt.x(), (Real)tt.y(), (Real)tt.z()));
	};
	auto onLeaveBound_pose = [](CArtiBodyFile::Bound b_this) {};

	CArtiBodyFile::Bound root_file_bnd = std::make_pair(abfile.root_joint(), root_file);

	int n_tms = (int)m_jointsRef.size();
	TransformArchive tms_bk(n_tms);
	for (int i_tm = 0; i_tm < n_tms; i_tm++)
	{
		_TRANSFORM& tm_i = tms_bk[i_tm];
		m_jointsRef[i_tm]->GetTransform()->CopyTo(tm_i);
	}


	for (i_frame = 0; i_frame < n_frames; i_frame ++)
	{
		abfile.TraverseBFS_boundtree_norecur(root_file_bnd, onEnterBound_pose, onLeaveBound_pose); //to pose body
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

CPGTheta::CPGTheta(const char* path)
	: m_rootBody(NULL)
{
	CArtiBodyFile artiFile(path);
	Initialize(artiFile);
}

CPGTheta::CPGTheta(const std::string& path)
	: m_rootBody(NULL)
{
	CArtiBodyFile artiFile(path);
	Initialize(artiFile);
}

CPGTheta::CPGTheta()
	: m_rootBody(NULL)
{
}

CPGTheta::CPGTheta(const CPGTheta& src)
	: m_rootBody(NULL)
{
	if (!CArtiBodyTree::Clone(src.m_rootBody, &m_rootBody))
		throw std::string("clone body failed");
	std::size_t n_motions = src.m_motions.size();
	m_motions.resize(n_motions);

	for (std::size_t i_motion = 0; i_motion < n_motions; i_motion ++)
	{
		TransformArchive& motions_i_dst = m_motions[i_motion];
		const TransformArchive& motions_i_src = src.m_motions[i_motion];
		motions_i_dst = motions_i_src;
	}
}

CPGTheta::~CPGTheta()
{
	if (NULL != m_rootBody)
		CArtiBodyTree::Destroy(m_rootBody);
}

void CPGTheta::Initialize(const CArtiBodyFile& artiFile)
{
	m_rootBody = artiFile.CreateBody();

	TransformArchive tm_bk;
	CArtiBodyTree::Serialize<true>(m_rootBody, tm_bk);

	int n_frames = artiFile.frames();
	m_motions.resize(n_frames);

	int i_frame = 0;

	auto onEnterBound_pose = [&src = artiFile, &i_frame](CArtiBodyFile::Bound b_this)
	{
		IKAssert(b_this.first->name() == b_this.second->GetName_c());
		const CArtiBodyFile::Joint_bvh_ptr joint_bvh = b_this.first;
		CArtiBodyNode* body_hik = b_this.second;
		Eigen::Affine3d delta_l = src.GetLocalDeltaTM(joint_bvh, i_frame);
		Eigen::Quaterniond r(delta_l.linear());
		Eigen::Vector3d tt(delta_l.translation());
		IJoint* body_joint = body_hik->GetJoint();
		body_joint->SetRotation(Eigen::Quaternionr((Real)r.w(), (Real)r.x(), (Real)r.y(), (Real)r.z()));
		body_joint->SetTranslation(Eigen::Vector3r((Real)tt.x(), (Real)tt.y(), (Real)tt.z()));
	};
	auto onLeaveBound_pose = [](CArtiBodyFile::Bound b_this) {};

	CArtiBodyFile::Bound root = std::make_pair(artiFile.root_joint(), m_rootBody);

	for (i_frame = 0; i_frame < n_frames; i_frame ++)
	{
		artiFile.TraverseBFS_boundtree_norecur(root, onEnterBound_pose, onLeaveBound_pose); //to pose body
		TransformArchive& tms_i = m_motions[i_frame];
		CArtiBodyTree::Serialize<true>(m_rootBody, tms_i);
	}

	CArtiBodyTree::Serialize<false>(m_rootBody, tm_bk);
	CArtiBodyTree::FK_Update<false>(m_rootBody);
}

bool CPGTheta::Merge(const CPGTheta& theta_other)
{
	bool body_eq = CArtiBodyTree::Similar(m_rootBody, theta_other.m_rootBody);
	if (body_eq)
	{
		m_motions.insert(m_motions.end()
					, theta_other.m_motions.begin()
					, theta_other.m_motions.end());
	}
	return body_eq;
}

CPGTheta::Query* CPGTheta::BeginQuery(const std::list<std::string>& joints) const
{
	CPGTheta::Query* query = new CPGTheta::Query;
	if (CArtiBodyTree::Clone(m_rootBody, &query->rootPose))
	{
		query->n_interests = CArtiBodyTree::GetBodies(query->rootPose, joints, query->interests);
		return query;
	}
	else
	{
		IKAssert(0); // not expect it to happen
		delete query;
		return NULL;
	}

}

void CPGTheta::EndQuery(CPGTheta::Query* query) const
{
	CArtiBodyTree::Destroy(query->rootPose);
	delete query;
}

void CPGTheta::QueryTheta(CPGTheta::Query* query, int i_theta, TransformArchive& tm_data) const
{
	PoseBody<false>(i_theta, query->rootPose);
	int i_tm = 0;
	for (auto body : query->interests)
	{
		_TRANSFORM& tm_i = tm_data[i_tm ++];
		body->GetJoint()->GetTransform()->CopyTo(tm_i);
	}
}

bool CPGTheta::SmallX(int n_theta_0, int n_theta_1)
{
	return (n_theta_0 + n_theta_1) < MAX_N_THETA_X;
}

bool CPGTheta::SmallXETB(int n_theta_0, int n_theta_1)
{
	return ((uint64_t)n_theta_0 * (uint64_t)n_theta_1) < (MED_N_THETA_X_ETB);
}

bool CPGTheta::MedianXETB(int n_theta_0, int n_theta_1)
{
	uint64_t size = (uint64_t)n_theta_0 * (uint64_t)n_theta_1;
	return (MED_N_THETA_X_ETB <= size)
		&& (size < (MAX_N_THETA_X_ETB));
}

bool CPGTheta::SmallHomo(int n_theta)
{
	return n_theta < MAX_N_THETA_HOMO;
}

bool CPGTheta::SmallHomoETB(int n_theta)
{
	return n_theta < MAX_N_THETA_HOMO_ETB;
}

CPG::CPG(std::size_t n_vs)
	: CPGTransition(n_vs)
{
}

CPG::CPG()
	: CPGTransition(0)
{

}

void CPG::Initialize(CPG& graph, const Registry& reg, const CPGTheta& theta_src)
{
	struct V_ERR
	{
		vertex_descriptor v_dst;
		Real err_T;
	};

	class V_ERRCompare
	{
	public:
		bool operator()(const V_ERR& v_err_1, const V_ERR& v_err_2)
		{
			return v_err_1.err_T < v_err_2.err_T;
		}
	};

	std::list<V_ERR> lstV_ERR_T;
	auto it_reg_v = reg.V.begin();
	IKAssert(0 == it_reg_v->v_src
			&& 0 == it_reg_v->v_dst); 	// the 'T' posture is supposed to be the first one to be registered

	CArtiBodyRef2File abfile(theta_src.GetBody(), (int)graph.m_vertices.size());

	theta_src.PoseBody<false>(0);
	abfile.UpdateMotion(0);
	CArtiBodyNode* body_theta_src = const_cast<CArtiBodyNode*>(theta_src.GetBody());
	TransformArchive tm_0;
	CArtiBodyTree::Serialize<true>(body_theta_src, tm_0);

	TransformArchive tm_i;
	for (it_reg_v ++; it_reg_v != reg.V.end(); it_reg_v ++) // skip the 'T' posture to avoid a self-pointing edge
	{
		const auto& reg_v = *it_reg_v;
		theta_src.PoseBody<false>((int)reg_v.v_src);
		abfile.UpdateMotion((int)reg_v.v_dst);
		CArtiBodyTree::Serialize<true>(body_theta_src, tm_i);
		lstV_ERR_T.push_back({reg_v.v_dst, TransformArchive::Error_q(tm_0, tm_i)});
	}
	theta_src.ResetPose<false>();
	graph.m_theta.Initialize(abfile);

	for (Registry::REGISTER_e reg_e : reg.E)
	{
		boost::add_edge(reg_e.v_0, reg_e.v_1, graph);
	}

#if defined _DEBUG
	Dump(graph, __FILE__, __LINE__);
#endif

	lstV_ERR_T.sort(V_ERRCompare()); // by err_T

	std::size_t deg_max = 0;
	auto v_range = boost::vertices(graph);
	for (auto it_v = v_range.first; it_v != v_range.second; it_v ++)
		deg_max = std::max(deg_max, boost::degree(*it_v, graph));

	std::size_t n_cnn_T = 0;
	for (auto it_v_err = lstV_ERR_T.begin()
		; it_v_err != lstV_ERR_T.end()
		  && n_cnn_T < deg_max
		; it_v_err ++, n_cnn_T ++)
		boost::add_edge(0, it_v_err->v_dst, graph);
#if defined _DEBUG
	Dump(graph, __FILE__, __LINE__);
#endif

}

void CPG::Save(const char* dir) const
{
	std::string file_name(m_theta.GetBody()->GetName_c());

	fs::path path_t(dir);
	std::string file_name_t(file_name); file_name_t += ".pg";
	path_t.append(file_name_t);
	SaveTransitions(path_t.u8string().c_str(), F_PG);

	fs::path htr_path(dir);
	std::string htr_file_name(file_name); htr_file_name += ".htr";
	htr_path.append(htr_file_name);
	int n_theta = m_theta.N_Theta();
	CArtiBodyRef2File abfile(m_theta.GetBody(), n_theta);
	for (int i_theta = 0; i_theta < n_theta; i_theta ++)
	{
		m_theta.PoseBody<false>(i_theta);
		abfile.UpdateMotion(i_theta);
	}
	m_theta.ResetPose<false>();
	abfile.WriteBvhFile(htr_path.u8string().c_str());
}

bool CPG::Load(const char* dir, const char* pg_name)
{
	fs::path dir_path(dir);
	std::string filename_transi(pg_name); filename_transi += ".pg";
	fs::path path_transi(dir_path); path_transi.append(filename_transi);
	bool loaded_transi = LoadTransitions(path_transi.u8string().c_str());

	std::string filename_theta(pg_name); filename_theta += ".htr";
	fs::path path_theta(dir_path); path_theta.append(filename_theta);
	bool loaded_theta = LoadThetas(path_theta.u8string().c_str());

	LOGIKVar(LogInfoCharPtr, pg_name);
	LOGIKVar(LogInfoBool, loaded_transi);
	LOGIKVar(LogInfoBool, loaded_theta);
	bool loaded =  (loaded_transi && loaded_theta);

#if defined _DEBUG
	IKAssert(!loaded || m_theta.N_Theta() == num_vertices(*this));
	if (loaded)
	{
		bool all_vertices_error_untagged = true;
		//check error
		auto v_range = boost::vertices(*this);
		for (vertex_iterator it_v = v_range.first; it_v != v_range.second && all_vertices_error_untagged; it_v++)
		{
			const auto& v_property = (*this)[*it_v];
			all_vertices_error_untagged = !ErrorTagged(v_property);
		}
		IKAssert(all_vertices_error_untagged);
	}
	LOGIKVar(LogInfoInt, m_theta.N_Theta());
#endif

	return loaded;
}

bool CPG::LoadThetas(const std::string& path_theta)
{
	try
	{
		CArtiBodyFile abfile(path_theta);
		m_theta.Initialize(abfile);
		return true;
	}
	catch (std::string &exp)
	{
		LOGIKVarErr(LogInfoCharPtr, exp.c_str());
		return false;
	}
}

CPG::~CPG()
{
}

CPGMatrixGen::CPGMatrixGen(const CPGTheta* theta)
	: Super(theta)
{
}

void CPGMatrixGen::Remove(vertex_descriptor v, const IErrorTB* errTB)
{
	CPGMatrixGen& graph = *this;
	// for a adjcent matrix graph, it does not remove a vertex
	std::list<vertex_descriptor> neighbors_v;
	auto vertices_range_neighbors = boost::adjacent_vertices(v, graph);
	for (auto it_vert_n = vertices_range_neighbors.first
		; it_vert_n != vertices_range_neighbors.second
		; it_vert_n++)
	{
		const auto& v_n_property = (graph)[*it_vert_n];
		neighbors_v.push_back(*it_vert_n);
	}
	auto edges_range_incident = boost::out_edges(v, graph);
	for (auto it_edge_incident = edges_range_incident.first
		; it_edge_incident != edges_range_incident.second
		; it_edge_incident++)
	{
		boost::remove_edge(*it_edge_incident, graph);
	}
	auto it_v_n = neighbors_v.begin();
	vertex_descriptor v_star = *it_v_n;
	for (it_v_n ++; it_v_n != neighbors_v.end(); it_v_n ++)
	{
		auto v_n = *it_v_n;
		if ( !(graph)[v_n].tag_rm
			&& errTB->Get(v, v_n) < errTB->Get(v, v_star))
			v_star = v_n;
	}

	for (it_v_n = neighbors_v.begin(); it_v_n != neighbors_v.end(); it_v_n ++)
	{
		auto v_n = *it_v_n;
		if (v_n != v_star)				// avoid self-pointing edge
			boost::add_edge(v_star, v_n, graph);
	}

}

CPGListGen::CPGListGen(const CPGTheta* theta)
	: Super(theta)
{
}

void CPGListGen::Remove(vertex_descriptor v, const IErrorTB* errTB)
{
	// for a adjcent matrix graph, it does not remove a vertex
	CPGListGen& graph = *this;
	// for a adjcent matrix graph, it does not remove a vertex
	std::list<vertex_descriptor> neighbors_v;
	auto vertices_range_neighbors = boost::adjacent_vertices(v, graph);
	for (auto it_vert_n = vertices_range_neighbors.first
		; it_vert_n != vertices_range_neighbors.second
		; it_vert_n++)
	{
		const auto& v_n_property = (graph)[*it_vert_n];
		neighbors_v.push_back(*it_vert_n);
	}
	auto edges_range_incident = boost::out_edges(v, graph);
	auto it_edge_incident = edges_range_incident.first;
	for ( auto it_edge_incident_p = it_edge_incident
		; it_edge_incident != edges_range_incident.second
		; it_edge_incident = it_edge_incident_p)
	{
		it_edge_incident_p ++;
		boost::remove_edge(*it_edge_incident, graph);
	}
	auto it_v_n = neighbors_v.begin();
	vertex_descriptor v_star = *it_v_n;
	for (it_v_n++; it_v_n != neighbors_v.end(); it_v_n++)
	{
		auto v_n = *it_v_n;
		if ( !(graph)[v_n].tag_rm
			&& errTB->Get(v, v_n) < errTB->Get(v, v_star))
			v_star = v_n;
	}

	auto vertices_range_neighbors_v_star = boost::adjacent_vertices(v_star, graph);
	std::set<vertex_descriptor> neighbors_v_star(vertices_range_neighbors_v_star.first
												, vertices_range_neighbors_v_star.second);

	for (it_v_n = neighbors_v.begin(); it_v_n != neighbors_v.end(); it_v_n++)
	{
		auto v_n = *it_v_n;
		if (
			   v_n != v_star 											// avoid self-pointing edge
			&& (neighbors_v_star.end() == neighbors_v_star.find(v_n)) 	// avoid duplicated edge
			)
			boost::add_edge(v_star, v_n, graph);
	}
}

bool CPGRuntime::Load(const char* dir, CArtiBodyNode* root)
{
	const char* pg_name = root->GetName_c();
	fs::path dir_path(dir);
	std::string filename_transi(pg_name); filename_transi += ".pg";
	fs::path path_transi(dir_path); path_transi.append(filename_transi);
	bool loaded_transi = LoadTransitions(path_transi.u8string().c_str());

	std::string filename_theta(pg_name); filename_theta += ".htr";
	fs::path path_theta(dir_path); path_theta.append(filename_theta);
	bool loaded_theta = LoadThetas(path_theta.u8string().c_str(), root);

	LOGIKVar(LogInfoCharPtr, pg_name);
	LOGIKVar(LogInfoBool, loaded_transi);
	LOGIKVar(LogInfoBool, loaded_theta);
	bool loaded =  (loaded_transi && loaded_theta);

#if defined _DEBUG
	IKAssert(!loaded || m_thetas->N_Theta() == num_vertices(*this));
	if (loaded)
	{
		bool all_vertices_error_untagged = true;
		//check error
		auto v_range = boost::vertices(*this);
		for (vertex_iterator it_v = v_range.first; it_v != v_range.second && all_vertices_error_untagged; it_v++)
		{
			const auto& v_property = (*this)[*it_v];
			all_vertices_error_untagged = !ErrorTagged(v_property);
		}
		IKAssert(all_vertices_error_untagged);
	}
#endif

	return loaded;
}

bool CPGRuntime::LoadThetas(const char* filePath, CArtiBodyNode* body_ref)
{
	if (NULL != m_thetas)
		delete m_thetas;
	bool loaded = false;
	try
	{
		m_thetas = new CPGThetaRuntime(filePath, body_ref);
		m_theta_star = 0;
		loaded = true;
	}
	catch(std::string& exp)
	{
		std::stringstream errInfo;
		errInfo << "Load " << filePath << " " << exp;
		LOGIKVarErr(LogInfoCharPtr, errInfo.str().c_str());
		m_thetas = NULL;
	}
	return loaded;
}

#undef MAX_N_THETA_HOMO_ETB
#undef MAX_N_THETA_X_ETB

#undef MAX_N_THETA_HOMO
#undef MAX_N_THETA_X
