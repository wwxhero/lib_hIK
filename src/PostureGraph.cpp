#include "pch.h"
#include "PostureGraph.hpp"

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

CPGThetaClose::CPGThetaClose(const char* path)
	: m_rootBody(NULL)
{
	Initialize(path);
}

CPGThetaClose::CPGThetaClose(const std::string& path)
	: m_rootBody(NULL)
{
	Initialize(path);
}

CPGThetaClose::~CPGThetaClose()
{
	CArtiBodyTree::Destroy(m_rootBody);
}

void CPGThetaClose::Initialize(const std::string& path)
{
	CArtiBodyFile artiFile(path);
	m_rootBody = artiFile.CreateBody(CArtiBodyFile::toType(path));

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

bool CPGThetaClose::Merge(const CPGThetaClose& f2b_other)
{
	bool body_eq = CArtiBodyTree::Similar(m_rootBody, f2b_other.m_rootBody);
	if (body_eq)
	{
		m_motions.insert(m_motions.end()
					, f2b_other.m_motions.begin()
					, f2b_other.m_motions.end());
	}
	return body_eq;
}

void CPGThetaClose::ETB_Setup(Eigen::MatrixXr& err_out, const std::list<std::string>& joints)
{
	unsigned int n_frames = frames();
	err_out.resize(n_frames, n_frames);

	TransformArchive tm_bk;
	CArtiBodyTree::Serialize<true>(m_rootBody, tm_bk); // backup the original configuration

	std::list<const CArtiBodyNode*> interest_bodies;
	int n_bodies = CArtiBodyTree::GetBodies(m_rootBody, joints, interest_bodies);
	TransformArchive tm_data_i(n_bodies);
	TransformArchive tm_data_j(n_bodies);

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
		PoseBody<false>(i_frame, m_rootBody);
		UpdateTransforms(interest_bodies, tm_data_i);
		for (unsigned int j_frame = 0; j_frame < i_frame; j_frame++)
		{
			PoseBody<false>(j_frame, m_rootBody);
			UpdateTransforms(interest_bodies, tm_data_j);
			auto& vis_scale_ij = err_out(i_frame, j_frame);
			auto& vis_scale_ji = err_out(j_frame, i_frame);
			auto err_ij = TransformArchive::Error_q(tm_data_i, tm_data_j);
			vis_scale_ij = err_ij;
			vis_scale_ji = err_ij;
		}
		err_out(i_frame, i_frame) = (Real)0;
	}

	CArtiBodyTree::Serialize<false>(m_rootBody, tm_bk); // restore the original configuration
	CArtiBodyTree::FK_Update<false>(m_rootBody);
}

template<typename G>
void Dump(G& g, const char* fileName, int lineNo)
{
	auto file_short = [](const char* file_f) -> const char*
	{
#ifdef _WIN32
#define DELIMITER '\\'
#else
#define DELIMITER '/'
#endif
		const char* p_delim = NULL;
		for (const char* p = file_f
			; *p != '\0'
			; p++)
		{
			if (*p == DELIMITER)
				p_delim = p;
		}
		assert(NULL != p_delim);
		return ++p_delim;
	};
	std::stringstream dot_path;
	dot_path << file_short(fileName) << "_" << lineNo << ".dot";
	std::ofstream dot_file(dot_path.str());
	IKAssert(std::ios_base::failbit != dot_file.rdstate());
	write_graphviz(dot_file, g);
}

CPGClose::CPGClose(std::size_t n_vs, const CPGThetaClose* theta_src)
	: CPGTransition(n_vs)
	, c_thetaSrc_ref(theta_src)
	, m_thetaFile(theta_src->GetBody(), (int)n_vs)
{
}

void CPGClose::Initialize(CPGClose& graph, const Registry& reg, const Eigen::MatrixXr& errTB_src, int pid_T_src)
{
	struct V_ERR
	{
		vertex_descriptor v_dst;
		Real err_T;
	};
	class V_ERRCompare {
	public:
		bool operator()(const V_ERR& v_err_1, const V_ERR& v_err_2)
		{
			return v_err_1.err_T < v_err_2.err_T;
		}
	};

	std::list<V_ERR> lstV_ERR_T;
	auto it_reg_v = reg.V.begin();
	IKAssert(pid_T_src == it_reg_v->v_src
			&& 0 == it_reg_v->v_dst); 	// the 'T' posture is supposed to be the first one to be registered
	graph.c_thetaSrc_ref->PoseBody<false>(pid_T_src);
	graph.m_thetaFile.UpdateMotion(0);
	for (it_reg_v ++; it_reg_v != reg.V.end(); it_reg_v ++) // skip the 'T' posture to avoid a self-pointing edge
	{
		const auto& reg_v = *it_reg_v;
		graph.c_thetaSrc_ref->PoseBody<false>((int)reg_v.v_src);
		graph.m_thetaFile.UpdateMotion((int)reg_v.v_dst);
		lstV_ERR_T.push_back({reg_v.v_dst, errTB_src(pid_T_src, reg_v.v_src)});
	}

	for (Registry::REGISTER_e reg_e : reg.E)
	{
		boost::add_edge(reg_e.v_0, reg_e.v_1, graph);
	}

#if defined _DEBUG
	Dump(graph, __FILE__, __LINE__);
#endif

	lstV_ERR_T.sort(V_ERRCompare()); // by err_T

	std::size_t deg_sigma = 0;
	std::size_t n_vs = 0;
	auto v_range = boost::vertices(graph);
	for (auto it_v = v_range.first; it_v != v_range.second; it_v ++, n_vs ++)
		deg_sigma += boost::degree(*it_v, graph);
	std::size_t deg_average = (std::size_t)floor((Real)deg_sigma/(Real)n_vs);

	std::size_t n_cnn_T = 0;
	for (auto it_v_err = lstV_ERR_T.begin()
		; it_v_err != lstV_ERR_T.end()
		  && n_cnn_T < deg_average
		; it_v_err ++, n_cnn_T ++)
		boost::add_edge(0, it_v_err->v_dst, graph);
#if defined _DEBUG
	Dump(graph, __FILE__, __LINE__);
#endif

}

void CPGClose::Save(const char* dir) const
{
	std::string file_name(c_thetaSrc_ref->GetBody()->GetName_c());

	fs::path path_t(dir);
	std::string file_name_t(file_name); file_name_t += ".pg";
	path_t.append(file_name_t);
	SaveTransitions(path_t.u8string().c_str(), F_PG);

	fs::path htr_path(dir);
	std::string htr_file_name(file_name); htr_file_name += ".htr";
	htr_path.append(htr_file_name);
	m_thetaFile.WriteBvhFile(htr_path.u8string().c_str());
}

CPGClose::~CPGClose()
{
}


CPostureGraphOpen::CPostureGraphOpen(const CPGThetaClose* theta)
	: PostureGraphMatrix< VertexGen, EdgeGen>((std::size_t)(theta->frames()))
	, m_theta(theta)
{
}

CPostureGraphOpen::~CPostureGraphOpen()
{
}

void CPostureGraphOpen::InitTransitions(CPostureGraphOpen& graph, const Eigen::MatrixXr& errTB, Real epsErr_deg, const std::vector<int>& postureids_ignore)
{
	std::set<int> pids_ignore(postureids_ignore.begin(), postureids_ignore.end());

	int n_frames = graph.m_theta->frames();
	int i_frame = 0;
	bool i_frame_ignored = (pids_ignore.end() != pids_ignore.find(i_frame));
	for (int i_frame_p = i_frame + 1; i_frame_p < n_frames; i_frame ++, i_frame_p ++)
	{
		bool i_frame_p_ignored = (pids_ignore.end() != pids_ignore.find(i_frame_p));
		if (!i_frame_ignored && !i_frame_p_ignored)
			boost::add_edge(i_frame, i_frame + 1, graph);
		i_frame_ignored = i_frame_p_ignored;
	}

#if defined _DEBUG
	Dump(graph, __FILE__, __LINE__);
#endif

	// err_epsilon = (1-cos(theta_eps_deg*deg2rad/2))*65535;
	Real err_epsilon = (1 - cos(deg2rad(epsErr_deg) / (Real)2));
	IKAssert(errTB.rows() == n_frames);
	for (int i_theta = 0; i_theta < n_frames; i_theta++) //to skip the 'T' posture
	{
		bool i_ignored = (pids_ignore.end() != pids_ignore.find(i_theta));
		if (i_ignored)
			continue;
		for (int j_theta = i_theta + 1; j_theta < n_frames; j_theta++)
		{
			bool j_ignored = (pids_ignore.end() != pids_ignore.find(j_theta));
			if (j_ignored)
				continue;
			if (errTB(i_theta, j_theta) < err_epsilon)
				boost::add_edge(i_theta, j_theta, graph);
		}
	}

#if defined _DEBUG
	Dump(graph, __FILE__, __LINE__);
#endif

	//tag rm for each vertex
	auto v_range = boost::vertices(graph);
	vertex_iterator it_v_end = v_range.second;
	for (vertex_iterator it_v = v_range.first; it_v != it_v_end; it_v++)
	{
		auto& v_property = (graph)[*it_v];
		v_property.tag_rm = false;
		v_property.deg = 0;
	}

	// compute degree for the vertex (work around for adjacent matrix)
	auto e_range = boost::edges(graph);
	const edge_iterator it_e_end = e_range.second;
	for (edge_iterator it_e = e_range.first; it_e != it_e_end; it_e++)
	{
		auto e = *it_e;
		vertex_descriptor v[] = { boost::source(e, graph), boost::target(e, graph) };
		(graph)[v[0]].deg++; (graph)[v[1]].deg++;
	}

	// tag edge degrees and sort edges by degree
	std::list<edge_descriptor> edges_eps;
	for (edge_iterator it_e = e_range.first; it_e != it_e_end; it_e++)
	{
		auto e = *it_e;
		edges_eps.push_back(e);
		vertex_descriptor v[] = { boost::source(e, graph), boost::target(e, graph) };
		std::size_t deg[] = { (graph)[v[0]].deg, (graph)[v[1]].deg };
		auto& e_property = (graph)[e];
		e_property.deg = std::max(deg[0], deg[1]);
	}

	class ComEdgeByDeg
	{
	public:
		ComEdgeByDeg(CPostureGraphOpen& g)
			: graph(g)
		{

		}
		bool operator()(const edge_descriptor& e_i, const edge_descriptor& e_j)
		{
			return (graph)[e_i].deg > (graph)[e_j].deg;
		}
	private:
		CPostureGraphOpen& graph;
	};
	edges_eps.sort(ComEdgeByDeg(graph));

	while (!edges_eps.empty())
	{
		bool exists_a_tagged_vertex = false;
		for (auto e : edges_eps)
		{
			vertex_descriptor v[] = { boost::source(e, graph), boost::target(e, graph) };
			std::size_t deg[] = { (graph)[v[0]].deg, (graph)[v[1]].deg };
			bool rm[] = { (graph)[v[0]].tag_rm, (graph)[v[1]].tag_rm };
			if (!rm[0] && !rm[1])
			{
				if (deg[0] < deg[1])
				{
					(graph)[v[1]].tag_rm = true;
					exists_a_tagged_vertex = true;
				}
				else if (deg[0] > deg[1])
				{
					(graph)[v[0]].tag_rm = true;
					exists_a_tagged_vertex = true;
				}
			}
		}
		if (!exists_a_tagged_vertex && !edges_eps.empty())
			(graph)[boost::source(*edges_eps.begin(), graph)].tag_rm = true;

		for (auto it_e = edges_eps.begin()
			; it_e != edges_eps.end()
			; )
		{
			auto e = *it_e;
			vertex_descriptor v[] = { boost::source(e, graph), boost::target(e, graph) };
			bool rm[] = { (graph)[v[0]].tag_rm, (graph)[v[1]].tag_rm };
			if (rm[0] || rm[1])
			{
				(graph)[v[0]].deg--; (graph)[v[1]].deg--;
				it_e = edges_eps.erase(it_e);
			}
			else
				it_e++;
		}
	}


	for (vertex_iterator it_v = v_range.first; it_v != it_v_end; it_v++)
	{
		const auto& v_property = (graph)[*it_v];
		if (v_property.tag_rm)
		{
			std::list<vertex_descriptor> neightbors_v;
			auto vertices_range_neighbors = boost::adjacent_vertices(*it_v, graph);
			for (auto it_vert_n = vertices_range_neighbors.first
				; it_vert_n != vertices_range_neighbors.second
				; it_vert_n++)
			{
				const auto& v_n_property = (graph)[*it_vert_n];
				neightbors_v.push_back(*it_vert_n);
			}

			auto edges_range_incident = boost::out_edges(*it_v, graph);
			for (auto it_edge_incident = edges_range_incident.first
				; it_edge_incident != edges_range_incident.second
				; it_edge_incident++)
			{
				boost::remove_edge(*it_edge_incident, graph);
			}

			for (auto it_v_i = neightbors_v.begin(); it_v_i != neightbors_v.end(); it_v_i++)
			{
				auto it_v_j = it_v_i;
				for (it_v_j++; it_v_j != neightbors_v.end(); it_v_j++)
				{
					vertex_descriptor v_i = *it_v_i;
					vertex_descriptor v_j = *it_v_j;
					if (errTB(v_i, v_j) > err_epsilon)
					{
						boost::add_edge(v_i, v_j, graph);
					}
				}
			}
		}
	}

#if defined _DEBUG
	Dump(graph, __FILE__, __LINE__);
#endif


}

CPGClose* CPostureGraphOpen::GenerateClosePG(const CPostureGraphOpen& graph_src, const Eigen::MatrixXr& errTB, int pid_T_src)
{
	CPGClose::Registry regG;
	regG.Register_v(pid_T_src); // 'T' posture is the first posture registered which has no edges
	auto e_range_src = boost::edges(graph_src);
	const edge_iterator it_e_end_src = e_range_src.second;
	for (auto it_e_src = e_range_src.first
		; it_e_src != it_e_end_src
		; it_e_src++)
	{
		auto e_src = *it_e_src;
		vertex_descriptor v_src[] = { boost::source(e_src, graph_src), boost::target(e_src, graph_src) };
		CPGClose::vertex_descriptor v_dst[] = { regG.Register_v(v_src[0]), regG.Register_v(v_src[1]) };
		regG.Register_e(v_dst[0], v_dst[1]);
	}
	CPGClose* graph_dst = new CPGClose(regG.V.size(), graph_src.Theta());
	CPGClose::Initialize(*graph_dst, regG, errTB, pid_T_src);
	return graph_dst;
}

bool CPGRuntime::Load(const char* dir, CArtiBodyNode* root)
{
	const char* pg_name = root->GetName_c();
	fs::path dir_path(dir);
	std::string filename_t(pg_name); filename_t += ".pg";
	fs::path path_t(dir_path); path_t.append(filename_t);
	bool loaded_t = LoadTransitions(path_t.u8string().c_str());

	std::string filename_theta(pg_name); filename_theta += ".htr";
	fs::path path_theta(dir_path); path_theta.append(filename_theta);
	bool loaded_theta = LoadThetas(path_theta.u8string().c_str(), root);

	LOGIKVar(LogInfoCharPtr, pg_name);
	LOGIKVar(LogInfoBool, loaded_t);
	LOGIKVar(LogInfoBool, loaded_theta);
	bool loaded =  (loaded_t && loaded_theta);

#if defined _DEBUG
	IKAssert(!loaded || m_thetas->frames() == num_vertices(*this));
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

void CPostureGraphOpen::Save(const char* dir, PG_FileType type) const
{
	fs::path file_path(dir);
	std::string file_name(m_theta->GetBody()->GetName_c()); file_name += ".dot";
	file_path.append(file_name);
	std::ofstream file(file_path);
	IKAssert(std::ios_base::failbit != file.rdstate());
	if (F_DOT)
		write_graphviz(file, *this);
	else
	{
		IKAssert(0); // does not support serialize for an adjacency matrix
	}
}