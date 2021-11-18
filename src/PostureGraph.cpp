#include "pch.h"
#include "PostureGraph.hpp"

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

CPostureGraphClose2File::CPostureGraphClose2File(std::size_t n_vs, const CFile2ArtiBody* theta_src)
	: CPGTransition(n_vs)
	, c_thetaSrc_ref(theta_src)
	, m_thetaFile(theta_src->GetBody(), (int)n_vs)
{
}

void CPostureGraphClose2File::Initialize(CPostureGraphClose2File& graph, const Registry& reg, const Eigen::MatrixXr& errTB_src, int pid_T_src)
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

void CPostureGraphClose2File::Save(const char* dir) const
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

CPostureGraphClose2File::~CPostureGraphClose2File()
{
}


CPostureGraphOpen::CPostureGraphOpen(const CFile2ArtiBody* theta)
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

CPostureGraphClose2File* CPostureGraphOpen::GenerateClosePG(const CPostureGraphOpen& graph_src, const Eigen::MatrixXr& errTB, int pid_T_src)
{
	CPostureGraphClose2File::Registry regG;
	regG.Register_v(pid_T_src); // 'T' posture is the first posture registered which has no edges
	auto e_range_src = boost::edges(graph_src);
	const edge_iterator it_e_end_src = e_range_src.second;
	for (auto it_e_src = e_range_src.first
		; it_e_src != it_e_end_src
		; it_e_src++)
	{
		auto e_src = *it_e_src;
		vertex_descriptor v_src[] = { boost::source(e_src, graph_src), boost::target(e_src, graph_src) };
		CPostureGraphClose2File::vertex_descriptor v_dst[] = { regG.Register_v(v_src[0]), regG.Register_v(v_src[1]) };
		regG.Register_e(v_dst[0], v_dst[1]);
	}
	CPostureGraphClose2File* graph_dst = new CPostureGraphClose2File(regG.V.size(), graph_src.Theta());
	CPostureGraphClose2File::Initialize(*graph_dst, regG, errTB, pid_T_src);
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
		m_thetas = new CFile2ArtiBodyRef(filePath, body_ref);
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