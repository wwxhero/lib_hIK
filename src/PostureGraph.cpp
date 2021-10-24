#include "pch.h"
#include <boost/config.hpp>
#include <iostream>
#include <algorithm>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/graph/adj_list_serialize.hpp>
#include <boost/property_map/property_map.hpp>
#include <string>
#include <boost/graph/graphviz.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include "filesystem_helper.hpp"
#include "PostureGraph.hpp"
#include "Math.hpp"
#include "ArtiBody.hpp"

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

template<typename VertexData, typename EdgeData>
class PostureGraphList
	: public boost::adjacency_list< boost::vecS
								, boost::vecS
								, boost::undirectedS
								, VertexData
								, EdgeData >
{
protected:
	PostureGraphList(std::size_t n_vertices)
		: boost::adjacency_list< boost::vecS
								, boost::vecS
								, boost::undirectedS
								, VertexData
								, EdgeData >(n_vertices)
	{
	}
	~PostureGraphList() {};

};

template<typename VertexData, typename EdgeData>
class PostureGraphMatrix
	: public boost::adjacency_matrix< boost::undirectedS
									, VertexData
									, EdgeData >
{
protected:
	PostureGraphMatrix(std::size_t n_vertices)
					: boost::adjacency_matrix<boost::undirectedS
											, VertexData
											, EdgeData >(n_vertices)
	{
	}
	~PostureGraphMatrix() {};

};


class CPostureGraphClose : public IPostureGraph
						 , public PostureGraphList<boost::no_property, boost::no_property>
{
	friend class CPostureGraphOpen;
public:
	struct Registry
	{
		std::size_t Register_v(std::size_t v_src)
		{
			auto it_src2dst = V_map.find(v_src);
			bool exists = (V_map.end() != it_src2dst);
			if (exists)
				return it_src2dst->second;
			else
			{
				std::size_t v_dst = V.size();
				V_map[v_src] = v_dst;
				V.push_back({ v_src, v_dst });
				return v_dst;
			}
		}

		void Register_e(std::size_t v_0, std::size_t v_1)
		{
			E.push_back({ v_0, v_1 });
		}

		typedef struct
		{
			std::size_t v_src;
			std::size_t v_dst;
		} REGISTER_v;

		typedef struct
		{
			std::size_t v_0;
			std::size_t v_1;
		} REGISTER_e;

		std::map<std::size_t, std::size_t> V_map;
		std::list<REGISTER_v> V;

		std::list<REGISTER_e> E;
	};
protected:
	CPostureGraphClose(std::size_t n_vs, const CFile2ArtiBody* theta_src)
		: PostureGraphList<boost::no_property, boost::no_property>(n_vs)
		, c_thetaSrc_ref(theta_src)
		, m_thetaBody(theta_src->CreateBody(BODY_TYPE::htr))
		, m_thetaFile(m_thetaBody, (int)n_vs)
	{
	}

	static void Initialize(CPostureGraphClose& graph, const Registry& reg, const Eigen::MatrixXr& errTB_src)
	{
		struct V_ERR
		{
			vertex_descriptor v_dst;
			Real err_0;
		};
		class V_ERRCompare {
		public:
			bool operator()(const V_ERR& v_err_1, const V_ERR& v_err_2)
			{
				return v_err_1.err_0 < v_err_2.err_0;
			}
		};

		std::list<V_ERR> lstV_ERR0;
		auto it_reg_v = reg.V.begin();
		IKAssert(0 == it_reg_v->v_src
				&& 0 == it_reg_v->v_dst);
		for (it_reg_v ++; it_reg_v != reg.V.end(); it_reg_v ++) // skip the 'T' posture to avoid a self-pointing edge
		{
			const auto& reg_v = *it_reg_v;
			graph.c_thetaSrc_ref->UpdateMotion((int)reg_v.v_src, graph.m_thetaBody);
			graph.m_thetaFile.UpdateMotion((int)reg_v.v_dst);
			lstV_ERR0.push_back({reg_v.v_dst, errTB_src(0, reg_v.v_src)});
		}

		for (Registry::REGISTER_e reg_e : reg.E)
		{
			boost::add_edge(reg_e.v_0, reg_e.v_1, graph);
		}
//#if defined _DEBUG
		Dump(graph, __FILE__, __LINE__);
//#endif

		lstV_ERR0.sort(V_ERRCompare()); // by err_0

		std::size_t deg_sigma = 0;
		std::size_t n_vs = 0;
		auto v_range = boost::vertices(graph);
		for (auto it_v = v_range.first; it_v != v_range.second; it_v ++, n_vs ++)
			deg_sigma += boost::degree(*it_v, graph);
		std::size_t deg_average = (std::size_t)floor((Real)deg_sigma/(Real)n_vs);

		std::size_t n_cnn_T = 0;
		for (auto it_v_err = lstV_ERR0.begin()
			; it_v_err != lstV_ERR0.end()
			  && n_cnn_T < deg_average
			; it_v_err ++, n_cnn_T ++)
			boost::add_edge(0, it_v_err->v_dst, graph);
//#if defined _DEBUG
		Dump(graph, __FILE__, __LINE__);
//#endif

	}
public:
	virtual ~CPostureGraphClose()
	{
		CArtiBodyTree::Destroy(m_thetaBody);
	}

	virtual void Save(const char* dir)
	{
		fs::path dot_path(dir);
		std::string dot_file_name(m_thetaBody->GetName_c()); dot_file_name += ".pg";
		dot_path.append(dot_file_name);
		std::ofstream dot_file(dot_path);
		IKAssert(std::ios_base::failbit != dot_file.rdstate());
		// write_graphviz(dot_file, *this);
		boost::archive::text_oarchive oa(dot_file);
		// oa << *this;
		boost::serialization::save(oa, *this, 0);

		fs::path htr_path(dir);
		std::string htr_file_name(m_thetaBody->GetName_c()); htr_file_name += ".htr";
		htr_path.append(htr_file_name);
		m_thetaFile.WriteBvhFile(htr_path.u8string().c_str());

	}
private:
	const CFile2ArtiBody* c_thetaSrc_ref;
	CArtiBodyNode* m_thetaBody;
	CArtiBody2File m_thetaFile;
};

struct VertexGen
{
	std::size_t deg;
	bool tag_rm;
};

struct EdgeGen
{
	std::size_t deg;
};

class CPostureGraphOpen : public IPostureGraph
						, public PostureGraphMatrix<VertexGen, EdgeGen>

{
public:
	CPostureGraphOpen(const CFile2ArtiBody* theta)
		: PostureGraphMatrix< VertexGen, EdgeGen>((std::size_t)(theta->frames()))
		, m_theta(theta)
	{
	}

	virtual ~CPostureGraphOpen()
	{
	}

	static void InitTransitions(CPostureGraphOpen& graph, const Eigen::MatrixXr& errTB, Real epsErr_deg)
	{
		const int i_frame_start = 1; //to skip the 'T' posture
		const int i_frame_end = graph.m_theta->frames() - 1;
		for (int i_frame = i_frame_start; i_frame < i_frame_end; i_frame++)
			boost::add_edge(i_frame, i_frame + 1, graph);

// #if defined _DEBUG
		Dump(graph, __FILE__, __LINE__);
// #endif

		// err_epsilon = (1-cos(theta_eps_deg*deg2rad/2))*65535;
		Real err_epsilon = (1 - cos(deg2rad(epsErr_deg) / (Real)2));
		std::size_t n_thetas = errTB.rows();
		for (std::size_t i_theta = 1; i_theta < n_thetas; i_theta++) //to skip the 'T' posture
		{
			for (std::size_t j_theta = i_theta + 1; j_theta < n_thetas; j_theta++)
			{
				if (errTB(i_theta, j_theta) < err_epsilon)
					boost::add_edge(i_theta, j_theta, graph);
			}
		}

// #if defined _DEBUG
		Dump(graph, __FILE__, __LINE__);
// #endif
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
		for (edge_iterator it_e = e_range.first; it_e != it_e_end; it_e ++)
		{
			auto e = *it_e;
			vertex_descriptor v[] = { boost::source(e, graph), boost::target(e, graph) };
			(graph)[v[0]].deg ++; (graph)[v[1]].deg ++;
		}

		// tag edge degrees and sort edges by degree
		std::list<edge_descriptor> edges_eps;
		for (edge_iterator it_e = e_range.first; it_e != it_e_end; it_e ++)
		{
			auto e = *it_e;
			edges_eps.push_back(e);
			vertex_descriptor v[] = {boost::source(e, graph), boost::target(e, graph)};
			std::size_t deg[] = {(graph)[v[0]].deg, (graph)[v[1]].deg };
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
				vertex_descriptor v[] = {boost::source(e, graph), boost::target(e, graph)};
				std::size_t deg[] = {(graph)[v[0]].deg, (graph)[v[1]].deg };
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
				vertex_descriptor v[] = {boost::source(e, graph), boost::target(e, graph)};
				bool rm[] = { (graph)[v[0]].tag_rm, (graph)[v[1]].tag_rm };
				if (rm[0] || rm[1])
				{
					(graph)[v[0]].deg --; (graph)[v[1]].deg --;
					it_e = edges_eps.erase(it_e);
				}
				else
					it_e ++;
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

// #if defined _DEBUG
		Dump(graph, __FILE__, __LINE__);
// #endif


	}

	static CPostureGraphClose* GenerateClosePG(const CPostureGraphOpen& graph_src, const Eigen::MatrixXr& errTB)
	{
		CPostureGraphClose::Registry regG;
		regG.Register_v(0); // 0 posture is reserved for 'T' posture which has no edges
		auto e_range_src = boost::edges(graph_src);
		const edge_iterator it_e_end_src = e_range_src.second;
		for (auto it_e_src = e_range_src.first
			; it_e_src != it_e_end_src
			; it_e_src++)
		{
			auto e_src = *it_e_src;
			vertex_descriptor v_src[] = { boost::source(e_src, graph_src), boost::target(e_src, graph_src) };
			CPostureGraphClose::vertex_descriptor v_dst[] = { regG.Register_v(v_src[0]), regG.Register_v(v_src[1]) };
			regG.Register_e(v_dst[0], v_dst[1]);
		}
		CPostureGraphClose* graph_dst = new CPostureGraphClose(regG.V.size(), graph_src.Theta());
		CPostureGraphClose::Initialize(*graph_dst, regG, errTB);
		return graph_dst;
	}

	virtual void Save(const char* dir)
	{
		fs::path dot_path(dir);
		std::string dot_file_name(m_theta->root_joint()->name()); dot_file_name += ".dot";
		dot_path.append(dot_file_name);
		std::ofstream dot_file(dot_path);
		IKAssert(std::ios_base::failbit != dot_file.rdstate());
		write_graphviz(dot_file, *this);
	}

	const CFile2ArtiBody* Theta() const { return m_theta; }
private:
	const CFile2ArtiBody* m_theta;
};





IPostureGraph* CPostureGraphGen::Generate(const CFile2ArtiBody* theta, const Eigen::MatrixXr& errTB, Real epsErr_deg)
{
	CPostureGraphOpen e_epsilon(theta);
	CPostureGraphOpen::InitTransitions(e_epsilon, errTB, epsErr_deg);
	CPostureGraphClose* pg = CPostureGraphOpen::GenerateClosePG(e_epsilon, errTB);
	// CPostureGraphClose* p_g = new PostureGraphClose(theta, vertices, edges);
	return pg;
}

void CPostureGraphGen::Destroy(IPostureGraph* pg)
{
	delete pg;
}

