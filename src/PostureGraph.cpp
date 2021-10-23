#include "pch.h"
#include <boost/config.hpp>
#include <iostream>
#include <algorithm>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/property_map/property_map.hpp>
#include <string>
#include <boost/graph/graphviz.hpp>
#include "filesystem_helper.hpp"
#include "PostureGraph.hpp"
#include "Math.hpp"

typedef std::size_t V;
typedef struct
{
	V v_i;
	V v_j;
} E;

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



struct VertexGen
{
	std::size_t deg;
	bool tag_rm;
};

struct EdgeGen
{
	std::size_t deg;
	std::size_t to_rm;
	// std::size_t tag_rm;
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
		graph.Dump(__FILE__, __LINE__);
// #endif

		// err_epsilon = (1-cos(theta_eps_deg*deg2rad/2))*65535;
		Real err_epsilon = (1 - cos(deg2rad(epsErr_deg) / (Real)2));
		std::size_t n_thetas = errTB.rows();
		for (std::size_t i_theta = 0; i_theta < n_thetas; i_theta++)
		{
			for (std::size_t j_theta = i_theta + 1; j_theta < n_thetas; j_theta++)
			{
				if (errTB(i_theta, j_theta) < err_epsilon)
					boost::add_edge(i_theta, j_theta, graph);
			}
		}

// #if defined _DEBUG
		graph.Dump(__FILE__, __LINE__);
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
		edge_iterator it_e_end = e_range.second;
		for (edge_iterator it_e = e_range.first; it_e != it_e_end; it_e++)
		{
			auto e = *it_e;
			vertex_descriptor v[] = { boost::source(e, graph), boost::target(e, graph) };
			(graph)[v[0]].deg++; (graph)[v[1]].deg++;
		}

		// tag edge degrees and sort edges by degree
		std::srand((unsigned int)std::time(0));
		std::vector<edge_descriptor> edges_eps;
		for (edge_iterator it_e = e_range.first; it_e != it_e_end; it_e++)
		{
			auto e = *it_e;
			edges_eps.push_back(e);
			vertex_descriptor v[] = {boost::source(e, graph), boost::target(e, graph)};
			std::size_t deg[] = {(graph)[v[0]].deg, (graph)[v[1]].deg };
			auto& e_property = (graph)[e];
			if (v[0] < v[1])
			{
				e_property.deg = deg[1];
				e_property.to_rm = 1;
			}
			else if (v[0] > v[1])
			{
				e_property.deg = deg[0];
				e_property.to_rm = 0;
			}
			else
			{
				e_property.deg = deg[0] - 1; // to give nodes other chance to be decided
				e_property.to_rm = std::rand()&0x1;
			}
			// e_property.tag_rm = false;
		}
		std::sort(edges_eps.begin()
				, edges_eps.end()
				, [&](const edge_descriptor& e_i, const edge_descriptor& e_j)->bool
					{
						return (graph)[e_i].deg > (graph)[e_j].deg;
					});
		int n_removed = 0;
		for (auto e : edges_eps)
		{
			vertex_descriptor v[] = { boost::source(e, graph), boost::target(e, graph) };
			bool rm[] = { (graph)[v[0]].tag_rm, (graph)[v[1]].tag_rm };
			if (!rm[0] && !rm[1])
			{
				auto e_property = (graph)[e];
				(graph)[v[e_property.to_rm]].tag_rm = true;
				n_removed ++;
			}
		}
		LOGIKVar(LogInfoInt, n_removed);

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
		graph.Dump(__FILE__, __LINE__);
// #endif





	}

	void RemoveDUPs(std::list<V> &lstV, std::list<E> &lstE)
	{
		// int n_frames = m_theta->frames();
		// Eigen::MatrixXi cnn_map(n_frames, n_frames);
		// auto v_range = boost::vertices(graph);
		// vertex_iterator it_v = v_range.first;
		// vertex_iterator it_v_end = v_range.second;
		// for (; it_v != it_v_end; it_v++)
		// {
		// 	vertex_descriptor v = *it_v;
		// 	bool rm = (graph)[v].tag_rm;
		// 	if (!rm)
		// 		lstV.push_back(v);
		// 	else
		// 	{
		// 		// traverse all neighbor vertices
		// 	}
		// }
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

	void Dump(const char* fileName, int lineNo)
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
		write_graphviz(dot_file, *this);
	}

private:
	const CFile2ArtiBody* m_theta;
};

IPostureGraph* CPostureGraphGen::Generate(const CFile2ArtiBody* theta, const Eigen::MatrixXr& errTB, Real epsErr_deg)
{
	CPostureGraphOpen* e_epsilon = new CPostureGraphOpen(theta);
	CPostureGraphOpen::InitTransitions(*e_epsilon, errTB, epsErr_deg);
	std::list<V> vertices;
	std::list<E> edges;
	e_epsilon->RemoveDUPs(vertices, edges);
	// CPostureGraphClose* p_g = new PostureGraphClose(theta, vertices, edges);
	return e_epsilon;
}

void CPostureGraphGen::Destroy(IPostureGraph* pg)
{
	delete pg;
}

