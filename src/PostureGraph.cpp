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

	void InitTransitions(const Eigen::MatrixXr& errTB, Real epsErr_deg)
	{
		const int i_frame_start = 1; //to skip the 'T' posture
		const int i_frame_end = m_theta->frames() - 1;
		for (int i_frame = i_frame_start; i_frame < i_frame_end; i_frame++)
			boost::add_edge(i_frame, i_frame + 1, *this);

		// #if defined _DEBUG
		Dump(__FILE__, __LINE__);
		// #endif

		// err_epsilon = (1-cos(theta_eps_deg*deg2rad/2))*65535;
		Real err_epsilon = (1 - cos(deg2rad(epsErr_deg) / (Real)2));
		std::size_t n_thetas = errTB.rows();
		for (std::size_t i_theta = 0; i_theta < n_thetas; i_theta++)
		{
			for (std::size_t j_theta = i_theta + 1; j_theta < n_thetas; j_theta++)
			{
				if (errTB(i_theta, j_theta) < err_epsilon)
					boost::add_edge(i_theta, j_theta, *this);
			}
		}

		// #if defined _DEBUG
		Dump(__FILE__, __LINE__);
		// #endif
		//tag rm for each vertex
		auto v_range = boost::vertices(*this);
		vertex_iterator it_v = v_range.first;
		vertex_iterator it_v_end = v_range.second;
		for (; it_v != it_v_end; it_v++)
		{
			auto& v_property = (*this)[*it_v];
			v_property.tag_rm = false;
			v_property.deg = 0;
		}

		// compute degree for the vertex (work around for adjacent matrix)
		auto e_range = boost::edges(*this);
		edge_iterator it_e_end = e_range.second;
		for (edge_iterator it_e = e_range.first; it_e != it_e_end; it_e++)
		{
			auto e = *it_e;
			vertex_descriptor v[] = { boost::source(e, *this), boost::target(e, *this) };
			(*this)[v[0]].deg++; (*this)[v[1]].deg++;			
		}

		// tag edge degrees and sort edges by degree
		std::srand((unsigned int)std::time(0));
		std::vector<edge_descriptor> edges_eps;
		for (edge_iterator it_e = e_range.first; it_e != it_e_end; it_e++)
		{
			auto e = *it_e;
			edges_eps.push_back(e);
			vertex_descriptor v[] = {boost::source(e, *this), boost::target(e, *this)};
			std::size_t deg[] = {(*this)[v[0]].deg, (*this)[v[1]].deg };
			auto& e_property = (*this)[e];
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
		}
		std::sort(edges_eps.begin()
				, edges_eps.end()
				, [&](const edge_descriptor& e_i, const edge_descriptor& e_j)->bool
					{
						return (*this)[e_i].deg > (*this)[e_j].deg;
					});
		int n_removed = 0;
		for (auto e : edges_eps)
		{
			vertex_descriptor v[] = { boost::source(e, *this), boost::target(e, *this) };
			bool rm[] = { (*this)[v[0]].tag_rm, (*this)[v[1]].tag_rm };
			if (!rm[0] && !rm[1])
			{
				auto e_property = (*this)[e];
				(*this)[v[e_property.to_rm]].tag_rm = true;
				n_removed ++;
			}
		}

		LOGIKVar(LogInfoInt, n_removed);



	}

	void RemoveDUPs(std::list<V> &lstV, std::list<E> &lstE)
	{
		// int n_frames = m_theta->frames();
		// Eigen::MatrixXi cnn_map(n_frames, n_frames);
		// auto v_range = boost::vertices(*this);
		// vertex_iterator it_v = v_range.first;
		// vertex_iterator it_v_end = v_range.second;
		// for (; it_v != it_v_end; it_v++)
		// {
		// 	vertex_descriptor v = *it_v;
		// 	bool rm = (*this)[v].tag_rm;
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
	e_epsilon->InitTransitions(errTB, epsErr_deg);
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

