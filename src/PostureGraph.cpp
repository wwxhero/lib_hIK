#include "pch.h"
#include "filesystem_helper.hpp"
#include "PostureGraph.hpp"
#include "Math.hpp"

template<typename VertexData, typename EdgeData>
class PostureGraphClose
	: public boost::adjacency_list< boost::vecS
								, boost::vecS
								, boost::undirectedS
								, VertexData
								, EdgeData >
{
protected:
	PostureGraphClose(std::size_t n_vertices)
		: boost::adjacency_list< boost::vecS
								, boost::vecS
								, boost::undirectedS
								, VertexData
								, EdgeData >(n_vertices)
	{
	}
	~PostureGraphClose() {};
};



struct VertexGen
{
	bool tag_rm;
};

struct EdgeGen
{
	int deg;
};

IPostureGraph* CPostureGraphGen::Generate(const CFile2ArtiBody* theta, const Eigen::MatrixXr& errTB, Real epsErr_deg)
{
	CPostureGraphOpen* e_epsilon = new CPostureGraphOpen(theta);
	e_epsilon->InitTransitions();
// #if defined _DEBUG
	e_epsilon->Dump(__LINE__);
// #endif

	// err_epsilon = (1-cos(theta_eps_deg*deg2rad/2))*65535;
	Real err_epsilon = (1-cos(deg2rad(epsErr_deg)/(Real)2));
	int n_thetas = errTB.rows();
	for (int i_theta = 0; i_theta < n_thetas; i_theta ++)
	{
		for (int j_theta = i_theta + 1; j_theta < n_thetas; j_theta ++)
		{
			if (errTB(i_theta, j_theta) < err_epsilon)
				boost::add_edge(i_theta, j_theta, *e_epsilon);
		}
	}

// #if defined _DEBUG
	e_epsilon->Dump(__LINE__);
// #endif

	return e_epsilon;
}

void CPostureGraphGen::Destroy(IPostureGraph* pg)
{
	delete pg;
}

