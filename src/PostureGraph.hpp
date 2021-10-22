#include "ArtiBodyFile.hpp"
#include <boost/config.hpp>
#include <iostream>
#include <algorithm>
#include <boost/graph/adjacency_list.hpp>
#include <boost/property_map/property_map.hpp>
#include <string>
#include <boost/graph/graphviz.hpp>

template<typename VertexData, typename EdgeData>
class PostureGraphBase
	: public boost::adjacency_list< boost::vecS, boost::vecS, boost::undirectedS, VertexData, EdgeData >
	//, public CArtiBody2File
{

protected:
	PostureGraphBase(std::size_t n_vertices)
		: boost::adjacency_list< boost::vecS, boost::vecS, boost::undirectedS, VertexData, EdgeData >(n_vertices)
	{
	}
	~PostureGraphBase() {};
public:
	virtual void Save(const char* dir)
	{
	}
};


struct VertexGen
{
	bool tag_rm;
};

struct EdgeGen
{
	int deg;
};

class CPostureGraphGen : public PostureGraphBase<VertexGen, EdgeGen>
{
	typedef PostureGraphBase<VertexGen, EdgeGen> Super;
public:
	CPostureGraphGen(std::size_t n_vertices)
		: Super(n_vertices)
	{
	}
	~CPostureGraphGen() {};
	virtual void Save(const char* dir);
public:
	static CPostureGraphGen* Generate(const CFile2ArtiBody* theta, const Eigen::MatrixXr& errTB);
	static void Destroy(CPostureGraphGen* pg);
};
