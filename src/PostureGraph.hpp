#include "ArtiBodyFile.hpp"
#include <boost/config.hpp>
#include <iostream>
#include <algorithm>
#include <boost/graph/adjacency_list.hpp>
#include <boost/property_map/property_map.hpp>
#include <string>
#include <boost/graph/graphviz.hpp>

class IPostureGraph
{
public:
	virtual ~IPostureGraph() {};
	virtual void Save(const char* dir) = 0;
	
};


class CPostureGraphGen
{
public:
	static IPostureGraph* Generate(const CFile2ArtiBody* theta, const Eigen::MatrixXr& errTB, Real epsErr);
	static void Destroy(IPostureGraph* pg);
};
