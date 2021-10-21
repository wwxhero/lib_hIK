#include "ArtiBodyFile.hpp"
#include <boost/config.hpp>
#include <iostream>
#include <algorithm>
#include <boost/graph/adjacency_list.hpp>
#include <boost/property_map/property_map.hpp>
#include <string>
#include <boost/graph/graphviz.hpp>
class CPostureGraph
	: public CArtiBody2File
{
private:
	CPostureGraph();
	~CPostureGraph();
public:
	void Save(const char* dir);
	static CPostureGraph* Generate(const CFile2ArtiBody* theta);
	static void Destroy(CPostureGraph* pg);
};