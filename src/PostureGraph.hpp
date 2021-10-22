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


class CPostureGraphOpen : public IPostureGraph
						, public boost::adjacency_list< boost::vecS
													, boost::vecS
													, boost::undirectedS >

{
public:
	CPostureGraphOpen(const CFile2ArtiBody* theta)
		: boost::adjacency_list< boost::vecS
								, boost::vecS
								, boost::undirectedS >((std::size_t)(theta->frames()))
		, m_theta(theta)
	{
	}

	virtual ~CPostureGraphOpen()
	{
	}

	void InitTransitions()
	{
		const int i_frame_start = 1; //to skip the 'T' posture
		const int i_frame_end = m_theta->frames() - 1;
		for (int i_frame = i_frame_start; i_frame < i_frame_end; i_frame++)
			boost::add_edge(i_frame, i_frame + 1, *this);
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

	void Dump(int lineNo)
	{
		std::stringstream dot_path;
		dot_path << lineNo << ".dot";
		std::ofstream dot_file(dot_path.str());
		IKAssert(std::ios_base::failbit != dot_file.rdstate());
		write_graphviz(dot_file, *this);
	}

private:
	const CFile2ArtiBody* m_theta;
};

class CPostureGraphGen
{
public:
	static IPostureGraph* Generate(const CFile2ArtiBody* theta, const Eigen::MatrixXr& errTB, Real epsErr);
	static void Destroy(IPostureGraph* pg);
};
