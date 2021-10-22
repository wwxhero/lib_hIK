#include "pch.h"
#include "filesystem_helper.hpp"
#include "PostureGraph.hpp"


CPostureGraphGen* CPostureGraphGen::Generate(const CFile2ArtiBody* theta, const Eigen::MatrixXr& errTB)
{
	CPostureGraphGen* g_org = new CPostureGraphGen(std::size_t(theta->frames()));
	const int i_frame_start = 1; //to skip the 'T' posture
	const int i_frame_end = theta->frames() - 1;
	for (int i_frame = i_frame_start; i_frame < i_frame_end; i_frame++)
		boost::add_edge(i_frame, i_frame + 1, *g_org);
	return g_org;
}

void CPostureGraphGen::Destroy(CPostureGraphGen* pg)
{
	delete pg;
}

void CPostureGraphGen::Save(const char* dir)
{
	fs::path dot_path(dir);
	dot_path.append("arti.dot");
	std::ofstream dot_file(dot_path);
	IKAssert(ios_base::failbit == dot_file.rdstate());
	write_graphviz(dot_file, *this);
}