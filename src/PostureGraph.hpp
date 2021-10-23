#include "ArtiBodyFile.hpp"

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
