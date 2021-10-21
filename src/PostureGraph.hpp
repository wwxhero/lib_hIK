#include "ArtiBodyFile.hpp"
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