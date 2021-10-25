#pragma once
#include "ArtiBodyFile.hpp"
#include <boost/config.hpp>
#include <iostream>
#include <algorithm>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/graph/adj_list_serialize.hpp>
#include <boost/property_map/property_map.hpp>
#include <string>
#include <boost/graph/graphviz.hpp>
// #include <boost/archive/binary_oarchive.hpp>
// #include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include "filesystem_helper.hpp"
#include "Math.hpp"
#include "ArtiBody.hpp"

enum PG_FileType {F_PG = 0, F_DOT};

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

class CPostureGraphClose : public PostureGraphList<boost::no_property, boost::no_property>
{
public:
	CPostureGraphClose(std::size_t n_vs)
		: PostureGraphList<boost::no_property, boost::no_property>(n_vs)
	{
	}

	void SaveTransitions(const char* filePath, PG_FileType type) const
	{
		std::ofstream file(filePath);
		IKAssert(std::ios_base::failbit != file.rdstate());
		if (F_DOT == type)
			write_graphviz(file, *this);
		else
		{
			boost::archive::text_oarchive oa(file);
			boost::serialization::save(oa, *this, (unsigned int)0);
		}
	}

	void LoadTransitions(const char* filePath)
	{
		std::ifstream file(filePath);
		IKAssert(std::ios_base::failbit != file.rdstate());
		boost::archive::text_iarchive ia(file);
		boost::serialization::load(ia, *this, (unsigned int)0);
	}
};


class CPostureGraphClose2File : public CPostureGraphClose
{
	friend class CPostureGraphOpen;
public:
	struct Registry
	{
		std::size_t Register_v(std::size_t v_src)
		{
			auto it_src2dst = V_map.find(v_src);
			bool exists = (V_map.end() != it_src2dst);
			if (exists)
				return it_src2dst->second;
			else
			{
				std::size_t v_dst = V.size();
				V_map[v_src] = v_dst;
				V.push_back({ v_src, v_dst });
				return v_dst;
			}
		}

		void Register_e(std::size_t v_0, std::size_t v_1)
		{
			E.push_back({ v_0, v_1 });
		}

		typedef struct
		{
			std::size_t v_src;
			std::size_t v_dst;
		} REGISTER_v;

		typedef struct
		{
			std::size_t v_0;
			std::size_t v_1;
		} REGISTER_e;

		std::map<std::size_t, std::size_t> V_map;
		std::list<REGISTER_v> V;

		std::list<REGISTER_e> E;
	};
protected:
	CPostureGraphClose2File(std::size_t n_vs, const CFile2ArtiBody* theta_src);
	static void Initialize(CPostureGraphClose2File& graph, const Registry& reg, const Eigen::MatrixXr& errTB_src);
public:
	virtual ~CPostureGraphClose2File();

	void Save(const char* dir, PG_FileType type = F_PG) const;

private:
	const CFile2ArtiBody* c_thetaSrc_ref;
	CArtiBodyNode* m_thetaBody;
	CArtiBody2File m_thetaFile;
};

class CFile2PostureGraphClose : public CPostureGraphClose
{
public:
	CFile2PostureGraphClose()
		: CPostureGraphClose(0)
	{
	}

};

struct VertexGen
{
	std::size_t deg;
	bool tag_rm;
};

struct EdgeGen
{
	std::size_t deg;
};

class CPostureGraphOpen : public PostureGraphMatrix<VertexGen, EdgeGen>

{
public:
	CPostureGraphOpen(const CFile2ArtiBody* theta);

	virtual ~CPostureGraphOpen();

	static void InitTransitions(CPostureGraphOpen& graph, const Eigen::MatrixXr& errTB, Real epsErr_deg);

	static CPostureGraphClose2File* GenerateClosePG(const CPostureGraphOpen& graph_src, const Eigen::MatrixXr& errTB);

	void Save(const char* dir, PG_FileType type = F_PG) const;

	const CFile2ArtiBody* Theta() const { return m_theta; }
private:
	const CFile2ArtiBody* m_theta;
};


