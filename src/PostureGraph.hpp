#pragma once
#pragma warning(push)
#pragma warning(disable: 4522 267)
#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/graph/adj_list_serialize.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#pragma warning(pop)
#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <queue>
#include "ArtiBodyFile.hpp"
#include "filesystem_helper.hpp"
#include "Math.hpp"
#include "ArtiBody.hpp"
#include "IKChain.hpp"


enum PG_FileType {F_PG = 0, F_DOT};

// it should be optimized to get rid of derivation to avoid unnecessary memory consumption from BvhObject from runtime.
class CThetaArtiBodyRef
{
public:
	CThetaArtiBodyRef(const char* path, CArtiBodyNode* body_ref);
	CThetaArtiBodyRef(const std::string& path, CArtiBodyNode* body_ref);
	template<bool G_SPACE>
	void PoseBody(int i_frame) const
	{
		const TransformArchive& motion_i = m_motions[i_frame];
		std::size_t n_tms = m_jointsRef.size();
		for (std::size_t j_tm = 0; j_tm < n_tms; j_tm ++)
		{
			const _TRANSFORM& tm_ij = motion_i[j_tm];
			IJoint* joint_j = m_jointsRef[j_tm];
			joint_j->GetTransform()->CopyFrom(tm_ij);
		}
		CArtiBodyTree::FK_Update<G_SPACE>(m_rootRef);
	}
	int frames() const
	{
		return (int)m_motions.size();
	}
private:
	void Initialize(const std::string& path, CArtiBodyNode* body_std);

private:
	std::vector<IJoint*> m_jointsRef;
	CArtiBodyNode* m_rootRef;
	std::vector<TransformArchive> m_motions;
};

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

struct VertexSearch : public boost::no_property
{
	Real err;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		err = CIKChain::ERROR_MIN;
		ar & err;
		err = CIKChain::ERROR_MIN;
	}
};

inline bool ErrorTagged(const VertexSearch& v_prop)
{
	return (v_prop.err > CIKChain::ERROR_MIN);
}

inline bool ErrorTagged(Real err)
{
	return (err > CIKChain::ERROR_MIN);
}

inline void EraseTag(VertexSearch& v_prop)
{
	v_prop.err = CIKChain::ERROR_MIN;
}

class CPGTransition : public PostureGraphList<VertexSearch, boost::no_property>
{
public:
	CPGTransition(std::size_t n_vs)
		: PostureGraphList<VertexSearch, boost::no_property>(n_vs)
	{
	}

	void SaveTransitions(const char* filePath, PG_FileType type) const
	{
		std::ofstream file(filePath, std::ofstream::binary);
		IKAssert(std::ios_base::failbit != file.rdstate());
		if (std::ios_base::failbit == file.rdstate())
		{
			LOGIKVarErr(LogInfoCharPtr, filePath);
			return;
		}
		if (F_DOT == type)
			write_graphviz(file, *this);
		else
		{
			boost::archive::binary_oarchive oa(file);
			boost::serialization::save(oa, *this, (unsigned int)0);
		}
	}

	bool LoadTransitions(const char* filePath)
	{
		std::ifstream file(filePath, std::ifstream::binary);
		bool loaded = (std::ios_base::failbit != file.rdstate());
		if (loaded)
		{
			boost::archive::binary_iarchive ia(file);
			boost::serialization::load(ia, *this, (unsigned int)0);
		}
		return loaded;
	}
};


class CPGClose : public CPGTransition
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

	private:
		std::map<std::size_t, std::size_t> V_map;
	public:
		std::list<REGISTER_v> V;
		std::list<REGISTER_e> E;
	};
protected:
	CPGClose(std::size_t n_vs, const CThetaArtiBody* theta_src);
	static void Initialize(CPGClose& graph_src, const Registry& reg, const Eigen::MatrixXr& errTB_src, int pid_T_src);
public:
	virtual ~CPGClose();

	void Save(const char* dir) const;

private:
	const CThetaArtiBody* c_thetaSrc_ref;
	CArtiBodyRef2File m_thetaFile;
};




class CPGRuntime : public CPGTransition
{
public:
	CPGRuntime()
		: CPGTransition(0)
		, m_thetas(NULL)
		, m_theta_star(0)
	{
	}

	~CPGRuntime()
	{
		if (NULL != m_thetas)
			delete m_thetas;
	}
	bool Load(const char* dir, CArtiBodyNode* rootBody);

	template<bool G_SPACE>
	int SetActivePosture(int pose_id, bool UpdatePose)
	{
		int pose_id_m = m_theta_star;
		m_theta_star = pose_id;
		if (UpdatePose)
		{
			m_thetas->PoseBody<G_SPACE>(pose_id);
		}
		return pose_id_m;
	}

	template<typename LAMBDA_Err>
	static int LocalMin(CPGRuntime& graph, LAMBDA_Err err)
	{
		auto ErrTheta = [&graph, err](vertex_descriptor theta) -> Real
			{
				graph.m_thetas->PoseBody<true>(theta);
				return err();
			};

		class GreatorThetaErr
		{
		public:
			explicit GreatorThetaErr(const CPGRuntime& a_graph)
				: m_graph_ref(a_graph)
			{
			}
			bool operator()(const vertex_descriptor& left, const vertex_descriptor& right) const
			{
				return m_graph_ref[left].err > m_graph_ref[right].err;
			}
		private:
			const CPGRuntime& m_graph_ref;
		} greator_thetaErr(graph);

		vertex_descriptor theta_star_k = graph.m_theta_star;
		IKAssert(!ErrorTagged(graph[theta_star_k]));
		LOGIKVar(LogInfoInt, theta_star_k);


		std::priority_queue<vertex_descriptor, std::vector<vertex_descriptor>, GreatorThetaErr> err_known (greator_thetaErr);
		// std::queue<vertex_descriptor> err_known;
		std::list<vertex_descriptor> tagged;
		graph[theta_star_k].err = ErrTheta(theta_star_k);
		tagged.push_back(theta_star_k);
		err_known.push(theta_star_k);

		struct ThetaErr
		{
			vertex_descriptor theta;
			Real err;
		} theta_err_kp = {boost::num_vertices(graph), REAL_MAX};
		int n_min = 0;

		const int N_CANDIDATES = 10;
		bool descend_local_min = true;
		while (!err_known.empty()
			&& descend_local_min)
		{
			vertex_descriptor theta = err_known.top();
			err_known.pop();
			// vertex_descriptor theta = err_known.front();
			Real err = graph[theta].err;
			bool local_min = true;
			auto vertices_range_neighbors = boost::adjacent_vertices(theta, graph);
			for (auto it_v_n = vertices_range_neighbors.first
				; it_v_n != vertices_range_neighbors.second
				; it_v_n ++)
			{
				vertex_descriptor theta_n = *it_v_n;
				Real& err_n = graph[theta_n].err;
				if (!ErrorTagged(err_n))
				{
					err_n = ErrTheta(theta_n);
					tagged.push_back(theta_n);
					err_known.push(theta_n);
					LOGIKVar(LogInfoInt, theta_n);
					LOGIKVar(LogInfoReal, err_n);
				}
				local_min = local_min && (err < err_n);
			}

			LOGIKVar(LogInfoInt, theta);

			if (local_min)
			{
				descend_local_min = (err < theta_err_kp.err);
				if (descend_local_min)
				{
					theta_err_kp.theta = theta;
					theta_err_kp.err = err;
				}
				descend_local_min = (descend_local_min || n_min < N_CANDIDATES);
				n_min ++;
				LOGIKVar(LogInfoInt, theta);
				LOGIKVar(LogInfoReal, err);
			}
		}

		for (auto theta_tagged : tagged)
			EraseTag(graph[theta_tagged]);
		IKAssert(-1 < (int)theta_err_kp.theta
			&& (int)theta_err_kp.theta < graph.m_thetas->frames());

		return (int)theta_err_kp.theta;
	}

private:
	bool LoadThetas(const char* filePath, CArtiBodyNode* body_ref);

	CThetaArtiBodyRef* m_thetas;
	vertex_descriptor m_theta_star;

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
	CPostureGraphOpen(const CThetaArtiBody* theta);

	virtual ~CPostureGraphOpen();

	static void InitTransitions(CPostureGraphOpen& graph, const Eigen::MatrixXr& errTB, Real epsErr_deg, const std::vector<int>& postureids_ignore);

	static CPGClose* GenerateClosePG(const CPostureGraphOpen& graph_src, const Eigen::MatrixXr& errTB, int pid_T_src);

	void Save(const char* dir, PG_FileType type = F_PG) const;

	const CThetaArtiBody* Theta() const { return m_theta; }
private:
	const CThetaArtiBody* m_theta;
};


