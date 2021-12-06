#pragma once
#pragma warning(push)
#pragma warning(disable: 4522 267)
#pragma push_macro("new")
#undef new
#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/graph/adj_list_serialize.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#pragma pop_macro("new")
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
#include "ErrorTB.hpp"

enum PG_FileType {F_PG = 0, F_DOT};

// it should be optimized to get rid of derivation to avoid unnecessary memory consumption from BvhObject from runtime.
class CPGThetaRuntime
{
public:
	CPGThetaRuntime(const char* path, CArtiBodyNode* body_ref);
	CPGThetaRuntime(const std::string& path, CArtiBodyNode* body_ref);
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
	int N_Theta() const
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

class CPGTheta
{
public:
	CPGTheta(const char* path);
	CPGTheta(const std::string& path);
	CPGTheta();
	CPGTheta(const CPGTheta& src);
	virtual ~CPGTheta();
public:
	template<bool G_SPACE>
	void PoseBody(int i_frame) const
	{
		PoseBody<G_SPACE>(i_frame, m_rootBody);
	}

	template<bool G_SPACE>
	void ResetPose() const
	{
		auto onEnterBodyReset = [](CArtiBodyNode* body)
			{
				IJoint* joint = body->GetJoint();
				joint->SetRotation(Eigen::Quaternionr::Identity());
				joint->SetTranslation(Eigen::Vector3r::Zero());
			};
		auto onLeaveBodyReset = [](CArtiBodyNode* body)
			{
			};
		CArtiBodyTree::TraverseDFS(m_rootBody, onEnterBodyReset, onLeaveBodyReset);
		CArtiBodyTree::FK_Update<G_SPACE>(m_rootBody);
	}

	struct Query
	{
		CArtiBodyNode* rootPose;
		std::list<const CArtiBodyNode*> interests;
		int n_interests;
	};

	Query* BeginQuery(const std::list<std::string>& joints) const;
	void EndQuery(Query* query) const;
	void QueryTheta(Query* query, int i_theta, TransformArchive& tm_data) const;

	int N_Theta() const {return (int)m_motions.size();}

	const CArtiBodyNode* GetBody() const { return m_rootBody; }
	CArtiBodyNode* GetBody() { return m_rootBody;  }

	bool Merge(const CPGTheta& f2b);
protected:
	template<bool G_SPACE>
	void PoseBody(int i_frame, CArtiBodyNode* body) const
	{
		IKAssert(i_frame < (int)m_motions.size());
		TransformArchive& tms_i = const_cast<TransformArchive&>(m_motions[i_frame]);
		CArtiBodyTree::Serialize<false>(body, tms_i);
		CArtiBodyTree::FK_Update<G_SPACE>(body);
	}

public:
	void Initialize(const CArtiBodyFile& abFile);
	static bool SmallX(int n_theta_0, int n_theta_1);
	static bool SmallXETB(int n_theta_0, int n_theta_1);
	static bool MedianXETB(int n_theta_0, int n_theta_1);
	static bool SmallHomo(int n_theta);
	static bool SmallHomoETB(int n_theta);

private:
	CArtiBodyNode* m_rootBody;
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

template<typename VertexData, typename EdgeData>
class PostureGraphEDynaList
	: public boost::adjacency_list< boost::listS
								, boost::vecS
								, boost::undirectedS
								, VertexData
								, EdgeData >
{
protected:
	PostureGraphEDynaList(std::size_t n_vertices)
		: boost::adjacency_list< boost::listS
								, boost::vecS
								, boost::undirectedS
								, VertexData
								, EdgeData >(n_vertices)
	{
	}
	~PostureGraphEDynaList() {};

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


class CPG : public CPGTransition
{
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

public:
	CPG(std::size_t n_vs);
	CPG();
	virtual ~CPG();

	static void Initialize(CPG& graph_src, const Registry& reg, const CPGTheta& theta_src);	
	bool Load(const char* dir, const char* pg_name);
	void Save(const char* dir) const;
	const CPGTheta& Theta() const
	{
		return m_theta;
	}
private:
	bool LoadThetas(const std::string& path_theta);
private:
	CPGTheta m_theta;
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

		const int N_CANDIDATES = 20;
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
			&& (int)theta_err_kp.theta < graph.m_thetas->N_Theta());

		return (int)theta_err_kp.theta;
	}

private:
	bool LoadThetas(const char* filePath, CArtiBodyNode* body_ref);

	CPGThetaRuntime* m_thetas;
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

template <typename GenBase>
class TPGGen : public GenBase
{
public:
	TPGGen(const CPGTheta* theta)
		: GenBase((std::size_t)(theta->N_Theta()))
		, m_theta(theta)
	{
	}
	virtual ~TPGGen()
	{
	}
	const CPGTheta* Theta() const { return m_theta; }
private:
	const CPGTheta* m_theta;
};

class CPGMatrixGen : public TPGGen<PostureGraphMatrix<VertexGen, EdgeGen>>
{
	typedef TPGGen<PostureGraphMatrix<VertexGen, EdgeGen>> Super;
public:
	typedef PostureGraphMatrix<VertexGen, EdgeGen>::vertex_descriptor vertex_descriptor;
	typedef PostureGraphMatrix<VertexGen, EdgeGen>::edge_descriptor edge_descriptor;
public:
	CPGMatrixGen(const CPGTheta* theta);
	void Remove(vertex_descriptor v, const IErrorTB* errTB);
};

class CPGListGen : public TPGGen<PostureGraphEDynaList<VertexGen, EdgeGen>>
{
	typedef TPGGen<PostureGraphEDynaList<VertexGen, EdgeGen>> Super;
public:
	typedef PostureGraphEDynaList<VertexGen, EdgeGen>::vertex_descriptor vertex_descriptor;
	typedef PostureGraphEDynaList<VertexGen, EdgeGen>::edge_descriptor edge_descriptor;
public:
	CPGListGen(const CPGTheta* theta);
	void Remove(vertex_descriptor v, const IErrorTB* errTB);
};


