#pragma once

template<typename TGraphGen, typename TVdesc, typename TEdesc>
class PGGenHelper
{
	typedef TVdesc vertex_descriptor;
	typedef TEdesc edge_descriptor;
	template<typename TGraphGen, typename TVdesc, typename TEdesc>
	class ComEdgeByDeg
	{
		typedef TVdesc vertex_descriptor;
		typedef TEdesc edge_descriptor;
	public:
		ComEdgeByDeg(TGraphGen& g)
			: graph(g)
		{

		}
		bool operator()(const edge_descriptor& e_i, const edge_descriptor& e_j)
		{
			auto deg_i = (graph)[e_i].deg;
			auto deg_j = (graph)[e_j].deg;
			return (deg_i > deg_j)
				|| (deg_i == deg_j &&
					(std::max(boost::source(e_i, graph), boost::target(e_i, graph))
						> std::max(boost::source(e_j, graph), boost::target(e_j, graph))));
		}
	private:
		TGraphGen& graph;
	};

public:
	static void EliminateDupTheta(TGraphGen& graph_eps, const std::vector<std::pair<int, int>>& transi_0, const IErrorTB* errTB, Real epsErr_deg)
	{
		//tag rm for each vertex
		auto v_range = boost::vertices(graph_eps);
		auto it_v_end = v_range.second;
		for (auto it_v = v_range.first; it_v != it_v_end; it_v++)
		{
			auto& v_property = (graph_eps)[*it_v];
			v_property.tag_rm = false;
			v_property.deg = 0;
		}

		// compute degree for the vertex (work around for adjacent matrix)
		auto e_range = boost::edges(graph_eps);
		const auto it_e_end = e_range.second;
		for (auto it_e = e_range.first; it_e != it_e_end; it_e++)
		{
			auto e = *it_e;
			vertex_descriptor v[] = { boost::source(e, graph_eps), boost::target(e, graph_eps) };
			(graph_eps)[v[0]].deg++; (graph_eps)[v[1]].deg++;
		}

		// tag edge degrees and sort edges by degree
		std::list<edge_descriptor> edges_eps;
		for (auto it_e = e_range.first; it_e != it_e_end; it_e++)
		{
			auto e = *it_e;
			edges_eps.push_back(e);
			vertex_descriptor v[] = { boost::source(e, graph_eps), boost::target(e, graph_eps) };
			std::size_t deg[] = { (graph_eps)[v[0]].deg, (graph_eps)[v[1]].deg };
			auto& e_property = (graph_eps)[e];
			e_property.deg = std::max(deg[0], deg[1]);
		}

		edges_eps.sort(ComEdgeByDeg<TGraphGen, vertex_descriptor, edge_descriptor>(graph_eps));
#ifdef _DEBUG
		LOGIKVarErr(LogInfoInt, edges_eps.size());
#endif

		// tag for vertices removal
		while (!edges_eps.empty())
		{
			bool exists_a_tagged_vertex = false;
			for (auto e : edges_eps)
			{
				vertex_descriptor v[] = { boost::source(e, graph_eps), boost::target(e, graph_eps) };
				std::size_t deg[] = { (graph_eps)[v[0]].deg, (graph_eps)[v[1]].deg };
				bool rm[] = { (graph_eps)[v[0]].tag_rm, (graph_eps)[v[1]].tag_rm };
				if (!rm[0] && !rm[1])
				{
					if (deg[0] < deg[1])
					{
						(graph_eps)[v[1]].tag_rm = true;
						exists_a_tagged_vertex = true;
					}
					else if (deg[0] > deg[1])
					{
						(graph_eps)[v[0]].tag_rm = true;
						exists_a_tagged_vertex = true;
					}
				}
			}
			if (!exists_a_tagged_vertex && !edges_eps.empty())
			{
				edge_descriptor e_sym_broker = *edges_eps.begin();
				vertex_descriptor v_sym_broker = std::max(boost::source(e_sym_broker, graph_eps)
														, boost::target(e_sym_broker, graph_eps));
				(graph_eps)[v_sym_broker].tag_rm = true;
#ifdef _DEBUG
				LOGIKVarErr(LogInfoInt, v_sym_broker);
				LOGIKVarErr(LogInfoInt, edges_eps.size());
#endif
			}

			for (auto it_e = edges_eps.begin()
				; it_e != edges_eps.end()
				; )
			{
				auto e = *it_e;
				vertex_descriptor v[] = { boost::source(e, graph_eps), boost::target(e, graph_eps) };
				bool rm[] = { (graph_eps)[v[0]].tag_rm, (graph_eps)[v[1]].tag_rm };
				if (rm[0] || rm[1])
				{
					(graph_eps)[v[0]].deg--; (graph_eps)[v[1]].deg--;
					it_e = edges_eps.erase(it_e);
				}
				else
					it_e++;
			}
		}

		// remove vertices
		TGraphGen& graph = graph_eps;

		for (auto e_0 : transi_0)
			boost::add_edge(e_0.first, e_0.second, graph);


		for (auto it_v = v_range.first; it_v != it_v_end; it_v++)
		{
			const auto& v_property = (graph)[*it_v];
			if (v_property.tag_rm)
			{
				graph.Remove(*it_v, errTB);
			}
		}

	}

	static void InitTransitions(TGraphGen& graph, const IErrorTB* errTB, Real epsErr_deg)
	{
		// initialize epsilon edges
		int n_theta = graph.Theta()->N_Theta();
		int i_theta = 1;
		for (int i_theta_p = i_theta + 1; i_theta_p < n_theta; i_theta ++, i_theta_p ++)
			boost::add_edge(i_theta, i_theta + 1, graph);

	#if defined _DEBUG
		Dump(graph, __FILE__, __LINE__);
	#endif

		Real err_epsilon = (1 - cos(deg2rad(epsErr_deg) / (Real)2));
		IKAssert(errTB->N_Theta() == n_theta);
		int n_transi_eps = 0;
		for (i_theta = 1; i_theta < n_theta; i_theta++)
		{
			for (int j_theta = i_theta + 2; j_theta < n_theta; j_theta++) // (i, i+1) is already in epsilon edges
			{
				if (errTB->Get(i_theta, j_theta) < err_epsilon)
				{
					boost::add_edge(i_theta, j_theta, graph);
					n_transi_eps ++;
				}
			}
		}

	#if defined _DEBUG
		LOGIKVarErr(LogInfoInt, n_transi_eps);
		Dump(graph, __FILE__, __LINE__);
	#endif

		if (n_transi_eps > 0)
		{
			// initialize not epsilon edges which is phi for homo pg generation
			std::vector<std::pair<int, int>> transi_0;
			// E = phi, E_eps = {(i, i+1)| i in THETA} U {(i, j) | Error(i, j) < err_eps}
			EliminateDupTheta(graph, transi_0, errTB, epsErr_deg);
		}

	#if defined _DEBUG
		Dump(graph, __FILE__, __LINE__);
	#endif
	}

	static bool MergeTransitions(TGraphGen& graph, const CPGTransition& pg_0, const CPGTransition& pg_1, const IErrorTB* errTB, Real epsErr_deg, int n_theta_0, int n_theta_1)
	{
		// initialize not epsilon edges
		const CPGTransition* pgs[] = {&pg_0, &pg_1};
		int i_v_base[] = {0, (int)boost::num_vertices(pg_0)};
		std::vector<std::pair<int, int>> transi_0(boost::num_edges(pg_0) + boost::num_edges(pg_1));
		int n_transi = 0;
		for (int i_pg = 0; i_pg < 2; i_pg ++)
		{
			int i_v_base_i = i_v_base[i_pg];
			auto pg_i = pgs[i_pg];
			auto e_range_i = boost::edges(*pgs[i_pg]);
			for (CPGTransition::edge_iterator it_e = e_range_i.first; it_e != e_range_i.second; it_e++)
			{
				auto e = *it_e;
				int i_theta_0 = boost::source(e, *pg_i) + i_v_base_i;
				int i_theta_1 = boost::target(e, *pg_i) + i_v_base_i;
				bool incident_T = ((n_theta_0 == i_theta_0 || 0 == i_theta_0)
									|| (n_theta_0 == i_theta_1 || 0 == i_theta_1));
				if (!incident_T)
					transi_0[n_transi ++] = std::make_pair(i_theta_0, i_theta_1);
			}
		}
		transi_0.resize(n_transi);

		// initialize epsilon edges
		int n_theta = n_theta_0 + n_theta_1;
		IKAssert(n_theta == errTB->N_Theta());
		Real err_epsilon = (1 - cos(deg2rad(epsErr_deg) / (Real)2));
		int n_transi_eps = 0;
		for (int i_theta = 1; i_theta < n_theta_0; i_theta ++)
		{
			for (int j_theta = n_theta_0 + 1; j_theta < n_theta; j_theta ++ )
			{
				if (errTB->Get(i_theta, j_theta) < err_epsilon)
				{
					boost::add_edge(i_theta, j_theta, graph);
					n_transi_eps ++;
				}
			}
		}

		// E = (E_0 U E_1), E_eps != phi
		bool merge_able = (n_transi_eps > 0);
		if (merge_able)
			EliminateDupTheta(graph, transi_0, errTB, epsErr_deg);

		return merge_able;
	}

	static CPG* GeneratePG(const TGraphGen& graph_src)
	{
		CPG::Registry regG;
		regG.Register_v(0); // 'T' posture is the first posture registered which has no edges
		auto e_range_src = boost::edges(graph_src);
		for (auto it_e_src = e_range_src.first
			; it_e_src != e_range_src.second
			; it_e_src++)
		{
			auto e_src = *it_e_src;
			vertex_descriptor v_src[] = { boost::source(e_src, graph_src), boost::target(e_src, graph_src) };
			CPG::vertex_descriptor v_dst[] = { regG.Register_v(v_src[0]), regG.Register_v(v_src[1]) };
			regG.Register_e(v_dst[0], v_dst[1]);
		}
		CPG* graph_dst = new CPG(regG.V.size());
		CPG::Initialize(*graph_dst, regG, *graph_src.Theta());
		return graph_dst;
	}
};

typedef PGGenHelper<CPGMatrixGen, CPGMatrixGen::vertex_descriptor, CPGMatrixGen::edge_descriptor> CPGMatrixGenHelper;
typedef PGGenHelper<CPGListGen, CPGListGen::vertex_descriptor, CPGListGen::edge_descriptor> CPGListGenHelper;

template<typename TPGGen, typename TPGGenHelper>
CPG* generate_pg_homo(CPGTheta& theta, const std::list<std::string>& joints, Real epsErr)
{
	IErrorTB* err_tb = IErrorTB::Factory::CreateHOMO(theta, joints);
	TPGGen pg_epsilon(&theta);
	TPGGenHelper::InitTransitions(pg_epsilon, err_tb, epsErr);
	CPG* pg = TPGGenHelper::GeneratePG(pg_epsilon);
	IErrorTB::Factory::Release(err_tb);
	return pg;
}