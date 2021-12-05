#pragma once 

#include "parallel_thread_helper.hpp"

void UpdateXETB_Parallel_CPU(ETBRect* errTB, const CPGTheta& theta, const std::list<std::string>& joints)
{
	class ThreadXUpdator : public CThread_W32
	{
	public:
		ThreadXUpdator()
			: m_id(0)
			, m_etb(NULL)
			, m_theta(NULL)
		{
		}
		~ThreadXUpdator()
		{
		}
		void Initialize_main(int n_threads, int id, ETBRect* etb, const CPGTheta& theta, const std::list<std::string>& joints)
		{
			m_nthreads = n_threads;
			m_id = id;
			m_etb = etb;
			m_theta = &theta;
			m_joints = &joints;
		}
		void Kickoff_main()
		{
			Execute_main();
		}
	private:
		virtual void Run_worker()
		{
			CPGTheta::Query* query = m_theta->BeginQuery(*m_joints);
			TransformArchive tm_data_i(query->n_interests);
			TransformArchive tm_data_j(query->n_interests);
			int lenTB = m_etb->Length();
			for (int i_offset = m_id; i_offset < lenTB; i_offset += m_nthreads)
			{
				std::pair<int, int> ij_theta = m_etb->Theta_ij(i_offset);
				auto& i_theta = ij_theta.first;
				auto& j_theta = ij_theta.second;
				m_theta->QueryTheta(query, i_theta, tm_data_i);
				m_theta->QueryTheta(query, j_theta, tm_data_j);
				m_etb->Set(i_theta, j_theta, TransformArchive::Error_q(tm_data_i, tm_data_j));
			}
			m_theta->EndQuery(query);
		}

		int m_nthreads;
		int m_id;
		ETBRect* m_etb;
		const CPGTheta* m_theta;
		const std::list<std::string>* m_joints;
	};

	CThreadPool_W32<ThreadXUpdator> pool;
	int n_threads = std::max(2, (CThreadPool_W32<ThreadXUpdator>::N_CPUCores() / 6));
	int i_thread = 0;
	pool.Initialize_main(n_threads,
						[&](ThreadXUpdator* thread)
							{
								thread->Initialize_main(n_threads,
														i_thread ++,
														errTB,
														theta,
														joints);
							});
	auto& threads = pool.WaitForAllReadyThreads_main();
	for (auto thread : threads)
		thread->Kickoff_main();
	pool.WaitForAllReadyThreads_main();
};