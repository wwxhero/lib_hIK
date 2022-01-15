#pragma once
#include "PostureGraph.hpp"
#include "parallel_thread_helper.hpp"
#include "Transform.hpp"

class CThreadPGProj : public CThread_W32
{
public:
	CThreadPGProj();
	void Initialize_main(CPGRuntime* pg, int radius);
	void UpdateFKProj_main();
private:
	virtual void Run_worker();
	CPGRuntime* volatile m_pg;
	volatile int m_radius;
	TransformArchive m_theta0;
};

class CPGRuntimeParallel
{
public:
	typedef std::vector<CThreadPGProj*>* Locker;
public:
	CPGRuntimeParallel();
	~CPGRuntimeParallel();
	bool Load(const char* pgDir, CArtiBodyNode* rootBody, int radius);
	void UpdateFKProj();

	template<bool G_SPACE>
	void ApplyActivePosture()
	{
		auto& workers = m_pool.WaitForAllReadyThreads_main(); // make sure no worker threads working
		IKAssert(1 == workers.size());
		m_pg->ApplyActivePosture<G_SPACE>();
		for (auto worker_i : workers)
			worker_i->HoldReadyOn_main();
	}

	template<bool G_SPACE>
	int SetActivePosture(int pose_id, bool UpdatePose)
	{
		auto& workers = m_pool.WaitForAllReadyThreads_main();
		IKAssert(1 == workers.size());
		int pose_id_ret = m_pg->SetActivePosture<G_SPACE>(pose_id, UpdatePose);
		for (auto worker_i : workers)
			worker_i->HoldReadyOn_main();
		return pose_id_ret;
	}

	CPGRuntime* Lock(Locker* locker);
	void UnLock(Locker locker);

	int Radius() const
	{
		return m_radius;
	}
private:
	CPGRuntime* m_pg;
	CThreadPool_W32<CThreadPGProj> m_pool;
	int m_radius;
};