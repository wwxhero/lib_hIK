#include "pch.h"
#include "PGRuntimeParallel.hpp"

CThreadPGProj::CThreadPGProj()
	: m_pg(NULL)
	, m_radius(0)
{
}

void CThreadPGProj::Initialize_main(CPGRuntime* pg, int radius)
{
	m_pg = pg;
	m_radius = radius;
}

void CThreadPGProj::UpdateFKProj_main()
{
	m_pg->GetRefBodyTheta(m_theta0);
	Execute_main();
}

void CThreadPGProj::Run_worker()
{
	int n_errs = 0;
	auto FK_Err = [&](int pose_id, bool* failed) -> Real
		{
			const TransformArchive& theta_i = m_pg->GetTheta(pose_id);
			Real err = TransformArchive::Error_q(m_theta0, theta_i);
			n_errs ++;
			*failed = (n_errs > m_radius);
			return err;
		};

	int theta_min = CPGRuntime::LocalMin(*m_pg, FK_Err);
	// LOGIKVarErr(LogInfoInt, theta_min);
	m_pg->SetActivePosture<true>(theta_min, false);
	// LOGIKVarErr(LogInfoInt, n_errs);
}


CPGRuntimeParallel::CPGRuntimeParallel()
	: m_pg(NULL)
{

}

CPGRuntimeParallel::~CPGRuntimeParallel()
{
	delete m_pg;
}

bool CPGRuntimeParallel::Load(const char* pgDir, CArtiBodyNode* rootBody, int radius)
{
	m_pg = new CPGRuntime();
	if (!m_pg->Load(pgDir, rootBody))
	{
		delete m_pg;
		m_pg = NULL;
		return false;
	}
	else
	{
		m_pg->SetActivePosture<false>(0, true);
		m_pool.Initialize_main(1,
							[&](CThreadPGProj* thread)
								{
									thread->Initialize_main(m_pg, radius);
								});
		return true;
	}

}

void CPGRuntimeParallel::UpdateFKProj()
{
	auto worker = m_pool.WaitForAReadyThread_main(INFINITE);
	worker->UpdateFKProj_main();
}

