#pragma once
#include "vector_types.h"

void ComputeXErr(const Real4* theta0_q, int n_theta0_q, const Real4* theta1_q, int n_theta1_q, Real* err_out, int64_t n_err, int n_joints);
void ComputeHErr(const Real4* theta, int n_theta, Real* err_out, int64_t n_err, int n_joints);

void UpdateXETB_Parallel_GPU(ETBRect* errTB, const CPGTheta& theta, int n_theta0, int n_theta1, const std::list<std::string>& joints)
{
	CPGTheta::Query* query = theta.BeginQuery(joints);

	TransformArchive tm_data(query->n_interests);
	int ranges[][2] = {
		{0, n_theta0},
		{n_theta0, n_theta0+n_theta1}
	};

	Real4* theta_q[2] = {
		new Real4[n_theta0*query->n_interests],
		new Real4[n_theta1*query->n_interests]
	};

	for (int i_range = 0; i_range < 2; i_range ++)
	{
		Real4* theta_q_i = theta_q[i_range];

		const int& range_i_start = ranges[i_range][0];
		const int& range_i_end = ranges[i_range][1];

		for (int i_theta = range_i_start; i_theta < range_i_end; i_theta ++)
		{
			theta.QueryTheta(query, i_theta, tm_data);
			int i_theta_i = i_theta - range_i_start;

			int i_theta_q_base = i_theta_i * query->n_interests;
			for (int i_joint = 0; i_joint < query->n_interests; i_joint ++)
			{
				int i_theta_q = i_theta_q_base + i_joint;
				theta_q_i[i_theta_q].w = tm_data[i_joint].r.w;
				theta_q_i[i_theta_q].x = tm_data[i_joint].r.x;
				theta_q_i[i_theta_q].y = tm_data[i_joint].r.y;
				theta_q_i[i_theta_q].z = tm_data[i_joint].r.z;
			}
		}		
	}

	int64_t n_err = errTB->Length();
	IKAssert(n_err == (int64_t)n_theta0 * (int64_t)n_theta1);
	ComputeXErr(theta_q[0], n_theta0,
				theta_q[1], n_theta1,
				errTB->Data(), n_err,
				query->n_interests);

	delete [] theta_q[0];
	delete [] theta_q[1];
	theta.EndQuery(query);
}

void UpdateHETB_Parallel_GPU(ETBTriL* errTB, const CPGTheta& theta, const std::list<std::string>& joints)
{
	CPGTheta::Query* query = theta.BeginQuery(joints);

	TransformArchive tm_data(query->n_interests);
	
	int n_theta = theta.N_Theta();

	Real4* theta_q = new Real4[n_theta*query->n_interests];

	for (int i_theta = 0; i_theta < n_theta; i_theta ++)
	{
		theta.QueryTheta(query, i_theta, tm_data);
		
		int i_theta_q_base = i_theta * query->n_interests;
		for (int i_joint = 0; i_joint < query->n_interests; i_joint ++)
		{
			int i_theta_q = i_theta_q_base + i_joint;
			theta_q[i_theta_q].w = tm_data[i_joint].r.w;
			theta_q[i_theta_q].x = tm_data[i_joint].r.x;
			theta_q[i_theta_q].y = tm_data[i_joint].r.y;
			theta_q[i_theta_q].z = tm_data[i_joint].r.z;
		}
	}

	int64_t n_err = errTB->Length();
	IKAssert(n_err == ((n_theta*(n_theta-1))>> 1));
	ComputeHErr(theta_q, n_theta,
				errTB->Data(), n_err,
				query->n_interests);

	delete [] theta_q;	
	theta.EndQuery(query);	
}