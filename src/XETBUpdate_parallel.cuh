#pragma once
#include "vector_types.h"

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

			for (int i_joint = 0; i_joint < query->n_interests; i_joint ++)
			{
				int i_theta_q = i_theta_i * query->n_interests + i_joint;
				theta_q_i[i_theta_q].w = tm_data[i_joint].r.w;
				theta_q_i[i_theta_q].x = tm_data[i_joint].r.x;
				theta_q_i[i_theta_q].y = tm_data[i_joint].r.y;
				theta_q_i[i_theta_q].z = tm_data[i_joint].r.z;
			}
		}		
	}

	int64_t n_err = errTB->Length();
	IKAssert(n_err == (int64_t)n_theta0 * (int64_t)n_theta1);
	// ComputeErr(theta_q[0], n_theta0, theta_q[1], n_theta1, errTB->data(), n_err);

	delete [] theta_q[0];
	delete [] theta_q[1];
	theta.EndQuery(query);
}