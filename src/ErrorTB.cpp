#include "pch.h"
#include "ErrorTB.hpp"
#include "PostureGraph.hpp"

void IErrorTB::Free(_ERROR_TB* err_tb)
{
	free(err_tb->data);
	err_tb->data = NULL;
	err_tb->n_rows = 0;
	err_tb->n_cols = 0;
}

// a lower triangular matrix stores posture Error(i, j) for i, j in Theta
class ETBTriL : public IErrorTB
{
public:
	ETBTriL(int n_theta)
		: m_nTheta(n_theta)
		, m_nEles((n_theta*(n_theta-1)) >> 1)
	{
		m_elements = (Real*)malloc(m_nEles * sizeof(Real));
	}

	virtual ~ETBTriL()
	{
		free(m_elements);
	}

	virtual void Set(int i_theta, int j_theta, Real err_ij)
	{
		IKAssert(i_theta != j_theta
			|| (-c_epsilon < err_ij && err_ij < c_epsilon));
		if (i_theta != j_theta)
		{
			int i_offset = Offset(i_theta, j_theta);
			m_elements[i_offset] = err_ij;
		}
	}

	virtual Real Get(int i_theta, int j_theta) const
	{
		if (i_theta == j_theta)
			return (Real)0;
		else
		{
			int i_offset = Offset(i_theta, j_theta);
			return m_elements[i_offset];
		}
	}

	virtual int N_Theta() const
	{
		return m_nTheta;
	}

	virtual void Alloc(_ERROR_TB* etb)
	{
		etb->n_rows = m_nTheta;
		etb->n_cols = m_nTheta;
		etb->data = (Real*)malloc(m_nTheta*m_nTheta * sizeof(Real));
		for (int i_row = 0; i_row < m_nTheta; i_row++)
		{
			for (int i_col = 0; i_col < m_nTheta; i_col++)
			{
				etb->data[i_row*m_nTheta+i_col] = Get(i_row, i_col);
			}
		}
	}

private:
	int Offset(int i_theta, int j_theta) const
	{
		//a lower triangular matrix, thus the matrix stores M(r, c) where r > c
		int i_row = i_theta;
		int i_col = j_theta;
		if (!(i_row > i_col))
			std::swap(i_row, i_col);

		int n_theta_preceeding = i_row;
		return ((n_theta_preceeding*(n_theta_preceeding-1)) >> 1) + i_col;
	}
private:
	Real* m_elements;
	int m_nEles;
	int m_nTheta;

};


IErrorTB* IErrorTB::Factory::CreateHOMO(const CPGThetaClose& theta, const std::list<std::string>& joints)
{
	IErrorTB* errTB = new ETBTriL(theta.N_Theta());
	unsigned int n_theta = theta.N_Theta();
	unsigned int n_theta_m = n_theta - 1;
	auto query = theta.BeginQuery(joints);
	TransformArchive tm_data_i(query->n_interests);
	TransformArchive tm_data_j(query->n_interests);
	for (unsigned int i_theta = 0; i_theta < n_theta_m; i_theta ++)
	{
		theta.QueryTheta(query, i_theta, tm_data_i);
		for (unsigned int j_theta = i_theta + 1; j_theta < n_theta; j_theta ++)
		{
			theta.QueryTheta(query, j_theta, tm_data_j);
			errTB->Set(i_theta, j_theta, TransformArchive::Error_q(tm_data_i, tm_data_j));
		}
	}
	theta.EndQuery(query);
	return errTB;
}

void IErrorTB::Factory::Release(IErrorTB* etb)
{
	delete etb;
}