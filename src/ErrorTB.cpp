#include "pch.h"
#include "ErrorTB.hpp"
#include "PostureGraph.hpp"

class IErrorTBImpl : public IErrorTB
{
public:
	virtual void Alloc(_ERROR_TB* etb)
	{
		etb->n_rows = N_Theta();
		etb->n_cols = N_Theta();
		etb->data = (Real*)malloc(etb->n_rows*etb->n_rows * sizeof(Real));
		for (int i_row = 0; i_row < etb->n_rows; i_row++)
		{
			for (int i_col = 0; i_col < etb->n_cols; i_col++)
			{
				etb->data[i_row*etb->n_cols +i_col] = Get(i_row, i_col);
			}
		}
	}

};

// a lower triangular matrix stores posture Error(i, j) for i, j in Theta
class ETBTriL : public IErrorTBImpl
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
		IKAssert(0 != i_theta || 0 != j_theta);
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

class ETBRect : public IErrorTBImpl
{
public:
	ETBRect(int n_rows, int n_cols)
		: m_nRows(n_rows)
		, m_nCols(n_cols)
		, m_nTheta0(m_nRows)
		, m_nTheta1(m_nCols)
	{
		m_elements = new Real[m_nRows*m_nCols];
	}

	virtual ~ETBRect()
	{
		delete [] m_elements;
	}

	virtual void Set(int i_theta, int j_theta, Real err_ij)
	{
		int i_offset = Offset(i_theta, j_theta);
		if (!(i_offset<0))
			m_elements[i_offset] = err_ij;
	}

	virtual Real Get(int i_theta, int j_theta) const
	{
		int i_offset = Offset(i_theta, j_theta);
		if (i_offset < 0)
			return (Real)N_Theta(); // error max: edges from same theta file are supposed no less than epsilon
		else
			return m_elements[i_offset];
	}

	virtual int Offset(int i_theta, int j_theta) const
	{
		int nTheta = m_nTheta0 + m_nTheta1;

		int segs[][2] = {
			{0, m_nTheta0},			//[0, m_nTheta0)
			{m_nTheta0, nTheta},	//[m_nTheta0, nTheta)
		};

		int ij[] = {i_theta, j_theta};
		int ij_seg[2] = {0};
		for (int i_ij = 0; i_ij < 2; i_ij ++)
		{
			for (int i_seg = 0; i_seg < 2; i_seg ++)
			{
				bool fall_in = (segs[i_seg][0] <= ij[i_ij]
							&& ij[i_ij] < segs[i_seg][1]);
				if (fall_in)
				{
					ij_seg[i_ij] = i_seg;
					break;
				}
			}
		}

		bool homo = (ij_seg[0] == ij_seg[1]);
		if (homo)
			return -1;
		else
		{
			int i_row = i_theta;
			int i_col = j_theta;
			if (!(i_row < i_col))
				std::swap(i_row, i_col);
			i_col -= m_nTheta0;
			IKAssert(i_row < m_nRows && i_col < m_nCols);
			return i_row * m_nCols + i_col;
		}
	}

	virtual int N_Theta() const
	{
		return m_nRows + m_nCols;
	}


private:
	int m_nRows;
	int m_nCols;
	int &m_nTheta0;
	int &m_nTheta1;
	Real* m_elements;
};

class ETBNull : public IErrorTBImpl
{
public:
	ETBNull()
		: m_refTheta(NULL)
		, m_refQuery(NULL)
	{
	}

	~ETBNull()
	{
		IKAssert((NULL == m_refTheta)
				== (NULL == m_refQuery));
		if (m_refQuery)
			m_refTheta->EndQuery(m_refQuery);
	}

	virtual void Set(int i_theta, int j_theta, Real err_ij)
	{
	}

	virtual Real Get(int i_theta, int j_theta) const
	{
		IKAssert(NULL != m_refTheta
			&& NULL != m_refQuery);
		m_refTheta->QueryTheta(m_refQuery, i_theta, m_thetaData_i);
		m_refTheta->QueryTheta(m_refQuery, j_theta, m_thetaData_j);
		return TransformArchive::Error_q(m_thetaData_i, m_thetaData_j);
	}

	virtual int N_Theta() const
	{
		IKAssert(NULL != m_refTheta);
		return m_refTheta->N_Theta();
	}

	void AttachThetaRef(const CPGTheta& theta, const std::list<std::string>& joints)
	{
		IKAssert(NULL == m_refTheta);
		m_refTheta = &theta;
		m_refQuery = theta.BeginQuery(joints);
		m_thetaData_i.Resize(m_refQuery->n_interests);
		m_thetaData_j.Resize(m_refQuery->n_interests);
	}

private:
	const CPGTheta* m_refTheta;
	CPGTheta::Query* m_refQuery;
	mutable TransformArchive m_thetaData_i;
	mutable TransformArchive m_thetaData_j;
};



void IErrorTB::Free(_ERROR_TB* err_tb)
{
	free(err_tb->data);
	err_tb->data = NULL;
	err_tb->n_rows = 0;
	err_tb->n_cols = 0;
}

IErrorTB* IErrorTB::Factory::CreateHOMO(const CPGTheta& theta, const std::list<std::string>& joints)
{
	unsigned int n_theta = theta.N_Theta();
	if (CPGTheta::SmallHomoETB(n_theta))
	{
		IErrorTB* errTB = new ETBTriL(n_theta);
		unsigned int n_theta_m = n_theta - 1;
		auto query = theta.BeginQuery(joints);
		TransformArchive tm_data_i(query->n_interests);
		TransformArchive tm_data_j(query->n_interests);
		for (unsigned int i_theta = 1; i_theta < n_theta_m; i_theta ++) // to skip the injected 'T' posture
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
	else
	{
		ETBNull* errTB =  new ETBNull();
		errTB->AttachThetaRef(theta, joints);
		return errTB;
	}
}

IErrorTB* IErrorTB::Factory::CreateX(const CPGTheta& theta, const std::list<std::string>& joints, int n_theta_0, int n_theta_1)
{
	int n_theta = theta.N_Theta();
	IKAssert(n_theta == n_theta_0 + n_theta_1);
	if (CPGTheta::SmallXETB(n_theta_0, n_theta_1))
	{
		IErrorTB* errTB = new ETBRect(n_theta_0, n_theta_1);
		auto query = theta.BeginQuery(joints);
		TransformArchive tm_data_i(query->n_interests);
		TransformArchive tm_data_j(query->n_interests);
		for (int i_theta = 1; i_theta < n_theta_0; i_theta ++) // to skip the injected 'T' posture
		{
			theta.QueryTheta(query, i_theta, tm_data_i);
			for (int j_theta = n_theta_0 + 1; j_theta < n_theta; j_theta ++)
			{
				theta.QueryTheta(query, j_theta, tm_data_j);
				errTB->Set(i_theta, j_theta, TransformArchive::Error_q(tm_data_i, tm_data_j));
			}
		}
		theta.EndQuery(query);
		return errTB;
	}
	else
	{
		ETBNull* errTB =  new ETBNull();
		errTB->AttachThetaRef(theta, joints);
		return errTB;
	}
}

void IErrorTB::Factory::Release(IErrorTB* etb)
{
	delete etb;
}

