#pragma once
#include "pch.h"
#include "Math.hpp"

class Matrix3 : public Eigen::Matrix<Real, 3, 3>
{
public:
	Matrix3()
		: Eigen::Matrix<Real, 3, 3>()
	{
	}

	Matrix3(const Eigen::Matrix<Real, 3, 3>& other)
		: Eigen::Matrix<Real, 3, 3>(other)
	{
	}

	Matrix3(const Matrix3& other)
		: Eigen::Matrix<Real, 3, 3>(other)
	{
	}

	Matrix3(const Real a_d[3][3])
	{
		setData(a_d);
	}

	void setData(const Real a_d[3][3])
	{
		Real* d = data();
		for (int i_r = 0; i_r < 3; i_r ++)
		{
			for (int i_c = 0; i_c < 3; i_c ++)
			{
				int i_offset = i_c * 3 + i_r;
				d[i_offset] = a_d[i_r][i_c];
			}
		}
	}

	std::string ToString() const
	{
		const Real* d = data();
		const unsigned int szBuff = 1024;
		char info[szBuff] = {0};
		char *offset_i = info;
		unsigned int size_i = szBuff;
		for (int i_r = 0; i_r < 3; i_r ++)
		{
			int szWritten = sprintf_s(offset_i, size_i, "\n[%7.4f\t%7.4f\t%7.4f]"
									, d[i_r], d[i_r + 3], d[i_r + 6]);
			offset_i += szWritten;
			size_i -= szWritten;
		}
		std::string strInfo(info);
		return std::move(strInfo);
	}

	Matrix3& operator=(const Eigen::Matrix<Real, 3, 3>& other)
	{
		Eigen::Matrix<Real, 3, 3>::operator=(other);
		return *this;
	}
};





