/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2001-2002 by NaN Holding BV.
 * All rights reserved.
 * Original author: Laurence
 * Modified by: Wanxin Wang
 */

/** \file
 * \ingroup iksolver
 */
#pragma once

#include "pch.h"
#include "IK_QJacobian.h"
#include "ArtiBody.hpp"
#include "macro_helper.h"
class IK_QSegment
{
public:
	enum Type
	{
		R_xyz = 0,
		R_Spherical
	};

	DECLARE_ENUM_STR(Type)

	enum DOFLim
	{
		R_theta = 0,
		R_tau,
		R_phi
	};

	DECLARE_ENUM_STR(DOFLim)

	enum TypeClamp
	{
		C_None = 0,
		C_Spherical,
		C_Direct
	};

	DECLARE_ENUM_STR(TypeClamp)

	static Real MIN_THETA;
	static Real MAX_THETA;
	static Real MIN_TAU;
	static Real MAX_TAU;
	static Real MIN_PHI;
	static Real MAX_PHI;

	// angle in [min, max]

public:
	IK_QSegment(Type type, int n_dof);
	virtual ~IK_QSegment();
	virtual bool Initialize(CArtiBodyNode* from, CArtiBodyNode* to);

	const char* GetName_c(int side = 0) const
	{
		return m_bodies[side]->GetName_c();
	}

	// number of degrees of freedom
	int NumberOfDoF() const
	{
    	return c_num_DoFs;
	}

	// unique id for this segment, for identification in the jacobian
	int DoFId() const
	{
    	IKAssert(m_DoF_id > -1);
    	return m_DoF_id;
	}

	void SetDoFId(int dof_id)
	{
    	m_DoF_id = dof_id;
	}

	// the max distance of the end of this bone from the local origin.
	Real MaxExtension() const
	{
    	return m_max_extension;
	}

	Eigen::Vector3r GlobalStart() const
	{
		const Transform* tm = m_bodies[c_idxFrom]->GetTransformLocal2World();
		IKAssert( t_tr == tm->Type() );
		return m_scale*tm->getTranslation();
	}

	Eigen::Vector3r GlobalEnd() const
	{
		//articulated body answers this question
		const Transform* tm = m_bodies[c_idxTo]->GetTransformLocal2World();
		IKAssert( t_tr == tm->Type() );
		return m_scale*tm->getTranslation();
	}

	// is a translational segment?
	bool Translational() const
	{
		return false; //currently, we don't have a translational joint
	}

	// locking (during inner clamping loop)
	virtual int Locked(bool locked[6]) const = 0;

	virtual void UnLock() = 0;

	// per dof joint weighting
	virtual int Weight(Real w[6]) const = 0;
	// recursively update the global coordinates of this segment, 'global'
 	// is the global transformation from the parent segment

	// get axis from rotation matrix for derivative computation
	virtual int Axis(Eigen::Vector3r axis[6]) const = 0;

	// update the angles using the dTheta's computed using the jacobian matrix
	virtual bool UpdateAngle(const IK_QJacobian &jacobian, Eigen::Vector3r &delta, bool *clamp) = 0;
	virtual void Lock(int dofId, IK_QJacobian &jacobian, Eigen::Vector3r &delta) = 0;

  	// set joint limits
	virtual void SetLimit(DOFLim, const Real lims[2]) = 0;
	// set joint weights (per axis)
	virtual void SetWeight(int dof_l, Real w) = 0;

	// in (0, 1) == [eps, 1-eps]
	Real Stiffness()
	{
		return m_stiffness;
	}
protected:
	Real m_scale;
	CArtiBodyNode* m_bodies[2];
	IJoint*	m_joints[2];
	Real m_max_extension;
	int m_DoF_id;
	Real m_stiffness;
public:
	const Type c_type;
	const int c_num_DoFs;
	const int c_idxFrom;
	const int c_idxTo;
};

class IK_QSegmentSO3 : public IK_QSegment
{
public:
	IK_QSegmentSO3();
	virtual bool Initialize(CArtiBodyNode* from, CArtiBodyNode* to) override;
	virtual void SetWeight(int dof_l, Real w);
	virtual int Weight(Real w[6]) const;
	virtual void SetLimit(DOFLim dof_l, const Real lims[2]);
	virtual int Axis(Eigen::Vector3r axis[6]) const;
	virtual int Locked(bool lock[6]) const;
	virtual void UnLock();
	virtual void Lock(int dofId, IK_QJacobian &jacobian, Eigen::Vector3r &delta);
protected:
	//true: clamp happens
	virtual bool ClampST(bool clamp[3], Eigen::Quaternionr& ori) { return false; };
	Real m_weight[3];
	bool m_locked[3];
	bool m_limited[3];
	Real m_limRange[3][2];
};

class IK_QIxyzSegment : public IK_QSegmentSO3
{
public:
	IK_QIxyzSegment();
	virtual bool UpdateAngle(const IK_QJacobian &jacobian, Eigen::Vector3r &delta, bool *clamp);
};

class IK_QSphericalSegment : public IK_QSegmentSO3
{
public:
	IK_QSphericalSegment();
	virtual bool UpdateAngle(const IK_QJacobian &jacobian, Eigen::Vector3r &delta, bool *clamp);
};

template <typename TSegmentSO3>
class TIK_SegClampSpheri : public TSegmentSO3
{
private:
	virtual bool ClampST(bool clamped[3], Eigen::Quaternionr& rotq) override
	{
		memset(clamped, false, sizeof(bool)*3);
		bool exist_a_lim = (TSegmentSO3::m_limited[TSegmentSO3::R_theta]
						||  TSegmentSO3::m_limited[TSegmentSO3::R_tau]
						||  TSegmentSO3::m_limited[TSegmentSO3::R_phi]);
		if (!exist_a_lim)
			return false;

		// const Eigen::Quaternionr& rotq0 = rotq;
		Real w = rotq.w();
		Real x = rotq.x();
		Real y = rotq.y();
		Real z = rotq.z();
		Real w_s = sqrt(w*w + y*y), x_s, z_s;	//swing: [w_s, x_s, 0, z_s]
		Real w_t, y_t; 								//twist: [w_t, 0, y_t, 0]
		if (abs(w_s) > c_epsilon)
		{
			w_t = w/w_s; y_t = y/w_s;
			x_s = (w*x + y*z)/w_s;
			z_s = (w*z - x*y)/w_s;
		}
		else
		{
			w_t = 1; y_t = 0;
			x_s = x; z_s = z;
		}

		Real tau = wrap_pi(2*atan2(y_t, w_t));
		Real theta = atan2(w_s, x_s);
		Real phi = acos(z_s);

		Real v[3] = {theta, tau, phi};

		Real v_clamp[3] = {0};
		bool clampedST[3] = {false};
		for (int i_dof = 0; i_dof < TSegmentSO3::c_num_DoFs; i_dof ++)
		{
			v_clamp[i_dof] = std::max(TSegmentSO3::m_limRange[i_dof][0], std::min(TSegmentSO3::m_limRange[i_dof][1], v[i_dof]));
			clampedST[i_dof] = (v_clamp[i_dof] != v[i_dof]);
		}

		bool clamped_on_swing = (clampedST[TSegmentSO3::R_theta] || clampedST[TSegmentSO3::R_phi]);
		bool clamped_on_twist = (clampedST[TSegmentSO3::R_tau]);
		bool exists_a_clamped = (clamped_on_swing
								|| clamped_on_twist);
		if (!exists_a_clamped)
			return false;
		else
		{
			Eigen::Quaternionr s_q(w_s, x_s, 0, z_s);
			Eigen::Quaternionr t_q(w_t, 0, y_t, 0);
			if (clamped_on_swing)
			{
				s_q.w() = sin(v_clamp[TSegmentSO3::R_phi])*sin(v_clamp[TSegmentSO3::R_theta]);
				s_q.x() = sin(v_clamp[TSegmentSO3::R_phi])*cos(v_clamp[TSegmentSO3::R_theta]);
				s_q.z() = cos(v_clamp[TSegmentSO3::R_phi]);
				clamped[0] = true;
				clamped[2] = true;
			}
			if (clamped_on_twist)
			{
				Real tau_half = clamped[TSegmentSO3::R_tau]*(Real)0.5;
				t_q.w() = cos(tau_half);
				t_q.y() = sin(tau_half);
				clamped[1] = true;
			}
			rotq = s_q * t_q;
			return true;
		}
	}
};

template <typename TSegmentSO3>
class TIK_SegClampDirect : public TSegmentSO3
{
private:
	virtual bool ClampST(bool clamped[3], Eigen::Quaternionr& rotq) override
	{
		memset(clamped, false, sizeof(bool)*3);
		bool exist_a_lim = (TSegmentSO3::m_limited[TSegmentSO3::R_theta]
						||  TSegmentSO3::m_limited[TSegmentSO3::R_tau]
						||  TSegmentSO3::m_limited[TSegmentSO3::R_phi]);
		if (!exist_a_lim)
			return false;

		// const Eigen::Quaternionr& rotq0 = rotq;
		Real w = rotq.w();
		Real x = rotq.x();
		Real y = rotq.y();
		Real z = rotq.z();
		Real w_s = sqrt(w*w + y*y), x_s, z_s;	//swing: [w_s, x_s, 0, z_s]
		Real w_t, y_t; 								//twist: [w_t, 0, y_t, 0]
		if (abs(w_s) > c_epsilon)
		{
			w_t = w/w_s; y_t = y/w_s;
			x_s = (w*x + y*z)/w_s;
			z_s = (w*z - x*y)/w_s;
		}
		else
		{
			w_t = 1; y_t = 0;
			x_s = x; z_s = z;
		}

		Real tau = wrap_pi(2*atan2(y_t, w_t));
		Real theta = atan2(w_s, x_s);
		Real phi = acos(z_s);

		Real v[3] = {theta, tau, phi};

		Real v_clamp[3] = {0};
		bool clampedST[3] = {false};
		for (int i_dof = 0; i_dof < TSegmentSO3::c_num_DoFs; i_dof ++)
		{
			v_clamp[i_dof] = std::max(TSegmentSO3::m_limRange[i_dof][0], std::min(TSegmentSO3::m_limRange[i_dof][1], v[i_dof]));
			clampedST[i_dof] = (v_clamp[i_dof] != v[i_dof]);
		}

		bool clamped_on_swing = (clampedST[TSegmentSO3::R_theta] || clampedST[TSegmentSO3::R_phi]);
		bool clamped_on_twist = (clampedST[TSegmentSO3::R_tau]);
		bool exists_a_clamped = (clamped_on_swing
								|| clamped_on_twist);
		if (!exists_a_clamped)
			return false;
		else
		{
			Eigen::Quaternionr s_q(w_s, x_s, 0, z_s);
			Eigen::Quaternionr t_q(w_t, 0, y_t, 0);
			if (clamped_on_swing)
			{
				s_q.w() = sin(v_clamp[TSegmentSO3::R_phi])*sin(v_clamp[TSegmentSO3::R_theta]);
				s_q.x() = sin(v_clamp[TSegmentSO3::R_phi])*cos(v_clamp[TSegmentSO3::R_theta]);
				s_q.z() = cos(v_clamp[TSegmentSO3::R_phi]);
				clamped[0] = true;
				clamped[2] = true;
			}
			if (clamped_on_twist)
			{
				Real tau_half = clamped[TSegmentSO3::R_tau]*(Real)0.5;
				t_q.w() = cos(tau_half);
				t_q.y() = sin(tau_half);
				clamped[1] = true;
			}
			rotq = s_q * t_q;
			return true;
		}
	}
};
