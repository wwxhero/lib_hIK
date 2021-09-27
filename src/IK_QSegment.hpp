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
		R_xyz = 0
	};

	DECLARE_ENUM_STR(Type)

public:
	IK_QSegment(Type type, int n_dof);
	virtual ~IK_QSegment();
	bool Initialize(CArtiBodyNode* from, CArtiBodyNode* to);

	const char* GetName_c(int side = 0) const
	{
		return m_bodies[side]->GetName_c();
	}

	//to unify the segment positions within [0, 1], for better numerical computation performance
	void Scale(Real s)
	{
		m_scale = s;
		m_max_extension *= s;
	}

	// number of degrees of freedom
	int NumberOfDoF() const
	{
    	return c_num_DoF;
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
	virtual bool Locked(int dof) const = 0;

	virtual void UnLock() = 0;

	// per dof joint weighting
	virtual Real Weight(int dof_l) const = 0;

	// recursively update the global coordinates of this segment, 'global'
 	// is the global transformation from the parent segment

	// get axis from rotation matrix for derivative computation
	virtual int Axis(Eigen::Vector3r axis[6]) const = 0;

	// update the angles using the dTheta's computed using the jacobian matrix
	virtual bool UpdateAngle(const IK_QJacobian &jacobian, Eigen::Vector3r &delta, bool *clamp) = 0;
	virtual void Lock(int dofId, IK_QJacobian &jacobian, Eigen::Vector3r &delta) = 0;

	virtual void UpdateAngleApply()
	{

	}

  	// set joint limits
	virtual void SetLimit(int, double, double)
	{
	}
	// set joint weights (per axis)
	virtual void SetWeight(int dof_l, Real w) = 0;

	// Eigen::Quaternionr GlobalTransform() const
	// {
	// 	//
	// 	return Eigen::Quaternionr::Identity();
	// }

	// the global transformation at the end of the segment
	// will be changed later
	Eigen::Affine3r GlobalTransform() const
	{
		const Transform* l2w = m_bodies[c_idxTo]->GetTransformLocal2World();
		Eigen::Affine3r tm_l2w_end;
		tm_l2w_end.linear() = l2w->getLinear();
		tm_l2w_end.translation() = m_scale * l2w->getTranslation();
		return tm_l2w_end;
	}


protected:
	Real m_scale;
	CArtiBodyNode* m_bodies[2];
	IJoint*	m_joints[2];
	Real m_max_extension;
	int m_DoF_id;
public:
	const Type c_type;
	const int c_num_DoF;
	const int c_idxFrom;
	const int c_idxTo;
};


class IK_QIxyzSegment : public IK_QSegment
{
public:
	IK_QIxyzSegment(const Real weight[3]);
	virtual Real Weight(int dof_l) const;
	virtual void SetWeight(int dof_l, Real w);
	virtual int Axis(Eigen::Vector3r axis[6]) const;
	virtual bool UpdateAngle(const IK_QJacobian &jacobian, Eigen::Vector3r &delta, bool *clamp);
	virtual bool Locked(int dof_l) const;
	virtual void UnLock();
	virtual void Lock(int dofId, IK_QJacobian &jacobian, Eigen::Vector3r &delta);
private:
	Real m_weight[3];
	bool m_locked[3];
	Eigen::Vector3r m_theta;
};
