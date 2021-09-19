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
 */

/** \file
 * \ingroup iksolver
 */

#pragma once

/**
 * @author Laurence Bourn
 * @date 28/6/2001
 */

#include <vector>
#include <list>

#include "Math.hpp"
#include "IK_QJacobian.h"
#include "IK_QSegment.hpp"
#include "IK_QTask.h"


class IK_QJacobianSolver
{
public:
	IK_QJacobianSolver();
	~IK_QJacobianSolver()
	{
	}

	// call setup once before solving, if it fails don't solve
	bool Setup(const std::vector<CIKChain::IKNode>& chain
			, const CArtiBodyNode* eef
			, std::list<IK_QTask *> &tasks);

	// returns true if converged, false if max number of iterations was used
	bool Solve(std::list<IK_QTask *> &tasks
			, const Real tolerance
			, const int max_iterations);

protected:
	bool UpdateAngles(Real &norm);

	Real ComputeScale();
	void Scale(Real scale, std::list<IK_QTask *> &tasks);

protected:
	IK_QJacobianSDLS m_jacobian;
	IK_QJacobianSDLS m_jacobian_sub;

	bool m_secondary_enabled;

	std::vector<IK_QSegment*> m_segments; //the corresponds to CIKChain::m_segments

};


