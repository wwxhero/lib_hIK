#include "pch.h"
#include "Math.hpp"
#include "ik_logger.h"

/* Calculates the rest matrix of a bone based on its vector and a roll around that vector. */
/**
 * Given `v = (v.x, v.y, v.z)` our (normalized) bone vector, we want the rotation matrix M
 * from the Y axis (so that `M * (0, 1, 0) = v`).
 * - The rotation axis a lays on XZ plane, and it is orthonormal to v,
 *   hence to the projection of v onto XZ plane.
 * - `a = (v.z, 0, -v.x)`
 *
 * We know a is eigenvector of M (so M * a = a).
 * Finally, we have w, such that M * w = (0, 1, 0)
 * (i.e. the vector that will be aligned with Y axis once transformed).
 * We know w is symmetric to v by the Y axis.
 * - `w = (-v.x, v.y, -v.z)`
 *
 * Solving this, we get (x, y and z being the components of v):
 * <pre>
 *     â”Œ (x^2 * y + z^2) / (x^2 + z^2),   x,   x * z * (y - 1) / (x^2 + z^2) â”
 * M = â”‚  x * (y^2 - 1)  / (x^2 + z^2),   y,    z * (y^2 - 1)  / (x^2 + z^2) â”‚
 *     â”” x * z * (y - 1) / (x^2 + z^2),   z,   (x^2 + z^2 * y) / (x^2 + z^2) â”˜
 * </pre>
 *
 * This is stable as long as v (the bone) is not too much aligned with +/-Y
 * (i.e. x and z components are not too close to 0).
 *
 * Since v is normalized, we have `x^2 + y^2 + z^2 = 1`,
 * hence `x^2 + z^2 = 1 - y^2 = (1 - y)(1 + y)`.
 *
 * This allows to simplifies M like this:
 * <pre>
 *     â”Œ 1 - x^2 / (1 + y),   x,     -x * z / (1 + y) â”
 * M = â”‚                -x,   y,                   -z â”‚
 *     â””  -x * z / (1 + y),   z,    1 - z^2 / (1 + y) â”˜
 * </pre>
 *
 * Written this way, we see the case v = +Y is no more a singularity.
 * The only one
 * remaining is the bone being aligned with -Y.
 *
 * Let's handle
 * the asymptotic behavior when bone vector is reaching the limit of y = -1.
 * Each of the four corner elements can vary from -1 to 1,
 * depending on the axis a chosen for doing the rotation.
 * And the "rotation" here is in fact established by mirroring XZ plane by that given axis,
 * then inversing the Y-axis.
 * For sufficiently small x and z, and with y approaching -1,
 * all elements but the four corner ones of M will degenerate.
 * So let's now focus on these corner elements.
 *
 * We rewrite M so that it only contains its four corner elements,
 * and combine the `1 / (1 + y)` factor:
 * <pre>
 *                    â”Œ 1 + y - x^2,        -x * z â”
 * M* = 1 / (1 + y) * â”‚                            â”‚
 *                    â””      -x * z,   1 + y - z^2 â”˜
 * </pre>
 *
 * When y is close to -1, computing 1 / (1 + y) will cause severe numerical instability,
 * so we ignore it and normalize M instead.
 * We know `y^2 = 1 - (x^2 + z^2)`, and `y < 0`, hence `y = -sqrt(1 - (x^2 + z^2))`.
 *
 * Since x and z are both close to 0, we apply the binomial expansion to the first order:
 * `y = -sqrt(1 - (x^2 + z^2)) = -1 + (x^2 + z^2) / 2`. Which gives:
 * <pre>
 *                        â”Œ  z^2 - x^2,  -2 * x * z â”
 * M* = 1 / (x^2 + z^2) * â”‚                         â”‚
 *                        â”” -2 * x * z,   x^2 - z^2 â”˜
 * </pre>
 */
void vec_roll_to_mat3_normalized(const Eigen::Vector3r& nor, const Real roll, Eigen::Matrix3r& bMatrix)
{
	//to do: using blender algorithm to compute local to world matrix
#define THETA_THRESHOLD_NEGY 1.0e-9f
#define THETA_THRESHOLD_NEGY_CLOSE 1.0e-5f

	assert(nor.norm() < 1 + c_epsilon
			&& nor.norm() > 1 - c_epsilon);


	Real theta = 1.0f + nor[1];

	/* With old algo, 1.0e-13f caused T23954 and T31333, 1.0e-6f caused T27675 and T30438,
	 * so using 1.0e-9f as best compromise.
	 *
	 * New algo is supposed much more precise, since less complex computations are performed,
	 * but it uses two different threshold values...
	 *
	 * Note: When theta is close to zero, we have to check we do have non-null X/Z components as well
	 *       (due to float precision errors, we can have nor = (0.0, -0.99999994, 0.0)...).
	 */
	if (theta > THETA_THRESHOLD_NEGY_CLOSE || ((nor[0] || nor[2]) && theta > THETA_THRESHOLD_NEGY)) {
		/* nor is *not* -Y.
		 * We got these values for free... so be happy with it... ;)
		 */
		bMatrix(1, 0) = -nor[0];
		bMatrix(0, 1) = nor[0];
		bMatrix(1, 1) = nor[1];
		bMatrix(2, 1) = nor[2];
		bMatrix(1, 2) = -nor[2];
		if (theta > THETA_THRESHOLD_NEGY_CLOSE) {
			/* If nor is far enough from -Y, apply the general case. */
			bMatrix(0, 0) = 1 - nor[0] * nor[0] / theta;
			bMatrix(2, 2) = 1 - nor[2] * nor[2] / theta;
			bMatrix(2, 0) = bMatrix(0, 2) = -nor[0] * nor[2] / theta;
		}
		else {
			/* If nor is too close to -Y, apply the special case. */
			theta = nor[0] * nor[0] + nor[2] * nor[2];
			bMatrix(0, 0) = (nor[0] + nor[2]) * (nor[0] - nor[2]) / -theta;
			bMatrix(2, 2) = -bMatrix(0, 0);
			bMatrix(2, 0) =  bMatrix(0, 2) = 2.0f * nor[0] * nor[2] / theta;
		}
	}
	else {
		/* If nor is -Y, simple symmetry by Z axis. */
		bMatrix.setIdentity();
		bMatrix(0, 0) = bMatrix(1, 1) = -1.0;
	}

	/* Make Roll matrix */
	bool with_roll = (roll < -c_epsilon) || (c_epsilon < roll);
	if (with_roll)
	{
		Eigen::Matrix3r rMatrix = Eigen::AngleAxis<Real>(roll, nor).toRotationMatrix();
		bMatrix = rMatrix * bMatrix;
	}


#undef THETA_THRESHOLD_NEGY
#undef THETA_THRESHOLD_NEGY_CLOSE
}


void vec_to_mat3_normalized_sim(const Eigen::Vector3r& nor, Eigen::Matrix3r& rotm)
{
	const Eigen::Vector3r up_sim = Eigen::Vector3r::UnitY();
	const Eigen::Vector3r forward_sim = Eigen::Vector3r::UnitZ();
	// Eigen::Vector3r& x = rot.col(0);
	// Eigen::Vector3r& y = rot.col(1);
	// Eigen::Vector3r& z = rot.col(2);
	Eigen::Vector3r x;
	Eigen::Vector3r y = nor;
	Eigen::Vector3r xX = y.cross(forward_sim);
	const Real sin_epsilon = sin(deg2rad((Real)1));
	Real X = xX.norm();
	if (sin_epsilon < X)
		x = xX/X;
	else
		x = up_sim;
	Eigen::Vector3r z = x.cross(y);
	rotm.col(0) = x;
	rotm.col(1) = y;
	rotm.col(2) = z;
#ifdef _DEBUG
	IKAssert(
		rotm(0, 0) == x(0) && rotm(0, 1) == y(0) && rotm(0, 2) == z(0) &&
		rotm(1, 0) == x(1) && rotm(1, 1) == y(1) && rotm(1, 2) == z(1) &&
		rotm(2, 0) == x(2) && rotm(2, 1) == y(2) && rotm(2, 2) == z(2) );
#endif

}