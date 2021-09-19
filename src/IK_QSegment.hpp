#pragma once
#include "IKChain.hpp"
class IK_QSegment
{
public:
	bool Initialize(const CIKChain::IKNode& from, const CIKChain::IKNode& to);
	//to unify the segment positions within [0, 1], for better numerical computation performance
	void Scale(Real s);
	// number of degrees of freedom
	int NumberOfDoF() const
	{
		return 0; // joint answers this question
	}

	// unique id for this segment, for identification in the jacobian
	int DoFId() const
	{
		return 0; // joint answers this question
	}

	void SetDoFId(int dof_id)
	{

	}

	// per dof joint weighting
	Real Weight(int dof) const
	{
		return (Real)0;
	}

	// set joint weights (per axis)
	void SetWeight(int, Real)
	{
	}

	// update the angles using the dTheta's computed using the jacobian matrix
	bool UpdateAngle(const IK_QJacobian &jacobian, Eigen::Vector3r &delta, bool *clamp)
	{
		return false; //the joint interprets dTheta
	}

	void UpdateAngleApply()
	{
		// does nothing, will be removed after testing
	}

	// locking (during inner clamping loop)
	bool Locked(int dof) const
	{
		return false;
	}

	void Lock(int dofId, IK_QJacobian &jacobian, Eigen::Vector3r &delta)
	{
	}

	void UnLock()
	{

	}

	// the max distance of the end of this bone from the local origin.
	Real MaxExtension() const
	{
		return 0; //artciulated body answers this question
	}

	Eigen::Vector3r GlobalStart() const
	{
		//artciulated body answers this question
		return Eigen::Vector3r(0, 0, 0);
	}

	Eigen::Vector3r GlobalEnd() const
	{
		//articulated body answers this question
		return Eigen::Vector3r(0, 0, 0);
	}

	// Eigen::Quaternionr GlobalTransform() const
	// {
	// 	//
	// 	return Eigen::Quaternionr::Identity();
	// }

	// the global transformation at the end of the segment
	// will be changed later
	Eigen::Affine3r GlobalTransform() const
	{
		//
		return Eigen::Affine3r::Identity();
	}


	Eigen::Vector3r Axis(int dof) const
	{
		//joint answers this question
		return Eigen::Vector3r(0, 0, 0);
	}

	// is a translational segment?
	bool Translational() const
	{
		return false; //currently, we don't have a translational joint
	}

	void FK_Update()
	{

	}
private:
	Real m_scale;
	CIKChain::IKNode m_from;
	CIKChain::IKNode m_to;
};

