#include "pch.h"
#include "Transform.hpp"

Eigen::Matrix3r Transform::getRotation_m() const
{
	Eigen::Matrix3r lin_m = getLinear();
	auto x = lin_m.col(0);
	x.normalize();
	auto y = lin_m.col(1);
	y.normalize();
	auto z = lin_m.col(2);
	z.normalize();
	return std::move(lin_m);
}

Eigen::Affine3r Transform::getAffine() const
{
	Eigen::Affine3r t;
	t.linear() = getLinear();
	t.translation() = getTranslation();
	return std::move(t);
}

void Transform_TRS::CopyTo(_TRANSFORM& tm) const
{
	Eigen::Matrix3r r_m, s_m;
	computeRotationScaling(&r_m, &s_m);
	Eigen::Quaternionr r(r_m);
	Eigen::Vector3r s(s_m.diagonal());
	Eigen::Vector3r t(translation());

	tm.s.x = s.x();
	tm.s.y = s.y();
	tm.s.z = s.z();

	tm.r.w = r.w();
	tm.r.x = r.x();
	tm.r.y = r.y();
	tm.r.z = r.z();

	tm.tt.x = t.x();
	tm.tt.y = t.y();
	tm.tt.z = t.z();
}

void Transform_TRS::CopyFrom(const _TRANSFORM& tm)
{
	Init(tm);
}

std::string Transform_TRS::ToString() const
{
	_TRANSFORM tm;
	CopyTo(tm);
	const unsigned int szBuff = 1024;
	char info[szBuff] = { 0 };
	sprintf_s(info, szBuff, "s[%.4f\t%.4f\t%.4f], r[%.4f\t%.4f\t%.4f\t%.4f], tt[%.4f\t%.4f\t%.4f]"
		, tm.s.x, tm.s.y, tm.s.z
		, tm.r.w, tm.r.x, tm.r.y, tm.r.z
		, tm.tt.x, tm.tt.y, tm.tt.z);
	std::string strInfo(info);
	return std::move(strInfo);
}

Eigen::Matrix3r Transform_TRS::getLinear() const
{
	Eigen::Matrix3r ret(linear());
	return std::move(ret);
}

void Transform_TRS::setLinear(const Eigen::Matrix3r& lin)
{
	linear() = lin;
}

Eigen::Vector3r Transform_TRS::getTranslation() const
{
	Eigen::Vector3r tt = translation();
	return std::move(tt);
}

void Transform_TRS::setTranslation(const Eigen::Vector3r& tt)
{
	translation() = tt;
}

Eigen::Affine3r Transform_TRS::getAffine() const
{
	return *this;
}

Eigen::Matrix3r Transform_R::getLinear() const
{
	return m_rotq.toRotationMatrix();
}

void Transform_R::setLinear(const Eigen::Matrix3r& tm_l)
{
	IKAssert(tm_l.isUnitary(c_rotm_epsilon));
	m_rotq = tm_l;
}

Eigen::Vector3r Transform_R::getTranslation() const
{
	return Eigen::Vector3r::Zero();
}

void Transform_R::setTranslation(const Eigen::Vector3r& tt)
{
	IKAssert(tt.norm() < c_epsilon);
}

std::string Transform_R::ToString() const
{
	const unsigned int szBuff = 1024;
	char info[szBuff] = { 0 };
	sprintf_s(info, szBuff, "r[%.4f\t%.4f\t%.4f\t%.4f]"
		, m_rotq.w(), m_rotq.x(), m_rotq.y(), m_rotq.z());
	std::string strInfo(info);
	return std::move(strInfo);
}

void Transform_R::CopyTo(_TRANSFORM& tm) const
{
	tm.s.x = tm.s.y = tm.s.z = 1;

	tm.r.w = m_rotq.w();
	tm.r.x = m_rotq.x();
	tm.r.y = m_rotq.y();
	tm.r.z = m_rotq.z();

	tm.tt.x = tm.tt.y = tm.tt.z = 0;
}

void Transform_R::CopyFrom(const _TRANSFORM& tm)
{
	Init(tm);
}

Eigen::Matrix3r Transform_R::getRotation_m() const
{
	return m_rotq.toRotationMatrix();
}


Eigen::Vector3r Transform_TR::getTranslation() const
{
	return m_tt;
}
void Transform_TR::setTranslation(const Eigen::Vector3r& tt)
{
	m_tt = tt;
}

std::string Transform_TR::ToString() const
{
	const unsigned int szBuff = 1024;
	char info[szBuff] = { 0 };
	sprintf_s(info, szBuff, "s[1.0000	1.0000	1.0000], r[%.4f\t%.4f\t%.4f\t%.4f], tt[%.4f\t%.4f\t%.4f]"
		, m_rotq.w(), m_rotq.x(), m_rotq.y(), m_rotq.z()
		, m_tt.x(), m_tt.y(), m_tt.z());
	std::string strInfo(info);
	return std::move(strInfo);
}

void Transform_TR::CopyTo(_TRANSFORM& tm) const
{
	tm.s.x = tm.s.y = tm.s.z = 1;

	tm.r.w = m_rotq.w();
	tm.r.x = m_rotq.x();
	tm.r.y = m_rotq.y();
	tm.r.z = m_rotq.z();

	tm.tt.x = m_tt.x();
	tm.tt.y = m_tt.y();
	tm.tt.z = m_tt.z();
}

void Transform_TR::CopyFrom(const _TRANSFORM& tm)
{
	Init(tm);
}

