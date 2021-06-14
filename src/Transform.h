#pragma once


class CTransform
{
public:
	CTransform(const _TRANSFORM& tm)
	{
		Eigen::Vector3r p(tm.tt.x, tm.tt.y, tm.tt.z);
		Eigen::Quaternionr r(tm.r.w, tm.r.x, tm.r.y, tm.r.z);
		Eigen::Vector3r s(tm.s.x, tm.s.y, tm.s.z);
		m_t.fromPositionOrientationScale(p, r, s);
	}

	CTransform()
	{
		m_t = Eigen::Affine3r::Identity();
	}

	CTransform& operator*(const CTransform& second)
	{
		m_t = m_t * second.m_t;
		return *this;
	}

	void CopyTo(_TRANSFORM& tm)
	{
		Eigen::Matrix3r r_m, s_m;
		m_t.computeRotationScaling(&r_m, &s_m);
		Eigen::Quaternionr r(r_m);
		Eigen::Vector3r s(s_m.diagonal());
		Eigen::Vector3r t(m_t.translation());

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
private:
	Eigen::Affine3r m_t;
};

