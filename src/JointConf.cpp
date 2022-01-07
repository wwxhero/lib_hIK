#include "pch.h"
#include "JointConf.hpp"

namespace CONF
{
	CJointConf::CJointConf(const char* a_name
						, IK_QSegment::Type a_type
						, const Real a_dexterity[3]
						, IK_QSegment::TypeClamp a_ctype)
		: name(a_name)
		, type(a_type)
		, dexterity{a_dexterity[0], a_dexterity[1], a_dexterity[2]}
		, lim {
				  {std::numeric_limits<Real>::lowest(), std::numeric_limits<Real>::max()}
				, {std::numeric_limits<Real>::lowest(), std::numeric_limits<Real>::max()}
				, {std::numeric_limits<Real>::lowest(), std::numeric_limits<Real>::max()}
			}
		, clamp(a_ctype)
	{
	}

#ifdef _DEBUG
	void CJointConf::Dump_Dbg() const
	{
		LOGIKVar(LogInfoCharPtr, name.c_str());
	}
#endif

};