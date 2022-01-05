#include "pch.h"
#include "JointConf.hpp"

namespace CONF
{
	CJointConf::CJointConf(const char* a_name, IK_QSegment::Type a_type, const Real a_dexterity[3])
		: name(a_name)
		, type(a_type)
		, dexterity{a_dexterity[0], a_dexterity[1], a_dexterity[2]}
		, lim {
				  {IK_QSegment::MIN_THETA-1, IK_QSegment::MAX_THETA+1}
				, {IK_QSegment::MIN_TAU-1, IK_QSegment::MAX_TAU+1}
				, {IK_QSegment::MIN_PHI-1, IK_QSegment::MAX_PHI+1}
			}
	{
	}

#ifdef _DEBUG
	void CJointConf::Dump_Dbg() const
	{
		LOGIKVar(LogInfoCharPtr, name.c_str());
	}
#endif

};