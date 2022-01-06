#pragma once
#include <string>
#include "IK_QSegment.hpp"
namespace CONF
{
	class CJointConf
	{
	public:
		CJointConf(const char* name, IK_QSegment::Type type, const Real dexterity[3], IK_QSegment::TypeClamp clamp);
#ifdef _DEBUG
		void Dump_Dbg() const;
#endif
		std::string name;
		IK_QSegment::Type type;
		Real dexterity[3];
		Real lim[3][2];
		IK_QSegment::TypeClamp clamp;
	};
};