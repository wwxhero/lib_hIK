#pragma once
#include <string>
#include "IK_QSegment.hpp"
namespace CONF
{
	class CJointConf
	{
	public:
		CJointConf(const char* name);
#ifdef _DEBUG
		void Dump_Dbg() const;
#endif
		std::string name;
		IK_QSegment::Type type;
	};
};