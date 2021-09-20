#pragma once
#include <string>

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
	};
};