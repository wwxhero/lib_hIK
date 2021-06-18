#pragma once
#include <string>
class CLogger
{
public:
	void Out(const std::string& logInfo);
};

extern CLogger g_logger;