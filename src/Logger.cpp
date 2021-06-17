#include "pch.h"
#include <iostream>
#include "Logger.h"

void CLogger::Out(const std::string& Info)
{
	std::string logItem("lib_hIK: \n\t");
	logItem += Info;
	std::cout << logItem << std::endl;
}

CLogger g_logger;