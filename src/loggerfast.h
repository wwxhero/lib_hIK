#ifndef LOGGERFAST_H
#define LOGGERFAST_H
#include "loggerSrv_i.h"
#if defined(_MSC_VER)
#include <synchapi.h>
typedef CRITICAL_SECTION CS;
#endif
class LoggerFast
{
public:
	LoggerFast() throw(...);
	LoggerFast(const char* path) throw(...);
	~LoggerFast();
	int __cdecl OutFmt(const char* fmt, ...);
	int Out(const char* fmt, va_list _ArgList);
	void Out(const char* content);
	void Flush();
private:
	void Initialize(const char* path) throw (...);
	ILogger* m_pLogger;
	const unsigned int c_fmtBuffSz;
	char* m_pFmtBuff;
	wchar_t* m_pFmtBuffw;
	CS m_cs;
};

LoggerFast& operator << (LoggerFast& logger, const std::string& info);
LoggerFast& operator << (LoggerFast& logger, double v);
LoggerFast& operator << (LoggerFast& logger, std::size_t v);
LoggerFast& operator << (LoggerFast& logger, int v);
LoggerFast& operator << (LoggerFast& logger, unsigned int v);



extern LoggerFast g_LoggerFast;
#endif
