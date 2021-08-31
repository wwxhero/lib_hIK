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
	~LoggerFast();
	int __cdecl OutFmt(const char* fmt, ...);
	int Out(const char* fmt, va_list _ArgList);
	void Flush();
private:
	ILogger* m_pLogger;
	const unsigned int c_fmtBuffSz;
	char* m_pFmtBuff;
	wchar_t* m_pFmtBuffw;
	CS m_cs;
};

extern LoggerFast g_LoggerFast;
#endif
