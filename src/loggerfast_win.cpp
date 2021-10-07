#include "pch.h"
#include <windows.h>
#include <comutil.h>
#include <comdef.h>
#include <cstdlib>
#include "loggerSrv_i.h"
#include "loggerfast.h"


void SimpleUTF16(const char* src, unsigned int len, wchar_t* dst)
{
	const char* p_src = src;
	const char* p_src_end = p_src + len;
	wchar_t* p_dst = dst;
	for (; p_src < p_src_end; p_src ++, p_dst ++)
	{
		*p_dst = *p_src;
	}
	*p_dst = L'\0';
}

LoggerFast::LoggerFast(const char* path) throw(...)
	: m_pLogger(NULL)
	, c_fmtBuffSz(1024)
	, m_pFmtBuff(NULL)
{
	Initialize(path);
}

LoggerFast::LoggerFast() throw(...)
	: m_pLogger(NULL)
	, c_fmtBuffSz(1024)
	, m_pFmtBuff(NULL)
{
	const char* envVar = "InternalLog";
	char c_fileName[1024] = {0};
	char *envFileName = NULL;
	if (NULL != (envFileName = std::getenv(envVar)))
		strcpy(c_fileName, envFileName);
	else
	{
		sprintf(c_fileName
				, "%s.txt"
				, envVar);
		fprintf(stdout
				, "Environment Variable <%s> is not set\n"
				, envVar);
		fflush(stdout);
	}
	Initialize(c_fileName);
}

void LoggerFast::Initialize(const char* path) throw (...)
{
	m_pFmtBuff = new char[c_fmtBuffSz];
	m_pFmtBuffw = new wchar_t[c_fmtBuffSz];

	::CoInitialize(NULL);

	HRESULT hResult = ::CoCreateInstance(CLSID_Logger
									, NULL
									, CLSCTX_INPROC_SERVER|CLSCTX_LOCAL_SERVER
									, IID_ILogger
									, (void **)&m_pLogger);
	if (SUCCEEDED(hResult))
	{
		_bstr_t path_bstr(path);
		m_pLogger->Create(path_bstr);
	}
	else
	{
		m_pLogger = NULL;
		const char* error = "Logger not installed on the platform";
		assert(0);
		throw error;
	}
	InitializeCriticalSection(&m_cs);
}

LoggerFast::~LoggerFast()
{
	delete [] m_pFmtBuff;
	delete [] m_pFmtBuffw;
	if (NULL != m_pLogger)
	{
		m_pLogger->Close();
		m_pLogger->Release();
		m_pLogger = NULL;
	}
	DeleteCriticalSection(&m_cs);
	::CoUninitialize();
}

int __cdecl LoggerFast::OutFmt(const char *_Format, ...)
{
	int _Result;
	va_list _ArgList;
	__crt_va_start(_ArgList, _Format);
	_Result = Out(_Format, _ArgList);
	__crt_va_end(_ArgList);
	return _Result;
}

int LoggerFast::Out(const char* _Format, va_list _ArgList)
{
	int _Result = 0;
	EnterCriticalSection(&m_cs);
	#pragma warning(push)
	#pragma warning(disable: 4996) // Deprecation
	_Result = _vsprintf_l(m_pFmtBuff, _Format, NULL, _ArgList);
	#pragma warning(pop)
	assert(_Result < (int)c_fmtBuffSz);
	if (_Result > 0)
	{
		SimpleUTF16(m_pFmtBuff, _Result, m_pFmtBuffw);
		_bstr_t data(m_pFmtBuffw);
		assert(_Result == data.length());
		m_pLogger->LogOut(data);
	}
	LeaveCriticalSection(&m_cs);
	return _Result;
}

void LoggerFast::Flush()
{
	EnterCriticalSection(&m_cs);
	m_pLogger->Dump();
	LeaveCriticalSection(&m_cs);
}

LoggerFast g_LoggerFast;

LoggerFast& operator << (LoggerFast& logger, const std::string& info)
{
	logger.OutFmt(info.c_str());
	return logger;
}

LoggerFast& operator << (LoggerFast& logger, double v)
{
	logger.OutFmt("%f", v);
	return logger;
}

LoggerFast& operator << (LoggerFast& logger, std::size_t v)
{
	logger.OutFmt("%d", v);
	return logger;
}

LoggerFast& operator << (LoggerFast& logger, int v)
{
	logger.OutFmt("%d", v);
	return logger;
}