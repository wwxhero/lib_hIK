#include "pch.h"
#include "ik_logger.h"
#include "loggerfast.h"

const char *file_short(const char *file_f)
{
#ifdef _WIN32
	#define DELIMITER '\\'
#else
	#define DELIMITER '/'
#endif
	const char* p_delim = NULL;
	for (const char* p = file_f
		; *p != '\0'
		; p ++)
	{
		if (*p == DELIMITER)
			p_delim = p;
	}
	assert(NULL != p_delim);
	return ++ p_delim;
}

void AssertionFail(const char *file, unsigned int line)
{
#ifndef SMOOTH_LOGGING
	fprintf(stdout
		, "[%s:%d] ASSERTION FAILED\n"
		, file_short(file)
		, line);
	fflush(stdout);
#else
	g_LoggerFast.OutFmt("[%s:%d] ASSERTION FAILED\n"
						, file_short(file)
						, line);
#endif
}

void LogInfo(const char* file, unsigned int line, const char *info)
{
#ifndef SMOOTH_LOGGING
	fprintf(stdout
		, "[%s:%d] %s\n"
		, file_short(file)
		, line
		, info);
	fflush(stdout);
#else
	g_LoggerFast.OutFmt("[%s:%d] %s\n"
						, file_short(file)
						, line
						, info);
#endif
}

void LogInfoWCharPtr(const char *file, unsigned int line, const char *token, const wchar_t* v)
{
	std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
	std::wstring strV_w(v);
	std::string strV_c = converter.to_bytes(strV_w);
#ifndef SMOOTH_LOGGING
	fprintf(stdout
		, "[%s:%d] %s = %s\n"
		, file_short(file)
		, line
		, token
		, strV_c.c_str());
	fflush(stdout);
#else
	g_LoggerFast.OutFmt("[%s:%d] %s = %s\n"
						, file_short(file)
						, line
						, token
						, strV_c.c_str());
#endif
}

void LogInfoCharPtr(const char *file, unsigned int line, const char *token, const char* v)
{
#ifndef SMOOTH_LOGGING
	fprintf(stdout
		, "[%s:%d] %s = %s\n"
		, file_short(file)
		, line
		, token
		, v);
	fflush(stdout);
#else
	g_LoggerFast.OutFmt("[%s:%d] %s = %s\n"
						, file_short(file)
						, line
						, token
						, v);
#endif
}

void LogInfoPtr(const char* file, unsigned int line, const char* token, const void* v)
{
#ifndef SMOOTH_LOGGING
	fprintf("[%s:%d] %s = %p\n"
			, file_short(file)
			, line
			, token
			, v);
	fflush(stdout);
#else
	g_LoggerFast.OutFmt("[%s:%d] %s = %p\n"
			, file_short(file)
			, line
			, token
			, v);
#endif
}

void LogInfoInt(const char* file, unsigned int line, const char* token, int v)
{
#ifndef SMOOTH_LOGGING
	fprintf(stdout
		, "[%s:%d] %s = %d\n"
		, file_short(file)
		, line
		, token
		, v);
	fflush(stdout);
#else
	g_LoggerFast.OutFmt("[%s:%d] %s = %d\n"
		, file_short(file)
		, line
		, token
		, v);
#endif
}

void LogInfoBool(const char* file, unsigned int line, const char* token, bool v)
{
	const char* repre_b[] = {"false", "true"};
	int repre_i = (v ? 1 : 0);
#ifndef SMOOTH_LOGGING
	fprintf(stdout
		, "[%s:%d] %s = %s\n"
		, file_short(file)
		, line
		, token
		, repre_b[repre_i]);
	fflush(stdout);
#else
	g_LoggerFast.OutFmt("[%s:%d] %s = %s\n"
		, file_short(file)
		, line
		, token
		, repre_b[repre_i]);
#endif
}

void LogInfoFloat(const char* file, unsigned int line, const char* token, float v)
{
#ifndef SMOOTH_LOGGING
	fprintf(stdout
		, "[%s:%d] %s = %.4f\n"
		, file_short(file)
		, line
		, token
		, v);
	fflush(stdout);
#else
	g_LoggerFast.OutFmt("[%s:%d] %s = %.4f\n"
						, file_short(file)
						, line
						, token
						, v);
#endif
}

template<typename TValue>
void LogInfo3x3_m(const char* file, unsigned int line, const char* token, const TValue m[3][3])
{
#ifndef SMOOTH_LOGGING
	fprintf(stdout
		, "[%s:%d] %s = [%.8f\t%.8f\t%.8f;\t%.8f\t%.8f\t%.8f;\t%.8f\t%.8f\t%.8f]\n"
		, file_short(file)
		, line
		, token
		, m[0][0], m[0][1], m[0][2]
		, m[1][0], m[1][1], m[1][2]
		, m[2][0], m[2][1], m[2][2]);
	fflush(stdout);
#else
	g_LoggerFast.OutFmt("[%s:%d] %s = [%.8f\t%.8f\t%.8f;\t%.8f\t%.8f\t%.8f;\t%.8f\t%.8f\t%.8f]\n"
						, file_short(file)
						, line
						, token
						, m[0][0], m[0][1], m[0][2]
						, m[1][0], m[1][1], m[1][2]
						, m[2][0], m[2][1], m[2][2]);
#endif
}

void LogInfoFloat3x3_m(const char* file, unsigned int line, const char* token, const float m[3][3])
{
	LogInfo3x3_m<float>(file, line, token, m);
}

void LogInfoDouble3x3(const char* file, unsigned int line, const char* token, const double* m)
{
#ifndef SMOOTH_LOGGING
	fprintf(stdout
		, "[%s:%d] %s = [%.8f\t%.8f\t%.8f;\t%.8f\t%.8f\t%.8f;\t%.8f\t%.8f\t%.8f]\n"
		, file_short(file)
		, line
		, token
		, m[0], m[1], m[2]
		, m[3], m[4], m[5]
		, m[6], m[7], m[8]);
	fflush(stdout);
#else
	g_LoggerFast.OutFmt("[%s:%d] %s = [%.8f\t%.8f\t%.8f;\t%.8f\t%.8f\t%.8f;\t%.8f\t%.8f\t%.8f]\n"
						, file_short(file)
						, line
						, token
						, m[0], m[1], m[2]
						, m[3], m[4], m[5]
						, m[6], m[7], m[8]);
#endif

}

void LogInfoDouble1x3(const char* file, unsigned int line, const char* token, const double* v)
{
#ifndef SMOOTH_LOGGING
	fprintf(stdout
		, "[%s:%d] %s = [%.8f\t%.8f\t%.8f]\n"
		, file_short(file)
		, line
		, token
		, v[0], v[1], v[2]);
	fflush(stdout);
#else
	g_LoggerFast.OutFmt("[%s:%d] %s = [%.8f\t%.8f\t%.8f]\n"
						, file_short(file)
						, line
						, token
						, v[0], v[1], v[2]);
#endif
}

typedef struct
{
	short value;
	const char* text;
} FlagText;

typedef FlagText EnumText;

void LogInfoFlag(short flag, FlagText* dfns, unsigned short n_dfn, const char* file, unsigned int line, const char* token)
{
	char str_flags[1024] = {0};
	char *p_flags_dst = str_flags;
	for (int i_flag = 0; i_flag < n_dfn; i_flag ++)
	{
		if (flag & dfns[i_flag].value)
		{
			if (p_flags_dst > str_flags)
				*p_flags_dst ++ = '|';
			const char* p_flags_src = dfns[i_flag].text;
			while(*p_flags_src != '\0')
			{
				*p_flags_dst ++ = *p_flags_src ++;
			}
		}
	}
	*p_flags_dst = '\0';

#ifndef SMOOTH_LOGGING
	fprintf(stdout
		, "[%s:%d] %s = %s\n"
		, file_short(file)
		, line
		, token
		, str_flags);
	fflush(stdout);
#else
	g_LoggerFast.OutFmt("[%s:%d] %s == %s\n"
						, file_short(file)
						, line
						, token
						, str_flags);
#endif
}

void LogInfoEnum(short flag, EnumText* dfns, unsigned short n_dfn, const char* file, unsigned int line, const char* token)
{
	const int len_str_flags = 1024;
	char str_flags[len_str_flags] = {0};
	char *p_flags_dst = str_flags;
	bool matched = false;
	for (int i_enum = 0
		; i_enum < n_dfn && !matched
		; i_enum ++)
	{
		matched = (flag == dfns[i_enum].value);
		if (matched)
		{
			const char* p_flags_src = dfns[i_enum].text;
			while (*p_flags_src != '\0')
			{
				*p_flags_dst ++ = *p_flags_src ++;
			}

		}
	}
	if (matched)
		*p_flags_dst = '\0';
	else
		strcpy_s(p_flags_dst, len_str_flags, "UnMatched");

#ifndef SMOOTH_LOGGING
	fprintf(stdout
		, "[%s:%d] %s = %s\n"
		, file_short(file)
		, line
		, token
		, str_flags);
	fflush(stdout);
#else
	g_LoggerFast.OutFmt("[%s:%d] %s == %s\n"
						, file_short(file)
						, line
						, token
						, str_flags);
#endif
}

int __cdecl LoggerFast_OutFmt(const char* _Format, ...)
{
	int _Result;
	va_list _ArgList;
	__crt_va_start(_ArgList, _Format);
	_Result = g_LoggerFast.Out(_Format, _ArgList);
	__crt_va_end(_ArgList);
	return _Result;
}