#include "pch.h"
#include "ik_logger.h"
#include "loggerfast.h"
#include "MoNode.h"
#include "MotionPipeConf.hpp"

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
	fprintf(stdout
			, "[%s:%d] %s = %p\n"
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

void LOGIKFlush()
{
#if defined SMOOTH_LOGGING
	g_LoggerFast.Flush();
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

// //#define DECLARE_TYPELOG(func)\
// //	void func(const char* file, unsigned int line, const char* token, short type);
// #define ENUM_START(LogInfoEnum_x)\
// 	void LogInfoEnum_x(const char* file, unsigned int line, const char* token, short type)\
// 	{\
// 		EnumText flagsDfn [] = {
// #define ENUM_END\
// 		};\
// 		LogInfoEnum(type, flagsDfn, sizeof(flagsDfn)/sizeof(EnumText), file, line, token);\
// 	}
// #define ENUM_ITEM(type)\
// 	{type, #type} ,


// // ENUM_START(LogInfoEnum_TM_TYPE)
// // 	ENUM_ITEM(CMoNode::homo)
// // 	ENUM_ITEM(CMoNode::cross)
// // 	ENUM_ITEM(CMoNode::unknown)
// // ENUM_END


// #undef ENUM_START
// #undef ENUM_END
// #undef ENUM_ITEM

// //#define DECLARE_FLAGLOG(func)\
// //	void func(const char* file, unsigned int line, const char* token, short flag);
// #define FLAG_START(LogInfoFlag_x)\
// 	void LogInfoFlag_x(const char* file, unsigned int line, const char* token, short flag)\
// 	{\
// 		FlagText flagsDfn [] = {
// #define FLAG_END\
// 		};\
// 		LogInfoFlag(flag, flagsDfn, sizeof(flagsDfn)/sizeof(FlagText), file, line, token);\
// 	}
// #define FLAG_ENTRY(flag)\
// 	{flag, #flag} ,

// FLAG_START(LogInfoFlag_con)
// 	FLAG_ENTRY(CONSTRAINT_IK_TIP)
// 	FLAG_ENTRY(CONSTRAINT_IK_ROT)
// 	/* targetless */
// 	FLAG_ENTRY(CONSTRAINT_IK_AUTO)
// 	/* autoik */
// 	FLAG_ENTRY(CONSTRAINT_IK_TEMP)
// 	FLAG_ENTRY(CONSTRAINT_IK_STRETCH)
// 	FLAG_ENTRY(CONSTRAINT_IK_POS)
// 	FLAG_ENTRY(CONSTRAINT_IK_SETANGLE)
// 	FLAG_ENTRY(CONSTRAINT_IK_GETANGLE)
// 	/* limit axis */
// 	FLAG_ENTRY(CONSTRAINT_IK_NO_POS_X)
// 	FLAG_ENTRY(CONSTRAINT_IK_NO_POS_Y)
// 	FLAG_ENTRY(CONSTRAINT_IK_NO_POS_Z)
// 	FLAG_ENTRY(CONSTRAINT_IK_NO_ROT_X)
// 	FLAG_ENTRY(CONSTRAINT_IK_NO_ROT_Y)
// 	FLAG_ENTRY(CONSTRAINT_IK_NO_ROT_Z)
// 	/* axis relative to target */
// 	FLAG_ENTRY(CONSTRAINT_IK_TARGETAXIS)
// FLAG_END

// FLAG_START(LogInfoFlag_bone)
// 	FLAG_ENTRY(BONE_IK_NO_XDOF)
// 	FLAG_ENTRY(BONE_IK_NO_YDOF)
// 	FLAG_ENTRY(BONE_IK_NO_ZDOF)
// 	FLAG_ENTRY(BONE_IK_XLIMIT)
// 	FLAG_ENTRY(BONE_IK_YLIMIT)
// 	FLAG_ENTRY(BONE_IK_ZLIMIT)
// 	FLAG_ENTRY(BONE_IK_ROTCTL)
// 	FLAG_ENTRY(BONE_IK_LINCTL)
// 	FLAG_ENTRY(BONE_IK_NO_XDOF_TEMP)
// 	FLAG_ENTRY(BONE_IK_NO_YDOF_TEMP)
// 	FLAG_ENTRY(BONE_IK_NO_ZDOF_TEMP)
// FLAG_END

// #undef FLAG_START
// #undef FLAG_END
// #undef FLAG_ENTRY

int __cdecl LoggerFast_OutFmt(const char* _Format, ...)
{
	int _Result;
	va_list _ArgList;
	__crt_va_start(_ArgList, _Format);
	_Result = g_LoggerFast.Out(_Format, _ArgList);
	__crt_va_end(_ArgList);
	return _Result;
}