#include <stdio.h>
#include "ik_logger.h"
#include "BLI_assert.h"
#include "DNA_constraint_types.h"
#include "DNA_action_types.h"
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
	BLI_assert(NULL != p_delim);
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
	char str_flags[1024] = {0};
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
		strcpy(p_flags_dst, "UnMatched");

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

//#define DECLARE_TYPELOG(func)\
//	void func(const char* file, unsigned int line, const char* token, short type);
#define ENUM_START(LogInfoEnum_x)\
	void LogInfoEnum_x(const char* file, unsigned int line, const char* token, short type)\
	{\
		EnumText flagsDfn [] = {
#define ENUM_END\
		};\
		LogInfoEnum(type, flagsDfn, sizeof(flagsDfn)/sizeof(EnumText), file, line, token);\
	}
#define ENUM_ITEM(type)\
	{type, #type} ,


ENUM_START(LogInfoEnum_contype)
	ENUM_ITEM(CONSTRAINT_TYPE_NULL)
	/** Unimplemented non longer :) - during constraints recode, Aligorith */
	ENUM_ITEM(CONSTRAINT_TYPE_CHILDOF)
	ENUM_ITEM(CONSTRAINT_TYPE_TRACKTO)
	ENUM_ITEM(CONSTRAINT_TYPE_KINEMATIC)
	ENUM_ITEM(CONSTRAINT_TYPE_FOLLOWPATH)
	/** Unimplemented no longer :) - Aligorith */
	ENUM_ITEM(CONSTRAINT_TYPE_ROTLIMIT)
	/** Unimplemented no longer :) - Aligorith */
	ENUM_ITEM(CONSTRAINT_TYPE_LOCLIMIT)
	/** Unimplemented no longer :) - Aligorith */
	ENUM_ITEM(CONSTRAINT_TYPE_SIZELIMIT)
	ENUM_ITEM(CONSTRAINT_TYPE_ROTLIKE)
	ENUM_ITEM(CONSTRAINT_TYPE_LOCLIKE)
	ENUM_ITEM(CONSTRAINT_TYPE_SIZELIKE)
	/** Unimplemented no longer :) - Aligorith. Scripts */
	ENUM_ITEM(CONSTRAINT_TYPE_PYTHON)
	ENUM_ITEM(CONSTRAINT_TYPE_ACTION)
	/** New Tracking constraint that locks an axis in place - theeth */
	ENUM_ITEM(CONSTRAINT_TYPE_LOCKTRACK)
	/** limit distance */
	ENUM_ITEM(CONSTRAINT_TYPE_DISTLIMIT)
	/** claiming this to be mine :) is in tuhopuu bjornmose */
	ENUM_ITEM(CONSTRAINT_TYPE_STRETCHTO)
	/** floor constraint */
	ENUM_ITEM(CONSTRAINT_TYPE_MINMAX)
	// CONSTRAINT_TYPE_DEPRECATED
	/** clampto constraint */
	ENUM_ITEM(CONSTRAINT_TYPE_CLAMPTO)
	/** transformation (loc/rot/size -> loc/rot/size) constraint */
	ENUM_ITEM(CONSTRAINT_TYPE_TRANSFORM)
	/** shrinkwrap (loc/rot) constraint */
	ENUM_ITEM(CONSTRAINT_TYPE_SHRINKWRAP)
	/** New Tracking constraint that minimizes twisting */
	ENUM_ITEM(CONSTRAINT_TYPE_DAMPTRACK)
	/** Spline-IK - Align 'n' bones to a curve */
	ENUM_ITEM(CONSTRAINT_TYPE_SPLINEIK)
	/** Copy transform matrix */
	ENUM_ITEM(CONSTRAINT_TYPE_TRANSLIKE)
	/** Maintain volume during scaling */
	ENUM_ITEM(CONSTRAINT_TYPE_SAMEVOL)
	/** Pivot Constraint */
	ENUM_ITEM(CONSTRAINT_TYPE_PIVOT)
	/** Follow Track Constraint */
	ENUM_ITEM(CONSTRAINT_TYPE_FOLLOWTRACK)
	/** Camera Solver Constraint */
	ENUM_ITEM(CONSTRAINT_TYPE_CAMERASOLVER)
	/** Object Solver Constraint */
	ENUM_ITEM(CONSTRAINT_TYPE_OBJECTSOLVER)
	/** Transform Cache Constraint */
	ENUM_ITEM(CONSTRAINT_TYPE_TRANSFORM_CACHE)
	/** Armature Deform Constraint */
	ENUM_ITEM(CONSTRAINT_TYPE_ARMATURE)
ENUM_END

#undef ENUM_START
#undef ENUM_END
#undef ENUM_ITEM

//#define DECLARE_FLAGLOG(func)\
//	void func(const char* file, unsigned int line, const char* token, short flag);
#define FLAG_START(LogInfoFlag_x)\
	void LogInfoFlag_x(const char* file, unsigned int line, const char* token, short flag)\
	{\
		FlagText flagsDfn [] = {
#define FLAG_END\
		};\
		LogInfoFlag(flag, flagsDfn, sizeof(flagsDfn)/sizeof(FlagText), file, line, token);\
	}
#define FLAG_ENTRY(flag)\
	{flag, #flag} ,

FLAG_START(LogInfoFlag_con)
	FLAG_ENTRY(CONSTRAINT_IK_TIP)
	FLAG_ENTRY(CONSTRAINT_IK_ROT)
	/* targetless */
	FLAG_ENTRY(CONSTRAINT_IK_AUTO)
	/* autoik */
	FLAG_ENTRY(CONSTRAINT_IK_TEMP)
	FLAG_ENTRY(CONSTRAINT_IK_STRETCH)
	FLAG_ENTRY(CONSTRAINT_IK_POS)
	FLAG_ENTRY(CONSTRAINT_IK_SETANGLE)
	FLAG_ENTRY(CONSTRAINT_IK_GETANGLE)
	/* limit axis */
	FLAG_ENTRY(CONSTRAINT_IK_NO_POS_X)
	FLAG_ENTRY(CONSTRAINT_IK_NO_POS_Y)
	FLAG_ENTRY(CONSTRAINT_IK_NO_POS_Z)
	FLAG_ENTRY(CONSTRAINT_IK_NO_ROT_X)
	FLAG_ENTRY(CONSTRAINT_IK_NO_ROT_Y)
	FLAG_ENTRY(CONSTRAINT_IK_NO_ROT_Z)
	/* axis relative to target */
	FLAG_ENTRY(CONSTRAINT_IK_TARGETAXIS)
FLAG_END

FLAG_START(LogInfoFlag_bone)
	FLAG_ENTRY(BONE_IK_NO_XDOF)
	FLAG_ENTRY(BONE_IK_NO_YDOF)
	FLAG_ENTRY(BONE_IK_NO_ZDOF)
	FLAG_ENTRY(BONE_IK_XLIMIT)
	FLAG_ENTRY(BONE_IK_YLIMIT)
	FLAG_ENTRY(BONE_IK_ZLIMIT)
	FLAG_ENTRY(BONE_IK_ROTCTL)
	FLAG_ENTRY(BONE_IK_LINCTL)
	FLAG_ENTRY(BONE_IK_NO_XDOF_TEMP)
	FLAG_ENTRY(BONE_IK_NO_YDOF_TEMP)
	FLAG_ENTRY(BONE_IK_NO_ZDOF_TEMP)
FLAG_END

#undef FLAG_START
#undef FLAG_END
#undef FLAG_ENTRY

int __cdecl LoggerFast_OutFmt(const char* _Format, ...)
{
	int _Result;
	va_list _ArgList;
	__crt_va_start(_ArgList, _Format);
	_Result = g_LoggerFast.Out(_Format, _ArgList);
	__crt_va_end(_ArgList);
	return _Result;
}