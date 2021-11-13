#pragma once
#ifndef __IK_LOGGER_H__
#	define __IK_LOGGER_H__
#	include <string.h>
#	include "articulated_body.h"
// #define SMOOTH_LOGGING
// #define PROFILE_IK

#	ifdef __cplusplus
extern "C" {
#	endif

const char *file_short(const char *file_f);

void LogInfo(const char *file, unsigned int line, const char *info);

void LogInfoWCharPtr(const char *file, unsigned int line, const char *token, const wchar_t* v);
void LogInfoCharPtr(const char *file, unsigned int line, const char *token, const char* v);
void LogInfoPtr(const char *file, unsigned int line, const char *token, const void* v);
void LogInfoInt(const char *file, unsigned int line, const char *token, int v);
void LogInfoBool(const char *file, unsigned int line, const char *token, bool v);
void LogInfoReal(const char *file, unsigned int line, const char *token, Real v);
void LogInfoReal3x3_m(const char *file, unsigned int line, const char *token, const Real m[3][3]);
void LogInfoReal3x3(const char *file, unsigned int line, const char *token, const Real *m);
void LogInfoReal1x3(const char *file, unsigned int line, const char *token, const Real *v);
void LogInfoTM(const char *file, unsigned int line, const char *token, const _TRANSFORM *v);
void LOGIKFlush();

void AssertionFail(const char *file, unsigned int line);


#	define DECLARE_ENUMLOG(LogInfoEnum_x) \
		void LogInfoEnum_x(const char *file, unsigned int line, const char *token, short type);

#	define DECLARE_FLAGLOG(LogInfoFlag_x) \
		void LogInfoFlag_x(const char *file, unsigned int line, const char *token, short flag);

#	if defined _DEBUG || defined SMOOTH_LOGGING

#		define LOGIKVar(func, var) func(__FILE__, __LINE__, #    var, var);
#		define LOGIKVarErr(func, var) func(__FILE__, __LINE__, "ERROR: "#var, var);
#		define LOGIKVarWarning(func, var) func(__FILE__, __LINE__, "WARNING: "#var, var);
#		define LOGIK(msg) LogInfo(__FILE__, __LINE__, msg);
#		if defined HARDASSERTION
#			define IKAssert assert
#		else
#			define IKAssert(v)\
				if(!(v))\
					AssertionFail(__FILE__, __LINE__);
#		endif
#	else

#		if !defined(NDEBUG)
#      		error Delcare for no-debugging purpose
#    	endif

#		define LOGIKVar(func, var)
#		define LOGIKVarErr(func, var) func(__FILE__, __LINE__, "ERROR: "#var, var);
#		define LOGIKVarWarning(func, var) func(__FILE__, __LINE__, "WARNING: "#var, var);
#		define LOGIK(msg)
#		define IKAssert(v)\
				if(!(v))\
					AssertionFail(__FILE__, __LINE__);

#	endif

#	ifdef PROFILE
#		define START_PROFILER(frame_id, token, rounds) \
			ULONGLONG ___tick_start = GetTickCount64(); \
			unsigned int ___line_start = __LINE__; \
			const char *___token = token; \
			unsigned int ___rounds = rounds; \
			unsigned int ___frame_id = frame_id; \
			for (int i = 0; i < rounds; i++) {

#		define STOP_PROFILER \
			} \
			ULONGLONG ___tick = GetTickCount64() - ___tick_start; \
			unsigned int ___line_end = __LINE__; \
			double ___millisec = (double)___tick / (double)___rounds; \
			LoggerFast_OutFmt("%s, %d:%d, frame_id=, %d, token=, %s, averange=, %f, sum=, %u\n", \
												file_short(__FILE__), \
												___line_start, \
												___line_end, \
												___frame_id, \
												___token, \
												___millisec,\
												___tick);\
			LOGIKFlush();

#		define START_PROFILER_AUTOFRAME(token, rounds) \
			START_PROFILER(g_profiler.frame_id, token, rounds)

#		define START_PROFILER_AUTOFRAME_IK(rounds) \
			TransformArchive ___tm_data; \
			CArtiBodyNode* ___root_body = CAST_2PBODY(mopipe->bodies[0]); \
			CArtiBodyTree::Serialize<true>(___root_body, ___tm_data); \
			START_PROFILER_AUTOFRAME("ik", rounds); \
			CArtiBodyTree::Serialize<false>(___root_body, ___tm_data); \
			CArtiBodyTree::FK_Update<false>(___root_body);

#		define STOP_PROFILER_IK \
			} \
			ULONGLONG ___tick = GetTickCount64() - ___tick_start; \
			unsigned int ___line_end = __LINE__; \
			double ___millisec = (double)___tick / (double)___rounds; \
			TransformArchive ___tm_data_prime; \
			CArtiBodyTree::Serialize<true>(___root_body, ___tm_data_prime); \
			LoggerFast_OutFmt("%s, %d:%d, frame_id=, %d, token=, %s, averange=, %f, sum=, %u, error=, %f\n", \
												file_short(__FILE__), \
												___line_start, \
												___line_end, \
												___frame_id, \
												___token, \
												___millisec, \
												___tick, \
												2*rad2deg(acos(1-TransformArchive::Error_q(___tm_data, ___tm_data_prime)))); \
			LOGIKFlush();

#		define PROFILE_FRAME(i_frame) \
			g_profiler.frame_id = i_frame

			typedef struct _Profile
			{
				unsigned int frame_id;
			} Profile;
			extern Profile g_profiler;
#	else
#		define START_PROFILER(frame_id, token, rounds)
#		define START_PROFILER_AUTOFRAME(token, rounds)
#		define PROFILE_FRAME(i_frame)
#		define STOP_PROFILER
#		define START_PROFILER_AUTOFRAME_IK(rounds)
#		define STOP_PROFILER_IK
#	endif



int __cdecl LoggerFast_OutFmt(const char *fmt, ...);

// DECLARE_FLAGLOG(LogInfoFlag_con)
// DECLARE_ENUMLOG(LogInfoEnum_TM_TYPE)
// DECLARE_FLAGLOG(LogInfoFlag_bone)

#  ifdef __cplusplus
}
#  endif

#endif
