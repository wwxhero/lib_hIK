// pch.h: This is a precompiled header file.
// Files listed below are compiled only once, improving build performance for future builds.
// This also affects IntelliSense performance, including code completion and many code browsing features.
// However, files listed here are ALL re-compiled if any one of them is updated between builds.
// Do not add files here that you will be updating frequently as this negates the performance advantage.

#ifndef PCH_H
#define PCH_H

// add headers that you want to pre-compile here
#if defined _WINDOWS
#	define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
// Windows Header Files
#	include <windows.h>
#endif

#include <assert.h>
#include <stdio.h>
#include <locale>
#include <codecvt>

// DLL export C functions:
//		https://docs.microsoft.com/en-us/cpp/build/exporting-from-a-dll-using-declspec-dllexport?view=msvc-160
// calling convention:
//		https://en.wikipedia.org/wiki/X86_calling_conventions#Microsoft_x64_calling_convention
#ifdef LIB_HIK_EXPORTS
#define HIKLIB(rtype, func)\
	__declspec(dllexport) rtype __stdcall func
#else
#define HIKLIB(rtype, func)\
	__declspec(dllimport) rtype __stdcall func
#endif

typedef float Real;
#define Real4 float4

#define REAL_MAX FLT_MAX

#ifdef _DEBUG

#define H_INVALID {NULL}

typedef struct _HBODY
{
	void* p;
} HBODY;

typedef struct _HMOTIONNODE
{
	void* p;
} HMOTIONNODE;

typedef struct _HBVH
{
	void* p;
} HBVH;

typedef struct _HCONF
{
	void* p;
} HCONF;

typedef struct _HCONFFKRC
{
	void* p;
} HCONFMOPIPE;

typedef struct _HPG
{
	void* p;
} HPG;

#define VALID_HANDLE(h)\
	((h).p != NULL)

#else

#define H_INVALID NULL
typedef void* HBODY;
typedef void* HMOTIONNODE;
typedef void* HBVH;
typedef void* HCONF;
typedef void* HCONFMOPIPE;
typedef void* HPG;

#define VALID_HANDLE(h)\
	((h) != NULL)


#endif

#include <crtdbg.h>
#if defined LEAK_CHECK
#define _CRTDBG_MAP_ALLOC
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#endif


#define HIKLIB_CB(rtype, func)\
		rtype (__stdcall func)

#endif //PCH_H
