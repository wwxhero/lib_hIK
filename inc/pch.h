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
#ifdef UTRECORDPARSER_EXPORTS
#define HIKLIB(rtype, func)\
	__declspec(dllexport) rtype __stdcall func
#else
#define HIKLIB(rtype, func)\
	__declspec(dllimport) rtype __stdcall func
#endif

#define H_INVALID NULL

typedef float Real;

typedef void* HBODY;
typedef void* HMOTIONNODE;


const Real c_epsilon = 1e-5f;

#endif //PCH_H
