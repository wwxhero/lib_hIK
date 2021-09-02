// dllmain.cpp : Defines the entry point for the DLL application.
#include "pch.h"
#include <filesystem>

HMODULE g_Module = NULL;

BOOL APIENTRY DllMain( HMODULE hModule,
                       DWORD  ul_reason_for_call,
                       LPVOID lpReserved
                     )
{
    switch (ul_reason_for_call)
    {
    case DLL_PROCESS_ATTACH:
    case DLL_THREAD_ATTACH:
        g_Module = hModule;
        break;
    case DLL_THREAD_DETACH:
    case DLL_PROCESS_DETACH:
        g_Module = NULL;
        break;
    }
    return TRUE;
}

