#pragma once

#include <crtdbg.h>
#if defined LEAK_CHECK
#define _CRTDBG_MAP_ALLOC
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#endif