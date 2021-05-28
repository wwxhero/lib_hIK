#pragma once
#ifdef UTRECORDPARSER_EXPORTS
#define IKAPI __declspec(dllexport)
#else
#define IKAPI __declspec(dllimport)
#endif
#ifdef __cplusplus
extern "C" {
#endif

float IKAPI ik_test(float theta);

#ifdef __cplusplus
}
#endif