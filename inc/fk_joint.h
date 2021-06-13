#pragma once
#include "articulated_body.h"

// this header is only for external FK usage, might be removed eventually

#ifdef __cplusplus

extern "C" {
#endif

HIKLIB(void,	set_joint_transform)(HBODY body, const _TRANSFORM* tm_l);
HIKLIB(void,	get_joint_transform)(HBODY body, _TRANSFORM* tm_l);

#ifdef __cplusplus
}
#endif