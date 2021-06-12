#pragma once
#include "pch.h"

#ifdef __cplusplus
extern "C" {
#endif

HIKLIB(HMOTIONPIPE, createHomoSpaceMotionPipe)(HBODY start, HBODY end);
HIKLIB(HMOTIONPIPE, createXSpaceMotionPipe)(HBODY start, HBODY end);
HIKLIB(HPIPELINE, 	createPipeline)(HMOTIONPIPE pipe);
HIKLIB(bool, 		appendPipe)(HPIPELINE line, HMOTIONPIPE pipe, HMOTIONPIPE pipe_suc);
HIKLIB(void, 		destroyMotionPipe)(HMOTIONPIPE pipe);
HIKLIB(void, 		executePipeLine)(HPIPELINE line);
HIKLIB(void, 		destroyPipeline)(HPIPELINE line);

#ifdef __cplusplus
}
#endif