#pragma once
#include "pch.h"

HMOTIONPIPE createHomoSpaceMotionPipe(HBODY start, HBODY end);
HMOTIONPIPE createXSpaceMotionPipe(HBODY start, HBODY end);
HPIPELINE createPipeline(HMOTIONPIPE pipe);
bool appendPipe(HPIPELINE line, HMOTIONPIPE pipe, HMOTIONPIPE pipe_suc);
void destroyMotionPipe(HMOTIONPIPE pipe);
void executePipeLine(HPIPELINE line);
void destroyPipeline(HPIPELINE line);