#include "pch.h"
#include "motion_pipeline.h"

HMOTIONPIPE createHomoSpaceMotionPipe(HBODY start, HBODY end)
{
	//the articulated bodies are in same coordinate space but in different postures
	return H_INVALID;
}

HMOTIONPIPE createXSpaceMotionPipe(HBODY start, HBODY end)
{
	//the articulated bodies are in different coordiante space but in same (aligned) posture
	return H_INVALID;
}

HPIPELINE createPipeline(HMOTIONPIPE pipe)
{
	//a pipe line contains several motion pipes
	return H_INVALID;
}

bool appendPipe(HPIPELINE line, HMOTIONPIPE pipe, HMOTIONPIPE pipe_suc)
{
	//p == rear(line) -> end(p) == start(pipe)
	return false;
}

void destroyMotionPipe(HMOTIONPIPE pipe)
{

}

void executePipeLine(HPIPELINE line)
{
	// execute the pipe in sequence
}

void destroyPipeline(HPIPELINE line)
{

}