/**
 * Copyright 1993-2015 NVIDIA Corporation.  All rights reserved.
 *
 * Please refer to the NVIDIA end user license agreement (EULA) associated
 * with this source code for terms and conditions that govern your use of
 * this software. Any use, reproduction, disclosure, or distribution of
 * this software and related documentation outside the terms of the EULA
 * is strictly prohibited.
 *
 */

/**
 * Vector addition: C = A + B.
 *
 * This sample is a very basic sample that implements element by element
 * vector addition. It is the same as the sample illustrating Chapter 2
 * of the programming guide with some additions like error checking.
 */
#include "pch.h"
#include <algorithm>
// For the CUDA runtime routines (prefixed with "cuda_")
#include <cuda_runtime.h>
#include <helper_cuda.h>

#include "ik_logger.h"

// ComputeErr_GPU<<<minGridSize, blockSize>>>(theta0_q_dev, n_theta0,
// 												theta1_q_dev, n_theta1,
// 											 	err_out_dev, n_err,
// 											 	n_joints);

__global__ void
ComputeErr_GPU(const Real4* theta0_q, int n_theta0, const Real4* theta1_q, int n_theta1, Real* err_out, int64_t n_err, int n_joints, int n_threads)
{
	for (int i_err = blockDim.x * blockIdx.x + threadIdx.x
		; i_err < n_err
		; i_err += n_threads)
	{
		int i_theta0 = i_err / n_theta1;
		int i_theta1 = i_err % n_theta1;
		Real sigma_k = 0;
		for (int i_joint = 0; i_joint < n_joints; i_joint ++)
		{
			auto q_0_i = theta0_q[i_theta0 * n_joints + i_joint];
			auto q_1_i = theta1_q[i_theta1 * n_joints + i_joint];
			auto err_k_ij = fabs( q_0_i.w * q_1_i.w
								+ q_0_i.x * q_1_i.x
								+ q_0_i.y * q_1_i.y
								+ q_0_i.z * q_1_i.z);
			sigma_k += min((Real)1.0, err_k_ij);
		}
		Real err_i = (Real)n_joints - sigma_k;
		err_out[i_err] = err_i;
	}
}

void ComputeErr(const Real4* theta0_q, int n_theta0, const Real4* theta1_q, int n_theta1, Real* err_out, int64_t n_err, int n_joints)
{
#if defined _GPU_PARALLEL
	size_t size_theta0 = n_theta0 * n_joints * sizeof(Real4);
	size_t size_theta1 = n_theta1 * n_joints * sizeof(Real4);
	size_t size_err = n_err * sizeof(Real);

	Real4* theta0_q_dev = NULL;
	Real4* theta1_q_dev = NULL;
	Real* err_out_dev = NULL;
	cudaError_t err = cudaSuccess;
	if (cudaSuccess != (err = cudaMalloc((void **)&theta0_q_dev, size_theta0))
	 || cudaSuccess != (err = cudaMalloc((void **)&theta1_q_dev, size_theta1))
	 || cudaSuccess != (err = cudaMalloc((void **)&err_out_dev, size_err)))
	{
		const char* errInfo = cudaGetErrorString(err);
		LOGIKVarErr(LogInfoCharPtr, errInfo);
		if (theta0_q_dev)
			cudaFree(theta0_q_dev);
		if (theta1_q_dev)
			cudaFree(theta1_q_dev);
		if (err_out_dev)
			cudaFree(err_out_dev);
		return;
	}
	
	if (cudaSuccess != (err = cudaMemcpy(theta0_q_dev, theta0_q, size_theta0, cudaMemcpyHostToDevice))
	 || cudaSuccess != (err = cudaMemcpy(theta1_q_dev, theta1_q, size_theta1, cudaMemcpyHostToDevice)))
	{
		const char* errInfo = cudaGetErrorString(err);
		LOGIKVarErr(LogInfoCharPtr, errInfo);
		if (theta0_q_dev)
			cudaFree(theta0_q_dev);
		if (theta1_q_dev)
			cudaFree(theta1_q_dev);
		if (err_out_dev)
			cudaFree(err_out_dev);		
		return;
	}

	int minGridSize = 0;
	int blockSize = 0;
	if (cudaSuccess != (err = cudaOccupancyMaxPotentialBlockSize(&minGridSize
																, &blockSize
																, (void*)ComputeErr_GPU)))
	{
		const char* errInfo = cudaGetErrorString(err);
		LOGIKVarErr(LogInfoCharPtr, errInfo);
		if (theta0_q_dev)
			cudaFree(theta0_q_dev);
		if (theta1_q_dev)
			cudaFree(theta1_q_dev);
		if (err_out_dev)
			cudaFree(err_out_dev);		
		return;
	}

	int n_threads = minGridSize * blockSize;
	// LOGIKVarErr(LogInfoInt, minGridSize);
	// LOGIKVarErr(LogInfoInt, blockSize);

	ComputeErr_GPU<<<minGridSize, blockSize>>>(theta0_q_dev, n_theta0,
												theta1_q_dev, n_theta1,
											 	err_out_dev, n_err,
											 	n_joints, n_threads);
	
	if (cudaSuccess != (err = cudaGetLastError()))
	{
		const char* errInfo = cudaGetErrorString(err);
		LOGIKVarErr(LogInfoCharPtr, errInfo);
		if (theta0_q_dev)
			cudaFree(theta0_q_dev);
		if (theta1_q_dev)
			cudaFree(theta1_q_dev);
		if (err_out_dev)
			cudaFree(err_out_dev);
		return;		
	}
	
	if (cudaSuccess != (err = cudaMemcpy(err_out, err_out_dev, size_err, cudaMemcpyDeviceToHost)))
	{
		const char* errInfo = cudaGetErrorString(err);
		LOGIKVarErr(LogInfoCharPtr, errInfo);
		if (theta0_q_dev)
			cudaFree(theta0_q_dev);
		if (theta1_q_dev)
			cudaFree(theta1_q_dev);
		if (err_out_dev)
			cudaFree(err_out_dev);
		return;
	}

	if (theta0_q_dev)
		cudaFree(theta0_q_dev);
	if (theta1_q_dev)
		cudaFree(theta1_q_dev);
	if (err_out_dev)
		cudaFree(err_out_dev);

#else
	for (int i_err = 0; i_err < n_err; i_err ++)
	{
		int i_theta0 = i_err / n_theta1;
		int i_theta1 = i_err % n_theta1;
		Real sigma_k = 0;
		for (int i_joint = 0; i_joint < n_joints; i_joint ++)
		{
			auto q_0_i = theta0_q[i_theta0 * n_joints + i_joint];
			auto q_1_i = theta1_q[i_theta1 * n_joints + i_joint];
			auto err_k_ij = fabs( q_0_i.w * q_1_i.w
								+ q_0_i.x * q_1_i.x
								+ q_0_i.y * q_1_i.y
								+ q_0_i.z * q_1_i.z);
			sigma_k += std::min((Real)1.0, err_k_ij);
		}
		Real err_i = (Real)n_joints - sigma_k;
		err_out[i_err] = err_i;
	}
#endif
}





