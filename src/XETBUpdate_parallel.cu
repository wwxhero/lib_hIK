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
 * It computes the error of quaternions of the two postures, equivalent to TransformArchive::Error_q
 *
 */
#include "pch.h"
#include <algorithm>
// For the CUDA runtime routines (prefixed with "cuda_")
#include <cuda_runtime.h>
#include <helper_cuda.h>

#include "ik_logger.h"

// ComputeHErr_GPU<<<minGridSize, blockSize>>>(theta_q_dev, size_theta,
// 										 	err_out_dev, n_errs,
// 										 	n_joints, n_threads)

__global__ void
ComputeHErr_GPU(const Real4* theta_q, int n_theta, Real* err_out, int64_t n_err, int n_joints, int n_threads)
{
	for (int64_t i_err = blockDim.x * blockIdx.x + threadIdx.x
		; i_err < n_err
		; i_err += n_threads)
	{
		int64_t i_err2 = (i_err << 1);
		int i_theta1 = (int)floor(sqrtf(i_err2));
		int i_theta0 = i_theta1 + 1;		
		int i_theta = 0;
		
		if (!(i_err2 < i_theta1 * (i_theta1 + 1)))
			i_theta = i_theta0;
		else
		{
			i_theta = i_theta1;
			// IKAssert(i_theta * (i_theta - 1) <= i_err2);
		}	

		int j_theta2 = (int)(i_err2 - (int64_t)i_theta*(int64_t)(i_theta - 1));
		// IKAssert(0 == (j_theta2&0x01));
		int j_theta = (j_theta2 >> 1);
		// IKAssert(i_theta < n_theta); 
		// IKAssert(j_theta < n_theta);
		// IKAssert(j_theta < i_theta);
		// if (!(j_theta < i_theta))
		// {
		// 	LOGIKVarErr(LogInfoInt, i_theta);
		// 	LOGIKVarErr(LogInfoInt, j_theta);
		// }
		Real sigma_i_joint = 0;
		int i_theta_q_base = i_theta * n_joints;
		int j_theta_q_base = j_theta * n_joints;
		for (int i_joint = 0; i_joint < n_joints; i_joint ++)
		{
			auto q_i = theta_q[i_theta_q_base + i_joint];
			auto q_j = theta_q[j_theta_q_base + i_joint];
			auto err_k_ij = fabs( q_i.w * q_j.w
								+ q_i.x * q_j.x
								+ q_i.y * q_j.y
								+ q_i.z * q_j.z);
			sigma_i_joint += min((Real)1.0, err_k_ij);
		}
		Real err_i = (Real)n_joints - sigma_i_joint;
		err_out[i_err] = err_i;
	}
}


// ComputeXErr_GPU<<<minGridSize, blockSize>>>(theta0_q_dev, n_theta0,
// 												theta1_q_dev, n_theta1,
// 											 	err_out_dev, n_err,
// 											 	n_joints);

__global__ void
ComputeXErr_GPU(const Real4* theta0_q, int n_theta0, const Real4* theta1_q, int n_theta1, Real* err_out, int64_t n_err, int n_joints, int n_threads)
{
	for (int64_t i_err = blockDim.x * blockIdx.x + threadIdx.x
		; i_err < n_err
		; i_err += n_threads)
	{
		int i_theta0 = i_err / n_theta1;
		int i_theta1 = i_err % n_theta1;
		Real sigma_i_joint = 0;
		int i_theta0_q_base = i_theta0 * n_joints;
		int i_theta1_q_base = i_theta1 * n_joints;
		for (int i_joint = 0; i_joint < n_joints; i_joint ++)
		{
			auto q_0_i = theta0_q[i_theta0_q_base + i_joint];
			auto q_1_i = theta1_q[i_theta1_q_base + i_joint];
			auto err_k_ij = fabs( q_0_i.w * q_1_i.w
								+ q_0_i.x * q_1_i.x
								+ q_0_i.y * q_1_i.y
								+ q_0_i.z * q_1_i.z);
			sigma_i_joint += min((Real)1.0, err_k_ij);
		}
		Real err_i = (Real)n_joints - sigma_i_joint;
		err_out[i_err] = err_i;
	}
}

void ComputeXErr(const Real4* theta0_q, int n_theta0, const Real4* theta1_q, int n_theta1, Real* err_out, int64_t n_errs, int n_joints)
{
#if defined _GPU_PARALLEL

	#define RETURN_IF_F(condition)\
		if (condition)\
		{\
			const char* errInfo = cudaGetErrorString(err);\
			LOGIKVarErr(LogInfoCharPtr, errInfo);\
			if (theta0_q_dev)\
				cudaFree(theta0_q_dev);\
			if (theta1_q_dev)\
				cudaFree(theta1_q_dev);\
			if (err_out_dev)\
				cudaFree(err_out_dev);\
			return;\
		}


	size_t size_theta0 = n_theta0 * n_joints * sizeof(Real4);
	size_t size_theta1 = n_theta1 * n_joints * sizeof(Real4);
	size_t size_err = n_errs * sizeof(Real);

	Real4* theta0_q_dev = NULL;
	Real4* theta1_q_dev = NULL;
	Real* err_out_dev = NULL;
	cudaError_t err = cudaSuccess;

	RETURN_IF_F(cudaSuccess != (err = cudaMalloc((void **)&theta0_q_dev, size_theta0))
	 			|| cudaSuccess != (err = cudaMalloc((void **)&theta1_q_dev, size_theta1))
	 			|| cudaSuccess != (err = cudaMalloc((void **)&err_out_dev, size_err)));
	
	RETURN_IF_F(cudaSuccess != (err = cudaMemcpy(theta0_q_dev, theta0_q, size_theta0, cudaMemcpyHostToDevice))
	 			|| cudaSuccess != (err = cudaMemcpy(theta1_q_dev, theta1_q, size_theta1, cudaMemcpyHostToDevice)));
		

	int minGridSize = 0;
	int blockSize = 0;
	RETURN_IF_F(cudaSuccess != (err = cudaOccupancyMaxPotentialBlockSize(&minGridSize
																		, &blockSize
																		, (void*)ComputeXErr_GPU)));

	int n_threads = minGridSize * blockSize;
	// LOGIKVarErr(LogInfoInt, minGridSize);
	// LOGIKVarErr(LogInfoInt, blockSize);

	ComputeXErr_GPU<<<minGridSize, blockSize>>>(theta0_q_dev, n_theta0,
												theta1_q_dev, n_theta1,
											 	err_out_dev, n_errs,
											 	n_joints, n_threads);
	
	RETURN_IF_F(cudaSuccess != (err = cudaGetLastError()));
	
	RETURN_IF_F(cudaSuccess != (err = cudaMemcpy(err_out, err_out_dev, size_err, cudaMemcpyDeviceToHost)));

	if (theta0_q_dev)
		cudaFree(theta0_q_dev);
	if (theta1_q_dev)
		cudaFree(theta1_q_dev);
	if (err_out_dev)
		cudaFree(err_out_dev);

	#undef RETURN_IF_F

#else
	for (int i_err = 0; i_err < n_errs; i_err ++)
	{
		int i_theta0 = i_err / n_theta1;
		int i_theta1 = i_err % n_theta1;
		Real sigma_i_joint = 0;
		int i_theta0_q_base = i_theta0 * n_joints;
		int i_theta1_q_base = i_theta1 * n_joints;
		for (int i_joint = 0; i_joint < n_joints; i_joint ++)
		{
			auto q_0_i = theta0_q[i_theta0_q_base + i_joint];
			auto q_1_i = theta1_q[i_theta1_q_base + i_joint];
			auto err_k_ij = fabs( q_0_i.w * q_1_i.w
								+ q_0_i.x * q_1_i.x
								+ q_0_i.y * q_1_i.y
								+ q_0_i.z * q_1_i.z);
			sigma_i_joint += std::min((Real)1.0, err_k_ij);
		}
		Real err_i = (Real)n_joints - sigma_i_joint;
		err_out[i_err] = err_i;
	}
#endif
}


void ComputeHErr(const Real4* theta_q, int n_theta, Real* err_out, int64_t n_errs, int n_joints)
{
#if defined _GPU_PARALLEL

	#define RETURN_IF_F(condition)\
		if (condition)\
		{\
			const char* errInfo = cudaGetErrorString(err);\
			LOGIKVarErr(LogInfoCharPtr, errInfo);\
			if (theta_q_dev)\
				cudaFree(theta_q_dev);\
			if (err_out_dev)\
				cudaFree(err_out_dev);\
			return;\
		}

	size_t size_theta = n_theta * n_joints * sizeof(Real4);
	size_t size_err = n_errs * sizeof(Real);

	Real4* theta_q_dev = NULL;
	Real* err_out_dev = NULL;
	cudaError_t err = cudaSuccess;

	RETURN_IF_F(cudaSuccess != (err = cudaMalloc((void **)&theta_q_dev, size_theta))
	 			|| cudaSuccess != (err = cudaMalloc((void **)&err_out_dev, size_err)));
	
	RETURN_IF_F(cudaSuccess != (err = cudaMemcpy(theta_q_dev, theta_q, size_theta, cudaMemcpyHostToDevice)));
	 			
	int minGridSize = 0;
	int blockSize = 0;
	RETURN_IF_F(cudaSuccess != (err = cudaOccupancyMaxPotentialBlockSize(&minGridSize
																		, &blockSize
																		, (void*)ComputeHErr_GPU)));

	int n_threads = minGridSize * blockSize;
	// LOGIKVarErr(LogInfoInt, minGridSize);
	// LOGIKVarErr(LogInfoInt, blockSize);

	ComputeHErr_GPU<<<minGridSize, blockSize>>>(theta_q_dev, n_theta,
											 	err_out_dev, n_errs,
											 	n_joints, n_threads);
	
	RETURN_IF_F(cudaSuccess != (err = cudaGetLastError()));
	
	RETURN_IF_F(cudaSuccess != (err = cudaMemcpy(err_out, err_out_dev, size_err, cudaMemcpyDeviceToHost)));

	if (theta_q_dev)
		cudaFree(theta_q_dev);
	if (err_out_dev)
		cudaFree(err_out_dev);

	#undef RETURN_IF_F
#else
	for (int64_t i_err = 0; i_err < n_errs; i_err ++)
	{
		int64_t i_err2 = (i_err << 1);
		int i_theta1 = (int)std::floor(std::sqrt(i_err2));
		int i_theta0 = i_theta1 + 1;		
		int i_theta = 0;
		
		if (!(i_err2 < i_theta1 * (i_theta1 + 1)))
			i_theta = i_theta0;
		else
		{
			i_theta = i_theta1;
			IKAssert(i_theta * (i_theta - 1) <= i_err2);
		}	

		int j_theta2 = (int)(i_err2 - (int64_t)i_theta*(int64_t)(i_theta - 1));
		IKAssert(0 == (j_theta2&0x01));
		int j_theta = (j_theta2 >> 1);
		IKAssert(i_theta < n_theta); 
		IKAssert(j_theta < n_theta);
		IKAssert(j_theta < i_theta);
		if (!(j_theta < i_theta))
		{
			LOGIKVarErr(LogInfoInt, i_theta);
			LOGIKVarErr(LogInfoInt, j_theta);
		}
		Real sigma_i_joint = 0;
		int i_theta_q_base = i_theta * n_joints;
		int j_theta_q_base = j_theta * n_joints;
		for (int i_joint = 0; i_joint < n_joints; i_joint ++)
		{
			auto q_i = theta_q[i_theta_q_base + i_joint];
			auto q_j = theta_q[j_theta_q_base + i_joint];
			auto err_k_ij = fabs( q_i.w * q_j.w
								+ q_i.x * q_j.x
								+ q_i.y * q_j.y
								+ q_i.z * q_j.z);
			sigma_i_joint += std::min((Real)1.0, err_k_ij);
		}
		Real err_i = (Real)n_joints - sigma_i_joint;
		err_out[i_err] = err_i;
	}
#endif
}
