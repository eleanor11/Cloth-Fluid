// This code contains NVIDIA Confidential Information and is disclosed to you
// under a form of NVIDIA software license agreement provided separately to you.
//
// Notice
// NVIDIA Corporation and its licensors retain all intellectual property and
// proprietary rights in and to this software and related documentation and
// any modifications thereto. Any use, reproduction, disclosure, or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA Corporation is strictly prohibited.
//
// ALL NVIDIA DESIGN SPECIFICATIONS, CODE ARE PROVIDED "AS IS.". NVIDIA MAKES
// NO WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ALL IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE.
//
// Information and code furnished is believed to be accurate and reliable.
// However, NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2013-2015 NVIDIA Corporation. All rights reserved.
#pragma once

#define STRINGIFY(A) #A

#include <core/shader.h>
#include <core/maths.h>

#include <cuda.h>
#include <cuda_runtime_api.h>
#include <cuda_gl_interop.h>

struct Mesh;

void ShadowCreate(GLuint& texture, GLuint& frameBuffer);
void ShadowBegin(GLuint texture, GLuint frameBuffer);
void ShadowEnd();


void ReflectCreate(GLuint& texture, int width, int height);
void ReflectDestroy(GLuint texture);
void ReflectBegin(Vec4 plane, int width, int height);
void ReflectEnd(GLuint texture, int width, int height);

void DrawPlanes(Vec4* planes, int n, float bias);
void DrawPlanesPositions(Vec4* planes, int n);
void DrawSky();

void DrawPositionsBegin(GLuint texture, GLuint framebuffer);
void DrawPositionsEnd(GLuint texture, GLuint framebuffer, float* buffer);

void DrawPoints(GLuint positions, GLuint color, int n, int offset, float radius, float screenWidth, float screenAspect, float fov, Vec3 lightPos, Vec3 lightTarget, Matrix44 lightTransform, GLuint shadowTex, bool showDensity, bool msaa);
void DrawMesh(const Mesh*, Vec3 color);
void DrawCloth(const Vec4* positions, const Vec4* normals, const float* uvs, const int* indices, int numTris, int numPositions, int colorIndex=3, float expand=0.0f, bool twosided=true, bool smooth=true);
void DrawBuffer(float* buffer, Vec3 camPos, Vec3 lightPos);
void DrawRope(Vec4* positions, int* indices, int numIndices, float radius, int color);
/*added*/
void DrawClothColor(const Vec4* positions, const Vec4* colors, const Vec4* normals, const float* uvs, const int* indices, int numTris, int numPositions, int colorIndex = 3, float expand = 0.0f, bool twosided = true, bool smooth = true);

void BindSolidShader(Vec3 lightPos, Vec3 lightTarget, Matrix44 lightTransform, GLuint shadowTex, float bias, Vec4 fogColor);
void UnbindSolidShader();

// new fluid renderer
struct FluidRenderer;

// buffers for a fluid system
struct FluidRenderBuffers
{
	GLuint mPositionVBO;
	GLuint mDensityVBO;
	GLuint mAnisotropyVBO[3];
	GLuint mDiffusePositionVBO;
	GLuint mDiffuseVelocityVBO;
	GLuint mDiffuseIndicesIBO;

	cudaGraphicsResource_t mPositionRes;
	cudaGraphicsResource_t mDensityRes;
	cudaGraphicsResource_t mAnisotropyRes[3];
	cudaGraphicsResource_t mDiffusePositionRes;
	cudaGraphicsResource_t mDiffuseVelocityRes;
	cudaGraphicsResource_t mDiffuseIndicesRes;

	int mNumFluidParticles;
	int mNumDiffuseParticles;
};

FluidRenderer* CreateFluidRenderer(uint32_t width, uint32_t height);
void DestroyFluidRenderer(FluidRenderer*);

FluidRenderBuffers CreateFluidRenderBuffers(int numParticles, int numDiffuseParticles);
void DestroyFluidRenderBuffers(FluidRenderBuffers buffers);

void RenderPoints(FluidRenderer*, GLuint positions, GLuint densities, int n, float radius, float screenWidth, float screenAspect, float fov, Vec3 lightPos, Vec3 lightTarget, Matrix44 lightTransform, GLuint shadowTex, Vec3 color);
void RenderEllipsoids(FluidRenderer* render, GLuint positions, GLuint q1, GLuint q2, GLuint q3, GLuint densities, int n, int offset, float radius, float screenWidth, float screenAspect, float fov, Vec3 lightPos, Vec3 lightTarget, Matrix44 lightTransform, GLuint shadowTex, GLuint reflectTex, Vec4 color, float blur, float ior, bool debug);
void RenderDiffuse(FluidRenderer* render, GLuint positions, GLuint velocities, GLuint indices, int n, float radius, float screenWidth, float screenAspect, float fov, Vec4 color, Vec3 lightPos, Vec3 lightTarget, Matrix44 lightTransform, GLuint shadowTex, float motionBlur,  float inscatter, float outscatter, bool shadow, bool front);
