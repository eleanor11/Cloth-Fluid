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

#include <cuda.h>
#include <cuda_runtime_api.h>

#include <vector>
#include <limits>
#include <algorithm>

#include "../core/maths.h"
#include "../core/voxelize.h"
#include "../core/sdf.h"

#include "../include/flex.h"
#include "../demo/cloth.h"

#include "flexExt.h"

#define cudaCheck(x) { cudaError_t err = x; if (err != cudaSuccess) { printf("Cuda error: %d in %s at %s:%d\n", err, #x, __FILE__, __LINE__); assert(0); } }

class Bitmap
{
public:

	typedef unsigned int Word;

	static const int kWordSize = sizeof(Word)*8;

	Bitmap(int numBits) : mBits((numBits+kWordSize-1)/kWordSize)
	{
	}

	inline void Set(int bit)
	{
		const int wordIndex = bit/kWordSize;
		const int bitIndex = bit&(kWordSize-1);

		const Word word = mBits[wordIndex];

		mBits[wordIndex] = word|(1<<bitIndex);
	}

	inline void Reset(int bit)
	{
		const int wordIndex = bit/kWordSize;
		const int bitIndex = bit&(kWordSize-1);

		const Word word = mBits[wordIndex];

		mBits[wordIndex] = word&~(1<<bitIndex);
	}

	inline bool IsSet(int bit)
	{
		const int wordIndex = bit/kWordSize;
		const int bitIndex = bit&(kWordSize-1);

		const Word word = mBits[wordIndex];

		return (word & (1<<bitIndex)) != 0;
	}

private:

	std::vector<Word> mBits;
};


// std::allocator compatible allocator for containers using pinned host memory (via flexAlloc)
template <class T>
struct PinnedAllocator 
{
   //    typedefs
	typedef T value_type;
	typedef value_type* pointer;
	typedef const value_type* const_pointer;
	typedef value_type& reference;
	typedef const value_type& const_reference;
	typedef std::size_t size_type;
	typedef std::ptrdiff_t difference_type;

    inline explicit PinnedAllocator() {}
    inline ~PinnedAllocator() {}
    inline explicit PinnedAllocator(PinnedAllocator const&) {}
    template<typename U>
    inline explicit PinnedAllocator(PinnedAllocator<U> const&) {}
  
	inline size_type max_size() const 
	{
	  	return std::numeric_limits<size_type>::max(); 
	}

	T* allocate(std::size_t n)
	{
		return (T*)flexAlloc(int(n*sizeof(T)));
	}

	void deallocate(T* p, std::size_t n)
	{
		flexFree(p);
	}

public : 
    //    convert an allocator<T> to allocator<U>
    template<typename U>
    struct rebind {
        typedef PinnedAllocator<U> other;
    };
	
	//    construction/destruction
	inline void construct(pointer p, const T& t) { new(p) T(t); }
	inline void destroy(pointer p) { p->~T(); }

};
template <class T, class U>
bool operator==(const PinnedAllocator<T>&, const PinnedAllocator<U>&);
template <class T, class U>
bool operator!=(const PinnedAllocator<T>&, const PinnedAllocator<U>&);


struct FlexExtContainer
{
	int mMaxParticles;
	
	FlexSolver* mSolver;

	// first n indices 
	std::vector<int, PinnedAllocator<int> > mActiveList;
	std::vector<int> mFreeList;

	std::vector<FlexExtInstance*> mInstances;

	// particles
	std::vector<Vec4, PinnedAllocator<Vec4> > mParticles;
	std::vector<Vec3, PinnedAllocator<Vec3> > mVelocities;
	std::vector<int,  PinnedAllocator<int> > mPhases;
	std::vector<Vec4, PinnedAllocator<Vec4> > mNormals;	

	// rigids
	std::vector<int> mRigidOffsets;
	std::vector<int> mRigidIndices;
	std::vector<float> mRigidCoefficients;
	std::vector<Matrix33, PinnedAllocator<Vec3> > mRigidRotations;
	std::vector<Vec3, PinnedAllocator<Vec3> > mRigidTranslations;
	std::vector<Vec3, PinnedAllocator<Vec3> > mRigidLocalPositions;

	// springs
	std::vector<int, PinnedAllocator<int> > mSpringIndices;
	std::vector<float, PinnedAllocator<float> > mSpringLengths;
	std::vector<float, PinnedAllocator<float> > mSpringCoefficients;

	// cloth
	std::vector<int, PinnedAllocator<int> > mTriangleIndices;
	std::vector<Vec3, PinnedAllocator<Vec3> > mTriangleNormals;	

	std::vector<int> mInflatableStarts;
	std::vector<int> mInflatableCounts;	
	std::vector<float> mInflatableRestVolumes;
	std::vector<float> mInflatableCoefficients;
	std::vector<float> mInflatableOverPressures;

	//force fields
	FlexExtForceField* mForceFieldsGpu;
	int mMaxForceFields;
	int mNumForceFields;
	int* mTmpActiveIndicesGpu;
	Vec4* mTmpParticlesGpu;
	Vec3* mTmpVelocitiesGpu;
	int mMaxTmpParticles;

	// needs compact
	bool mNeedsCompact;
	// needs to update active list
	bool mNeedsActiveListRebuild;
};



namespace
{
const int kNumThreadsPerBlock = 256;

// writes data to the device depending on source type
void WriteDeviceData(void *dst, const void *src, size_t count, FlexMemory source)
{
	cudaMemcpyKind kind;

	// host or device source
	if (source == eFlexMemoryHost || source == eFlexMemoryHostAsync)
		kind = cudaMemcpyHostToDevice;
	else
		kind = cudaMemcpyDeviceToDevice;
			
	// synchronous or async copy
	if (source == eFlexMemoryHostAsync || source == eFlexMemoryDeviceAsync)		
	{
		cudaCheck(cudaMemcpyAsync(dst, src, count, kind, 0));
	}
	else
	{
		cudaCheck(cudaMemcpy(dst, src, count, kind));
	}
}

// compacts all constraints into linear arrays
void CompactObjects(FlexExtContainer* c)
{
	// rigids
	c->mRigidOffsets.resize(1);
	c->mRigidIndices.resize(0);
	c->mRigidCoefficients.resize(0);
	c->mRigidRotations.resize(0);
	c->mRigidLocalPositions.resize(0);
	c->mRigidTranslations.resize(0);

	int totalNumSprings = 0;
	int totalNumTris = 0;

	// pre-calculate array sizes
	for (size_t i=0; i < c->mInstances.size(); ++i)
	{
		FlexExtInstance* inst = c->mInstances[i];
		
		const FlexExtAsset* asset = inst->mAsset;

		// index into the triangle array for this instance
		inst->mTriangleIndex = totalNumTris;

		totalNumSprings += asset->mNumSprings;
		totalNumTris += asset->mNumTriangles;
	}

	// springs
	c->mSpringIndices.resize(totalNumSprings*2);
	c->mSpringLengths.resize(totalNumSprings);
	c->mSpringCoefficients.resize(totalNumSprings);

	// cloth
	c->mTriangleIndices.resize(totalNumTris*3);
	c->mTriangleNormals.resize(totalNumTris);

	// inflatables
	c->mInflatableStarts.resize(0);
	c->mInflatableCounts.resize(0);	
	c->mInflatableRestVolumes.resize(0);
	c->mInflatableCoefficients.resize(0);
	c->mInflatableOverPressures.resize(0);

	int* __restrict dstSpringIndices = (totalNumSprings)?&c->mSpringIndices[0]:NULL;
	float* __restrict dstSpringLengths = (totalNumSprings)?&c->mSpringLengths[0]:NULL;
	float* __restrict dstSpringCoefficients = (totalNumSprings)?&c->mSpringCoefficients[0]:NULL;

	int* __restrict dstTriangleIndices = (totalNumTris)?&c->mTriangleIndices[0]:NULL;	

	// go through each instance and update springs, rigids, etc
	for (size_t i=0; i < c->mInstances.size(); ++i)
	{
		FlexExtInstance* inst = c->mInstances[i];
		
		const FlexExtAsset* asset = inst->mAsset;

		// map indices from the asset to the instance
		const int* __restrict remap = &inst->mParticleIndices[0];
		
		// flatten spring data
		int numSprings = asset->mNumSprings;
		const int numSpringIndices = asset->mNumSprings*2;
		const int* __restrict srcSpringIndices = asset->mSpringIndices;		

		for (int i=0; i < numSpringIndices; ++i)
		{
			*dstSpringIndices = remap[*srcSpringIndices];

			++dstSpringIndices;
			++srcSpringIndices;
		}

		memcpy(dstSpringLengths, asset->mSpringRestLengths, numSprings*sizeof(float));
		memcpy(dstSpringCoefficients, asset->mSpringCoefficients, numSprings*sizeof(float));
		
		dstSpringLengths += numSprings;
		dstSpringCoefficients += numSprings;

		// rigids
		if (asset->mRigidStiffness > 0.0f)
		{
			inst->mRigidIndex = int(c->mRigidOffsets.size())-1;

			for (int i=0; i < asset->mNumParticles; ++i)
			{
				c->mRigidIndices.push_back(remap[i]);
				c->mRigidLocalPositions.push_back(Vec3(&asset->mParticles[i*4])-Vec3(asset->mRigidCenter));
			}

			// start index of rigid
			c->mRigidOffsets.push_back(int(c->mRigidIndices.size()));
			c->mRigidCoefficients.push_back(asset->mRigidStiffness);
			c->mRigidRotations.push_back(Matrix33::Identity());
			c->mRigidTranslations.push_back(Vec3());
		}

		if (asset->mNumTriangles)
		{
			// triangles
			const int numTriIndices = asset->mNumTriangles*3;
			const int* __restrict srcTriIndices = asset->mTriangleIndices;

			for (int i=0; i < numTriIndices; ++i)
			{
				*dstTriangleIndices = remap[*srcTriIndices];

				++dstTriangleIndices;
				++srcTriIndices;
			}			

			if (asset->mInflatable)
			{
				c->mInflatableStarts.push_back(inst->mTriangleIndex);
				c->mInflatableCounts.push_back(asset->mNumTriangles);
				c->mInflatableRestVolumes.push_back(asset->mInflatableVolume);
				c->mInflatableCoefficients.push_back(asset->mInflatableStiffness);
				c->mInflatableOverPressures.push_back(asset->mInflatablePressure);		
			}
		}
	}

	// springs
	if (c->mSpringLengths.size())
		flexSetSprings(c->mSolver, &c->mSpringIndices[0], &c->mSpringLengths[0], &c->mSpringCoefficients[0], int(c->mSpringLengths.size()), eFlexMemoryHostAsync);
	else
		flexSetSprings(c->mSolver, NULL, NULL, NULL, 0, eFlexMemoryHostAsync);

	// rigids
	if (c->mRigidCoefficients.size())
		flexSetRigids(c->mSolver, &c->mRigidOffsets[0], &c->mRigidIndices[0], (float*)&c->mRigidLocalPositions[0], NULL, &c->mRigidCoefficients[0], (float*)&c->mRigidRotations[0], int(c->mRigidCoefficients.size()), eFlexMemoryHostAsync);
	else
		flexSetRigids(c->mSolver, NULL, NULL, NULL, NULL, NULL, NULL, 0, eFlexMemoryHostAsync);

	// triangles
	if (c->mTriangleIndices.size())
		flexSetDynamicTriangles(c->mSolver, &c->mTriangleIndices[0], NULL, int(c->mTriangleIndices.size()/3), eFlexMemoryHostAsync);
	else
		flexSetDynamicTriangles(c->mSolver, NULL, NULL, 0, eFlexMemoryHostAsync);

	// inflatables
	if (c->mInflatableCounts.size())
		flexSetInflatables(c->mSolver, &c->mInflatableStarts[0], &c->mInflatableCounts[0], &c->mInflatableRestVolumes[0], &c->mInflatableOverPressures[0], &c->mInflatableCoefficients[0], int(c->mInflatableCounts.size()),eFlexMemoryHost);
	else
		flexSetInflatables(c->mSolver, NULL, NULL, NULL, NULL, NULL, 0, eFlexMemoryHostAsync);

	c->mNeedsCompact = false;
}

} // anonymous namespace


FlexExtContainer* flexExtCreateContainer(FlexSolver* solver, int maxParticles)
{
	FlexExtContainer* c = new FlexExtContainer();

	c->mSolver = solver;
	c->mMaxParticles = maxParticles;

	// initialize free list
	c->mFreeList.resize(maxParticles);
	for (int i=0; i < maxParticles; ++i)
		c->mFreeList[i] = i;

	c->mActiveList.resize(maxParticles);
	c->mParticles.resize(maxParticles);
	c->mVelocities.resize(maxParticles);
	c->mPhases.resize(maxParticles);
	c->mNormals.resize(maxParticles);

	// force fields
	c->mForceFieldsGpu = NULL;
	c->mMaxForceFields = 0;
	c->mNumForceFields = 0;
	c->mTmpActiveIndicesGpu = NULL;
	c->mTmpParticlesGpu = NULL;
	c->mTmpVelocitiesGpu = NULL;
	c->mMaxTmpParticles = 0;

	c->mNeedsCompact = false;

	return c;
}

void flexExtDestroyContainer(FlexExtContainer* c)
{
	// force fields
	cudaCheck(cudaFree(c->mForceFieldsGpu));
	cudaCheck(cudaFree(c->mTmpActiveIndicesGpu));
	cudaCheck(cudaFree(c->mTmpParticlesGpu));
	cudaCheck(cudaFree(c->mTmpVelocitiesGpu));

	delete c;
}

int flexExtAllocParticles(FlexExtContainer* c, int n, int* indices)
{
	const int numToAlloc = Min(int(c->mFreeList.size()), n);
	const int start = int(c->mFreeList.size())-numToAlloc;

	if (numToAlloc)
	{
		memcpy(indices, &c->mFreeList[start], numToAlloc*sizeof(int));
		c->mFreeList.resize(start);
	}

	c->mNeedsActiveListRebuild = true;

	return numToAlloc;
}

void flexExtFreeParticles(FlexExtContainer* c, int n, const int* indices)
{
#if _DEBUG
	for (int i=0; i < n; ++i)
	{
		// check valid values
		assert(indices[i] >= 0 && indices[i] < int(c->mFreeList.capacity()));

		// check for double delete
		assert(std::find(c->mFreeList.begin(), c->mFreeList.end(), indices[i]) == c->mFreeList.end());
	}
#endif

	c->mFreeList.insert(c->mFreeList.end(), indices, indices+n);

	c->mNeedsActiveListRebuild = true;
}

int flexExtGetActiveList(FlexExtContainer* c, int* indices)
{
	int count = 0;

	Bitmap inactive(c->mMaxParticles);

	// create bitmap
	for (size_t i=0; i < c->mFreeList.size(); ++i)
	{
		// if this fires then somehow a duplicate has ended up in the free list (double delete)
		assert(!inactive.IsSet(c->mFreeList[i]));

		inactive.Set(c->mFreeList[i]);
	}

	// iterate bitmap to find active elements
	for (int i=0; i < c->mMaxParticles; ++i)
		if (inactive.IsSet(i) == false)
			indices[count++] = i;

	return count;
}

void flexExtGetParticleData(FlexExtContainer* c, float** particles, float** velocities, int** phases, float** normals)
{
	if (particles && c->mParticles.size())
		*particles = (float*)&c->mParticles[0];

	if (velocities && c->mVelocities.size())
		*velocities = (float*)&c->mVelocities[0];

	if (phases && c->mPhases.size())
		*phases = (int*)&c->mPhases[0];

	if (normals && c->mNormals.size())
		*normals = (float*)&c->mNormals[0];
}

void flexExtGetTriangleData(FlexExtContainer* c, int** indices, float** normals)
{
	if (indices && c->mTriangleIndices.size())
		*indices = &c->mTriangleIndices[0];
	
	if (normals && c->mTriangleNormals.size())
		*normals = (float*)&c->mTriangleNormals[0];
}

void flexExtGetRigidData(FlexExtContainer* c, float** rotations, float** positions)
{
	if (rotations && c->mRigidRotations.size())
		*rotations = (float*)&c->mRigidRotations[0];

	if (positions && c->mRigidTranslations.size())
		*positions = (float*)&c->mRigidTranslations[0];
}

FlexExtInstance* flexExtCreateInstance(FlexExtContainer* c, const FlexExtAsset* asset, const float* transform, float vx, float vy, float vz, int phase, float invMassScale)
{	
	const int numParticles = asset->mNumParticles;

	// check if asset will fit
	if (int(c->mFreeList.size()) < numParticles)
		return NULL;

	FlexExtInstance* inst = new FlexExtInstance();

	inst->mAsset = asset;
	inst->mTriangleIndex = -1;
	inst->mRigidIndex = -1;
	inst->mInflatableIndex = -1;
	inst->mUserData = NULL;
	inst->mNumParticles = numParticles;
	
	// allocate particles for instance
	inst->mParticleIndices = new int[numParticles];
	int n = flexExtAllocParticles(c, numParticles, &inst->mParticleIndices[0]);
	assert(n == numParticles);
	(void)n;

	c->mInstances.push_back(inst);

	const Matrix44 xform(transform);

	for (int i=0; i < numParticles; ++i)
	{
		const int index = inst->mParticleIndices[i];

		// add transformed particles to the container
		c->mParticles[index] = xform*Vec4(Vec3(&asset->mParticles[i*4]), 1.0f);
		c->mParticles[index].w = asset->mParticles[i*4+3]*invMassScale;
		c->mVelocities[index] = Vec3(vx, vy, vz);
		c->mPhases[index] = phase;
		c->mNormals[index] = Vec4(0.0f);
	}

	c->mNeedsCompact = true;
	c->mNeedsActiveListRebuild = true;

	return inst;
}

void flexExtDestroyInstance(FlexExtContainer* c, const FlexExtInstance* inst)
{
	flexExtFreeParticles(c, inst->mNumParticles, &inst->mParticleIndices[0]);
	delete[] inst->mParticleIndices;

	// TODO: O(N) remove
	std::vector<FlexExtInstance*>::iterator iter = std::find(c->mInstances.begin(), c->mInstances.end(), inst);
	assert(iter != c->mInstances.end());
	c->mInstances.erase(iter);

	c->mNeedsCompact = true;
	c->mNeedsActiveListRebuild = true;

	delete inst;
}

void flexExtTickContainer(FlexExtContainer* c, float dt, int substeps, FlexTimers* timers)
{
	// update the device
	flexExtPushToDevice(c);

	// update solver
	flexUpdateSolver(c->mSolver, dt, substeps, timers);

	// update host
	flexExtPullFromDevice(c);

	// ensure memory transfers have finished
	flexSetFence();
	flexWaitFence();
}

void flexExtPushToDevice(FlexExtContainer* c)
{
	if (c->mNeedsActiveListRebuild)
	{
		// update active list
		int n = flexExtGetActiveList(c, &c->mActiveList[0]);
		flexSetActive(c->mSolver, &c->mActiveList[0], n, eFlexMemoryHostAsync);

		c->mNeedsActiveListRebuild = false;
	}

	// push any changes to soler
	flexSetParticles(c->mSolver, (float*)&c->mParticles[0], int(c->mParticles.size()), eFlexMemoryHostAsync);
	flexSetVelocities(c->mSolver, (float*)&c->mVelocities[0], int(c->mVelocities.size()), eFlexMemoryHostAsync);
	flexSetPhases(c->mSolver, &c->mPhases[0], int(c->mPhases.size()), eFlexMemoryHostAsync);
	flexSetNormals(c->mSolver, (float*)&c->mNormals[0], int(c->mNormals.size()), eFlexMemoryHostAsync);
	
	if (c->mNeedsCompact)
		CompactObjects(c);
}

void flexExtPullFromDevice(FlexExtContainer* c)
{
	// read back particle data
	flexGetParticles(c->mSolver, (float*)&c->mParticles[0], int(c->mParticles.size()), eFlexMemoryHostAsync);
	flexGetVelocities(c->mSolver, (float*)&c->mVelocities[0], int(c->mVelocities.size()), eFlexMemoryHostAsync);
	flexGetPhases(c->mSolver, &c->mPhases[0], int(c->mPhases.size()), eFlexMemoryHostAsync);
	flexGetNormals(c->mSolver, (float*)&c->mNormals[0], int(c->mNormals.size()), eFlexMemoryHostAsync);

	// read back rigid transforms
	if (c->mRigidCoefficients.size())
		flexGetRigidTransforms(c->mSolver, (float*)&c->mRigidRotations[0], (float*)&c->mRigidTranslations[0], eFlexMemoryHostAsync);
}

namespace
{
	struct Key
	{
		Key(int i, float d) : index(i), depth(d) {}

		int index;
		float depth;
		
		bool operator < (const Key& rhs) const { return depth < rhs.depth; }
	};
}

int flexExtCreateWeldedMeshIndices(const float* vertices, int numVertices, int* uniqueIndices, int* originalToUniqueMap, float threshold)
{
	memset(originalToUniqueMap, -1, numVertices*sizeof(int));

	const Vec3* positions = (const Vec3*)vertices;

	// use a sweep and prune style search to accelerate neighbor finding
	std::vector<Key> keys;
	for (int i=0; i < numVertices; i++)
		keys.push_back(Key(i, positions[i].z));

	std::sort(keys.begin(), keys.end());

	int uniqueCount = 0;

	// sweep keys to find matching verts
	for (int i=0; i < numVertices; ++i)
	{
		// we are a duplicate, skip
		if (originalToUniqueMap[keys[i].index] != -1)
			continue;

		// scan forward until no vertex can be closer than threshold
		for (int j=i+1; j < numVertices && (keys[j].depth-keys[i].depth) <= threshold; ++j)
		{
			float distance = Length(Vector3(positions[keys[i].index])-Vector3(positions[keys[j].index]));

			if (distance <= threshold)
				originalToUniqueMap[keys[j].index] = uniqueCount;
		}

		originalToUniqueMap[keys[i].index] = uniqueCount;

		uniqueIndices[uniqueCount++] = keys[i].index;
	}

	return uniqueCount;
}

namespace
{

float SampleSDF(const float* sdf, int dim, int x, int y, int z)
{
	assert(x < dim && x >= 0);
	assert(y < dim && y >= 0);
	assert(z < dim && z >= 0);

	return sdf[z*dim*dim + y*dim + x];
}

// return normal of signed distance field
Vec3 SampleSDFGrad(const float* sdf, int dim, int x, int y, int z)
{
	int x0 = std::max(x-1, 0);
	int x1 = std::min(x+1, dim-1);

	int y0 = std::max(y-1, 0);
	int y1 = std::min(y+1, dim-1);

	int z0 = std::max(z-1, 0);
	int z1 = std::min(z+1, dim-1);

	float dx = (SampleSDF(sdf, dim, x1, y, z) - SampleSDF(sdf, dim, x0, y, z))*(dim*0.5f);
	float dy = (SampleSDF(sdf, dim, x, y1, z) - SampleSDF(sdf, dim, x, y0, z))*(dim*0.5f);
	float dz = (SampleSDF(sdf, dim, x, y, z1) - SampleSDF(sdf, dim, x, y, z0))*(dim*0.5f);

	return Vec3(dx, dy, dz);
}

} // anonymous namespace

FlexExtAsset* flexExtCreateRigidFromMesh(const float* vertices, int numVertices, const int* indices, int numTriangleIndices, float spacing)
{
	std::vector<Vec4> particles;
	std::vector<Vec4> normals;
	std::vector<int> phases;

	const Vec3* positions = (Vec3*)vertices;

	Vec3 meshLower(FLT_MAX), meshUpper(-FLT_MAX);
	for (int i=0; i < numVertices; ++i)
	{
		meshLower = Min(meshLower, positions[i]);
		meshUpper = Max(meshUpper, positions[i]);
	}

	Vec3 edges = meshUpper-meshLower;
	float maxEdge = std::max(std::max(edges.x, edges.y), edges.z);

	// tweak spacing to avoid edge cases for particles laying on the boundary
	// just covers the case where an edge is a whole multiple of the spacing.
	float spacingEps = spacing*(1.0f - 1e-4f);

	// make sure to have at least one particle in each dimension
	int dx, dy, dz;
	dx = spacing > edges.x ? 1 : int(edges.x/spacingEps);
	dy = spacing > edges.y ? 1 : int(edges.y/spacingEps);
	dz = spacing > edges.z ? 1 : int(edges.z/spacingEps);

	int maxDim = std::max(std::max(dx, dy), dz);

	// expand border by two voxels to ensure adequate sampling at edges
	meshLower -= 2.0f*Vec3(spacing);
	meshUpper += 2.0f*Vec3(spacing);
	maxDim += 4;

	// we shift the voxelization bounds so that the voxel centers
	// lie symmetrically to the center of the object. this reduces the 
	// chance of missing features, and also better aligns the particles
	// with the mesh
	Vec3 meshOffset;
	meshOffset.x = 0.5f * (spacing - (edges.x - (dx-1)*spacing));
	meshOffset.y = 0.5f * (spacing - (edges.y - (dy-1)*spacing));
	meshOffset.z = 0.5f * (spacing - (edges.z - (dz-1)*spacing));
	meshLower -= meshOffset;

	// don't allow samplings with > 64 per-side	
	if (maxDim > 64)
		return NULL;

	std::vector<uint32_t> voxels(maxDim*maxDim*maxDim);

	Voxelize(vertices, numVertices, indices, numTriangleIndices, maxDim, maxDim, maxDim, &voxels[0], meshLower, meshLower + Vec3(maxDim*spacing));

	std::vector<float> sdf(maxDim*maxDim*maxDim);
	MakeSDF(&voxels[0], maxDim, maxDim, maxDim, &sdf[0]);

	Vec3 center;

	for (int x=0; x < maxDim; ++x)
	{
		for (int y=0; y < maxDim; ++y)
		{
			for (int z=0; z < maxDim; ++z)
			{
				const int index = z*maxDim*maxDim + y*maxDim + x;

				// if voxel is marked as occupied the add a particle
				if (voxels[index])
				{
					Vec3 position = meshLower + spacing*Vec3(float(x) + 0.5f, float(y) + 0.5f, float(z) + 0.5f);

						// normalize the sdf value and transform to world scale
					Vec3 n = SafeNormalize(SampleSDFGrad(&sdf[0], maxDim, x, y, z));
					float d = sdf[index]*maxEdge;

					normals.push_back(Vec4(n, d));
					particles.push_back(Vec4(position.x, position.y, position.z, 1.0f));						
					phases.push_back(0);

					center += position;
				}
			}
		}
	}

	FlexExtAsset* asset = new FlexExtAsset();
	memset(asset, 0, sizeof(asset));

	if (particles.size())
	{
		// store center of mass
		center /= float(particles.size());
		asset->mRigidCenter[0] = center.x;
		asset->mRigidCenter[1] = center.y;
		asset->mRigidCenter[2] = center.z;

		asset->mNumParticles = int(particles.size());

		asset->mParticles = new float[particles.size()*4];
		memcpy(asset->mParticles, &particles[0], sizeof(Vec4)*particles.size());

		// todo: normals
	}

	return asset;
}

FlexExtAsset* flexExtCreateClothFromMesh(const float* particles, int numVertices, const int* indices, int numTriangles, float stretchStiffness, float bendStiffness, float tetherStiffness, float tetherGive, float pressure)
{
	FlexExtAsset* asset = new FlexExtAsset();
	memset(asset, 0, sizeof(asset));

	asset->mParticles = new float[numVertices*4];
	memcpy(asset->mParticles, particles, sizeof(float)*4);	

	asset->mTriangleIndices = new int[numTriangles*3];
	memcpy(asset->mTriangleIndices, indices, numTriangles*3*sizeof(int));

	asset->mNumParticles = numVertices;
	asset->mNumTriangles = numTriangles;

	// create cloth mesh
	ClothMesh cloth((Vec4*)particles, numVertices, indices, numTriangles*3, stretchStiffness, bendStiffness, true);

	if (cloth.mValid)
	{
		// create tethers
		if (tetherStiffness > 0.0f)
		{
			std::vector<int> anchors;
			anchors.reserve(numVertices);

			// find anchors
			for (int i=0; i < numVertices; ++i)
			{
				Vec4& particle = ((Vec4*)particles)[i];

				if (particle.w == 0.0f)
					anchors.push_back(i);
			}

			if (anchors.size())
			{
				// create tethers
				for (int i=0; i < numVertices; ++i)
				{
					Vec4& particle = ((Vec4*)particles)[i];
					if (particle.w == 0.0f)
						continue;

					float minSqrDist = FLT_MAX;
					int minIndex = -1;

					// find the closest attachment point
					for (int a=0; a < int(anchors.size()); ++a)
					{
						Vec4& attachment = ((Vec4*)particles)[anchors[a]];

						float distSqr = LengthSq(Vec3(particle)-Vec3(attachment));
						if (distSqr < minSqrDist)
						{
							minSqrDist = distSqr;
							minIndex = anchors[a];
						}
					}

					// add a tether
					if (minIndex != -1)
					{						
						cloth.mConstraintIndices.push_back(i);
						cloth.mConstraintIndices.push_back(minIndex);
						cloth.mConstraintRestLengths.push_back(sqrtf(minSqrDist)*(1.0f + tetherGive));						
						
						// negative stiffness indicates tether (unilateral constraint)
						cloth.mConstraintCoefficients.push_back(-tetherStiffness);
					}
				}
			}
		}

		const int numSprings = int(cloth.mConstraintCoefficients.size());

		asset->mSpringIndices = new int[numSprings*2];
		asset->mSpringCoefficients = new float[numSprings];
		asset->mSpringRestLengths = new float[numSprings];
		asset->mNumSprings = numSprings;

		for (int i=0; i < numSprings; ++i)
		{
			asset->mSpringIndices[i*2+0] = cloth.mConstraintIndices[i*2+0];
			asset->mSpringIndices[i*2+1] = cloth.mConstraintIndices[i*2+1];			
			asset->mSpringRestLengths[i] = cloth.mConstraintRestLengths[i];
			asset->mSpringCoefficients[i] = cloth.mConstraintCoefficients[i];
		}

		if (pressure > 0.0f)
		{
			asset->mInflatable = true;
			asset->mInflatableVolume = cloth.mRestVolume;
			asset->mInflatableStiffness = cloth.mConstraintScale;
			asset->mInflatablePressure = pressure;
		}
	}
	else
	{
		flexExtDestroyAsset(asset);
		return NULL;
	}

	return asset;
}

void flexExtDestroyAsset(FlexExtAsset* asset)
{
	delete[] asset->mParticles;
	delete[] asset->mSpringIndices;
	delete[] asset->mSpringCoefficients;
	delete[] asset->mSpringRestLengths;
	delete[] asset->mTriangleIndices;

	delete asset;
}

void flexExtSetForceFields(FlexExtContainer* c, const FlexExtForceField* forceFields, int numForceFields, FlexMemory source)
{
	// re-alloc if necessary
	if (numForceFields > c->mMaxForceFields)
	{
		cudaCheck(cudaFree(c->mForceFieldsGpu));
		cudaCheck(cudaMalloc(&c->mForceFieldsGpu, sizeof(FlexExtForceField)*numForceFields));
		c->mMaxForceFields = numForceFields;
	}
	c->mNumForceFields = numForceFields;

	if (numForceFields > 0)
	{
		WriteDeviceData(c->mForceFieldsGpu, forceFields, numForceFields*sizeof(FlexExtForceField), source);
	}
}

__global__ void UpdateForceFields(int numParticles, const int* __restrict__ activeIndices, const Vec4* __restrict__ positions, Vec3* __restrict__ velocities, const FlexExtForceField* __restrict__ forceFields, int numForceFields, float dt)
{
	const int i = blockIdx.x*blockDim.x + threadIdx.x;
	
	for (int f = 0; f < numForceFields; f++)
	{
		const FlexExtForceField& forceField = forceFields[f];

		if (i < numParticles)
		{
			const int index = activeIndices[i];

			Vec4 p = positions[index];
			Vec3 v = velocities[index];

			Vec3 localPos = Vec3(p.x, p.y, p.z) - Vec3(forceField.mPosition[0], forceField.mPosition[1], forceField.mPosition[2]);

			float length = Length(localPos);
			if (length > forceField.mRadius)
			{
				continue;
			}
			
			Vec3 fieldDir;
			if (length > 0.0f)
			{
				fieldDir = localPos / length;
			}
			else
			{
				fieldDir = localPos;
			}

			// If using linear falloff, scale with distance.
			float fieldStrength = forceField.mStrength;
			if (forceField.mLinearFalloff)
			{
				fieldStrength *= (1.0f - (length / forceField.mRadius));
			}

			// Apply force
			Vec3 force = localPos * fieldStrength;

			float unitMultiplier;
			if (forceField.mMode == eFlexExtModeForce)
			{
				unitMultiplier = dt * p.w; // time/mass
			} 
			else if (forceField.mMode == eFlexExtModeImpulse)
			{
				unitMultiplier = p.w; // 1/mass
			}
			else if (forceField.mMode == eFlexExtModeVelocityChange)
			{
				unitMultiplier = 1.0f;
			}

			Vec3 deltaVelocity = fieldDir * fieldStrength * unitMultiplier;
			velocities[index] = v + deltaVelocity;
		}
	}
}

void flexExtApplyForceFields(FlexExtContainer* c, float dt)
{
	int numParticles = flexGetActiveCount(c->mSolver);

	if (numParticles && c->mNumForceFields)
	{
		// reallocate temp buffers if necessary
		if (int(c->mParticles.size()) > c->mMaxTmpParticles)
		{
			c->mMaxTmpParticles = int(c->mParticles.size());
			cudaCheck(cudaFree(c->mTmpActiveIndicesGpu));
			cudaCheck(cudaFree(c->mTmpParticlesGpu));
			cudaCheck(cudaFree(c->mTmpVelocitiesGpu));
			cudaCheck(cudaMalloc(&c->mTmpActiveIndicesGpu, sizeof(int)*c->mMaxTmpParticles));
			cudaCheck(cudaMalloc(&c->mTmpParticlesGpu, sizeof(Vec4)*c->mMaxTmpParticles));
			cudaCheck(cudaMalloc(&c->mTmpVelocitiesGpu, sizeof(Vec3)*c->mMaxTmpParticles));
		}

		flexGetActive(c->mSolver, c->mTmpActiveIndicesGpu, eFlexMemoryDeviceAsync);
		flexGetParticles(c->mSolver, (float*)c->mTmpParticlesGpu, int(c->mParticles.size()), eFlexMemoryDeviceAsync);
		flexGetVelocities(c->mSolver, (float*)c->mTmpVelocitiesGpu, int(c->mParticles.size()), eFlexMemoryDeviceAsync);

		const int kNumBlocks = (numParticles+kNumThreadsPerBlock-1)/kNumThreadsPerBlock;

		UpdateForceFields<<<kNumBlocks, kNumThreadsPerBlock>>>(
			numParticles,
			c->mTmpActiveIndicesGpu,
	   		c->mTmpParticlesGpu,
	   		c->mTmpVelocitiesGpu,
			c->mForceFieldsGpu,
			c->mNumForceFields,
			dt
			);

		flexSetVelocities(c->mSolver, (float*)c->mTmpVelocitiesGpu, int(c->mParticles.size()), eFlexMemoryDeviceAsync);
	}
}

