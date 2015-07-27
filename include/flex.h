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


#ifndef FLEX_H
#define FLEX_H

//! \cond HIDDEN_SYMBOLS
#if _WIN32
#define FLEX_API __declspec(dllexport)
#else
#define FLEX_API
#endif

// least 2 significant digits define minor version, eg: 10 -> version 0.10
#define FLEX_VERSION 80

//! \endcond

extern "C" {

/**
 * Opaque type representing a collection of particles and constraints
 */
struct FlexSolver;

/**
 * Simulation parameters for a solver
 */
enum FlexRelaxationMode
{
	eFlexRelaxationGlobal = 0,			//!< The relaxation factor is a fixed multiplier on each constraint's position delta
	eFlexRelaxationLocal  = 1			//!< The relaxation factor is a fixed multiplier on each constraint's delta divided by the particle's constraint count, convergence will be slower but more reliable
};

struct FlexParams
{
    int mNumIterations;                 //!< Number of solver iterations to perform per-substep

    float mGravity[3];                  //!< Constant acceleration applied to all particles
    float mRadius;                      //!< The maximum interaction radius for particles
    float mSolidRestDistance;			//!< The distance non-fluid particles attempt to maintain from each other, must be in the range (0, radius]
	float mFluidRestDistance;           //!< The distance fluid particles are spaced at the rest density, must be in the range (0, radius], for fluids this should generally be 50-70% of mRadius, for rigids this can simply be the same as the particle radius

    // common params
    float mDynamicFriction;             //!< Coefficient of friction used when colliding against shapes
	float mStaticFriction;				//!< Coefficient of static friction used when colliding against shapes
	float mParticleFriction;			//!< Coefficient of friction used when colliding particles
    float mRestitution;                 //!< Coefficient of restitution used when colliding against shapes, particle collisions are always inelastic
	float mAdhesion;                    //!< Controls how strongly particles stick to surfaces they hit, default 0.0, range [0.0, +inf]
    float mSleepThreshold;              //!< Particles with a velocity magnitude < this threshold will be considered fixed
    float mMaxVelocity;                 //!< Particle velocity will be clamped to this value at the end of each step
    float mShockPropagation;            //!< Artificially decrease the mass of particles based on height from a fixed reference point, this makes stacks and piles converge faster
    float mDissipation;                 //!< Damps particle velocity based on how many particle contacts it has
	float mDamping;						//!< Viscous drag force, applies a force proportional, and opposite to the particle velocity
	float mEnableCCD;					//!< If true then a second collision detection pass will be executed against triangle meshes to prevent tunneling, usually not necessary, only enable if having tunnelling problems
	
    // cloth params
    float mWind[3];                     //!< Constant acceleration applied to particles that belong to dynamic triangles, drag needs to be > 0 for wind to affect triangles
    float mDrag;                        //!< Drag force applied to particles belonging to dynamic triangles, proportional to velocity^2*area in the negative velocity direction
    float mLift;                        //!< Lift force applied to particles belonging to dynamic triangles, proportional to velocity^2*area in the direction perpendicular to velocity and (if possible), parallel to the plane normal

    // fluid params
    bool mFluid;                        //!< If true then particles with phase 0 are considered fluid particles and interact using the position based fluids method
    float mCohesion;                    //!< Control how strongly particles hold each other together, default: 0.025, range [0.0, +inf]
    float mSurfaceTension;              //!< Controls how strongly particles attempt to minimize surface area, default: 0.0, range: [0.0, +inf]    
    float mViscosity;                   //!< Smoothes particle velocities using XSPH viscosity
    float mVorticityConfinement;        //!< Increases vorticity by applying rotational forces to particles
    float mAnisotropyScale;             //!< Control how much anisotropy is present in resulting ellipsoids for rendering, if zero then anisotropy will not be calculated, see flexGetAnisotropy()
    float mSmoothing;                   //!< Control the strength of Laplacian smoothing in particles for rendering, if zero then smoothed positions will not be calculated, see flexGetSmoothParticles()
    float mSolidPressure;               //!< Add pressure from solid surfaces to particles
	float mFreeSurfaceDrag;				//!< Drag force applied to boundary fluid particles
	float mBuoyancy;					//!< Gravity is scaled by this value for fluid particles

	// diffuse params
    float mDiffuseThreshold;            //!< Particles with kinetic energy + divergence above this threshold will spawn new diffuse particles
	float mDiffuseBuoyancy;				//!< Scales force opposing gravity that diffuse particles receive
	float mDiffuseDrag;					//!< Scales force diffuse particles receive in direction of neighbor fluid particles
	int mDiffuseBallistic;				//!< The number of neighbors below which a diffuse particle is considered ballistic
	float mDiffuseSortAxis[3];			//!< Diffuse particles will be sorted by depth along this axis if non-zero

    // rigid params
    float mPlasticThreshold;            //!< Particles belonging to rigid shapes that move with a position delta magnitude > threshold will be permanently deformed in the rest pose
    float mPlasticCreep;                //!< Controls the rate at which particles in the rest pose are deformed for particles passing the deformation threshold 

    // collision params
    float mCollisionDistance;           //!< Distance particles maintain against shapes	
    float mParticleCollisionMargin;     //!< Increases the radius used during neighbor finding, this is useful if particles are expected to move significantly during a single step to ensure contacts aren't missed on subsequent iterations
	float mShapeCollisionMargin;		//!< Increases the radius used during contact finding against kinematic shapes

	float mPlanes[8][4];				//!< Collision planes in the form ax + by + cz + d = 0
	int mNumPlanes;						//!< Num collision planes

	FlexRelaxationMode mRelaxationMode;	//!< How the relaxation is applied inside the solver
    float mRelaxationFactor;			//!< Control the convergence rate of the parallel solver, default: 1, values greater than 1 may lead to instability
};

/**
 * Flags that control the a particle's behavior and grouping
 */
enum FlexPhase
{
	eFlexPhaseGroupMask = 0x00ffffff,		//!< Low 24 bits represent the particle group for controlling collisions	

	eFlexPhaseSelfCollide = 1 << 24,	//!< Flag specifying whether this particle will interact with particles of the same group
	eFlexPhaseFluid = 1 << 25,			//!< Flag specifying whether this particle will generate fluid density constraints for its overlapping neighbors	
};

/**
 * Generate a bit set for the particle phase, the group should be an integer < 2^24, and the flags should be a combination of FlexPhase enum values
 */
FLEX_API inline int flexMakePhase(int group, int flags) { return (group & eFlexPhaseGroupMask) | flags; }

/**
 * Signed distance field collision shape, note that only cubic fields are currently supported (mWidth=mHeight=mDepth)
 */
struct FlexSDF
{
	float mLower[3];					//!< Shape AABB lower bounds in world space
	float mUpper[3];					//!< Shape AABB upper bounds in world space
	float mInvEdgeLength[3];			//!< 1/(mUpper-mLower)

	unsigned int mWidth;				//!< Field x dimension in voxels
	unsigned int mHeight;				//!< Field y dimension in voxels
	unsigned int mDepth;				//!< Field z dimension in voxels

	const float* mField;				//!< SDF voxel data, must be mWidth*mHeight*mDepth in length
};


/**
 * Time spent in each section of the solver update, times in seconds, see flexUpdateSolver()
 */
struct FlexTimers
{
	float mPredict;						//!< Time spent in prediction
	float mCreateCellIndices;			//!< Time spent creating grid indices
	float mSortCellIndices;				//!< Time spent sorting grid indices
	float mCreateGrid;					//!< Time spent creating grid
	float mReorder;						//!< Time spent reordering particles
	float mCollideParticles;			//!< Time spent finding particle neighbors
	float mCollideConvexes;				//!< Time spent colliding convex shapes
	float mCollideTriangles;			//!< Time spent colliding triangle shapes
	float mCollideFields;				//!< Time spent colliding signed distance field shapes
	float mCalculateDensity;			//!< Time spent calculating fluid density
	float mSolveDensities;				//!< Time spent solving density constraints
	float mSolveVelocities;				//!< Time spent solving velocity constraints
	float mSolveShapes;					//!< Time spent solving rigid body constraints
	float mSolveSprings;				//!< Time spent solving distance constraints
	float mSolveContacts;				//!< Time spent solving contact constraints
	float mSolveInflatables;			//!< Time spent solving pressure constraints
	float mCalculateAnisotropy;			//!< Time spent calculating particle anisotropy for fluid
	float mUpdateDiffuse;				//!< Time spent updating diffuse particles
	float mUpdateTriangles;				//!< Time spent updating dynamic triangles
	float mUpdateNormals;				//!< Time spent updating vertex normals
	float mFinalize;					//!< Time spent finalizing state
	float mUpdateBounds;				//!< Time spent updating particle bounds
	float mTotal;						//!< Sum of all timers above
};


/**
 * Flex error types
 */
enum FlexError
{
	//! The API call returned with no errors. 
    eFlexErrorNone   			= 0,

    //! The header version does not match the library binary
    eFlexErrorWrongVersion		= 1,

	//! The GPU associated with the calling thread does not meet
	//! requirements. An SM3.0 GPU or above is required
	eFlexErrorInsufficientGPU	= 2,
};
  

/**
 * Designates a memory space for getting/settings data to/from
 */
enum FlexMemory
{
	//! Host (CPU) memory
	eFlexMemoryHost					= 0,		

	//! Device (GPU) memory
	eFlexMemoryDevice				= 1,

	//! Host (CPU) memory asynchronous, when used
	//! with a flexGet/flexSet method the memory
	//! transfer will be asynchronous and should be
	//! synchronized with flexWaitFence()
	eFlexMemoryHostAsync			= 2,

	//! Device (GPU) memory asynchronous, when used
	//! with a flexGet/flexSet method the memory
	//! transfer will be asynchronous and should be
	//! synchronized with flexWaitFence()
	eFlexMemoryDeviceAsync			= 3
};

/**
 * Error reporting callback.
 */
typedef void (*FlexErrorCallback)(const char* msg, const char* file, int line);

/**
 * Initialize library, should be called before any other API function.
 *
 * @note Flex uses the calling thread's CUDA context for all operations 
 * so users should make sure the same context is used for all API calls 
 * (this should be the case if just using the default runtime context).
 */
FLEX_API FlexError flexInit(int version=FLEX_VERSION, FlexErrorCallback errorFunc=NULL);

/**
 * Shutdown library, users should manually destroy any previously created   
 * solvers to ensure memory is freed before calling this method.
 */
FLEX_API void flexShutdown();

/**
 * Get library version number
 */
FLEX_API int flexGetVersion();

/**
 * Create a new particle solver
 *
 * @param[in] maxParticles Maximum number of simulation particles possible for this solver
 * @param[in] maxDiffuseParticles Maximum number of diffuse (non-simulation) particles possible for this solver
 * @param[in] maxNeighborsPerParticle Maximum number of neighbors per particle possible for this solver
 */
FLEX_API FlexSolver* flexCreateSolver(int maxParticles, int maxDiffuseParticles, unsigned char maxNeighborsPerParticle = 96);
/**
 * Delete a particle solver
 */
FLEX_API void flexDestroySolver(FlexSolver* s);

/**
 * Move particle solver forward in time
 *
 * @param[in] s A valid solver
 * @param[in] dt Time to integrate the solver forward in time by
 * @param[in] substeps The time dt will be divided into the number of sub-steps given by this parameter
 * @param[out] timers If non-NULL this struct will be filled out with profiling information for the step, note that profiling can substantially slow down overal performance so this param should only be non-NULL in non-release builds
 */
FLEX_API void flexUpdateSolver(FlexSolver* s, float dt, int substeps, FlexTimers* timers);

/**
 * Update solver paramters
 *
 * @param[in] s A valid solver
 * @param[in] params Parameters structure in host memory, see FlexParams
 */
FLEX_API void flexSetParams(FlexSolver* s, const FlexParams* params);

/**
 * Set the active particles indices in the solver
 * 
 * @param[in] s A valid solver
 * @param[in] indices Holds the indices of particles that have been made active
 * @param[in] n Number of particles to allocate
 * @param[in] source The memory space of the indices
 */
FLEX_API void flexSetActive(FlexSolver* s, const int* indices, int n, FlexMemory source);

/**
 * Return the active particle indices
 * 
 * @param[in] s A valid solver
 * @param[out] indices An array of indices at least activeCount in length
 * @param[in] target The memory space of the destination buffer
 */
FLEX_API void flexGetActive(FlexSolver* s, int* indices, FlexMemory target);

/**
 * Return the number of active particles in the solver
 * 
 * @param[in] s A valid solver
 * @return The number of active particles in the solver
 */
FLEX_API int flexGetActiveCount(FlexSolver* s);

/**
 * Set the particles state of the solver, a particle consists of 4 floating point numbers, it's x,y,z position followed by it's inverse mass (1/m)
 * 
 * @param[in] s A valid solver
 * @param[in] p Pointer to an array of particle data, should be 4*n in length
 * @param[in] n The number of particles to set
 * @param[in] source The memory space of the source buffer
 */
FLEX_API void flexSetParticles(FlexSolver* s, const float* p, int n, FlexMemory source);
/**
 * Get the particles state of the solver, a particle consists of 4 floating point numbers, it's x,y,z position followed by it's inverse mass (1/m)
 * 
 * @param[in] s A valid solver
 * @param[out] p Pointer to an array of 4*n floats that will be filled out with the particle data, can be either a host or device pointer
 * @param[in] n The number of particles to get, must be less than max particles passed to flexCreateSolver
 * @param[in] target The memory space of the destination buffer
 */
FLEX_API void flexGetParticles(FlexSolver* s, float* p, int n, FlexMemory target);

/**
 * Get the Laplacian smoothed particle positions for rendering, see FlexParams::mSmoothing
 * 
 * @param[in] s A valid solver
 * @param[out] p Pointer to an array of 4*n floats that will be filled out with the data, can be either a host or device pointer
 * @param[in] n The number of smooth particles to return
 * @param[in] target The memory space of the destination buffer
 */
FLEX_API void flexGetSmoothParticles(FlexSolver* s, float* p, int n, FlexMemory target);

/**
 * Set the particle velocities, each velocity is a 3-tuple of x,y,z floating point values
 * 
 * @param[in] s A valid solver
 * @param[in] v Pointer to an array of 3*n floats
 * @param[in] n The number of velocities to set 
 * @param[in] source The memory space of the source buffer
 */
FLEX_API void flexSetVelocities(FlexSolver* s, const float* v, int n, FlexMemory source);
/**
 * Get the particle velocities, each velocity is a 3-tuple of x,y,z floating point values
 * 
 * @param[in] s A valid solver
 * @param[out] v Pointer to an array of 3*n floats that will be filled out with the data, can be either a host or device pointer
 * @param[in] n The number of velocities to get
 * @param[in] target The memory space of the destination buffer
 */
FLEX_API void flexGetVelocities(FlexSolver* s, float* v, int n, FlexMemory target);

/**
 * Set the particles phase id array, each particle has an associated phase id which 
 * controls how it interacts with other particles. Particles with phase 0 interact with all
 * other phase types.
 *
 * Particles with a non-zero phase id only interact with particles whose phase differs 
 * from theirs. This is useful, for example, to stop particles belonging to a single
 * rigid shape from interacting with each other.
 * 
 * Phase 0 is used to indicate fluid particles when FlexParams::mFluid is set.
 * 
 * @param[in] s A valid solver
 * @param[in] phases Pointer to an array of n integers containing the phases
 * @param[in] n The number of phases to set
 * @param[in] source The memory space of the source buffer
 */
FLEX_API void flexSetPhases(FlexSolver* s, const int* phases, int n, FlexMemory source);
/**
 * Get the particle phase ids
 * 
 * @param[in] s A valid solver
 * @param[out] phases Pointer to an array of n integers that will be filled with the phase data, can be either a host or device pointer
 * @param[in] n The number of phases to get
 * @param[in] target The memory space of the destination buffer
 */
FLEX_API void flexGetPhases(FlexSolver* s, int* phases, int n, FlexMemory target);

/**
 * Set spring constraints for the solver. Each spring consists of two particle indices
 * stored consecutively, a rest-length, and a stiffness value.
 * 
 * @param[in] s A valid solver
 * @param[in] indices Pointer to the spring indices array, should be 2*numSprings length, 2 indices per-spring
 * @param[in] restLengths Pointer to an array of rest lengths, should be numSprings length
 * @param[in] stiffness Pointer to the spring stiffness coefficents, should be numSprings in length, a negative stiffness value represents a tether constraint
 * @param[in] numSprings The number of springs to set
 * @param[in] source The memory space of the source buffer
 */
FLEX_API void flexSetSprings(FlexSolver* s, const int* indices, const float* restLengths, const float* stiffness, int numSprings, FlexMemory source);
/**
 * Get the spring constraints for the solver
 * 
 * @param[in] s A valid solver
 * @param[out] indices Pointer to the spring indices array, should be 2*numSprings length, 2 indices per-spring
 * @param[out] restLengths Pointer to an array of rest lengths, should be numSprings length
 * @param[out] stiffness Pointer to the spring stiffness coefficents, should be numSprings in length, a negative stiffness value represents a unilateral tether constraint (only resists stretching, not compression), valid range [-1, 1]
 * @param[in] numSprings The number of springs to get
 * @param[in] target The memory space of the destination buffers
 */
FLEX_API void flexGetSprings(FlexSolver* s, int* indices, float* restLengths, float* stiffness, int numSprings, FlexMemory target);

/**
 * Set rigid body constraints for the solver. 
 * @note A particle should not belong to more than one rigid body at a time.
 * 
 * @param[in] s A valid solver
 * @param[in] offsets Pointer to an array of start offsets for a rigid in the indices array, should be numRigids+1 in length, the first entry must be 0
 * @param[in] indices Pointer to an array of indices for the rigid bodies, the indices for the jth rigid body start at indices[offsets[j]] and run to indices[offsets[j+1]] exclusive
 * @param[in] restPositions Pointer to an array of local space positions relative to the rigid's center of mass (average position), this should be at least 4*numIndices in length in the format x,y,z,w
 * @param[in] restNormals Pointer to an array of local space normals, this should be at least 4*numIndices in length in the format x,y,z,w where w is the (negative) signed distance of the particle inside it's shape
 * @param[in] stiffness Pointer to an array of rigid stiffness coefficents, should be numRigids in length, valid values in range [0, 1]
 * @param[in] rotations Pointer to an array of 3x3 rotation matrices (9*numRigids in length)
 * @param[in] numRigids The number of rigid bodies to set
 * @param[in] source The memory space of the source buffer
 */
FLEX_API void flexSetRigids(FlexSolver* s, const int* offsets, const int* indices, const float* restPositions, const float* restNormals, const float* stiffness, const float* rotations, int numRigids, FlexMemory source);

/**
 * Set per-particle normals to the solver, these will be overwritten after each simulation step
 * 
 * @param[in] s A valid solver
 * @param[in] normals Pointer to an array of normals, should be 4*n in length
 * @param[in] n The number of normals to set
 * @param[in] source The memory space of the source buffer
 */
FLEX_API void flexSetNormals(FlexSolver* s, const float* normals, int n, FlexMemory source);

/**
 * Get per-particle normals from the solver, these are the world-space normals computed during surface tension and rigid body calculations
 * 
 * @param[in] s A valid solver
 * @param[out] normals Pointer to an array of normals, should be 4*n in length
 * @param[in] n The number of normals to get
 * @param[in] target The memory space of the destination buffer
 */
FLEX_API void flexGetNormals(FlexSolver* s, float* normals, int n, FlexMemory target);

/**
 * Get the rotation matrices for the rigid bodies in the solver
 * 
 * @param[in] s A valid solver
 * @param[out] rotations Pointer to an array of 3x3 rotation matrices to hold the rigid rotations, should be 9*numRigids floats in length
 * @param[out] translations Pointer to an array of vectors to hold the rigid translations, should be 3*numRigids floats in length
 * @param[in] target The memory space of the destination buffer
 */
FLEX_API void flexGetRigidTransforms(FlexSolver* s, float* rotations, float* translations, FlexMemory target);

/**
 * Set the convex collision shapes for the solver, the convex data is specified
 * in structure of array (SOA) format.
 * 
 * @param[in] s A valid solver
 * @param[in] aabbMin Point to an array of lower AABB coordinates for each convex in world space, should be 4*numConvexes in length in x,y,z,* format
 * @param[in] aabbMax Pointer to an array of upper AABB coordinates for each convex in world space, should be 4*numConvexes in length in x,y,z,* format
 * @param[in] planeOffsets Pointer to an array of start offsets into the planes array for each convex, should be numConvexes in length
 * @param[in] planeCounts Pointer to an array of counts representing the number of planes belonging to each convex, should be numConvexes in length
 * @param[in] planes Pointer to an array of planes defining the convex shapes in ax + by + cz + d = 0 form, planes are specified in a local coordinate solver, should be 4*numPlanes in length
 * @param[in] positions Pointer to an array of translations for each convex in world space, should be 4*numConvexes in length
 * @param[in] rotations Pointer to an an array of rotations for each convex stored as quaternion, should be 4*numConvexes in length
 * @param[in] prevPositions Pointer to an array of translations for each convex at the start of the time step, should be 4*numConvexes in length
 * @param[in] prevRotations Pointer to an an array of rotations for each convex stored as a quaternion at the start of the time step, should be 4*numConvexes in length
 * @param[in] flags Whether the convex is considered static (0), or dynamic (1), collisions with static shapes are prioritized over dynamic ones
 * @param[in] numConvexes The number of convexes
 * @param[in] numPlanes The total number of planes for all convexes
 * @param[in] source The memory space of the source buffer
 */
FLEX_API void flexSetConvexes (FlexSolver* s, const float* aabbMin, const float* aabbMax, const int* planeOffsets, const int* planeCounts, const float* planes, const float* positions, const float* rotations, const float* prevPositions, const float* prevRotations, const int* flags, int numConvexes, int numPlanes, FlexMemory source);
/**
 * Set the triangle mesh collision data for the solver, triangles are treated as two-sided and collided using a continuous collison detection method to prevent tunnelling
 * 
 * @param[in] s A valid solver
 * @param[in] indices Pointer to an array of triangle vertex indices, should be 3*numTris in length
 * @param[in] vertices Pointer to an array of vertex positions in world space, should be 3*numPositions in length
 * @param[in] numTris The number of triangles
 * @param[in] numVertices The number of mesh vertices
 * @param[in] cellSize The size of grid cell used for broad phase collision culling, should be set relative to particle radius, e.g. 2*radius
 * @param[in] source The memory space of the source buffer
 */
FLEX_API void flexSetTriangles(FlexSolver* s, const int* indices, const float* vertices, int numTris, int numVertices, float cellSize, FlexMemory source);	
/**
 * Set the signed distance field collision shapes, see FlexSDF.
 * 
 * @param[in] s A valid solver
 * @param[in] shapes Pointer to an array of signed distance field shapes
 * @param[in] numShapes The number of shapes 
 */
FLEX_API void flexSetFields(FlexSolver* s, const FlexSDF* shapes, int numShapes);

/**
 * Set dynamic triangles mesh indices, typically used for cloth. Flex will calculate normals and 
 * apply wind and drag effects to connected particles. See FlexParams::mDrag, FlexParams::mWind.
 * 
 * @param[in] s A valid solver
 * @param[in] indices Pointer to an array of triangle indices into the particles array, should be 3*numTris in length
 * @param[in] normals Pointer to an array of triangle normals, should be 3*numTris in length, can be NULL
 * @param[in] numTris The number of dynamic triangles
 * @param[in] source The memory space of the source buffers
 */
FLEX_API void flexSetDynamicTriangles(FlexSolver* s, const int* indices, const float* normals, int numTris, FlexMemory source);
/**
 * Get the dynamic triangle indices and normals.
  * 
 * @param[in] s A valid solver
 * @param[out] indices Pointer to an array of triangle indices into the particles array, should be 3*numTris in length, if NULL indices will not be returned
 * @param[out] normals Pointer to an array of triangle normals, should be 3*numTris in length, if NULL normals will be not be returned
 * @param[in] numTris The number of dynamic triangles
 * @param[in] target The memory space of the destination arrays
 */
FLEX_API void flexGetDynamicTriangles(FlexSolver* s, int* indices, float* normals, int numTris, FlexMemory target);

/**
 * Set inflatable shapes, an inflatable is a range of dynamic triangles that represent a closed mesh.
 * Each inflatable has a given rest volume, constraint scale (roughly equivalent to stiffness), and "over pressure"
 * that controls how much the shape is inflated.
 * 
 * @param[in] s A valid solver
 * @param[in] startTris Pointer to an array of offsets into the solver's dynamic triangles for each inflatable, should be numInflatables in length
 * @param[in] numTris Pointer to an array of triangle counts for each inflatable, should be numInflatablesin length
 * @param[in] restVolumes Pointer to an array of rest volumes for the inflatables, should be numInflatables in length
 * @param[in] overPressures Pointer to an array of floats specifying the pressures for each inflatable, a value of 1.0 means the rest volume, > 1.0 means over-inflated, and < 1.0 means under-inflated, should be numInflatables in length
 * @param[in] constraintScales Pointer to an array of scaling factors for the constraint, this is roughly equivalent to stiffness but includes a constraint scaling factor from position-based dynamics, see helper code for details, should be numInflatables in length
 * @param[in] numInflatables Number of inflatables to set
 * @param[in] source The memory space of the source buffers
 */
FLEX_API void flexSetInflatables(FlexSolver* s, const int* startTris, const int* numTris, float* restVolumes, float* overPressures, float* constraintScales, int numInflatables, FlexMemory source);

/**
 * Get the density values for fluid particles
 * 
 * @param[in] s A valid solver
 * @param[out] densities Pointer to an array of floats, should be maxParticles in length, density values are normalized between [0, 1] where 1 represents the rest density
 * @param[in] target The memory space of the destination arrays
 */
FLEX_API void flexGetDensities(FlexSolver* s, float* densities, FlexMemory target);
/**
 * Get the anisotropy of fluid particles, the particle distribution for a particle is represented
 * by 3 orthogonal vectors. Each 3-vector has unit length with the variance along that axis
 * packed into the w component, i.e.: x,y,z,lambda.
 *
 * The anisotropy defines an oriented ellipsoid in worldspace that can be used for rendering
 * or surface extraction.
 * 
 * @param[in] s A valid solver
 * @param[out] q1 Pointer to an array of floats that receive the first basis vector and scale, should be 4*maxParticles in length
 * @param[out] q2 Pointer to an array of floats that receive the second basis vector and scale, should be 4*maxParticles in length
 * @param[out] q3 Pointer to an array of floats that receive the third basis vector and scale, should be 4*maxParticles in length
 * @param[in] target The memory space of the destination arrays
 */
FLEX_API void flexGetAnisotropy(FlexSolver* s, float* q1, float* q2, float* q3, FlexMemory target);
/**
 * Get the state of the diffuse particles. Diffuse particles are passively advected by the fluid
 * velocity field.
 * 
 * @param[in] s A valid solver
 * @param[out] p Pointer to an array of floats, should be 4*maxParticles in length, the w component represents the particles lifetime with 1 representing a new particle, and 0 representing an inactive particle
 * @param[out] v Pointer to an array of floats, should be 4*maxParticles in length, the w component is not used
 * @param[out] indices Pointer to an array of ints that specify particle indices in depth sorted order, should be maxParticles in length, see FlexParams::mDiffuseSortDir
 * @param[in] target The memory space of the destination arrays
 */
FLEX_API int flexGetDiffuseParticles(FlexSolver* s, float* p, float* v, int* indices, FlexMemory target);	
/**
 * Set the state of the diffuse particles. Diffuse particles are passively advected by the fluid
 * velocity field.
 * 
 * @param[in] s A valid solver
 * @param[in] p Pointer to an array of floats, should be 4*n in length, the w component represents the particles lifetime with 1 representing a new particle, and 0 representing an inactive particle
 * @param[in] v Pointer to an array of floats, should be 4*n in length, the w component is not used
 * @param[in] n Number of diffuse particles to set
 * @param[in] source The memory space of the source buffer
 */
FLEX_API void flexSetDiffuseParticles(FlexSolver* s, const float* p, const float* v, int n, FlexMemory source);

/**
 * Get the particle contact planes. Note this will only include contacts that were active on the last substep of an update, and will include all contact planes generated within FlexParam::mShapeCollisionMargin.
 * 
 * @param[in] s A valid solver
 * @param[out] planes Pointer to a destination buffer containing the contact planes for the particle, each particle can have up to 4 contact planes so this buffer should be 16*maxParticles in length
 * @param[out] indices Pointer to an array of indices into the contacts buffer, the contact planes for the i'th particle are given by planes[indices[i]], should be maxParticles in length
 * @param[out] counts Pointer to an array of contact counts for each particle (will be < 4), this buffer should be maxParticles in length
 * @param[in] target The memory space of the target buffers
 */
FLEX_API void flexGetContacts(FlexSolver* s, float* planes, int* indices, unsigned char* counts, FlexMemory target);

/**
 * Get the world space AABB of all particles in the solver.
 * 
 * @param[in] s A valid solver
 * @param[out] lower Pointer to an array of 3 floats to receive the lower bounds
 * @param[out] upper Pointer to an array of 3 floats to receive the upper bounds
 */
FLEX_API void flexGetBounds(FlexSolver* s, float* lower, float* upper);

/**
 * Allocates size bytes of memory from the optimal memory pool. Using this function
 * is optional, but when passed to flexGet/flexSet methods it may result
 * in significantly faster transfers, memory used with async transfers should
 * be allocated by this method to ensure optimal performance. For CUDA implementations
 * this method will return pinned host memory from cudaMallocHost().
 *
 @param[in] size The number of bytes to alloc
 @return pointer to the allocated memory
 */
FLEX_API void* flexAlloc(int size);

/** 
 * Free memory allocated through flexAlloc
 *
 * @param[in] ptr Pointer returned from flexAlloc
 */
FLEX_API void flexFree(void* ptr);

/** 
 * Sets a fence that can be used to synchronize the calling thread 
 * with any outstanding GPU work, typically used with async transfers
 * to ensure any flexGet/flexSet calls have completed. 
 *
   \code{.c}
		// update solver
		flexUpdateSolver(solver, dt, iterations, NULL);

		// read back state
		flexGetParticles(solver, &particles, n, eFlexMemoryHostAsync);
		flexGetVelocities(solver, &velocities, n, eFlexMemoryHostAsync);
		flexGetDensities(solver, &densities, n, eFlexMemoryHostAsync);

		// insert fence
		flexSetFence();

		// perform asynchronous CPU work
		
		// wait for queued work to finish
		flexWaitFence();

  \endcode
 */
FLEX_API void flexSetFence();

/** 
 * Wait's for the work scheduled before the last call to flexSetFence() to complete
 * If flexSetFence() has not yet been called then this is function returns immediately
 *
 */
FLEX_API void flexWaitFence();

//! \cond HIDDEN_SYMBOLS

/**
 * Debug methods (unsupported)
 */
FLEX_API void flexSetDebug(FlexSolver* s, bool enable);
FLEX_API void flexGetConvexGrid(FlexSolver* s, int* grid, float* lower, float* upper, int* axis);
FLEX_API void flexGetStaticTriangleGrid(FlexSolver* s, int* counts, float* lower, float* cellEdge);
FLEX_API void flexStartRecord(FlexSolver* s, const char* file);
FLEX_API void flexStopRecord(FlexSolver* s);

//! \endcond

} // extern "C"

#endif // FLEX_H
