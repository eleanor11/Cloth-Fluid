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
#include <core/types.h>
#include <core/maths.h>
#include <core/platform.h>
#include <core/maths.h>
#include <core/shader.h>
#include <core/mesh.h>
#include <core/voxelize.h>
#include <core/sdf.h>
#include <core/pfm.h>
#include <core/tga.h>
#include <core/perlin.h>

#include <iostream>
#include <map>

#include "include/flex.h"
#include "shaders.h"
#include "convex.h"
#include "cloth.h"

#include "imgui.h"
#include "imguiRenderGL.h"

using namespace std;

int g_screenWidth = 1280;
int g_screenHeight = 720;
bool g_msaa = true;
GLuint g_msaaFbo;
GLuint g_msaaColorBuf;
GLuint g_msaaDepthBuf;

int g_numSubsteps;
int g_cudaDevice;

FluidRenderer* g_renderer;
FluidRenderBuffers g_renderBuffers;
FlexSolver* g_flex;
FlexParams g_params;
FlexSDF g_shape;

int g_maxDiffuseParticles;
unsigned char g_maxNeighborsPerParticle;
int g_numExtraParticles;
int g_numExtraMultiplier = 1;

Mesh* g_mesh;
vector<int> g_meshSkinIndices;
vector<float> g_meshSkinWeights;
vector<Point3> g_meshRestPositions;
const int g_numSkinWeights = 4;

Mesh* g_staticMesh;
Mesh* g_skyMesh;

vector<Vec4> g_positions;
vector<Vec4> g_restPositions;
vector<Vec3> g_velocities;
vector<float> g_densities;
vector<Vec4> g_anisotropy1;
vector<Vec4> g_anisotropy2;
vector<Vec4> g_anisotropy3;
vector<Vec4> g_normals;
vector<Vec4> g_diffusePositions;
vector<int> g_phases;
vector<int> g_activeIndices;

/*added*/

float g_clothRadius;

float g_kAbsorption;
vector<bool> g_absorbable;

Vec4 g_clothColor;
vector<Vec4> g_colors;

float g_kDiffusion;
float g_kDiffusionGravity;
float g_maxSaturation;
vector<float> g_saturations;
vector<Vec3> g_triangleCenters;
vector<Vec3> g_thetas;

float g_mDrip;
vector<float> g_dripBuffer;

bool g_absorb;
bool g_diffuse;
bool g_drip;

/*added end*/


Vec3 g_camPos(6.0f, 8.0f, 18.0f);
Vec3 g_camAngle(0.0f, -DegToRad(15.0f), 0.0f);
Vec3 g_camVel(0.0f);
Vec3 g_camSmoothVel(0.0f);
float g_camSpeed;
float g_camNear;
float g_camFar;

Vec3 g_lightPos;
Vec3 g_lightDir;
Vec3 g_lightTarget;

bool g_pause = true;
bool g_step = false;
bool g_capture = false;
bool g_showHelp = true;
bool g_fullscreen = false;
bool g_wireframe = false;
bool g_debug = false;

bool g_recording = false;
bool g_emit = false;
bool g_warmup = false;

float g_windTime = 0.0f;
float g_windFrequency = 0.1f;
float g_windStrength = 0.0f;

bool g_wavePool = false;
float g_waveTime = 0.0f;
float g_wavePlane;
float g_waveFrequency = 1.5f;
float g_waveAmplitude = 1.0f;
float g_waveFloorTilt = 0.0f;

Vec3 g_sceneLower;
Vec3 g_sceneUpper;

float g_blur;
float g_ior;
bool g_drawEllipsoids;
bool g_drawPoints;
bool g_drawMesh;
bool g_drawCloth;
float g_expandCloth;	// amount to expand cloth along normal (to account for particle radius)

bool g_drawOpaque;
int g_drawSprings;		// 0: no draw, 1: draw stretch 2: draw tether
bool g_drawNormals = false;
bool g_drawDiffuse;
bool g_drawConvexGrid = false;
bool g_drawDensity = false;
bool g_drawRopes;
bool g_drawReflection = true;
float g_pointScale;
float g_ropeScale;
float g_drawPlaneBias;	// move planes along their normal for rendering

extern float gShadowBias;

float g_diffuseScale;
float g_diffuseMotionScale;
bool g_diffuseShadow;
float g_diffuseInscatter;
float g_diffuseOutscatter;

float g_dt = 1.0f/60.0f;	// the time delta used for simulation
float g_realdt;				// the real world time delta 
int g_scene = 0;
int g_selectedScene = g_scene;
int g_levelScroll;			// offset for level selection scroll area

int g_frame = 0;
int g_numSolidParticles = 0;

int g_mouseParticle = -1;
float g_mouseT = 0.0f;
Vec3 g_mousePos;

// mouse
static int lastx;
static int lasty;
static int lastb = -1;

char g_device[256];

bool g_profile = false;

// shadow buffers
GLuint g_shadowTex; 
GLuint g_shadowBuf; 

// reflect buffers
GLuint g_reflectTex;

Vec4 g_fluidColor;
Vec4 g_diffuseColor;
Vec3 g_meshColor;
Vec3  g_clearColor;
float g_lightDistance;
float g_fogDistance;

FILE* g_ffmpeg;

void DrawConvexes();

// convexes
vector<Vec4> g_convexPositions;
vector<Vec4> g_convexRotations;
vector<Vec4> g_convexPrevPositions;
vector<Vec4> g_convexPrevRotations;
vector<Vec4> g_convexPlanes;
vector<uint32_t> g_convexStarts;
vector<uint32_t> g_convexLengths;
vector<Vec4> g_convexAabbMin;
vector<Vec4> g_convexAabbMax;
vector<int> g_convexFlags;

// rigids
vector<int> g_rigidOffsets;
vector<int> g_rigidIndices;
vector<int> g_rigidMeshSize;
vector<float> g_rigidCoefficients;
vector<Matrix33> g_rigidRotations;
vector<Vec3> g_rigidTranslations;
vector<Vec3> g_rigidLocalPositions;
vector<Vec4> g_rigidLocalNormals;

// springs
vector<int> g_springIndices;
vector<float> g_springLengths;
vector<float> g_springStiffness;

vector<int> g_triangles;
vector<Vec3> g_triangleNormals;
vector<Vec3> g_uvs;

class Scene;
vector<Scene*> g_scenes;

struct Emitter
{
	Emitter() : mSpeed(0.0f), mEnabled(false), mLeftOver(0.0f), mWidth(8)   {}

	Vec3 mPos;
	Vec3 mDir;
	Vec3 mRight;
	float mSpeed;
	bool mEnabled;
	float mLeftOver;
	int mWidth;
};

vector<Emitter> g_emitters(1);	// first emitter is the camera 'gun'

struct Rope
{
	std::vector<int> mIndices;
};

vector<Rope> g_ropes;

inline float sqr(float x) { return x*x; }

#define cudaCheck(x) { cudaError_t err = x; if (err != cudaSuccess) { printf("Cuda error: %d in %s at %s:%d\n", err, #x, __FILE__, __LINE__); assert(0); } }


#include "helpers.h"
#include "scenes.h"

void Init(int scene, bool centerCamera=true)
{	
	RandInit();
	
	g_frame = 0;

	if (g_flex)
	{
		DestroyFluidRenderBuffers(g_renderBuffers);

		flexDestroySolver(g_flex);
		g_flex = NULL;
	}

	g_pause = false;

	g_positions.resize(0);
	g_velocities.resize(0);
	g_phases.resize(0);

	g_rigidOffsets.resize(0);
	g_rigidIndices.resize(0);
	g_rigidMeshSize.resize(0);
	g_rigidRotations.resize(0);
	g_rigidTranslations.resize(0);
	g_rigidCoefficients.resize(0);
	g_rigidLocalPositions.resize(0);
	g_rigidLocalNormals.resize(0);

	g_springIndices.resize(0);
	g_springLengths.resize(0);
	g_springStiffness.resize(0);
	g_triangles.resize(0);
	g_triangleNormals.resize(0);
	g_uvs.resize(0);

	g_meshSkinIndices.resize(0);
	g_meshSkinWeights.resize(0);

	g_emitters.resize(1);
	g_emitters[0].mEnabled = false;
	g_emitters[0].mSpeed = 1.0f;

	g_convexPositions.resize(0);
	g_convexRotations.resize(0);
	g_convexPrevPositions.resize(0);
	g_convexPrevRotations.resize(0);
	g_convexPlanes.resize(0);
	g_convexStarts.resize(0);
	g_convexLengths.resize(0);
	g_convexAabbMin.resize(0);
	g_convexAabbMax.resize(0);
	g_convexFlags.resize(0);

	g_ropes.resize(0);


	/*added initiate begin*/
	g_kAbsorption = 0.0;
	g_kDiffusion = 0.0;
	g_kDiffusionGravity = 0.0;

	/*set origin cloth color*/
	g_clothRadius = 0.1* 0.25f;
	g_clothColor = Vec4(0.3f, 1.0f, 1.0f, 1.0f);
	g_colors.resize(0);

	g_maxSaturation = 3.0;
	g_absorbable.resize(0);
	g_saturations.resize(0);
	g_triangleCenters.resize(0);
	g_thetas.resize(0);

	g_mDrip = 0.5;
	g_dripBuffer.resize(0);


	g_absorb = false;
	g_diffuse = false;
	g_drip = false;

	/*added initiate end*/



	// remove collision shapes
	delete g_mesh; g_mesh = NULL;
	delete g_shape.mField; g_shape.mField = NULL;
	delete g_staticMesh; g_staticMesh = NULL;

	g_dt = 1.0f/60.0f;
	g_waveTime = 0.0f;
	g_windTime = 0.0f;
	g_windStrength = 1.0f;

	g_blur = 1.0f;
	g_fluidColor = Vec4(0.1f, 0.4f, 0.8f, 1.0f);
	g_meshColor = Vec3(0.9f, 0.9f, 0.9f);
	g_drawEllipsoids = false;
	g_drawPoints = true;
	g_drawCloth = true;
	g_expandCloth = 0.0f;

	g_drawOpaque = false;
	g_drawSprings = false;
	g_drawDiffuse = false;
	g_drawMesh = true;
	g_drawRopes = true;
	g_drawDensity = false;
	g_ior = 1.0f;
	g_lightDistance = 2.0f;
	g_fogDistance = 0.005f;
	g_camSpeed = 0.075f;
	g_camNear = 0.01f;
	g_camFar = 1000.0f;

	// hack, set this into the renderer	
	gShadowBias = 0.05f;

	g_pointScale = 1.0f;
	g_ropeScale = 1.0f;
	g_drawPlaneBias = 0.0f;
	
	// sim params
	g_params.mGravity[0] = 0.0f;
	g_params.mGravity[1] = -9.8f;
	g_params.mGravity[2] = 0.0f;

	g_params.mWind[0] = 0.0f;
	g_params.mWind[1] = 0.0f;
	g_params.mWind[2] = 0.0f;

	g_params.mRadius = 0.15f;
	g_params.mViscosity = 0.0f;
	g_params.mDynamicFriction = 0.0f;
	g_params.mStaticFriction = 0.0f;
	g_params.mParticleFriction = 0.0f; // scale friction between particles by default
	g_params.mFreeSurfaceDrag = 0.0f;
	g_params.mDrag = 0.0f;
	g_params.mLift = 0.0f;
	g_params.mNumIterations = 3;
	g_params.mFluidRestDistance = 0.0f;
	g_params.mSolidRestDistance = 0.0f;
	g_params.mAnisotropyScale = 1.0f;
	g_params.mDissipation = 0.0f;
	g_params.mDamping = 0.0f;
	g_params.mParticleCollisionMargin = 0.0f;
	g_params.mShapeCollisionMargin = 0.0f;
	g_params.mCollisionDistance = 0.0f;
	g_params.mPlasticThreshold = 0.0f;
	g_params.mPlasticCreep = 0.0f;
	g_params.mFluid = false;
	g_params.mSleepThreshold = 0.0f;
	g_params.mShockPropagation = 0.0f;
	g_params.mRestitution = 0.0f;
	g_params.mSmoothing = 1.0f;
	g_params.mMaxVelocity = FLT_MAX;
	g_params.mRelaxationMode = eFlexRelaxationLocal;
	g_params.mRelaxationFactor = 1.0f;
	g_params.mSolidPressure = 1.0f;
	g_params.mAdhesion = 0.0f;
	g_params.mCohesion = 0.025f;
	g_params.mSurfaceTension = 0.0f;
	g_params.mVorticityConfinement = 0.0f;
	g_params.mBuoyancy = 1.0f;
	g_params.mDiffuseThreshold = 100.0f;
	g_params.mDiffuseBuoyancy = 1.0f;
	g_params.mDiffuseDrag = 0.8f;
	g_params.mDiffuseBallistic = 16;
	g_params.mDiffuseSortAxis[0] = 0.0f;
	g_params.mDiffuseSortAxis[1] = 0.0f;
	g_params.mDiffuseSortAxis[2] = 0.0f;
	g_params.mEnableCCD = false;

	g_numSubsteps = 2;

	// planes created after particles
	g_params.mNumPlanes = 1;

	g_diffuseScale = 0.5f;
	g_diffuseColor = 1.0f;
	g_diffuseMotionScale = 1.0f;
	g_diffuseShadow = false;
	g_diffuseInscatter = 0.8f;
	g_diffuseOutscatter = 0.53f;

	// reset phase 0 particle color to blue
	extern Colour gColors[];
	gColors[0] = Colour(0.0f, 0.5f, 1.0f);

	g_numSolidParticles = 0;

	g_waveFrequency = 1.5f;
	g_waveAmplitude = 1.5f;
	g_waveFloorTilt = 0.0f;
	g_emit = false;
	g_warmup = false;

	g_mouseParticle = -1;
	
	g_maxDiffuseParticles = 0;	// number of diffuse particles
	g_maxNeighborsPerParticle = 96;
	g_numExtraParticles = 0;	// number of particles allocated but not made active	

	g_sceneLower = FLT_MAX;
	g_sceneUpper = -FLT_MAX;

	// create scene
	g_scenes[g_scene]->Initialize();

	uint32_t numParticles = g_positions.size();
	uint32_t maxParticles = numParticles + g_numExtraParticles*g_numExtraMultiplier;

	/*added*/

	g_colors.resize(numParticles);
	g_absorbable.resize(maxParticles);

	/*added end*/


	// by default solid particles use the maximum radius
	if (g_params.mFluid && g_params.mSolidRestDistance == 0.0f)
		g_params.mSolidRestDistance = g_params.mFluidRestDistance;
	else
		g_params.mSolidRestDistance = g_params.mRadius;

	// collision distance with shapes half the radius
	if (g_params.mCollisionDistance == 0.0f)
	{
		g_params.mCollisionDistance = g_params.mRadius*0.5f;

		if (g_params.mFluid)
			g_params.mCollisionDistance = g_params.mFluidRestDistance*0.5f;
	}

	// default particle friction to 10% of shape friction
	if (g_params.mParticleFriction == 0.0f)
		g_params.mParticleFriction = g_params.mDynamicFriction*0.1f; 

	// add a margin for detecting contacts between particles and shapes
	if (g_params.mShapeCollisionMargin == 0.0f)
		g_params.mShapeCollisionMargin = g_params.mCollisionDistance*0.25f;
		
	// calculate particle bounds
	Vec3 lower, upper;
	GetParticleBounds(lower, upper);

	// expand
	lower -= Vec3(g_params.mCollisionDistance);
	upper += Vec3(g_params.mCollisionDistance);

	// accommodate mesh
	if (g_mesh)
	{
		Vec3 meshLower, meshUpper;
		g_mesh->GetBounds(meshLower, meshUpper);

		lower = Min(lower, meshLower);
		upper = Max(upper, meshUpper);
	}

	// acommodate convexes
	for (size_t i=0; i < g_convexAabbMin.size(); ++i)
	{
		lower = Min(lower, Vec3(g_convexAabbMin[i]));
		upper = Max(upper, Vec3(g_convexAabbMax[i]));
	}

	g_sceneLower = Min(g_sceneLower, lower);
	g_sceneUpper = Max(g_sceneUpper, upper);

	// update collision planes to match flexs
	//Vec3 up = Normalize(Vec3(-0.05f, 1.0f, 0.0f));
	Vec3 up = Normalize(Vec3(-g_waveFloorTilt, 1.0f, 0.0f));
	(Vec4&)g_params.mPlanes[0] = Vec4(up.x, up.y, up.z, 0.0f);
	(Vec4&)g_params.mPlanes[1] = Vec4(0.0f, 0.0f, 1.0f, -g_sceneLower.z);
	(Vec4&)g_params.mPlanes[2] = Vec4(1.0f, 0.0f, 0.0f, -g_sceneLower.x);
	(Vec4&)g_params.mPlanes[3] = Vec4(-1.0f, 0.0f, 0.0f, g_sceneUpper.x);
	(Vec4&)g_params.mPlanes[4] = Vec4(0.0f, 0.0f, -1.0f, g_sceneUpper.z);
	(Vec4&)g_params.mPlanes[5] = Vec4(0.0f, -1.0f, 0.0f, g_sceneUpper.y);
	
	g_wavePlane = g_params.mPlanes[2][3];

	g_diffusePositions.resize(g_maxDiffuseParticles);
	
	g_normals.resize(0);
	g_normals.resize(maxParticles);

	// initialize normals (just for rendering before simulation starts)
	int numTris = g_triangles.size()/3;
	for (int i=0; i < numTris; ++i)
	{
		Vec3 v0 = Vec3(g_positions[g_triangles[i*3+0]]);
		Vec3 v1 = Vec3(g_positions[g_triangles[i*3+1]]);
		Vec3 v2 = Vec3(g_positions[g_triangles[i*3+2]]);

		Vec3 n = Cross(v1-v0, v2-v0);

		g_normals[g_triangles[i*3+0]] += Vec4(n, 0.0f);
		g_normals[g_triangles[i*3+1]] += Vec4(n, 0.0f);
		g_normals[g_triangles[i*3+2]] += Vec4(n, 0.0f);
	}

	for (int i=0; i < int(maxParticles); ++i)
		g_normals[i] = Vec4(SafeNormalize(Vec3(g_normals[i]), Vec3(0.0f, 1.0f, 0.0f)), 0.0f);

	g_flex = flexCreateSolver(maxParticles, g_maxDiffuseParticles, g_maxNeighborsPerParticle); 
		
	flexSetParams(g_flex, &g_params);
	flexSetParticles(g_flex, (float*)&g_positions[0], numParticles, eFlexMemoryHost);
	flexSetVelocities(g_flex, (float*)&g_velocities[0], numParticles, eFlexMemoryHost);
	flexSetNormals(g_flex, (float*)&g_normals[0], numParticles, eFlexMemoryHost);
	
	g_activeIndices.resize(maxParticles);
	for (size_t i=0; i < g_activeIndices.size(); ++i)
		g_activeIndices[i] = i;

	flexSetActive(g_flex, &g_activeIndices[0], numParticles, eFlexMemoryHost);

	if (g_shape.mField)
		flexSetFields(g_flex, &g_shape, 1);

	g_positions.resize(maxParticles);
	g_velocities.resize(maxParticles);
	g_phases.resize(maxParticles);

	g_densities.resize(maxParticles);
	g_anisotropy1.resize(maxParticles);
	g_anisotropy2.resize(maxParticles);
	g_anisotropy3.resize(maxParticles);
	
	// center camera on particles
	if (centerCamera)
	{
		g_camPos = Vec3((g_sceneLower.x+g_sceneUpper.x)*0.5f, min(g_sceneUpper.y*1.25f, 7.0f), g_sceneUpper.z + min(g_sceneUpper.y, 8.0f)*2.0f);
		g_camAngle = Vec3(0.0f, -DegToRad(10.0f), 0.0f);
	}
	
	// convexes
	if (g_convexPositions.size())
	{
		flexSetConvexes(
			g_flex, 
			(float*)&g_convexAabbMin[0], 
			(float*)&g_convexAabbMax[0], 
			(int*)&g_convexStarts[0], (int*)&g_convexLengths[0], 
			(float*)&g_convexPlanes[0], 
			(float*)&g_convexPositions[0], 
			(float*)&g_convexRotations[0], 
			(float*)&g_convexPrevPositions[0],
			(float*)&g_convexPrevRotations[0],
			&g_convexFlags[0],
			g_convexStarts.size(),
			g_convexPlanes.size(), eFlexMemoryHost);
	}

	// rigids
	if (g_rigidOffsets.size())
	{
		assert(g_rigidOffsets.size() > 1);

		const int numRigids = g_rigidOffsets.size()-1;

		// calculate local rest space positions
		g_rigidLocalPositions.resize(g_rigidOffsets.back());
		CalculateRigidOffsets(&g_positions[0], &g_rigidOffsets[0], &g_rigidIndices[0], numRigids, &g_rigidLocalPositions[0]);

		g_rigidRotations.resize(g_rigidOffsets.size()-1, Matrix33::Identity());
		g_rigidTranslations.resize(g_rigidOffsets.size()-1, Vec3());

		flexSetRigids(g_flex, &g_rigidOffsets[0], &g_rigidIndices[0], (float*)&g_rigidLocalPositions[0], g_rigidLocalNormals.size()?(float*)&g_rigidLocalNormals[0]:NULL, &g_rigidCoefficients[0], (float*)&g_rigidRotations[0], numRigids, eFlexMemoryHost);		
	}

	// springs
	if (g_springIndices.size())
	{
		assert((g_springIndices.size()&1) == 0);
		assert((g_springIndices.size()/2) == g_springLengths.size());
		flexSetSprings(g_flex, &g_springIndices[0], &g_springLengths[0], &g_springStiffness[0], g_springLengths.size(), eFlexMemoryHost);
	}

	if (g_triangles.size())
	{
		flexSetDynamicTriangles(g_flex, &g_triangles[0], &g_triangleNormals[0].x, g_triangles.size()/3, eFlexMemoryHost);
	}

	// save mesh positions for skinning
	if (g_mesh)
	{
		g_meshRestPositions = g_mesh->m_positions;
	}
	else
	{
		g_meshRestPositions.resize(0);
	}

	if (g_staticMesh)
	{
		flexSetTriangles(g_flex, (int*)&g_staticMesh->m_indices[0], (float*)&g_staticMesh->m_positions[0], g_staticMesh->GetNumFaces(), g_staticMesh->GetNumVertices(), g_params.mRadius*4.0f, eFlexMemoryHost);
	}


	flexSetPhases(g_flex, &g_phases[0], g_phases.size(), eFlexMemoryHost);

	g_scenes[g_scene]->PostInitialize();


	// create render data
	g_renderBuffers = CreateFluidRenderBuffers(maxParticles + 1, g_maxDiffuseParticles);

	g_restPositions = g_positions;

	if (g_warmup)
	{
		printf("Warming up sim..\n");

		// warm it up (relax positions to reach rest density without affecting velocity)
		FlexParams copy = g_params;
		copy.mNumIterations = 4;	

		flexSetParams(g_flex, &copy);

		const int kWarmupIterations = 100;

		for (int i=0; i < kWarmupIterations; ++i)
		{
			FlexTimers timers;
			flexUpdateSolver(g_flex, 0.0001f, 1, &timers);
			flexSetVelocities(g_flex, (float*)&g_velocities[0], maxParticles, eFlexMemoryHost);
		}

		// udpate host copy
		flexGetParticles(g_flex, &g_positions[0].x, g_positions.size(), eFlexMemoryHost);

		printf("Finished warm up.\n");
	}
}

void Reset()
{
	Init(g_scene, false);
}

void GLUTUpdate()
{
	static FlexTimers timers;
	static double lastTime;

	// real elapsed frame time
	double currentTime = GetSeconds();
	
	// simple low-pass filter to reduce noise in the FPS counter
	g_realdt = float(currentTime-lastTime)*0.8f + g_realdt*0.2f;
	lastTime = currentTime;
	
	

	if (!g_pause || g_step)	
	{	
		float spin = DegToRad(15.0f);

		const Vec3 forward(-sinf(g_camAngle.x+spin)*cosf(g_camAngle.y), sinf(g_camAngle.y), -cosf(g_camAngle.x+spin)*cosf(g_camAngle.y));
		const Vec3 right(Normalize(Cross(forward, Vec3(0.0f, 1.0f, 0.0f))));


		/*added*/
		/*  do absorb   */
		if (g_absorb){

			Absorbing();

			//g_saturations[0] = 1.0f;
			//g_saturations[1] = 1.0f;

			//g_saturations[962] = 1.0f;
			//g_saturations[963] = 1.0f;
			//g_saturations[399] = 1.0f;
			//g_saturations[295] = 1.0f;
		}

		/*	do diffuse	*/

		if (g_diffuse){

			CalculateTriangleCenters();
			CalculateThetas();

			g_kDiffusion = 0.3;
			g_kDiffusionGravity = 0.2;
			DiffuseCloth();
		}

		/*  do drip    */
		if (g_drip){
			Dripping();
		}

		/*added end*/


		// process emitters
		if (g_emit)
		{			
			int activeCount = flexGetActiveCount(g_flex);

			g_emitters[0].mDir = Normalize(forward+Vec3(0.0, 0.4f, 0.0f));
			g_emitters[0].mRight = right;
			g_emitters[0].mPos = g_camPos + forward*1.f + Vec3(0.0f, 0.2f, 0.0f) + right*0.65f;

			size_t e=0;

			 // skip camera emitter when moving forward or things get messy
			if (g_camSmoothVel.z >= 0.025f)
				e = 1;

			for (; e < g_emitters.size(); ++e)
			{
				if (!g_emitters[e].mEnabled)
					continue;

				Vec3 emitterDir = g_emitters[e].mDir;
				Vec3 emitterRight = g_emitters[e].mRight;
				Vec3 emitterPos = g_emitters[e].mPos;

				float r;
				int phase;

				if (g_params.mFluid)
				{
					r = g_params.mFluidRestDistance;
					phase = flexMakePhase(0, eFlexPhaseSelfCollide | eFlexPhaseFluid);
				}
				else
				{
					r = g_params.mSolidRestDistance;
					phase = flexMakePhase(0, eFlexPhaseSelfCollide);
				}

				float numParticles = (g_emitters[e].mSpeed / r)*g_dt;

				// whole number to emit
				int n = int(numParticles + g_emitters[e].mLeftOver);
				
				if (n)
					g_emitters[e].mLeftOver = (numParticles + g_emitters[e].mLeftOver)-n;
				else
					g_emitters[e].mLeftOver += numParticles;

				// create a grid of particles (n particles thick)
				for (int k = 0; k < n; ++k)
				{
					int emitterWidth = g_emitters[e].mWidth;
					emitterWidth = 3;
					int numParticles = emitterWidth*emitterWidth;
					for (int i=0; i < numParticles; ++i)
					{
						float x = float(i%emitterWidth) - emitterWidth/2;
						float y = float((i/emitterWidth)%emitterWidth) - emitterWidth/2;

						if ((sqr(x) + sqr(y)) <= (emitterWidth/2)*(emitterWidth/2))
						{
							Vec3 up = Normalize(Cross(emitterDir, emitterRight));
							Vec3 offset = r*(emitterRight*x + up*y) + float(k)*emitterDir*r;

							if (size_t(activeCount) < g_positions.size())
							{
								g_positions[activeCount] = Vec4(emitterPos + offset, 1.0f);
								g_velocities[activeCount] = emitterDir*g_emitters[e].mSpeed;
								g_phases[activeCount] = phase;
								g_absorbable[activeCount] = true;

								activeCount++;
							}
						}
					}
				}
			}

			flexSetActive(g_flex, &g_activeIndices[0], activeCount, eFlexMemoryHost);
		}

		// mouse picking
		if (g_mouseParticle != -1)
		{
			Vec3 p = Lerp(Vec3(g_positions[g_mouseParticle]), g_mousePos, 0.8f);
			Vec3 delta = p - Vec3(g_positions[g_mouseParticle]);
			
			
			g_positions[g_mouseParticle].x = p.x;
			g_positions[g_mouseParticle].y = p.y;
			g_positions[g_mouseParticle].z = p.z;

			g_velocities[g_mouseParticle].x = delta.x/g_dt;
			g_velocities[g_mouseParticle].y = delta.y/g_dt;
			g_velocities[g_mouseParticle].z = delta.z/g_dt;
		}

	
		memset(&timers, 0, sizeof(timers));

		g_windTime += g_dt;
	
		const Vec3 kWindDir = Vec3(3.0f, 15.0f, 0.0f);
		const float kNoise = Perlin1D(g_windTime*g_windFrequency, 10, 0.25f);
		Vec3 wind = g_windStrength*kWindDir*Vec3(kNoise, fabsf(kNoise), 0.0f);
				
		g_params.mWind[0] = wind.x;
		g_params.mWind[1] = wind.y;
		g_params.mWind[2] = wind.z;

		if (g_wavePool)
		{
			g_waveTime += g_dt;

			g_params.mPlanes[2][3] = g_wavePlane + (sinf(float(g_waveTime)*g_waveFrequency - kPi*0.5f)*0.5f + 0.5f)*g_waveAmplitude;
		}

		// give scene a chance to make changes
		g_scenes[g_scene]->Update();

		// always set positions to flush any other updates (i.e. mouse picking)
		flexSetParticles(g_flex, &g_positions[0].x, g_positions.size(), eFlexMemoryHost);
		flexSetVelocities(g_flex, &g_velocities[0].x, g_velocities.size(), eFlexMemoryHost);	
		flexSetPhases(g_flex, &g_phases[0], g_phases.size(), eFlexMemoryHost);

		flexSetParams(g_flex, &g_params);
		flexUpdateSolver(g_flex, g_dt, g_numSubsteps, g_profile?&timers:NULL);

		g_frame++;
		g_step = false;
	}

	Vec3 forward(-sinf(g_camAngle.x)*cosf(g_camAngle.y), sinf(g_camAngle.y), -cosf(g_camAngle.x)*cosf(g_camAngle.y));
	Vec3 right(Normalize(Cross(forward, Vec3(0.0f, 1.0f, 0.0f))));

	g_camSmoothVel = Lerp(g_camSmoothVel, g_camVel, 0.1f);
	g_camPos += (forward*g_camSmoothVel.z + right*g_camSmoothVel.x + Cross(right, forward)*g_camSmoothVel.y);

	glBindFramebuffer(GL_DRAW_FRAMEBUFFER_EXT, g_msaaFbo);
	glClearColor(powf(g_clearColor.x, 1.0f/2.2f), powf(g_clearColor.y, 1.0f/2.2f), powf(g_clearColor.z, 1.0f/2.2f), 0.0f);	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glDisable(GL_LIGHTING);
	glDisable(GL_BLEND);
	
	if (g_wireframe)
		glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );

	glPointSize(5.0f);

	float fov = kPi/4.0f;
	float aspect = float(g_screenWidth)/g_screenHeight;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(RadToDeg(fov), aspect, g_camNear, g_camFar);
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glRotatef(RadToDeg(-g_camAngle.x), 0.0f, 1.0f, 0.0f);
	glRotatef(RadToDeg(-g_camAngle.y), cosf(-g_camAngle.x), 0.0f, sinf(-g_camAngle.x));	
	glTranslatef(-g_camPos.x, -g_camPos.y, -g_camPos.z);
	
	// allow each scene to provide a modified world space basis (i.e.: UE3 coord system)
	Matrix44 basis = g_scenes[g_scene]->GetBasis();
	glMultMatrixf((float*)&basis);

	//---------------------------------------------------

	int numParticles = flexGetActiveCount(g_flex);
	int numDiffuse = 0;

	// need up to date positions host side for interaction / debug rendering
	flexGetParticles(g_flex, &g_positions[0].x, g_positions.size(), eFlexMemoryHost);
	flexGetVelocities(g_flex, &g_velocities[0].x, g_velocities.size(), eFlexMemoryHost);
	flexGetNormals(g_flex, &g_normals[0].x, g_normals.size(), eFlexMemoryHost);
	
	// readback normals
	if (g_triangles.size())
		flexGetDynamicTriangles(g_flex, &g_triangles[0], &g_triangleNormals[0].x, g_triangles.size()/3, eFlexMemoryHost);

	//---------------------------------------------------
	// map render buffers

	void* positionsPtr;
	void* densityPtr;
	void* anisotropyPtr[3];
	void* diffusePosPtr;
	void* diffuseVelPtr = 0;
	void* diffuseIdxPtr = 0;

	cudaGraphicsResource* resources[8];
	const int numResources = (g_renderBuffers.mNumDiffuseParticles)?8:5;

	resources[0] = g_renderBuffers.mPositionRes;
	resources[1] = g_renderBuffers.mAnisotropyRes[0];
	resources[2] = g_renderBuffers.mAnisotropyRes[1];
	resources[3] = g_renderBuffers.mAnisotropyRes[2];
	resources[4] = g_renderBuffers.mDensityRes;
	
	if (g_renderBuffers.mNumDiffuseParticles)
	{
		resources[5] = g_renderBuffers.mDiffusePositionRes;
		resources[6] = g_renderBuffers.mDiffuseVelocityRes;
		resources[7] = g_renderBuffers.mDiffuseIndicesRes;
	}

	
	cudaCheck(cudaGraphicsMapResources(numResources, resources));

	size_t s;
	cudaCheck(cudaGraphicsResourceGetMappedPointer(&positionsPtr, &s, g_renderBuffers.mPositionRes));
	cudaCheck(cudaGraphicsResourceGetMappedPointer(&anisotropyPtr[0], &s, g_renderBuffers.mAnisotropyRes[0]));
	cudaCheck(cudaGraphicsResourceGetMappedPointer(&anisotropyPtr[1], &s, g_renderBuffers.mAnisotropyRes[1]));
	cudaCheck(cudaGraphicsResourceGetMappedPointer(&anisotropyPtr[2], &s, g_renderBuffers.mAnisotropyRes[2]));
	cudaCheck(cudaGraphicsResourceGetMappedPointer(&densityPtr, &s, g_renderBuffers.mDensityRes));

	// read back new positions
	if (g_drawEllipsoids)
		flexGetSmoothParticles(g_flex, (float*)positionsPtr, numParticles, eFlexMemoryDeviceAsync);
	else
		flexGetParticles(g_flex, (float*)positionsPtr, numParticles, eFlexMemoryDeviceAsync);

	flexGetAnisotropy(g_flex, (float*)anisotropyPtr[0], (float*)anisotropyPtr[1], (float*)anisotropyPtr[2], eFlexMemoryDeviceAsync);
	
	if (g_drawDensity)
		flexGetDensities(g_flex, (float*)densityPtr, eFlexMemoryDeviceAsync);
	else
		flexGetPhases(g_flex, (int*)densityPtr, numParticles, eFlexMemoryDeviceAsync);

	if (g_renderBuffers.mNumDiffuseParticles)
	{
		cudaCheck(cudaGraphicsResourceGetMappedPointer(&diffusePosPtr, &s, g_renderBuffers.mDiffusePositionRes));
		cudaCheck(cudaGraphicsResourceGetMappedPointer(&diffuseVelPtr, &s, g_renderBuffers.mDiffuseVelocityRes));
		cudaCheck(cudaGraphicsResourceGetMappedPointer(&diffuseIdxPtr, &s, g_renderBuffers.mDiffuseIndicesRes));

		numDiffuse = flexGetDiffuseParticles(g_flex, (float*)diffusePosPtr,(float*)diffuseVelPtr, (int*)diffuseIdxPtr, eFlexMemoryDeviceAsync);		
	}

	cudaCheck(cudaGraphicsUnmapResources(numResources, resources, 0));

	//-----------------
	// calculate lighting for scene

	Vec3 sceneExtents = g_sceneUpper - g_sceneLower;
	Vec3 sceneCenter = 0.5f*(g_sceneUpper + g_sceneLower);

	g_lightDir = Normalize(Vec3(5.0f, 15.0f, 7.5f));
	g_lightPos = sceneCenter + g_lightDir*Length(sceneExtents)*g_lightDistance;
	g_lightTarget = sceneCenter;

	// calculate tight bounds for shadow frustum
	float lightFov = 2.0f*atanf(Length(g_sceneUpper-sceneCenter)/Length(g_lightPos-sceneCenter));
	Matrix44 lightPerspective = ProjectionMatrix(RadToDeg(lightFov), 1.0f, 1.0f, 1000.0f);
	Matrix44 lightView = LookAtMatrix(Point3(g_lightPos), Point3(g_lightTarget));
	Matrix44 lightTransform = lightPerspective*lightView;

	// non-fluid particles maintain radius distance (not 2.0f*radius) so multiply by a half
	float radius = g_params.mSolidRestDistance;

	// fluid particles overlap twice as much again, so half the radius again
	if (g_params.mFluid)
		radius = g_params.mFluidRestDistance;

	radius *= 0.5f;
	radius *= g_pointScale;

	//----------------
	// shadowing pass 

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadMatrixf(lightPerspective);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadMatrixf(lightView);

	// Skin rigids
	if (g_rigidOffsets.size() && g_meshSkinIndices.size())
	{
		flexGetRigidTransforms(g_flex, (float*)&g_rigidRotations[0], (float*)&g_rigidTranslations[0], eFlexMemoryHost);
		SkinMesh();
	}

	// create shadow maps
	ShadowBegin(g_shadowTex, g_shadowBuf);

	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(8.f, 8.f);

	int shadowParticles = numParticles; 
	int shadowParticlesOffset =0;

	if (!g_drawPoints)
	{
		shadowParticles = 0;
		
		if (g_drawEllipsoids && g_params.mFluid)
		{
			shadowParticles = numParticles-g_numSolidParticles;
			shadowParticlesOffset = g_numSolidParticles;
		}
	}
	else
	{
		int offset = g_drawMesh?g_numSolidParticles:0;

		shadowParticles = numParticles-offset;
		shadowParticlesOffset = offset;
	}

	DrawPoints(g_renderBuffers.mPositionVBO, g_renderBuffers.mDensityVBO, shadowParticles, shadowParticlesOffset, radius*0.7f, 2048, 1.0f, lightFov, g_lightPos, g_lightTarget, lightTransform, g_shadowTex, g_drawDensity, false);

	if (g_drawMesh)
		DrawMesh(g_mesh, g_meshColor);

	DrawMesh(g_staticMesh, g_meshColor);
	DrawConvexes();

	if (g_drawCloth)
		DrawCloth(&g_positions[0], &g_normals[0], g_uvs.size()?&g_uvs[0].x:NULL, &g_triangles[0], g_triangles.size()/3, g_positions.size(), 3, g_expandCloth);

	if (g_drawRopes)
	{
		for (size_t i=0; i < g_ropes.size(); ++i)
			DrawRope(&g_positions[0], &g_ropes[i].mIndices[0], g_ropes[i].mIndices.size(), radius*g_ropeScale, i);
	}

	// give scene a chance to do custom drawing
	g_scenes[g_scene]->Draw(1);

	glDisable(GL_POLYGON_OFFSET_FILL);

	ShadowEnd();
	
	glViewport(0, 0, g_screenWidth, g_screenHeight);
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	//----------------
	// reflection pass

	if (g_drawReflection && g_params.mFluid)
	{
		const int reflectWidth = g_screenWidth/2;
		const int reflectHeight = g_screenHeight/2;

		BindSolidShader(g_lightPos, g_lightTarget, lightTransform, g_shadowTex, 0.0f, Vec4(g_clearColor, g_fogDistance));
		ReflectBegin(Vec4(0.0f, 1.0f, 0.0f, -g_params.mRadius*0.5f), reflectWidth, reflectHeight);

		glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		{
			if (g_drawMesh)
				DrawMesh(g_mesh, g_meshColor);

			DrawMesh(g_staticMesh, g_meshColor);
			DrawConvexes();

			if (g_drawCloth)
				DrawCloth(&g_positions[0], &g_normals[0],  g_uvs.size()?&g_uvs[0].x:NULL, &g_triangles[0], g_triangles.size()/3, g_positions.size(), 3, g_expandCloth);

			// draw solid particles separately
			if (g_numSolidParticles && g_drawPoints)
				DrawPoints(g_renderBuffers.mPositionVBO, g_renderBuffers.mDensityVBO, g_numSolidParticles, 0, radius, float(g_screenWidth), aspect, fov, g_lightPos, g_lightTarget, lightTransform, g_shadowTex, g_drawDensity, g_msaa);

		}
		ReflectEnd(g_reflectTex, reflectWidth, reflectHeight);	

		glViewport(0, 0, g_screenWidth, g_screenHeight);
	}
	
	//----------------
	// lighting pass

	glClearColor(powf(g_clearColor.x, 1.0f/2.2f), powf(g_clearColor.y, 1.0f/2.2f), powf(g_clearColor.z, 1.0f/2.2f), 0.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	BindSolidShader(g_lightPos, g_lightTarget, lightTransform, g_shadowTex, 0.0f, Vec4(g_clearColor, g_fogDistance));

	DrawPlanes((Vec4*)g_params.mPlanes, g_params.mNumPlanes, g_drawPlaneBias);

	if (g_drawMesh)
		DrawMesh(g_mesh, g_meshColor);

	DrawMesh(g_staticMesh, g_meshColor);
	DrawConvexes();

	if (g_drawCloth && g_absorb){
		/*change cloth color*/
		CalculateClothColors();
		DrawClothColor(&g_positions[0], &g_colors[0], &g_normals[0], g_uvs.size() ? &g_uvs[0].x : NULL, &g_triangles[0], g_triangles.size() / 3, g_positions.size(), 3, g_expandCloth);
		//DrawCloth(&g_positions[0], &g_normals[0], g_uvs.size() ? &g_uvs[0].x : NULL, &g_triangles[0], g_triangles.size() / 3, g_positions.size(), 3, g_expandCloth);
	}

	if (g_drawCloth){
		DrawCloth(&g_positions[0], &g_normals[0], g_uvs.size() ? &g_uvs[0].x : NULL, &g_triangles[0], g_triangles.size() / 3, g_positions.size(), 3, g_expandCloth);
	}
	if (g_drawRopes)
	{
		for (size_t i=0; i < g_ropes.size(); ++i)
			DrawRope(&g_positions[0], &g_ropes[i].mIndices[0], g_ropes[i].mIndices.size(), g_params.mRadius*0.5f*g_ropeScale, i);
	}

	// give scene a chance to do custom drawing
	g_scenes[g_scene]->Draw(0);

	UnbindSolidShader();

	// first pass of diffuse particles (behind fluid surface)
	if (g_drawDiffuse)
		RenderDiffuse(g_renderer, g_renderBuffers.mDiffusePositionVBO, g_renderBuffers.mDiffuseVelocityVBO, g_renderBuffers.mDiffuseIndicesIBO, numDiffuse, radius*g_diffuseScale, float(g_screenWidth), aspect, fov, g_diffuseColor, g_lightPos, g_lightTarget, lightTransform, g_shadowTex, g_diffuseMotionScale, g_diffuseInscatter, g_diffuseOutscatter,  g_diffuseShadow, false);
		
	if (g_drawEllipsoids && g_params.mFluid)
	{
		// draw solid particles separately
		if (g_numSolidParticles && g_drawPoints)
			DrawPoints(g_renderBuffers.mPositionVBO, g_renderBuffers.mDensityVBO, g_numSolidParticles, 0, radius, float(g_screenWidth), aspect, fov, g_lightPos, g_lightTarget, lightTransform, g_shadowTex, g_drawDensity, g_msaa);

		if (dynamic_cast<RayleighTaylor3D*>(g_scenes[g_scene]))
		{
			// SIGGRAPH hack, two pass render of fluids for multiple phases
			int layer0 = 128*24*4;
			int layer1 = 128*24*24;
			int layer2 = 128*24*24;

			RenderEllipsoids(g_renderer, g_renderBuffers.mPositionVBO, g_renderBuffers.mAnisotropyVBO[0], g_renderBuffers.mAnisotropyVBO[1], g_renderBuffers.mAnisotropyVBO[2], g_renderBuffers.mDensityVBO, layer0, 0, radius, float(g_screenWidth), aspect, fov, g_lightPos, g_lightTarget, lightTransform, g_shadowTex, g_reflectTex, Vec4(0.0f, 0.0f, 0.0f, 0.0f), g_blur, g_ior, g_drawOpaque);
			RenderEllipsoids(g_renderer, g_renderBuffers.mPositionVBO, g_renderBuffers.mAnisotropyVBO[0], g_renderBuffers.mAnisotropyVBO[1], g_renderBuffers.mAnisotropyVBO[2], g_renderBuffers.mDensityVBO, layer1, layer0, radius, float(g_screenWidth), aspect, fov, g_lightPos, g_lightTarget, lightTransform, g_shadowTex, g_reflectTex, Vec4(0.3f, 1.3f, 0.7f, 0.0f), g_blur, g_ior, g_drawOpaque);
			RenderEllipsoids(g_renderer, g_renderBuffers.mPositionVBO, g_renderBuffers.mAnisotropyVBO[0], g_renderBuffers.mAnisotropyVBO[1], g_renderBuffers.mAnisotropyVBO[2], g_renderBuffers.mDensityVBO, layer2, layer0+layer1, radius, float(g_screenWidth), aspect, fov, g_lightPos, g_lightTarget, lightTransform, g_shadowTex, g_reflectTex, Vec4(0.1f, 0.4f, 0.8f, 0.75f), g_blur, g_ior, g_drawOpaque);			
		}
		else if (dynamic_cast<RayleighTaylor2D*>(g_scenes[g_scene]))
		{
			int layer0 = 128*4;
			int layer1 = 128*24;
			int layer2 = 128*24;

			RenderEllipsoids(g_renderer, g_renderBuffers.mPositionVBO, g_renderBuffers.mAnisotropyVBO[0], g_renderBuffers.mAnisotropyVBO[1], g_renderBuffers.mAnisotropyVBO[2], g_renderBuffers.mDensityVBO, layer0, 0, radius, float(g_screenWidth), aspect, fov, g_lightPos, g_lightTarget, lightTransform, g_shadowTex, g_reflectTex, Vec4(0.0f, 0.0f, 0.0f, 0.0f), g_blur, g_ior, g_drawOpaque);
			RenderEllipsoids(g_renderer, g_renderBuffers.mPositionVBO, g_renderBuffers.mAnisotropyVBO[0], g_renderBuffers.mAnisotropyVBO[1], g_renderBuffers.mAnisotropyVBO[2], g_renderBuffers.mDensityVBO, layer1, layer0, radius, float(g_screenWidth), aspect, fov, g_lightPos, g_lightTarget, lightTransform, g_shadowTex, g_reflectTex, Vec4(0.3f, 1.3f, 0.7f, 0.0f), g_blur, g_ior, g_drawOpaque);			
			RenderEllipsoids(g_renderer, g_renderBuffers.mPositionVBO, g_renderBuffers.mAnisotropyVBO[0], g_renderBuffers.mAnisotropyVBO[1], g_renderBuffers.mAnisotropyVBO[2], g_renderBuffers.mDensityVBO, layer2, layer0+layer1, radius, float(g_screenWidth), aspect, fov, g_lightPos, g_lightTarget, lightTransform, g_shadowTex, g_reflectTex, Vec4(0.1f, 0.4f, 0.8f, 0.75f), g_blur, g_ior, g_drawOpaque);			
		}
		else
		{
			// render fluid surface
			RenderEllipsoids(g_renderer, g_renderBuffers.mPositionVBO, g_renderBuffers.mAnisotropyVBO[0], g_renderBuffers.mAnisotropyVBO[1], g_renderBuffers.mAnisotropyVBO[2], g_renderBuffers.mDensityVBO, numParticles-g_numSolidParticles, g_numSolidParticles, radius, float(g_screenWidth), aspect, fov, g_lightPos, g_lightTarget, lightTransform, g_shadowTex, g_reflectTex, g_fluidColor, g_blur, g_ior, g_drawOpaque);
		}
		
		// second pass of diffuse particles for particles in front of fluid surface
		if (g_drawDiffuse)
			RenderDiffuse(g_renderer, g_renderBuffers.mDiffusePositionVBO, g_renderBuffers.mDiffuseVelocityVBO, g_renderBuffers.mDiffuseIndicesIBO, numDiffuse, radius*g_diffuseScale, float(g_screenWidth), aspect, fov, g_diffuseColor, g_lightPos, g_lightTarget, lightTransform, g_shadowTex, g_diffuseMotionScale, g_diffuseInscatter, g_diffuseOutscatter, g_diffuseShadow, true);
	}
	else
	{
		// draw all particles as spheres
		if (g_drawPoints)
		{
			int offset = g_drawMesh?g_numSolidParticles:0;

			DrawPoints(g_renderBuffers.mPositionVBO, g_renderBuffers.mDensityVBO, numParticles-offset, offset, radius, float(g_screenWidth), aspect, fov, g_lightPos, g_lightTarget, lightTransform, g_shadowTex, g_drawDensity, g_msaa);
		}
	}


	//-----------------------
	// Debug rendering

	if (g_mouseParticle != -1)
	{
		// draw mouse spring
		glUseProgram(0);
		glDisable(GL_DEPTH_TEST);
		glDisable(GL_BLEND);

		for (int i=0; i < 8; ++i)
		{
			glActiveTexture(GL_TEXTURE0 + i);
			glDisable(GL_TEXTURE_2D);
		}
	
		glBegin(GL_LINES);
		glVertex3fv(g_mousePos);
		glVertex3fv(g_positions[g_mouseParticle]);
		glEnd();

		glPointSize(10.0f);

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glEnable(GL_POINT_SPRITE);
		glEnable(GL_POINT_SMOOTH);
		glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);

		glBegin(GL_POINTS);
		glColor3f(1.0f, 1.0f, 1.0f);
		glVertex3fv(g_positions[g_mouseParticle]);
		glEnd();

		glBegin(GL_POINTS);
		glColor3f(1.0f, 1.0f, 1.0f);
		glVertex3fv(g_mousePos);
		glEnd();

		glEnable(GL_DEPTH_TEST);
	}

	// springs
	if (g_drawSprings)
	{
		glUseProgram(0);
		glDisable(GL_BLEND);
		glDisable(GL_TEXTURE_2D);
				
		if (g_drawSprings == 1)
		{
			// stretch 
			glColor4f(0.0f, 0.0f, 1.0f, 0.8f);	
		}
		if (g_drawSprings == 2)
		{
			// tether
			glColor4f(0.0f, 1.0f, 0.0f, 0.8f);
		}


		glBegin(GL_LINES);	

		size_t start = 0;

		for (size_t i=start; i < g_springLengths.size(); ++i)
		{
			if (g_drawSprings == 1 && g_springStiffness[i] < 0.0f)
				continue;
			if (g_drawSprings == 2 && g_springStiffness[i] > 0.0f)
				continue;

			int a = g_springIndices[i*2];
			int b = g_springIndices[i*2+1];

			glVertex3fv(g_positions[a]);
			glVertex3fv(g_positions[b]);
		}

		glEnd();
		glDisable(GL_BLEND);

	}

	if (g_drawNormals)
	{

		flexGetNormals(g_flex, (float*)&g_normals[0], g_normals.size(), eFlexMemoryHost);

		glUseProgram(0);
		glDisable(GL_BLEND);
		glDisable(GL_TEXTURE_2D);

		glColor3f(0.0f, 1.0f, 0.0f);
		glBegin(GL_LINES);

		for (size_t i=0; i < g_normals.size(); ++i)
		{
			glVertex3fv(g_positions[i]);
			glVertex3fv(g_positions[i] - g_normals[i]*g_normals[i].w);
		}

		glEnd();
	}
	
	if (0)
	{
		// visualize bounds
		Vec3 lower, upper;
		flexGetBounds(g_flex, lower, upper);

		DrawBoundingBox(lower, upper);
	}

	if (g_drawConvexGrid)
	{
		// visualize convex grid
		const int gridDim = 128;
		static int gridCounts[gridDim*gridDim];
		Vec2 gridLower;
		Vec2 gridUpper;		
		int gridAxis;

		flexGetConvexGrid(g_flex, gridCounts, gridLower, gridUpper, &gridAxis);

		Vec2 gridEdge = (gridUpper-gridLower)/gridDim;

		glUseProgram(0);
		glEnable(GL_DEPTH_TEST);
		glDisable(GL_BLEND);
		glDisable(GL_TEXTURE_2D);
		glPointSize(10.0f);
		glBegin(GL_POINTS);
	
		const int c1 = (gridAxis+2)%3;
		const int c2 = (gridAxis+1)%3;
		
		Vec3 e1, e2;
		e1[c1] = 1.0f;
		e2[c2] = 1.0f;

		for (int y=0; y < gridDim; ++y)
		{
			for (int x=0; x < gridDim; ++x)
			{
				glColor3f(gridCounts[y*gridDim + x]/1.0f, 0.5f, 0.5f);

				Vec3 p = (gridLower.x + gridEdge.x*(x+0.5f))*e1 + (gridLower.y + gridEdge.y*(y+0.5f))*e2;
				
				glVertex3fv(p);
			}
		}

		glEnd();
	}

	// draw triangle grid
	if (0 && g_staticMesh)
	{
		const int gridDim = 128;
		const int gridSize = gridDim*gridDim*gridDim;
		std::vector<int> counts(gridSize);

		Vec3 lower;
		float width;

		//flexGetStaticTriangleGrid(g_flex, &counts[0], &lower.x, &width);
	
		glUseProgram(0);
		glEnable(GL_DEPTH_TEST);
		glDisable(GL_BLEND);
		glDisable(GL_TEXTURE_2D);

		/*
		glPointSize(10.0f);
		glEnable(GL_POINT_SPRITE);
		glBegin(GL_POINTS);
		*/

		for (int z=0; z < gridDim; ++z)
		{
			for (int y=0; y < gridDim; ++y)
			{
				for (int x=0; x < gridDim; ++x)
				{
					const int idx = y*gridDim*gridDim + z*gridDim + x;
					const int count = counts[idx];

					if (count)
					{
						if (count == 1)
							glColor3f(0.5f, 0.5, 0.5f);
						else
							glColor3f(0.0f, 0.5f, 0.0f);

						DrawBoundingBox(lower + Vec3(float(x), float(y), float(z))*width, lower + Vec3(float(x+1), float(y+1), float(z+1))*width);

						//glVertex3fv(cellCenter);
					}
				}
			}
		}

		//glEnd();
	}

	if (g_msaa)
	{
		// blit the msaa buffer to the window
		glVerify(glBindFramebuffer(GL_READ_FRAMEBUFFER_EXT, g_msaaFbo));
		glVerify(glBindFramebuffer(GL_DRAW_FRAMEBUFFER_EXT, 0));
		glVerify(glBlitFramebuffer(0, 0, g_screenWidth, g_screenHeight, 0, 0, g_screenWidth, g_screenHeight, GL_COLOR_BUFFER_BIT, GL_LINEAR));
	}

	// render help to back buffer
	glVerify(glBindFramebuffer(GL_FRAMEBUFFER, 0));
	glClear(GL_DEPTH_BUFFER_BIT);


	if (g_showHelp)
	{				
		glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );

		glActiveTexture(GL_TEXTURE0);
		glDisable(GL_TEXTURE_2D);
		glDisable(GL_TEXTURE_RECTANGLE_ARB);	
		glActiveTexture(GL_TEXTURE1);
		glDisable(GL_TEXTURE_2D);
		glActiveTexture(GL_TEXTURE2);
		glDisable(GL_TEXTURE_2D);
		glActiveTexture(GL_TEXTURE3);
		glDisable(GL_TEXTURE_2D);
		glActiveTexture(GL_TEXTURE4);
		glDisable(GL_TEXTURE_2D);
		glDisable(GL_TEXTURE_CUBE_MAP);
		glActiveTexture(GL_TEXTURE5);
		glDisable(GL_TEXTURE_2D);

		glActiveTexture(GL_TEXTURE0);

		glDisable(GL_BLEND);
		glDisable(GL_LIGHTING);
		glDisable(GL_BLEND);
		glDisable(GL_POINT_SPRITE);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();
		
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		gluOrtho2D(0, g_screenWidth, 0, g_screenHeight);

		int x = g_screenWidth-200;
		int y = g_screenHeight - 23;
			
		glUseProgram(0);
		glDisable(GL_DEPTH_TEST);
		glDisable(GL_CULL_FACE);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDisable(GL_TEXTURE_2D);
		glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

		// imgui
		unsigned char button = 0;
		if (lastb == GLUT_LEFT_BUTTON)
			button = IMGUI_MBUT_LEFT;
		else if (lastb == GLUT_RIGHT_BUTTON)
			button = IMGUI_MBUT_RIGHT;

		imguiBeginFrame(lastx, g_screenHeight-lasty, button, 0);
		
		x += 180;

		if (1)
		{
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Frame: %d", g_frame); y -= 13;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Frame Time Total: %.2fms", g_realdt*1000.0f); y -= 13;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Particle Count: %d", numParticles); y -= 13;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Diffuse Count: %d", numDiffuse); y -= 13;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT,  "Rigid Count: %d", g_rigidOffsets.size()>0?g_rigidOffsets.size()-1:0); y -= 13;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT,  "Spring Count: %d", g_springLengths.size()); y -= 13;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT,  "Num Substeps: %d", g_numSubsteps); y -= 13;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT,  "Num Iterations: %d", g_params.mNumIterations); y -= 13;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT,  "CUDA Device: %s", g_device); y -= 26;
		}
		
		if (g_profile)
		{
			
			
			DrawImguiString(x, y, Vec3(0.0f, 1.0f, 1.0f), IMGUI_ALIGN_RIGHT, "Total GPU Sim Time: %.2fms", timers.mTotal); y -= 26;

			DrawImguiString(x, y, Vec3(0.0f, 1.0f, 0.0f), IMGUI_ALIGN_RIGHT, "GPU Timers"); y -= 13;

			glColor3f(1.0f, 1.0f, 1.0f);
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Predict: %.2fms", timers.mPredict); y -= 13;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Create Cell Indices: %.2fms", timers.mCreateCellIndices); y -= 13;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Sort Cell Indices: %.2fms", timers.mSortCellIndices); y -= 13;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Reorder: %.2fms", timers.mReorder); y -= 13;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "CreateGrid: %.2fms", timers.mCreateGrid); y -= 13;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Collide Particles: %.2fms", timers.mCollideParticles); y -= 13;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Collide Convexes: %.2fms", timers.mCollideConvexes); y -= 13;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Collide Triangles: %.2fms", timers.mCollideTriangles); y -= 13;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Calculate Density: %.2fms", timers.mCalculateDensity); y -= 13;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Solve Densities: %.2fms", timers.mSolveDensities); y -= 13;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Solve Velocities: %.2fms", timers.mSolveVelocities); y -= 13;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Solve Rigids: %.2fms", timers.mSolveShapes); y -= 13;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Solve Springs: %.2fms", timers.mSolveSprings); y -= 13;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Solve Inflatables: %.2fms", timers.mSolveInflatables); y -= 13;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Solve Contacts: %.2fms", timers.mSolveContacts); y -= 13;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Finalize: %.2fms", timers.mFinalize); y -= 13;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Update Triangles: %.2fms", timers.mUpdateTriangles); y -= 13;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Update Normals: %.2fms", timers.mUpdateNormals); y -= 13;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Update Bounds: %.2fms", timers.mUpdateBounds); y -= 13;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Calculate Anisotropy: %.2fms", timers.mCalculateAnisotropy); y -= 13;		
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Update Diffuse: %.2fms", timers.mUpdateDiffuse); y -= 26;

		}

		x -= 180;

		int uiOffset = 250;
		int uiBorder = 20;
		int uiWidth = 250;
		int uiHeight = g_screenHeight-uiOffset-uiBorder*3;
		int uiLeft = uiBorder;

		imguiBeginScrollArea("Scene", uiLeft , g_screenHeight-uiBorder-uiOffset, uiWidth, uiOffset, &g_levelScroll);

		int newScene = -1;
		for (int i = 0; i < int(g_scenes.size()); ++i)
		{
			unsigned int color = g_scene==i?imguiRGBA(255,151,61,200):imguiRGBA(255,255,255,200);
			if (imguiItem(g_scenes[i]->GetName(), true, color))
			{
				newScene = i;
			}
		}

		if (newScene != -1)
		{
			g_scene = newScene;
			Init(g_scene);
		}

		imguiEndScrollArea();

		static int scroll = 0;
		imguiBeginScrollArea("Options", uiLeft, g_screenHeight-uiBorder-uiHeight-uiOffset-uiBorder, uiWidth, uiHeight, &scroll); 
		imguiSeparatorLine();

		// global options
		imguiLabel("Global");
		if (imguiCheck("Emit particles", g_emit))
			g_emit = !g_emit;

		if (imguiCheck("Pause", g_pause))
			g_pause = !g_pause;

		imguiSeparatorLine();

		if (imguiCheck("Wireframe", g_wireframe))
			g_wireframe = !g_wireframe;

		if (imguiCheck("Draw Points", g_drawPoints))
			g_drawPoints = !g_drawPoints;

		if (imguiCheck("Draw Fluid", g_drawEllipsoids))			
			g_drawEllipsoids = !g_drawEllipsoids;

		if (imguiCheck("Draw Mesh", g_drawMesh))
		{
			g_drawMesh = !g_drawMesh;
			g_drawRopes = !g_drawRopes;
		}

		if (imguiCheck("Draw Springs", bool(g_drawSprings!=0)))
			g_drawSprings = (g_drawSprings)?0:1;

		imguiSeparatorLine();

		// scene options
		g_scenes[g_scene]->DoGui();

		if (imguiButton("Reset Scene"))
			Reset();

		imguiSeparatorLine();

		float n = float(g_numSubsteps);
		if (imguiSlider("Num Substeps", &n, 1, 10, 1))
			g_numSubsteps = int(n);

		n = float(g_params.mNumIterations);
		if (imguiSlider("Num Iterations", &n, 1, 20, 1))
			g_params.mNumIterations = int(n);

		imguiSeparatorLine();
		imguiSlider("Gravity X", &g_params.mGravity[0], -50.0f, 50.0f, 1.0f);
		imguiSlider("Gravity Y", &g_params.mGravity[1], -50.0f, 50.0f, 1.0f);
		imguiSlider("Gravity Z", &g_params.mGravity[2], -50.0f, 50.0f, 1.0f);

		imguiSeparatorLine();
		imguiSlider("Radius", &g_params.mRadius, 0.01f, 0.5f, 0.01f);
		imguiSlider("Solid Radius", &g_params.mSolidRestDistance, 0.0f, 0.5f, 0.001f);
		imguiSlider("Fluid Radius", &g_params.mFluidRestDistance, 0.0f, 0.5f, 0.001f);
		imguiSlider("Collision Distance", &g_params.mCollisionDistance, 0.0f, 0.5f, 0.001f);
		imguiSlider("Collision Margin", &g_params.mShapeCollisionMargin, 0.0f, 5.0f, 0.01f);

		//// common params
		//imguiSeparatorLine();
		//imguiSlider("Dynamic Friction", &g_params.mDynamicFriction, 0.0f, 1.0f, 0.01f);
		//imguiSlider("Static Friction", &g_params.mStaticFriction, 0.0f, 1.0f, 0.01f);
		//imguiSlider("Particle Friction", &g_params.mParticleFriction, 0.0f, 1.0f, 0.01f);
		//imguiSlider("Restitution", &g_params.mRestitution, 0.0f, 1.0f, 0.01f);
		//imguiSlider("SleepThreshold", &g_params.mSleepThreshold, 0.0f, 1.0f, 0.01f);
		//imguiSlider("Shock Propagation", &g_params.mShockPropagation, 0.0f, 10.0f, 0.01f);
		//imguiSlider("Damping", &g_params.mDamping, 0.0f, 10.0f, 0.01f);
		//imguiSlider("Dissipation", &g_params.mDissipation, 0.0f, 0.01f, 0.0001f);
		//imguiSlider("SOR", &g_params.mRelaxationFactor, 0.0f, 5.0f, 0.01f);
	
		// cloth params
		imguiSeparatorLine();
		imguiSlider("Wind", &g_windStrength, -1.0f, 1.0f, 0.01f);
		imguiSlider("Drag", &g_params.mDrag, 0.0f, 1.0f, 0.01f);
		imguiSlider("Lift", &g_params.mLift, 0.0f, 1.0f, 0.01f);
		imguiSeparatorLine();

		// fluid params
		if (imguiCheck("Fluid", g_params.mFluid))
			g_params.mFluid = !g_params.mFluid;

		imguiSlider("Adhesion", &g_params.mAdhesion, 0.0f, 10.0f, 0.01f);
		imguiSlider("Cohesion", &g_params.mCohesion, 0.0f, 0.2f, 0.0001f);
		imguiSlider("Surface Tension", &g_params.mSurfaceTension, 0.0f, 50.0f, 0.01f);
		imguiSlider("Viscosity", &g_params.mViscosity, 0.0f, 120.0f, 0.01f);
		imguiSlider("Vorticicty Confinement", &g_params.mVorticityConfinement, 0.0f, 120.0f, 0.1f);
		imguiSlider("Solid Pressure", &g_params.mSolidPressure, 0.0f, 1.0f, 0.01f);
		imguiSlider("Surface Drag", &g_params.mFreeSurfaceDrag, 0.0f, 1.0f, 0.01f);
		imguiSlider("Buoyancy", &g_params.mBuoyancy, -1.0f, 1.0f, 0.01f);

		imguiSeparatorLine();
		//imguiSlider("Anisotropy Scale", &g_params.mAnisotropyScale, 0.0f, 30.0f, 0.01f);
		//imguiSlider("Smoothing", &g_params.mSmoothing, 0.0f, 1.0f, 0.01f);

		//// diffuse params
		//imguiSeparatorLine();		
		//imguiSlider("Diffuse Threshold", &g_params.mDiffuseThreshold, 0.0f, 1000.0f, 1.0f);
		//imguiSlider("Diffuse Buoyancy", &g_params.mDiffuseBuoyancy, 0.0f, 2.0f, 0.01f);
		//imguiSlider("Diffuse Drag", &g_params.mDiffuseDrag, 0.0f, 2.0f, 0.01f);
		//imguiSlider("Diffuse Scale", &g_diffuseScale, 0.0f, 1.5f, 0.01f);
		//imguiSlider("Diffuse Alpha", &g_diffuseColor.w, 0.0f, 3.0f, 0.01f);
		//imguiSlider("Diffuse Inscatter", &g_diffuseInscatter, 0.0f, 2.0f, 0.01f);
		//imguiSlider("Diffuse Outscatter", &g_diffuseOutscatter, 0.0f, 2.0f, 0.01f);
		//imguiSlider("Diffuse Motion Blur", &g_diffuseMotionScale, 0.0f, 5.0f, 0.1f);

		//n = float(g_params.mDiffuseBallistic);
		//if (imguiSlider("Diffuse Ballistic", &n, 1, 40, 1))
		//	g_params.mDiffuseBallistic = int(n);

		imguiEndScrollArea();
		imguiEndFrame();
		imguiRenderGLDraw();

		if (g_capture)
		{
			int width = 5;

			int x = lastx;
			int y = g_screenHeight-lasty;

			glColor3f(1.0f, 1.0f, 1.0f);
			glBegin(GL_LINES);
			glVertex2f(float(x-width), float(y));
			glVertex2f(float(x+width), float(y));
			glVertex2f(float(x), float(y-width));
			glVertex2f(float(x), float(y+width));
			glEnd();
		}

		// restore camera transform (for picking)
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
		
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
	}


	if (g_capture && !g_pause)
	{
		TgaImage img;
		img.m_width = g_screenWidth;
		img.m_height = g_screenHeight;		
		img.m_data = new uint32_t[g_screenWidth*g_screenHeight];			

		glVerify(glReadBuffer(GL_BACK));
		glReadPixels(0, 0, g_screenWidth, g_screenHeight, GL_RGBA, GL_UNSIGNED_BYTE, img.m_data);

		fwrite(img.m_data, sizeof(uint32_t)*g_screenWidth*g_screenHeight, 1, g_ffmpeg);
		
		delete[] img.m_data;
	}

	glutSwapBuffers();


}

void GLUTReshape(int width, int height)
{
	printf("Reshaping\n");

	if (g_msaa)
	{
		glVerify(glBindFramebuffer( GL_FRAMEBUFFER, 0));

		if (g_msaaFbo)
		{
			glVerify(glDeleteFramebuffers(1, &g_msaaFbo));
			glVerify(glDeleteRenderbuffers(1, &g_msaaColorBuf));
			glVerify(glDeleteRenderbuffers(1, &g_msaaDepthBuf));
		}

		int samples;
		glGetIntegerv(GL_MAX_SAMPLES_EXT, &samples);

		glVerify(glGenFramebuffers( 1, &g_msaaFbo));
		glVerify(glBindFramebuffer( GL_FRAMEBUFFER, g_msaaFbo));

		glVerify(glGenRenderbuffers(1, &g_msaaColorBuf));
		glVerify(glBindRenderbuffer(GL_RENDERBUFFER, g_msaaColorBuf));
		glVerify(glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, GL_RGBA8, width, height));
		
		glVerify(glGenRenderbuffers(1, &g_msaaDepthBuf));
		glVerify(glBindRenderbuffer(GL_RENDERBUFFER, g_msaaDepthBuf));
		glVerify(glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, GL_DEPTH_COMPONENT, width, height));
		glVerify(glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, g_msaaDepthBuf));

		glVerify(glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, g_msaaColorBuf));

		GLenum status = glCheckFramebufferStatus( GL_FRAMEBUFFER );
		assert(status == GL_FRAMEBUFFER_COMPLETE);		
	}

	if (!g_renderer || (width != g_screenWidth || height != g_screenHeight))
	{
		if (g_renderer)
			DestroyFluidRenderer(g_renderer);
		g_renderer = CreateFluidRenderer(width, height);

		if (g_reflectTex)
			ReflectDestroy(g_reflectTex);
		ReflectCreate(g_reflectTex, width/2, height/2);
	}
	
	g_screenWidth = width;
	g_screenHeight = height;
}

void GLUTArrowKeys(int key, int x, int y)
{
	switch(key)
	{
		case GLUT_KEY_DOWN:
		{
			if (g_selectedScene < int(g_scenes.size())-1)
				g_selectedScene++;

			break;
		}
		case GLUT_KEY_UP:
		{
			if (g_selectedScene > 0)
				g_selectedScene--;

			break;
		}
		case GLUT_KEY_LEFT:
		{
			if (g_scene > 0)
				--g_scene;
			Init(g_scene);

			// update scroll UI to center on selected scene
			g_levelScroll = max((g_scene-4)*24, 0);
			break;
		}
		case GLUT_KEY_RIGHT:
		{
			if (g_scene < int(g_scenes.size())-1)
				++g_scene;
			Init(g_scene);

			// update scroll UI to center on selected scene
			g_levelScroll = max((g_scene-4)*24, 0);
			break;
		}
	}
}
	
void GLUTArrowKeysUp(int key, int x, int y)
{
}

void GLUTKeyboardDown(unsigned char key, int x, int y)
{
	if (key > '0' && key <= '9')
	{
		g_scene = key-'0' - 1;
		Init(g_scene);
		return;
	}

	float kSpeed = g_camSpeed;

	switch (key)
	{
		case 'w':
		{
			g_camVel.z = kSpeed;
			break;
		}
		case 'W':
		{
			g_camVel.z = kSpeed*4.0f;
			break;
		}
		case 's':
		{
			g_camVel.z = -kSpeed;
			break;
		}
		case 'a':
		{
			g_camVel.x = -kSpeed;
			break;
		}
		case 'd':
		{
			g_camVel.x = kSpeed;
			break;
		}
		case 'q':
		{
			g_camVel.y = kSpeed;
			break;
		}
		case 'z':
		{
			//g_drawCloth = !g_drawCloth;
			g_camVel.y = -kSpeed;
			break;
		}

		case 'u':
		{
			if (g_fullscreen)
			{
				glutReshapeWindow(1280, 720);
				g_fullscreen = false;
			}
			else
			{
				glutFullScreen();
				g_fullscreen = true;
			}
			break;
		}
		case 'r':
		{
			Reset();
			break;
		}
		case 'y':
		{
			g_wavePool = !g_wavePool;
			break;
		}
		case 'c':
		{
#if _WIN32
			if (!g_ffmpeg)
			{
				// open ffmpeg stream

				int i=0;
				char buf[255];
				FILE* f = NULL;

				do
				{
					sprintf(buf, "../../movies/output%d.mp4", i);
					f = fopen(buf, "rb");
					if (f)
						fclose(f);

					++i;
				}
				while (f);

				const char* str = "ffmpeg -r 60 -f rawvideo -pix_fmt rgba -s 1280x720 -i - "
                  "-threads 0 -preset fast -y -crf 19 -pix_fmt yuv420p -tune animation -vf vflip %s";

				char cmd[1024];
				sprintf(cmd, str, buf);

				g_ffmpeg = _popen(cmd, "wb");
				assert(g_ffmpeg);
			}
			else
			{
				_pclose(g_ffmpeg);
				g_ffmpeg = NULL;
			}

			g_capture = !g_capture;
			g_frame = 0;
#endif
			break;
		}
		case 'p':
		{
			g_pause = !g_pause;
			break;
		}
		case 'o':
		{
			g_step = true;
			break;
		}
		case 'h':
		{
			g_showHelp = !g_showHelp;
			break;
		}
		case 'e':
		{
			g_drawEllipsoids = !g_drawEllipsoids;
			break;
		}
		case 't':
		{
			g_drawOpaque = !g_drawOpaque;
			break;
		}
		case 'v':
		{
			g_drawPoints = !g_drawPoints;
			break;
		}
		case 'f':
		{
			g_drawSprings = (g_drawSprings+1)%3;
			break;
		}
		case 'i':
		{
			g_drawDiffuse = !g_drawDiffuse;
			break;
		}
		case 'm':
		{
			g_drawMesh = !g_drawMesh;
			break;
		}
		case 'n':
		{
			g_drawRopes = !g_drawRopes;
			break;
		}
		case 'j':
		{
			g_windTime = 0.0f;
			g_windStrength = 1.5f;
			g_windFrequency = 0.2f;
			break;
		}
		case '.':
		{
			g_profile = !g_profile;
			break;
		}
		case 'g':
		{
			if (g_params.mGravity[1] != 0.0f)
				g_params.mGravity[1] = 0.0f;
			else
				g_params.mGravity[1] = -9.8f;

			break;
		}
		case 'b':
		{
			if (g_convexStarts.size())
			{
				g_convexStarts.pop_back();

				flexSetConvexes(
					g_flex, 
					(float*)&g_convexAabbMin[0], 
					(float*)&g_convexAabbMax[0], 
					(int*)&g_convexStarts[0], (int*)&g_convexLengths[0], 
					(float*)&g_convexPlanes[0], 
					(float*)&g_convexPositions[0], 
					(float*)&g_convexRotations[0], 
					(float*)&g_convexPrevPositions[0],
					(float*)&g_convexPrevRotations[0],
					&g_convexFlags[0],
					g_convexStarts.size(),
					g_convexPlanes.size(), eFlexMemoryHost);
			}
			break;
		}		
		case '-':
		{
			if (g_params.mNumPlanes)
				g_params.mNumPlanes--;

			break;
		}
		case ' ':
		{
			g_emit = !g_emit;
			break;
		}
		case ';':
		{
			g_debug = !g_debug;
			break;
		}
		case 'k':
		{
			if (!g_recording)
			{
				flexStartRecord(g_flex, "capture.bin");
				g_recording = true;
			}
			else
			{
				flexStopRecord(g_flex);
				g_recording = false;
			}
			break;
		}
		case 13:
		{
			g_scene = g_selectedScene;
			Init(g_scene);
			break;
		}
		case 27:
		{
			// for nvprof
			cudaDeviceReset();
#if _WIN32
			if (g_ffmpeg)
				_pclose(g_ffmpeg);
#endif

			exit(0);
			break;
		}
	};

	g_scenes[g_scene]->KeyDown(key);
}

void GLUTKeyboardUp(unsigned char key, int x, int y)
{
	switch (key)
	{
		case 'w':
		{
			g_camVel.z = 0.0f;
			break;
		}
		case 's':
		{
			g_camVel.z = 0.0f;
			break;
		}
		case 'a':
		{
			g_camVel.x = 0.0f;
			break;
		}
		case 'd':
		{
			g_camVel.x = 0.0f;
			break;
		}
		case 'q':
		case 'z':
		{
			g_camVel.y = 0.0f;
			break;
		}
	};
}

void GLUTEntryFunc(int state)
{
	if (state == GLUT_LEFT)
		g_camVel = Vec3(0.0f, 0.0f, 0.0f);
}


void GLUTMouseFunc(int b, int state, int x, int y)
{	
	switch (state)
	{
		case GLUT_UP:
		{
			lastx = x;
			lasty = y;
			lastb = -1;
			
			if (g_mouseParticle != -1)
			{
				/*
				g_springIndices.pop_back();
				g_springIndices.pop_back();
				g_springLengths.pop_back();
				g_springStiffness.pop_back();

				// remove particle
				int numActive = flexGetActiveCount(g_flex);
				flexSetActiveCount(g_flex, numActive - 1);


				// remove spring
				flexSetSprings(g_flex, &g_springIndices[0], &g_springLengths[0], &g_springStiffness[0], g_springLengths.size());
				*/

				// put particle mass back where it was
				g_positions[g_mouseParticle].w *= 1000.0f;

				// need to update positions straight away otherwise particle might be left with increased mass
				flexSetParticles(g_flex, &g_positions[0].x, g_positions.size(), eFlexMemoryHost);

				g_mouseParticle = -1;
			}

			break;
		}
		case GLUT_DOWN:
		{
			lastx = x;
			lasty = y;
			lastb = b;

			if (glutGetModifiers() == GLUT_ACTIVE_SHIFT && lastb == GLUT_LEFT_BUTTON)
			{
				assert(g_mouseParticle == -1);

				Vec3 origin, dir;
				GetViewRay(x, g_screenHeight-y, origin, dir);
				
				// need up to date positions
				flexGetParticles(g_flex, &g_positions[0].x, g_positions.size(), eFlexMemoryHost);
				flexGetVelocities(g_flex, &g_velocities[0].x, g_velocities.size(), eFlexMemoryHost);

				g_mouseParticle = PickParticle(origin, dir, &g_positions[0], &g_phases[0], g_positions.size(), g_params.mRadius*0.8f, g_mouseT);

				if (g_mouseParticle != -1)
				{
					printf("picked: %d, mass: %f v: %f %f %f\n", g_mouseParticle, g_positions[g_mouseParticle].w, g_velocities[g_mouseParticle].x, g_velocities[g_mouseParticle].y, g_velocities[g_mouseParticle].z);

					g_mousePos = origin + dir*g_mouseT;
					g_positions[g_mouseParticle].w *= 0.001f;		// increase picked particle's mass to force it towards the point

					flexSetParticles(g_flex, &g_positions[0].x, g_positions.size(), eFlexMemoryHost);
				}

				if (0 && g_mouseParticle != -1)
				{
					int index = flexGetActiveCount(g_flex);

					assert(size_t(index) < g_positions.size());

					// add particle
					g_positions[index] = Vec4(origin + g_mouseT*dir, 0.0f);
					g_velocities[index] = Vec3(0.0f);
					g_phases[index] = g_phases[g_mouseParticle];

					g_springIndices.push_back(index);
					g_springIndices.push_back(g_mouseParticle);
					g_springLengths.push_back(0.05f);
					g_springStiffness.push_back(0.8f);

					// add particle to sim
					int numActive = flexGetActiveCount(g_flex);
					flexSetActive(g_flex, &g_activeIndices[0], numActive+1, eFlexMemoryHost);

					flexSetParticles(g_flex, (float*)&g_positions[0], g_positions.size(), eFlexMemoryHost);
					flexSetVelocities(g_flex, (float*)&g_velocities[0], g_velocities.size(), eFlexMemoryHost);
					flexSetPhases(g_flex, &g_phases[0], g_phases.size(), eFlexMemoryHost);
					flexSetSprings(g_flex, &g_springIndices[0], &g_springLengths[0], &g_springStiffness[0], g_springLengths.size(), eFlexMemoryHost);
				}
			}
		}
	}
}

void GLUTPassiveMotionFunc(int x, int y)
{
	lastx = x;
	lasty = y;
}


void GLUTMotionFunc(int x, int y)
{
	float dx = float(x-lastx);
	float dy = float(y-lasty);

	lastx = x;
	lasty = y;

	if (lastb == GLUT_RIGHT_BUTTON)
	{
		const float kSensitivity = DegToRad(0.1f);
		const float kMaxDelta = FLT_MAX;

		g_camAngle.x -= Clamp(dx*kSensitivity, -kMaxDelta, kMaxDelta);
		g_camAngle.y -= Clamp(dy*kSensitivity, -kMaxDelta, kMaxDelta);
	}
	else
	{
		// move particle
		if (g_mouseParticle != -1)
		{
			Vec3 origin, dir;
			GetViewRay(x, g_screenHeight-y, origin, dir);

			g_mousePos = origin + dir*g_mouseT;
		}		
	}
}

void DrawConvexes()
{
	// convexes
	for (uint32_t i=0; i < g_convexStarts.size(); ++i)
	{
		ConvexMeshBuilder builder(&g_convexPlanes[g_convexStarts[i]]);
		builder(g_convexLengths[i]);

		for (uint32_t v=0; v < builder.mVertices.size(); ++v)
			builder.mVertices[v] = rotate(Vec3(g_convexRotations[i]), g_convexRotations[i].w, builder.mVertices[v]) + Vec3(g_convexPositions[i]);

		glBegin(GL_TRIANGLES);
		if (g_convexFlags[i] == 0)
		{
			// static
			glColor3f(1.0f, 1.0f, 1.0f);
		}
		else
		{
			// dynamic
			glColor3f(0.25f, 1.0f, 0.25f);
		}

		for (uint32_t j=0; j < builder.mIndices.size(); j+=3)
		{
			uint32_t a = builder.mIndices[j+0];
			uint32_t b = builder.mIndices[j+1];
			uint32_t c = builder.mIndices[j+2];

			Vec3 n = Normalize(Cross(builder.mVertices[b]-builder.mVertices[a], builder.mVertices[c]-builder.mVertices[a]));

			glNormal3fv(n);
			glVertex3fv(builder.mVertices[a]);
			glVertex3fv(builder.mVertices[b]);
			glVertex3fv(builder.mVertices[c]);
		}

		glEnd();

		//glColor3f(1.0f, 0.0f, 0.0f);
		//DrawBoundingBox(Vec3(g_convexAabbMin[i]), Vec3(g_convexAabbMax[i]));
	}
}

#if _WIN32
void APIENTRY glErrorCallback( GLenum _source, 
 GLenum _type, GLuint _id, GLenum _severity, 
 GLsizei _length, const char* _message, 
 void* _userParam) 
{ 
 printf("%s\n", _message); 
} 
#endif

void ErrorCallback(const char* msg, const char* file, int line)
{
	printf("Flex: %s - %s:%d\n", msg, file, line);
	assert(0);
}


int main(int argc, char* argv[])
{		
	bool fullscreen = false;

	// process command line args
	for (int i=1; i < argc; ++i)
	{
		int d;
		if (sscanf(argv[i], "-device=%d", &d))
			g_cudaDevice = d;
		
		if (sscanf(argv[i], "-msaa=%d", &d))
			g_msaa = d != 0;

		int w=1280;
		int h=720;
		if (sscanf(argv[i], "-fullscreen=%dx%d", &w, &h) == 2)
		{
			g_screenWidth = w;
			g_screenHeight = h;

			fullscreen = true;
		}
		else if (strstr(argv[i], "-fullscreen"))
		{
			g_screenHeight = -1;
			g_screenWidth = -1;
			fullscreen = true;
		}
		else if (sscanf(argv[i], "-multiplier=%d", &d) == 1)
		{
			g_numExtraMultiplier = d;
		}
	}
	
	// set cuda device
	cudaSetDevice(g_cudaDevice);

	// retrieve device name
	cudaDeviceProp prop;
	cudaCheck(cudaGetDeviceProperties(&prop, g_cudaDevice));
	memcpy(g_device, prop.name, 256);	

#if 0
	// disabled tests
	g_scenes.push_back(new Lighthouse("Lighthouse"));
	g_scenes.push_back(new RayleighTaylor2D("Rayleigh-Taylor2D"));	
	g_scenes.push_back(new RayleighTaylor3D("Rayleigh-Taylor3D"));		
	g_scenes.push_back(new GranularShape("Granular Dragon"));
	g_scenes.push_back(new ThinBox("Thin Box"));
	
	g_scenes.push_back(new SmokeOpen("Smoke Dirichlet"));
	g_scenes.push_back(new SmokeCloth("Smoke Cloth"));
	g_scenes.push_back(new SmokeClosed("Smoke Neumann"));
	g_scenes.push_back(new GranularShape("Granular Dragon"));
	g_scenes.push_back(new Deformables("Deformables"));
	g_scenes.push_back(new ThinBox("Thin Box"));
	g_scenes.push_back(new ClothCCD("Cloth CCD"));
	g_scenes.push_back(new NonConvex("NonConvex"));	

	g_scenes.push_back(new BunnyBath("Bunny Bath", false));
	g_scenes.push_back(new ArmadilloShower("Armadillo Water Shower", false));
	g_scenes.push_back(new ArmadilloShower("Armadillo Goo Shower", true));

	g_scenes.push_back(new Darts("Darts"));
	g_scenes.push_back(new Restitution("Restitution"));	

	g_scenes.push_back(new Player("../../data/capture.bin"));	

#endif	



	// populate scenes list
	/*g_scenes.push_back(new MixedPile("Mixed Pile"));
	g_scenes.push_back(new FrictionRamp("Friction Ramp"));
	g_scenes.push_back(new FrictionMovingShape("Friction Moving"));
	
	g_scenes.push_back(new EnvironmentalCloth("Env Cloth Small", 6, 6, 40, 16));
	g_scenes.push_back(new EnvironmentalCloth("Env Cloth Large", 16, 32, 10, 3));

	g_scenes.push_back(new Viscosity("Viscosity Low", 0.5f));
	g_scenes.push_back(new Viscosity("Viscosity Med", 3.0f));
	g_scenes.push_back(new Viscosity("Viscosity High", 5.0f, 0.12f));
	g_scenes.push_back(new Adhesion("Adhesion"));

	g_scenes.push_back(new Inflatable("Inflatables"));
	g_scenes.push_back(new LowDimensionalShapes("Low Dimensional Shapes"));
	g_scenes.push_back(new ClothLayers("Cloth Layers"));
	
	g_scenes.push_back(new Buoyancy("Buoyancy"));
	g_scenes.push_back(new Melting("Melting"));
	g_scenes.push_back(new SurfaceTension("Surface Tension Low",  0.0f));
	g_scenes.push_back(new SurfaceTension("Surface Tension Med", 10.0f));
	g_scenes.push_back(new SurfaceTension("Surface Tension High", 20.0f));
	g_scenes.push_back(new GooGun("Goo Gun", true));
	g_scenes.push_back(new WaterBalloon("Water Balloons"));
	g_scenes.push_back(new TriangleMesh("Triangle Mesh"));	
	g_scenes.push_back(new Tearing("Tearing"));
	g_scenes.push_back(new PotPourri("PotPourri"));
	g_scenes.push_back(new DamBreak("DamBreak  5cm", 0.05f));
	g_scenes.push_back(new DamBreak("DamBreak 10cm", 0.1f));
	g_scenes.push_back(new DamBreak("DamBreak 15cm", 0.15f));	
	g_scenes.push_back(new Pasta("Pasta"));
	g_scenes.push_back(new BananaPile("Bananas"));
	g_scenes.push_back(new RockPool("Rock Pool"));
	g_scenes.push_back(new GameMesh("Game Mesh Fluid Rigid", 0));
	g_scenes.push_back(new GameMesh("Game Mesh Particles", 1));
	g_scenes.push_back(new GameMesh("Game Mesh Fluid", 2));	
	g_scenes.push_back(new GameMesh("Game Mesh Cloth", 3));
	g_scenes.push_back(new ParachutingBunnies("Parachuting Bunnies"));
	g_scenes.push_back(new RigidPile("Rigid2", 2));
	g_scenes.push_back(new RigidPile("Rigid4", 4));
	g_scenes.push_back(new RigidPile("Rigid8", 12));
	g_scenes.push_back(new FlagCloth("Flag Cloth"));
	g_scenes.push_back(new GranularPile("Granular Pile"));	
	g_scenes.push_back(new RigidFluidCoupling("Rigid Fluid Coupling"));
	g_scenes.push_back(new FluidBlock("Fluid Block"));
	g_scenes.push_back(new PlasticBunnies("Plastic Bunnies"));
	g_scenes.push_back(new PlasticStack("Plastic Stack"));
	g_scenes.push_back(new FluidClothCoupling("Fluid Cloth Coupling Water", false));
	g_scenes.push_back(new FluidClothCoupling("Fluid Cloth Coupling Goo", true));
	g_scenes.push_back(new BunnyBath("Bunny Bath Dam", true));*/

	g_scenes.push_back(new FluidClothCoupling("Fluid Cloth Coupling Water", false));

    // init gl
    glutInit(&argc, argv);

	if (g_msaa)
		glEnable(GL_MULTISAMPLE);
		
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(g_screenWidth, g_screenHeight);
	
	if (fullscreen)
	{
		char gameString[255];
		sprintf(gameString, "%dx%d:32@60", g_screenWidth, g_screenHeight);

		glutGameModeString(gameString);

		if (glutGameModeGet(GLUT_GAME_MODE_POSSIBLE))
			glutEnterGameMode();
		else
		{
			fullscreen = false;
		}
	}

	if (!fullscreen)
	{
		glutCreateWindow("Flex Demo");
		glutPositionWindow(200, 100);
	}

	glewInit();
	FlexError err = flexInit(FLEX_VERSION, ErrorCallback);

	if (err != eFlexErrorNone)
	{
		printf("Error (%d), could not initialize flex\n", err);
		exit(-1);
	}

	if (fullscreen)
		GLUTReshape(g_screenWidth, g_screenHeight);

	imguiRenderGLInit("../../data/DroidSans.ttf");

#if 0
    if(glDebugMessageCallback)
	{
        glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
        glDebugMessageCallback(glErrorCallback, NULL);
        GLuint unusedIds = 0;
        glDebugMessageControl(GL_DONT_CARE,
            GL_DONT_CARE,
            GL_DONT_CARE,
            0,
            &unusedIds,
            true);
    }
#endif

	// create shadow maps
	ShadowCreate(g_shadowTex, g_shadowBuf);

	// init default scene
	Init(g_scene);

    glutMouseFunc(GLUTMouseFunc);
    glutReshapeFunc(GLUTReshape);
    glutDisplayFunc(GLUTUpdate);
    glutKeyboardFunc(GLUTKeyboardDown);
    glutKeyboardUpFunc(GLUTKeyboardUp);
    glutIdleFunc(GLUTUpdate);
    glutSpecialFunc(GLUTArrowKeys);
    glutSpecialUpFunc(GLUTArrowKeysUp);
    glutMotionFunc(GLUTMotionFunc);
	glutEntryFunc(GLUTEntryFunc);
	glutPassiveMotionFunc(GLUTPassiveMotionFunc);

    glutMainLoop();

}

