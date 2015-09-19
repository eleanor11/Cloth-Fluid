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

class Scene
{
public:

	Scene(const char* name) : mName(name) {}
	
	virtual void Initialize() = 0;
	virtual void PostInitialize() {}
	virtual void Update() {}
	virtual void Draw(int pass) {}
	virtual void KeyDown(int key) {}
	virtual void DoGui() {}
	virtual Matrix44 GetBasis() { return Matrix44::kIdentity; }

	virtual const char* GetName() { return mName; }

	const char* mName;
};


class RigidPile : public Scene
{
public:

	RigidPile(const char* name, int brickHeight) : Scene(name), mHeight(brickHeight)
	{
	}
	
	virtual void Initialize()
	{
		int sx = 2;
		int sy = mHeight;
		int sz = 2;

		Vec3 lower(0.0f, 1.5f + g_params.mRadius*0.25f, 0.0f);

		int dimx = 10;
		int dimy = 10;
		int dimz = 10;

		float radius = g_params.mRadius;

		if (1)
		{
			// create a basic grid
			for (int y=0; y < dimy; ++y)
				for (int z=0; z < dimz; ++z)
					for (int x=0; x < dimx; ++x)
						CreateParticleShape(
						"../../data/box.ply", 
						(g_params.mRadius*0.905f)*Vec3(float(x*sx), float(y*sy), float(z*sz)) + (g_params.mRadius*0.1f)*Vec3(float(x),float(y),float(z)) + lower,
						g_params.mRadius*0.9f*Vec3(float(sx), float(sy), float(sz)), 0.0f, g_params.mRadius*0.9f, Vec3(0.0f), 1.0f, true, 1.0f, flexMakePhase(g_rigidOffsets.size()+1, 0), true, 0.002f);// 0.002f);
		

			// plinth
			CreateConvex();

		}
		else
		{
			// brick work
			int wdimx = 10;
			int wdimy = 10;

			int bdimx = 4;
			int bdimy = 2;
			int bdimz = 2;

			for (int y=0; y < wdimy; ++y)
			{
				for (int x=0; x < wdimx; ++x)
				{
					Vec3 lower = Vec3(x*bdimx*radius + 0.5f*radius, 0.93f*bdimy*y*radius, 0.0f);

					if (y&1)
						lower += Vec3(bdimx*0.25f*radius + 0.5f*radius, 0.0f, 0.0f);

					//CreateParticleGrid(lower, bdimx, bdimy, bdimz, radius, Vec3(0.0f), 1.0f, true, g_rigidOffsets.size()+1, 0.0f);
					CreateParticleShape("../../data/box.ply", lower + RandomUnitVector()*Vec3(0.0f, 0.0f, 0.0f), Vec3(bdimx*radius, bdimy*radius, bdimz*radius), 0.0f, radius, Vec3(0.0f), 1.0f, true, 1.0f, flexMakePhase(g_rigidOffsets.size()+1, 0), true, 0.0f, Vec3(0.0f, 0.0f, y*0.0001f));
				}
			}
			
			if (0)
			{
				// create a basic grid
				for (int y=0; y < dimy; ++y)
					for (int z=0; z < 1; ++z)
						for (int x=0; x < 1; ++x)
							CreateParticleShape(
							"../../data/box.ply", 
							0.99f*(g_params.mRadius)*Vec3(float(x*sx), float(y*sy), float(z*sz)),
							g_params.mRadius*Vec3(float(sx), float(sy), float(sz)), 0.0f, g_params.mRadius, Vec3(0.0f), 1.0f, true, 1.0f, flexMakePhase(g_rigidOffsets.size()+1, 0), true, 0.0f);

				// create a basic grid
				for (int y=0; y < dimy; ++y)
					for (int z=0; z < 1; ++z)
						for (int x=0; x < 1; ++x)
							CreateParticleShape(
							"../../data/box.ply", 
							0.99f*(g_params.mRadius)*Vec3(float(sx*2 + x*sx), float(y*sy), float(z*sz)),
							g_params.mRadius*Vec3(float(sx), float(sy), float(sz)), 0.0f, g_params.mRadius, Vec3(0.0f), 1.0f, true, 1.0f, flexMakePhase(g_rigidOffsets.size()+1, 0), true, 0.0f);
			}

		}
		

		if (0)
		{
			float stretchStiffness = 1.0f;
			float bendStiffness = 0.5f;
			float shearStiffness = 0.7f;

			int dimx = 40;
			int dimy = 40;

			CreateSpringGrid(Vec3(-1.0f, 1.0f + g_params.mRadius*0.5f, -1.0f), dimx, dimy, 1, g_params.mRadius*0.9f, flexMakePhase(0, eFlexPhaseSelfCollide), stretchStiffness, bendStiffness, shearStiffness, Vec3(0.0f), 1.0f);
		}


		//g_numExtraParticles = 32*1024;		
		g_numSubsteps = 2;
		g_params.mNumIterations = 4;

		g_params.mRadius *= 1.0f;
		g_params.mDynamicFriction = 0.4f;
		g_params.mDissipation = 0.01f;
		g_params.mParticleCollisionMargin = g_params.mRadius*0.05f;
		g_params.mSleepThreshold = g_params.mRadius*0.25f;
		g_params.mShockPropagation = 3.f;
		
		g_windStrength = 0.0f;

		// draw options
		g_drawPoints = false;

		g_emitters[0].mEnabled = true;
		g_emitters[0].mSpeed = (g_params.mRadius*2.0f/g_dt);
	}

	virtual void Update()
	{
			
	}

	int mHeight;
};




class MixedPile : public Scene
{
public:

	MixedPile(const char* name) : Scene(name)
	{
	}
	

	std::vector<ClothMesh*> mCloths;
	std::vector<float> mRestVolume;
	std::vector<int> mTriOffset;
	std::vector<int> mTriCount;
	std::vector<float> mOverPressure;
	std::vector<float> mConstraintScale;
	std::vector<float> mSplitThreshold;

	void AddInflatable(const Mesh* mesh, float overPressure, int phase, float invmass=1.0f)
	{
		const int startVertex = g_positions.size();

		// add mesh to system
		for (size_t i=0; i < mesh->GetNumVertices(); ++i)
		{
			const Vec3 p = Vec3(mesh->m_positions[i]);
				
			g_positions.push_back(Vec4(p.x, p.y, p.z, invmass));
			g_velocities.push_back(0.0f);
			g_phases.push_back(phase);
		}

		int triOffset = g_triangles.size();
		int triCount = mesh->GetNumFaces();

		mTriOffset.push_back(triOffset/3);
		mTriCount.push_back(mesh->GetNumFaces());
		mOverPressure.push_back(overPressure);

		for (size_t i=0; i < mesh->m_indices.size(); i+=3)
		{
			int a = mesh->m_indices[i+0];
			int b = mesh->m_indices[i+1];
			int c = mesh->m_indices[i+2];

			Vec3 n = -Normalize(Cross(mesh->m_positions[b]-mesh->m_positions[a], mesh->m_positions[c]-mesh->m_positions[a]));
			g_triangleNormals.push_back(n);

			g_triangles.push_back(a + startVertex);
			g_triangles.push_back(b + startVertex);
			g_triangles.push_back(c + startVertex);
		}

		// create a cloth mesh using the global positions / indices
		ClothMesh* cloth = new ClothMesh(&g_positions[0], g_positions.size(), &g_triangles[triOffset],triCount*3, 0.8f, 1.0f);

		for (size_t i=0; i < cloth->mConstraintIndices.size(); ++i)
			g_springIndices.push_back(cloth->mConstraintIndices[i]);

		g_springStiffness.insert(g_springStiffness.end(), cloth->mConstraintCoefficients.begin(), cloth->mConstraintCoefficients.end());
		g_springLengths.insert(g_springLengths.end(), cloth->mConstraintRestLengths.begin(), cloth->mConstraintRestLengths.end());

		mCloths.push_back(cloth);

		// add inflatable params
		mRestVolume.push_back(cloth->mRestVolume);
		mConstraintScale.push_back(cloth->mConstraintScale);
	}


	virtual void Initialize()
	{
		
		Vec3 start(0.0f, 0.5f + g_params.mRadius*0.25f, 0.0f);

		float radius = g_params.mRadius;

		int group = 1;
		
		if (1)
		{
			mCloths.resize(0);
			mRestVolume.resize(0);
			mTriOffset.resize(0);
			mTriCount.resize(0);
			mOverPressure.resize(0);
			mConstraintScale.resize(0);
			mSplitThreshold.resize(0);

			Vec3 lower(0.0f), upper(0.0f);
			float size = 1.0f + radius;

			for (int i=0; i < 9; ++i)
				{
				Mesh* mesh = ImportMesh("../../data/sphere.ply");
				mesh->Normalize();
				mesh->Transform(TranslationMatrix(Point3(lower.x + i%3*size, upper.y + 2.0f, (upper.z+lower.z)*0.5f + i/3*size)));
				
				AddInflatable(mesh, 1.0f, flexMakePhase(group++, 0), 2.0f);
				delete mesh;
			}
		}


		if (1)
		{
			const int minSize[3] = { 2, 1, 3 };
			const int maxSize[3] = { 4, 3, 6 };

			Vec4 color = Vec4(SrgbToLinear(Colour(Vec4(201.0f, 158.0f, 106.0f, 255.0f)/255.0f)));

			Vec3 lower(0.0f), upper(5.0f);
			GetParticleBounds(lower,upper);

			int dimx = 3;
			int dimy = 10;
			int dimz = 3;

			for (int y=0; y < dimy; ++y)
			{
				for (int z=0; z < dimz; ++z)
				{
					for (int x=0; x < dimx; ++x)
					{
						CreateParticleShape(
						"../../data/box.ply", 					
						Vec3(x + 0.5f,0,z+ 0.5f)*(1.0f+radius) + Vec3(0.0f, upper.y + (y+2.0f)*maxSize[1]*g_params.mRadius, 0.0f),
						Vec3(float(Rand(minSize[0], maxSize[0])),
							 float(Rand(minSize[1], maxSize[1])), 
							 float(Rand(minSize[2], maxSize[2])))*g_params.mRadius*0.9f, 0.0f, g_params.mRadius*0.9f, Vec3(0.0f), 1.0f, true, 1.0f, flexMakePhase(group++, 0), true,0.0f,0.0f, 0.0f, color);
					}
				}
			}
		}		

		
		if (1)
		{
			Vec3 lower, upper;
			GetParticleBounds(lower,upper);
			Vec3 center = (upper+lower)*0.5f;
			center.y = upper.y;
		
			for (int i=0; i < 20; ++i)
			{
				Rope r;
				Vec3 offset = Vec3(sinf(k2Pi*float(i)/20), 0.0f, cosf(k2Pi*float(i)/20));

				CreateRope(r, center + offset, Normalize(offset + Vec3(0.0f, 4.0f, 0.0f)), 1.2f, 50, 50*radius, flexMakePhase(group++, eFlexPhaseSelfCollide), 0.0f, 10.0f, 0.0f);
				g_ropes.push_back(r);
			}
		}

		Vec3 lower, upper;
		GetParticleBounds(lower, upper);

		Vec3 center = (lower+upper)*0.5f;
		center.y = 0.0f;

		float width = (upper-lower).x;
		float edge = 0.25f;
		float height = 1.0f;
		CreateConvex(Vec3(edge, height, width), center + Vec3(-width, height/2.0f, 0.0f));
		CreateConvex(Vec3(edge, height, width), center + Vec3(width, height/2.0f, 0.0f));

		CreateConvex(Vec3(width-edge, height, edge), center + Vec3(0.0f, height/2.0f, width-edge));
		CreateConvex(Vec3(width-edge, height, edge), center + Vec3(0.0f, height/2.0f, -(width-edge)));
	
		//g_numExtraParticles = 32*1024;		
		g_numSubsteps = 2;
		g_params.mNumIterations = 7;

		g_params.mRadius *= 1.0f;
		g_params.mSolidRestDistance = g_params.mRadius;
		g_params.mFluidRestDistance = g_params.mRadius*0.55f;
		g_params.mDynamicFriction = 0.6f;
		g_params.mStaticFriction = 0.75f;
		g_params.mParticleFriction = 0.3f;
		g_params.mDissipation = 0.0f;
		g_params.mParticleCollisionMargin = g_params.mRadius*0.05f;
		g_params.mSleepThreshold = g_params.mRadius*0.125f;
		g_params.mShockPropagation = 0.0f;
		g_params.mRestitution = 0.0f;
		g_params.mCollisionDistance = g_params.mRadius*0.5f;
		g_params.mFluid = false;		
	
		// separte solid particle count
		g_numSolidParticles = g_positions.size();
		// number of fluid particles to allocate
		g_numExtraParticles = 32*1024;		
		
		g_params.mNumPlanes = 1;
		g_windStrength = 0.0f;

		g_lightDistance *= 0.5f;

		// draw options
		g_drawPoints = true;
		g_expandCloth = g_params.mRadius*0.5f;
		
		g_emitters[0].mEnabled = true;
		g_emitters[0].mSpeed = (g_params.mRadius*0.5f/g_dt);
		g_emitters[0].mSpeed = (g_params.mRadius/g_dt);

		extern Colour gColors[];
		gColors[0] = Colour(0.805f, 0.702f, 0.401f);		
	}

	virtual void Update()
	{
		flexSetInflatables(g_flex, &mTriOffset[0], &mTriCount[0], &mRestVolume[0], &mOverPressure[0], &mConstraintScale[0], mCloths.size(), eFlexMemoryHost);
	}
	
	int mHeight;
};

class PotPourri : public Scene
{
public:

	PotPourri(const char* name) : Scene(name)
	{
	}
	
	virtual void Initialize()
	{
		int sx = 2;
		int sy = 2;
		int sz = 2;

		Vec3 lower(0.0f, 4.2f + g_params.mRadius*0.25f, 0.0f);

		int dimx = 5;
		int dimy = 10;
		int dimz = 5;

		float radius = g_params.mRadius;
		int group = 0;

		if (1)
		{
			// create a basic grid
			for (int y=0; y < dimy; ++y)
				for (int z=0; z < dimz; ++z)
					for (int x=0; x < dimx; ++x)
						CreateParticleShape(
						"../../data/box.ply", 
						(g_params.mRadius*0.905f)*Vec3(float(x*sx), float(y*sy), float(z*sz)) + (g_params.mRadius*0.1f)*Vec3(float(x),float(y),float(z)) + lower,
						g_params.mRadius*0.9f*Vec3(float(sx), float(sy), float(sz)), 0.0f, g_params.mRadius*0.9f, Vec3(0.0f), 1.0f, true, 1.0f, flexMakePhase(group++, 0), true, 0.001f);

			// plinth
			CreateConvex();

		}

		if (1)
		{
			int dimx = 60;
			int dimy = 40;

			float stretchStiffness = 1.0f;
			float bendStiffness = 0.5f;
			float shearStiffness = 0.7f;

			int clothStart = g_positions.size();
		
			CreateSpringGrid(Vec3(0.0f, 0.0f, -1.0f), dimx, dimy, 1, radius*0.5f, flexMakePhase(group++, 0), stretchStiffness, bendStiffness, shearStiffness, Vec3(0.0f), 1.0f);
		
			int corner0 = clothStart + 0;
			int corner1 = clothStart + dimx-1;
			int corner2 = clothStart + dimx*(dimy-1);
			int corner3 = clothStart + dimx*dimy-1;

			g_positions[corner0].w = 0.0f;
			g_positions[corner1].w = 0.0f;
			g_positions[corner2].w = 0.0f;
			g_positions[corner3].w = 0.0f;

			// add tethers
			for (int i=clothStart; i < int(g_positions.size()); ++i)
			{
				float x = g_positions[i].x;
				g_positions[i].y = 4.0f - sinf(DegToRad(15.0f))*x;
				g_positions[i].x = cosf(DegToRad(25.0f))*x;

				if (i != corner0 && i != corner1 && i != corner2 && i != corner3)
				{
					float stiffness = -0.5f;
					float give = 0.05f;

					CreateSpring(corner0, i, stiffness, give);			
					CreateSpring(corner1, i, stiffness, give);
					CreateSpring(corner2, i, stiffness, give);			
					CreateSpring(corner3, i, stiffness, give);
				}
			}

			g_positions[corner1] = g_positions[corner0] + (g_positions[corner1]-g_positions[corner0])*0.9f;
			g_positions[corner2] = g_positions[corner0] + (g_positions[corner2]-g_positions[corner0])*0.9f;
			g_positions[corner3] = g_positions[corner0] + (g_positions[corner3]-g_positions[corner0])*0.9f;
		}


		for (int i=0; i < 50; ++i)
			CreateParticleShape("../../data/banana.obj", Vec3(0.4f, 8.5f + i*0.25f, 0.25f) + RandomUnitVector()*radius*0.25f, Vec3(1), 0.0f, radius, Vec3(0.0f), 1.0f, true, 0.5f, flexMakePhase(group++, 0), true, radius*0.1f, 0.0f, 0.0f, 1.25f*Vec4(0.875f, 0.782f, 0.051f, 1.0f));		
		

		//g_numExtraParticles = 32*1024;		
		g_numSubsteps = 2;
		g_params.mNumIterations = 4;

		g_params.mRadius *= 1.0f;
		g_params.mStaticFriction = 0.7f;
		g_params.mDynamicFriction = 0.75f;
		g_params.mDissipation = 0.01f;
		g_params.mParticleCollisionMargin = g_params.mRadius*0.05f;
		g_params.mSleepThreshold = g_params.mRadius*0.25f;	
		g_params.mDamping = 0.25f;

		g_windStrength = 0.0f;

		// draw options
		g_drawPoints = false;

		g_emitters[0].mEnabled = true;
		g_emitters[0].mSpeed = (g_params.mRadius*2.0f/g_dt);
	}

	virtual void Update()
	{
			
	}
};

class ClothLayers : public Scene
{
public:

	ClothLayers(const char* name) :
	  Scene(name) {}

	virtual void Initialize()
	{	
	
		float stretchStiffness = 1.0f;
		float bendStiffness = 0.8f;
		float shearStiffness = 0.5f;

		int dimx = 64;
		int dimz = 64;
		float radius = 0.05f;
		int phase = flexMakePhase(0, eFlexPhaseSelfCollide);

		CreateSpringGrid(Vec3(-0.6f, 2.9f, -0.6f), dimx, dimz, 1, radius, phase, stretchStiffness, bendStiffness, shearStiffness, 0.0f, 1.0f);
		CreateSpringGrid(Vec3(-0.6f, 2.6f, -0.6f), dimx, dimz, 1, radius, phase, stretchStiffness, bendStiffness, shearStiffness, 0.0f, 1.0f);
		CreateSpringGrid(Vec3(-0.6f, 2.3f, -0.6f), dimx, dimz, 1, radius, phase, stretchStiffness, bendStiffness, shearStiffness, 0.0f, 1.0f);
		CreateSpringGrid(Vec3(-0.6f, 2.0f, -0.6f), dimx, dimz, 1, radius, phase, stretchStiffness, bendStiffness, shearStiffness, 0.0f, 1.0f);

		Vec3 lower, upper;
		GetParticleBounds(lower, upper);

		g_staticMesh = ImportMesh("../../data/sphere.ply");
		g_staticMesh->Normalize(2.0f);
		g_staticMesh->CalculateNormals();		

		g_params.mRadius = radius*1.0f;
		g_params.mDynamicFriction = 0.1625f;
		g_params.mDissipation = 0.0f;
		g_params.mNumIterations = 8;
		g_params.mViscosity = 0.0f;
		g_params.mDrag = 0.05f;
		g_params.mCollisionDistance = radius;
		g_params.mRelaxationFactor = 1.3f;

		g_numSubsteps = 3;


		g_windStrength = 0.0f;

		// draw options
		g_drawPoints = true;
		g_drawSprings = false;
	}
};


class FrictionMovingShape: public Scene
{
public:

	FrictionMovingShape(const char* name) :
	  Scene(name) {}

	virtual void Initialize()
	{	
		float stretchStiffness = 0.5f;
		float bendStiffness = 0.5f;
		float shearStiffness = 0.5f;

		float radius = 0.05f;

		int dimx = 20;
		int dimz = 20;
		int phase = flexMakePhase(0, eFlexPhaseSelfCollide);

		for (int i=0; i < 3; ++i)
			CreateSpringGrid(Vec3(-dimx*radius*0.5f, 1.5f + i*0.2f, -dimz*radius*0.5f), dimx, dimz, 1, radius, phase, stretchStiffness, bendStiffness, shearStiffness, 0.0f, 1.0f);

		g_params.mRadius = radius*1.0f;
		g_params.mDynamicFriction = 0.25f;
		g_params.mParticleFriction = 0.25f;
		g_params.mDissipation = 0.0f;
		g_params.mNumIterations = 8;
		g_params.mViscosity = 0.0f;
		g_params.mDrag = 0.05f;
		g_params.mCollisionDistance = radius;
		g_params.mRelaxationMode = eFlexRelaxationGlobal;
		g_params.mRelaxationFactor = 0.25f;
		g_params.mNumPlanes = 1;

		g_numSubsteps = 2;

		g_windStrength = 0.0f;

		// draw options
		g_drawPoints = false;
		g_drawSprings = false;

		mTime = 0.0f;
	}

	void Update()
	{
		
		g_convexAabbMax.resize(0);
		g_convexAabbMin.resize(0);
		g_convexLengths.resize(0);
		g_convexPlanes.resize(0);
		g_convexPositions.resize(0);
		g_convexRotations.resize(0);
		g_convexPrevPositions.resize(0);
		g_convexPrevRotations.resize(0);
		g_convexStarts.resize(0);
		g_convexFlags.resize(0);

		float lastTime = mTime;
		mTime += g_dt;

		CreateConvex(Vec3(1.0f, 1.0f, 1.0f), Vec3(sinf(mTime), 0.5f, 0.0f), QuatFromAxisAngle(Vec3(0.0f, 1.0f, 0.0f), mTime), 0);
		
		g_convexPrevPositions[0] = Vec4(sinf(lastTime), 0.5f, 0.0f, 0.0f);
		g_convexPrevRotations[0] = QuatFromAxisAngle(Vec3(0.0f, 1.0f, 0.0f), lastTime);

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
	float mTime;
};

class EnvironmentalCloth: public Scene
{
public:

	EnvironmentalCloth(const char* name, int dimx, int dimz, int gridx, int gridz) :
	  Scene(name), 
	  mDimX(dimx),
	  mDimZ(dimz),
	  mGridX(gridx),
	  mGridZ(gridz) {}

	virtual void Initialize()
	{	
		float scale = 1.0f;

		float minSize = 0.5f*scale;
		float maxSize = 1.0f*scale;

		for (int i=0; i < 5; i++)
			CreateRandomConvex(10, Vec3(i*2.0f, 0.0f, Randf(0.0f, 2.0f)), minSize, maxSize, Vec3(0.0f, 1.0f, 0.0f), Randf(0.0f, k2Pi));
		
		float stretchStiffness = 0.9f;
		float bendStiffness = 0.8f;
		float shearStiffness = 0.5f;

		int dimx = mDimX;
		int dimz = mDimZ;
		float radius = 0.05f*scale;
		g_params.mGravity[1] *= scale;

		int gridx = mGridX;
		int gridz = mGridZ;

		int clothIndex = 0;
		int phase = flexMakePhase(0, eFlexPhaseSelfCollide);

		for (int x=0; x < gridx; ++x)
		{
			for (int y=0; y < 1; ++y)
			{
				for (int z=0; z < gridz; ++z)
				{
					clothIndex++;

					CreateSpringGrid(Vec3(x*dimx*radius, scale*(1.0f + z*0.5f), z*dimx*radius), dimx, dimz, 1, radius, phase, stretchStiffness, bendStiffness, shearStiffness, Vec3(Randf(-0.2f, 0.2f)), 1.0f);
				}
			}
		}

		g_params.mRadius = radius*1.05f;
		g_params.mDynamicFriction = 0.25f;
		g_params.mDissipation = 0.0f;
		g_params.mNumIterations = 8;
		g_params.mViscosity = 0.0f;
		g_params.mDrag = 0.1f;
		g_params.mLift = 0.5f;
		g_params.mCollisionDistance = 0.05f;
		
		// cloth converges faster with a global relaxation factor
		g_params.mRelaxationMode = eFlexRelaxationGlobal;
		g_params.mRelaxationFactor = 0.25f;

		g_windStrength = 0.0f;
		g_numSubsteps = 2;

		// draw options
		g_drawPoints = false;
		g_drawSprings = false;
	}
	
	int mDimX;
	int mDimZ;
	int mGridX;
	int mGridZ;
};

class FlagCloth: public Scene
{
public:

	FlagCloth(const char* name) : Scene(name) {}

	void Initialize()
	{
		int dimx = 64;
		int dimz = 32;
		float radius = 0.05f;

		float stretchStiffness = 0.9f;
		float bendStiffness = 1.0f;
		float shearStiffness = 0.9f;
		int phase = flexMakePhase(0, eFlexPhaseSelfCollide);
	
		CreateSpringGrid(Vec3(0.0f, 0.0f, -3.0f), dimx, dimz, 1, radius, phase, stretchStiffness, bendStiffness, shearStiffness, 0.0f, 1.0f);

		const int c1 = 0;
		const int c2 = dimx*(dimz-1);

		g_positions[c1].w = 0.0f;
		g_positions[c2].w = 0.0f;

		// add tethers
		for (int i=0; i < int(g_positions.size()); ++i)
		{
			// hack to rotate cloth
			swap(g_positions[i].y, g_positions[i].z);
			g_positions[i].y *= -1.0f;

			g_velocities[i] = RandomUnitVector()*0.1f;

			float minSqrDist = FLT_MAX;

			if (i != c1 && i != c2)
			{
				float stiffness = -0.8f;
				float give = 0.1f;

				float sqrDist = LengthSq(Vec3(g_positions[c1])-Vec3(g_positions[c2]));

				if (sqrDist < minSqrDist)
				{
					CreateSpring(c1, i, stiffness, give);
					CreateSpring(c2, i, stiffness, give);

					minSqrDist = sqrDist;
				}
			}
		}
		
		g_params.mRadius = radius*1.0f;
		g_params.mDynamicFriction = 0.25f;
		g_params.mDissipation = 0.0f;
		g_params.mNumIterations = 4;
		g_params.mDrag = 0.06f;
		g_params.mRelaxationFactor = 1.0f;

		g_numSubsteps = 2;

		// draw options
		g_drawPoints = false;
		g_drawSprings = false;
		g_windFrequency *= 2.0f;
		g_windStrength = 10.0f;

	}

	void Update()
	{
		const Vec3 kWindDir = Vec3(3.0f, 15.0f, 0.0f);
		const float kNoise = fabsf(Perlin1D(g_windTime*0.05f, 2, 0.25f));
		Vec3 wind = g_windStrength*kWindDir*Vec3(kNoise, kNoise*0.1f, -kNoise*0.1f);
				
		g_params.mWind[0] = wind.x;
		g_params.mWind[1] = wind.y;
		g_params.mWind[2] = wind.z;	
	}
};


class RigidFluidCoupling : public Scene
{
public:

	RigidFluidCoupling(const char* name) : Scene(name) {}

	virtual void Initialize()
	{
		float minSize = 0.5f;
		float maxSize = 1.0f;
		
		float radius = 0.1f;
		int group = 0;

		Randf();

		for (int i=0; i < 5; i++)
			CreateRandomConvex(10, Vec3(i*2.0f, 0.0f, Randf(0.0f, 2.0f)), minSize, maxSize, Vec3(0.0f, 1.0f, 0.0f), Randf(0.0f, k2Pi*10.0f));

		for (int z=0; z < 10; ++z)
			for (int x=0; x < 50; ++x)
				CreateParticleShape(
					"../../data/box.ply", 
					Vec3(x*radius*2 - 1.0f, 1.0f + radius, 1.f + z*2.0f*radius) + 0.5f*Vec3(Randf(radius), 0.0f, Randf(radius)), 
					Vec3(2.0f, 2.0f + Randf(0.0f, 4.0f), 2.0f)*radius*0.5f, 
					0.0f,
					radius*0.5f, 
					Vec3(0.0f), 
					1.0f, 
					true,
					1.0f,
					flexMakePhase(group++, 0),
					true,
					0.0f);

	
		// separte solid particle count
		g_numSolidParticles = g_positions.size();

		// number of fluid particles to allocate
		g_numExtraParticles = 64*1024;

		g_params.mRadius = radius;
		g_params.mDynamicFriction = 0.5f;
		g_params.mFluid = true;
		g_params.mViscosity = 0.1f;		
		g_params.mNumIterations = 3;
		g_params.mVorticityConfinement = 0.0f;
		g_params.mAnisotropyScale = 25.0f;
		g_params.mFluidRestDistance = g_params.mRadius*0.55f;		
		

		g_emitters[0].mEnabled = true;	
		g_emitters[0].mSpeed = 2.0f*(g_params.mFluidRestDistance)/g_dt;

		// draw options
		g_drawPoints = false;
		g_drawEllipsoids = true;
	}
};

class FluidBlock : public Scene
{
public:

	FluidBlock(const char* name) : Scene(name) {}

	virtual void Initialize()
	{
		float minSize = 0.5f;
		float maxSize = 0.7f;
		
		float radius = 0.1f;
		float restDistance = radius*0.55f;
		int group = 0;

		CreateRandomConvex(6, Vec3(5.0f, -0.1f, 0.6f), 1.0f, 1.0f, Vec3(1.0f, 1.0f, 0.0f), 0.0f);
		g_convexPlanes[3] = Vec4(Normalize(Vec3(-1.0f, 1.0f, 0.0f)), -1.0f);
		g_convexPlanes[2].w *= 2.85f;
		g_convexAabbMin[0].x -= 3.0f; // hack 		

		float ly = 0.5f;
		
		CreateRandomConvex(10, Vec3(2.5f, ly*0.5f, 1.f), minSize*0.5f, maxSize*0.5f, Vec3(0.0f, 1.0f, 0.0f), Randf(0.0f, 2.0f*kPi));

		CreateRandomConvex(12, Vec3(3.8f, ly-0.5f, 1.f), minSize, maxSize, Vec3(1.0f, 0.0f, 0.0f), Randf(0.0f, 2.0f*kPi));
		CreateRandomConvex(12, Vec3(3.8f, ly-0.5f, 2.6f), minSize, maxSize, Vec3(1.0f, 0.0f, 0.0f), 0.2f + Randf(0.0f, 2.0f*kPi));

		CreateRandomConvex(12, Vec3(4.6f, ly, 0.2f), minSize, maxSize, Vec3(1.0f, 0.0f, 1.0f), Randf(0.0f, 2.0f*kPi));
		CreateRandomConvex(12, Vec3(4.6f, ly, 2.0f), minSize, maxSize, Vec3(1.0f, 0.0f, 1.0f), 0.2f + Randf(0.0f, 2.0f*kPi));
	
		const char* mesh = "../../data/torus.obj";

		float size = 0.3f;
		for (int i=0; i < 32; ++i)
			CreateParticleShape(mesh, Vec3(4.5f, 2.0f + radius*2.0f*i, 1.0f), size, 0.0f, radius*0.5f, Vec3(0.0f, 0.0f, 0.0f), 0.125f, true, 1.0f, flexMakePhase(group++, 0), true, 0.0f);			

		g_numSolidParticles = g_positions.size();	

		float sizex = 1.76f;
		float sizey = 2.20f;
		float sizez = 3.50f;

		int x = int(sizex/restDistance);
		int y = int(sizey/restDistance);
		int z = int(sizez/restDistance);

		CreateParticleGrid(Vec3(0.0f, restDistance*0.5f, 0.0f), x, y, z, restDistance, Vec3(0.0f), 1.0f, false, 0.0f, flexMakePhase(group++, eFlexPhaseSelfCollide | eFlexPhaseFluid));		

		g_params.mRadius = radius;
		g_params.mDynamicFriction = 0.0f;
		g_params.mFluid = true;
		g_params.mViscosity = 0.0f;
		g_params.mNumIterations = 3;
		g_params.mVorticityConfinement = 40.f;
		g_params.mAnisotropyScale = 20.0f;
		g_params.mFluidRestDistance = restDistance;
		g_params.mNumPlanes = 5;		
		//g_params.mCohesion = 0.05f;

		g_maxDiffuseParticles = 128*1024;
		g_diffuseScale = 0.75f;

		g_waveFloorTilt = -0.025f; 
		
		g_lightDistance *= 0.5f;

		// draw options
		g_drawDensity = true;
		g_drawDiffuse = true;
		g_drawEllipsoids = true;
		g_drawPoints = false;
	}
};


class Buoyancy : public Scene
{
public:

	Buoyancy(const char* name) : Scene(name) {}

	virtual void Initialize()
	{	
		float radius = 0.1f;
		float restDistance = radius*0.5f;
		int group = 0;

		const char* mesh = "../../data/sphere.ply";

		int n = 3;
		float spacing = 64*restDistance*0.9f/(2.0f*n);
		float sampling = restDistance*0.8f;
		Vec3 size = sampling*12.0f;

		const float mass[] = {1.0f, 0.25f, 0.005f };

		for (int j=0; j < 1; ++j)
			for (int i=0; i < n; ++i)
				CreateParticleShape(mesh, Vec3(spacing - 0.5f*size.x + i*spacing*2, 2.0f + j*size.y*1.2f, 0.6f), size, 0.0f, sampling, Vec3(0.0f, 0.0f, 0.0f), mass[i], true, 1.0f, flexMakePhase(group++, 0), true, 0.0001f);			
				
		g_numSolidParticles = g_positions.size();	

		int x = 64;
		int y = 20;
		int z = 32;

		CreateParticleGrid(Vec3(0.0f), x, y, z, restDistance*0.9f, Vec3(0.0f), 1.0f, false, 0.0f, flexMakePhase(group++, eFlexPhaseSelfCollide | eFlexPhaseFluid));

		g_params.mRadius = radius;
		g_params.mDynamicFriction = 0.1f;
		g_params.mFluid = true;
		g_params.mViscosity = 2.0f;
		g_params.mNumIterations = 4;
		g_params.mVorticityConfinement = 180.f;
		g_params.mAnisotropyScale = 20.0f;
		g_params.mFluidRestDistance = restDistance;
		g_params.mNumPlanes = 5;
		g_params.mCohesion = 0.001625f;
		g_params.mCollisionDistance = restDistance;
		g_params.mSolidPressure = 0.1f;
		g_params.mRestitution = 0.0f;
		g_params.mRelaxationFactor = 1.0f;
		g_params.mRelaxationMode = eFlexRelaxationLocal;

		g_numSubsteps = 2;

		g_maxDiffuseParticles = 64*1024;
		g_diffuseScale = 0.25f;		
		g_diffuseShadow = false;
		g_diffuseColor = 1.5f;
		g_diffuseMotionScale = 1.5f;
		g_params.mDiffuseBallistic = 35;
		g_params.mDiffuseThreshold *= 0.1f;
		
		g_drawPlaneBias = g_params.mCollisionDistance*1.5f;

		g_lightDistance *= 0.65f;
		
		g_fluidColor = Vec4(0.2f, 0.6f, 0.9f, 1.0f);

		// draw options
		g_drawDensity = true;
		g_drawDiffuse = true;
		g_drawEllipsoids = true;
		g_drawPoints = false;

		g_warmup = true;
	}

};

class LowDimensionalShapes: public Scene
{
public:

	LowDimensionalShapes(const char* name) : Scene(name) {}

	virtual void Initialize()
	{
		float radius = 0.1f;
		int group = 0;

		const char* mesh = "../../data/box.ply";

		CreateParticleShape(mesh, Vec3(0.0f, 1.0f, 0.0f), Vec3(1.2f, 0.001f, 1.2f), 0.0f, radius, Vec3(0.0f, 0.0f, 0.0f), 1.0f, true, 1.0f, flexMakePhase(group++, 0), true, 0.0f);			

		for (int i=0; i < 64; ++i)
			CreateParticleShape(mesh, Vec3(i/8*radius, 0.0f, i%8*radius), Vec3(0.1f, 0.8f, 0.1f), 0.0f, radius, Vec3(0.0f, 0.0f, 0.0f), 1.0f, true, 1.0f, flexMakePhase(group++, 0), true, 0.0f);			
				
		g_params.mRadius = radius;
		g_params.mDynamicFriction = 1.0f;
		g_params.mFluid = false;
		g_params.mFluidRestDistance = radius;
		g_params.mViscosity = 0.0f;
		g_params.mNumIterations = 4;
		g_params.mVorticityConfinement = 0.f;
		g_params.mAnisotropyScale = 20.0f;
		g_params.mNumPlanes = 1;
		g_params.mCollisionDistance = radius*0.5f;
		g_params.mShockPropagation = 5.0f;
				
		g_numSubsteps = 2;

		g_maxDiffuseParticles = 0;
		g_diffuseScale = 0.75f;

		g_lightDistance *= 1.5f;
		
		g_fluidColor = Vec4(0.2f, 0.6f, 0.9f, 1.0f);

		// draw options
		g_drawDensity = false;
		g_drawDiffuse = false;
		g_drawEllipsoids = false;
		g_drawPoints = true;
		g_drawMesh = false;

		g_warmup = false;

	}
		
};


class ArmadilloShower : public Scene
{
public:

	ArmadilloShower(const char* name, bool viscous) : Scene(name), mViscous(viscous) {}

	virtual void Initialize()
	{
		float minSize = 0.5f;
		float maxSize = 1.0f;
		
		float radius = 0.1f;

		for (int i=0; i < 5; i++)
			CreateRandomConvex(10, Vec3(i*2.0f, 0.0f, Randf(0.0f, 2.0f)), minSize, maxSize, Vec3(0.0f, 1.0f, 0.0f), Randf(0.0f, k2Pi*10.0f));

		g_params.mRadius = radius;
		
		g_params.mFluid = true;	
		g_params.mNumIterations = 3;
		g_params.mVorticityConfinement = 0.0f;
		g_params.mFluidRestDistance = g_params.mRadius*0.5f;
		g_params.mAnisotropyScale = 20.0f;

		if (mViscous)
		{
			g_fluidColor = Vec4(0.0f, 0.8f, 0.2f, 1.0f);

			g_params.mDynamicFriction = 0.5f;
			g_params.mViscosity = 10.85f;
			g_params.mCohesion = 0.25f;
		}
		else
		{
			g_params.mDynamicFriction = 0.025f;
			g_params.mViscosity = 0.05f;
		}

		g_numExtraParticles = 64*1024;

		g_diffuseScale = 1.0f;

		CreateSDF("../../data/armadillo.ply", 2.0f, Vec3(2.0f, 0.0f, -1.0f));

		Vec3 meshLower, meshUpper;
		g_mesh->GetBounds(meshLower, meshUpper);

		Emitter e1;
		e1.mDir = Vec3(-1.0f, 0.0f, 0.0f);
		e1.mRight = Vec3(0.0f, 0.0f, 1.0f);
		e1.mPos = Vec3(-1.0f, 0.15f, 0.50f) +  meshLower + Vec3(meshUpper.x, meshUpper.y*0.75f, meshUpper.z*0.5f);
		e1.mSpeed = (g_params.mRadius*0.5f/g_dt)*2.0f;	// 2 particle layers per-frame
		e1.mEnabled = true;

		Emitter e2;
		e2.mDir = Vec3(1.0f, 0.0f, 0.0f);
		e2.mRight = Vec3(0.0f, 0.0f, -1.0f);
		e2.mPos = Vec3(-1.0f, 0.15f, 0.50f) + meshLower + Vec3(0.0f, meshUpper.y*0.75f, meshUpper.z*0.5f);
		e2.mSpeed = (g_params.mRadius*0.5f/g_dt)*2.0f; // 2 particle layers per-frame
		e2.mEnabled = true;

		g_emitters.push_back(e1);
		g_emitters.push_back(e2);

		g_emit = true;

		// draw options		
		g_drawEllipsoids = true;
	}

	bool mViscous;
};

class PlasticBunnies : public Scene
{
public:

	PlasticBunnies(const char* name) : Scene(name) {}

	virtual void Initialize()
	{
		g_params.mRadius = 0.225f;
		
		g_params.mNumIterations = 2;
		g_params.mDynamicFriction = 0.5f;
		g_params.mStaticFriction = 0.5f;
		g_params.mDissipation = 0.0f;
		g_params.mViscosity = 0.0f;	// TODO: fix viscosity / particle friction in rigid particle case

		const float spacing = g_params.mRadius*0.49f;

		const char* mesh = "../../data/bunny.ply";

		float size = 1.0f;
		int group = 0;

		for (int i=0; i < 16; ++i)
			CreateParticleShape(mesh, Vec3(0.0f, 3.0f + size*i*1.1f, 0.0f), 1.0, 0.0f, spacing, Vec3(0.0f, 0.0f, 0.0f), 1.0f, true, 1.0f, flexMakePhase(group++, 0), true, 0.0f);

		g_rigidLocalNormals.resize(0);

		g_params.mPlasticThreshold = 0.0015f;
		g_params.mPlasticCreep = 0.125f;

			// plinth
		CreateConvex(2.0f);
		g_convexPositions[0] += Vec4(0.0f, 1.0f, 0.0f, 0.0f);

		g_numSubsteps = 2;

		g_lightDistance *= 0.35f;
		// draw options		
		g_drawPoints = false;
	}

};

class PlasticStack : public Scene
{
public:

	PlasticStack(const char* name) : Scene(name) {}
	
	virtual void Initialize()
	{
		g_params.mRadius = 0.225f;
		
		g_params.mNumIterations = 2;
		g_params.mDynamicFriction = 0.5f;
		g_params.mDissipation = 0.0f;
		g_params.mViscosity = 0.0f;

		// plinth
		CreateConvex(2.0f);
		g_convexPositions[0] += Vec4(0.0f, 1.0f, 0.0f, 0.0f);

		const float rotation = -kPi*0.5f;
		const float spacing = g_params.mRadius*0.5f;

		// alternative box and sphere shapes
		const char* mesh[] = 
		{ 
			"../../data/box_high.ply",
			"../../data/sphere.ply" 
		};

		Vec3 lower = Vec3(4.0f, 1.0f, 0.0f);
		float sizeInc = 0.0f;
		float size = 1.0f;
		int group = 0;

		for (int i=0; i < 8; ++i)
		{
			CreateParticleShape(mesh[i%2], lower, size + i*sizeInc, rotation, spacing, Vec3(.0f, 0.0f, 0.0f), 1.0f, true, 1.0f, flexMakePhase(group++, 0), true, 0.0f, 0.0f, g_params.mRadius*0.5f);

			lower += Vec3(0.0f, size + i*sizeInc + 0.2f, 0.0f);
		}

		g_params.mPlasticThreshold = 0.00025f;
		g_params.mPlasticCreep = 0.125f;

		g_numSubsteps = 4;

		// draw options		
		g_drawPoints = false;
	}
};

class ParachutingBunnies : public Scene
{
public:

	ParachutingBunnies(const char* name) : Scene(name) {}

	void Initialize()
	{
		float stretchStiffness = 1.0f;
		float bendStiffness = 0.8f;
		float shearStiffness = 0.8f;

		int dimx = 32;
		int dimy = 32;
		float radius = 0.055f;

		float height = 10.0f;
		float spacing = 1.5f;
		int numBunnies = 2;
		int group = 0;

		for (int i=0; i < numBunnies; ++i)
		{
			CreateSpringGrid(Vec3(i*dimx*radius, height + i*spacing, 0.0f), dimx, dimy, 1, radius, flexMakePhase(group++, eFlexPhaseSelfCollide), stretchStiffness, bendStiffness, shearStiffness, Vec3(0.0f), 1.1f);

			const int startIndex = i*dimx*dimy;

			int corner0 = startIndex + 0;
			int corner1 = startIndex + dimx-1;
			int corner2 = startIndex + dimx*(dimy-1);
			int corner3 = startIndex + dimx*dimy-1;

			CreateSpring(corner0, corner1, 1.f,-0.1f);
			CreateSpring(corner1, corner3, 1.f,-0.1f);
			CreateSpring(corner3, corner2, 1.f,-0.1f);
			CreateSpring(corner0, corner2, 1.f,-0.1f);
		}

		for (int i=0; i < numBunnies; ++i)
		{		
			Vec3 velocity = RandomUnitVector()*1.0f;
			float size = radius*8.5f;

			CreateParticleShape("../../data/bunny.ply", Vec3(i*dimx*radius + radius*0.5f*dimx - 0.5f*size, height + i*spacing-0.5f, radius*0.5f*dimy - 0.5f), size, 0.0f, radius, velocity, 0.15f, true, 1.0f, flexMakePhase(group++, 0), true, 0.0f);			

			const int startIndex = i*dimx*dimy;
			const int attachIndex = g_positions.size()-1;
			g_positions[attachIndex].w = 2.0f;

			int corner0 = startIndex + 0;
			int corner1 = startIndex + dimx-1;
			int corner2 = startIndex + dimx*(dimy-1);
			int corner3 = startIndex + dimx*dimy-1;

			Vec3 attachPosition = (Vec3(g_positions[corner0]) + Vec3(g_positions[corner1]) + Vec3(g_positions[corner2]) + Vec3(g_positions[corner3]))*0.25f;
			attachPosition.y = height + i*spacing-0.5f;

			if (1)
			{
				int c[4] = {corner0, corner1, corner2, corner3};

				for (int i=0; i < 4; ++i)
				{
					Rope r;

					int start = g_positions.size();
	
					r.mIndices.push_back(attachIndex);

					Vec3 d0 = Vec3(g_positions[c[i]])-attachPosition;
					CreateRope(r, attachPosition, Normalize(d0), 1.2f, int(Length(d0)/radius*1.1f), Length(d0), flexMakePhase(group++, 0), 0.0f, 0.5f, 0.0f);

					r.mIndices.push_back(c[i]);
					g_ropes.push_back(r);

					int end = g_positions.size()-1;
					

					CreateSpring(attachIndex, start, 1.2f, -0.5f);
					CreateSpring(c[i], end, 1.0f);
				}
			}
		}

		if (1)
		{
			// falling objects
			Vec3 lower, upper;
			GetParticleBounds(lower, upper);

			Vec3 center = (lower+upper)*0.5f;
			center.y = 0.0f;

			float width = (upper-lower).x*0.5f;
			float edge = 0.125f;
			float height = 0.5f;
		
			// big blocks
			for (int i=0; i < 3; ++i)
				CreateParticleShape("../../data/box.ply", center + Vec3(float(i)-1.0f, 5.0f, 0.0f), radius*9, 0.0f, radius*0.9f, Vec3(0.0f), 0.5f, true, 1.0f, flexMakePhase(group++, 0), true, 0.0f, 0.0f, -radius*1.5f);		

			// small blocks			
			for (int j=0; j < 2; ++j)
				for (int i=0; i < 8; ++i)
					CreateParticleShape("../../data/box.ply", Vec3(lower.x + 0.5f, 0.0f, lower.z - 0.5f) + Vec3(float(i/3), 6.0f + float(j), float(i%3)) + RandomUnitVector()*0.5f, radius*4, 0.0f, radius*0.9f, Vec3(0.0f), 1.f, true, 1.0f, flexMakePhase(group++, 0), true, 0.0f, 0.0f, -radius*2.0f);			

			g_numSolidParticles = g_positions.size();

			{
				CreateConvex(Vec3(edge, height, width+edge*2.0f), center + Vec3(-width - edge, height/2.0f, 0.0f));
				CreateConvex(Vec3(edge, height, width+edge*2.0f), center + Vec3(width + edge, height/2.0f, 0.0f));

				CreateConvex(Vec3(width+2.0f*edge, height, edge), center + Vec3(0.0f, height/2.0f, -(width+edge)));
				CreateConvex(Vec3(width+2.0f*edge, height, edge), center + Vec3(0.0f, height/2.0f, width+edge));

				float fluidWidth = width;
				float fluidHeight = height*1.25f;

				int particleWidth = int(2.0f*fluidWidth/radius);
				int particleHeight = int(fluidHeight/radius); 

				CreateParticleGrid(center - Vec3(fluidWidth, 0.0f, fluidWidth), particleWidth, particleHeight, particleWidth, radius, Vec3(0.0f), 2.0f, false, 0.0f, flexMakePhase(group++, eFlexPhaseSelfCollide | eFlexPhaseFluid));
			}
		}

		g_params.mFluid = true;
		g_params.mRadius = 0.1f;
		g_params.mFluidRestDistance = radius;
		g_params.mNumIterations = 4;
		g_params.mViscosity = 1.0f;
		g_params.mDynamicFriction = 0.05f;
		g_params.mStaticFriction = 0.0f;
		g_params.mParticleCollisionMargin = 0.0f;
		g_params.mCollisionDistance = g_params.mFluidRestDistance*0.5f;
		g_params.mVorticityConfinement = 120.0f;
		g_params.mCohesion = 0.0025f;
		g_params.mDrag = 0.06f;
		g_params.mLift = 0.f;
		g_params.mSolidPressure = 0.0f;
		g_params.mAnisotropyScale = 22.0f;
		g_params.mSmoothing = 1.0f;
		g_params.mRelaxationFactor = 1.0f;
		
		g_maxDiffuseParticles = 64*1024;
		g_diffuseScale = 0.25f;		
		g_diffuseShadow = false;
		g_diffuseColor = 2.5f;
		g_diffuseMotionScale = 1.5f;
		g_params.mDiffuseThreshold *= 0.01f;
		g_params.mDiffuseBallistic = 35;

		g_windStrength = 0.0f;
		g_windFrequency = 0.0f;

		g_numSubsteps = 2;

		// draw options		
		g_drawEllipsoids = true;
		g_drawPoints = false;
		g_drawDiffuse = true;
		g_drawSprings = 0;

		g_ropeScale = 0.2f;
		g_warmup = false;
	}
};

class FluidClothCoupling : public Scene			//triangle && without thickness
{
public:

	FluidClothCoupling(const char* name, bool viscous, int opt = 0) : Scene(name), mViscous(viscous), option(opt) {}

	void Initialize()
	{
		is_hollow = false;
		is_cube = false;
		is_complex = false;
		is_point = false;

		sceneNum = 0;

		g_emitterWidth = 3;

		g_kAbsorption = 1.0;
		g_kMaxAbsorption = 0.3;
		g_kDiffusion = 0.3;
		g_kDiffusionGravity = 0.2;

		g_absorb = true;
		g_diffuse = true;
		g_drip = true;

		g_camInit = false;
		g_camPos = Vec3(0.7f, 1.7f, 2.9f);
		g_camAngle = Vec3(0.0f, -0.4f, 0.0f);


		float stretchStiffness = 1.0f;
		float bendStiffness = 0.4f;
		float shearStiffness = 0.4f;

		int dimx = 32;
		int dimy = 32;
		float radius = 0.1f;
		float invmass = 0.25f;
		int group = 0;

		g_dripBuffer.resize(0);
		g_saturations.resize(0);
		g_triangleCenters.resize(0);
		g_triangleNeighbours.resize(0);
		g_thetas.resize(0);


		{
			g_dx = dimx;
			g_dy = dimy;
			g_dz = 1;

			g_numTriangles = (dimx - 1) * (dimy - 1) * 2;

			int clothStart = 0;
		
			//CreateSpringGrid(Vec3(0.0f, 1.0f, 0.0f), dimx, dimy, 1, radius*0.25f, flexMakePhase(group++, 0), stretchStiffness, bendStiffness, shearStiffness, Vec3(0.0f), invmass);
			if (option == 1){
				CreateSpringGridVertical(Vec3(0.3f, 1.0f, 0.0f), dimx, dimy, 1, radius*0.25f, flexMakePhase(group++, 0), stretchStiffness, bendStiffness, shearStiffness, Vec3(0.0f), invmass);
			}
			else {
				CreateSpringGrid(Vec3(0.3f, 1.0f, 0.0f), dimx, dimy, 1, radius*0.25f, flexMakePhase(group++, 0), stretchStiffness, bendStiffness, shearStiffness, Vec3(0.0f), invmass);
			}

			int corner0 = clothStart + 0;
			int corner1 = clothStart + dimx-1;
			int corner2 = clothStart + dimx*(dimy-1);
			int corner3 = clothStart + dimx*dimy-1;

			g_positions[corner0].w = 0.0f;
			g_positions[corner1].w = 0.0f;
			g_positions[corner2].w = 0.0f;
			g_positions[corner3].w = 0.0f;

			if (option == 1){
				g_positions[corner0].w = 1.0f;
				g_positions[corner2].w = 1.0f;
			}

			if (option == 2){
				float move = 0.05f;
				g_positions[corner0].x = g_positions[corner0].x + move;
				g_positions[corner0].y = g_positions[corner0].y + move;
				g_positions[corner0].z = g_positions[corner0].z + move;
				g_positions[corner1].x = g_positions[corner1].x - move;
				g_positions[corner1].y = g_positions[corner1].y + move;
				g_positions[corner1].z = g_positions[corner1].z + move;
				g_positions[corner2].x = g_positions[corner2].x + move;
				g_positions[corner2].y = g_positions[corner2].y + move;
				g_positions[corner2].z = g_positions[corner2].z - move;
				g_positions[corner3].x = g_positions[corner3].x - move;
				g_positions[corner3].y = g_positions[corner3].y + move;
				g_positions[corner3].z = g_positions[corner3].z - move;
			}

			// add tethers
			for (int i=clothStart; i < int(g_positions.size()); ++i)
			{
				/*float x = g_positions[i].x;
				g_positions[i].y = 1.5f - sinf(DegToRad(25.0f))*x;
				g_positions[i].x = cosf(DegToRad(25.0f))*x;*/

				//g_positions[i].y += 0.5f-g_positions[i].x;

				if (i != corner0 && i != corner1 && i != corner2 && i != corner3)
				{
					float stiffness = -0.5f;
					float give = 0.05f;

					if (option == 0 || option == 2){
						CreateSpring(corner0, i, stiffness, give);
						CreateSpring(corner1, i, stiffness, give);
						CreateSpring(corner2, i, stiffness, give);
						CreateSpring(corner3, i, stiffness, give);
					}
					else if (option == 1){
						CreateSpring(corner1, i, stiffness, give);
						CreateSpring(corner3, i, stiffness, give);
					}
				}
			}

			/*g_positions[corner1] = g_positions[corner0] + (g_positions[corner1]-g_positions[corner0])*0.9f;
			g_positions[corner2] = g_positions[corner0] + (g_positions[corner2]-g_positions[corner0])*0.9f;
			g_positions[corner3] = g_positions[corner0] + (g_positions[corner3]-g_positions[corner0])*0.9f;
		*/
		}

/*		{
			// net
			int clothStart = g_positions.size();
		
			CreateSpringGrid(Vec3(0.75f, 1.0f, 0.0f), dimx, dimy, 1, radius*0.25f, flexMakePhase(group++, 0), stretchStiffness, bendStiffness, shearStiffness, Vec3(0.0f), invmass);
		
			int corner0 = clothStart + 0;
			int corner1 = clothStart + dimx-1;
			int corner2 = clothStart + dimx*(dimy-1);
			int corner3 = clothStart + dimx*dimy-1;

			g_positions[corner0].w = 0.0f;
			g_positions[corner1].w = 0.0f;
			g_positions[corner2].w = 0.0f;
			g_positions[corner3].w = 0.0f;

			// add tethers
			for (int i=clothStart; i < int(g_positions.size()); ++i)
			{
				if (i != corner0 && i != corner1 && i != corner2 && i != corner3)
				{
					float stiffness = -0.5f;
					float give = 0.1f;

					CreateSpring(corner0, i, stiffness, give);			
					CreateSpring(corner1, i, stiffness, give);
					CreateSpring(corner2, i, stiffness, give);			
					CreateSpring(corner3, i, stiffness, give);
				}
			}

			g_positions[corner1] = g_positions[corner0] + (g_positions[corner1]-g_positions[corner0])*0.8f;
			g_positions[corner2] = g_positions[corner0] + (g_positions[corner2]-g_positions[corner0])*0.8f;
			g_positions[corner3] = g_positions[corner0] + (g_positions[corner3]-g_positions[corner0])*0.8f;

		}

		{
			// net
			int clothStart = g_positions.size();
		
			CreateSpringGrid(Vec3(1.5f, 0.5f, 0.0f), dimx, dimy, 1, radius*0.25f, flexMakePhase(group++, 0), stretchStiffness, bendStiffness, shearStiffness, Vec3(0.0f), invmass);
		
			int corner0 = clothStart + 0;
			int corner1 = clothStart + dimx-1;
			int corner2 = clothStart + dimx*(dimy-1);
			int corner3 = clothStart + dimx*dimy-1;

			g_positions[corner0].w = 0.0f;
			g_positions[corner1].w = 0.0f;
			g_positions[corner2].w = 0.0f;
			g_positions[corner3].w = 0.0f;

			// add tethers
			for (int i=clothStart; i < int(g_positions.size()); ++i)
			{
				if (i != corner0 && i != corner1 && i != corner2 && i != corner3)
				{
					float stiffness = -0.5f;
					float give = 0.1f;

					CreateSpring(corner0, i, stiffness, give);			
					CreateSpring(corner1, i, stiffness, give);
					CreateSpring(corner2, i, stiffness, give);			
					CreateSpring(corner3, i, stiffness, give);
				}
			}

			g_positions[corner1] = g_positions[corner0] + (g_positions[corner1]-g_positions[corner0])*0.8f;
			g_positions[corner2] = g_positions[corner0] + (g_positions[corner2]-g_positions[corner0])*0.8f;
			g_positions[corner3] = g_positions[corner0] + (g_positions[corner3]-g_positions[corner0])*0.8f;

		}
*/

		g_numSolidParticles = g_positions.size();
		g_ior = 1.0f;

		g_numExtraParticles = 64*1024;

		g_params.mRadius = radius;
		g_params.mFluid = true;
		g_params.mNumIterations = 5;
		g_params.mVorticityConfinement = 0.0f;
		g_params.mAnisotropyScale = 30.0f;
		g_params.mFluidRestDistance = g_params.mRadius*0.5f;
		g_params.mSmoothing = 0.5f;
		g_params.mSolidPressure = 0.25f;
		g_numSubsteps = 3;		
		//g_params.mNumIterations = 6;

		g_params.mMaxVelocity = 0.5f*g_numSubsteps*g_params.mRadius/g_dt;

		g_maxDiffuseParticles = 32*1024;
		g_diffuseScale = 0.5f;
		g_lightDistance = 3.0f;
		g_pointScale = 0.5f;

		// for viscous goo
		if (mViscous)
		{
			g_fluidColor = Vec4(0.0f, 0.8f, 0.2f, 1.0f);

			g_params.mDynamicFriction = 0.3f;
			g_params.mCohesion = 0.025f;
			g_params.mViscosity = 50.85f;			
		}
		else
		{
			g_params.mDynamicFriction = 0.125f;
			g_params.mViscosity = 0.1f;
			g_params.mCohesion = 0.0035f;
			g_params.mViscosity = 4.0f;
		}

		g_emitters[0].mEnabled = false;

		Emitter e;
		e.mDir = Normalize(Vec3(1.0f, 0.0f, 0.0f));
		e.mEnabled = true;
		e.mPos = Vec3(-0.25f, 1.75f, 0.5f);
		e.mRight = Cross(e.mDir, Vec3(0.0f, 0.0f, 1.0f));
		e.mSpeed = (g_params.mFluidRestDistance/(g_dt*2.0f));

		g_emitters.push_back(e);


		// draw options		
		g_drawPoints = false;
		g_drawSprings = false;
		g_drawEllipsoids = true;


	}

	virtual void DoGui(){

		imguiLabel("Scene");

		if (imguiCheck("Dyeing", bool(g_absorb != 0 && g_diffuse != 0 && g_drip != 0))){
			if (g_absorb && g_diffuse && g_drip){
				g_absorb = false;
				g_diffuse = false;
				g_drip = false;
			}
			else {
				g_absorb = true;
				g_diffuse = true;
				g_drip = true;
			}
		}

		if (imguiCheck("Absorbing", bool(g_absorb != 0)))
			g_absorb = !g_absorb;
		if (imguiCheck("Diffusing", bool(g_diffuse != 0)))
			g_diffuse = !g_diffuse;
		if (imguiCheck("Dripping", bool(g_drip != 0)))
			g_drip = !g_drip;
		if (imguiCheck("Mark", bool(g_markColor != 0)))
			g_markColor = !g_markColor;

		imguiSlider("k Absorption", &g_kAbsorption, 0.0, 1.0, 0.1);
		imguiSlider("k Diffusion", &g_kDiffusion, 0.0, 1.0, 0.1);
		imguiSlider("k Diffusion Gravity", &g_kDiffusionGravity, 0.0, 1.0, 0.1);

		imguiSeparatorLine();
	}

	bool mViscous;
	int option;
};

class FluidClothCoupling2 : public Scene
{
public:

	FluidClothCoupling2(const char* name, bool viscous, int option = 0) : Scene(name), mViscous(viscous), option(option) {}

	void Initialize()
	{
		sceneNum = 2;

		is_hollow = true;
		is_cube = false;
		is_complex = false;
		is_point = false;

		g_absorb = true;
		g_diffuse = true;
		g_drip = true;
		g_markColor = false;

		g_emitterWidth = 5;

		g_kAbsorption = 1.0;
		g_kMaxAbsorption = 0.3;
		g_kDiffusion = 0.3;
		g_kDiffusionGravity = 0.2;

		g_camInit = false;
		g_camPos = Vec3(0.7f, 1.7f, 2.9f);
		g_camAngle = Vec3(0.0f, -0.4f, 0.0f);


		float stretchStiffness = 1.0f;
		float bendStiffness = 0.4f;
		float shearStiffness = 0.4f;

		int dimx = 32;
		int dimy = 32;
		int dimz = 6;
		float radius = 0.1f;
		float invmass = 0.25f;
		int group = 0;

		if (option == 1){
			g_emitterWidth = 3;
			dimx = 1;
			dimy = 32;
			dimz = 32;
		}

		g_dripBuffer.resize(0);
		g_saturations.resize(0);
		g_triangleCenters.resize(0);
		g_triangleNeighbours.resize(0);
		g_thetas.resize(0);
		g_pointTriangleNums.resize(dimx * dimy * dimz);
		g_pointTriangles.resize(dimx * dimy * dimz * 2);
		g_trianglePoints.resize(0);

		{
			g_dx = dimx;
			g_dy = dimy;
			g_dz = dimz;

			g_numTriangles = (dimx - 1) * (dimy - 1) + (dimx - 1) * (dimz - 1) + (dimy - 1) * (dimz - 1);
			g_numTriangles = g_numTriangles * 2 * 2;
			g_numPoints = dimx * dimy * dimz;

			int clothStart = 0;

			//CreateSpringGrid(Vec3(0.0f, 1.0f, 0.0f), dimx, dimy, 1, radius*0.25f, flexMakePhase(group++, 0), stretchStiffness, bendStiffness, shearStiffness, Vec3(0.0f), invmass);
			CreateSpringGrid2(Vec3(0.3f, 1.0f, 0.0f), dimx, dimy, dimz, radius*0.25f, flexMakePhase(group++, 0), stretchStiffness, bendStiffness, shearStiffness, Vec3(0.0f), invmass);

			CalculateTriangleNeighbours();

			int corner0 = clothStart + 0 + (dimz - 1) * dimx * dimy;
			int corner1 = clothStart + dimx - 1 + (dimz - 1) * dimx * dimy;
			int corner2 = clothStart + dimx*(dimy - 1) + (dimz - 1) * dimx * dimy;
			int corner3 = clothStart + dimx*dimy - 1 + (dimz - 1) * dimx * dimy;
			int corner4 = clothStart + 0;
			int corner5 = clothStart + dimx - 1;
			int corner6 = clothStart + dimx*(dimy - 1);
			int corner7 = clothStart + dimx*dimy - 1;

			g_positions[corner0].w = 0.0f;
			g_positions[corner1].w = 0.0f;
			g_positions[corner2].w = 0.0f;
			g_positions[corner3].w = 0.0f;
			g_positions[corner4].w = 0.0f;
			g_positions[corner5].w = 0.0f;
			g_positions[corner6].w = 0.0f;
			g_positions[corner7].w = 0.0f;

			// add tethers
			for (int i = clothStart; i < int(g_positions.size()); ++i)
			{

				if (i != corner0 && i != corner1 && i != corner2 && i != corner3)
				{
					float stiffness = -0.5f;
					float give = 0.05f;

					CreateSpring(corner0, i, stiffness, give);
					CreateSpring(corner1, i, stiffness, give);
					CreateSpring(corner2, i, stiffness, give);
					CreateSpring(corner3, i, stiffness, give);
					CreateSpring(corner4, i, stiffness, give);
					CreateSpring(corner5, i, stiffness, give);
					CreateSpring(corner6, i, stiffness, give);
					CreateSpring(corner7, i, stiffness, give);
				}
			}

		}

		g_numSolidParticles = g_positions.size();
		g_ior = 1.0f;

		g_numExtraParticles = 64 * 1024;

		g_params.mRadius = radius;
		g_params.mFluid = true;
		g_params.mNumIterations = 5;
		g_params.mVorticityConfinement = 0.0f;
		g_params.mAnisotropyScale = 30.0f;
		g_params.mFluidRestDistance = g_params.mRadius*0.5f;
		g_params.mSmoothing = 0.5f;
		g_params.mSolidPressure = 0.25f;
		g_numSubsteps = 3;
		//g_params.mNumIterations = 6;

		g_params.mMaxVelocity = 0.5f*g_numSubsteps*g_params.mRadius / g_dt;

		g_maxDiffuseParticles = 32 * 1024;
		g_diffuseScale = 0.5f;
		g_lightDistance = 3.0f;
		g_pointScale = 0.5f;

		// for viscous goo
		if (mViscous)
		{
			g_fluidColor = Vec4(0.0f, 0.8f, 0.2f, 1.0f);

			g_params.mDynamicFriction = 0.3f;
			g_params.mCohesion = 0.025f;
			g_params.mViscosity = 50.85f;
		}
		else
		{
			g_params.mDynamicFriction = 0.125f;
			g_params.mViscosity = 0.1f;
			g_params.mCohesion = 0.0035f;
			g_params.mViscosity = 4.0f;
		}

		g_emitters[0].mEnabled = false;

		Emitter e;
		e.mDir = Normalize(Vec3(1.0f, 0.0f, 0.0f));
		e.mEnabled = true;
		e.mPos = Vec3(-0.25f, 1.75f, 0.5f);
		e.mRight = Cross(e.mDir, Vec3(0.0f, 0.0f, 1.0f));
		e.mSpeed = (g_params.mFluidRestDistance / (g_dt*2.0f));

		if (option == 1){
			e.mPos = Vec3(0.0f, 1.5f, 0.5f);
		}

		g_emitters.push_back(e);

		// draw options		
		g_drawPoints = false;
		g_drawSprings = false;
		g_drawEllipsoids = true;

	}

	virtual void DoGui(){

		imguiLabel("Scene");

		if (imguiCheck("Dyeing", bool(g_absorb != 0 && g_diffuse != 0 && g_drip != 0))){
			if (g_absorb && g_diffuse && g_drip){
				g_absorb = false;
				g_diffuse = false;
				g_drip = false;
			}
			else {
				g_absorb = true;
				g_diffuse = true;
				g_drip = true;
			}
		}

		if (imguiCheck("Absorbing", bool(g_absorb != 0)))
			g_absorb = !g_absorb;
		if (imguiCheck("Diffusing", bool(g_diffuse != 0)))
			g_diffuse = !g_diffuse;
		if (imguiCheck("Dripping", bool(g_drip != 0)))
			g_drip = !g_drip;
		if (imguiCheck("Mark", bool(g_markColor != 0)))
			g_markColor = !g_markColor;

		imguiSlider("k Absorption", &g_kAbsorption, 0.0, 1.0, 0.1);
		imguiSlider("k Diffusion", &g_kDiffusion, 0.0, 1.0, 0.1);
		imguiSlider("k Diffusion Gravity", &g_kDiffusionGravity, 0.0, 1.0, 0.1);

		imguiSeparatorLine();
	}


	bool mViscous;
	int option = option;
};
class FluidClothCoupling3 : public Scene			//triangle && thickness
{
public:

	FluidClothCoupling3(const char* name, bool viscous) : Scene(name), mViscous(viscous) {}

	void Initialize()
	{
		is_hollow = true;
		is_cube = false;
		is_complex = false;
		is_point = false;

		sceneNum = 0;

		g_absorb = true;
		g_diffuse = true;
		g_drip = true;
		g_markColor = false;

		g_emitterWidth = 5;

		g_kAbsorption = 1.0;
		g_kMaxAbsorption = 0.5;
		g_kDiffusion = 0.3;
		g_kDiffusionGravity = 0.2;

		g_camInit = false;
		g_camPos = Vec3(0.7f, 1.7f, 2.9f);
		g_camAngle = Vec3(0.0f, -0.4f, 0.0f);


		float stretchStiffness = 1.0f;
		float bendStiffness = 0.4f;
		float shearStiffness = 0.4f;

		int dimx = 32;
		int dimy = 32;
		int dimz = 6;
		float radius = 0.1f;
		float invmass = 0.25f;
		int group = 0;

		g_dripBuffer.resize(0);
		g_saturations.resize(0);
		g_triangleCenters.resize(0);
		g_triangleNeighbours.resize(0);
		g_thetas.resize(0);


		{
			g_dx = dimx;
			g_dy = dimy;
			g_dz = dimz;

			g_numTriangles = (dimx - 1) * (dimy - 1) + (dimx - 1) * (dimz - 1) + (dimy - 1) * (dimz - 1);
			g_numTriangles = g_numTriangles * 2 * 2;
			//g_numTriangles = 0;

			int clothStart = 0;

			CreateSpringGrid3(Vec3(0.3f, 1.0f, 0.0f), dimx, dimy, dimz, radius*0.25f, flexMakePhase(group++, 0), stretchStiffness, bendStiffness, shearStiffness, Vec3(0.0f), invmass);

			int corner0 = clothStart + 0 + (dimz - 1) * dimx * dimy;
			int corner1 = clothStart + dimx - 1 + (dimz - 1) * dimx * dimy;
			int corner2 = clothStart + dimx*(dimy - 1) + (dimz - 1) * dimx * dimy;
			int corner3 = clothStart + dimx*dimy - 1 + (dimz - 1) * dimx * dimy;

			g_positions[corner0].w = 0.0f;
			g_positions[corner1].w = 0.0f;
			g_positions[corner2].w = 0.0f;
			g_positions[corner3].w = 0.0f;

			// add tethers
			for (int i = clothStart; i < int(g_positions.size()); ++i)
			{

				if (i != corner0 && i != corner1 && i != corner2 && i != corner3)
				{
					float stiffness = -0.5f;
					float give = 0.05f;

					CreateSpring(corner0, i, stiffness, give);
					CreateSpring(corner1, i, stiffness, give);
					CreateSpring(corner2, i, stiffness, give);
					CreateSpring(corner3, i, stiffness, give);
				}
			}

		}

		g_numSolidParticles = g_positions.size();
		g_ior = 1.0f;

		g_numExtraParticles = 64 * 1024;

		g_params.mRadius = radius;
		g_params.mFluid = true;
		g_params.mNumIterations = 5;
		g_params.mVorticityConfinement = 0.0f;
		g_params.mAnisotropyScale = 30.0f;
		g_params.mFluidRestDistance = g_params.mRadius*0.5f;
		g_params.mSmoothing = 0.5f;
		g_params.mSolidPressure = 0.25f;
		g_numSubsteps = 3;
		//g_params.mNumIterations = 6;

		g_params.mMaxVelocity = 0.5f*g_numSubsteps*g_params.mRadius / g_dt;

		g_maxDiffuseParticles = 32 * 1024;
		g_diffuseScale = 0.5f;
		g_lightDistance = 3.0f;
		g_pointScale = 0.5f;

		// for viscous goo
		if (mViscous)
		{
			g_fluidColor = Vec4(0.0f, 0.8f, 0.2f, 1.0f);

			g_params.mDynamicFriction = 0.3f;
			g_params.mCohesion = 0.025f;
			g_params.mViscosity = 50.85f;
		}
		else
		{
			g_params.mDynamicFriction = 0.125f;
			g_params.mViscosity = 0.1f;
			g_params.mCohesion = 0.0035f;
			g_params.mViscosity = 4.0f;
		}

		g_emitters[0].mEnabled = false;

		Emitter e;
		e.mDir = Normalize(Vec3(1.0f, 0.0f, 0.0f));
		e.mEnabled = true;
		e.mPos = Vec3(-0.25f, 1.75f, 0.5f);
		e.mRight = Cross(e.mDir, Vec3(0.0f, 0.0f, 1.0f));
		e.mSpeed = (g_params.mFluidRestDistance / (g_dt*2.0f));

		g_emitters.push_back(e);

		// draw options		
		g_drawPoints = false;
		g_drawSprings = false;
		g_drawEllipsoids = true;


	}
	virtual void DoGui(){

		imguiLabel("Scene");

		if (imguiCheck("Dyeing", bool(g_absorb != 0 && g_diffuse != 0 && g_drip != 0))){
			if (g_absorb && g_diffuse && g_drip){
				g_absorb = false;
				g_diffuse = false;
				g_drip = false;
			}
			else {
				g_absorb = true;
				g_diffuse = true;
				g_drip = true;
			}
		}

		if (imguiCheck("Absorbing", bool(g_absorb != 0)))
			g_absorb = !g_absorb;
		if (imguiCheck("Diffusing", bool(g_diffuse != 0)))
			g_diffuse = !g_diffuse;
		if (imguiCheck("Dripping", bool(g_drip != 0)))
			g_drip = !g_drip;
		if (imguiCheck("Mark", bool(g_markColor != 0)))
			g_markColor = !g_markColor;

		imguiSlider("k Absorption", &g_kAbsorption, 0.0, 1.0, 0.1);
		imguiSlider("k Diffusion", &g_kDiffusion, 0.0, 1.0, 0.1);
		imguiSlider("k Diffusion Gravity", &g_kDiffusionGravity, 0.0, 1.0, 0.1);

		imguiSeparatorLine();
	}

	bool mViscous;
};

class FluidClothCoupling4 : public Scene			//cube
{
public:

	FluidClothCoupling4(const char* name, bool viscous) : Scene(name), mViscous(viscous) {}

	void Initialize()
	{
		is_cube = true;
		is_point = false;
		is_complex = false;
		is_hollow = false;

		sceneNum = 0;

		g_absorb = true;
		g_diffuse = true;
		g_drip = true;
		g_markColor = false;

		g_emitterWidth = 3;

		g_kAbsorption = 1.0;
		g_kMaxAbsorption = 0.5;
		g_kDiffusion = 0.2;
		g_kDiffusionGravity = 0.2;

		g_camInit = false;
		g_camPos = Vec3(0.7f, 1.7f, 2.9f);
		g_camAngle = Vec3(0.0f, -0.4f, 0.0f);


		float stretchStiffness = 1.0f;
		float bendStiffness = 0.4f;
		float shearStiffness = 0.4f;

		int dimx = 32;
		int dimy = 32;
		int dimz = 6;
		float radius = 0.1f;
		float invmass = 0.25f;
		int group = 0;

		g_dripBuffer.resize(0);
		g_saturations.resize(0);
		g_cubeCenters.resize(0);
		g_thetas.resize(0);


		{
			g_dx = dimx;
			g_dy = dimy;
			g_dz = dimz;

			g_numTriangles = (dimx - 1) * (dimy - 1) + (dimx - 1) * (dimz - 1) + (dimy - 1) * (dimz - 1);
			g_numTriangles = g_numTriangles * 2 * 2;
			
			g_numCubes = (dimx - 1) * (dimy - 1) * (dimz - 1);

			int clothStart = 0;

			CreateSpringGridCube(Vec3(0.3f, 1.0f, 0.0f), dimx, dimy, dimz, radius*0.25f, flexMakePhase(group++, 0), stretchStiffness, bendStiffness, shearStiffness, Vec3(0.0f), invmass);

			int corner0 = clothStart + 0 + (dimz - 1) * dimx * dimy;
			int corner1 = clothStart + dimx - 1 + (dimz - 1) * dimx * dimy;
			int corner2 = clothStart + dimx*(dimy - 1) + (dimz - 1) * dimx * dimy;
			int corner3 = clothStart + dimx*dimy - 1 + (dimz - 1) * dimx * dimy;

			g_positions[corner0].w = 0.0f;
			g_positions[corner1].w = 0.0f;
			g_positions[corner2].w = 0.0f;
			g_positions[corner3].w = 0.0f;

			// add tethers
			for (int i = clothStart; i < int(g_positions.size()); ++i)
			{

				if (i != corner0 && i != corner1 && i != corner2 && i != corner3)
				{
					float stiffness = -0.5f;
					float give = 0.05f;

					CreateSpring(corner0, i, stiffness, give);
					CreateSpring(corner1, i, stiffness, give);
					CreateSpring(corner2, i, stiffness, give);
					CreateSpring(corner3, i, stiffness, give);
				}
			}

		}

		g_numSolidParticles = g_positions.size();
		g_ior = 1.0f;

		g_numExtraParticles = 64 * 1024;

		g_params.mRadius = radius;
		g_params.mFluid = true;
		g_params.mNumIterations = 5;
		g_params.mVorticityConfinement = 0.0f;
		g_params.mAnisotropyScale = 30.0f;
		g_params.mFluidRestDistance = g_params.mRadius*0.5f;
		g_params.mSmoothing = 0.5f;
		g_params.mSolidPressure = 0.25f;
		g_numSubsteps = 3;
		//g_params.mNumIterations = 6;

		g_params.mMaxVelocity = 0.5f*g_numSubsteps*g_params.mRadius / g_dt;

		g_maxDiffuseParticles = 32 * 1024;
		g_diffuseScale = 0.5f;
		g_lightDistance = 3.0f;
		g_pointScale = 0.5f;

		// for viscous goo
		if (mViscous)
		{
			g_fluidColor = Vec4(0.0f, 0.8f, 0.2f, 1.0f);

			g_params.mDynamicFriction = 0.3f;
			g_params.mCohesion = 0.025f;
			g_params.mViscosity = 50.85f;
		}
		else
		{
			g_params.mDynamicFriction = 0.125f;
			g_params.mViscosity = 0.1f;
			g_params.mCohesion = 0.0035f;
			g_params.mViscosity = 4.0f;
		}

		g_emitters[0].mEnabled = false;

		Emitter e;
		e.mDir = Normalize(Vec3(1.0f, 0.0f, 0.0f));
		e.mEnabled = true;
		e.mPos = Vec3(-0.25f, 1.75f, 0.5f);
		e.mRight = Cross(e.mDir, Vec3(0.0f, 0.0f, 1.0f));
		e.mSpeed = (g_params.mFluidRestDistance / (g_dt*2.0f));

		g_emitters.push_back(e);

		// draw options		
		g_drawPoints = false;
		g_drawSprings = false;
		g_drawEllipsoids = true;


	}
	virtual void DoGui(){

		imguiLabel("Scene");

		if (imguiCheck("Dyeing", bool(g_absorb != 0 && g_diffuse != 0 && g_drip != 0))){
			if (g_absorb && g_diffuse && g_drip){
				g_absorb = false;
				g_diffuse = false;
				g_drip = false;
			}
			else {
				g_absorb = true;
				g_diffuse = true;
				g_drip = true;
			}
		}

		if (imguiCheck("Absorbing", bool(g_absorb != 0)))
			g_absorb = !g_absorb;
		if (imguiCheck("Diffusing", bool(g_diffuse != 0)))
			g_diffuse = !g_diffuse;
		if (imguiCheck("Dripping", bool(g_drip != 0)))
			g_drip = !g_drip;
		if (imguiCheck("Mark", bool(g_markColor != 0)))
			g_markColor = !g_markColor;

		imguiSlider("k Absorption", &g_kAbsorption, 0.0, 1.0, 0.1);
		imguiSlider("k Diffusion", &g_kDiffusion, 0.0, 1.0, 0.1);
		imguiSlider("k Diffusion Gravity", &g_kDiffusionGravity, 0.0, 1.0, 0.1);
		
		imguiSeparatorLine();
	}

	bool mViscous;
};
class FluidClothCoupling5 : public Scene			
{
public:

	FluidClothCoupling5(const char* name, bool viscous) : Scene(name), mViscous(viscous) {}

	void Initialize()
	{
		is_cube = false;
		is_point = true;
		is_complex = false;
		is_hollow = true;

		sceneNum = 0;

		g_absorb = true;
		g_diffuse = true;
		g_drip = true;
		g_markColor = false;

		g_emitterWidth = 3;

		g_kAbsorption = 1.0;
		g_kMaxAbsorption = 0.5;
		g_kDiffusion = 0.2;
		g_kDiffusionGravity = 0.05;

		g_camInit = false;
		g_camPos = Vec3(0.7f, 1.7f, 2.9f);
		g_camAngle = Vec3(0.0f, -0.4f, 0.0f);



		float stretchStiffness = 1.0f;
		float bendStiffness = 0.4f;
		float shearStiffness = 0.4f;

		int dimx = 32;
		int dimy = 32;
		int dimz = 8;
		float radius = 0.1f;
		float invmass = 0.25f;
		int group = 0;

		g_dripBuffer.resize(0);
		g_saturations.resize(0);
		g_thetas.resize(0);

		{
			g_dx = dimx;
			g_dy = dimy;
			g_dz = dimz;

			g_numTriangles = (dimx - 1) * (dimy - 1) + (dimx - 1) * (dimz - 1) + (dimy - 1) * (dimz - 1);
			g_numTriangles = g_numTriangles * 2 * 2;

			g_numPoints = dimx * dimy * dimz;

			int clothStart = 0;

			CreateSpringGridPoint(Vec3(0.2f, 1.0f, 0.0f), dimx, dimy, dimz, radius*0.25f, flexMakePhase(group++, 0), stretchStiffness, bendStiffness, shearStiffness, Vec3(0.0f), invmass);

			int corner0 = clothStart + 0 + dimx * dimy * (dimz - 1);
			int corner1 = clothStart + dimx - 1 + dimx * dimy * (dimz - 1);
			int corner2 = clothStart + dimx*(dimy - 1) + dimx * dimy * (dimz - 1);
			int corner3 = clothStart + dimx*dimy - 1 + dimx * dimy * (dimz - 1);

			g_positions[corner0].w = 0.0f;
			g_positions[corner1].w = 0.0f;
			g_positions[corner2].w = 0.0f;
			g_positions[corner3].w = 0.0f;

			// add tethers
			for (int i = clothStart; i < int(g_positions.size()); ++i)
			{

				if (i != corner0 && i != corner1 && i != corner2 && i != corner3)
				{
					float stiffness = -0.5f;
					float give = 0.05f;

					CreateSpring(corner0, i, stiffness, give);
					CreateSpring(corner1, i, stiffness, give);
					CreateSpring(corner2, i, stiffness, give);
					CreateSpring(corner3, i, stiffness, give);
				}
			}

		}

		g_numSolidParticles = g_positions.size();
		g_ior = 1.0f;

		g_numExtraParticles = 64 * 1024;

		g_params.mRadius = radius;
		g_params.mFluid = true;
		g_params.mNumIterations = 5;
		g_params.mVorticityConfinement = 0.0f;
		g_params.mAnisotropyScale = 30.0f;
		g_params.mFluidRestDistance = g_params.mRadius*0.5f;
		g_params.mSmoothing = 0.5f;
		g_params.mSolidPressure = 0.25f;
		g_numSubsteps = 3;
		//g_params.mNumIterations = 6;

		g_params.mMaxVelocity = 0.5f*g_numSubsteps*g_params.mRadius / g_dt;

		g_maxDiffuseParticles = 32 * 1024;
		g_diffuseScale = 0.5f;
		g_lightDistance = 3.0f;
		g_pointScale = 0.5f;

		// for viscous goo
		if (mViscous)
		{
			g_fluidColor = Vec4(0.0f, 0.8f, 0.2f, 1.0f);

			g_params.mDynamicFriction = 0.3f;
			g_params.mCohesion = 0.025f;
			g_params.mViscosity = 50.85f;
		}
		else
		{
			g_params.mDynamicFriction = 0.125f;
			g_params.mViscosity = 0.1f;
			g_params.mCohesion = 0.0035f;
			g_params.mViscosity = 4.0f;
		}

		g_emitters[0].mEnabled = false;

		Emitter e;
		e.mDir = Normalize(Vec3(1.0f, 0.0f, 0.0f));
		e.mEnabled = true;
		e.mPos = Vec3(-0.25f, 1.75f, 0.5f);
		e.mRight = Cross(e.mDir, Vec3(0.0f, 0.0f, 1.0f));
		e.mSpeed = (g_params.mFluidRestDistance / (g_dt*2.0f));

		g_emitters.push_back(e);

		// draw options		
		g_drawPoints = false;
		g_drawSprings = false;
		g_drawEllipsoids = true;

	}
	virtual void DoGui(){

		imguiLabel("Scene");

		if (imguiCheck("Dyeing", bool(g_absorb != 0 && g_diffuse != 0 && g_drip != 0))){
			if (g_absorb && g_diffuse && g_drip){
				g_absorb = false;
				g_diffuse = false;
				g_drip = false;
			}
			else {
				g_absorb = true;
				g_diffuse = true;
				g_drip = true;
			}
		}

		if (imguiCheck("Absorbing", bool(g_absorb != 0)))
			g_absorb = !g_absorb;
		if (imguiCheck("Diffusing", bool(g_diffuse != 0)))
			g_diffuse = !g_diffuse;
		if (imguiCheck("Dripping", bool(g_drip != 0)))
			g_drip = !g_drip;
		if (imguiCheck("Mark", bool(g_markColor != 0)))
			g_markColor = !g_markColor;

		imguiSlider("k Absorption", &g_kAbsorption, 0.0, 1.0, 0.1);
		imguiSlider("k Diffusion", &g_kDiffusion, 0.0, 1.0, 0.1);
		imguiSlider("k Diffusion Gravity", &g_kDiffusionGravity, 0.0, 1.0, 0.1);

		imguiSeparatorLine();
	}

	bool mViscous;
};
class FluidClothCoupling6 : public Scene			//cube
{
public:

	FluidClothCoupling6(const char* name, bool viscous) : Scene(name), mViscous(viscous) {}

	void Initialize()
	{
		is_cube = false;
		is_point = true;
		is_complex = false;
		is_hollow = false;

		sceneNum = 0;

		g_absorb = true;
		g_diffuse = true;
		g_drip = true;
		g_markColor = false;

		g_emitterWidth = 3;

		g_kAbsorption = 1.0;
		g_kMaxAbsorption = 0.5;
		g_kDiffusion = 0.2;
		g_kDiffusionGravity = 0.2;

		g_camInit = false;
		g_camPos = Vec3(0.7f, 1.7f, 2.9f);
		g_camAngle = Vec3(0.0f, -0.4f, 0.0f);


		float stretchStiffness = 1.0f;
		float bendStiffness = 0.4f;
		float shearStiffness = 0.4f;

		int dimx = 32;
		int dimy = 32;
		int dimz = 8;
		float radius = 0.1f;
		float invmass = 0.25f;
		int group = 0;

		g_dripBuffer.resize(0);
		g_saturations.resize(0);
		g_thetas.resize(0);


		{
			g_dx = dimx;
			g_dy = dimy;
			g_dz = dimz;

			g_numTriangles = (dimx - 1) * (dimy - 1) + (dimx - 1) * (dimz - 1) + (dimy - 1) * (dimz - 1);
			g_numTriangles = g_numTriangles * 2 * 2;

			g_numPoints = dimx * dimy * dimz;

			int clothStart = 0;

			CreateSpringGridPoint(Vec3(0.2f, 1.0f, 0.0f), dimx, dimy, dimz, radius*0.25f, flexMakePhase(group++, 0), stretchStiffness, bendStiffness, shearStiffness, Vec3(0.0f), invmass);

			int corner0 = clothStart + 0 + (dimz - 1) * dimx * dimy;
			int corner1 = clothStart + dimx - 1 + (dimz - 1) * dimx * dimy;
			int corner2 = clothStart + dimx*(dimy - 1) + (dimz - 1) * dimx * dimy;
			int corner3 = clothStart + dimx*dimy - 1 + (dimz - 1) * dimx * dimy;

			g_positions[corner0].w = 0.0f;
			g_positions[corner1].w = 0.0f;
			g_positions[corner2].w = 0.0f;
			g_positions[corner3].w = 0.0f;

			// add tethers
			for (int i = clothStart; i < int(g_positions.size()); ++i)
			{

				if (i != corner0 && i != corner1 && i != corner2 && i != corner3)
				{
					float stiffness = -0.5f;
					float give = 0.05f;

					CreateSpring(corner0, i, stiffness, give);
					CreateSpring(corner1, i, stiffness, give);
					CreateSpring(corner2, i, stiffness, give);
					CreateSpring(corner3, i, stiffness, give);
				}
			}

		}

		g_numSolidParticles = g_positions.size();
		g_ior = 1.0f;

		g_numExtraParticles = 64 * 1024;

		g_params.mRadius = radius;
		g_params.mFluid = true;
		g_params.mNumIterations = 5;
		g_params.mVorticityConfinement = 0.0f;
		g_params.mAnisotropyScale = 30.0f;
		g_params.mFluidRestDistance = g_params.mRadius*0.5f;
		g_params.mSmoothing = 0.5f;
		g_params.mSolidPressure = 0.25f;
		g_numSubsteps = 3;
		//g_params.mNumIterations = 6;

		g_params.mMaxVelocity = 0.5f*g_numSubsteps*g_params.mRadius / g_dt;

		g_maxDiffuseParticles = 32 * 1024;
		g_diffuseScale = 0.5f;
		g_lightDistance = 3.0f;
		g_pointScale = 0.5f;

		// for viscous goo
		if (mViscous)
		{
			g_fluidColor = Vec4(0.0f, 0.8f, 0.2f, 1.0f);

			g_params.mDynamicFriction = 0.3f;
			g_params.mCohesion = 0.025f;
			g_params.mViscosity = 50.85f;
		}
		else
		{
			g_params.mDynamicFriction = 0.125f;
			g_params.mViscosity = 0.1f;
			g_params.mCohesion = 0.0035f;
			g_params.mViscosity = 4.0f;
		}

		g_emitters[0].mEnabled = false;

		Emitter e;
		e.mDir = Normalize(Vec3(1.0f, 0.0f, 0.0f));
		e.mEnabled = true;
		e.mPos = Vec3(-0.25f, 1.75f, 0.5f);
		e.mRight = Cross(e.mDir, Vec3(0.0f, 0.0f, 1.0f));
		e.mSpeed = (g_params.mFluidRestDistance / (g_dt*2.0f));

		g_emitters.push_back(e);

		// draw options		
		g_drawPoints = false;
		g_drawSprings = false;
		g_drawEllipsoids = true;

	}
	virtual void DoGui(){

		imguiLabel("Scene");

		if (imguiCheck("Dyeing", bool(g_absorb != 0 && g_diffuse != 0 && g_drip != 0))){
			if (g_absorb && g_diffuse && g_drip){
				g_absorb = false;
				g_diffuse = false;
				g_drip = false;
			}
			else {
				g_absorb = true;
				g_diffuse = true;
				g_drip = true;
			}
		}

		if (imguiCheck("Absorbing", bool(g_absorb != 0)))
			g_absorb = !g_absorb;
		if (imguiCheck("Diffusing", bool(g_diffuse != 0)))
			g_diffuse = !g_diffuse;
		if (imguiCheck("Dripping", bool(g_drip != 0)))
			g_drip = !g_drip;
		if (imguiCheck("Mark", bool(g_markColor != 0)))
			g_markColor = !g_markColor;

		imguiSlider("k Absorption", &g_kAbsorption, 0.0, 1.0, 0.1);
		imguiSlider("k Diffusion", &g_kDiffusion, 0.0, 1.0, 0.1);
		imguiSlider("k Diffusion Gravity", &g_kDiffusionGravity, 0.0, 1.0, 0.1);

		imguiSeparatorLine();
	}

	bool mViscous;
};

class FluidObjectHollow : public Scene			//cube
{
public:

	FluidObjectHollow(const char* name, bool viscous) : Scene(name), mViscous(viscous) {}

	void Initialize()
	{
		is_cube = false;
		is_point = false;
		is_complex = true;
		is_hollow = false;

		sceneNum = 0;

		g_absorb = true;
		g_diffuse = true;
		g_drip = true;
		g_markColor = false;

		g_emitterWidth = 5;

		g_kAbsorption = 1.0;
		g_kMaxAbsorption = 0.5;
		g_kDiffusion = 0.2;
		g_kDiffusionGravity = 0.2;

		g_camInit = false;
		g_camPos = Vec3(-0.5f, 1.5f, 2.5f);
		g_camAngle = Vec3(0.0f, -0.4f, 0.0f);

		float radius = 0.1f;

		g_positions.resize(0);
		g_normals.resize(0);

		g_dripBuffer.resize(0);
		g_saturations.resize(0);
		g_triangleCenters.resize(0);
		g_triangleNeighbours.resize(0);
		g_thetas.resize(0);

		g_pointTriangleNums.resize(0);
		g_pointTriangles.resize(0);
		g_trianglePoints.resize(0);

		if (g_mesh)
			delete g_mesh;
		{

			float shapeSize = 1.0f;
			CreateSDF2("../../data/bunny.ply", shapeSize, Vec3(-shapeSize*0.5f, 0.0f, -shapeSize*0.5f), g_params.mCollisionDistance*0.125f);

			Vec3 lower, upper;
			g_mesh->GetBounds(lower, upper);

			CalculateTriangleNeighbours();

		}

		g_numSolidParticles = g_positions.size();
		g_ior = 1.0f;

		g_numExtraParticles = 64 * 1024;

		g_params.mRadius = radius;
		g_params.mFluid = true;
		g_params.mNumIterations = 5;
		g_params.mVorticityConfinement = 0.0f;
		g_params.mAnisotropyScale = 30.0f;
		g_params.mFluidRestDistance = g_params.mRadius*0.5f;
		g_params.mSmoothing = 0.5f;
		g_params.mSolidPressure = 0.25f;
		g_numSubsteps = 3;
		//g_params.mNumIterations = 6;

		g_params.mMaxVelocity = 0.5f*g_numSubsteps*g_params.mRadius / g_dt;

		g_maxDiffuseParticles = 32 * 1024;
		g_diffuseScale = 0.5f;
		g_lightDistance = 3.0f;
		g_pointScale = 0.5f;

		// for viscous goo
		if (mViscous)
		{
			g_fluidColor = Vec4(0.0f, 0.8f, 0.2f, 1.0f);

			g_params.mDynamicFriction = 0.3f;
			g_params.mCohesion = 0.025f;
			//g_params.mViscosity = 50.85f;
			g_params.mViscosity = 100.0f;
		}
		else
		{
			g_params.mDynamicFriction = 0.125f;
			g_params.mViscosity = 0.1f;
			g_params.mCohesion = 0.0035f;
			g_params.mViscosity = 4.0f;
		}

		g_emitters[0].mEnabled = false;

		Emitter e;
		e.mDir = Normalize(Vec3(1.0f, 0.0f, 0.0f));
		e.mEnabled = true;
		e.mPos = Vec3(-1.0f, 1.75f, 0.0f);
		e.mRight = Cross(e.mDir, Vec3(0.0f, 0.0f, 1.0f));
		e.mSpeed = (g_params.mFluidRestDistance / (g_dt*2.0f));

		g_emitters.push_back(e);

		// draw options		
		g_drawPoints = false;
		g_drawSprings = false;
		g_drawEllipsoids = true;

	}
	virtual void DoGui(){

		imguiLabel("Scene");

		if (imguiCheck("Dyeing", bool(g_absorb != 0 && g_diffuse != 0 && g_drip != 0))){
			if (g_absorb && g_diffuse && g_drip){
				g_absorb = false;
				g_diffuse = false;
				g_drip = false;
			}
			else {
				g_absorb = true;
				g_diffuse = true;
				g_drip = true;
			}
		}

		if (imguiCheck("Absorbing", bool(g_absorb != 0)))
			g_absorb = !g_absorb;
		if (imguiCheck("Diffusing", bool(g_diffuse != 0)))
			g_diffuse = !g_diffuse;
		if (imguiCheck("Dripping", bool(g_drip != 0)))
			g_drip = !g_drip;
		if (imguiCheck("Mark", bool(g_markColor != 0)))
			g_markColor = !g_markColor;

		imguiSlider("k Absorption", &g_kAbsorption, 0.0, 1.0, 0.1);
		imguiSlider("k Diffusion", &g_kDiffusion, 0.0, 1.0, 0.1);
		imguiSlider("k Diffusion Gravity", &g_kDiffusionGravity, 0.0, 1.0, 0.1);

		imguiSeparatorLine();
	}

	bool mViscous;
};
class FluidObjectHollow2 : public Scene			//cube
{
public:

	FluidObjectHollow2(const char* name, bool viscous) : Scene(name), mViscous(viscous) {}

	void Initialize()
	{
		is_cube = false;
		is_point = false;
		is_complex = true;
		is_hollow = false;

		sceneNum = 0;

		g_absorb = false;
		g_diffuse = false;
		g_drip = false;
		g_markColor = false;

		g_emitterWidth = 10;

		g_kAbsorption = 1.0;
		g_kMaxAbsorption = 0.5;
		g_kDiffusion = 0.2;
		g_kDiffusionGravity = 0.2;

		g_camInit = false;
		g_camPos = Vec3(-0.5f, 1.5f, 2.5f);
		g_camAngle = Vec3(0.0f, -0.4f, 0.0f);

		float radius = 0.1f;

		g_positions.resize(0);

		g_dripBuffer.resize(0);
		g_saturations.resize(0);
		g_triangleCenters.resize(0);
		g_triangleNeighbours.resize(0);
		g_thetas.resize(0);

		g_pointTriangleNums.resize(0);
		g_pointTriangles.resize(0);
		g_trianglePoints.resize(0);


		{

			float shapeSize = 0.5f;
			CreateSDF2("../../data/armadillo.ply", shapeSize, Vec3(-shapeSize*0.5f, 0.0f, -shapeSize*0.5f), g_params.mCollisionDistance*0.125f);

			Vec3 lower, upper;
			g_mesh->GetBounds(lower, upper);

			CalculateTriangleNeighbours();
		}

		g_numSolidParticles = g_positions.size();
		g_ior = 1.0f;

		g_numExtraParticles = 64 * 1024;

		g_params.mRadius = radius;
		g_params.mFluid = true;
		g_params.mNumIterations = 5;
		g_params.mVorticityConfinement = 0.0f;
		g_params.mAnisotropyScale = 30.0f;
		g_params.mFluidRestDistance = g_params.mRadius*0.5f;
		g_params.mSmoothing = 0.5f;
		g_params.mSolidPressure = 0.25f;
		g_numSubsteps = 3;
		//g_params.mNumIterations = 6;

		g_params.mMaxVelocity = 0.5f*g_numSubsteps*g_params.mRadius / g_dt;

		g_maxDiffuseParticles = 32 * 1024;
		g_diffuseScale = 0.5f;
		g_lightDistance = 3.0f;
		g_pointScale = 0.5f;

		// for viscous goo
		if (mViscous)
		{
			g_fluidColor = Vec4(0.0f, 0.8f, 0.2f, 1.0f);

			g_params.mDynamicFriction = 0.3f;
			g_params.mCohesion = 0.025f;
			g_params.mViscosity = 50.85f;
		}
		else
		{
			g_params.mDynamicFriction = 0.125f;
			g_params.mViscosity = 0.1f;
			g_params.mCohesion = 0.0035f;
			g_params.mViscosity = 4.0f;
		}

		g_emitters[0].mEnabled = false;

		Emitter e;
		e.mDir = Normalize(Vec3(1.0f, 0.0f, 0.0f));
		e.mEnabled = true;
		e.mPos = Vec3(-1.0f, 1.75f, 0.0f);
		e.mRight = Cross(e.mDir, Vec3(0.0f, 0.0f, 1.0f));
		e.mSpeed = (g_params.mFluidRestDistance / (g_dt*2.0f));

		g_emitters.push_back(e);

		// draw options		
		g_drawPoints = false;
		g_drawSprings = false;
		g_drawEllipsoids = true;

	}
	virtual void DoGui(){

		imguiLabel("Scene");

		if (imguiCheck("Dyeing", bool(g_absorb != 0 && g_diffuse != 0 && g_drip != 0))){
			if (g_absorb && g_diffuse && g_drip){
				g_absorb = false;
				g_diffuse = false;
				g_drip = false;
			}
			else {
				g_absorb = true;
				g_diffuse = true;
				g_drip = true;
			}
		}

		if (imguiCheck("Absorbing", bool(g_absorb != 0)))
			g_absorb = !g_absorb;
		if (imguiCheck("Diffusing", bool(g_diffuse != 0)))
			g_diffuse = !g_diffuse;
		if (imguiCheck("Dripping", bool(g_drip != 0)))
			g_drip = !g_drip;
		if (imguiCheck("Mark", bool(g_markColor != 0)))
			g_markColor = !g_markColor;

		imguiSlider("k Absorption", &g_kAbsorption, 0.0, 1.0, 0.1);
		imguiSlider("k Diffusion", &g_kDiffusion, 0.0, 1.0, 0.1);
		imguiSlider("k Diffusion Gravity", &g_kDiffusionGravity, 0.0, 1.0, 0.1);

		imguiSeparatorLine();
	}

	bool mViscous;
};

class FluidObjectSolid : public Scene			//cube
{
public:

	FluidObjectSolid(const char* name, bool viscous) : Scene(name), mViscous(viscous) {}

	void Initialize()
	{
		is_cube = false;
		is_point = false;
		is_complex = true;
		is_hollow = false;

		sceneNum = 0;

		g_absorb = false;
		g_diffuse = false;
		g_drip = false;
		g_markColor = false;

		g_emitterWidth = 5;

		g_kAbsorption = 1.0;
		g_kMaxAbsorption = 0.5;
		g_kDiffusion = 0.2;
		g_kDiffusionGravity = 0.2;

		g_camInit = false;
		g_camPos = Vec3(-0.5f, 1.5f, 2.5f);
		g_camAngle = Vec3(0.0f, -0.4f, 0.0f);

		float radius = 0.1f;

		g_positions.resize(0);

		g_dripBuffer.resize(0);
		g_saturations.resize(0);
		g_triangleCenters.resize(0);
		g_triangleNeighbours.resize(0);
		g_thetas.resize(0);

		g_pointTriangleNums.resize(0);
		g_pointTriangles.resize(0);
		g_trianglePoints.resize(0);


		{

			float shapeSize = 1.0f;

			int phase = flexMakePhase(0, eFlexPhaseSelfCollide); 
			float spacing = g_params.mRadius * 0.4f;

			CreateParticleShape2("../../data/bunny.ply", Vec3(-shapeSize*0.5f, 0.0f, -shapeSize*0.5f), shapeSize, 0.0f, spacing, Vec3(0.0f, 0.0f, 0.0f), 0.0f, true, 1.f, phase, true, 0.0f);


			CalculateTriangleNeighbours();

		}

		g_numSolidParticles = g_positions.size();
		g_ior = 1.0f;

		g_numExtraParticles = 64 * 1024;

		g_params.mRadius = radius;
		g_params.mFluid = true;
		g_params.mNumIterations = 5;
		g_params.mVorticityConfinement = 0.0f;
		g_params.mAnisotropyScale = 30.0f;
		g_params.mFluidRestDistance = g_params.mRadius*0.5f;
		g_params.mSmoothing = 0.5f;
		g_params.mSolidPressure = 0.25f;
		g_numSubsteps = 3;
		//g_params.mNumIterations = 6;

		g_params.mMaxVelocity = 0.5f*g_numSubsteps*g_params.mRadius / g_dt;

		g_maxDiffuseParticles = 32 * 1024;
		g_diffuseScale = 0.5f;
		g_lightDistance = 3.0f;
		g_pointScale = 0.5f;

		// for viscous goo
		if (mViscous)
		{
			g_fluidColor = Vec4(0.0f, 0.8f, 0.2f, 1.0f);

			g_params.mDynamicFriction = 0.3f;
			g_params.mCohesion = 0.025f;
			//g_params.mViscosity = 50.85f;
			g_params.mViscosity = 100.0f;
		}
		else
		{
			g_params.mDynamicFriction = 0.125f;
			g_params.mViscosity = 0.1f;
			g_params.mCohesion = 0.0035f;
			g_params.mViscosity = 4.0f;
		}

		g_emitters[0].mEnabled = false;

		Emitter e;
		e.mDir = Normalize(Vec3(1.0f, 0.0f, 0.0f));
		e.mEnabled = true;
		e.mPos = Vec3(-1.0f, 1.75f, 0.0f);
		e.mRight = Cross(e.mDir, Vec3(0.0f, 0.0f, 1.0f));
		e.mSpeed = (g_params.mFluidRestDistance / (g_dt*2.0f));

		g_emitters.push_back(e);

		// draw options		
		g_drawPoints = false;
		g_drawSprings = false;
		g_drawEllipsoids = true;

	}
	virtual void DoGui(){

		imguiLabel("Scene");

		if (imguiCheck("Dyeing", bool(g_absorb != 0 && g_diffuse != 0 && g_drip != 0))){
			if (g_absorb && g_diffuse && g_drip){
				g_absorb = false;
				g_diffuse = false;
				g_drip = false;
			}
			else {
				g_absorb = true;
				g_diffuse = true;
				g_drip = true;
			}
		}

		if (imguiCheck("Absorbing", bool(g_absorb != 0)))
			g_absorb = !g_absorb;
		if (imguiCheck("Diffusing", bool(g_diffuse != 0)))
			g_diffuse = !g_diffuse;
		if (imguiCheck("Dripping", bool(g_drip != 0)))
			g_drip = !g_drip;
		if (imguiCheck("Mark", bool(g_markColor != 0)))
			g_markColor = !g_markColor;

		imguiSlider("k Absorption", &g_kAbsorption, 0.0, 1.0, 0.1);
		imguiSlider("k Diffusion", &g_kDiffusion, 0.0, 1.0, 0.1);
		imguiSlider("k Diffusion Gravity", &g_kDiffusionGravity, 0.0, 1.0, 0.1);

		imguiSeparatorLine();
	}

	bool mViscous;
};

class Deformables : public Scene
{
public:

	Deformables(const char* name) : Scene(name) {}

	void Initialize()
	{
		g_params.mDynamicFriction = 0.25f;

		for (int i=0; i < 5; i++)
			CreateRandomConvex(10, Vec3(i*2.0f, 0.0f, Randf(0.0f, 2.0f)), 0.5f, 1.0f, Vec3(0.0f, 1.0f, 0.0f), Randf(0.0f, k2Pi));

		if (0)
		{
			int group = 0;

			float minSize = 0.2f;
			float maxSize = 0.4f;

			for (int z=0; z < 1; ++z)
				for (int y=0; y < 1; ++y)
					for (int x=0; x < 5; ++x)
						CreateRandomBody(12, Vec3(2.0f*x, 2.0f + y, 1.0f + 1.0f*z), minSize, maxSize, RandomUnitVector(), Randf(0.0f, k2Pi), 1.0f, flexMakePhase(group++, 0), 0.25f);
		}
		else
		{
			CreateTetMesh("../../data/tets/duck.tet", Vec3(2.0f, 1.0f, 2.0f), 2.00000105f, 1.0f, 0);
			CreateTetMesh("../../data/tets/duck.tet", Vec3(2.0f, 3.0f, 2.0f), 2.00000105f, 1.0f, 1);
		}
	
		g_params.mNumIterations = 5;
		g_params.mRelaxationFactor = 1.0f;
		g_params.mRadius = 0.025f;

		// draw options		
		g_drawPoints = true;
		g_drawSprings = false;
	}
};


class GranularPile : public Scene
{
public:

	GranularPile(const char* name) : Scene(name) {}

	virtual void Initialize()
	{
		// granular pile
		float radius = 0.075f;

		Vec3 lower(8.0f, 4.0f, 2.0f);

		CreateParticleShape("../../data/sphere.ply", lower, 1.0f, 0.0f, radius, 0.0f, 0.f, true, 1.0f, flexMakePhase(1, 0), true, 0.00f);
		g_numSolidParticles = g_positions.size();

		CreateParticleShape("../../data/sandcastle.obj", Vec3(-2.0f, 0.0f, 0.0f), 4.0f, 0.0f, radius*1.0001f, 0.0f, 1.0f, false, 0.0f, flexMakePhase(0, eFlexPhaseSelfCollide), false, 0.00f);

		g_numSubsteps = 2;

		g_params.mRadius = radius;
		g_params.mStaticFriction = 1.0f;
		g_params.mDynamicFriction = 0.5f;
		g_params.mViscosity = 0.0f;
		g_params.mNumIterations = 12;
		g_params.mParticleCollisionMargin = g_params.mRadius*0.25f;	// 5% collision margin
		g_params.mSleepThreshold = g_params.mRadius*0.25f;
		g_params.mShockPropagation = 6.f;
		g_params.mRestitution = 0.0001f;
		g_params.mRelaxationFactor = 1.f;
		g_params.mDamping = 0.14f;
		g_params.mNumPlanes = 1;
		
		// draw options
		g_drawPoints = true;		
		g_warmup = false;

		// hack, change the color of phase 0 particles to 'sand'
		extern Colour gColors[];
		gColors[0] = Colour(0.805f, 0.702f, 0.401f);		
	}

	void Update()
	{
		// launch ball after 3 seconds
		if (g_frame == 180)
		{
			for (int i=0; i < g_numSolidParticles; ++i)
			{
				g_positions[i].w = 0.9f;
				g_velocities[i] = Vec3(-15.0f, 0.0f, 0.0f);
			}
		}
	}
};


class GranularShape : public Scene
{
public:

	GranularShape(const char* name) : Scene(name) {}

	void Initialize()
	{
		// granular dragon
		CreateParticleShape("../../data/dragon.obj",Vec3(0.0f, 2.5f, 0.0f), 16.0f, DegToRad(-20.0f), g_params.mRadius*1.05f, Vec3(0.0f, 0.0f, 0.0f), 1.0f, false, 0.0f, flexMakePhase(0, eFlexPhaseSelfCollide), false, g_params.mRadius*0.05f);
		
		CreateConvex(Vec3(8.0f, 8.0f, 5.0f));
		g_convexPositions[0] += Vec4(0.0f, -1.5f, 0.0f, 0.0f);
		
		g_params.mStaticFriction = 1.0f;
		g_params.mDynamicFriction = 0.65f;
		g_params.mDissipation = 0.01f;
		g_params.mNumIterations = 6;
		g_params.mParticleCollisionMargin = g_params.mRadius*0.5f;	// 5% collision margin
		g_params.mSleepThreshold = g_params.mRadius*0.35f;
		g_params.mShockPropagation = 3.f;
		g_params.mRestitution = 0.01f;
		g_params.mGravity[1] *= 1.f;

		g_numSubsteps = 3;

		extern Colour gColors[];
		gColors[1] = Colour(0.805f, 0.702f, 0.401f);		

		// draw options		
		g_drawPoints = true;
	}
};

class BunnyBath : public Scene
{
public:

	BunnyBath(const char* name, bool dam) : Scene(name), mDam(dam) {}

	void Initialize()
	{
		float radius = 0.1f;

		// deforming bunny
		float s = radius*0.5f;
		float m = 0.25f;
		int group = 0;

		CreateParticleShape("../../data/bunny.ply", Vec3(4.0f, 0.0f, 0.0f), 0.5f, 0.0f, s, Vec3(0.0f, 0.0f, 0.0f), m, true, 1.0f, flexMakePhase(group++, 0), true, 0.0f);
		CreateParticleShape("../../data/box.ply", Vec3(4.0f, 0.0f, 1.0f), 0.45f, 0.0f, s, Vec3(0.0f, 0.0f, 0.0f), m, true, 1.0f, flexMakePhase(group++, 0), true, 0.0f);
		CreateParticleShape("../../data/bunny.ply", Vec3(3.0f, 0.0f, 0.0f), 0.5f, 0.0f, s, Vec3(0.0f, 0.0f, 0.0f), m, true, 1.0f, flexMakePhase(group++, 0), true, 0.0f);
		CreateParticleShape("../../data/sphere.ply", Vec3(3.0f, 0.0f, 1.0f), 0.45f, 0.0f, s, Vec3(0.0f, 0.0f, 0.0f), m, true, 1.0f, flexMakePhase(group++, 0), true, 0.0f);
		CreateParticleShape("../../data/bunny.ply", Vec3(2.0f, 0.0f, 1.0f), 0.5f, 0.0f, s, Vec3(0.0f, 0.0f, 0.0f), m, true, 1.0f, flexMakePhase(group++, 0), true, 0.0f);
		CreateParticleShape("../../data/box.ply", Vec3(2.0f, 0.0f, 0.0f), 0.45f, 0.0f, s, Vec3(0.0f, 0.0f, 0.0f), m, true, 1.0f, flexMakePhase(group++, 0), true, 0.0f);

		g_numSolidParticles = g_positions.size();		

		float restDistance = radius*0.55f;

		if (mDam)
		{
			CreateParticleGrid(Vec3(0.0f, 0.0f, 0.6f), 24, 48, 24, restDistance, Vec3(0.0f), 1.0f, false, 0.0f, flexMakePhase(group++, eFlexPhaseSelfCollide | eFlexPhaseFluid), 0.005f);
			g_lightDistance *= 0.5f;
		}

		g_sceneLower = Vec3(0.0f);

		g_numSubsteps = 2;

		g_params.mRadius = radius;
		g_params.mDynamicFriction = 0.01f; 
		g_params.mFluid = true;
		g_params.mViscosity = 2.0f;
		g_params.mNumIterations = 4;
		g_params.mVorticityConfinement = 40.0f;
		g_params.mAnisotropyScale = 30.0f;
		g_params.mFluidRestDistance = restDistance;
		g_params.mSolidPressure = 0.f;
		g_params.mRelaxationFactor = 0.0f;
		g_params.mCohesion = 0.02f;
		g_params.mCollisionDistance = 0.01f;		

		g_maxDiffuseParticles = 64*1024;
		g_diffuseScale = 0.5f;

		g_fluidColor = Vec4(0.113f, 0.425f, 0.55f, 1.f);

		Emitter e1;
		e1.mDir = Vec3(1.0f, 0.0f, 0.0f);
		e1.mRight = Vec3(0.0f, 0.0f, -1.0f);
		e1.mPos = Vec3(radius, 1.f, 0.65f);
		e1.mSpeed = (restDistance/g_dt)*2.0f; // 2 particle layers per-frame
		e1.mEnabled = true;

		g_emitters.push_back(e1);

		g_numExtraParticles = 48*1024;

		g_lightDistance = 1.8f;

		g_params.mNumPlanes = 5;

		g_waveFloorTilt = 0.0f;
		g_waveFrequency = 1.5f;
		g_waveAmplitude = 2.0f;
		
		// draw options		
		g_drawPoints = false;
		g_drawEllipsoids = true;
		g_drawDiffuse = true;
	}

	bool mDam;
};


class TriangleMesh : public Scene
{
public:

	TriangleMesh(const char* name) : Scene(name) {}

	void Initialize()
	{
		float radius = 0.05f;
		CreateParticleGrid(Vec3(0.4f, 1.0f + radius*0.5f, 0.1f), 16, 4, 16, radius, Vec3(0.0f), 1.0f, false, 0.0f, flexMakePhase(0, eFlexPhaseSelfCollide), 0.0f);

		g_staticMesh = CreateDiscMesh(1.0f, 8);
		g_staticMesh->Transform(TranslationMatrix(Point3(0.0f, 0.5f, 0.0f)));
		g_staticMesh->m_positions[0].y -= 0.25f;	
		g_staticMesh->CalculateNormals();
		
		g_params.mRadius = radius;
		g_params.mDynamicFriction = 0.025f;
		g_params.mDissipation = 0.0f;
		g_params.mRestitution = 0.0;
		g_params.mNumIterations = 4;
		g_params.mParticleCollisionMargin = g_params.mRadius*0.05f;

		g_numSubsteps = 2;

		// draw options		
		g_drawPoints = true;
	}
	
};

class Restitution : public Scene
{
public:

	Restitution(const char* name) : Scene(name) {}

	void Initialize()
	{
		float radius = 0.05f;
		CreateParticleGrid(Vec3(0.0f, 1.0f, 0.0f), 1, 1, 1, radius, Vec3(0.0f), 1.0f, false, 0.0f, flexMakePhase(0, eFlexPhaseSelfCollide), 0.0f);

		g_params.mRadius = radius;
		g_params.mDynamicFriction = 0.025f;
		g_params.mDissipation = 0.0f;
		g_params.mRestitution = 1.0;
		g_params.mNumIterations = 4;

		g_numSubsteps = 4;

		// draw options		
		g_drawPoints = true;
	}
	
};


class GameMesh : public Scene
{
public:

	GameMesh(const char* name, int scene) : Scene(name), mScene(scene) {}

	void Initialize()
	{

		g_staticMesh = ImportMesh("../../data/testzone.obj");
		g_staticMesh->Normalize(100.0f);
		g_staticMesh->CalculateNormals();
		g_staticMesh->Transform(TranslationMatrix(Point3(0.0f, -5.0f, 0.0f)));
		
		Vec3 lower, upper;
		g_staticMesh->GetBounds(lower, upper);		
		Vec3 center = (lower+upper)*0.5f;

		int group = 0;
	
		// rigids
		if (mScene == 0)
		{
			float radius = 0.1f;
			
			for (int z=0; z < 80; ++z)
				for (int x=0; x < 80; ++x)
					CreateParticleGrid(
						center - Vec3(-16.0f, 0.0f, 15.0f) + Vec3(x*radius*2 - 1.0f, 1.0f + radius, 1.f + z*2.0f*radius) + 0.5f*Vec3(Randf(radius), 0.0f, Randf(radius)), 
						2, 2 + int(Randf(0.0f, 4.0f)), 2, radius*0.5f, Vec3(0.0f), 1.0f, true, 1.0f, flexMakePhase(group++, 0));

			// separte solid particle count
			g_numSolidParticles = g_positions.size();

			g_numExtraParticles = 32*1024;
	
			g_params.mRadius = radius;
			g_params.mDynamicFriction = 0.3f;
			g_params.mDissipation = 0.0f;
			g_params.mFluid = true;
			g_params.mFluidRestDistance = g_params.mRadius*0.5f;
			g_params.mViscosity = 0.05f;
			g_params.mAnisotropyScale = 20.0f;
			g_params.mNumIterations = 2;
			g_params.mNumPlanes = 1;
			g_params.mSleepThreshold = g_params.mRadius*0.3f;
			g_params.mMaxVelocity = 1.5f*g_params.mRadius/g_dt;
			g_numSubsteps = 2;
		
			g_emitters[0].mEnabled = true;	
			g_emitters[0].mSpeed = 2.0f*(g_params.mRadius*0.5f)/g_dt;

			// draw options		
			g_drawPoints = true;
			g_drawMesh = false;
		}

		// basic particles
		if (mScene == 1)
		{
			float radius = 0.1f;

			CreateParticleGrid(center - Vec3(2.0f, 7.0f, 2.0f) , 32, 64, 32, radius*1.02f, Vec3(0.0f), 1.0f, false, 0.0f, flexMakePhase(0, eFlexPhaseSelfCollide), 0.0f);
		
			g_params.mRadius = radius;
			g_params.mDynamicFriction = 0.1f;
			g_params.mDissipation = 0.0f;
			g_params.mNumIterations = 4;
			g_params.mNumPlanes = 1;
			g_params.mFluid = false;
			g_params.mParticleCollisionMargin = g_params.mRadius*0.1f;
			g_params.mRestitution = 0.0f;

			g_numSubsteps = 2;

			// draw options		
			g_drawPoints = true;
		}

		// fluid particles
		if (mScene == 2)
		{
			float radius = 0.1f;
			float restDistance = radius*0.6f;

			CreateParticleGrid(center - Vec3(0.0f, 7.0f, 2.0f) , 32, 64, 32, restDistance, Vec3(0.0f), 1.0f, false, 0.0f, flexMakePhase(0, eFlexPhaseSelfCollide | eFlexPhaseFluid), 0.0f);
		
			g_params.mRadius = radius;
			g_params.mDynamicFriction = 0.1f;
			g_params.mDissipation = 0.0f;
			g_params.mNumPlanes = 1;
			g_params.mFluidRestDistance = restDistance;
			g_params.mViscosity = 0.5f;
			g_params.mNumIterations = 3;
			g_params.mAnisotropyScale = 30.0f;
			g_params.mSmoothing = 0.5f;
			g_params.mFluid = true;
			g_params.mRelaxationFactor = 1.0f;
			g_params.mRestitution = 0.0f;
			g_params.mSmoothing = 0.5f;

			g_numSubsteps = 2;

			// draw options	
			g_drawPoints = false;
			g_drawEllipsoids = true;
			g_drawDiffuse = true;

			g_lightDistance = 5.0f;
		}
		
		// cloth
		if (mScene == 3)
		{
			float stretchStiffness = 1.0f;
			float bendStiffness = 0.8f;
			float shearStiffness = 0.8f;

			int dimx = 32;
			int dimz = 32;
			float radius = 0.05f;

			int gridx = 8;
			int gridz = 3;

			for (int x=0; x < gridx; ++x)
			{
				for (int y=0; y < 1; ++y)
				{
					for (int z=0; z < gridz; ++z)
					{
						CreateSpringGrid(center - Vec3(9.0f, 1.0f, 0.1f) + Vec3(x*dimx*radius, 0.0f, z*1.0f), dimx, dimz, 1, radius*0.95f, flexMakePhase(0, eFlexPhaseSelfCollide), stretchStiffness, bendStiffness, shearStiffness, 0.0f, 1.0f);
					}
				}
			}

			Vec3 l, u;
			GetParticleBounds(l, u);
			
			Vec3 center = (u+l)*0.5f;
			printf("%f %f %f\n", center.x, center.y, center.z);
		
			g_params.mRadius = radius*1.0f;
			g_params.mDynamicFriction = 0.4f;
			g_params.mStaticFriction = 0.5f;
			g_params.mDissipation = 0.0f;
			g_params.mNumIterations = 8;
			g_params.mDrag = 0.06f;
			g_params.mSleepThreshold = g_params.mRadius*0.125f;
			g_params.mRelaxationFactor = 2.0f;
			g_params.mCollisionDistance = g_params.mRadius*0.5f;

			g_windStrength = 0.0f;
		
			g_numSubsteps = 2;

			// draw options		
			g_drawPoints = false;
		}
	}

	int mScene;
};



class RockPool: public Scene
{
public:

	RockPool(const char* name) : Scene(name) {}

	void Initialize()
	{
		float radius = 0.1f;

		// convex rocks
		float minSize = 0.1f;
		float maxSize = 0.5f;

		for (int i=0; i < 4; i++)
			for (int j=0; j < 2; j++)
			CreateRandomConvex(10, Vec3(48*radius*0.5f + i*maxSize*2.0f, 0.0f, j*maxSize*2.0f), minSize, maxSize, Vec3(0.0f, 1.0f, 0.0f), Randf(0.0f, k2Pi));

		CreateParticleGrid(Vec3(0.0f, radius*0.5f, -1.0f), 32, 32, 32, radius*0.55f, Vec3(0.0f), 1.0f, false, 0.0f, flexMakePhase(0, eFlexPhaseSelfCollide | eFlexPhaseFluid), 0.005f);

		g_numSubsteps = 2;

		g_params.mRadius = radius;
		g_params.mDynamicFriction = 0.00f;
		g_params.mFluid = true;
		g_params.mViscosity = 0.01f;
		g_params.mNumIterations = 2;
		g_params.mVorticityConfinement = 75.0f;
		g_params.mAnisotropyScale = 30.0f;		
		g_params.mFluidRestDistance = radius*0.6f;
		g_params.mRelaxationFactor = 1.0f;
		g_params.mSmoothing = 0.5f;
		g_params.mDiffuseThreshold *= 0.25f;
		g_params.mCohesion = 0.05f;

		g_maxDiffuseParticles = 64*1024;		
		g_diffuseScale = 0.5f;
		g_params.mDiffuseBallistic = 16;
		g_params.mDiffuseBuoyancy = 1.0f;
		g_params.mDiffuseDrag = 1.0f;

		g_emitters[0].mEnabled = false;

		g_params.mNumPlanes = 5;

		g_waveFloorTilt = 0.0f;
		g_waveFrequency = 1.5f;
		g_waveAmplitude = 2.0f;
		
		// draw options		
		g_drawPoints = false;
		g_drawEllipsoids = true;
		g_drawDiffuse = true;
		g_lightDistance = 1.8f;

		g_numExtraParticles = 80*1024;

		Emitter e1;
		e1.mDir = Vec3(-1.0f, 0.0f, 0.0f);
		e1.mRight = Vec3(0.0f, 0.0f, 1.0f);
		e1.mPos = Vec3(3.8f, 1.f, 1.f) ;
		e1.mSpeed = (g_params.mFluidRestDistance/g_dt)*2.0f;	// 2 particle layers per-frame
		e1.mEnabled = true;

		Emitter e2;
		e2.mDir = Vec3(1.0f, 0.0f, 0.0f);
		e2.mRight = Vec3(0.0f, 0.0f, -1.0f);
		e2.mPos = Vec3(2.f, 1.f, -0.f);
		e2.mSpeed = (g_params.mFluidRestDistance/g_dt)*2.0f; // 2 particle layers per-frame
		e2.mEnabled = true;

		g_emitters.push_back(e1);
		g_emitters.push_back(e2);
	}
};

class GooGun : public Scene
{
public:

	GooGun(const char* name, bool viscous) : Scene(name), mViscous(viscous) {}

	virtual void Initialize()
	{
		float minSize = 0.5f;
		float maxSize = 1.0f;
		
		float radius = 0.1f;

		for (int i=0; i < 5; i++)
			CreateRandomConvex(10, Vec3(i*2.0f, 0.0f, Randf(0.0f, 2.0f)), minSize, maxSize, Vec3(0.0f, 1.0f, 0.0f), Randf(0.0f, k2Pi*10.0f));

		g_params.mRadius = radius;
		
		g_params.mFluid = true;
		g_params.mNumIterations = 3;
		g_params.mVorticityConfinement = 0.0f;
		g_params.mFluidRestDistance = g_params.mRadius*0.55f;
		g_params.mAnisotropyScale = 2.0f/radius;
		g_params.mSmoothing = 0.5f;
		g_params.mRelaxationFactor = 1.f;
		g_params.mRestitution = 0.0f;
		g_params.mCollisionDistance = 0.01f;
		g_params.mShapeCollisionMargin = g_params.mCollisionDistance*0.25f;

		if (mViscous)
		{
			g_fluidColor = Vec4(0.0f, 0.8f, 0.2f, 1.0f);

			g_params.mDynamicFriction = 1.0f;
			g_params.mViscosity = 50.0f;
			g_params.mAdhesion = 0.5f;
			g_params.mCohesion = 0.3f;
			g_params.mSurfaceTension = 0.0f;
		}
		else
		{
			g_params.mDynamicFriction = 0.25f;			
			g_params.mViscosity = 0.5f;			
			g_params.mCohesion = 0.05f;
			g_params.mAdhesion= 0.0f;
		}

		g_numExtraParticles = 64*1024;

		CreateSDF("../../data/armadillo.ply", 2.0f, Vec3(2.0f, 0.0f, -1.0f));

		g_emitters[0].mEnabled = true;
		g_emitters[0].mSpeed = (g_params.mFluidRestDistance*2.f/g_dt);

		// draw options		
		g_drawEllipsoids = true;
		g_pause = false;
	}

	bool mViscous;
};

class Adhesion : public Scene
{
public:

	Adhesion(const char* name) : Scene(name){}

	virtual void Initialize()
	{
		float radius = 0.1f;

		g_params.mRadius = radius;
		
		g_params.mFluid = true;
		g_params.mNumIterations = 3;
		g_params.mVorticityConfinement = 0.0f;
		g_params.mFluidRestDistance = g_params.mRadius*0.55f;
		g_params.mAnisotropyScale = 3.0f/radius;
		g_params.mSmoothing = 0.5f;
		g_params.mRelaxationFactor = 1.f;
		g_params.mRestitution = 0.0f;
		g_params.mCollisionDistance = 0.01f;

		g_fluidColor = Vec4(0.0f, 0.8f, 0.2f, 1.0f);
		//g_fluidColor = Vec4(0.7f, 0.6f, 0.6f, 0.2f);

		g_params.mDynamicFriction = 0.5f;
		g_params.mViscosity = 50.0f;
		g_params.mAdhesion = 0.5f;
		g_params.mCohesion = 0.08f;
		g_params.mSurfaceTension = 0.0f;

		g_numExtraParticles = 64*1024;

		CreateConvex(Vec3(1.0f, 6.0f, 0.1f));
		CreateConvex(Vec3(1.0f, 0.1f, 6.0f), Vec3(-1.0f, 3.0f, 0.0f));

		g_emitters[0].mEnabled = true;
		g_emitters[0].mSpeed = (g_params.mFluidRestDistance*2.f/g_dt);

		g_params.mNumPlanes = 3;

		// draw options		
		g_drawEllipsoids = true;

		g_pause = false;
	}

	bool mViscous;
};



class Viscosity : public Scene
{
public:

	Viscosity(const char* name, float viscosity=1.0f, float dissipation=0.0f) : Scene(name), mViscosity(viscosity), mDissipation(dissipation) {}

	virtual void Initialize()
	{
		float radius = 0.1f;
		float restDistance = radius*0.5f;

		g_params.mRadius = radius;
		
		g_params.mFluid = true;
		g_params.mNumIterations = 3;
		g_params.mVorticityConfinement = 0.0f;
		g_params.mFluidRestDistance = restDistance;
		g_params.mAnisotropyScale = 3.0f/radius;
		g_params.mSmoothing = 0.35f;
		g_params.mRelaxationFactor = 1.f;
		g_params.mRestitution = 0.0f;
		g_params.mCollisionDistance = 0.00125f;
		g_params.mShapeCollisionMargin = g_params.mCollisionDistance*0.25f;
		g_params.mDissipation = mDissipation;

		g_params.mGravity[1] *= 2.0f;

		g_fluidColor = Vec4(1.0f, 1.0f, 1.0f, 0.0f);
		g_meshColor = Vec3(0.7f, 0.8f, 0.9f)*0.7f;

		g_params.mDynamicFriction = 1.0f;
		g_params.mStaticFriction = 0.0f;
		g_params.mViscosity = 20.0f + 20.0f*mViscosity;
		g_params.mAdhesion = 0.1f*mViscosity;
		g_params.mCohesion = 0.05f*mViscosity;
		g_params.mSurfaceTension = 0.0f;	

		float shapeSize = 2.0f;
		CreateSDF("../../data/bunny.ply", shapeSize, Vec3(-shapeSize*0.5f, 0.0f, -shapeSize*0.5f), g_params.mCollisionDistance*0.125f);

		Vec3 lower, upper;
		g_mesh->GetBounds(lower, upper);
		Vec3 center = (upper+lower)*0.5f;

		float emitterSize = 1.f;

		Emitter e;
		e.mEnabled = true;
		e.mWidth = int(emitterSize/restDistance);
		e.mPos = Vec3(center.x-0.2f, upper.y + 0.75f, center.z);
		e.mDir = Vec3(0.0f, -1.0f, 0.0f);
		e.mRight = Vec3(1.0f, 0.0f, 0.0f);
		e.mSpeed = (restDistance*2.f/g_dt);
		
		g_sceneUpper.z = 5.0f;

		g_emitters.push_back(e);
				
		g_numExtraParticles = 64*1024;
		
		g_lightDistance *= 2.5f;

		// draw options		
		g_drawEllipsoids = true;

		g_emit = true;
		g_pause = false;
	}

	virtual void DoGui()
	{
		imguiSlider("Emitter Pos", &g_emitters.back().mPos.x, -1.0f, 1.0f, 0.001f);
	}

	float mViscosity;
	float mDissipation;
};


class Lighthouse: public Scene
{
public:

	Lighthouse(const char* name) : Scene(name) {}

	virtual void Initialize()
	{
		float radius = 0.15f;
		float restDistance = radius*0.6f;

		CreateSDF("../../data/lighthouse.ply", 10.0f, Vec3(4.0f, 0.0f, 0.0f));

		CreateParticleGrid(Vec3(0.0f, 0.3f, 0.0f), 48, 48, 128, restDistance, Vec3(0.0f), 1.0f, false, 0.0f, flexMakePhase(0, eFlexPhaseSelfCollide | eFlexPhaseFluid), 0.005f);

		g_sceneLower = 0.0f;
		g_sceneUpper = Vec3(12, 0.0f, 0.0f);

		g_numSubsteps = 2;

		g_params.mRadius = radius;
		g_params.mDynamicFriction = 0.f;
		g_params.mFluid = true;
		g_params.mViscosity = 0.01f;		
		g_params.mNumIterations = 3;
		g_params.mVorticityConfinement = 50.0f;
		g_params.mAnisotropyScale = 20.0f;
		g_params.mFluidRestDistance = restDistance;
		g_params.mGravity[1] *= 0.5f;
		g_params.mCohesion *= 0.5f;

		g_fluidColor = Vec4(0.413f, 0.725f, 0.85f, 0.7f);

		g_maxDiffuseParticles = 1024*1024;
		g_diffuseScale = 0.3f;
		g_diffuseShadow = true;
		g_diffuseColor = 1.0f;
		g_diffuseMotionScale = 1.0f;
		g_params.mDiffuseThreshold *= 10.f;
		g_params.mDiffuseBallistic = 4;
		g_params.mDiffuseBuoyancy = 2.0f;
		g_params.mDiffuseDrag = 1.0f;

		g_params.mNumPlanes = 5;

		g_waveFrequency = 1.2f;
		g_waveAmplitude = 2.2f;
		g_waveFloorTilt = 0.1f; 

		
		// draw options		
		g_drawPoints = false;
		g_drawEllipsoids = true;
		g_drawDiffuse = true;
	}
};


class DamBreak: public Scene
{
public:

	DamBreak(const char* name, float radius) : Scene(name), mRadius(radius) {}

	virtual void Initialize()
	{
		const float radius = mRadius;
		const float restDistance = mRadius*0.65f;

		int dx = int(ceilf(1.0f / restDistance));
		int dy = int(ceilf(2.0f / restDistance));
		int dz = int(ceilf(1.0f / restDistance));

		CreateParticleGrid(Vec3(0.0f, restDistance, 0.0f), dx, dy, dz, restDistance, Vec3(0.0f), 1.0f, false, 0.0f, flexMakePhase(0, eFlexPhaseSelfCollide | eFlexPhaseFluid), restDistance*0.01f);

		g_sceneLower = Vec3(0.0f, 0.0f, -0.5f);
		g_sceneUpper = Vec3(3.0f, 0.0f, -0.5f);

		g_numSubsteps = 2;

		g_params.mFluid = true;
		g_params.mRadius = radius;
		g_params.mFluidRestDistance = restDistance;
		g_params.mDynamicFriction = 0.f;
		g_params.mRestitution = 0.001f;

		g_params.mNumIterations = 3;
		g_params.mRelaxationFactor = 1.0f;

		g_params.mSmoothing = 0.4f;
		g_params.mAnisotropyScale = 3.0f/radius;

		g_params.mViscosity = 0.001f;
		g_params.mCohesion = 0.1f;
		g_params.mVorticityConfinement = 80.0f;
		g_params.mSurfaceTension = 0.0f;
		
		g_params.mNumPlanes = 5;
		
		// limit velocity to CFL condition
		g_params.mMaxVelocity = 0.5f*radius*g_numSubsteps/g_dt;
		
		g_maxDiffuseParticles = 0;

		g_fluidColor = Vec4(0.113f, 0.425f, 0.55f, 1.0f);

		g_waveFrequency = 1.0f;
		g_waveAmplitude = 2.0f;
		g_waveFloorTilt = 0.0f; 
	
		// draw options		
		g_drawPoints = true;
		g_drawEllipsoids = false;
		g_drawDiffuse = true;
	}

	float mRadius;

};


class Pasta: public Scene
{
public:

	Pasta(const char* name) : Scene(name) {}

	virtual void Initialize()
	{
		float radius = 0.1f;
		float length = 15.0f;
		int n = 20;

		for (int i=0; i < n; ++i)
		{
			float theta = k2Pi*float(i)/n;

			Rope r;
			CreateRope(r, 0.5f*Vec3(cosf(theta), 2.0f, sinf(theta)), Vec3(0.0f, 1.0f, 0.0f), 0.25f, int(length/radius), length, flexMakePhase(0, eFlexPhaseSelfCollide));
			g_ropes.push_back(r);
		}

		g_numSubsteps = 3;

		g_staticMesh = ImportMeshFromObj("../../data/bowl.obj");
		g_staticMesh->Normalize(2.0f);
		g_staticMesh->CalculateNormals();
		g_staticMesh->Transform(TranslationMatrix(Point3(-1.0f, 0.0f, -1.0f)));

		g_params.mNumIterations = 6;
		g_params.mRadius = radius;
		g_params.mDynamicFriction = 0.4f;
		g_params.mDissipation = 0.001f;
		g_params.mSleepThreshold = g_params.mRadius*0.2f;
		g_params.mRelaxationFactor = 1.3f;
		g_params.mRestitution = 0.0f;

		g_lightDistance *= 0.5f;
		g_drawPoints = false;
	}
};



class NonConvex : public Scene
{
public:

	NonConvex(const char* name) : Scene(name)
	{
	}
	
	virtual void Initialize()
	{
		float radius = 0.15f;
		int group = 0;

		for (int i=0; i < 1; ++i)
			CreateParticleShape("../../data/bowl.obj", Vec3(0.0f, 1.0f + 0.5f*i + radius*0.5f, 0.0f), Vec3(1.5f), 0.0f, radius*0.8f, Vec3(0.0f), 1.0f, true, 1.0f, flexMakePhase(group++, 0), true, 0.0f, Vec3(0.0f));

		for (int i=0; i < 50; ++i)
			CreateParticleShape("../../data/banana.obj", Vec3(0.4f, 2.5f + i*0.25f, 0.25f) + RandomUnitVector()*radius*0.25f, Vec3(1), 0.0f, radius, Vec3(0.0f), 1.0f, true, 0.5f, flexMakePhase(group++, 0), true, radius*0.1f, 0.0f, 0.0f, 1.25f*Vec4(0.875f, 0.782f, 0.051f, 1.0f));		
				
		CreateConvex();

		g_numSubsteps = 3;
		g_params.mNumIterations = 3;

		g_params.mRadius *= 1.0f;
		g_params.mDynamicFriction = 0.35f;
		g_params.mDissipation = 0.0f;
		g_params.mParticleCollisionMargin = g_params.mRadius*0.05f;
		g_params.mSleepThreshold = g_params.mRadius*0.2f;
		g_params.mShockPropagation = 3.0f;
		g_params.mGravity[1] *= 1.0f;
		g_params.mRestitution = 0.01f;
		g_params.mDamping = 0.25f;

		// draw options
		g_drawPoints = false;

		g_emitters[0].mEnabled = true;
		g_emitters[0].mSpeed = (g_params.mRadius*2.0f/g_dt);
	}

	virtual void Update()
	{
			
	}
};



class BananaPile : public Scene
{
public:

	BananaPile(const char* name) : Scene(name)
	{
	}
	
	virtual void Initialize()
	{
		float s = 1.0f;

		Vec3 lower(0.0f, 1.0f + g_params.mRadius*0.25f, 0.0f);

		int dimx = 3;
		int dimy = 40;
		int dimz = 2;

		float radius = g_params.mRadius;
		int group = 0;
		
		// create a basic grid
		for (int x=0; x < dimx; ++x)
		{
			for (int y=0; y < dimy; ++y)
			{
				for (int z=0; z < dimz; ++z)
				{
					CreateParticleShape("../../data/banana.obj", lower + (s*1.1f)*Vec3(float(x), y*0.4f, float(z)), Vec3(s), 0.0f, radius*0.95f, Vec3(0.0f), 1.0f, true, 0.8f, flexMakePhase(group++, 0), true, radius*0.1f, 0.0f, 0.0f, 1.25f*Vec4(0.875f, 0.782f, 0.051f, 1.0f));		
				}
			}
		}		

		// plinth
		CreateConvex();

		g_numSubsteps = 3;
		g_params.mNumIterations = 2;

		g_params.mRadius *= 1.0f;
		g_params.mDynamicFriction = 0.25f;
		g_params.mDissipation = 0.03f;
		g_params.mParticleCollisionMargin = g_params.mRadius*0.05f;
		g_params.mSleepThreshold = g_params.mRadius*0.2f;
		g_params.mShockPropagation = 2.5f;
		g_params.mRestitution = 0.55f;
		g_params.mDamping = 0.25f;

		// draw options
		g_drawPoints = false;

		g_emitters[0].mEnabled = true;
		g_emitters[0].mSpeed = (g_params.mRadius*2.0f/g_dt);
	}

	virtual void Update()
	{
			
	}
};

class Tearing : public Scene
{
public:

	Tearing(const char* name) : Scene(name) {}

	void Initialize()
	{
		Mesh* mesh = ImportMesh("../../data/irregular_plane.obj");
		mesh->Transform(RotationMatrix(kPi, Vec3(0.0f, 1.0f, 0.0f))*RotationMatrix(kPi*0.5f, Vec3(1.0f, 0.0f, 0.0f))*ScaleMatrix(2.0f));

		Vec3 lower, upper;
		mesh->GetBounds(lower, upper);

		float radius = 0.035f;
		int phase = flexMakePhase(0, eFlexPhaseSelfCollide);
		
		for (size_t i=0; i < mesh->GetNumVertices(); ++i)
		{
			Vec3 p = Vec3(mesh->m_positions[i]);

			float invMass = 1.0f;

			if (p.y == upper.y)
				invMass = 0.0f;

			p += Vec3(0.0f, 1.5f, 0.0f);

			g_positions.push_back(Vec4(p.x, p.y, p.z, invMass));
			g_velocities.push_back(0.0f);
			g_phases.push_back(phase);
		}

		mCloth = new ClothMesh(&g_positions[0], mesh->GetNumVertices(), (int*)&mesh->m_indices[0], mesh->GetNumFaces()*3, 0.8f, 0.8f);

		g_numExtraParticles = 1000;

		g_triangles.assign(mesh->m_indices.begin(), mesh->m_indices.end());
		g_triangleNormals.resize(mesh->GetNumFaces(), Vec3(0.0f, 0.0f, 1.0f));
		
		g_springIndices = mCloth->mConstraintIndices;
		g_springStiffness = mCloth->mConstraintCoefficients;
		g_springLengths = mCloth->mConstraintRestLengths;
		
		g_params.mRadius = radius;
		g_params.mDynamicFriction = 0.025f;
		g_params.mDissipation = 0.0f;
		g_params.mNumIterations = 32;
		g_params.mParticleCollisionMargin = g_params.mRadius*0.05f;
		g_params.mRelaxationFactor = 1.0f;
		g_params.mDrag = 0.03f;
				
		g_numSubsteps = 2;

		// draw options		
		g_drawPoints = false;		
	}

	void Update()
	{
		g_params.mWind[0] = 0.1f;
		g_params.mWind[1] = 0.1f;
		g_params.mWind[2] = -0.2f;	
		g_windStrength= 6.0f;

		Vec3 splitPlane;
		int splitIndex = -1;
		int splitCount = 0;
		
		for (size_t i=0; i < g_springLengths.size() && splitCount < 4; ++i)
		{
			int a = g_springIndices[i*2+0];
			int b = g_springIndices[i*2+1];

			Vec3 p = Vec3(g_positions[a]);
			Vec3 q = Vec3(g_positions[b]);

			// check strain and break if greather than max threshold
			if (Length(p-q) > g_springLengths[i]*3.0f)
			{
				splitIndex = Randf() > 0.5f ? a : b;
				splitPlane = Normalize(p-q);

				if (Split(splitIndex, splitPlane))
					splitCount++;
			}
		}

	}

	bool Split(int splitIndex, Vec3 splitPlane)
	{
		int newIndex = flexGetActiveCount(g_flex);

		printf("Splitting: %d NewIndex: %d Plane: %f %f %f\n", splitIndex, newIndex, splitPlane.x, splitPlane.y, splitPlane.z);

		if (mCloth->SplitVertex(&g_positions[0], newIndex, splitIndex, splitPlane))
		{				

			g_positions[newIndex] = g_positions[splitIndex];
			g_velocities[newIndex] = g_velocities[splitIndex];
			g_phases[newIndex ] = g_phases[splitIndex];

			g_triangles.resize(0);

			for (size_t i=0; i < mCloth->mTris.size(); ++i)
			{
				g_triangles.insert(g_triangles.end(), mCloth->mTris[i].vertices, mCloth->mTris[i].vertices+3);
			}

			g_springIndices = mCloth->mConstraintIndices;
			g_springStiffness = mCloth->mConstraintCoefficients;
			g_springLengths = mCloth->mConstraintRestLengths;

			flexSetSprings(g_flex, &g_springIndices[0], &g_springLengths[0], &g_springStiffness[0], g_springLengths.size(), eFlexMemoryHost);
			flexSetDynamicTriangles(g_flex, &g_triangles[0], &g_triangleNormals[0].x, g_triangles.size()/3, eFlexMemoryHost);
			
			flexSetParticles(g_flex, &g_positions[0].x, g_positions.size(), eFlexMemoryHost);
			flexSetVelocities(g_flex, &g_velocities[0].x, g_velocities.size(), eFlexMemoryHost);			
			flexSetPhases(g_flex, &g_phases[0], g_phases.size(), eFlexMemoryHost);

			flexSetActive(g_flex, &g_activeIndices[0], newIndex+1, eFlexMemoryHost);

			return true;
		}

		return false;
	}

	ClothMesh* mCloth;
};




class Inflatable : public Scene
{
public:

	Inflatable(const char* name) : Scene(name) {}
	
	virtual ~Inflatable()
	{
		for (size_t i=0; i < mCloths.size(); ++i)
			delete mCloths[i];
	}

	void AddInflatable(const Mesh* mesh, float overPressure, int phase)
	{
		const int startVertex = g_positions.size();

		// add mesh to system
		for (size_t i=0; i < mesh->GetNumVertices(); ++i)
		{
			const Vec3 p = Vec3(mesh->m_positions[i]);
				
			g_positions.push_back(Vec4(p.x, p.y, p.z, 1.0f));
			g_velocities.push_back(0.0f);
			g_phases.push_back(phase);
		}

		int triOffset = g_triangles.size();
		int triCount = mesh->GetNumFaces();

		mTriOffset.push_back(triOffset/3);
		mTriCount.push_back(mesh->GetNumFaces());
		mOverPressure.push_back(overPressure);

		for (size_t i=0; i < mesh->m_indices.size(); i+=3)
		{
			int a = mesh->m_indices[i+0];
			int b = mesh->m_indices[i+1];
			int c = mesh->m_indices[i+2];

			Vec3 n = -Normalize(Cross(mesh->m_positions[b]-mesh->m_positions[a], mesh->m_positions[c]-mesh->m_positions[a]));
			g_triangleNormals.push_back(n);

			g_triangles.push_back(a + startVertex);
			g_triangles.push_back(b + startVertex);
			g_triangles.push_back(c + startVertex);
		}

		// create a cloth mesh using the global positions / indices
		ClothMesh* cloth = new ClothMesh(&g_positions[0], g_positions.size(), &g_triangles[triOffset],triCount*3, 0.8f, 1.0f);

		for (size_t i=0; i < cloth->mConstraintIndices.size(); ++i)
			g_springIndices.push_back(cloth->mConstraintIndices[i]);

		g_springStiffness.insert(g_springStiffness.end(), cloth->mConstraintCoefficients.begin(), cloth->mConstraintCoefficients.end());
		g_springLengths.insert(g_springLengths.end(), cloth->mConstraintRestLengths.begin(), cloth->mConstraintRestLengths.end());

		mCloths.push_back(cloth);

		// add inflatable params
		mRestVolume.push_back(cloth->mRestVolume);
		mConstraintScale.push_back(cloth->mConstraintScale);
	}

	void Initialize()
	{
		mCloths.resize(0);
		mRestVolume.resize(0);
		mTriOffset.resize(0);
		mTriCount.resize(0);
		mOverPressure.resize(0);
		mConstraintScale.resize(0);
		mSplitThreshold.resize(0);

		float minSize = 0.75f;
		float maxSize = 1.0f;

		//// convex rocks
		//for (int i=0; i < 5; i++)
		//	CreateRandomConvex(10, Vec3(i*2.0f, 0.0f, Randf(0.0f, 2.0f)), minSize, maxSize, Vec3(0.0f, 1.0f, 0.0f), Randf(0.0f, k2Pi));
		
		float radius = 0.12f;
		int group = 0;

		const char* meshes[2] = 
		{ 
			"../../data/box_high_weld.ply",
			"../../data/sphere.ply"
		};

		mPressure = 1.0f;

		//for (int y=0; y < 2; ++y)
		//{
		//	for (int i=0; i < 4; ++i)
		//	{
		//		Mesh* mesh = ImportMesh(meshes[(i+y)&1]);
		//		mesh->Normalize();
		//		mesh->Transform(TranslationMatrix(Point3(i*2.0f, 1.0f + y*2.0f, 1.5f)));
		//		AddInflatable(mesh, mPressure, flexMakePhase(group++, 0));
		//		delete mesh;
		//	}
		//}

		Mesh* mesh = ImportMesh(meshes[0]);
		mesh->Normalize();
		mesh->Transform(TranslationMatrix(Point3(0.0, 1.0, 0.0)));

		AddInflatable(mesh, mPressure, flexMakePhase(group++, 0));

		delete mesh;

		//g_numExtraParticles = 20000;
		//
		//g_params.mRadius = radius;
		//g_params.mDynamicFriction = 0.4f;
		//g_params.mDissipation = 0.0f;
		//g_params.mNumIterations = 10;
		//g_params.mParticleCollisionMargin = g_params.mRadius*0.05f;
		//g_params.mDrag = 0.0f;
		//g_params.mCollisionDistance = 0.01f;

		//// better convergence with global relaxation factor
		//g_params.mRelaxationMode = eFlexRelaxationGlobal;
		//g_params.mRelaxationFactor = 0.25f;

		//g_windStrength = 0.0f;

		//g_numSubsteps = 2;

		//mSplitThreshold.resize(mCloths.size(), 45.0f);


		g_numExtraParticles = 64 * 1024;

		g_params.mRadius = radius;
		g_params.mFluid = true;
		g_params.mNumIterations = 5;
		g_params.mVorticityConfinement = 0.0f;
		g_params.mAnisotropyScale = 30.0f;
		g_params.mFluidRestDistance = g_params.mRadius*0.5f;
		g_params.mSmoothing = 0.5f;
		g_params.mSolidPressure = 0.25f;
		g_numSubsteps = 3;
		g_params.mNumIterations = 6;

		g_params.mMaxVelocity = 0.5f*g_numSubsteps*g_params.mRadius / g_dt;

		g_maxDiffuseParticles = 32 * 1024;
		g_diffuseScale = 0.5f;
		g_lightDistance = 3.0f;
		g_pointScale = 0.5f;

		g_fluidColor = Vec4(0.0f, 0.8f, 0.2f, 1.0f);

//		g_params.mDynamicFriction = 0.3f;
		g_params.mCohesion = 0.025f;
		g_params.mViscosity = 50.85f;
		

		g_emitters[0].mEnabled = false;

		Emitter e;
		e.mDir = Normalize(Vec3(1.0f, 0.0f, 0.0f));
		e.mEnabled = true;
		e.mPos = Vec3(-0.25f, 1.75f, 0.5f);
		e.mRight = Cross(e.mDir, Vec3(0.0f, 0.0f, 1.0f));
		e.mSpeed = (g_params.mFluidRestDistance / (g_dt*2.0f));

		g_emitters.push_back(e);

		// draw options		
		g_drawPoints = false;
		g_drawSprings = 0;
		g_drawCloth = false;
	}

	void PostInitialize()
	{
		flexSetInflatables(g_flex, &mTriOffset[0], &mTriCount[0], &mRestVolume[0], &mOverPressure[0], &mConstraintScale[0], mCloths.size(), eFlexMemoryHost);
	}

	virtual void DoGui()
	{

		imguiLabel("Scene");

		if (imguiSlider("Over Pressure", &mPressure, 0.25f, 3.0f, 0.001f))
		{
			for (int i=0; i < int(mOverPressure.size()); ++i)
				mOverPressure[i] = mPressure;

			flexSetInflatables(g_flex, &mTriOffset[0], &mTriCount[0], &mRestVolume[0], &mOverPressure[0], &mConstraintScale[0], mCloths.size(), eFlexMemoryHost);
		}


		imguiSeparatorLine();
	}
	
	virtual void Draw(int pass)
	{
		if (!g_drawMesh)
			return;

		int indexStart = 0;

		for (size_t i=0; i < mCloths.size(); ++i)
		{
			DrawCloth(&g_positions[0], &g_normals[0], NULL, &g_triangles[indexStart], mCloths[i]->mTris.size(), g_positions.size(), i%6, g_params.mRadius*0.35f);

			indexStart += mCloths[i]->mTris.size()*3;
		}
	}

	float mPressure;

	std::vector<ClothMesh*> mCloths;
	std::vector<float> mRestVolume;
	std::vector<int> mTriOffset;
	std::vector<int> mTriCount;
	std::vector<float> mOverPressure;
	std::vector<float> mConstraintScale;
	std::vector<float> mSplitThreshold;
};



class WaterBalloon : public Scene
{
public:

	WaterBalloon(const char* name) : Scene(name) {}
	
	virtual ~WaterBalloon()
	{
		for (size_t i=0; i < mCloths.size(); ++i)
			delete mCloths[i];
	}

	void AddInflatable(const Mesh* mesh, float overPressure, float invMass, int phase)
	{
		const int startVertex = g_positions.size();

		// add mesh to system
		for (size_t i=0; i < mesh->GetNumVertices(); ++i)
		{
			const Vec3 p = Vec3(mesh->m_positions[i]);
				
			g_positions.push_back(Vec4(p.x, p.y, p.z, invMass));
			g_velocities.push_back(0.0f);
			g_phases.push_back(phase);
		}

		int triOffset = g_triangles.size();
		int triCount = mesh->GetNumFaces();

		mTriOffset.push_back(triOffset/3);
		mTriCount.push_back(mesh->GetNumFaces());
		mOverPressure.push_back(overPressure);

		for (size_t i=0; i < mesh->m_indices.size(); i+=3)
		{
			int a = mesh->m_indices[i+0];
			int b = mesh->m_indices[i+1];
			int c = mesh->m_indices[i+2];

			Vec3 n = -Normalize(Cross(mesh->m_positions[b]-mesh->m_positions[a], mesh->m_positions[c]-mesh->m_positions[a]));
			g_triangleNormals.push_back(n);

			g_triangles.push_back(a + startVertex);
			g_triangles.push_back(b + startVertex);
			g_triangles.push_back(c + startVertex);
		}

		// create a cloth mesh using the global positions / indices
		ClothMesh* cloth = new ClothMesh(&g_positions[0], g_positions.size(), &g_triangles[triOffset],triCount*3, 1.0f, 1.0f);

		for (size_t i=0; i < cloth->mConstraintIndices.size(); ++i)
			g_springIndices.push_back(cloth->mConstraintIndices[i]);

		g_springStiffness.insert(g_springStiffness.end(), cloth->mConstraintCoefficients.begin(), cloth->mConstraintCoefficients.end());
		g_springLengths.insert(g_springLengths.end(), cloth->mConstraintRestLengths.begin(), cloth->mConstraintRestLengths.end());

		mCloths.push_back(cloth);

		// add inflatable params
		mRestVolume.push_back(cloth->mRestVolume);
		mConstraintScale.push_back(cloth->mConstraintScale);
	}

	void Initialize()
	{
		mCloths.resize(0);
		mRestVolume.resize(0);
		mTriOffset.resize(0);
		mTriCount.resize(0);
		mOverPressure.resize(0);
		mConstraintScale.resize(0);
		mSplitThreshold.resize(0);

		float minSize = 0.25f;
		float maxSize = 0.5f;
		float spacing = 4.0f;

		// convex rocks
		for (int i=0; i < 4; i++)
			for (int j=0; j < 1; j++)
			CreateRandomConvex(10, Vec3(i*maxSize*spacing, 0.0f, j*maxSize*spacing), minSize, maxSize, Vec3(0.0f, 1.0f, 0.0f), Randf(0.0f, k2Pi));

		float radius = 0.1f;
		int group = 0;

		g_numExtraParticles = 20000;
		g_numSubsteps = 3;
		
		g_params.mRadius = radius;
		g_params.mDynamicFriction = 0.125f;
		g_params.mDissipation = 0.0f;
		g_params.mNumIterations = 5;
		g_params.mParticleCollisionMargin = g_params.mRadius*0.05f;
		g_params.mRelaxationFactor = 1.5f;
		g_params.mDrag = 0.0f;
		g_params.mAnisotropyScale = 25.0f;
		g_params.mSmoothing = 1.f;
		g_params.mMaxVelocity = 0.5f*g_numSubsteps*radius/g_dt;
		g_params.mGravity[1] *= 1.0f;
		g_params.mCollisionDistance = 0.005f;
		g_params.mSolidPressure = 0.0f;

		g_params.mFluid = true;				
	
		g_params.mFluidRestDistance = radius*0.65f;
		g_params.mViscosity = 0.0;
		g_params.mAdhesion = 0.0f;
		g_params.mCohesion = 0.02f;
		

		// add inflatables
		Mesh* mesh = ImportMesh("../../data/sphere_high.ply");

		for (int y=0; y < 2; ++y)
		for (int i=0; i < 2; ++i)
		{
			Vec3 lower = Vec3(2.0f + i*2.0f, 0.4f + y*1.2f, 1.0f);

			mesh->Normalize();
			mesh->Transform(TranslationMatrix(Point3(lower)));
			
			AddInflatable(mesh, 1.0f, 0.25f, flexMakePhase(group++, 0));

			mSplitThreshold.push_back(5.0f);
		}
		
		g_numSolidParticles = g_positions.size();

		// fill inflatables with water
		std::vector<Vec3> positions(10000);
		int n = PoissonSample3D(0.45f, g_params.mRadius*0.42f, &positions[0], positions.size(), 10000);
		//int n = TightPack3D(0.45f, g_params.mRadius*0.42f, &positions[0], positions.size());

		for (size_t i=0; i < mCloths.size(); ++i)
		{
			const int vertStart = i*mesh->GetNumVertices();
			const int vertEnd = vertStart + mesh->GetNumVertices();
			
			const int phase = flexMakePhase(group++, eFlexPhaseSelfCollide | eFlexPhaseFluid);

			Vec3 center;
			for (int v=vertStart; v < vertEnd; ++v)
				center += Vec3(g_positions[v]);

			center /= float(vertEnd-vertStart);

			printf("%d, %d - %f %f %f\n", vertStart, vertEnd, center.x, center.y, center.z);

			for (int i=0; i < n; ++i)
			{
				g_positions.push_back(Vec4(center + positions[i], 1.0f));
				g_velocities.push_back(0.0f);
				g_phases.push_back(phase);
			}
		}

		delete mesh;

		g_drawPoints = false;
		g_drawEllipsoids = true;
		g_drawSprings = 0;
		g_drawCloth = false;
		g_warmup = true;

	}

	void Update()
	{
		// force larger radius for solid interactions to prevent interpenetration
		g_params.mSolidRestDistance = g_params.mRadius;

		int active = flexGetActiveCount(g_flex);

		int splitCount = 0;
		int springStart = 0;

		for (size_t clothIndex=0; clothIndex < mCloths.size(); ++clothIndex)
		{
			float splitThreshold = mSplitThreshold[clothIndex];

			int springEnd = springStart + mCloths[clothIndex]->mConstraintRestLengths.size();

			for (int i=springStart; i < springEnd; ++i)
			{
				int a = g_springIndices[i*2+0];
				int b = g_springIndices[i*2+1];

				Vec3 p = Vec3(g_positions[a]);
				Vec3 q = Vec3(g_positions[b]);

				if (Length(p-q) > g_springLengths[i]*splitThreshold)
				{
					// choose one of the distance constraint's particles randomly
					int splitIndex = Randf() > 0.5f ? a : b;
					Vec3 splitPlane = Normalize(p-q);

					if (Split(clothIndex, splitIndex, active + splitCount, splitPlane))
					{				
						splitCount++;

						// weaken springs after first split
						//mPressure = 0.0f;
						//mSplitThreshold = 2.0f;
					
						// disable pressure constraint
						mConstraintScale[clothIndex] = 0.0f;

						if (splitCount == 1)
						{
							ClothMesh* cloth = mCloths[clothIndex];

							// add fake 'pop' velocity
							for (size_t t=0; t < cloth->mTris.size(); ++t)
							{
								int a = cloth->mTris[t].vertices[0];
								int b = cloth->mTris[t].vertices[1];
								int c = cloth->mTris[t].vertices[2];

								float kSize = 0.f;

								Vec3 n = kSize*Cross(Vec3(g_positions[b])-Vec3(g_positions[a]), Vec3(g_positions[c])-Vec3(g_positions[a]));

								g_velocities[a] += n;
								g_velocities[b] += n;
								g_velocities[c] += n;

								/*
								// make particles self collide once popped
								g_phases[a] = 0;
								g_phases[b] = 0;
								g_phases[c] = 0;
								*/
								
							}

							//mSplitThreshold[clothIndex] = 1.7f;
							splitThreshold = 1.5f;
						}
					}				
				}
			}

			springStart = springEnd;
		}

		//flexSetInflatables(g_flex, &mTriOffset[0], &mTriCount[0], &mRestVolume[0], &mOverPressure[0], &mConstraintScale[0], mCloths.size());

		if (splitCount)
		{
			g_triangles.resize(0);
			g_springIndices.resize(0);
			g_springStiffness.resize(0);
			g_springLengths.resize(0);

			for (size_t c=0; c < mCloths.size(); ++c)
			{
				ClothMesh* cloth = mCloths[c];

				for (size_t i=0; i < cloth->mTris.size(); ++i)
				{
					g_triangles.push_back(cloth->mTris[i].vertices[0]);
					g_triangles.push_back(cloth->mTris[i].vertices[1]);
					g_triangles.push_back(cloth->mTris[i].vertices[2]);
				}

				for (size_t i=0; i < cloth->mConstraintIndices.size(); ++i)
					g_springIndices.push_back(cloth->mConstraintIndices[i]);
				
				g_springStiffness.insert(g_springStiffness.end(), cloth->mConstraintCoefficients.begin(), cloth->mConstraintCoefficients.end());
				g_springLengths.insert(g_springLengths.end(), cloth->mConstraintRestLengths.begin(), cloth->mConstraintRestLengths.end());
			}

			// update solver
			flexSetSprings(g_flex, &g_springIndices[0], &g_springLengths[0], &g_springStiffness[0], g_springLengths.size(), eFlexMemoryHost);
			flexSetDynamicTriangles(g_flex, &g_triangles[0], &g_triangleNormals[0].x, g_triangles.size()/3, eFlexMemoryHost);
			
			flexSetParticles(g_flex, &g_positions[0].x, g_positions.size(), eFlexMemoryHost);
			flexSetVelocities(g_flex, &g_velocities[0].x, g_velocities.size(), eFlexMemoryHost);			
			flexSetPhases(g_flex, &g_phases[0], g_phases.size(), eFlexMemoryHost);

			flexSetActive(g_flex, &g_activeIndices[0], active + splitCount, eFlexMemoryHost);
		}
	}

	bool Split(int clothIndex, int splitIndex, int newIndex, Vec3 splitPlane)
	{
		//printf("Splitting: %d NewIndex: %d Plane: %f %f %f\n", splitIndex, newIndex, splitPlane.x, splitPlane.y, splitPlane.z);

		if (mCloths[clothIndex]->SplitVertex(&g_positions[0], newIndex, splitIndex, splitPlane))
		{	
			// clone vertex
			g_positions[newIndex] = g_positions[splitIndex];
			g_velocities[newIndex] = g_velocities[splitIndex];
			g_phases[newIndex ] = g_phases[splitIndex];

			return true;
		}

		return false;
	}

	virtual void Draw(int pass)
	{
		if (!g_drawMesh)
			return;

		int indexStart = 0;

		for (size_t i=0; i < mCloths.size(); ++i)
		{
			DrawCloth(&g_positions[0], &g_normals[0], NULL, &g_triangles[indexStart], mCloths[i]->mTris.size(), g_positions.size(), (i+2)%6, g_params.mRadius*0.25f);

			indexStart += mCloths[i]->mTris.size()*3;
		}
	}

	std::vector<ClothMesh*> mCloths;
	std::vector<float> mRestVolume;
	std::vector<int> mTriOffset;
	std::vector<int> mTriCount;
	std::vector<float> mOverPressure;
	std::vector<float> mConstraintScale;
	std::vector<float> mSplitThreshold;
};



class SurfaceTension : public Scene
{
public:

	SurfaceTension(const char* name, float surfaceTension) : Scene(name), mSurfaceTension(surfaceTension) {}

	virtual void Initialize()
	{
		mCounter = 0;
		
		float radius = 0.1f;
		float restDistance = radius*0.55f;
		int phase = flexMakePhase(0, eFlexPhaseSelfCollide | eFlexPhaseFluid);

		CreateParticleGrid(Vec3(0.0f, 0.2f, -1.0f), 16, 64, 16, restDistance, Vec3(0.0f), 1.0f, false, 0.0f, phase, 0.005f);

		g_params.mRadius = radius;
		
		g_params.mFluid = true;
		g_params.mNumIterations = 3;
		g_params.mVorticityConfinement = 0.0f;
		g_params.mFluidRestDistance = restDistance;
		g_params.mAnisotropyScale = 2.5f/radius;
		g_params.mSmoothing = 0.5f;
		g_params.mRelaxationFactor = 1.f;
		g_params.mRestitution = 0.0f;
		g_params.mCollisionDistance = 0.01f;

		g_params.mDynamicFriction = 0.25f;			
		g_params.mViscosity = 0.5f;			
		g_params.mCohesion = 0.1f;
		g_params.mAdhesion= 0.0f;
		g_params.mSurfaceTension = mSurfaceTension;

		g_params.mGravity[1] = 0.0f;

		g_numExtraParticles = 64*1024;

		g_emitters[0].mEnabled = true;
		g_emitters[0].mSpeed = (g_params.mFluidRestDistance*2.f/g_dt);

		g_lightDistance *= 2.0f;

		// draw options		
		g_drawEllipsoids = true;
	}

	void Update()
	{
		if (g_frame == 300)
			g_params.mGravity[1] = -9.8f;
	}

	int mCounter;
	float mSurfaceTension;
};

class Melting : public Scene
{
public:

	Melting(const char* name) : Scene(name) {}

	virtual void Initialize()
	{
		g_params.mRadius = 0.1f;
		
		g_params.mNumIterations = 2;
		g_params.mDynamicFriction = 0.25f;
		g_params.mDissipation = 0.0f;
		g_params.mViscosity = 0.0f;	
		g_params.mFluid = true;
		g_params.mCohesion = 0.0f;
		g_params.mFluidRestDistance = g_params.mRadius*0.6f;
		g_params.mAnisotropyScale = 4.0f/g_params.mRadius;
		g_params.mSmoothing = 0.5f;

		const float spacing = g_params.mRadius*0.5f;

		const char* mesh = "../../data/bunny.ply";

		int phase = flexMakePhase(0, eFlexPhaseSelfCollide | eFlexPhaseFluid);
		float size = 1.2f;

		for (int i=0; i < 1; ++i)
			for (int j=0; j < 3; ++j)
				CreateParticleShape(mesh, Vec3(-2.0f + j*size, 3.0f + j*size, i*size), size, 0.0f, spacing, Vec3(0.0f, 0.0f, 0.0f), 1.0f, true, 1.f, phase, false, 0.0f);			

		// plinth
		CreateConvex(2.0f);
		g_convexPositions[0] += Vec4(0.0f, 1.0f, 0.0f, 0.0f);

		g_numSubsteps = 2;
		
		// draw options		
		g_drawPoints = true;		
		g_drawMesh = false;

		mFrame = 0;
	}

	void Update()
	{
		const int start = 130;

		if (mFrame >= start)
		{
			float stiffness = max(0.0f, 1.0f - (mFrame-start) / 100.0f);

			for (size_t i=0; i < g_rigidCoefficients.size(); ++i)
				g_rigidCoefficients[i] = stiffness;

			flexSetRigids(g_flex, &g_rigidOffsets[0], &g_rigidIndices[0], (float*)&g_rigidLocalPositions[0], (float*)&g_rigidLocalNormals[0], &g_rigidCoefficients[0], (float*)&g_rigidRotations[0], g_rigidOffsets.size()-1, eFlexMemoryHost);	

			g_params.mCohesion = Lerp(0.05f, 0.0f, stiffness);
			g_params.mFluidRestDistance = Lerp(g_params.mRadius*0.6f, g_params.mRadius*0.25f, stiffness);
		}

		++mFrame;
	}

	int mFrame;
};

class Darts : public Scene
{
public:

	Darts(const char* name) : Scene(name) {}
	
	void Initialize()
	{
		float radius = 0.1f;
		int phase = flexMakePhase(0, 0);
		
		if (1)
		{
			Vec3 v = Vec3(10.0f, 0.0f, 0.0f);

			float y = 8.0f;

			g_positions.push_back(Vec4(0.0f, y, 0.0f, 1.0f));
			g_velocities.push_back(v);
			g_phases.push_back(phase);

			g_positions.push_back(Vec4(-1.0f, y, -0.5f, 0.9f));
			g_velocities.push_back(v);
			g_phases.push_back(phase);

			g_positions.push_back(Vec4(-1.0f, y, 0.5f, 0.9f));
			g_velocities.push_back(v);
			g_phases.push_back(phase);

			g_triangles.push_back(0);
			g_triangles.push_back(1);
			g_triangles.push_back(2);
			g_triangleNormals.push_back(0.0f);

			CreateSpring(0, 1, 1.0f);
			CreateSpring(1, 2, 1.0f);
			CreateSpring(2, 0, 1.0f);

			g_positions[0].y -= radius*2.5f;
			//g_positions[0].x = 1.0f;
			//g_positions[1].y += radius;
		}

		g_params.mDrag = 0.01f;
		g_params.mLift = 0.1f;
		g_params.mDynamicFriction = 0.25f;
		//g_params.mGravity[1] = 0.0f;

		g_drawPoints = false;
	}

	void Update()
	{
		g_params.mWind[0] = 0.0f;
		g_params.mWind[1] = 0.0f;
		g_params.mWind[2] = 0.0f;
	}
};


class RayleighTaylor3D : public Scene
{
public:

	RayleighTaylor3D(const char* name) : Scene(name) {}

	int base;
	int width;
	int height;
	int depth;

	virtual void Initialize() 
	{
		float radius = 0.05f;
		float restDistance = radius*0.5f;

		width = 128;
		height = 24;
		depth = 24;

		base = 4;

		float sep = restDistance*0.9f;
		int group = 0;

		CreateParticleGrid(Vec3(0.0f, 0.0f, 0.0f), width, base, depth, sep, Vec3(0.0f), 0.0f, false, 0.0f, flexMakePhase(group++, 0), 0.0f);
		CreateParticleGrid(Vec3(0.0f, base*sep, 0.0f), width, height, depth, sep, Vec3(0.0f), 0.24f, false, 0.0f, flexMakePhase(group++, eFlexPhaseSelfCollide | eFlexPhaseFluid), restDistance*0.01f);		
		CreateParticleGrid(Vec3(0.0f, sep*height+base*sep, 0.0f), width, height, depth, sep, Vec3(0.0f), 0.25f, false, 0.0f, flexMakePhase(group++, eFlexPhaseSelfCollide | eFlexPhaseFluid), restDistance*0.01f);		
				
		g_params.mGravity[1] = -9.f;
		g_params.mRadius = radius;
		g_params.mDynamicFriction = 0.00f;
		g_params.mFluid = true;
		g_params.mViscosity = 2.0f;
		g_params.mNumIterations = 10;
		g_params.mVorticityConfinement = 0.0f;
		g_params.mAnisotropyScale = 50.0f;
		g_params.mSmoothing = 1.f;
		g_params.mFluidRestDistance = restDistance;
		g_params.mNumPlanes = 5;		
		g_params.mCohesion = 0.0002125f;
		g_params.mSurfaceTension = 0.0f;
		g_params.mCollisionDistance = 0.001f;//restDistance*0.5f;
		//g_params.mSolidPressure = 0.2f;

		g_params.mRelaxationFactor = 20.0f;
		g_numSubsteps = 5;

		g_fluidColor = Vec4(0.2f, 0.6f, 0.9f, 1.0f);

		g_lightDistance *= 0.85f;
		
		// draw options
		g_drawDensity = true;
		g_drawDiffuse = false;
		g_drawEllipsoids = false;
		g_drawPoints = true;

		g_pointScale = 0.9f;
		g_blur = 2.0f;

		g_warmup = true;
	}

	virtual void Update()
	{
		if (g_params.mNumPlanes == 4)
			g_params.mDynamicFriction = 0.2f;

		if (g_frame == 32)
		{
			int layer1start = width*depth*base;
			int layer1end = layer1start + width*height*depth;
			for (int i=layer1start; i < layer1end; ++i)
				g_positions[i].w = 1.0f;
		}
	}
};


class RayleighTaylor2D : public Scene
{
public:

	RayleighTaylor2D(const char* name) : Scene(name) {}

	int base;
	int width;
	int height;
	int depth;

	virtual void Initialize() 
	{
		float radius = 0.05f;
		float restDistance = radius*0.5f;

		width = 128;
		height = 24;
		depth = 1;

		base = 4;

		float sep = restDistance*0.7f;
		int group = 0;

		CreateParticleGrid(Vec3(0.0f, 0.0f, 0.0f), width, base, depth, sep, Vec3(0.0f), 0.0f, false, 0.0f, flexMakePhase(group++, 0), 0.0f);
		CreateParticleGrid(Vec3(0.0f, base*sep, 0.0f), width, height, depth, sep, Vec3(0.0f), 1.0f, false, 0.0f, flexMakePhase(group++, eFlexPhaseSelfCollide | eFlexPhaseFluid), restDistance*0.01f);		
		CreateParticleGrid(Vec3(0.0f, sep*height+base*sep, 0.0f), width, height, depth, sep, Vec3(0.0f), 0.25f, false, 0.0f, flexMakePhase(group++, eFlexPhaseSelfCollide | eFlexPhaseFluid), restDistance*0.01f);		
				
		g_params.mGravity[1] = -9.f;
		g_params.mRadius = radius;
		g_params.mDynamicFriction = 0.00f;
		g_params.mFluid = true;
		g_params.mViscosity = 0.0f;
		g_params.mNumIterations = 5;
		g_params.mVorticityConfinement = 0.0f;
		g_params.mAnisotropyScale = 50.0f;
		g_params.mSmoothing = 1.f;
		g_params.mFluidRestDistance = restDistance;
		g_params.mNumPlanes = 5;		
		g_params.mCohesion = 0.0025f;
		g_params.mSurfaceTension = 0.0f;
		g_params.mCollisionDistance = 0.001f;
		g_params.mRestitution = 0.0f;		

		g_params.mRelaxationFactor = 1.0f;
		g_numSubsteps = 10;

		g_fluidColor = Vec4(0.2f, 0.6f, 0.9f, 1.0f);

		g_lightDistance *= 0.85f;
		
		// draw options
		g_drawDensity = true;
		g_drawDiffuse = false;
		g_drawEllipsoids = false;
		g_drawPoints = true;

		g_pointScale = 0.9f;
		g_blur = 2.0f;

		g_warmup = true;
	}
};



class ThinBox : public Scene
{
public:

	ThinBox(const char* name) : Scene(name) {}

	int base;
	int width;
	int height;
	int depth;

	virtual void Initialize() 
	{
		float radius = 0.03f;

		width = 16;
		height = 8;
		depth = 8;

		base = 4;

		float sep = radius;		
		
		CreateParticleGrid(Vec3(0.0f, radius, 0.0f), width, height, depth, sep, Vec3(0.0f), 1.0f, false, 0.0f, flexMakePhase(0, eFlexPhaseSelfCollide), 0.0f);

		Vec3 upper;
		Vec3 lower;
		GetParticleBounds(lower, upper);
		lower -= Vec3(radius*0.5f);
		upper += Vec3(radius*0.5f);

		Vec3 center = 0.5f*(upper+lower);

		float width = (upper-lower).x*0.5f;
		float depth = (upper-lower).z*0.5f;
		float edge = 0.0075f*0.5f;
		float height = 8*radius;
		
		CreateConvex(Vec3(edge, height, depth), center + Vec3(-width, height/2, 0.0f));
		CreateConvex(Vec3(edge, height, depth), center + Vec3(width, height/2, 0.0f));
		CreateConvex(Vec3(width-edge, height, edge), center + Vec3(0.0f, height/2, (depth-edge)));
		CreateConvex(Vec3(width-edge, height, edge), center + Vec3(0.0f, height/2, -(depth-edge)));
		CreateConvex(Vec3(width, edge, depth), Vec3(center.x, lower.y, center.z));
				
		g_params.mGravity[1] = -9.f;
		g_params.mRadius = radius;
		g_params.mDynamicFriction = 0.0f;
		g_params.mFluid = false;
		g_params.mNumIterations = 5;
		g_params.mNumPlanes = 1;
		g_params.mRestitution = 0.0f;		
		g_params.mCollisionDistance = radius;
		g_params.mParticleCollisionMargin = radius*0.5f;

		g_params.mRelaxationFactor = 0.0f;
		g_numSubsteps = 2;

		g_lightDistance *= 0.85f;
		
		// draw options
		g_drawPoints = true;
		g_warmup = false;
	}
};



class FrictionRamp : public Scene
{
public:

	FrictionRamp(const char* name) : Scene(name) {}

	virtual void Initialize()
	{
		float radius = 0.1f;

		g_params.mRadius = radius;
		g_params.mDynamicFriction = 0.35f;
		g_params.mDissipation = 0.0f;
		g_params.mNumIterations = 8;
		g_params.mViscosity = 0.0f;
		g_params.mDrag = 0.0f;
		g_params.mLift = 0.0f;
		g_params.mCollisionDistance = radius*0.5f;
		
		g_windStrength = 0.0f;
		
		g_numSubsteps = 1;

		// draw options
		g_drawPoints = false;
		g_wireframe = false;
		g_drawSprings = false;
		
		for (int i=0; i < 3; ++i)
		{
			// box
			CreateParticleShape("../../data/box.ply", Vec3(0.0f, 3.5f, -i*2.0f), 0.5f, 0.0f, radius, 0.0f, 1.0f, true, 1.0f, flexMakePhase(i, 0), true, 0.0f);

			// ramp
			CreateConvex(Vec3(5.0f, 0.5f, 1.f), Vec3(3.0f, 1.0f, -i*2.0f), QuatFromAxisAngle(Vec3(0.0f, 0.0f, 1.0f), DegToRad(-11.25f*(i+1))));			
		}
	}
};


class Player : public Scene
{
public:

	Player(const char* filename) : Scene("Player"), mFilename(filename), mRecording(NULL)
	{	
	}

	virtual void Initialize()
	{
		if (!mRecording)
			mRecording = fopen(mFilename, "rb");

		if (mRecording)
			fseek(mRecording, 0, SEEK_SET);

		// read first frame
		ReadFrame();

		g_lightDistance = 1.0f;
		g_fogDistance = 0.0f;

		g_camSpeed *= 100.0f;
		g_camNear *= 100.0f;
		g_camFar *= 100.0f;

		mCollisionOnly = false;
	}

	virtual Matrix44 GetBasis()
	{
		/* coordinate fip for Unreal captures
		Matrix44 flip = Matrix44::kIdentity;
		flip.SetCol(1, Vec4(0.0f, 0.0f, -1.0f, 0.0f));
		flip.SetCol(2, Vec4(0.0f, 1.0f, 0.0f, 0.0f));

		return flip;
		*/

		return Matrix44::kIdentity;
	}

	template<typename Element>
	void ReadArray(std::vector<Element>& dest, bool skip=false)
	{
		if (feof(mRecording))
			return;

		int length;
		int r;
		r = fread(&length, sizeof(int), 1, mRecording);
		
		int numElements = length/sizeof(Element);
		dest.resize(numElements);
		
		if (!skip)
			r = fread(&dest[0], length, 1, mRecording);
		else		
			r = fseek(mRecording, length, SEEK_CUR);

		(void)r;
	}

	template <typename Element>
	void ReadValue(Element& e, bool skip=false)
	{
		if (feof(mRecording))
			return;

		int r;
		if (!skip)
			r = fread(&e, sizeof(e), 1, mRecording);
		else
			r = fseek(mRecording, sizeof(e), SEEK_CUR);

		(void)r;
	}

	void ReadFrame(bool collisionOnly=false)
	{
		if (!mRecording)
			return;

		// params
		ReadValue(g_params, collisionOnly);

		// particle data
		ReadArray(g_positions, collisionOnly);
		ReadArray(g_velocities, collisionOnly);
		ReadArray(g_phases, collisionOnly);
		ReadArray(g_activeIndices);

		// spring data
		ReadArray(g_springIndices, collisionOnly);
		ReadArray(g_springLengths, collisionOnly);
		ReadArray(g_springStiffness, collisionOnly);

		// triangle data
		ReadArray(g_triangles, collisionOnly);
		ReadArray(g_triangleNormals, collisionOnly);

		// convex shapes
		ReadArray(g_convexAabbMin);
		ReadArray(g_convexAabbMax);
		ReadArray(g_convexStarts);
		ReadArray(g_convexLengths);
		ReadArray(g_convexPlanes);
		ReadArray(g_convexPositions);
		ReadArray(g_convexRotations);
		ReadArray(g_convexFlags);
	}

	virtual void DoGui()
	{
		if (imguiCheck("Playback Simulation", !mCollisionOnly))
			mCollisionOnly = !mCollisionOnly;
	}

	virtual void Update()
	{
		ReadFrame(mCollisionOnly);

		// update active set
		flexSetActive(g_flex, &g_activeIndices[0], g_activeIndices.size(), eFlexMemoryHost);

		// update collision shapes
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

	const char* mFilename;
	FILE* mRecording;

	bool mCollisionOnly;
};
