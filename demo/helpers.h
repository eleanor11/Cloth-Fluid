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

#include <stdarg.h>

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
	int x0 = max(x-1, 0);
	int x1 = min(x+1, dim-1);

	int y0 = max(y-1, 0);
	int y1 = min(y+1, dim-1);

	int z0 = max(z-1, 0);
	int z1 = min(z+1, dim-1);

	float dx = (SampleSDF(sdf, dim, x1, y, z) - SampleSDF(sdf, dim, x0, y, z))*(dim*0.5f);
	float dy = (SampleSDF(sdf, dim, x, y1, z) - SampleSDF(sdf, dim, x, y0, z))*(dim*0.5f);
	float dz = (SampleSDF(sdf, dim, x, y, z1) - SampleSDF(sdf, dim, x, y, z0))*(dim*0.5f);

	return Vec3(dx, dy, dz);
}

void GetParticleBounds(Vec3& lower, Vec3& upper)
{
	lower = Vec3(FLT_MAX);
	upper = Vec3(-FLT_MAX);

	for (size_t i=0; i < g_positions.size(); ++i)
	{
		lower = Min(Vec3(g_positions[i]), lower);
		upper = Max(Vec3(g_positions[i]), upper);
	}
}

void CreateParticleGrid(Vec3 lower, int dimx, int dimy, int dimz, float radius, Vec3 velocity, float invMass, bool rigid, float rigidStiffness, int phase, float jitter=0.005f)
{
	if (rigid && g_rigidIndices.empty())
		g_rigidOffsets.push_back(0);

	for (int x=0; x < dimx; ++x)
	{
		for (int y=0; y < dimy; ++y)
		{
			for (int z=0; z < dimz; ++z)
			{
				if (rigid)
					g_rigidIndices.push_back(int(g_positions.size()));

				Vec3 position = lower + Vec3(float(x), float(y), float(z))*radius + RandomUnitVector()*jitter;

				g_positions.push_back(Vec4(position.x, position.y, position.z, invMass));
				g_velocities.push_back(velocity);
				g_phases.push_back(phase);
			}
		}
	}

	if (rigid)
	{
		g_rigidCoefficients.push_back(rigidStiffness);
		g_rigidOffsets.push_back(int(g_rigidIndices.size()));
	}
}

void CreateParticleSphere(Vec3 center, int dim, float radius, Vec3 velocity, float invMass, bool rigid, float rigidStiffness, int phase, float jitter=0.005f)
{
	if (rigid && g_rigidIndices.empty())
			g_rigidOffsets.push_back(0);

	for (int x=0; x <= dim; ++x)
	{
		for (int y=0; y <= dim; ++y)
		{
			for (int z=0; z <= dim; ++z)
			{
				float sx = x - dim*0.5f;
				float sy = y - dim*0.5f;
				float sz = z - dim*0.5f;

				if (sx*sx + sy*sy + sz*sz <= dim*dim/4)
				{
					if (rigid)
						g_rigidIndices.push_back(int(g_positions.size()));

					Vec3 position = center + radius*Vec3(sx, sy, sz) + RandomUnitVector()*jitter;

					g_positions.push_back(Vec4(position.x, position.y, position.z, invMass));
					g_velocities.push_back(velocity);
					g_phases.push_back(phase);
				}
			}
		}
	}

	if (rigid)
	{
		g_rigidCoefficients.push_back(rigidStiffness);
		g_rigidOffsets.push_back(int(g_rigidIndices.size()));
	}
}

void CreateParticleShape(const char* file, Vec3 lower, Vec3 scale, float rotation, float spacing, Vec3 velocity, float invMass, bool rigid, float rigidStiffness, int phase, bool skin, float jitter=0.005f, Vec3 skinOffset=0.0f, float skinExpand=0.0f, Vec4 color=Vec4(0.0f))
{
	if (rigid && g_rigidIndices.empty())
			g_rigidOffsets.push_back(0);

	Mesh* mesh = ImportMesh(file);

	if (!mesh)
	{
		printf("Could not open mesh for reading: %s\n", file);
		return;
	}

	int startIndex = int(g_positions.size());

	{
		mesh->Transform(RotationMatrix(rotation, Vec3(0.0f, 1.0f, 0.0f)));

		Vec3 meshLower, meshUpper;
		mesh->GetBounds(meshLower, meshUpper);

		Vec3 edges = meshUpper-meshLower;
		float maxEdge = max(max(edges.x, edges.y), edges.z);

		// put mesh at the origin and scale to specified size
		Matrix44 xform = ScaleMatrix(scale/maxEdge)*TranslationMatrix(Point3(-meshLower));

		mesh->Transform(xform);
		mesh->GetBounds(meshLower, meshUpper);

		// recompute expanded edges
		edges = meshUpper-meshLower;
		maxEdge = max(max(edges.x, edges.y), edges.z);

		// tweak spacing to avoid edge cases for particles laying on the boundary
		// just covers the case where an edge is a whole multiple of the spacing.
		float spacingEps = spacing*(1.0f - 1e-4f);

		// make sure to have at least one particle in each dimension
		int dx, dy, dz;
		dx = spacing > edges.x ? 1 : int(edges.x/spacingEps);
		dy = spacing > edges.y ? 1 : int(edges.y/spacingEps);
		dz = spacing > edges.z ? 1 : int(edges.z/spacingEps);

		int maxDim = max(max(dx, dy), dz);

		// expand border by two voxels to ensure adequate sampling at edges
		meshLower -= 2.0f*Vec3(spacing);
		meshUpper += 2.0f*Vec3(spacing);
		maxDim += 4;

		vector<uint32_t> voxels(maxDim*maxDim*maxDim);

		// we shift the voxelization bounds so that the voxel centers
		// lie symmetrically to the center of the object. this reduces the 
		// chance of missing features, and also better aligns the particles
		// with the mesh
		Vec3 meshOffset;
		meshOffset.x = 0.5f * (spacing - (edges.x - (dx-1)*spacing));
		meshOffset.y = 0.5f * (spacing - (edges.y - (dy-1)*spacing));
		meshOffset.z = 0.5f * (spacing - (edges.z - (dz-1)*spacing));
		meshLower -= meshOffset;

		//Voxelize(*mesh, dx, dy, dz, &voxels[0], meshLower - Vec3(spacing*0.05f) , meshLower + Vec3(maxDim*spacing) + Vec3(spacing*0.05f));
		Voxelize((const float*)&mesh->m_positions[0], mesh->m_positions.size(), (const int*)&mesh->m_indices[0], mesh->m_indices.size(), maxDim, maxDim, maxDim, &voxels[0], meshLower, meshLower + Vec3(maxDim*spacing));

		vector<float> sdf(maxDim*maxDim*maxDim);
		MakeSDF(&voxels[0], maxDim, maxDim, maxDim, &sdf[0]);

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
						if (rigid)
							g_rigidIndices.push_back(int(g_positions.size()));

						Vec3 position = lower + meshLower + spacing*Vec3(float(x) + 0.5f, float(y) + 0.5f, float(z) + 0.5f) + RandomUnitVector()*jitter;

						 // normalize the sdf value and transform to world scale
						Vec3 n = SafeNormalize(SampleSDFGrad(&sdf[0], maxDim, x, y, z));
						float d = sdf[index]*maxEdge;

						if (rigid)
							g_rigidLocalNormals.push_back(Vec4(n, d));

						g_positions.push_back(Vec4(position.x, position.y, position.z, invMass));						
						g_velocities.push_back(velocity);
						g_phases.push_back(phase);
					}
				}
			}
		}
		mesh->Transform(ScaleMatrix(1.0f + skinExpand)*TranslationMatrix(Point3(-0.5f*(meshUpper+meshLower))));
		mesh->Transform(TranslationMatrix(Point3(lower + 0.5f*(meshUpper+meshLower))));	
	}
	
	if (skin)
	{
		g_rigidMeshSize.push_back(mesh->GetNumVertices());

		int startVertex = 0;

		// append to mesh
		if (g_mesh)
		{
			startVertex = g_mesh->GetNumVertices();

			g_mesh->Transform(TranslationMatrix(Point3(skinOffset)));
			g_mesh->AddMesh(*mesh);

			delete mesh;
		}
		else
			g_mesh = mesh;

		mesh = g_mesh;

		const Colour colors[7] = 
		{
			Colour(0.0f, 0.5f, 1.0f),
			Colour(0.797f, 0.354f, 0.000f),			
			Colour(0.000f, 0.349f, 0.173f),
			Colour(0.875f, 0.782f, 0.051f),
			Colour(0.01f, 0.170f, 0.453f),
			Colour(0.673f, 0.111f, 0.000f),
			Colour(0.612f, 0.194f, 0.394f) 
		};

		for (uint32_t i=startVertex; i < mesh->GetNumVertices(); ++i)
		{
			int indices[g_numSkinWeights] = { -1, -1, -1, -1 };
			float distances[g_numSkinWeights] = {FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX };
			
			if (LengthSq(color) == 0.0f)
				mesh->m_colours[i] = 1.25f*colors[phase%7];
			else
				mesh->m_colours[i] = Colour(color);

			// find closest n particles
			for (size_t j=startIndex; j < g_positions.size(); ++j)
			{
				float dSq = LengthSq(Vec3(mesh->m_positions[i])-Vec3(g_positions[j]));

				// insertion sort
				int w=0;
				for (; w < 4; ++w)
					if (dSq < distances[w])
						break;
				
				if (w < 4)
				{
					// shuffle down
					for (int s=3; s > w; --s)
					{
						indices[s] = indices[s-1];
						distances[s] = distances[s-1];
					}

					distances[w] = dSq;
					indices[w] = int(j);				
				}
			}

			// weight particles according to distance
			float wSum = 0.0f;

			for (int w=0; w < 4; ++w)
			{				
				// convert to inverse distance
				distances[w] = 1.0f/(0.1f + powf(distances[w], .125f));

				wSum += distances[w];

			}

			float weights[4];
			for (int w=0; w < 4; ++w)
				weights[w] = distances[w]/wSum;

			for (int j=0; j < 4; ++j)
			{
				g_meshSkinIndices.push_back(indices[j]);
				g_meshSkinWeights.push_back(weights[j]);
			}
		}
	}
	else
	{
		delete mesh;
	}

	if (rigid)
	{
		g_rigidCoefficients.push_back(rigidStiffness);
		g_rigidOffsets.push_back(int(g_rigidIndices.size()));
	}
}

void SkinMesh()
{
	if (g_mesh)
	{
		int startVertex = 0;

		for (size_t r=0; r < g_rigidRotations.size(); ++r)
		{
			const Matrix33 rotation = g_rigidRotations[r];
			const int numVertices = g_rigidMeshSize[r];

			for (int i=startVertex; i < numVertices+startVertex; ++i)
			{
				Vec3 skinPos;

				for (int w=0; w < 4; ++w)
				{
					// small shapes can have < 4 particles
					if (g_meshSkinIndices[i*4+w] > -1)
					{
						assert(g_meshSkinWeights[i*4+w] < FLT_MAX);

						int index = g_meshSkinIndices[i*4+w];
						float weight = g_meshSkinWeights[i*4+w];

						skinPos += (rotation*(g_meshRestPositions[i]-Point3(g_restPositions[index])) + Vec3(g_positions[index]))*weight;
					}
				}

				g_mesh->m_positions[i] = Point3(skinPos);
			}

			startVertex += numVertices;
		}

		g_mesh->CalculateNormals();
	}
}

void CreateConvex(Vec3 halfEdge = Vec3(2.0f), Vec3 center=Vec3(0.0f), Vec4 quat=Vec4(0.0f, 0.0f, 0.0f, 1.0f), int flags=0)
{

	if (LengthSq(center) == 0.0f)
	{
		Vec3 lower, upper;
		GetParticleBounds(lower, upper);

		center = (lower+upper)*0.5f;
		center.y = 0.0f;
	}

	// create a box
	for (int i=0; i < 1; ++i)
	{
		int startPlane = int(g_convexPlanes.size());

		g_convexPlanes.push_back(Vec4(1.0f, 0.0f, 0.0f, -halfEdge.x));
		g_convexPlanes.push_back(Vec4(-1.0f, 0.0f, 0.0f, -halfEdge.x));

		g_convexPlanes.push_back(Vec4(0.0f, 1.0f, 0.0f, -halfEdge.y*0.5f));
		g_convexPlanes.push_back(Vec4(0.0f, -1.0f, 0.0f, -halfEdge.y));

		g_convexPlanes.push_back(Vec4(0.0f, 0.0f, 1.0f, -halfEdge.z));
		g_convexPlanes.push_back(Vec4(0.0f, 0.0f, -1.0f, -halfEdge.z));
		
		g_convexStarts.push_back(startPlane);
		g_convexLengths.push_back(6);

		g_convexPositions.push_back(Vec4(center.x, center.y, center.z, 0.0f));
		g_convexRotations.push_back(quat);

		g_convexPrevPositions.push_back(g_convexPositions.back());
		g_convexPrevRotations.push_back(g_convexRotations.back());

		// set aabbs
		ConvexMeshBuilder builder(&g_convexPlanes[startPlane]);
		builder(6);
			
		Vec3 lower(FLT_MAX), upper(-FLT_MAX);
		for (size_t v=0; v < builder.mVertices.size(); ++v)
		{
			Vec3 p =  rotate(Vec3(g_convexRotations.back()), g_convexRotations.back().w, builder.mVertices[v]) + Vec3(g_convexPositions.back());
			lower = Min(lower, p);
			upper = Max(upper, p);
		}
		g_convexAabbMin.push_back(Vec4(lower.x, lower.y, lower.z, 0.0f));
		g_convexAabbMax.push_back(Vec4(upper.x, upper.y, upper.z, 0.0f));		
		g_convexFlags.push_back(flags);
	}
}

void CreateSDF(const Mesh* mesh, uint32_t dim, Vec3 lower, Vec3 upper, float* sdf)
{
	if (mesh)
	{
		printf("Begin mesh voxelization\n");

		double startVoxelize = GetSeconds();

		uint32_t* volume = new uint32_t[dim*dim*dim];
		Voxelize((const float*)&mesh->m_positions[0], mesh->m_positions.size(), (const int*)&mesh->m_indices[0], mesh->m_indices.size(), dim, dim, dim, volume, lower, upper);

		printf("End mesh voxelization (%.2fs)\n", (GetSeconds()-startVoxelize));
	
		printf("Begin SDF gen (fast marching method)\n");

		double startSDF = GetSeconds();

		MakeSDF(volume, dim, dim, dim, sdf);

		printf("End SDF gen (%.2fs)\n", (GetSeconds()-startSDF));
	
		delete[] volume;
	}
}


void CreateRandomConvex(int numPlanes, Vec3 position, float minDist, float maxDist, Vec3 axis, float angle)
{
	// 12-kdop
	const Vec3 directions[] = { 
		Vec3(1.0f, 0.0f, 0.0f),
		Vec3(0.0f, 1.0f, 0.0f),
		Vec3(0.0f, 0.0f, 1.0f),
		Vec3(-1.0f, 0.0f, 0.0f),
		Vec3(0.0f, -1.0f, 0.0f),
		Vec3(0.0f, 0.0f, -1.0f),
		Vec3(1.0f, 1.0f, 0.0f),
		Vec3(-1.0f, -1.0f, 0.0f),
		Vec3(1.0f, 0.0f, 1.0f),
		Vec3(-1.0f, 0.0f, -1.0f),
		Vec3(0.0f, 1.0f, 1.0f),
		Vec3(0.0f, -1.0f, -1.0f),
	 };

	numPlanes = max(4, numPlanes);

	int index = int(g_convexLengths.size());
	int startPlane = int(g_convexPlanes.size());

	// create a box
	for (int i=0; i < numPlanes; ++i)
	{
		// pick random dir and distance
		Vec3 dir = Normalize(directions[i]);//RandomUnitVector();
		float dist = Randf(minDist, maxDist);

		g_convexPlanes.push_back(Vec4(dir.x, dir.y, dir.z, -dist));
	}

	g_convexStarts.push_back(startPlane);
	g_convexLengths.push_back(numPlanes);

	g_convexPositions.push_back(Vec4(position.x, position.y, position.z, 0.0f));
	g_convexRotations.push_back(QuatFromAxisAngle(axis, angle));

	g_convexPrevPositions.push_back(g_convexPositions.back());
	g_convexPrevRotations.push_back(g_convexRotations.back());

	// set aabbs
	ConvexMeshBuilder builder(&g_convexPlanes[startPlane]);
	builder(numPlanes);
			
	Vec3 lower(FLT_MAX), upper(-FLT_MAX);
	for (size_t v=0; v < builder.mVertices.size(); ++v)
	{
		Vec3 p =  rotate(Vec3(g_convexRotations[index]), g_convexRotations[index].w, builder.mVertices[v]) + Vec3(g_convexPositions[index]);
		lower = Min(lower, p);
		upper = Max(upper, p);
	}
	g_convexAabbMin.push_back(Vec4(lower.x, lower.y, lower.z, 0.0f));
	g_convexAabbMax.push_back(Vec4(upper.x, upper.y, upper.z, 0.0f));		
	g_convexFlags.push_back(0);
}

void CreateRandomBody(int numPlanes, Vec3 position, float minDist, float maxDist, Vec3 axis, float angle, float invMass, int phase, float stiffness)
{
	// 12-kdop
	const Vec3 directions[] = { 
		Vec3(1.0f, 0.0f, 0.0f),
		Vec3(0.0f, 1.0f, 0.0f),
		Vec3(0.0f, 0.0f, 1.0f),
		Vec3(-1.0f, 0.0f, 0.0f),
		Vec3(0.0f, -1.0f, 0.0f),
		Vec3(0.0f, 0.0f, -1.0f),
		Vec3(1.0f, 1.0f, 0.0f),
		Vec3(-1.0f, -1.0f, 0.0f),
		Vec3(1.0f, 0.0f, 1.0f),
		Vec3(-1.0f, 0.0f, -1.0f),
		Vec3(0.0f, 1.0f, 1.0f),
		Vec3(0.0f, -1.0f, -1.0f),
	 };

	numPlanes = max(4, numPlanes);

	vector<Vec4> planes;

	// create a box
	for (int i=0; i < numPlanes; ++i)
	{
		// pick random dir and distance
		Vec3 dir = Normalize(directions[i]);//RandomUnitVector();
		float dist = Randf(minDist, maxDist);

		planes.push_back(Vec4(dir.x, dir.y, dir.z, -dist));
	}

	// set aabbs
	ConvexMeshBuilder builder(&planes[0]);
	builder(numPlanes);
			
	int startIndex = int(g_positions.size());

	for (size_t v=0; v < builder.mVertices.size(); ++v)
	{
		Vec4 q = QuatFromAxisAngle(axis, angle);
		Vec3 p =  rotate(Vec3(q), q.w, builder.mVertices[v]) + position;

		g_positions.push_back(Vec4(p.x, p.y, p.z, invMass));
		g_velocities.push_back(0.0f);
		g_phases.push_back(phase);

		// add spring to all verts with higher index
		for (size_t i=v+1; i < builder.mVertices.size(); ++i)
		{
			int a = startIndex + int(v);
			int b = startIndex + int(i);

			g_springIndices.push_back(a);
			g_springIndices.push_back(b);
			g_springLengths.push_back(Length(builder.mVertices[v]-builder.mVertices[i]));
			g_springStiffness.push_back(stiffness);

		}
	}	

	for (size_t t=0; t < builder.mIndices.size(); ++t)
		g_triangles.push_back(startIndex + builder.mIndices[t]);		

	// lazy
	g_triangleNormals.resize(g_triangleNormals.size() + builder.mIndices.size()/3, Vec3(0.0f));
}

void CreateSDF(const char* meshFile, float scale, Vec3 lower, float expand=0.0f)
{
	// voxelize mesh
	Mesh* mesh = ImportMesh(meshFile);
	
	if (!mesh)
	{
		printf("Could not open mesh for reading: %s\n", meshFile);
	    return;
	}	
	else		
	{
		Vec3 minExtents, maxExtents, edges;
		mesh->GetBounds(minExtents, maxExtents);
		edges = maxExtents-minExtents;

		// normalize mesh scale
		float longestAxis = max(max(edges.x, edges.y), edges.z);
			
		mesh->Transform(TranslationMatrix(Point3(-Vec3(minExtents))));
		//mesh->Transform(RotationMatrix(-kPi*0.0f, Vec3(1.0f, 0.0f, 0.0f)));
		mesh->Transform(ScaleMatrix(scale/longestAxis));
		mesh->Transform(TranslationMatrix(Point3(lower)));

		mesh->GetBounds(minExtents, maxExtents);
		mesh->m_colours.resize(0);

		// store mesh 
		g_mesh = mesh;

		// square extents
		edges = maxExtents-minExtents;
		longestAxis = max(max(edges.x, edges.y), edges.z);
		edges = longestAxis;

		minExtents = minExtents - edges*0.1f;
		maxExtents = minExtents + edges*1.1f;
		edges = maxExtents-minExtents;

		// try and load the sdf from disc if it exists
		string sdfFile = string(meshFile, strrchr(meshFile, '.')) + ".pfm";
			
		PfmImage sdf;
		if (!PfmLoad(sdfFile.c_str(), sdf))
		{
			const int dim = 128;

			sdf.m_width = dim;
			sdf.m_height = dim;
			sdf.m_depth = dim;
			sdf.m_data = new float[dim*dim*dim];

			printf("Cooking SDF: %s - dim: %d^3\n", sdfFile.c_str(), dim);

			CreateSDF(mesh, dim, minExtents, maxExtents, sdf.m_data);

			PfmSave(sdfFile.c_str(), sdf);
		}

		printf("Loaded SDF, %d\n", sdf.m_width);

		assert(sdf.m_width == sdf.m_height && sdf.m_width == sdf.m_depth);

		// cheap collision offset
		int numVoxels = int(sdf.m_width*sdf.m_height*sdf.m_depth);
		for (int i=0; i < numVoxels; ++i)
			sdf.m_data[i] += expand;

		// set up flex collision shape
		g_shape.mWidth = sdf.m_width;
		g_shape.mHeight = sdf.m_height;
		g_shape.mDepth = sdf.m_depth;
		(Vec3&)g_shape.mLower = minExtents;
		(Vec3&)g_shape.mUpper = maxExtents;
		(Vec3&)g_shape.mInvEdgeLength = Vec3(1.0f/edges.x, 1.0f/edges.y, 1.0f/edges.z);
		g_shape.mField = sdf.m_data;
	}
}

inline int GridIndex(int x, int y, int dx) { return y*dx + x; }

void CreateSpring(int i, int j, float stiffness, float give=0.0f)
{
	g_springIndices.push_back(i);
	g_springIndices.push_back(j);
	g_springLengths.push_back((1.0f+give)*Length(Vec3(g_positions[i])-Vec3(g_positions[j])));
	g_springStiffness.push_back(stiffness);	
}

void CreateSpringGrid(Vec3 lower, int dx, int dy, int dz, float radius, int phase, float stretchStiffness, float bendStiffness, float shearStiffness, Vec3 velocity, float invMass)
{
	int baseIndex = int(g_positions.size());

	for (int z=0; z < dz; ++z)
	{
		for (int y=0; y < dy; ++y)
		{
			for (int x=0; x < dx; ++x)
			{
				Vec3 position = lower + radius*Vec3(float(x), float(z), float(y));

				g_positions.push_back(Vec4(position.x, position.y, position.z, invMass));
				g_velocities.push_back(velocity);
				g_phases.push_back(phase);

				if (x > 0 && y > 0)
				{
					g_triangles.push_back(baseIndex + GridIndex(x-1, y-1, dx));
					g_triangles.push_back(baseIndex + GridIndex(x, y-1, dx));
					g_triangles.push_back(baseIndex + GridIndex(x, y, dx));
					
					g_triangles.push_back(baseIndex + GridIndex(x-1, y-1, dx));
					g_triangles.push_back(baseIndex + GridIndex(x, y, dx));
					g_triangles.push_back(baseIndex + GridIndex(x-1, y, dx));

					g_triangleNormals.push_back(Vec3(0.0f, 1.0f, 0.0f));
					g_triangleNormals.push_back(Vec3(0.0f, 1.0f, 0.0f));

					g_saturations.push_back(0.0);
					g_saturations.push_back(0.0);
				}
			}
		}
	}	

	// horizontal
	for (int y=0; y < dy; ++y)
	{
		for (int x=0; x < dx; ++x)
		{
			int index0 = y*dx + x;

			if (x > 0)
			{
				int index1 = y*dx + x - 1;
				CreateSpring(baseIndex + index0, baseIndex + index1, stretchStiffness);
			}

			if (x > 1)
			{
				int index2 = y*dx + x - 2;
				CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
			}

			if (y > 0 && x < dx-1)
			{
				int indexDiag = (y-1)*dx + x + 1;
				CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
			}

			if (y > 0 && x > 0)
			{
				int indexDiag = (y-1)*dx + x - 1;
				CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
			}
		}
	}

	// vertical
	for (int x=0; x < dx; ++x)
	{
		for (int y=0; y < dy; ++y)
		{
			int index0 = y*dx + x;

			if (y > 0)
			{
				int index1 = (y-1)*dx + x;
				CreateSpring(baseIndex + index0, baseIndex + index1, stretchStiffness);
			}

			if (y > 1)
			{
				int index2 = (y-2)*dx + x;
				CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
			}
		}
	}	
}

void CreateSpringGrid2(Vec3 lower, int dx, int dy, int dz, float radius, int phase, float stretchStiffness, float bendStiffness, float shearStiffness, Vec3 velocity, float invMass)
{
	int baseIndex = int(g_positions.size());

	for (int z = 0; z < dz; ++z)
	{
		for (int y = 0; y < dy; ++y)
		{
			for (int x = 0; x < dx; ++x)
			{
				Vec3 position = lower + radius*Vec3(float(x), float(z), float(y));

				g_positions.push_back(Vec4(position.x, position.y, position.z, invMass));
				g_velocities.push_back(velocity);
				g_phases.push_back(phase);

				if (x > 0 && y > 0)
				{
					g_triangles.push_back(baseIndex + GridIndex(x - 1, y - 1, dx));
					g_triangles.push_back(baseIndex + GridIndex(x, y - 1, dx));
					g_triangles.push_back(baseIndex + GridIndex(x, y, dx));

					g_triangles.push_back(baseIndex + GridIndex(x - 1, y - 1, dx));
					g_triangles.push_back(baseIndex + GridIndex(x, y, dx));
					g_triangles.push_back(baseIndex + GridIndex(x - 1, y, dx));

					g_triangleNormals.push_back(Vec3(0.0f, 1.0f, 0.0f));
					g_triangleNormals.push_back(Vec3(0.0f, 1.0f, 0.0f));

					g_saturations.push_back(0.0);
					g_saturations.push_back(0.0);
				}
			}
		}
	}

	// horizontal
	for (int z = 0; z < dz; ++z)
	{
		for (int y = 0; y < dy; ++y)
		{
			for (int x = 0; x < dx; ++x)
			{
				int index0 = y*dx + x;

				if (x > 0)
				{
					int index1 = z * dy * dx + y*dx + x - 1;
					CreateSpring(baseIndex + index0, baseIndex + index1, stretchStiffness);
				}

				if (x > 1)
				{
					int index2 = z * dy * dx + y*dx + x - 2;
					CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
				}

				if (y > 0 && x < dx - 1)
				{
					int indexDiag = z * dy * dx + (y - 1)*dx + x + 1;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}

				if (y > 0 && x > 0)
				{
					int indexDiag = z * dy * dx + (y - 1)*dx + x - 1;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}
			}
		}
	}

	// vertical

	for (int z = 0; z < dz; ++z)
	{
		for (int x = 0; x < dx; ++x)
		{
			for (int y = 0; y < dy; ++y)
			{
				int index0 = y*dx + x;

				if (y > 0)
				{
					int index1 = (y - 1)*dx + x;
					CreateSpring(baseIndex + index0, baseIndex + index1, stretchStiffness);
				}

				if (y > 1)
				{
					int index2 = (y - 2)*dx + x;
					CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
				}
			}
		}
	}
}


void CreateRope(Rope& rope, Vec3 start, Vec3 dir, float stiffness, int segments, float length, int phase, float spiralAngle=0.0f, float invmass=1.0f, float give=0.075f)
{
	rope.mIndices.push_back(int(g_positions.size()));

	g_positions.push_back(Vec4(start.x, start.y, start.z, invmass));
	g_velocities.push_back(0.0f);
	g_phases.push_back(phase);//int(g_positions.size()));
	
	Vec3 left, right;
	BasisFromVector(dir, &left, &right);

	float segmentLength = length/segments;
	Vec3 spiralAxis = dir;
	float spiralHeight = spiralAngle/(2.0f*kPi)*(length/segments);

	if (spiralAngle > 0.0f)
		dir = left;

	Vec3 p = start;

	for (int i=0; i < segments; ++i)
	{
		int prev = int(g_positions.size())-1;

		p += dir*segmentLength;

		// rotate 
		if (spiralAngle > 0.0f)
		{
			p += spiralAxis*spiralHeight;

			dir = RotationMatrix(spiralAngle, spiralAxis)*dir;
		}

		rope.mIndices.push_back(int(g_positions.size()));

		g_positions.push_back(Vec4(p.x, p.y, p.z, 1.0f));
		g_velocities.push_back(0.0f);
		g_phases.push_back(phase);//int(g_positions.size()));

		// stretch
		CreateSpring(prev, prev+1, stiffness, give);

		// tether
		//if (i > 0 && i%4 == 0)
			//CreateSpring(prev-3, prev+1, -0.25f);
		
		// bending spring
		if (i > 0)
			CreateSpring(prev-1, prev+1, stiffness*0.5f, give);
	}
}

namespace
{
	struct Tri
	{
		int a;
		int b;
		int c;

		Tri(int a, int b, int c) : a(a), b(b), c(c) {}

		bool operator < (const Tri& rhs)
		{
			if (a != rhs.a)
				return a < rhs.a;
			else if (b != rhs.b)
				return b < rhs.b;
			else
				return c < rhs.c;
		}
	};
}


namespace
{
	struct TriKey
	{
		int orig[3];
		int indices[3];

		TriKey(int a, int b, int c)		
		{
			orig[0] = a;
			orig[1] = b;
			orig[2] = c;

			indices[0] = a;
			indices[1] = b;
			indices[2] = c;

			std::sort(indices, indices+3);
		}			

		bool operator < (const TriKey& rhs) const
		{
			if (indices[0] != rhs.indices[0])
				return indices[0] < rhs.indices[0];
			else if (indices[1] != rhs.indices[1])
				return indices[1] < rhs.indices[1];
			else
				return indices[2] < rhs.indices[2];
		}
	};
}

void CreateTetMesh(const char* filename, Vec3 lower, float scale, float stiffness, int phase)
{
	FILE* f = fopen(filename, "r");

	char line[2048];

	if (f)
	{
		typedef std::map<TriKey, int> TriMap;
		TriMap triCount;

		const int vertOffset = g_positions.size();

		Vec3 meshLower(FLT_MAX);
		Vec3 meshUpper(-FLT_MAX);

		bool firstTet = true;

		while (!feof(f))
		{
			if (fgets(line, 2048, f))
			{
				switch(line[0])
				{
				case '#':
					break;
				case 'v':
					{
						Vec3 pos;
						sscanf(line, "v %f %f %f", &pos.x, &pos.y, &pos.z);

						g_positions.push_back(Vec4(pos.x, pos.y, pos.z, 1.0f));
						g_velocities.push_back(0.0f);
						g_phases.push_back(phase);

						meshLower = Min(pos, meshLower);
						meshUpper = Max(pos, meshUpper);
						break;
					}
				case 't':
					{
						if (firstTet)
						{
							Vec3 edges = meshUpper-meshLower;
							float maxEdge = max(edges.x, max(edges.y, edges.z));

							// normalize positions
							for (int i=vertOffset; i < int(g_positions.size()); ++i)
							{
								Vec3 p = lower + (Vec3(g_positions[i])-meshLower)*scale/maxEdge;
								g_positions[i] = Vec4(p, g_positions[i].w);
							}

							firstTet = false;
						}

						int indices[4];
						sscanf(line, "t %d %d %d %d", &indices[0], &indices[1], &indices[2], &indices[3]);

						indices[0] += vertOffset;
						indices[1] += vertOffset;
						indices[2] += vertOffset;
						indices[3] += vertOffset;

						CreateSpring(indices[0], indices[1], stiffness);
						CreateSpring(indices[0], indices[2], stiffness);
						CreateSpring(indices[0], indices[3], stiffness);
				
						CreateSpring(indices[1], indices[2], stiffness);
						CreateSpring(indices[1], indices[3], stiffness);
						CreateSpring(indices[2], indices[3], stiffness);

						TriKey k1(indices[0], indices[2], indices[1]);
						triCount[k1] += 1;

						TriKey k2(indices[1], indices[2], indices[3]);
						triCount[k2] += 1;

						TriKey k3(indices[0], indices[1], indices[3]);
						triCount[k3] += 1;

						TriKey k4(indices[0], indices[3], indices[2]);
						triCount[k4] += 1;

						break;
					}
				}
			}
		}

		for (TriMap::iterator iter=triCount.begin(); iter != triCount.end(); ++iter)
		{
			TriKey key = iter->first;

			// only output faces that are referenced by one tet (open faces)
			if (iter->second == 1)
			{
				g_triangles.push_back(key.orig[0]);
				g_triangles.push_back(key.orig[1]);
				g_triangles.push_back(key.orig[2]);
				g_triangleNormals.push_back(0.0f);
			}
		}


		fclose(f);
	}
}

// unproject a pixel coordinate using the current OpenGL viewing transforms
void GetViewRay(int x, int y, Vec3& origin, Vec3& dir)
{
	double modelview[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);

	double projection[16];
	glGetDoublev(GL_PROJECTION_MATRIX, projection);

	int viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);

	double nearPos[3];
	gluUnProject(double(x), double(y), 0.0f, modelview, projection, viewport, &nearPos[0], &nearPos[1], &nearPos[2]);

	double farPos[3];
	gluUnProject(double(x), double(y), 1.0f, modelview, projection, viewport, &farPos[0], &farPos[1], &farPos[2]);
	
	origin = Vec3(float(nearPos[0]), float(nearPos[1]), float(nearPos[2]));
	dir = Normalize(Vec3(float(farPos[0]-nearPos[0]), float(farPos[1]-nearPos[1]), float(farPos[2]-nearPos[2])));
}

// finds the closest particle to a view ray
int PickParticle(Vec3 origin, Vec3 dir, Vec4* particles, int* phases, int n, float radius, float &t)
{
	float maxDistSq = radius*radius;
	float minT = FLT_MAX;
	int minIndex = -1;

	for (int i=0; i < n; ++i)
	{
		if (phases[i] & eFlexPhaseFluid)
			continue;

		Vec3 delta = Vec3(particles[i])-origin;
		float t = Dot(delta, dir);

		if (t > 0.0f)
		{
			Vec3 perp = delta - t*dir;

			float dSq = LengthSq(perp);

			if (dSq < maxDistSq && t < minT)
			{
				minT = t;
				minIndex = i;
			}
		}
	}

	t = minT;

	return minIndex;
}

// calculates local space positions given a set of particles and rigid indices
void CalculateRigidOffsets(const Vec4* restPositions, const int* offsets, const int* indices, int numRigids, Vec3* localPositions)
{
	int count = 0;

	for (int r=0; r < numRigids; ++r)
	{
		const int startIndex = offsets[r];
		const int endIndex = offsets[r+1];

		const int n = endIndex-startIndex;

		assert(n);

		Vec3 com;
	
		for (int i=startIndex; i < endIndex; ++i)
		{
			const int r = indices[i];

			com += Vec3(restPositions[r]);
		}

		com /= float(n);

		for (int i=startIndex; i < endIndex; ++i)
		{
			const int r = indices[i];

			localPositions[count++] = Vec3(restPositions[r])-com;
		}
	}
}

void DrawImguiString(int x, int y, Vec3 color, int align, const char* s, ...)
{
	char buf[2048];

	va_list args;

	va_start(args, s);
	vsnprintf(buf, 2048, s, args);
	va_end(args);

	glColor3fv(color);
	//DrawStringS(x ,y, buf);
	imguiDrawText(x, y, align, buf, imguiRGBA((unsigned char)(color.x*255), (unsigned char)(color.y*255), (unsigned char)(color.z*255)));
}


bool Collide(int x, int y){
	Vec4 posX = g_positions[x];
	Vec4 posY = g_positions[y];
	
	if (posX.y - posY.y > g_params.mSolidRestDistance) return false;
	if (posX.x - posY.x > g_params.mSolidRestDistance) return false;
	if (posX.z - posY.z > g_params.mSolidRestDistance) return false;

	float dist = sqrt(sqr(posX.x - posY.x) + sqr(posX.y - posY.y) + sqr(posX.z - posY.z));

	if (dist <= g_params.mSolidRestDistance){
		//printf("position %d\n", y);
		return true;
	}

	return false;
}

void UpdatingSaturations(int idx){

	int dimx = g_dx;
	int dimy = g_dy;
	int dx = 2;
	int dy = (dimy - 1) * 2;

	int x = idx % dimx;
	int y = idx / dimx;
	int j = x * dx + y * dy;

	float tmp = g_mDrip;

	if (y == 0){
		if (x == 0){
			tmp = tmp / 2;
			g_saturations[j] += tmp;
			g_saturations[j + 1] += tmp;
		}
		else if (x == dimx - 1){
			tmp = tmp;
			g_saturations[j - dx] += tmp;
		}
		else{
			tmp = tmp / 3;
			g_saturations[j] += tmp;
			g_saturations[j + 1] += tmp;
			g_saturations[j - dx] += tmp;
		}
	}
	else if (y == dimx - 1){
		if (x == 0){
			tmp = tmp;
			g_saturations[j - dy + 1] += tmp;
		}
		else if (x == dimx - 1){
			tmp = tmp / 2;
			g_saturations[j - dx - dy] += tmp;
			g_saturations[j - dx - dy + 1] += tmp;
		}
		else {
			tmp = tmp / 3;
			g_saturations[j - dy + 1] += tmp;
			g_saturations[j - dy - dx] += tmp;
			g_saturations[j - dy - dx + 1] += tmp;
		}

	}
	else{
		if (x == 0){
			tmp = tmp / 3;
			g_saturations[j] += tmp;
			g_saturations[j + 1] += tmp;
			g_saturations[j - dy + 1] += tmp;
		}
		else if (x == dimx - 1){
			tmp = tmp / 3;
			g_saturations[j - dx] += tmp;
			g_saturations[j - dx - dy] += tmp;
			g_saturations[j - dx - dy + 1] += tmp;
		}
		else {
			tmp = tmp / 6;
			g_saturations[j] += tmp;
			g_saturations[j + 1] += tmp;
			g_saturations[j - dx] += tmp;
			g_saturations[j - dy + 1] += tmp;
			g_saturations[j - dy - dx] += tmp;
			g_saturations[j - dy - dx + 1] += tmp;
		}
	}

}

void Absorbing(){
	int activeCount = flexGetActiveCount(g_flex);
	//printf("active: %d", activeCount);

	int i = g_numSolidParticles;
	while (i < activeCount){
		int collidePosition = -1;
		for (int j = 0; j < g_numSolidParticles; j++){
			if (Collide(i, j)){
				collidePosition = j;
				break;
			}
		}

		//i is absorbable & collided
		if (g_absorbable[i] && collidePosition > -1){
			//cloth position j 
			UpdatingSaturations(collidePosition);

			//fluid point i
			g_positions[i] = g_positions[activeCount - 1];
			g_positions[activeCount - 1] = Vec4();

			g_velocities[i] = g_velocities[activeCount - 1];
			g_velocities[activeCount - 1] = 0.0f;

			g_phases[i] = g_phases[activeCount - 1];
			g_phases[activeCount - 1] = 0;

			activeCount--;
			continue;
		}
		//i is not absorable & not collided
		if (!g_absorbable[i] && collidePosition == -1){
			g_absorbable[i] = true;
		}

		i++;
	}


	//printf("   %d\n", activeCount);
	flexSetActive(g_flex, &g_activeIndices[0], activeCount, eFlexMemoryHost);

}


void CalculateClothColors(){

	int dimx = g_dx;
	int dimy = g_dy;
	int j = 0;
	int dx = 2;
	int dy = (dimx - 1) * 2;
	Vec4 color = g_clothColor;
	Vec4 colorBase = color / g_maxSaturation;
	for (int y = 0; y < dimy; y++)
		for (int x = 0; x < dimx; x++){

			float saturation = 0.0;
			/*complex*/
			if (y == 0){
				if (x == 0){
					saturation = (g_saturations[j] + g_saturations[j + 1]) / 2;
				}
				else if (x == dimx - 1){
					saturation = g_saturations[j - dx];
				}
				else{
					saturation = (g_saturations[j] + g_saturations[j + 1] + g_saturations[j - dx]) / 3;
				}
			}
			else if (y == dimx - 1){
				if (x == 0){
					saturation = g_saturations[j - dy + 1];
				}
				else if (x == dimx - 1){
					saturation = (g_saturations[j - dx - dy] + g_saturations[j - dx - dy + 1]) / 2;
				}
				else {
					saturation = (g_saturations[j - dy + 1] + g_saturations[j - dy - dx] + g_saturations[j - dy - dx + 1]) / 3;
				}

			}
			else{
				if (x == 0){
					saturation = (g_saturations[j] + g_saturations[j + 1] + g_saturations[j - dy + 1]) / 3;
				}
				else if (x == dimx - 1){
					saturation = (g_saturations[j - dx] + g_saturations[j - dx - dy] + g_saturations[j - dx - dy + 1]) / 3;
				}
				else {
					saturation = (g_saturations[j] + g_saturations[j + 1] + g_saturations[j - dx] +
						g_saturations[j - dy + 1] + g_saturations[j - dy - dx] + g_saturations[j - dy - dx + 1]) / 6;
				}
			}
			if (x != dimx - 1)
				j += 2;

			//printf("%f\n", saturation);


			/*naive version*/
			//if (x == dimx - 1 && y == dimx - 1){
			//	saturation = (g_saturations[j - dx - dy] + g_saturations[j - dx - dy + 1]) / 2;
			//}
			//else if (x == dimx - 1){
			//	/*right boardary*/
			//	saturation = g_saturations[j - dx];
			//	//printf("right: %f\n", saturation);
			//}
			//else if (y == dimx - 1){
			//	/*bottom boardary*/
			//	saturation = g_saturations[j - dy + 1];
			//	//printf("bottom: %f\n", saturation);
			//}
			//else {
			//	saturation = (g_saturations[j] + g_saturations[j + 1]) / 2;
			//	j += 2;
			//}

			//Vec4 color = Vec4(1.0f, 0.3f, 0.3f, 1.0f);

			g_colors[y * dimx + x] = colorBase * (g_maxSaturation - saturation);
			//printf("%d: %f, %f, %f\n", y * dimx + x, g_colors[y * dimx + x].x, g_colors[y * dimx + x].y, g_colors[y * dimx + x].z);
		}
}

void CalculateTriangleCenters(){

	if (g_triangleCenters.size() == 0)
		g_triangleCenters.resize(g_numTriangles);

	int dimx = g_dx;
	int dimy = g_dy;
	int index = 0;
	for (int y = 0; y < dimy; y++)
		for (int x = 0; x < dimx; x++){
			if (x > 0 && y > 0){
				Vec3 position0 = g_positions[GridIndex(x - 1, y - 1, dimx)];
				Vec3 position1 = g_positions[GridIndex(x, y - 1, dimx)];
				Vec3 position2 = g_positions[GridIndex(x, y, dimx)];
				Vec3 position3 = g_positions[GridIndex(x - 1, y, dimx)];

				Vec3 center1 = position1 + (position0 - position1 + position2 - position1) / 3;
				Vec3 center2 = position3 + (position0 - position3 + position2 - position3) / 3;

				g_triangleCenters[index] = center1;
				g_triangleCenters[index + 1] = center2;

				//printf("%d, %f, %f, %f\n", GridIndex(x - 1, y - 1, dimx), position0.x, position0.y, position0.z);
				//printf("%d, %f, %f, %f\n", GridIndex(x, y - 1, dimx), position1.x, position1.y, position1.z);
				//printf("%d, %f, %f, %f\n", GridIndex(x, y, dimx), position2.x, position2.y, position2.z);

				//printf("%d, %f, %f, %f\n", index, center1.x, center1.y, center1.z);
				//printf("%d, %f, %f, %f\n", index + 1, center2.x, center2.y, center2.z);

				index += 2;

			}

		}


}

Vec3 calculateCosTheta(int index, int idx1, int idx2, int idx3, int min, int max, int dimx){
	float t1 = 10.0, t2 = 10.0, t3 = 10.0;
	if (idx1 >= min && idx1 < max && ((index - 1) % dimx != 0)){
		Vec3 dir = g_triangleCenters[idx1] - g_triangleCenters[index];
		Vec3 gDir = Vec3(0.0, -1.0, 0.0);
		t1 = (-dir.y) / (sqrt(sqr(dir.x) + sqr(dir.y) + sqr(dir.z)));
	}
	if (idx2 >= min && idx2 < max && ((index + 2) % dimx != 0)){
		Vec3 dir = g_triangleCenters[idx2] - g_triangleCenters[index];
		Vec3 gDir = Vec3(0.0, -1.0, 0.0);
		t2 = (-dir.y) / (sqrt(sqr(dir.x) + sqr(dir.y) + sqr(dir.z)));
	}
	if (idx3 >= min && idx3 < max){
		Vec3 dir = g_triangleCenters[idx3] - g_triangleCenters[index];
		Vec3 gDir = Vec3(0.0, -1.0, 0.0);
		t3 = (-dir.y) / (sqrt(sqr(dir.x) + sqr(dir.y) + sqr(dir.z)));
	}

	//printf("%f, %f, %f\n", t1, t2, t3);

	return Vec3(t1, t2, t3);
}

void CalculateThetas(){

	if (g_thetas.size() == 0)
		g_thetas.resize(g_numTriangles);

	int dy = (g_dx - 1) * 2;
	int numMax = g_numTriangles;
	int numGrid = g_numTriangles / 2;
	int index = 0;
	for (int i = 0; i < numGrid; i++){
		g_thetas[index] = calculateCosTheta(index, index - dy + 1, index + 3, index + 1, 0, numMax, dy);
		index++;
		g_thetas[index] = calculateCosTheta(index, index - 3, index - 1, index + dy - 1, 0, numMax, dy);
		index++;

		//printf("1:%d, %f, %f, %f\n", index, g_thetas[index].x, g_thetas[index].y, g_thetas[index].z);
		//printf("%d, %f, %f\n", index, g_thetas[index], g_thetas[index + 1]);
	}

}

float getMin(float x, float y){
	if (x < y) return x;
	else return y;
}

void DiffuseCloth(){

	int numGrid = g_numTriangles / 2;
	int dy = (g_dx - 1) * 2;

	vector<float> deltas;
	deltas.resize(numGrid * 2);

	int index = 0;

	for (int i = 0; i < numGrid; i++){
		{
			float sSum = 0;
			float si = g_saturations[index];
			//printf("%f, %f, %d\n", si, g_saturations[index], index);
			Vec3 thetas = g_thetas[index];
			//printf("%d, %f, %f, %f\n", index, thetas.x, thetas.y, thetas.z);
			Vec3 deltasin = Vector3(0.0, 0.0, 0.0);
			if (thetas.x <= 1.0){
				deltasin.x = getMin(0.0f, g_kDiffusion * (si - g_saturations[index - dy + 1]) + g_kDiffusionGravity * si * thetas.x);
				sSum += deltasin.x;
			}
			if (thetas.y <= 1.0){
				deltasin.y = getMin(0.0f, g_kDiffusion * (si - g_saturations[index + 3]) + g_kDiffusionGravity * si * thetas.y);
				sSum += deltasin.y;
			}
			if (thetas.z <= 1.0){
				deltasin.z = getMin(0.0f, g_kDiffusion * (si - g_saturations[index + 1]) + g_kDiffusionGravity * si * thetas.z);
				sSum += deltasin.z;
			}

			float normFac = 1.0;
			if (sSum > si){
				normFac = si / sSum;
			}

			//printf("%f, %f, %f\n", sSum, si, normFac);

			if (thetas.x <= 1.0){
				deltas[index] += -normFac * deltasin.x;
				deltas[index - dy + 1] += normFac * deltasin.x;//* ai/an = 1.0
			}
			if (thetas.y <= 1.0){
				deltas[index] += -normFac * deltasin.y;
				deltas[index + 3] += normFac * deltasin.y;//* ai/an = 1.0
			}
			if (thetas.z <= 1.0){
				deltas[index] += -normFac * deltasin.z;
				deltas[index + 1] += normFac * deltasin.z;//* ai/an = 1.0
			}

			index++;
		}
		{
			float sSum = 0;
			float si = g_saturations[index];
			Vec3 thetas = g_thetas[index];
			//printf("%f, %f, %d\n", si, g_saturations[index], index);
			Vec3 deltasin = Vector3(0.0, 0.0, 0.0);
			if (thetas.x <= 1.0){
				deltasin.x = getMin(0.0f, g_kDiffusion * (si - g_saturations[index - 3]) + g_kDiffusionGravity * si * thetas.x);
				sSum += deltasin.x;
			}
			if (thetas.y <= 1.0){
				deltasin.y = getMin(0.0f, g_kDiffusion * (si - g_saturations[index - 1]) + g_kDiffusionGravity * si * thetas.y);
				sSum += deltasin.y;
			}
			if (thetas.z <= 1.0){
				deltasin.z = getMin(0.0f, g_kDiffusion * (si - g_saturations[index + dy - 1]) + g_kDiffusionGravity * si * thetas.z);
				sSum += deltasin.z;
			}

			float normFac = 1.0;
			if (sSum > si){
				normFac = si / sSum;
			}

			if (thetas.x <= 1.0){
				deltas[index] += -normFac * deltasin.x;
				deltas[index - 3] += normFac * deltasin.x;//* ai/an = 1.0
			}
			if (thetas.y <= 1.0){
				deltas[index] += -normFac * deltasin.y;
				deltas[index - 1] += normFac * deltasin.y;//* ai/an = 1.0
			}
			if (thetas.z <= 1.0){
				deltas[index] += -normFac * deltasin.z;
				deltas[index + dy - 1] += normFac * deltasin.z;//* ai/an = 1.0
			}
			index++;
		}
	}
	int triangleNum = g_numTriangles;
	for (int i = 0; i < triangleNum; i++){
		g_saturations[i] += deltas[i];
		if (g_saturations[i] < 0.0){
			//printf("mark0\n");
			g_saturations[i] = 0.0;
		}
		//if (g_saturations[i] > 1.0){
		//	//printf("mark\n");
		//	//g_saturations[i] = 1.0;
		//}
		//printf("%d, %f, %f\n", i, deltas[i], g_saturations[i]);

	}
}

void CreateParticle(int index, int &activeCount){


	Vec3 emitterDir = Vec3(0.0f, -1.0f, 0.0f);
	Vec3 emitterRight = Vec3(-1.0f, 0.0f, 0.0f);

	//position
	Vec3 centerPos = g_triangleCenters[index];
	Vec3 aPos;
	Vec3 bPos;
	Vec3 cPos;

	int x = index / 2 % (g_dx - 1);
	int y = index / 2 / (g_dx - 1);

	if (index % 2 == 0){
		aPos = Vec3(g_positions[y * g_dx + x]);
		bPos = Vec3(g_positions[y * g_dx + x + 1]);
		cPos = Vec3(g_positions[y * g_dx + x + 1 + g_dx]);
	}
	else {
		aPos = Vec3(g_positions[y * g_dx + x]);
		bPos = Vec3(g_positions[y * g_dx + x + 1 + g_dx]);
		cPos = Vec3(g_positions[y * g_dx + x + g_dx]);
	}

	int a = rand() % 101;
	int b = rand() % (101 - a);
	int c = rand() % (101 - a - b);
	Vec3 emitterPos = (centerPos * (101 - a - b - c) + aPos * a + bPos * b + cPos * c) / 100.0;
	emitterPos -= g_triangleNormals[index] * g_params.mCollisionDistance * 2.0;

	/*naive version*/
	//Vec3 emitterPos = g_triangleCenters[index] - g_triangleNormals[index] * g_params.mCollisionDistance * 2.0;

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

	if (size_t(activeCount) < g_positions.size()){
		g_positions[activeCount] = Vec4(emitterPos, 1.0f);
		g_velocities[activeCount] = Vec3(0.0f, 0.0f, 0.0f);
		g_phases[activeCount] = phase;
		g_absorbable[activeCount] = false;
		activeCount++;

	}


}

void Dripping(){

	if (g_dripBuffer.size() == 0)
		g_dripBuffer.resize(g_numTriangles);

	int triangleNum = g_numTriangles;

	int activeCount = flexGetActiveCount(g_flex);
	//printf("%d ", activeCount);

	for (int i = 0; i < triangleNum; i++){
		if (g_saturations[i] > g_maxSaturation){
			float m = g_saturations[i] - g_maxSaturation;
			g_dripBuffer[i] += m;
			while (g_dripBuffer[i] > g_mDrip){

				CreateParticle(i, activeCount);

				g_dripBuffer[i] -= g_mDrip;
			}
			g_saturations[i] = g_maxSaturation;
		}
	}

	//printf(" %d \n", activeCount);

	flexSetActive(g_flex, &g_activeIndices[0], activeCount, eFlexMemoryHost);
}