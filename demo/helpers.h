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
void CreateParticleShape2(const char* file, Vec3 lower, Vec3 scale, float rotation, float spacing, Vec3 velocity, float invMass, bool rigid, float rigidStiffness, int phase, bool skin, float jitter = 0.005f, Vec3 skinOffset = 0.0f, float skinExpand = 0.0f, Vec4 color = Vec4(0.0f))
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

		Vec3 edges = meshUpper - meshLower;
		float maxEdge = max(max(edges.x, edges.y), edges.z);

		// put mesh at the origin and scale to specified size
		Matrix44 xform = ScaleMatrix(scale / maxEdge)*TranslationMatrix(Point3(-meshLower));

		mesh->Transform(xform);
		mesh->GetBounds(meshLower, meshUpper);

		// recompute expanded edges
		edges = meshUpper - meshLower;
		maxEdge = max(max(edges.x, edges.y), edges.z);

		// tweak spacing to avoid edge cases for particles laying on the boundary
		// just covers the case where an edge is a whole multiple of the spacing.
		float spacingEps = spacing*(1.0f - 1e-4f);

		// make sure to have at least one particle in each dimension
		int dx, dy, dz;
		dx = spacing > edges.x ? 1 : int(edges.x / spacingEps);
		dy = spacing > edges.y ? 1 : int(edges.y / spacingEps);
		dz = spacing > edges.z ? 1 : int(edges.z / spacingEps);

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
		meshOffset.x = 0.5f * (spacing - (edges.x - (dx - 1)*spacing));
		meshOffset.y = 0.5f * (spacing - (edges.y - (dy - 1)*spacing));
		meshOffset.z = 0.5f * (spacing - (edges.z - (dz - 1)*spacing));
		meshLower -= meshOffset;

		//Voxelize(*mesh, dx, dy, dz, &voxels[0], meshLower - Vec3(spacing*0.05f) , meshLower + Vec3(maxDim*spacing) + Vec3(spacing*0.05f));
		Voxelize((const float*)&mesh->m_positions[0], mesh->m_positions.size(), (const int*)&mesh->m_indices[0], mesh->m_indices.size(), maxDim, maxDim, maxDim, &voxels[0], meshLower, meshLower + Vec3(maxDim*spacing));

		vector<float> sdf(maxDim*maxDim*maxDim);
		MakeSDF(&voxels[0], maxDim, maxDim, maxDim, &sdf[0]);

		for (int x = 0; x < maxDim; ++x)
		{
			for (int y = 0; y < maxDim; ++y)
			{
				for (int z = 0; z < maxDim; ++z)
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
						float d = sdf[index] * maxEdge;

						if (rigid)
							g_rigidLocalNormals.push_back(Vec4(n, d));

						g_positions.push_back(Vec4(position.x, position.y, position.z, invMass));
						g_velocities.push_back(velocity);
						g_phases.push_back(phase);
					}
				}
			}
		}
		mesh->Transform(ScaleMatrix(1.0f + skinExpand)*TranslationMatrix(Point3(-0.5f*(meshUpper + meshLower))));
		mesh->Transform(TranslationMatrix(Point3(lower + 0.5f*(meshUpper + meshLower))));
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

		for (uint32_t i = startVertex; i < mesh->GetNumVertices(); ++i)
		{
			int indices[g_numSkinWeights] = { -1, -1, -1, -1 };
			float distances[g_numSkinWeights] = { FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX };

			if (LengthSq(color) == 0.0f)
				mesh->m_colours[i] = 1.25f*colors[phase % 7];
			else
				mesh->m_colours[i] = Colour(color);

			// find closest n particles
			for (size_t j = startIndex; j < g_positions.size(); ++j)
			{
				float dSq = LengthSq(Vec3(mesh->m_positions[i]) - Vec3(g_positions[j]));

				// insertion sort
				int w = 0;
				for (; w < 4; ++w)
					if (dSq < distances[w])
						break;

				if (w < 4)
				{
					// shuffle down
					for (int s = 3; s > w; --s)
					{
						indices[s] = indices[s - 1];
						distances[s] = distances[s - 1];
					}

					distances[w] = dSq;
					indices[w] = int(j);
				}
			}

			// weight particles according to distance
			float wSum = 0.0f;

			for (int w = 0; w < 4; ++w)
			{
				// convert to inverse distance
				distances[w] = 1.0f / (0.1f + powf(distances[w], .125f));

				wSum += distances[w];

			}

			float weights[4];
			for (int w = 0; w < 4; ++w)
				weights[w] = distances[w] / wSum;

			for (int j = 0; j < 4; ++j)
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

void CreateSDF2(const char* meshFile, float scale, Vec3 lower, float expand = 0.0f)
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
		edges = maxExtents - minExtents;

		// normalize mesh scale
		float longestAxis = max(max(edges.x, edges.y), edges.z);

		mesh->Transform(TranslationMatrix(Point3(-Vec3(minExtents))));
		//mesh->Transform(RotationMatrix(-kPi*0.0f, Vec3(1.0f, 0.0f, 0.0f)));
		mesh->Transform(ScaleMatrix(scale / longestAxis));
		mesh->Transform(TranslationMatrix(Point3(lower)));

		mesh->GetBounds(minExtents, maxExtents);
		mesh->m_colours.resize(0);

		/*add begin*/
		g_numPoints = mesh->GetNumVertices();
		g_numTriangles = mesh->GetNumFaces();

		g_saturations.resize(g_numTriangles, 0.0);
		g_triangleNeighbours.resize(g_numTriangles, Vec3(-1.0, -1.0, -1.0));

		g_pointTriangleNums = mesh->m_pointTriangleNums;
		g_pointTriangles = mesh->m_pointTriangles;
		g_trianglePoints = mesh->m_trianglePoints;

		mesh->m_pointTriangleNums.resize(0); 
		mesh->m_pointTriangles.resize(0);
		mesh->m_trianglePoints.resize(0);
		/*add end*/

		// store mesh 
		g_mesh = mesh;

		// square extents
		edges = maxExtents - minExtents;
		longestAxis = max(max(edges.x, edges.y), edges.z);
		edges = longestAxis;

		minExtents = minExtents - edges*0.1f;
		maxExtents = minExtents + edges*1.1f;
		edges = maxExtents - minExtents;

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
		for (int i = 0; i < numVoxels; ++i)
			sdf.m_data[i] += expand;

		// set up flex collision shape
		g_shape.mWidth = sdf.m_width;
		g_shape.mHeight = sdf.m_height;
		g_shape.mDepth = sdf.m_depth;
		(Vec3&)g_shape.mLower = minExtents;
		(Vec3&)g_shape.mUpper = maxExtents;
		(Vec3&)g_shape.mInvEdgeLength = Vec3(1.0f / edges.x, 1.0f / edges.y, 1.0f / edges.z);
		g_shape.mField = sdf.m_data;
	}
}


inline int GridIndex(int x, int y, int dx) { return y*dx + x; }
inline int GridIndex(int x, int y, int z, int dx, int dy){ return z * dx * dy + y * dx + x; }


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
void CreateSpringGridVertical(Vec3 lower, int dx, int dy, int dz, float radius, int phase, float stretchStiffness, float bendStiffness, float shearStiffness, Vec3 velocity, float invMass)
{
	int baseIndex = int(g_positions.size());

	for (int z = 0; z < dz; ++z)
	{
		for (int y = 0; y < dy; ++y)
		{
			for (int x = 0; x < dx; ++x)
			{
				Vec3 position = lower + radius*Vec3(float(z), float(x), float(y));

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
	for (int y = 0; y < dy; ++y)
	{
		for (int x = 0; x < dx; ++x)
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

			if (y > 0 && x < dx - 1)
			{
				int indexDiag = (y - 1)*dx + x + 1;
				CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
			}

			if (y > 0 && x > 0)
			{
				int indexDiag = (y - 1)*dx + x - 1;
				CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
			}
		}
	}

	// vertical
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

void renewTrianglePoints(int idxp, int idxpt, int idxt){

	Vec4 tmp = g_pointTriangles[idxp];
	switch (idxpt)
	{
	case 0:
		tmp.w = idxt;
		break;
	case 1:
		tmp.x = idxt;
		break;
	case 2:
		tmp.y = idxt;
		break;
	case 3:
		tmp.z = idxt;
		break;
	default:
		break;
	}
	g_pointTriangles[idxp] = tmp;
}
void AddTrianglesToPoints(int idxp, int idxt1, int idxt2){
	//{
	//	Vec4 tmp1 = g_pointTriangles[idxp * 2];
	//	Vec4 tmp2 = g_pointTriangles[idxp * 2 + 1];
	//	printf("point %d\n", idxp);
	//	printf("%d %d %d %d %d %d %d %d\n", int(tmp1.w), int(tmp1.x), int(tmp1.y), int(tmp1.z), int(tmp2.w), int(tmp2.x), int(tmp2.y), int(tmp2.z));
	//}
	int idx = g_pointTriangleNums[idxp];
	if (idxt1 > -1){
		if (idx < 4){
			renewTrianglePoints(idxp * 2, idx, idxt1);
			g_pointTriangleNums[idxp] = idx + 1;
			idx++;
		}
		else {
			renewTrianglePoints(idxp * 2 + 1, idx - 4, idxt1);
			g_pointTriangleNums[idxp] = idx + 1;
			idx++;
		}
	}
	if (idxt2 > -1){
		if (idx < 4){
			renewTrianglePoints(idxp * 2, idx, idxt2);
			g_pointTriangleNums[idxp] = idx + 1;
		}
		else {
			renewTrianglePoints(idxp * 2 + 1, idx - 4, idxt2);
			g_pointTriangleNums[idxp] = idx + 1;
		}
	}
	//{
	//	Vec4 tmp1 = g_pointTriangles[idxp * 2];
	//	Vec4 tmp2 = g_pointTriangles[idxp * 2 + 1];
	//	printf("%d %d %d %d %d %d %d %d\n", int(tmp1.w), int(tmp1.x), int(tmp1.y), int(tmp1.z), int(tmp2.w), int(tmp2.x), int(tmp2.y), int(tmp2.z));
	//}
}
void CreateSpringGrid2(Vec3 lower, int dx, int dy, int dz, float radius, int phase, float stretchStiffness, float bendStiffness, float shearStiffness, Vec3 velocity, float invMass)
{
	int baseIndex = int(g_positions.size());

	int index = 0;

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

				int i = GridIndex(x, y, z, dx, dy);
				g_pointTriangleNums[i] = 0;
				g_pointTriangles[i * 2] = Vector4(-1.0, -1.0, -1.0, -1.0);
				g_pointTriangles[i * 2 + 1] = Vector4(-1.0, -1.0, -1.0, -1.0);

				if (x > 0 && y > 0 && (z == 0 || z == dz - 1)){
					int p1 = GridIndex(x - 1, y - 1, z, dx, dy);
					int p2 = GridIndex(x, y - 1, z, dx, dy);
					int p3 = GridIndex(x, y, z, dx, dy);
					int p4 = GridIndex(x - 1, y, z, dx, dy);
					//p1 < p2 < p4 < p3

					g_triangles.push_back(baseIndex + p1);
					g_triangles.push_back(baseIndex + p2);
					g_triangles.push_back(baseIndex + p3);

					g_triangles.push_back(baseIndex + p1);
					g_triangles.push_back(baseIndex + p3);
					g_triangles.push_back(baseIndex + p4);
					
					if (z == 0){
						g_triangleNormals.push_back(Vec3(0.0f, -1.0f, 0.0f));
						g_triangleNormals.push_back(Vec3(0.0f, -1.0f, 0.0f));
					}
					else {
						g_triangleNormals.push_back(Vec3(0.0f, 1.0f, 0.0f));
						g_triangleNormals.push_back(Vec3(0.0f, 1.0f, 0.0f));
					}

					g_saturations.push_back(0.0);
					g_saturations.push_back(0.0);

					g_triangleNeighbours.push_back(Vec3(-1.0, -1.0, -1.0));
					g_triangleNeighbours.push_back(Vec3(-1.0, -1.0, -1.0));

					g_trianglePoints.push_back(Vec3(p1, p2, p3));
					g_trianglePoints.push_back(Vec3(p1, p4, p3));

					AddTrianglesToPoints(p1, index, index + 1);
					AddTrianglesToPoints(p2, index, -1);
					AddTrianglesToPoints(p3, index, index + 1);
					AddTrianglesToPoints(p4, index + 1, -1);
					index += 2;
				}
				if (x > 0 && z > 0 && (y == 0 || y == dy - 1)){
					int p1 = GridIndex(x - 1, y, z - 1, dx, dy);
					int p2 = GridIndex(x, y, z - 1, dx, dy);
					int p3 = GridIndex(x, y, z, dx, dy);
					int p4 = GridIndex(x - 1, y, z, dx, dy);

					g_triangles.push_back(baseIndex + p1);
					g_triangles.push_back(baseIndex + p2);
					g_triangles.push_back(baseIndex + p3);

					g_triangles.push_back(baseIndex + p1);
					g_triangles.push_back(baseIndex + p3);
					g_triangles.push_back(baseIndex + p4);

					if (y == 0){
						g_triangleNormals.push_back(Vec3(0.0f, 0.0f, -1.0f));
						g_triangleNormals.push_back(Vec3(0.0f, 0.0f, -1.0f));
					}
					else {
						g_triangleNormals.push_back(Vec3(0.0f, 0.0f, 1.0f));
						g_triangleNormals.push_back(Vec3(0.0f, 0.0f, 1.0f));
					}

					g_saturations.push_back(0.0);
					g_saturations.push_back(0.0);

					g_triangleNeighbours.push_back(Vec3(0, 0, 0));
					g_triangleNeighbours.push_back(Vec3(0, 0, 0));

					g_trianglePoints.push_back(Vec3(p1, p2, p3));
					g_trianglePoints.push_back(Vec3(p1, p4, p3));

					AddTrianglesToPoints(p1, index, index + 1);
					AddTrianglesToPoints(p2, index, -1);
					AddTrianglesToPoints(p3, index, index + 1);
					AddTrianglesToPoints(p4, index + 1, -1);
					index += 2;
				}
				if (y > 0 && z > 0 && (x == 0 || x == dx - 1)){
					int p1 = GridIndex(x, y - 1, z - 1, dx, dy);
					int p2 = GridIndex(x, y, z - 1, dx, dy);
					int p3 = GridIndex(x, y, z, dx, dy);
					int p4 = GridIndex(x, y - 1, z, dx, dy);

					g_triangles.push_back(baseIndex + p1);
					g_triangles.push_back(baseIndex + p2);
					g_triangles.push_back(baseIndex + p3);

					g_triangles.push_back(baseIndex + p1);
					g_triangles.push_back(baseIndex + p3);
					g_triangles.push_back(baseIndex + p4);

					if (x == 0){
						g_triangleNormals.push_back(Vec3(-1.0f, 0.0f, 0.0f));
						g_triangleNormals.push_back(Vec3(-1.0f, 0.0f, 0.0f));
					}
					else {
						g_triangleNormals.push_back(Vec3(1.0f, 0.0f, 0.0f));
						g_triangleNormals.push_back(Vec3(1.0f, 0.0f, 0.0f));
					}

					g_saturations.push_back(0.0);
					g_saturations.push_back(0.0);

					g_triangleNeighbours.push_back(Vec3(0, 0, 0));
					g_triangleNeighbours.push_back(Vec3(0, 0, 0));

					g_trianglePoints.push_back(Vec3(p1, p2, p3));
					g_trianglePoints.push_back(Vec3(p1, p4, p3));

					AddTrianglesToPoints(p1, index, index + 1);
					AddTrianglesToPoints(p2, index, -1);
					AddTrianglesToPoints(p3, index, index + 1);
					AddTrianglesToPoints(p4, index + 1, -1);
					index += 2;
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
				int index0 = z * dx * dy + y*dx + x;

				if (x > 0){
					int index1 = z * dy * dx + y*dx + x - 1;
					CreateSpring(baseIndex + index0, baseIndex + index1, stretchStiffness);
				}

				if (x > 1){
					int index2 = z * dy * dx + y*dx + x - 2;
					CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
				}

				if (y > 0){
					int index1 = z * dy * dx + (y - 1)*dx + x;
					CreateSpring(baseIndex + index0, baseIndex + index1, stretchStiffness);
				}

				if (y > 1){
					int index2 = z * dy * dx + (y - 2)*dx + x;
					CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
				}

				if (z > 0){
					int index1 = (z - 1) * dy * dx + y * dx + x;
					CreateSpring(baseIndex + index0, baseIndex + index1, bendStiffness);
				}

				if (z > 1){
					int index2 = (z - 2) * dy * dx + y * dx + x;
					CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
				}

				if (y > 0 && x < dx - 1){
					int indexDiag = z * dy * dx + (y - 1)*dx + x + 1;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}

				if (y > 0 && x > 0){
					int indexDiag = z * dy * dx + (y - 1)*dx + x - 1;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}

				if (z > 0 && x < dx - 1){
					int indexDiag = (z - 1) * dy * dx + y * dx + x + 1;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}

				if (z > 0 && x > 0){
					int indexDiag = (z - 1) * dy * dx + y * dx + x - 1;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}

				if (z > 0 && y < dy - 1){
					int indexDiag = (z - 1) * dy * dx + (y + 1) * dx + x;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}

				if (z > 0 && y > 0){
					int indexDiag = (z - 1) * dy * dx + (y - 1) * dx + x;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}

			}
		}
	}

}

void CreateSpringGrid3(Vec3 lower, int dx, int dy, int dz, float radius, int phase, float stretchStiffness, float bendStiffness, float shearStiffness, Vec3 velocity, float invMass)
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
			}
		}
	}

	//triangles
	//down
	{
		int z = 0;
		for (int y = 0; y < dy; y++){
			for (int x = 0; x < dx; x++){
				if (x > 0 && y > 0){
					g_triangles.push_back(baseIndex + GridIndex(x - 1, y - 1, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y - 1, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z, dx, dy));

					g_triangles.push_back(baseIndex + GridIndex(x - 1, y - 1, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x - 1, y, z, dx, dy));

					g_triangleNormals.push_back(Vec3(0.0f, 1.0f, 0.0f));
					g_triangleNormals.push_back(Vec3(0.0f, 1.0f, 0.0f));

					g_saturations.push_back(0.0);
					g_saturations.push_back(0.0);
				}
			}
		}
	}
	//up
	{
		int z = dz - 1;
		for (int y = 0; y < dy; y++){
			for (int x = 0; x < dx; x++){
				if (x > 0 && y > 0){
					g_triangles.push_back(baseIndex + GridIndex(x - 1, y - 1, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y - 1, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x - 1, y, z, dx, dy));

					g_triangles.push_back(baseIndex + GridIndex(x, y - 1, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x - 1, y, z, dx, dy));

					g_triangleNormals.push_back(Vec3(0.0f, 1.0f, 0.0f));
					g_triangleNormals.push_back(Vec3(0.0f, 1.0f, 0.0f));

					g_saturations.push_back(0.0);
					g_saturations.push_back(0.0);
				}
			}
		}
	}
	//back
	{
		int y = 0;
		for (int z = 0; z < dz; z++){
			for (int x = 0; x < dx; x++){
				if (x > 0 && z > 0){
					g_triangles.push_back(baseIndex + GridIndex(x - 1, y, z - 1, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x - 1, y, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z - 1, dx, dy));

					g_triangles.push_back(baseIndex + GridIndex(x - 1, y, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z - 1, dx, dy));

					g_triangleNormals.push_back(Vec3(0.0f, 0.0f, 1.0f));
					g_triangleNormals.push_back(Vec3(0.0f, 0.0f, 1.0f));

					g_saturations.push_back(0.0);
					g_saturations.push_back(0.0);
				}
			}
		}
	}
	//front
	{
		int y = dy - 1;
		for (int z = 0; z < dz; z++){
			for (int x = 0; x < dx; x++){
				if (x > 0 && z > 0){
					g_triangles.push_back(baseIndex + GridIndex(x - 1, y, z - 1, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z - 1, dx, dy));

					g_triangles.push_back(baseIndex + GridIndex(x - 1, y, z - 1, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x - 1, y, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z, dx, dy));

					g_triangleNormals.push_back(Vec3(0.0f, 0.0f, 1.0f));
					g_triangleNormals.push_back(Vec3(0.0f, 0.0f, 1.0f));

					g_saturations.push_back(0.0);
					g_saturations.push_back(0.0);
				}
			}
		}
	}
	//left
	{
		int x = 0;
		for (int z = 0; z < dz; z++){
			for (int y = 0; y < dy; y++){
				if (y > 0 && z > 0){
					g_triangles.push_back(baseIndex + GridIndex(x, y - 1, z - 1, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y - 1, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z - 1, dx, dy));

					g_triangles.push_back(baseIndex + GridIndex(x, y - 1, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z - 1, dx, dy));

					g_triangleNormals.push_back(Vec3(1.0f, 0.0f, 0.0f));
					g_triangleNormals.push_back(Vec3(1.0f, 0.0f, 0.0f));

					g_saturations.push_back(0.0);
					g_saturations.push_back(0.0);

				}
			}
		}
	}
	//right
	{

		int x = dx - 1;
		for (int z = 0; z < dz; z++){
			for (int y = 0; y < dy; y++){
				if (y > 0 && z > 0){
					g_triangles.push_back(baseIndex + GridIndex(x, y - 1, z - 1, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z - 1, dx, dy));

					g_triangles.push_back(baseIndex + GridIndex(x, y - 1, z - 1, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y - 1, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z, dx, dy));

					g_triangleNormals.push_back(Vec3(1.0f, 0.0f, 0.0f));
					g_triangleNormals.push_back(Vec3(1.0f, 0.0f, 0.0f));

					g_saturations.push_back(0.0);
					g_saturations.push_back(0.0);

				}
			}
		}
	}

	// spring
	for (int z = 0; z < dz; ++z)
	{
		for (int y = 0; y < dy; ++y)
		{
			for (int x = 0; x < dx; ++x)
			{
				int index0 = z * dx * dy + y*dx + x;

				if (x > 0){
					int index1 = z * dy * dx + y*dx + x - 1;
					CreateSpring(baseIndex + index0, baseIndex + index1, stretchStiffness);
				}

				if (x > 1){
					int index2 = z * dy * dx + y*dx + x - 2;
					CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
				}

				if (y > 0){
					int index1 = z * dy * dx + (y - 1)*dx + x;
					CreateSpring(baseIndex + index0, baseIndex + index1, stretchStiffness);
				}

				if (y > 1){
					int index2 = z * dy * dx + (y - 2)*dx + x;
					CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
				}

				if (z > 0){
					int index1 = (z - 1) * dy * dx + y * dx + x;
					CreateSpring(baseIndex + index0, baseIndex + index1, bendStiffness);
				}

				if (z > 1){
					int index2 = (z - 2) * dy * dx + y * dx + x;
					CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
				}

				if (y > 0 && x < dx - 1){
					int indexDiag = z * dy * dx + (y - 1)*dx + x + 1;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}

				if (y > 0 && x > 0){
					int indexDiag = z * dy * dx + (y - 1)*dx + x - 1;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}

				if (z > 0 && x < dx - 1){
					int indexDiag = (z - 1) * dy * dx + y * dx + x + 1;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}

				if (z > 0 && x > 0){
					int indexDiag = (z - 1) * dy * dx + y * dx + x - 1;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}

				if (z > 0 && y < dy - 1){
					int indexDiag = (z - 1) * dy * dx + (y + 1) * dx + x;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}

				if (z > 0 && y > 0){
					int indexDiag = (z - 1) * dy * dx + (y - 1) * dx + x;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}

			}
		}
	}

}

void CreateSpringGridCube(Vec3 lower, int dx, int dy, int dz, float radius, int phase, float stretchStiffness, float bendStiffness, float shearStiffness, Vec3 velocity, float invMass)
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

				if (x > 0 && y > 0 && z > 0){
					g_saturations.push_back(0.0);
				}
			}
		}
	}

	//triangles
	//down
	{
		int z = 0;
		for (int y = 0; y < dy; y++){
			for (int x = 0; x < dx; x++){
				if (x > 0 && y > 0){
					g_triangles.push_back(baseIndex + GridIndex(x - 1, y - 1, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y - 1, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z, dx, dy));

					g_triangles.push_back(baseIndex + GridIndex(x - 1, y - 1, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x - 1, y, z, dx, dy));

					g_triangleNormals.push_back(Vec3(0.0f, 1.0f, 0.0f));
					g_triangleNormals.push_back(Vec3(0.0f, 1.0f, 0.0f));

				}
			}
		}
	}
	//up
	{
		int z = dz - 1;
		for (int y = 0; y < dy; y++){
			for (int x = 0; x < dx; x++){
				if (x > 0 && y > 0){
					g_triangles.push_back(baseIndex + GridIndex(x - 1, y - 1, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y - 1, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x - 1, y, z, dx, dy));

					g_triangles.push_back(baseIndex + GridIndex(x, y - 1, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x - 1, y, z, dx, dy));

					g_triangleNormals.push_back(Vec3(0.0f, 1.0f, 0.0f));
					g_triangleNormals.push_back(Vec3(0.0f, 1.0f, 0.0f));

				}
			}
		}
	}
	//back
	{
		int y = 0;
		for (int z = 0; z < dz; z++){
			for (int x = 0; x < dx; x++){
				if (x > 0 && z > 0){
					g_triangles.push_back(baseIndex + GridIndex(x - 1, y, z - 1, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x - 1, y, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z - 1, dx, dy));

					g_triangles.push_back(baseIndex + GridIndex(x - 1, y, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z - 1, dx, dy));

					g_triangleNormals.push_back(Vec3(0.0f, 0.0f, 1.0f));
					g_triangleNormals.push_back(Vec3(0.0f, 0.0f, 1.0f));

				}
			}
		}
	}
	//front
	{
		int y = dy - 1;
		for (int z = 0; z < dz; z++){
			for (int x = 0; x < dx; x++){
				if (x > 0 && z > 0){
					g_triangles.push_back(baseIndex + GridIndex(x - 1, y, z - 1, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z - 1, dx, dy));

					g_triangles.push_back(baseIndex + GridIndex(x - 1, y, z - 1, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x - 1, y, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z, dx, dy));

					g_triangleNormals.push_back(Vec3(0.0f, 0.0f, 1.0f));
					g_triangleNormals.push_back(Vec3(0.0f, 0.0f, 1.0f));

				}
			}
		}
	}
	//left
	{
		int x = 0;
		for (int z = 0; z < dz; z++){
			for (int y = 0; y < dy; y++){
				if (y > 0 && z > 0){
					g_triangles.push_back(baseIndex + GridIndex(x, y - 1, z - 1, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y - 1, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z - 1, dx, dy));

					g_triangles.push_back(baseIndex + GridIndex(x, y - 1, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z - 1, dx, dy));

					g_triangleNormals.push_back(Vec3(1.0f, 0.0f, 0.0f));
					g_triangleNormals.push_back(Vec3(1.0f, 0.0f, 0.0f));

				}
			}
		}
	}
	//right
	{

		int x = dx - 1;
		for (int z = 0; z < dz; z++){
			for (int y = 0; y < dy; y++){
				if (y > 0 && z > 0){
					g_triangles.push_back(baseIndex + GridIndex(x, y - 1, z - 1, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z - 1, dx, dy));

					g_triangles.push_back(baseIndex + GridIndex(x, y - 1, z - 1, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y - 1, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z, dx, dy));

					g_triangleNormals.push_back(Vec3(1.0f, 0.0f, 0.0f));
					g_triangleNormals.push_back(Vec3(1.0f, 0.0f, 0.0f));


				}
			}
		}
	}

	// spring
	for (int z = 0; z < dz; ++z)
	{
		for (int y = 0; y < dy; ++y)
		{
			for (int x = 0; x < dx; ++x)
			{
				int index0 = z * dx * dy + y*dx + x;

				if (x > 0){
					int index1 = z * dy * dx + y*dx + x - 1;
					CreateSpring(baseIndex + index0, baseIndex + index1, stretchStiffness);
				}

				if (x > 1){
					int index2 = z * dy * dx + y*dx + x - 2;
					CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
				}

				if (y > 0){
					int index1 = z * dy * dx + (y - 1)*dx + x;
					CreateSpring(baseIndex + index0, baseIndex + index1, stretchStiffness);
				}

				if (y > 1){
					int index2 = z * dy * dx + (y - 2)*dx + x;
					CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
				}

				if (z > 0){
					int index1 = (z - 1) * dy * dx + y * dx + x;
					CreateSpring(baseIndex + index0, baseIndex + index1, bendStiffness);
				}

				if (z > 1){
					int index2 = (z - 2) * dy * dx + y * dx + x;
					CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
				}

				if (y > 0 && x < dx - 1){
					int indexDiag = z * dy * dx + (y - 1)*dx + x + 1;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}

				if (y > 0 && x > 0){
					int indexDiag = z * dy * dx + (y - 1)*dx + x - 1;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}

				if (z > 0 && x < dx - 1){
					int indexDiag = (z - 1) * dy * dx + y * dx + x + 1;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}

				if (z > 0 && x > 0){
					int indexDiag = (z - 1) * dy * dx + y * dx + x - 1;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}

				if (z > 0 && y < dy - 1){
					int indexDiag = (z - 1) * dy * dx + (y + 1) * dx + x;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}

				if (z > 0 && y > 0){
					int indexDiag = (z - 1) * dy * dx + (y - 1) * dx + x;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}

			}
		}
	}

}

void CreateSpringGridPointHollow(Vec3 lower, int dx, int dy, int dz, float radius, int phase, float stretchStiffness, float bendStiffness, float shearStiffness, Vec3 velocity, float invMass){
	
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

				g_saturations.push_back(0.0);

				if (x > 0 && y > 0 && (z == 0 || z == dz - 1)){
					g_triangles.push_back(baseIndex + GridIndex(x - 1, y - 1, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y - 1, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z, dx, dy));

					g_triangles.push_back(baseIndex + GridIndex(x - 1, y - 1, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x - 1, y, z, dx, dy));

					g_triangleNormals.push_back(Vec3(0.0f, 1.0f, 0.0f));
					g_triangleNormals.push_back(Vec3(0.0f, 1.0f, 0.0f));

				}
				if (x > 0 && z > 0 && (y == 0 || y == dy - 1)){
					g_triangles.push_back(baseIndex + GridIndex(x - 1, y, z - 1, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z - 1, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z, dx, dy));

					g_triangles.push_back(baseIndex + GridIndex(x - 1, y, z - 1, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x - 1, y, z, dx, dy));

					g_triangleNormals.push_back(Vec3(0.0f, 0.0f, 1.0f));
					g_triangleNormals.push_back(Vec3(0.0f, 0.0f, 1.0f));

				}
				if (y > 0 && z > 0 && (x == 0 || x == dx - 1)){
					g_triangles.push_back(baseIndex + GridIndex(x, y - 1, z - 1, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z - 1, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z, dx, dy));

					g_triangles.push_back(baseIndex + GridIndex(x, y - 1, z - 1, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y - 1, z, dx, dy));

					g_triangleNormals.push_back(Vec3(1.0f, 0.0f, 0.0f));
					g_triangleNormals.push_back(Vec3(1.0f, 0.0f, 0.0f));

				}
			}
		}
	}


	// spring
	for (int z = 0; z < dz; ++z)
	{
		for (int y = 0; y < dy; ++y)
		{
			for (int x = 0; x < dx; ++x)
			{
				int index0 = z * dx * dy + y*dx + x;

				if (x > 0){
					int index1 = z * dy * dx + y*dx + x - 1;
					CreateSpring(baseIndex + index0, baseIndex + index1, stretchStiffness);
				}

				if (x > 1){
					int index2 = z * dy * dx + y*dx + x - 2;
					CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
				}

				if (y > 0){
					int index1 = z * dy * dx + (y - 1)*dx + x;
					CreateSpring(baseIndex + index0, baseIndex + index1, stretchStiffness);
				}

				if (y > 1){
					int index2 = z * dy * dx + (y - 2)*dx + x;
					CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
				}

				if (z > 0){
					int index1 = (z - 1) * dy * dx + y * dx + x;
					CreateSpring(baseIndex + index0, baseIndex + index1, bendStiffness);
				}

				if (z > 1){
					int index2 = (z - 2) * dy * dx + y * dx + x;
					CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
				}

				if (y > 0 && x < dx - 1){
					int indexDiag = z * dy * dx + (y - 1)*dx + x + 1;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}

				if (y > 0 && x > 0){
					int indexDiag = z * dy * dx + (y - 1)*dx + x - 1;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}

				if (z > 0 && x < dx - 1){
					int indexDiag = (z - 1) * dy * dx + y * dx + x + 1;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}

				if (z > 0 && x > 0){
					int indexDiag = (z - 1) * dy * dx + y * dx + x - 1;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}

				if (z > 0 && y < dy - 1){
					int indexDiag = (z - 1) * dy * dx + (y + 1) * dx + x;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}

				if (z > 0 && y > 0){
					int indexDiag = (z - 1) * dy * dx + (y - 1) * dx + x;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}

			}
		}
	}

}

void CreateSpringGridPoint(Vec3 lower, int dx, int dy, int dz, float radius, int phase, float stretchStiffness, float bendStiffness, float shearStiffness, Vec3 velocity, float invMass)
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

				g_saturations.push_back(0.0);

				if (x > 0 && y > 0 && (z == 0 || z == dz - 1)){
					g_triangles.push_back(baseIndex + GridIndex(x - 1, y - 1, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y - 1, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z, dx, dy));

					g_triangles.push_back(baseIndex + GridIndex(x - 1, y - 1, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x - 1, y, z, dx, dy));

					g_triangleNormals.push_back(Vec3(0.0f, 1.0f, 0.0f));
					g_triangleNormals.push_back(Vec3(0.0f, 1.0f, 0.0f));

				}
				if (x > 0 && z > 0 && (y == 0 || y == dy - 1)){
					g_triangles.push_back(baseIndex + GridIndex(x - 1, y, z - 1, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z - 1, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z, dx, dy));

					g_triangles.push_back(baseIndex + GridIndex(x - 1, y, z - 1, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x - 1, y, z, dx, dy));

					g_triangleNormals.push_back(Vec3(0.0f, 0.0f, 1.0f));
					g_triangleNormals.push_back(Vec3(0.0f, 0.0f, 1.0f));

				}
				if (y > 0 && z > 0 && (x == 0 || x == dx - 1)){
					g_triangles.push_back(baseIndex + GridIndex(x, y - 1, z - 1, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z - 1, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z, dx, dy));

					g_triangles.push_back(baseIndex + GridIndex(x, y - 1, z - 1, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y, z, dx, dy));
					g_triangles.push_back(baseIndex + GridIndex(x, y - 1, z, dx, dy));

					g_triangleNormals.push_back(Vec3(1.0f, 0.0f, 0.0f));
					g_triangleNormals.push_back(Vec3(1.0f, 0.0f, 0.0f));

				}
			}
		}
	}

	// spring
	for (int z = 0; z < dz; ++z)
	{
		for (int y = 0; y < dy; ++y)
		{
			for (int x = 0; x < dx; ++x)
			{
				int index0 = z * dx * dy + y*dx + x;

				if (x > 0){
					int index1 = z * dy * dx + y*dx + x - 1;
					CreateSpring(baseIndex + index0, baseIndex + index1, stretchStiffness);
				}

				if (x > 1){
					int index2 = z * dy * dx + y*dx + x - 2;
					CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
				}

				if (y > 0){
					int index1 = z * dy * dx + (y - 1)*dx + x;
					CreateSpring(baseIndex + index0, baseIndex + index1, stretchStiffness);
				}

				if (y > 1){
					int index2 = z * dy * dx + (y - 2)*dx + x;
					CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
				}

				if (z > 0){
					int index1 = (z - 1) * dy * dx + y * dx + x;
					CreateSpring(baseIndex + index0, baseIndex + index1, bendStiffness);
				}

				if (z > 1){
					int index2 = (z - 2) * dy * dx + y * dx + x;
					CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
				}

				if (y > 0 && x < dx - 1){
					int indexDiag = z * dy * dx + (y - 1)*dx + x + 1;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}

				if (y > 0 && x > 0){
					int indexDiag = z * dy * dx + (y - 1)*dx + x - 1;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}

				if (z > 0 && x < dx - 1){
					int indexDiag = (z - 1) * dy * dx + y * dx + x + 1;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}

				if (z > 0 && x > 0){
					int indexDiag = (z - 1) * dy * dx + y * dx + x - 1;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}

				if (z > 0 && y < dy - 1){
					int indexDiag = (z - 1) * dy * dx + (y + 1) * dx + x;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
				}

				if (z > 0 && y > 0){
					int indexDiag = (z - 1) * dy * dx + (y - 1) * dx + x;
					CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
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




/*added */

/*initiate*/	
int trans(int ip, int idx){
	Vec4 tmp1 = g_pointTriangles[ip * 2];
	Vec4 tmp2 = g_pointTriangles[ip * 2 + 1];

	switch (idx)
	{
	case 0:
		return int(tmp1.w);
	case 1:
		return int(tmp1.x);
	case 2:
		return int(tmp1.y);
	case 3:
		return int(tmp1.z);
	case 4:
		return int(tmp2.w);
	case 5:
		return int(tmp2.x);
	case 6:
		return int(tmp2.y);
	case 7:
		return int(tmp2.z);
	default:
		break;
	}
	return 0;
}
void renewNeighbour(int it, int p, int in){
	Vec3 neighbours = g_triangleNeighbours[it];
	if (p == 0){
		neighbours.x = in;
	}
	else if (p == 1){
		neighbours.y = in;
	}
	else if (p == 2){
		neighbours.z = in;
	}
	g_triangleNeighbours[it] = neighbours;
}
void checkNeighbour(int ip, int it1, int it2){

	int idx1 = trans(ip, it1);
	int idx2 = trans(ip, it2);

//	printf("%d %d ", idx1, idx2);


	Vec3 point1 = g_trianglePoints[idx1];
	Vec3 point2 = g_trianglePoints[idx2];

//	printf("(%d, %d, %d) (%d, %d, %d)\n", int(point1.x), int(point1.y), int(point1.z), int(point2.x), int(point2.y), int(point2.z));

	int p1 = -1, p2 = -1;

	if (point1.x == point2.x && point1.y == point2.y){
		p1 = 0; p2 = 0;
	}
	else if (point1.x == point2.y && point1.y == point2.z){
		p1 = 0; p2 = 1;
	}
	else if (point1.x == point2.x && point1.y == point2.z){
		p1 = 0; p2 = 2;
	}
	else if (point1.y == point2.x && point1.z == point2.y){
		p1 = 1; p2 = 0;
	}
	else if (point1.y == point2.y && point1.z == point2.z){
		p1 = 1; p2 = 1;
	}
	else if (point1.y == point2.x && point1.z == point2.z){
		p1 = 1; p2 = 2;
	}
	else if (point1.x == point2.x && point1.z == point2.y){
		p1 = 2; p2 = 0;
	}
	else if (point1.x == point2.y && point1.z == point2.z){
		p1 = 2; p2 = 1;
	}
	else if (point1.x == point2.x && point1.z == point2.z){
		p1 = 2; p2 = 2;
	}

	if (p1 >= 0){
		renewNeighbour(idx1, p1, idx2);
	}
	if (p2 >= 0){
		renewNeighbour(idx2, p2, idx1);
	}
}
void CalculateTriangleNeighbours(){
	if (g_triangleNeighbours.size() == 0)
		g_triangleNeighbours.resize(g_numTriangles);

	for (int i = 0; i < g_numPoints; i++){
		int num = g_pointTriangleNums[i];
		//printf("%d: %d\n", i, num);
		for (int j = 0; j < num; j++){
			for (int k = j + 1; k < num; k++){
				checkNeighbour(i, j, k);
			}
		}
	}

	//for (int i = 0; i < g_numTriangles; i++){
	//	Vec3 neighbours = g_triangleNeighbours[i];
	//	printf("%d: %d %d %d\t", i, int(neighbours.x), int(neighbours.y), int(neighbours.z));
	//	Vec3 points = g_trianglePoints[i];
	//	printf("%d %d %d\n", int(points.x), int(points.y), int(points.z));
	//}
}


float getMin(float x, float y){
	if (x < y) return x;
	else return y;
}
float getMax(float x, float y){
	if (x > y) return x;
	else return y;
}


/*colors*/

void CalculateColors(){
	float maxSaturation = g_maxSaturation * g_kMaxAbsorption;

	Vec4 color = g_clothColor;
	Vec4 colorBase = color / maxSaturation;


	for (int i = 0; i < g_numPoints; i++){

		float saturation = 0.0;
		int num = g_pointTriangleNums[i];

		Vec4 tmp1 = g_pointTriangles[i * 2];
		Vec4 tmp2 = g_pointTriangles[i * 2 + 1];

		switch (num)
		{
		case 8:
			saturation += g_saturations[int(tmp2.z)];
		case 7:
			saturation += g_saturations[int(tmp2.y)];
		case 6:
			saturation += g_saturations[int(tmp2.x)];
		case 5:
			saturation += g_saturations[int(tmp2.w)];
		case 4:
			saturation += g_saturations[int(tmp1.z)];
		case 3:
			saturation += g_saturations[int(tmp1.y)];
		case 2:
			saturation += g_saturations[int(tmp1.x)];
		case 1:
			saturation += g_saturations[int(tmp1.w)];
		default:
			break;
		}

		if (num > 0){
			saturation = saturation / num;
		}

		//if (i % 4 == 0) printf("\n");
		//printf("%d %f\t", i, saturation);

		if (saturation < 0.0) saturation = 0.0;
		if (saturation > maxSaturation) saturation = maxSaturation;

		if (g_markColor && saturation > 0.0){
			g_colors[i] = g_markColors[(int)(saturation / maxSaturation * 10)];
		}
		else {
			g_colors[i] = colorBase * (maxSaturation - saturation);
		}

	}


}

void CalculateMeshColors(){
	if (g_mesh->m_colours.size() == 0){
		g_mesh->m_colours.resize(g_numPoints);
	}

	/*test*/
	if (0){
		for (int i = 0; i < g_numPoints; i++){
			g_mesh->m_colours[i] = Colour(1.0, 1.0, 1.0, 1.0);
			
		}
		if (colorFlag > 0){
			int i = colorFlag;
			Vec3 points = g_trianglePoints[i];
			g_mesh->m_colours[int(points.x)] = Colour(1.0, 0.0, 0.0, 1.0);
			g_mesh->m_colours[int(points.y)] = Colour(1.0, 0.0, 0.0, 1.0);
			g_mesh->m_colours[int(points.z)] = Colour(1.0, 0.0, 0.0, 1.0);
		}
		return;
	}

	float maxSaturation = g_maxSaturation * g_kMaxAbsorption;

	Vec4 color = g_clothColor;
	Vec4 colorBase = color / maxSaturation;

	int j = 0;

	for (int i = 0; i < g_numPoints; i++){

		//if (g_mesh->m_positions[i].y < g_mesh->m_positions[j].y){
		//	j = i;
		//}

		float saturation = 0.0;
		int num = g_pointTriangleNums[i];

		Vec4 tmp1 = g_pointTriangles[i * 2];
		Vec4 tmp2 = g_pointTriangles[i * 2 + 1];

		switch (num)
		{
		case 8:
			saturation += g_saturations[int(tmp2.z)];
		case 7:
			saturation += g_saturations[int(tmp2.y)];
		case 6:
			saturation += g_saturations[int(tmp2.x)];
		case 5:
			saturation += g_saturations[int(tmp2.w)];
		case 4:
			saturation += g_saturations[int(tmp1.z)];
		case 3:
			saturation += g_saturations[int(tmp1.y)];
		case 2:
			saturation += g_saturations[int(tmp1.x)];
		case 1:
			saturation += g_saturations[int(tmp1.w)];
		default:
			break;
		}

		if (num > 0){
			saturation = saturation / num;
		}

		//if (i % 4 == 0) printf("\n");
		//printf("%d %f\t", i, saturation);

		if (saturation < 0.0) saturation = 0.0;
		if (saturation > maxSaturation) saturation = maxSaturation;

		if (g_markColor && saturation > 0.0){
			Vec4 color = g_markColors[(int)(saturation / maxSaturation * 10)];
			g_mesh->m_colours[i] = Colour(color.x, color.y, color.z, color.w);
		}
		else {
			Vec4 color = colorBase * (maxSaturation - saturation);
			g_mesh->m_colours[i] = Colour(color.x, color.y, color.z, color.w);
			//g_mesh->m_colours[i] = Colour(0.0, 1.0, 1.0, 1.0);
		}

	}

	//j = 365;
	//printf("%d\n", j);
	//int t = int(g_pointTriangles[j * 2].x);
	//Vec3 normal = g_mesh->m_normals[t];
	//printf("(%f, %f, %f)\n", normal.x, normal.y, normal.z);
	//int p1 = int(g_trianglePoints[t].x);
	//int p2 = int(g_trianglePoints[t].y);
	//int p3 = int(g_trianglePoints[t].z);
	//g_mesh->m_colours[p1] = Colour(1.0, 0.0, 0.0, 0.0);
	//g_mesh->m_colours[p2] = Colour(1.0, 0.0, 0.0, 0.0);
	//g_mesh->m_colours[p3] = Colour(1.0, 0.0, 0.0, 0.0);
	//g_mesh->m_colours[j] = Colour(0.0, 0.0, 0.0, 0.0);
}

void CalculateClothColorsPoint(){
	float maxSaturation = g_maxSaturation * g_kMaxAbsorption;

	Vec4 color = g_clothColor;
	Vec4 colorBase = color / maxSaturation;

	for (int i = 0; i < g_numPoints; i++){
		float saturation = g_saturations[i];

		if (saturation < 0.0) saturation = 0.0;
		if (saturation > maxSaturation) saturation = maxSaturation;

		if (g_markColor && saturation > 0.0){
			g_colors[i] = g_markColors[(int)(saturation / maxSaturation * 10)];
		}
		else {
			g_colors[i] = colorBase * (maxSaturation - saturation);
		}

	}
}
void CalculateClothColorsCube(){
	int dimx = g_dx;
	int dimy = g_dy;
	int dimz = g_dz;

	int dx = 1;
	int dy = dimx - 1;
	int dz = (dimx - 1) * (dimy - 1);

	float maxSaturation = g_maxSaturation * g_kMaxAbsorption;

	Vec4 color = g_clothColor;
	Vec4 colorBase = color / maxSaturation;

	int index = 0;
	int max = g_numCubes;

	for (int z = 0; z < dimz; z++){
		for (int y = 0; y < dimy; y++){
			for (int x = 0; x < dimx; x++){

				float saturation = 0.0;
				float sum = 0.0;
				int num = 0;
				int xx, yy, zz;

				bool flag = false;

				//up right front
				xx = x; yy = y; zz = z;
				if (xx >= 0 && xx < dimx - 1 && yy >= 0 && yy < dimy - 1 && zz >= 0 && zz < dimz - 1){
					sum += g_saturations[xx + yy * dy + zz * dz];
					num++;
					//if (g_saturations[xx + yy * dy + zz * dz] >= maxSaturation){
					//	printf("1 ");
					//	flag = true;
					//}
				}
				//up left front
				xx = x - 1; yy = y; zz = z;
				if (xx >= 0 && xx < dimx - 1 && yy >= 0 && yy < dimy - 1 && zz >= 0 && zz < dimz - 1){
					sum += g_saturations[xx + yy * dy + zz * dz];
					num++;
					//if (g_saturations[xx + yy * dy + zz * dz] >= maxSaturation){
					//	printf("2 ");
					//	flag = true;
					//}
				}
				//up right back
				xx = x; yy = y - 1; zz = z;
				if (xx >= 0 && xx < dimx - 1 && yy >= 0 && yy < dimy - 1 && zz >= 0 && zz < dimz - 1){
					sum += g_saturations[xx + yy * dy + zz * dz];
					num++;
					//if (g_saturations[xx + yy * dy + zz * dz] >= maxSaturation){
					//	printf("3 ");
					//	flag = true;
					//}
				}
				//up left back
				xx = x - 1; yy = y - 1; zz = z;
				if (xx >= 0 && xx < dimx - 1 && yy >= 0 && yy < dimy - 1 && zz >= 0 && zz < dimz - 1){
					sum += g_saturations[xx + yy * dy + zz * dz];
					num++;
					//if (g_saturations[xx + yy * dy + zz * dz] >= maxSaturation){
					//	printf("4 ");
					//	flag = true;
					//}
				}
				//down right front
				xx = x; yy = y; zz = z - 1;
				if (xx >= 0 && xx < dimx - 1 && yy >= 0 && yy < dimy - 1 && zz >= 0 && zz < dimz - 1){
					sum += g_saturations[xx + yy * dy + zz * dz];
					num++;
					//if (g_saturations[xx + yy * dy + zz * dz] >= maxSaturation){
					//	printf("5 ");
					//	flag = true;
					//}
				}
				//down left front
				xx = x - 1; yy = y; zz = z - 1;
				if (xx >= 0 && xx < dimx - 1 && yy >= 0 && yy < dimy - 1 && zz >= 0 && zz < dimz - 1){
					sum += g_saturations[xx + yy * dy + zz * dz];
					num++;
					//if (g_saturations[xx + yy * dy + zz * dz] >= maxSaturation){
					//	printf("6 ");
					//	flag = true;
					//}
				}
				//down right back
				xx = x; yy = y - 1; zz = z - 1;
				if (xx >= 0 && xx < dimx - 1 && yy >= 0 && yy < dimy - 1 && zz >= 0 && zz < dimz - 1){
					sum += g_saturations[xx + yy * dy + zz * dz];
					num++;
					//if (g_saturations[xx + yy * dy + zz * dz] >= maxSaturation){
					//	printf("7 ");
					//	flag = true;
					//}
				}
				//down left back
				xx = x - 1; yy = y - 1; zz = z - 1;
				if (xx >= 0 && xx < dimx - 1 && yy >= 0 && yy < dimy - 1 && zz >= 0 && zz < dimz - 1){
					sum += g_saturations[xx + yy * dy + zz * dz];
					num++;
					//if (g_saturations[xx + yy * dy + zz * dz] >= maxSaturation){
					//	printf("8 ");
					//	flag = true;
					//}
				}

				saturation = sum / num;

				//printf("%d, %d, %d, %d: %f\n", index, x, y, z, saturation);
				//if (flag){
				//	printf("\n%d %d %d, %d %f\n", x, y, z, num, saturation);
				//}

				if (saturation < 0.0) saturation = 0.0;
				if (saturation > maxSaturation) saturation = maxSaturation;

				if (g_markColor && saturation > 0.0){
					g_colors[index] = g_markColors[(int)(saturation / maxSaturation * 10)];
				}
				else {
					g_colors[index] = colorBase * (maxSaturation - saturation);
				}

				index++;
			}
		}
	}

}
void CalculateClothColorsHollow(){

	int dimx = g_dx;
	int dimy = g_dy;
	int dimz = g_dz;
	int dx = 2;
	int dy = (dimx - 1) * 2;
	int dz = (dimx - 1) * (dimy - 1) * 2;

	int triangle_z = (dimx - 1) * (dimy - 1) * 2;	//up & down
	int triangle_y = (dimx - 1) * (dimz - 1) * 2;	//back & front
	int triangle_x = (dimy - 1) * (dimz - 1) * 2;	//left & right

	int start_down = 0;
	int start_up = start_down + triangle_z;
	int start_back = start_up + triangle_z;
	int start_front = start_back + triangle_y;
	int start_left = start_front + triangle_y;
	int start_right = start_left + triangle_x;

	int triangle_dx = (dimx - 1) * 2;
	int triangle_dy = (dimy - 1) * 2;

	float maxSaturation = g_maxSaturation * g_kMaxAbsorption;

	Vec4 color = g_clothColor;
	Vec4 colorBase = color / maxSaturation;

	//down
	{
		int z = 0;
		for (int y = 1; y < dimy - 1; y++){
			for (int x = 1; x < dimx - 1; x++){
				int idx0 = start_down + (x - 1) * 2 + (y - 1) * triangle_dx;		//down

				float saturation = (g_saturations[idx0] + g_saturations[idx0 + 1] + g_saturations[idx0 + 3] + g_saturations[idx0 + triangle_dx] + g_saturations[idx0 + triangle_dx + 2] + g_saturations[idx0 + triangle_dx + 3]) / 6;
				if (saturation < 0.0) saturation = 0.0;
				if (saturation > maxSaturation) saturation = maxSaturation;

				if (g_markColor && saturation > 0.0){
					g_colors[z * dimx * dimy + y * dimx + x] = g_markColors[(int)(saturation / maxSaturation * 10)];
				}
				else {
					g_colors[z * dimx * dimy + y * dimx + x] = colorBase * (maxSaturation - saturation);
				}
			}
		}
		//back edge
		{
			int y = 0;
			for (int x = 1; x < dimx - 1; x++){

				int idx0 = start_down + (x - 1) * 2;		//down
				int idx1 = start_back + x * 2 - 1;			//back

				float saturation = (g_saturations[idx0] + g_saturations[idx0 + 2] + g_saturations[idx0 + 3] + g_saturations[idx1] + g_saturations[idx1 - 1] + g_saturations[idx1 + 1]) / 6;
				if (saturation < 0.0) saturation = 0.0;
				if (saturation > maxSaturation) saturation = maxSaturation;

				if (g_markColor && saturation > 0.0){
					g_colors[z * dimx * dimy + y * dimx + x] = g_markColors[(int)(saturation / maxSaturation * 10)];
				}
				else {
					g_colors[z * dimx * dimy + y * dimx + x] = colorBase * (maxSaturation - saturation);
				}
			}
			//left point
			{
				int x = 0;

				int idx0 = start_down;				//down
				int idx1 = start_back;		//back
				int idx2 = start_left;		//left

				float saturation = (g_saturations[idx0] + g_saturations[idx0 + 1] + g_saturations[idx1] + g_saturations[idx2]) / 4;
				if (saturation < 0.0) saturation = 0.0;
				if (saturation > maxSaturation) saturation = maxSaturation;

				if (g_markColor && saturation > 0.0){
					g_colors[z * dimx * dimy + y * dimx + x] = g_markColors[(int)(saturation / maxSaturation * 10)];
				}
				else {
					g_colors[z * dimx * dimy + y * dimx + x] = colorBase * (maxSaturation - saturation);
				}
			}
			//right point
			{
				int x = dimx - 1;

				int idx0 = start_down + triangle_dx - 2;		//down
				int idx1 = start_back + triangle_dx - 2;		//back
				int idx2 = start_right;							//right

				float saturation = (g_saturations[idx0] + g_saturations[idx1] + g_saturations[idx1 + 1] + g_saturations[idx2] + g_saturations[idx2 + 1]) / 5;
				if (saturation < 0.0) saturation = 0.0;
				if (saturation > maxSaturation) saturation = maxSaturation;

				if (g_markColor && saturation > 0.0){
					g_colors[z * dimx * dimy + y * dimx + x] = g_markColors[(int)(saturation / maxSaturation * 10)];
				}
				else {
					g_colors[z * dimx * dimy + y * dimx + x] = colorBase * (maxSaturation - saturation);
				}
			}
		}
		//front edge
		{
			int y = dimy - 1;
			for (int x = 1; x < dimx - 1; x++){
				int idx0 = start_down + (x - 1) * 2 + (y - 1) * triangle_dx;	//down
				int idx1 = start_front + (x - 1) * 2;							//front

				float saturation = (g_saturations[idx0] + g_saturations[idx0 + 1] + g_saturations[idx0 + 3] + g_saturations[idx1] + g_saturations[idx1 + 2] + g_saturations[idx1 + 3]) / 6;
				if (saturation < 0.0) saturation = 0.0;
				if (saturation > maxSaturation) saturation = maxSaturation;

				if (g_markColor && saturation > 0.0){
					g_colors[z * dimx * dimy + y * dimx + x] = g_markColors[(int)(saturation / maxSaturation * 10)];
				}
				else {
					g_colors[z * dimx * dimy + y * dimx + x] = colorBase * (maxSaturation - saturation);
				}
			}

			//left point
			{
				int x = 0;

				int idx0 = start_down + (y - 1) * triangle_dx + 1;	//down
				int idx1 = start_front;								//front
				int idx2 = start_left + triangle_dy - 2;			//left

				float saturation = (g_saturations[idx0] + g_saturations[idx1] + g_saturations[idx1 + 1] + g_saturations[idx2] + g_saturations[idx2 + 1]) / 5;
				if (saturation < 0.0) saturation = 0.0;
				if (saturation > maxSaturation) saturation = maxSaturation;

				if (g_markColor && saturation > 0.0){
					g_colors[z * dimx * dimy + y * dimx + x] = g_markColors[(int)(saturation / maxSaturation * 10)];
				}
				else {
					g_colors[z * dimx * dimy + y * dimx + x] = colorBase * (maxSaturation - saturation);
				}
			}
			//right point
			{
				int x = dimx - 1;

				int idx0 = start_down + y * triangle_dx - 2;		//down
				int idx1 = start_front + triangle_dx - 2;			//front
				int idx2 = start_right + triangle_dy - 2;			//right

				float saturation = (g_saturations[idx0] + g_saturations[idx0 + 1] + g_saturations[idx1] + g_saturations[idx2]) / 4;
				if (saturation < 0.0) saturation = 0.0;
				if (saturation > maxSaturation) saturation = maxSaturation;

				if (g_markColor && saturation > 0.0){
					g_colors[z * dimx * dimy + y * dimx + x] = g_markColors[(int)(saturation / maxSaturation * 10)];
				}
				else {
					g_colors[z * dimx * dimy + y * dimx + x] = colorBase * (maxSaturation - saturation);
				}
			}
		}
		//left edge
		{
			int x = 0;
			for (int y = 1; y < dimy - 1; y++){
				int idx0 = start_down + y * triangle_dx;				//down
				int idx1 = start_left + y * 2;							//left

				float saturation = (g_saturations[idx0] + g_saturations[idx0 - triangle_dx + 1] + g_saturations[idx0 + 1] + g_saturations[idx1] + g_saturations[idx1 + 1] + g_saturations[idx1 + 2]) / 6;
				if (saturation < 0.0) saturation = 0.0;
				if (saturation > maxSaturation) saturation = maxSaturation;

				if (g_markColor && saturation > 0.0){
					g_colors[z * dimx * dimy + y * dimx + x] = g_markColors[(int)(saturation / maxSaturation * 10)];
				}
				else {
					g_colors[z * dimx * dimy + y * dimx + x] = colorBase * (maxSaturation - saturation);
				}
			}
		}
		//right edge
		{
			int x = dimx - 1;
			for (int y = 1; y < dimy - 1; y++){
				int idx0 = start_down + y * triangle_dx - 2;				//down
				int idx1 = start_right + y * 2;					//right

				float saturation = (g_saturations[idx0] + g_saturations[idx0 + 1] + g_saturations[idx0 + triangle_dx] + g_saturations[idx1] + g_saturations[idx1 + 2] + g_saturations[idx1 + 3]) / 6;
				if (saturation < 0.0) saturation = 0.0;
				if (saturation > maxSaturation) saturation = maxSaturation;

				if (g_markColor && saturation > 0.0){
					g_colors[z * dimx * dimy + y * dimx + x] = g_markColors[(int)(saturation / maxSaturation * 10)];
				}
				else {
					g_colors[z * dimx * dimy + y * dimx + x] = colorBase * (maxSaturation - saturation);
				}
			}
		}
	}
	//up
	{
		int z = dimz - 1;
		for (int y = 1; y < dimy - 1; y++){
			for (int x = 1; x < dimx - 1; x++){
				int idx0 = start_up + x * 2 + (y - 1) * triangle_dx;		//up

				float saturation = (g_saturations[idx0] + g_saturations[idx0 - 1] + g_saturations[idx0 + 1] + g_saturations[idx0 + triangle_dx] + g_saturations[idx0 + triangle_dx - 2] + g_saturations[idx0 + triangle_dx - 1]) / 6;
				if (saturation < 0.0) saturation = 0.0;
				if (saturation > maxSaturation) saturation = maxSaturation;

				if (g_markColor && saturation > 0.0){
					g_colors[z * dimx * dimy + y * dimx + x] = g_markColors[(int)(saturation / maxSaturation * 10)];
				}
				else {
					g_colors[z * dimx * dimy + y * dimx + x] = colorBase * (maxSaturation - saturation);
				}
			}
		}
		//back edge
		{
			int y = 0;
			for (int x = 1; x < dimx - 1; x++){

				int idx0 = start_up + x * 2;								//up
				int idx1 = start_back + (z - 1) * triangle_dx + 2;			//back

				float saturation = (g_saturations[idx0] + g_saturations[idx0 - 1] + g_saturations[idx0 - 2] + g_saturations[idx1] + g_saturations[idx1 - 1] + g_saturations[idx1 + 1]) / 6;
				if (saturation < 0.0) saturation = 0.0;
				if (saturation > maxSaturation) saturation = maxSaturation;

				if (g_markColor && saturation > 0.0){
					g_colors[z * dimx * dimy + y * dimx + x] = g_markColors[(int)(saturation / maxSaturation * 10)];
				}
				else {
					g_colors[z * dimx * dimy + y * dimx + x] = colorBase * (maxSaturation - saturation);
				}
			}

			//left point
			{
				int x = 0;

				int idx0 = start_up;								//up
				int idx1 = start_back + (z - 1) * triangle_dx;		//back
				int idx2 = start_left + (z - 1) * triangle_dy;		//left

				float saturation = (g_saturations[idx0] + g_saturations[idx1] + g_saturations[idx1 + 1] + g_saturations[idx2] + g_saturations[idx2 + 1]) / 5;
				if (saturation < 0.0) saturation = 0.0;
				if (saturation > maxSaturation) saturation = maxSaturation;

				if (g_markColor && saturation > 0.0){
					g_colors[z * dimx * dimy + y * dimx + x] = g_markColors[(int)(saturation / maxSaturation * 10)];
				}
				else {
					g_colors[z * dimx * dimy + y * dimx + x] = colorBase * (maxSaturation - saturation);
				}
			}
			//right point
			{
				int x = dimx - 1;

				int idx0 = start_up + triangle_dx - 2;					//up
				int idx1 = start_back + z * triangle_dx - 1;			//back
				int idx2 = start_right + (z - 1) * triangle_dy + 1;		//right

				float saturation = (g_saturations[idx0] + g_saturations[idx0 + 1] + g_saturations[idx1] + g_saturations[idx2]) / 4;
				if (saturation < 0.0) saturation = 0.0;
				if (saturation > maxSaturation) saturation = maxSaturation;

				if (g_markColor && saturation > 0.0){
					g_colors[z * dimx * dimy + y * dimx + x] = g_markColors[(int)(saturation / maxSaturation * 10)];
				}
				else {
					g_colors[z * dimx * dimy + y * dimx + x] = colorBase * (maxSaturation - saturation);
				}
			}
		}
		//front edge
		{
			int y = dimy - 1;
			for (int x = 1; x < dimx - 1; x++){
				int idx0 = start_up + x * 2 + (y - 1) * triangle_dx;			//up
				int idx1 = start_front + (z - 1) * triangle_dx + (x - 1) * 2;	//front

				float saturation = (g_saturations[idx0] + g_saturations[idx0 - 1] + g_saturations[idx0 + 1] + g_saturations[idx1] + g_saturations[idx1 + 1] + g_saturations[idx1 + 3]) / 6;
				if (saturation < 0.0) saturation = 0.0;
				if (saturation > maxSaturation) saturation = maxSaturation;

				if (g_markColor && saturation > 0.0){
					g_colors[z * dimx * dimy + y * dimx + x] = g_markColors[(int)(saturation / maxSaturation * 10)];
				}
				else {
					g_colors[z * dimx * dimy + y * dimx + x] = colorBase * (maxSaturation - saturation);
				}
			}

			//left point
			{
				int x = 0;

				int idx0 = start_up + (y - 1) * triangle_dx;		//up
				int idx1 = start_front + (z - 1) * triangle_dx + 1;	//front
				int idx2 = start_left + z * triangle_dy - 1;		//left

				float saturation = (g_saturations[idx0] + g_saturations[idx0 + 1] + g_saturations[idx1] + g_saturations[idx2]) / 4;
				if (saturation < 0.0) saturation = 0.0;
				if (saturation > maxSaturation) saturation = maxSaturation;

				if (g_markColor && saturation > 0.0){
					g_colors[z * dimx * dimy + y * dimx + x] = g_markColors[(int)(saturation / maxSaturation * 10)];
				}
				else {
					g_colors[z * dimx * dimy + y * dimx + x] = colorBase * (maxSaturation - saturation);
				}
			}
			//right point
			{
				int x = dimx - 1;

				int idx0 = start_up + y * triangle_dx - 1;		//up
				int idx1 = start_front + z * triangle_dx - 2;	//front
				int idx2 = start_right + z * triangle_dy - 2;			//right

				float saturation = (g_saturations[idx0] + g_saturations[idx1] + g_saturations[idx1 + 1] + g_saturations[idx2] + g_saturations[idx2 + 1]) / 5;
				if (saturation < 0.0) saturation = 0.0;
				if (saturation > maxSaturation) saturation = maxSaturation;

				if (g_markColor && saturation > 0.0){
					g_colors[z * dimx * dimy + y * dimx + x] = g_markColors[(int)(saturation / maxSaturation * 10)];
				}
				else {
					g_colors[z * dimx * dimy + y * dimx + x] = colorBase * (maxSaturation - saturation);
				}
			}
		}
		//left edge
		{
			int x = 0;
			for (int y = 1; y < dimy - 1; y++){
				int idx0 = start_up + (y - 1) * triangle_dx;			//up
				int idx1 = start_left + (z - 1) * triangle_dy + y * 2;	//left

				float saturation = (g_saturations[idx0] + g_saturations[idx0 + 1] + g_saturations[idx0 + triangle_dx] + g_saturations[idx1] + g_saturations[idx1 - 1] + g_saturations[idx1 + 1]) / 6;
				if (saturation < 0.0) saturation = 0.0;
				if (saturation > maxSaturation) saturation = maxSaturation;

				if (g_markColor && saturation > 0.0){
					g_colors[z * dimx * dimy + y * dimx + x] = g_markColors[(int)(saturation / maxSaturation * 10)];
				}
				else {
					g_colors[z * dimx * dimy + y * dimx + x] = colorBase * (maxSaturation - saturation);
				}
			}
		}
		//right edge
		{
			int x = dimx - 1;
			for (int y = 1; y < dimy - 1; y++){
				int idx0 = start_up + y * triangle_dx - 1;							//up
				int idx1 = start_right + (z - 1) * triangle_dy + y * 2;				//right

				float saturation = (g_saturations[idx0] + g_saturations[idx0 + triangle_dx - 1] + g_saturations[idx0 + triangle_dx] + g_saturations[idx1] + g_saturations[idx1 + 1] + g_saturations[idx1 + 3]) / 6;
				if (saturation < 0.0) saturation = 0.0;
				if (saturation > maxSaturation) saturation = maxSaturation;

				if (g_markColor && saturation > 0.0){
					g_colors[z * dimx * dimy + y * dimx + x] = g_markColors[(int)(saturation / maxSaturation * 10)];
				}
				else {
					g_colors[z * dimx * dimy + y * dimx + x] = colorBase * (maxSaturation - saturation);
				}
			}

		}
	}
	//back
	{
		int y = 0;
		for (int z = 1; z < dimz - 1; z++){
			for (int x = 1; x < dimx - 1; x++){
				int idx0 = start_back + x * 2 + (z - 1) * triangle_dx;		//back

				float saturation = (g_saturations[idx0] + g_saturations[idx0 - 1] + g_saturations[idx0 + 1] + g_saturations[idx0 + triangle_dx] + g_saturations[idx0 + triangle_dx - 2] + g_saturations[idx0 + triangle_dx - 1]) / 6;
				if (saturation < 0.0) saturation = 0.0;
				if (saturation > maxSaturation) saturation = maxSaturation;

				if (g_markColor && saturation > 0.0){
					g_colors[z * dimx * dimy + y * dimx + x] = g_markColors[(int)(saturation / maxSaturation * 10)];
				}
				else {
					g_colors[z * dimx * dimy + y * dimx + x] = colorBase * (maxSaturation - saturation);
				}

			}
		}
		//left edge
		{
			int x = 0;
			for (int z = 1; z < dimz - 1; z++){
				int idx0 = start_back + (z - 1) * triangle_dx;				//back
				int idx1 = start_left + (z - 1) * triangle_dy;				//left

				float saturation = (g_saturations[idx0] + g_saturations[idx0 + 1] + g_saturations[idx0 + triangle_dx] + g_saturations[idx1] + g_saturations[idx1 + 1] + g_saturations[idx1 + triangle_dy]) / 6;
				if (saturation < 0.0) saturation = 0.0;
				if (saturation > maxSaturation) saturation = maxSaturation;

				if (g_markColor && saturation > 0.0){
					g_colors[z * dimx * dimy + y * dimx + x] = g_markColors[(int)(saturation / maxSaturation * 10)];
				}
				else {
					g_colors[z * dimx * dimy + y * dimx + x] = colorBase * (maxSaturation - saturation);
				}
			}
		}
		//right edge
		{
			int x = dimx - 1;
			for (int z = 1; z < dimz - 1; z++){
				int idx0 = start_back + z * triangle_dx - 1;				//back
				int idx1 = start_right + z * triangle_dy + 1;				//right

				float saturation = (g_saturations[idx0] + g_saturations[idx0 + triangle_dx - 1] + g_saturations[idx0 + triangle_dx] + g_saturations[idx1] + g_saturations[idx1 - 1] + g_saturations[idx1 - triangle_dy]) / 6;
				if (saturation < 0.0) saturation = 0.0;
				if (saturation > maxSaturation) saturation = maxSaturation;

				if (g_markColor && saturation > 0.0){
					g_colors[z * dimx * dimy + y * dimx + x] = g_markColors[(int)(saturation / maxSaturation * 10)];
				}
				else {
					g_colors[z * dimx * dimy + y * dimx + x] = colorBase * (maxSaturation - saturation);
				}
			}
		}
	}
	//front
	{
		int y = dimy - 1;
		for (int z = 1; z < dimz - 1; z++){
			for (int x = 1; x < dimx - 1; x++){
				int idx0 = start_front + (x - 1) * 2 + (z - 1) * triangle_dx;		//front

				float saturation = (g_saturations[idx0] + g_saturations[idx0 + 1] + g_saturations[idx0 + 3] + g_saturations[idx0 + triangle_dx] + g_saturations[idx0 + triangle_dx + 2] + g_saturations[idx0 + triangle_dx + 3]) / 6;
				if (saturation < 0.0) saturation = 0.0;
				if (saturation > maxSaturation) saturation = maxSaturation;

				if (g_markColor && saturation > 0.0){
					g_colors[z * dimx * dimy + y * dimx + x] = g_markColors[(int)(saturation / maxSaturation * 10)];
				}
				else {
					g_colors[z * dimx * dimy + y * dimx + x] = colorBase * (maxSaturation - saturation);
				}

			}
		}
		//left edge
		{
			int x = 0;
			for (int z = 1; z < dimz - 1; z++){
				int idx0 = start_front + z * triangle_dx;				//front
				int idx1 = start_left + z * triangle_dy - 1;			//left

				float saturation = (g_saturations[idx0] + g_saturations[idx0 + 1] + g_saturations[idx0 - triangle_dx + 1] + g_saturations[idx1] + g_saturations[idx1 + 1] + g_saturations[idx1 - triangle_dy + 1]) / 6;
				if (saturation < 0.0) saturation = 0.0;
				if (saturation > maxSaturation) saturation = maxSaturation;

				if (g_markColor && saturation > 0.0){
					g_colors[z * dimx * dimy + y * dimx + x] = g_markColors[(int)(saturation / maxSaturation * 10)];
				}
				else {
					g_colors[z * dimx * dimy + y * dimx + x] = colorBase * (maxSaturation - saturation);
				}
			}
		}
		//right edge
		{
			int x = dimx - 1;
			for (int z = 1; z < dimz - 1; z++){
				int idx0 = start_front + z * triangle_dx - 1;			//front
				int idx1 = start_right + z * triangle_dy - 1;			//right

				float saturation = (g_saturations[idx0] + g_saturations[idx0 - 1] + g_saturations[idx0 + triangle_dx - 1] + g_saturations[idx1] + g_saturations[idx1 - 1] + g_saturations[idx1 + triangle_dy - 1]) / 6;
				if (saturation < 0.0) saturation = 0.0;
				if (saturation > maxSaturation) saturation = maxSaturation;

				if (g_markColor && saturation > 0.0){
					g_colors[z * dimx * dimy + y * dimx + x] = g_markColors[(int)(saturation / maxSaturation * 10)];
				}
				else {
					g_colors[z * dimx * dimy + y * dimx + x] = colorBase * (maxSaturation - saturation);
				}
			}

		}
	}
	//left
	{
		int x = 0;
		for (int z = 1; z < dimz - 1; z++){
			for (int y = 1; y < dimy - 1; y++){
				int idx0 = start_left + y * 2 + (z - 1) * triangle_dy;		//left

				float saturation = (g_saturations[idx0] + g_saturations[idx0 - 1] + g_saturations[idx0 + 1] + g_saturations[idx0 + triangle_dx] + g_saturations[idx0 + triangle_dx - 2] + g_saturations[idx0 + triangle_dx - 1]) / 6;
				if (saturation < 0.0) saturation = 0.0;
				if (saturation > maxSaturation) saturation = maxSaturation;

				if (g_markColor && saturation > 0.0){
					g_colors[z * dimx * dimy + y * dimx + x] = g_markColors[(int)(saturation / maxSaturation * 10)];
				}
				else {
					g_colors[z * dimx * dimy + y * dimx + x] = colorBase * (maxSaturation - saturation);
				}

			}
		}
	}
	//right
	{
		int x = dimx - 1;
		for (int z = 1; z < dimz - 1; z++){
			for (int y = 1; y < dimy - 1; y++){
				int idx0 = start_right + (y - 1) * 2 + (z - 1) * triangle_dy;		//right

				float saturation = (g_saturations[idx0] + g_saturations[idx0 + 1] + g_saturations[idx0 + 3] + g_saturations[idx0 + triangle_dx] + g_saturations[idx0 + triangle_dx + 2] + g_saturations[idx0 + triangle_dx + 3]) / 6;
				if (saturation < 0.0) saturation = 0.0;
				if (saturation > maxSaturation) saturation = maxSaturation;

				if (g_markColor && saturation > 0.0){
					g_colors[z * dimx * dimy + y * dimx + x] = g_markColors[(int)(saturation / maxSaturation * 10)];
				}
				else {
					g_colors[z * dimx * dimy + y * dimx + x] = colorBase * (maxSaturation - saturation);
				}

			}
		}

	}



}
void CalculateClothColors(){
	if (sceneNum == 2){
		CalculateColors();
		return;
	}

	if (is_point){
		CalculateClothColorsPoint();
		return;
	}
	if (is_cube){
		CalculateClothColorsCube();
		return;
	}
	if (is_hollow){
		CalculateClothColorsHollow();
		return;
	}

	int dimx = g_dx;
	int dimy = g_dy;
	int dimz = g_dz;
	int j = 0;
	int dx = 2;
	int dy = (dimx - 1) * 2;
	int dz = (dimx - 1) * (dimy - 1) * 2;

	float maxSaturation = g_maxSaturation * g_kMaxAbsorption;

	Vec4 color = g_clothColor;
	Vec4 colorBase = color / maxSaturation;


	for (int z = 0; z < dimz; z++){
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

				if (saturation < 0.0) saturation = 0.0;
				if (saturation > maxSaturation) saturation = maxSaturation;

				if (g_markColor && saturation > 0.0){
					g_colors[z * dimx * dimy + y * dimx + x] = g_markColors[(int)(saturation / maxSaturation * 10)];
				}
				else {
					g_colors[z * dimx * dimy + y * dimx + x] = colorBase * (maxSaturation - saturation);
				}
				//printf("%d: %f, %f, %f\n", y * dimx + x, g_colors[y * dimx + x].x, g_colors[y * dimx + x].y, g_colors[y * dimx + x].z);
			}
	}
}


/*absorb*/

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

void UpdatingSaturationsCommon(int idx){

	int num = g_pointTriangleNums[idx];

	Vec4 tmp1 = g_pointTriangles[idx * 2];
	Vec4 tmp2 = g_pointTriangles[idx * 2 + 1];

	float tmp = g_mDrip / num;

	switch (num)
	{
	case 8:
		g_saturations[int(tmp2.z)] += tmp;
	case 7:
		g_saturations[int(tmp2.y)] += tmp;
	case 6:
		g_saturations[int(tmp2.x)] += tmp;
	case 5:
		g_saturations[int(tmp2.w)] += tmp;
	case 4:
		g_saturations[int(tmp1.z)] += tmp;
	case 3:
		g_saturations[int(tmp1.y)] += tmp;
	case 2:
		g_saturations[int(tmp1.x)] += tmp;
	case 1:
		g_saturations[int(tmp1.w)] += tmp;
	default:
		break;
	}

}
void UpdatingSaturationsPoint(int idx){
	g_saturations[idx] += g_mDrip;
}
void UpdatingSaturationsCube(int idx){
	int dimx = g_dx;
	int dimy = g_dy;
	int dimz = g_dz;
	int dx = 1;
	int dy = dimy - 1;
	int dz = (dimx - 1) * (dimy - 1);

	int z = idx / (dimx * dimy);
	int idxz = idx % (dimx * dimy);
	int x = idxz % dimx;
	int y = idxz / dimx;

	int sum = 1;
	if (x < dimx - 1 && x > 0)
		sum *= 2;
	if (y < dimy - 1 && y > 0)
		sum *= 2;
	if (z < dimz - 1 && z > 0)
		sum *= 2;

	float tmp = g_mDrip / sum;

	int xx, yy, zz;

	//up right front
	xx = x; yy = y; zz = z;
	if (xx >= 0 && xx < dimx - 1 && yy >= 0 && yy < dimy - 1 && zz >= 0 && zz < dimz - 1){
		g_saturations[xx + yy * dy + zz * dz] += tmp;
	}
	//up left front
	xx = x - 1; yy = y; zz = z;
	if (xx >= 0 && xx < dimx - 1 && yy >= 0 && yy < dimy - 1 && zz >= 0 && zz < dimz - 1){
		g_saturations[xx + yy * dy + zz * dz] += tmp;
	}
	//up right back
	xx = x; yy = y - 1; zz = z;
	if (xx >= 0 && xx < dimx - 1 && yy >= 0 && yy < dimy - 1 && zz >= 0 && zz < dimz - 1){
		g_saturations[xx + yy * dy + zz * dz] += tmp;
	}
	//up left back
	xx = x - 1; yy = y - 1; zz = z;
	if (xx >= 0 && xx < dimx - 1 && yy >= 0 && yy < dimy - 1 && zz >= 0 && zz < dimz - 1){
		g_saturations[xx + yy * dy + zz * dz] += tmp;
	}
	//down right front
	xx = x; yy = y; zz = z - 1;
	if (xx >= 0 && xx < dimx - 1 && yy >= 0 && yy < dimy - 1 && zz >= 0 && zz < dimz - 1){
		g_saturations[xx + yy * dy + zz * dz] += tmp;
	}
	//down left front
	xx = x - 1; yy = y; zz = z - 1;
	if (xx >= 0 && xx < dimx - 1 && yy >= 0 && yy < dimy - 1 && zz >= 0 && zz < dimz - 1){
		g_saturations[xx + yy * dy + zz * dz] += tmp;
	}
	//down right back
	xx = x; yy = y - 1; zz = z - 1;
	if (xx >= 0 && xx < dimx - 1 && yy >= 0 && yy < dimy - 1 && zz >= 0 && zz < dimz - 1){
		g_saturations[xx + yy * dy + zz * dz] += tmp;
	}
	//down left back
	xx = x - 1; yy = y - 1; zz = z - 1;
	if (xx >= 0 && xx < dimx - 1 && yy >= 0 && yy < dimy - 1 && zz >= 0 && zz < dimz - 1){
		g_saturations[xx + yy * dy + zz * dz] += tmp;
	}


}
void UpdatingSaturationsHollow(int idx){
	int dimx = g_dx;
	int dimy = g_dy;
	int dimz = g_dz;
	int dx = 2;
	int dy = (dimy - 1) * 2;
	int dz = (dimx - 1) * (dimy - 1) * 2;

	int z = idx / (dimx * dimy);
	int idxz = idx % (dimx * dimy);
	int x = idxz % dimx;
	int y = idxz / dimx;

	int triangle_z = (dimx - 1) * (dimy - 1) * 2;	//up & down
	int triangle_y = (dimx - 1) * (dimz - 1) * 2;	//back & front
	int triangle_x = (dimy - 1) * (dimz - 1) * 2;	//left & right

	int start_down = 0;
	int start_up = start_down + triangle_z;
	int start_back = start_up + triangle_z;
	int start_front = start_back + triangle_y;
	int start_left = start_front + triangle_y;
	int start_right = start_left + triangle_x;

	int triangle_dx = (dimx - 1) * 2;
	int triangle_dy = (dimy - 1) * 2;

	//down face
	if (z == 0){
		//back edge
		if (y == 0){
			//left point
			if (x == 0){
				float tmp = g_mDrip / 4;
				int idx0 = start_down;		//down
				int idx1 = start_back;		//back
				int idx2 = start_left;		//left

				g_saturations[idx0] += tmp; g_saturations[idx0 + 1] += tmp;
				g_saturations[idx1] += tmp; g_saturations[idx2] += tmp;

			}
			//right point
			else if (x == dimx - 1){
				float tmp = g_mDrip / 5;
				int idx0 = start_down + triangle_dx - 2;		//down
				int idx1 = start_back + triangle_dx - 2;		//back
				int idx2 = start_right;							//right

				g_saturations[idx0] += tmp; g_saturations[idx1] += tmp;
				g_saturations[idx1 + 1] += tmp; g_saturations[idx2] += tmp;
				g_saturations[idx2 + 1] += tmp;

			}
			else {
				float tmp = g_mDrip / 6;

				int idx0 = start_down + (x - 1) * 2;		//down
				int idx1 = start_back + x * 2 - 1;			//back

				g_saturations[idx0] += tmp; g_saturations[idx0 + 2] += tmp;
				g_saturations[idx0 + 3] += tmp; g_saturations[idx1] += tmp;
				g_saturations[idx1 - 1] += tmp; g_saturations[idx1 + 1] += tmp;
			}
		}
		//front edge
		else if (y == dimy - 1){
			//left point
			if (x == 0){
				float tmp = g_mDrip / 5;
				int idx0 = start_down + (y - 1) * triangle_dx + 1;	//down
				int idx1 = start_front;								//front
				int idx2 = start_left + triangle_dy - 2;			//left

				g_saturations[idx0] += tmp; g_saturations[idx1] += tmp;
				g_saturations[idx1 + 1] += tmp; g_saturations[idx2] += tmp;
				g_saturations[idx2 + 1] += tmp;;

			}
			//right point
			else if (x == dimx - 1){
				float tmp = g_mDrip / 4;
				int idx0 = start_down + y * triangle_dx - 2;		//down
				int idx1 = start_front + triangle_dx - 2;			//front
				int idx2 = start_right + triangle_dy - 2;			//right

				g_saturations[idx0] += tmp; g_saturations[idx0 + 1] += tmp;
				g_saturations[idx1] += tmp; g_saturations[idx2] += tmp;

			}
			else {
				float tmp = g_mDrip / 6;
				int idx0 = start_down + (x - 1) * 2 + (y - 1) * triangle_dx;	//down
				int idx1 = start_front + (x - 1) * 2;							//front

				g_saturations[idx0] += tmp; g_saturations[idx0 + 1] += tmp;
				g_saturations[idx0 + 3] += tmp; g_saturations[idx1] += tmp;
				g_saturations[idx1 + 2] += tmp; g_saturations[idx1 + 3] += tmp;

			}
		}
		//left edge
		else if (x == 0){
			float tmp = g_mDrip / 6;
			int idx0 = start_down + y * triangle_dx;				//down
			int idx1 = start_left + y * 2;							//left

			g_saturations[idx0] += tmp; g_saturations[idx0 - triangle_dx + 1] += tmp;
			g_saturations[idx0 + 1] += tmp; g_saturations[idx1] += tmp;
			g_saturations[idx1 + 1] += tmp; g_saturations[idx1 + 2] += tmp;

		}
		//right edge
		else if (x == dimx - 1){
			float tmp = g_mDrip / 6;
			int idx0 = start_down + y * triangle_dx - 2;				//down
			int idx1 = start_right + y * 2;					//right

			g_saturations[idx0] += tmp; g_saturations[idx0 + 1] += tmp;
			g_saturations[idx0 + triangle_dx] += tmp; g_saturations[idx1] += tmp;
			g_saturations[idx1 + 2] += tmp; g_saturations[idx1 + 3] += tmp;

		}
		else {
			float tmp = g_mDrip / 6;
			int idx0 = start_down + (x - 1) * 2 + (y - 1) * triangle_dx;		//down

			g_saturations[idx0] += tmp; g_saturations[idx0 + 1] += tmp;
			g_saturations[idx0 + 3] += tmp; g_saturations[idx0 + triangle_dx] += tmp;
			g_saturations[idx0 + triangle_dx + 2] += tmp; g_saturations[idx0 + triangle_dx + 3] += tmp;

		}
	}
	//up face
	else if (z == dimz - 1){
		//back edge
		if (y == 0){
			//left point
			if (x == 0){
				float tmp = g_mDrip / 5;
				int idx0 = start_up;								//up
				int idx1 = start_back + (z - 1) * triangle_dx;		//back
				int idx2 = start_left + (z - 1) * triangle_dy;		//left

				g_saturations[idx0] += tmp; g_saturations[idx1] += tmp;
				g_saturations[idx1 + 1] += tmp; g_saturations[idx2] += tmp;
				g_saturations[idx2 + 1] += tmp;

			}
			//right point
			else if (x == dimx - 1){
				float tmp = g_mDrip / 4;	
				int idx0 = start_up + triangle_dx - 2;					//up
				int idx1 = start_back + z * triangle_dx - 1;			//back
				int idx2 = start_right + (z - 1) * triangle_dy + 1;		//right

				g_saturations[idx0] += tmp; g_saturations[idx0 + 1] += tmp;
				g_saturations[idx1] += tmp; g_saturations[idx2] += tmp;
			}
			else {
				float tmp = g_mDrip / 6;
				int idx0 = start_up + x * 2;								//up
				int idx1 = start_back + (z - 1) * triangle_dx + 2;			//back

				g_saturations[idx0] += tmp; g_saturations[idx0 - 1] += tmp;
				g_saturations[idx0 - 2] += tmp; g_saturations[idx1] += tmp;
				g_saturations[idx1 - 1] += tmp; g_saturations[idx1 + 1] += tmp;

			}
		}
		//front edge
		else if (y == dimy - 1){
			//left point
			if (x == 0){
				float tmp = g_mDrip / 4;
				int idx0 = start_up + (y - 1) * triangle_dx;		//up
				int idx1 = start_front + (z - 1) * triangle_dx + 1;	//front
				int idx2 = start_left + z * triangle_dy - 1;		//left

				g_saturations[idx0] += tmp; g_saturations[idx0 + 1] += tmp;
				g_saturations[idx1] += tmp; g_saturations[idx2] += tmp;

			}
			//right point
			else if (x == dimx - 1){
				float tmp = g_mDrip / 5;
				int idx0 = start_up + y * triangle_dx - 1;		//up
				int idx1 = start_front + z * triangle_dx - 2;	//front
				int idx2 = start_right + z * triangle_dy - 2;			//right

				g_saturations[idx0] += tmp; g_saturations[idx1] += tmp;
				g_saturations[idx1 + 1] += tmp; g_saturations[idx2] += tmp;
				g_saturations[idx2 + 1] += tmp;

			}
			else {
				float tmp = g_mDrip / 4;
				int idx0 = start_up + x * 2 + (y - 1) * triangle_dx;			//up
				int idx1 = start_front + (z - 1) * triangle_dx + (x - 1) * 2;	//front

				g_saturations[idx0] += tmp; g_saturations[idx0 - 1] += tmp;
				g_saturations[idx0 + 1] += tmp; g_saturations[idx1] += tmp;
				g_saturations[idx1 + 1] += tmp; g_saturations[idx1 + 3] += tmp;

			}
		}
		//left edge
		else if (x == 0){
			float tmp = g_mDrip / 6;
			int idx0 = start_up + (y - 1) * triangle_dx;			//up
			int idx1 = start_left + (z - 1) * triangle_dy + y * 2;	//left

			g_saturations[idx0] += tmp; g_saturations[idx0 + 1] += tmp;
			g_saturations[idx0 + triangle_dx] += tmp; g_saturations[idx1] += tmp;
			g_saturations[idx1 - 1] += tmp; g_saturations[idx1 + 1] += tmp;

		}
		//right edge
		else if (x == dimx - 1){
			float tmp = g_mDrip / 6;
			int idx0 = start_up + y * triangle_dx - 1;							//up
			int idx1 = start_right + (z - 1) * triangle_dy + y * 2;				//right

			g_saturations[idx0] += tmp; g_saturations[idx0 + triangle_dx - 1] += tmp;
			g_saturations[idx0 + triangle_dx] += tmp; g_saturations[idx1] += tmp;
			g_saturations[idx1 + 1] += tmp; g_saturations[idx1 + 3] += tmp;

		}
		else {
			float tmp = g_mDrip / 6;
			int idx0 = start_up + x * 2 + (y - 1) * triangle_dx;		//up

			g_saturations[idx0] += tmp; g_saturations[idx0 - 1] += tmp; 
			g_saturations[idx0 + 1] += tmp; g_saturations[idx0 + triangle_dx] += tmp;
			g_saturations[idx0 + triangle_dx - 2] += tmp; g_saturations[idx0 + triangle_dx - 1] += tmp;

		}
	}
	//back face
	else if (y == 0){
		//left edge
		if (x == 0){
			float tmp = g_mDrip / 6;
			int idx0 = start_back + (z - 1) * triangle_dx;				//back
			int idx1 = start_left + (z - 1) * triangle_dy;				//left

			g_saturations[idx0] += tmp; g_saturations[idx0 + 1] += tmp;
			g_saturations[idx0 + triangle_dx] += tmp; g_saturations[idx1] += tmp;
			g_saturations[idx1 + 1] += tmp; g_saturations[idx1 + triangle_dy] += tmp;

		}
		//right edge
		if (x == dimx - 1){
			float tmp = g_mDrip / 6;
			int idx0 = start_back + z * triangle_dx - 1;				//back
			int idx1 = start_right + z * triangle_dy + 1;				//right

			g_saturations[idx0] += tmp; g_saturations[idx0 + triangle_dx - 1] += tmp;
			g_saturations[idx0 + triangle_dx] += tmp; g_saturations[idx1] += tmp;
			g_saturations[idx1 - 1] += tmp; g_saturations[idx1 - triangle_dy] += tmp;

		}
		else {
			float tmp = g_mDrip / 6;
			int idx0 = start_back + x * 2 + (z - 1) * triangle_dx;		//back

			g_saturations[idx0] += tmp; g_saturations[idx0 - 1] += tmp;
			g_saturations[idx0 + 1] += tmp; g_saturations[idx0 + triangle_dx] += tmp;
			g_saturations[idx0 + triangle_dx - 2] += tmp; g_saturations[idx0 + triangle_dx - 1] += tmp;

		}
	}
	//front face
	else if (y == dimy - 1){
		//left edge
		if (x == 0){
			float tmp = g_mDrip / 6;
			int idx0 = start_front + z * triangle_dx;				//front
			int idx1 = start_left + z * triangle_dy - 1;			//left

			g_saturations[idx0] += tmp; g_saturations[idx0 + 1] += tmp;
			g_saturations[idx0 - triangle_dx + 1] += tmp; g_saturations[idx1] += tmp;
			g_saturations[idx1 + 1] += tmp; g_saturations[idx1 - triangle_dy + 1] += tmp;

		}
		//right edge
		if (x == dimx - 1){
			float tmp = g_mDrip / 6;
			int idx0 = start_front + z * triangle_dx - 1;			//front
			int idx1 = start_right + z * triangle_dy - 1;			//right

			g_saturations[idx0] += tmp; g_saturations[idx0 - 1] += tmp;
			g_saturations[idx0 + triangle_dx - 1] += tmp; g_saturations[idx1] += tmp;
			g_saturations[idx1 - 1] += tmp; g_saturations[idx1 + triangle_dy - 1] += tmp;

		}
		else {
			float tmp = g_mDrip / 6;
			int idx0 = start_front + (x - 1) * 2 + (z - 1) * triangle_dx;		//front

			g_saturations[idx0] += tmp; g_saturations[idx0 + 1] += tmp;
			g_saturations[idx0 + 3] += tmp; g_saturations[idx0 + triangle_dx] += tmp;
			g_saturations[idx0 + triangle_dx + 2] += tmp; g_saturations[idx0 + triangle_dx + 3] += tmp;

		}

	}
	//left face
	else if (x == 0){
		float tmp = g_mDrip / 6;
		int idx0 = start_left + y * 2 + (z - 1) * triangle_dy;		//left

		g_saturations[idx0] += tmp; g_saturations[idx0 - 1] += tmp;
		g_saturations[idx0 + 1] += tmp; g_saturations[idx0 + triangle_dx] += tmp;
		g_saturations[idx0 + triangle_dx - 2] += tmp; g_saturations[idx0 + triangle_dx - 1] += tmp;

	}
	//right face
	else if (x == dimx - 1){
		float tmp = g_mDrip / 6;
		int idx0 = start_right + (y - 1) * 2 + (z - 1) * triangle_dy;		//right

		g_saturations[idx0] += tmp; g_saturations[idx0 + 1] += tmp;
		g_saturations[idx0 + 3] += tmp; g_saturations[idx0 + triangle_dx] += tmp;
		g_saturations[idx0 + triangle_dx + 2] += tmp; g_saturations[idx0 + triangle_dx + 3] += tmp;

	}

}
void UpdatingSaturations(int idx){

	if (sceneNum == 2){
		UpdatingSaturationsCommon(idx);
		return;
	}

	if (is_point){
		UpdatingSaturationsPoint(idx);
		return;
	}
	if (is_cube){
		UpdatingSaturationsCube(idx);
		return;
	}
	if (is_hollow){
		UpdatingSaturationsHollow(idx);
		return;
	}

	int dimx = g_dx;
	int dimy = g_dy;
	int dimz = g_dz;
	int dx = 2;
	int dy = (dimy - 1) * 2;
	int dz = (dimx - 1) * (dimy - 1) * 2;

	int z = idx / (dimx * dimy);
	int idxz = idx % (dimx * dimy);
	int x = idxz % dimx;
	int y = idxz / dimx;
	int j = x * dx + y * dy + z * dz;

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
			//proportion
			int tmp = rand() % 10;
			if (tmp >= 10 * g_kAbsorption){
				i++;
				continue;
			}

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

	//int tmp = 0;
	//for (int i = 0; i < g_numTriangles; i++){
	//	if (g_saturations[i] > g_maxSaturation * g_kAbsorption){
	//		if (tmp % 4 == 0)
	//			printf("\n");
	//		tmp++;
	//		printf("%d: %f\t", i, g_saturations[i]);
	//	}
	//}
	//printf("\n%d\n", tmp);

	flexSetActive(g_flex, &g_activeIndices[0], activeCount, eFlexMemoryHost);

}

// complex
bool CollideComplex(int x, int y){
	Vec4 posX = g_positions[x];
	Vec4 posY = g_mesh->m_positions[y];

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
void UpdatingSaturationsComplex(int idx){
	int num = g_pointTriangleNums[idx];

	Vec4 tmp1 = g_pointTriangles[idx * 2];
	Vec4 tmp2 = g_pointTriangles[idx * 2 + 1];

	float tmp = g_mDrip / num;

	switch (num)
	{
	case 8:
		g_saturations[int(tmp2.z)] += tmp;
	case 7:
		g_saturations[int(tmp2.y)] += tmp;
	case 6:
		g_saturations[int(tmp2.x)] += tmp;
	case 5:
		g_saturations[int(tmp2.w)] += tmp;
	case 4:
		g_saturations[int(tmp1.z)] += tmp;
	case 3:
		g_saturations[int(tmp1.y)] += tmp;
	case 2:
		g_saturations[int(tmp1.x)] += tmp;
	case 1:
		g_saturations[int(tmp1.w)] += tmp;
	default:
		break;
	}
}
void AbsorbingComplex(){
	int activeCount = flexGetActiveCount(g_flex);
//	printf("active: %d", activeCount);

	int i = g_numSolidParticles;
//	printf("  %d", i);
	while (i < activeCount){
		int collidePosition = -1;
		for (int j = 0; j < g_numPoints; j++){
			if (CollideComplex(i, j)){
				collidePosition = j;
				break;
			}
		}

		//i is absorbable & collided
		if (g_absorbable[i] && collidePosition > -1){
			//proportion
			int tmp = rand() % 10;
			if (tmp >= 10 * g_kAbsorption){
				i++;
				continue;
			}

			//cloth position j 
			UpdatingSaturationsComplex(collidePosition);

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
		if (g_emitTime[i] > 0 && collidePosition == -1){
			g_emitTime[i] = g_emitTime[i] - 1;
			g_velocities[i] = Vec3(0.0f, 0.0f, 0.0f);
		}

		i++;
	}

//	printf(" active: %d\n", activeCount);
	flexSetActive(g_flex, &g_activeIndices[0], activeCount, eFlexMemoryHost);

}


/*diffuse*/

// piece
void CalculateTriangleCenters(){

	if (g_triangleCenters.size() == 0)
		g_triangleCenters.resize(g_numTriangles);

	//printf("size: %d\n", g_triangleCenters.size());

	int dimx = g_dx;
	int dimy = g_dy;
	int dimz = g_dz;
	int index = 0;
	for (int z = 0; z < dimz; z++)
		for (int y = 0; y < dimy; y++)
			for (int x = 0; x < dimx; x++){
				if (x > 0 && y > 0){
					Vec3 position0 = g_positions[GridIndex(x - 1, y - 1, z, dimx, dimy)];
					Vec3 position1 = g_positions[GridIndex(x, y - 1, z, dimx, dimy)];
					Vec3 position2 = g_positions[GridIndex(x, y, z, dimx, dimy)];
					Vec3 position3 = g_positions[GridIndex(x - 1, y, z, dimx, dimy)];

					Vec3 center1 = position1 + (position0 - position1 + position2 - position1) / 3;
					Vec3 center2 = position3 + (position0 - position3 + position2 - position3) / 3;

					g_triangleCenters[index] = center1;
					g_triangleCenters[index + 1] = center2;

					//printf("%d, %f, %f, %f\n", GridIndex(x - 1, y - 1, dimx), position0.x, position0.y, position0.z);
					//printf("%d, %f, %f, %f\n", GridIndex(x, y - 1, dimx), position1.x, position1.y, position1.z);
					//printf("%d, %f, %f, %f\n", GridIndex(x, y, dimx), position2.x, position2.y, position2.z);

					//printf("%d, %f, %f, %f\n", index, center1.x, center1.y, center1.z);
					//printf("%d, %f, %f, %f\n", index + 1, center2.x, center2.y, center2.z);
					//printf("%d, %d, %d\n", x, y, z);

					index += 2;

				}

			}


}
Vec3 calculateCosTheta(int index, int idx1, int idx2, int idx3){
	float t1 = 10.0, t2 = 10.0, t3 = 10.0;

	int dy = (g_dx - 1) * 2;
	int dz = (g_dx - 1) * (g_dy - 1) * 2;
	int z = index / dz;
	int min = z * dz;
	int max = min + dz - 1;
	
	if (idx1 >= min && idx1 < max && ((index - 1) % dy != 0)){
		Vec3 dir = g_triangleCenters[idx1] - g_triangleCenters[index];
		Vec3 gDir = Vec3(0.0, -1.0, 0.0);
		t1 = (-dir.y) / (sqrt(sqr(dir.x) + sqr(dir.y) + sqr(dir.z)));
	}
	if (idx2 >= min && idx2 < max && ((index + 2) % dy != 0)){
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
		g_thetas[index] = calculateCosTheta(index, index - dy + 1, index + 3, index + 1);
		index++;
		g_thetas[index] = calculateCosTheta(index, index - 3, index - 1, index + dy - 1);
		index++;

		//printf("1:%d, %f, %f, %f\n", index, g_thetas[index].x, g_thetas[index].y, g_thetas[index].z);
		//printf("%d, %f, %f\n", index, g_thetas[index], g_thetas[index + 1]);
	}

}

//common
void CalculateTriangleCenters2(){
	if (g_triangleCenters.size() == 0)
		g_triangleCenters.resize(g_numTriangles);
	
	for (int i = 0; i < g_numTriangles; i++){
		Vec3 points = g_trianglePoints[i];
		Vec3 position0 = g_positions[int(points.x)];
		Vec3 position1 = g_positions[int(points.y)];
		Vec3 position2 = g_positions[int(points.z)];

		g_triangleCenters[i] = position1 + (position0 - position1 + position2 - position1) / 3;

	}
}
Vec3 calculateCosTheta2(int index){
	float t1 = 10.0, t2 = 10.0, t3 = 10.0;
	Vec3 neighbours = g_triangleNeighbours[index];
	if (neighbours.x >= 0){
		Vec3 dir = g_triangleCenters[int(neighbours.x)] - g_triangleCenters[index];
		Vec3 gDir = Vec3(0.0, -1.0, 0.0);
		t1 = (-dir.y) / (sqrt(sqr(dir.x) + sqr(dir.y) + sqr(dir.z)));
	}
	if (neighbours.y >= 0){
		Vec3 dir = g_triangleCenters[int(neighbours.y)] - g_triangleCenters[index];
		Vec3 gDir = Vec3(0.0, -1.0, 0.0);
		t2 = (-dir.y) / (sqrt(sqr(dir.x) + sqr(dir.y) + sqr(dir.z)));
	}
	if (neighbours.z >= 0){
		Vec3 dir = g_triangleCenters[int(neighbours.z)] - g_triangleCenters[index];
		Vec3 gDir = Vec3(0.0, -1.0, 0.0);
		t3 = (-dir.y) / (sqrt(sqr(dir.x) + sqr(dir.y) + sqr(dir.z)));
	}
	return Vec3(t1, t2, t3);
}
void CalculateThetas2(){

	if (g_thetas.size() == 0)
		g_thetas.resize(g_numTriangles);

	for (int i = 0; i < g_numTriangles; i++){
		g_thetas[i] = calculateCosTheta2(i);
	}
}

//hollow special
void CalculateTriangleCentersHollow(){

	if (g_triangleCenters.size() == 0)
		g_triangleCenters.resize(g_numTriangles);

	//printf("size: %d\n", g_triangleCenters.size());

	int dimx = g_dx;
	int dimy = g_dy;
	int dimz = g_dz;
	int index = 0;

	//up
	{
		int z = 0;
		for (int y = 0; y < dimy; y++){
			for (int x = 0; x < dimx; x++){
				if (x > 0 && y > 0){
					Vec3 position0 = g_positions[GridIndex(x - 1, y - 1, z, dimx, dimy)];
					Vec3 position1 = g_positions[GridIndex(x, y - 1, z, dimx, dimy)];
					Vec3 position2 = g_positions[GridIndex(x, y, z, dimx, dimy)];
					Vec3 position3 = g_positions[GridIndex(x - 1, y, z, dimx, dimy)];

					Vec3 center1 = position1 + (position0 - position1 + position2 - position1) / 3;
					Vec3 center2 = position3 + (position0 - position3 + position2 - position3) / 3;

					g_triangleCenters[index] = center1;
					g_triangleCenters[index + 1] = center2;

					index += 2;
				}
			}
		}
	}
	//down
	{
		int z = dimz - 1;
		for (int y = 0; y < dimy; y++){
			for (int x = 0; x < dimx; x++){
				if (x > 0 && y > 0){
					Vec3 position0 = g_positions[GridIndex(x - 1, y - 1, z, dimx, dimy)];
					Vec3 position1 = g_positions[GridIndex(x, y - 1, z, dimx, dimy)];
					Vec3 position2 = g_positions[GridIndex(x, y, z, dimx, dimy)];
					Vec3 position3 = g_positions[GridIndex(x - 1, y, z, dimx, dimy)];

					Vec3 center1 = position1 + (position0 - position1 + position2 - position1) / 3;
					Vec3 center2 = position3 + (position0 - position3 + position2 - position3) / 3;

					g_triangleCenters[index] = center1;
					g_triangleCenters[index + 1] = center2;

					index += 2;
				}
			}
		}
	}

	//back
	{
		int y = 0;
		for (int z = 0; z < dimz; z++){
			for (int x = 0; x < dimx; x++){
				if (x > 0 && z > 0){
					Vec3 position0 = g_positions[GridIndex(x - 1, y, z - 1, dimx, dimy)];
					Vec3 position1 = g_positions[GridIndex(x, y, z - 1, dimx, dimy)];
					Vec3 position2 = g_positions[GridIndex(x, y, z, dimx, dimy)];
					Vec3 position3 = g_positions[GridIndex(x - 1, y, z, dimx, dimy)];

					Vec3 center1 = position1 + (position0 - position1 + position2 - position1) / 3;
					Vec3 center2 = position3 + (position0 - position3 + position2 - position3) / 3;

					g_triangleCenters[index] = center1;
					g_triangleCenters[index + 1] = center2;

					index += 2;

				}
			}
		}
	}
	//front
	{
		int y = dimy - 1;
		for (int z = 0; z < dimz; z++){
			for (int x = 0; x < dimx; x++){
				if (x > 0 && z > 0){
					Vec3 position0 = g_positions[GridIndex(x - 1, y, z - 1, dimx, dimy)];
					Vec3 position1 = g_positions[GridIndex(x, y, z - 1, dimx, dimy)];
					Vec3 position2 = g_positions[GridIndex(x, y, z, dimx, dimy)];
					Vec3 position3 = g_positions[GridIndex(x - 1, y, z, dimx, dimy)];

					Vec3 center1 = position1 + (position0 - position1 + position2 - position1) / 3;
					Vec3 center2 = position3 + (position0 - position3 + position2 - position3) / 3;

					g_triangleCenters[index] = center1;
					g_triangleCenters[index + 1] = center2;

					index += 2;
				}
			}
		}
	}
	//left
	{
		int x = 0;
		for (int z = 0; z < dimz; z++){
			for (int y = 0; y < dimy; y++){
				if (y > 0 && z > 0){
					Vec3 position0 = g_positions[GridIndex(x, y - 1, z - 1, dimx, dimy)];
					Vec3 position1 = g_positions[GridIndex(x, y, z - 1, dimx, dimy)];
					Vec3 position2 = g_positions[GridIndex(x, y, z, dimx, dimy)];
					Vec3 position3 = g_positions[GridIndex(x, y - 1, z, dimx, dimy)];

					Vec3 center1 = position1 + (position0 - position1 + position2 - position1) / 3;
					Vec3 center2 = position3 + (position0 - position3 + position2 - position3) / 3;

					g_triangleCenters[index] = center1;
					g_triangleCenters[index + 1] = center2;

					index += 2;

				}
			}
		}
	}
	//right
	{
		int x = dimx - 1;
		for (int z = 0; z < dimz; z++){
			for (int y = 0; y < dimy; y++){
				if (y > 0 && z > 0){
					Vec3 position0 = g_positions[GridIndex(x, y - 1, z - 1, dimx, dimy)];
					Vec3 position1 = g_positions[GridIndex(x, y, z - 1, dimx, dimy)];
					Vec3 position2 = g_positions[GridIndex(x, y, z, dimx, dimy)];
					Vec3 position3 = g_positions[GridIndex(x, y - 1, z, dimx, dimy)];

					Vec3 center1 = position1 + (position0 - position1 + position2 - position1) / 3;
					Vec3 center2 = position3 + (position0 - position3 + position2 - position3) / 3;

					g_triangleCenters[index] = center1;
					g_triangleCenters[index + 1] = center2;

					index += 2;

				}
			}
		}
	}

}
Vec3 calculateCosThetaHollow(int index, int idx1, int idx2, int idx3){
	float t1, t2, t3;
	{
		Vec3 dir = g_triangleCenters[idx1] - g_triangleCenters[index];
		Vec3 gDir = Vec3(0.0, -1.0, 0.0);
		t1 = (-dir.y) / (sqrt(sqr(dir.x) + sqr(dir.y) + sqr(dir.z)));
	}
	{
		Vec3 dir = g_triangleCenters[idx2] - g_triangleCenters[index];
		Vec3 gDir = Vec3(0.0, -1.0, 0.0);
		t2 = (-dir.y) / (sqrt(sqr(dir.x) + sqr(dir.y) + sqr(dir.z)));
	}
	{
		Vec3 dir = g_triangleCenters[idx3] - g_triangleCenters[index];
		Vec3 gDir = Vec3(0.0, -1.0, 0.0);
		t3 = (-dir.y) / (sqrt(sqr(dir.x) + sqr(dir.y) + sqr(dir.z)));
	}

	return Vec3(t1, t2, t3);
}
void CalculateThetasHollow(){

	if (g_thetas.size() == 0)
		g_thetas.resize(g_numTriangles);
	if (g_triangleNeighbours.size() == 0)
		g_triangleNeighbours.resize(g_numTriangles);

	//printf("%d\n", g_numTriangles);

	int grid_x = g_dx - 1;
	int grid_y = g_dy - 1;
	int grid_z = g_dz - 1;

	int triangle_z = grid_x * grid_y * 2;	//up & down
	int triangle_y = grid_x * grid_z * 2;	//back & front
	int triangle_x = grid_y * grid_z * 2;	//left & right

	int start_down = 0;
	int start_up = start_down + triangle_z;
	int start_back = start_up + triangle_z;
	int start_front = start_back + triangle_y;
	int start_left = start_front + triangle_y;
	int start_right = start_left + triangle_x;

	int triangle_dx = grid_x * 2;
	int triangle_dy = grid_y * 2;

	int index = 0;
	//down
	{
		int z = 0;
		for (int y = 0; y < grid_y; y++){
			for (int x = 0; x < grid_x; x++){
				{
					int idx1 = index - triangle_dx + 1;
					int idx2 = index + 3;
					int idx3 = index + 1;

					if (y == 0){
						//triangle idx1 is at back face
						idx1 = start_back + (x * 2);
					}
					if (x == grid_x - 1){
						//triangle idx2 is at right face
						idx2 = start_right + (y * 2);
					}
					g_thetas[index] = calculateCosThetaHollow(index, idx1, idx2, idx3);
					g_triangleNeighbours[index] = Vec3(idx1, idx2, idx3);

					index++;
				}
				{
					int idx1 = index - 3;
					int idx2 = index - 1;
					int idx3 = index + triangle_dx - 1;

					if (x == 0){
						//triangle idx1 is at left face
						idx1 = start_left + (y * 2);
					}
					if (y == grid_y - 1){
						//triangle idx3 is at front face
						idx3 = start_front + (x * 2);
					}
					g_thetas[index] = calculateCosThetaHollow(index, idx1, idx2, idx3);
					g_triangleNeighbours[index] = Vec3(idx1, idx2, idx3);
					index++;
				}
			}
		}
	}
	//up
	{
		int z = grid_z;
		for (int y = 0; y < grid_y; y++){
			for (int x = 0; x < grid_x; x++){
				{
					int idx1 = index - triangle_dx + 1;
					int idx2 = index + 1;
					int idx3 = index - 1;

					if (y == 0){
						//triangle idx1 is at back face
						idx1 = start_back + ((grid_z - 1) * triangle_dx) + (x * 2 + 1);
					}
					if (x == 0){
						//triangle idx3 is at left face
						idx3 = start_left + ((grid_z - 1) * triangle_dy) + (y * 2 + 1);
					}
					g_thetas[index] = calculateCosThetaHollow(index, idx1, idx2, idx3);
					g_triangleNeighbours[index] = Vec3(idx1, idx2, idx3);
					index++;
				}
				{
					int idx1 = index - 1;
					int idx2 = index + 1;
					int idx3 = index + triangle_dx - 1;

					if (x == grid_x - 1){
						//triangle idx2 is at right face
						idx2 = start_right + ((grid_z - 1) * triangle_dy) + (y * 2 + 1);
					}
					if (y == grid_y - 1){
						//triangle idx3 is at front face
						idx3 = start_front + ((grid_z - 1) * triangle_dx) + (x * 2 + 1);
					}
					g_thetas[index] = calculateCosThetaHollow(index, idx1, idx2, idx3);
					g_triangleNeighbours[index] = Vec3(idx1, idx2, idx3);
					index++;
				}
			}
		}
	}
	//back
	{
		int y = 0;
		for (int z = 0; z < grid_z; z++){
			for (int x = 0; x < grid_x; x++){
				{
					int idx1 = index - 1;
					int idx2 = index + 1;
					int idx3 = index - triangle_dx + 1;

					if (x == 0){
						//triangle idx1 is at left face
						idx1 = start_left + (z * triangle_dy);
					}
					if (z == 0){
						//triangle idx3 is at down face
						idx3 = start_down + (x * 2);
					}
					g_thetas[index] = calculateCosThetaHollow(index, idx1, idx2, idx3);
					g_triangleNeighbours[index] = Vec3(idx1, idx2, idx3);

					index++;
				}
				{
					int idx1 = index + triangle_dx - 1;
					int idx2 = index + 1;
					int idx3 = index - 1;

					if (z == grid_z - 1){
						//triangle idx1 is at up face
						idx1 = start_up + (x * 2);
					}
					if (x == grid_x - 1){
						//triangle idx2 is at right face
						idx2 = start_right + (z * triangle_dy + 1);
					}
					g_thetas[index] = calculateCosThetaHollow(index, idx1, idx2, idx3);
					g_triangleNeighbours[index] = Vec3(idx1, idx2, idx3);

					index++;
				}
			}
		}
	}
	//front
	{
		int y = grid_y;
		for (int z = 0; z < grid_z; z++){
			for (int x = 0; x < grid_x; x++){
				{
					int idx1 = index + 1;
					int idx2 = index + 3;
					int idx3 = index - triangle_dx + 1;

					if (x == grid_x - 1){
						//triangle idx2 is at right face
						idx2 = start_right + ((z + 1) * triangle_dy - 2);
					}

					if (z == 0){
						//triangle idx3 is at down face
						idx3 = start_down + ((grid_y - 1) * triangle_dx) + (x * 2 + 1);
					}

					g_thetas[index] = calculateCosThetaHollow(index, idx1, idx2, idx3);
					g_triangleNeighbours[index] = Vec3(idx1, idx2, idx3);
					index++;
				}
				{
					int idx1 = index + triangle_dx - 1;
					int idx2 = index - 1;
					int idx3 = index - 3;

					if (z == grid_z - 1){
						//triangle idx1 is at up face
						idx1 = start_up + ((grid_y - 1) * triangle_dx) + (x * 2 + 1);
					}

					if (x == 0){
						//triangle idx3 is at left face
						idx3 = start_left + ((z + 1) * triangle_dy - 1);
					}

					g_thetas[index] = calculateCosThetaHollow(index, idx1, idx2, idx3);
					g_triangleNeighbours[index] = Vec3(idx1, idx2, idx3);
					index++;
				}
			}
		}

	}
	//left
	{
		int x = 0;
		for (int z = 0; z < grid_z; z++){
			for (int y = 0; y < grid_y; y++){
				{
					int idx1 = index - 1;
					int idx2 = index + 1;
					int idx3 = index - triangle_dy + 1;

					if (y == 0){
						//triangle idx1 is at back face
						idx1 = start_back + (z * triangle_dx);
					}
					if (z == 0){
						//triangle idx3 is at down face
						idx3 = start_down + (y * triangle_dx + 1);
					}

					g_thetas[index] = calculateCosThetaHollow(index, idx1, idx2, idx3);
					g_triangleNeighbours[index] = Vec3(idx1, idx2, idx3);
					index++;
				}
				{
					int idx1 = index + triangle_dy - 1;
					int idx2 = index - 1;
					int idx3 = index - 3;

					if (z == grid_z - 1){
						//triangle idx1 is at up face
						idx1 = start_up + (y * triangle_dx);
					}
					if (y == grid_y - 1){
						//triangle idx2 is at front face
						idx2 = start_front + (z * triangle_dx + 1);
					}

					g_thetas[index] = calculateCosThetaHollow(index, idx1, idx2, idx3);
					g_triangleNeighbours[index] = Vec3(idx1, idx2, idx3);
					index++;
				}
			}
		}
	}
	//right
	{
		int x = grid_x; 
		for (int z = 0; z < grid_z; z++){
			for (int y = 0; y < grid_y; y++){
				{
					int idx1 = index + 1;
					int idx2 = index + 3;
					int idx3 = index - triangle_dy + 1;

					if (y == grid_y - 1){
						//triangle idx2 is at front face
						idx2 = start_front + ((z + 1) * triangle_dx - 2);
					}
					if (z == 0){
						//triangle idx3 is at down face
						idx3 = start_down + ((y + 1) * triangle_dx - 2);
					}

					g_thetas[index] = calculateCosThetaHollow(index, idx1, idx2, idx3);
					g_triangleNeighbours[index] = Vec3(idx1, idx2, idx3);
					index++;
				}
				{
					int idx1 = index + triangle_dy - 1;
					int idx2 = index - 1;
					int idx3 = index - 3;

					if (z == grid_z - 1){
						//triangle idx1 is at up face
						idx1 = start_up + ((y + 1) * triangle_dx - 1);
					}
					if (y == 0){
						//triangle idx3 is at back face
						idx3 = start_back + ((z + 1) * triangle_dx - 1);
					}

					g_thetas[index] = calculateCosThetaHollow(index, idx1, idx2, idx3);
					g_triangleNeighbours[index] = Vec3(idx1, idx2, idx3);
					index++;
				}
			}
		}
	}

}

//cube
void CalculateCubeCenters(){
	if (g_cubeCenters .size() == 0)
		g_cubeCenters.resize(g_numTriangles);

	int dimx = g_dx - 1;
	int dimy = g_dy - 1;
	int dimz = g_dz - 1;

	int dx = 1;
	int dy = g_dx;
	int dz = g_dx * g_dy;
	int delta = dx + dy + dz;

	int index = 0;

	for (int z = 0; z < dimz; z++)
		for (int y = 0; y < dimy; y++)
			for (int x = 0; x < dimx; x++){
				int pos = x * dx + y * dy + z * dz;
				Vec3 pos0 = g_positions[pos];
				Vec3 pos1 = g_positions[pos + delta];

				g_cubeCenters[index] = (pos0 + pos1) / 2;

				index++;
			}
}
Vec3 calculateCosThetaCube(int index, int idx1, int idx2, int idx3){
	float t1 = 10.0, t2 = 10.0, t3 = 10.0;

	if (idx1 >= 0)
	{
		Vec3 dir = g_cubeCenters[idx1] - g_cubeCenters[index];
		Vec3 gDir = Vec3(0.0, -1.0, 0.0);
		t1 = (-dir.y) / (sqrt(sqr(dir.x) + sqr(dir.y) + sqr(dir.z)));
	}
	if (idx2 >= 0)
	{
		Vec3 dir = g_cubeCenters[idx2] - g_cubeCenters[index];
		Vec3 gDir = Vec3(0.0, -1.0, 0.0);
		t2 = (-dir.y) / (sqrt(sqr(dir.x) + sqr(dir.y) + sqr(dir.z)));
	}
	if (idx3 >= 0)
	{
		Vec3 dir = g_cubeCenters[idx3] - g_cubeCenters[index];
		Vec3 gDir = Vec3(0.0, -1.0, 0.0);
		t3 = (-dir.y) / (sqrt(sqr(dir.x) + sqr(dir.y) + sqr(dir.z)));
	}

	return Vec3(t1, t2, t3);
}
void CalculateThetasCube(){
	if (g_thetas.size() == 0)
		g_thetas.resize(g_numCubes * 2);

	int dimx = g_dx - 1;
	int dimy = g_dy - 1;
	int dimz = g_dz - 1;

	int dx = 1;
	int dy = dimx;
	int dz = dimx * dimy;

	int index = 0;

	for (int z = 0; z < dimz; z++){
		for (int y = 0; y < dimy; y++){
			for (int x = 0; x < dimx; x++){
				int idx1 = -1, idx2 = -1, idx3 = -1, idx4 = -1, idx5 = -1, idx6 = -1;
				//down
				if (z - 1 >= 0){
					idx1 = index - dz;
				}
				//up
				if (z + 1 < dimz){
					idx2 = index + dz;
				}
				//back
				if (y - 1 >= 0){
					idx3 = index - dy;
				}
				//front
				if (y + 1 < dimy){
					idx4 = index + dy;
				}
				//left
				if (x - 1 >= 0){
					idx5 = index - dx;
				}
				//right
				if (x + 1 < dimx){
					idx6 = index + dx;
				}

				g_thetas[index * 2] = calculateCosThetaCube(index, idx1, idx2, idx3);
				g_thetas[index * 2 + 1] = calculateCosThetaCube(index, idx4, idx5, idx6);

				index++;
			}
		}
	}

}

//point
Vec3 calculateCosThetaPoint(int index, int idx1, int idx2, int idx3){
	float t1 = 10.0, t2 = 10.0, t3 = 10.0;

	if (idx1 >= 0)
	{
		Vec3 dir = g_positions[idx1] - g_positions[index];
		Vec3 gDir = Vec3(0.0, -1.0, 0.0);
		t1 = (-dir.y) / (sqrt(sqr(dir.x) + sqr(dir.y) + sqr(dir.z)));
	}
	if (idx2 >= 0)
	{
		Vec3 dir = g_positions[idx2] - g_positions[index];
		Vec3 gDir = Vec3(0.0, -1.0, 0.0);
		t2 = (-dir.y) / (sqrt(sqr(dir.x) + sqr(dir.y) + sqr(dir.z)));
	}
	if (idx3 >= 0)
	{
		Vec3 dir = g_positions[idx3] - g_positions[index];
		Vec3 gDir = Vec3(0.0, -1.0, 0.0);
		t3 = (-dir.y) / (sqrt(sqr(dir.x) + sqr(dir.y) + sqr(dir.z)));
	}

	return Vec3(t1, t2, t3);
}
void CalvulateThetasPoint0(){		//hollow
	if (g_thetas.size() == 0)
		g_thetas.resize(g_numPoints * 2);

	int dimx = g_dx;
	int dimy = g_dy;
	int dimz = g_dz;

	int dx = 1;
	int dy = dimx;
	int dz = dimx * dimy;

	int index = 0;

	for (int z = 0; z < dimz; z++){
		for (int y = 0; y < dimy; y++){
			for (int x = 0; x < dimx; x++){
				int idx1 = -1, idx2 = -1, idx3 = -1, idx4 = -1, idx5 = -1, idx6 = -1;
				if (x == 0 || x == dimx - 1 || y == 0 || y == dimy - 1){
					//down
					if (z - 1 >= 0){
						idx1 = index - dz;
					}
					//up
					if (z + 1 < dimz){
						idx2 = index + dz;
					}
				}
				if (x == 0 || x == dimx - 1 || z == 0 || z == dimz - 1){
					//back
					if (y - 1 >= 0){
						idx3 = index - dy;
					}
					//front
					if (y + 1 < dimy){
						idx4 = index + dy;
					}
				}
				if (y == 0 || y == dimy - 1 || z == 0 || z == dimz - 1){
					//left
					if (x - 1 >= 0){
						idx5 = index - dx;
					}
					//right
					if (x + 1 < dimx){
						idx6 = index + dx;
					}
				}
				g_thetas[index * 2] = calculateCosThetaPoint(index, idx1, idx2, idx3);
				g_thetas[index * 2 + 1] = calculateCosThetaPoint(index, idx4, idx5, idx6);

				index++;
			}
		}
	}
}
void CalculateThetasPoint(){		//solid
	if (is_hollow){
		CalvulateThetasPoint0();
		return;
	}

	if (g_thetas.size() == 0)
		g_thetas.resize(g_numPoints * 2);

	int dimx = g_dx;
	int dimy = g_dy;
	int dimz = g_dz;

	int dx = 1;
	int dy = dimx;
	int dz = dimx * dimy;

	int index = 0;

	for (int z = 0; z < dimz; z++){
		for (int y = 0; y < dimy; y++){
			for (int x = 0; x < dimx; x++){
				int idx1 = -1, idx2 = -1, idx3 = -1, idx4 = -1, idx5 = -1, idx6 = -1;
				//down
				if (z - 1 >= 0){
					idx1 = index - dz;
				}
				//up
				if (z + 1 < dimz){
					idx2 = index + dz;
				}
				//back
				if (y - 1 >= 0){
					idx3 = index - dy;
				}
				//front
				if (y + 1 < dimy){
					idx4 = index + dy;
				}
				//left
				if (x - 1 >= 0){
					idx5 = index - dx;
				}
				//right
				if (x + 1 < dimx){
					idx6 = index + dx;
				}

				g_thetas[index * 2] = calculateCosThetaPoint(index, idx1, idx2, idx3);
				g_thetas[index * 2 + 1] = calculateCosThetaPoint(index, idx4, idx5, idx6);

				index++;
			}
		}
	}
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
			Vec3 thetas = g_thetas[index];
			Vec3 deltasin = Vector3(0.0, 0.0, 0.0);
			if (thetas.x <= 1.0){
				deltasin.x = getMax(0.0f, g_kDiffusion * (si - g_saturations[index - dy + 1]) + g_kDiffusionGravity * si * thetas.x);
				sSum += deltasin.x;
			}
			if (thetas.y <= 1.0){
				deltasin.y = getMax(0.0f, g_kDiffusion * (si - g_saturations[index + 3]) + g_kDiffusionGravity * si * thetas.y);
				sSum += deltasin.y;
			}
			if (thetas.z <= 1.0){
				deltasin.z = getMax(0.0f, g_kDiffusion * (si - g_saturations[index + 1]) + g_kDiffusionGravity * si * thetas.z);
				sSum += deltasin.z;
			}

			float normFac = 1.0;
			if (sSum > si){
				normFac = si / sSum;
			}

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


			//if (!(deltas[index] >= 0)){
			//	printf("%d, %f, %f, %f, %f\n", index, g_saturations[index], thetas.x, thetas.y, thetas.z);
			//	printf("%f, %f, %f\n", sSum, si, normFac);
			//	printf("%f, %f, %f, %f\n\n", deltas[index], deltas[index - dy + 1], deltas[index + 3], deltas[index + 1]);
			//}

			index++;
		}
		{
			float sSum = 0;
			float si = g_saturations[index];
			Vec3 thetas = g_thetas[index];
			//printf("%f, %f, %d\n", si, g_saturations[index], index);
			Vec3 deltasin = Vector3(0.0, 0.0, 0.0);
			if (thetas.x <= 1.0){
				deltasin.x = getMax(0.0f, g_kDiffusion * (si - g_saturations[index - 3]) + g_kDiffusionGravity * si * thetas.x);
				sSum += deltasin.x;
			}
			if (thetas.y <= 1.0){
				deltasin.y = getMax(0.0f, g_kDiffusion * (si - g_saturations[index - 1]) + g_kDiffusionGravity * si * thetas.y);
				sSum += deltasin.y;
			}
			if (thetas.z <= 1.0){
				deltasin.z = getMax(0.0f, g_kDiffusion * (si - g_saturations[index + dy - 1]) + g_kDiffusionGravity * si * thetas.z);
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

			//if (!(deltas[index] >= 0)){
			//	printf("%d, %f, %f, %f, %f\n", index, g_saturations[index], thetas.x, thetas.y, thetas.z);
			//	printf("%f, %f, %f\n", sSum, si, normFac);
			//	printf("%f, %f, %f, %f\n\n", deltas[index], deltas[index - dy + 1], deltas[index + 3], deltas[index + 1]);
			//}

			index++;
		}
	}
	int triangleNum = g_numTriangles;
	int tmp = (g_dx - 1) * 2;
	for (int i = 0; i < triangleNum; i++){
		g_saturations[i] += deltas[i];
		if (g_saturations[i] < 0){
			if (g_saturations[i] == -0.0){
				printf("mark\n");
			}
			g_saturations[i] = 0;
		}
		/*if (g_saturations[i] < 0){
			printf("<0: %d\n", i);
		}*/
		//if (i % tmp == 4 && i / tmp < 8)
		//	printf("\n");
		//if (i % tmp == 8 && i / tmp < 8)
		//	printf("\n");
		//if (i % tmp == 8 && i / tmp == 8)
		//	printf("\n");
		//if (i % tmp < 8 && i / tmp < 8)
		//	printf("%d %.2f %.2f\t", i, deltas[i], g_saturations[i]);
	}

	//tmp = 0;
	//for (int i = 0; i < g_numTriangles; i++){
	//	if (g_saturations[i] > g_maxSaturation * g_kAbsorption){
	//		if (tmp % 4 == 0)
	//			printf("\n");
	//		tmp++;
	//		printf("%d: %f\t", i, g_saturations[i]);
	//	}
	//}
	//printf("\n%d\n", tmp);


}

void test(){

	vector<float> deltas;
	deltas.reserve(g_numTriangles);

	float max = g_maxSaturation * g_kAbsorption;
	for (int i = 0; i < g_numTriangles; i++){
		if (g_saturations[i] == max){
			Vec3 neighbours = g_triangleNeighbours[i];
			deltas[int(neighbours.x)] = max;
			deltas[int(neighbours.y)] = max;
			deltas[int(neighbours.z)] = max;
		}
	}
	for (int i = 0; i < g_numTriangles; i++){
		if (deltas[i] == max){
			g_saturations[i] = max;
		}
		if (g_saturations[i] == max){
			Vec3 points = g_trianglePoints[i];
			printf("%d (%d, %d, %d)\t", i, int(points.x), int(points.y), int(points.z));
			printf(" (%d, %d, %d)", int(points.x) % 100 % 10, int(points.x) % 100 / 10, int(points.x) / 100);
			printf(" (%d, %d, %d)", int(points.y) % 100 % 10, int(points.y) % 100 / 10, int(points.y) / 100);
			printf(" (%d, %d, %d)\n", int(points.z) % 100 % 10, int(points.z) % 100 / 10, int(points.z) / 100);
		}
	}
	printf("\n");
}
void DiffuseCloth2(){

	vector<float> deltas;
	deltas.resize(g_numTriangles);

	for (int i = 0; i < g_numTriangles; i++){
		float sSum = 0;
		float si = g_saturations[i];
		Vec3 thetas = g_thetas[i];
		Vec3 neighbours = g_triangleNeighbours[i];
		Vec3 deltasin = Vector3(0.0, 0.0, 0.0);

		if (thetas.x <= 1.0){
			deltasin.x = getMax(0.0f, g_kDiffusion * (si - g_saturations[int(neighbours.x)]) + g_kDiffusionGravity * si * thetas.x);
			sSum += deltasin.x;
		}
		if (thetas.y <= 1.0){
			deltasin.y = getMax(0.0f, g_kDiffusion * (si - g_saturations[int(neighbours.y)]) + g_kDiffusionGravity * si * thetas.y);
			sSum += deltasin.y;
		}
		if (thetas.z <= 1.0){
			deltasin.z = getMax(0.0f, g_kDiffusion * (si - g_saturations[int(neighbours.z)]) + g_kDiffusionGravity * si * thetas.z);
			sSum += deltasin.z;
		}

		float normFac = 1.0;
		if (sSum > si){
			normFac = si / sSum;
		}

		if (thetas.x <= 1.0){
			deltas[i] += -normFac * deltasin.x;
			deltas[int(neighbours.x)] += normFac * deltasin.x;//* ai/an = 1.0
		}
		if (thetas.y <= 1.0){
			deltas[i] += -normFac * deltasin.y;
			deltas[int(neighbours.y)] += normFac * deltasin.y;//* ai/an = 1.0
		}
		if (thetas.z <= 1.0){
			deltas[i] += -normFac * deltasin.z;
			deltas[int(neighbours.z)] += normFac * deltasin.z;//* ai/an = 1.0
		}
	}

	for (int i = 0; i < g_numTriangles; i++){
		g_saturations[i] += deltas[i];
		if (g_saturations[i] < 0){
			if (g_saturations[i] == -0.0){
				printf("mark\n");
			}
			g_saturations[i] = 0;
		}

	}
}

void DiffuseClothHollow(){


	vector<float> deltas;
	deltas.resize(g_numTriangles);

	for (int i = 0; i < g_numTriangles; i++){
		float sSum = 0;
		float si = g_saturations[i];
		Vec3 thetas = g_thetas[i];
		Vec3 neighbours = g_triangleNeighbours[i];
		Vec3 deltasin = Vector3(0.0, 0.0, 0.0);

		//printf("%d\n", i);
		//printf("%d %d %d\n", neighbours.x, neighbours.y, neighbours.z);
		//printf("%f %f %f\n", thetas.x, thetas.y, thetas.z);

		if (thetas.x <= 1.0){
			deltasin.x = getMax(0.0f, g_kDiffusion * (si - g_saturations[int(neighbours.x)]) + g_kDiffusionGravity * si * thetas.x);
			sSum += deltasin.x;
		}
		if (thetas.y <= 1.0){
			deltasin.y = getMax(0.0f, g_kDiffusion * (si - g_saturations[int(neighbours.y)]) + g_kDiffusionGravity * si * thetas.y);
			sSum += deltasin.y;
		}
		if (thetas.z <= 1.0){
			deltasin.z = getMax(0.0f, g_kDiffusion * (si - g_saturations[int(neighbours.z)]) + g_kDiffusionGravity * si * thetas.z);
			sSum += deltasin.z;
		}

		float normFac = 1.0;
		if (sSum > si){
			normFac = si / sSum;
		}

		if (thetas.x <= 1.0){
			deltas[i] += -normFac * deltasin.x;
			deltas[int(neighbours.x)] += normFac * deltasin.x;//* ai/an = 1.0
		}
		if (thetas.y <= 1.0){
			deltas[i] += -normFac * deltasin.y;
			deltas[int(neighbours.y)] += normFac * deltasin.y;//* ai/an = 1.0
		}
		if (thetas.z <= 1.0){
			deltas[i] += -normFac * deltasin.z;
			deltas[int(neighbours.z)] += normFac * deltasin.z;//* ai/an = 1.0
		}
	}

	for (int i = 0; i < g_numTriangles; i++){
		g_saturations[i] += deltas[i];
		if (g_saturations[i] < 0){
			if (g_saturations[i] == -0.0){
				printf("mark\n");
			}
			g_saturations[i] = 0;
		}

	}
}

void DiffuseClothCube(){
	int numCube = g_numCubes;

	int dx = 1;
	int dy = g_dx - 1;
	int dz = (g_dx - 1) * (g_dy - 1);

	vector<float> deltas;
	deltas.resize(numCube);

	for (int i = 0; i < numCube; i++){
		float sSum = 0;
		float si = g_saturations[i];
		Vec3 thetas0 = g_thetas[i * 2];
		Vec3 thetas1 = g_thetas[i * 2 + 1];
		Vec3 deltasin0 = Vector3(0.0, 0.0, 0.0);
		Vec3 deltasin1 = Vector3(0.0, 0.0, 0.0);

		if (thetas0.x <= 1.0){
			deltasin0.x = getMax(0.0f, g_kDiffusion * (si - g_saturations[i - dz]) + g_kDiffusionGravity * si * thetas0.x);
			sSum += deltasin0.x;
		}
		if (thetas0.y <= 1.0){
			deltasin0.y = getMax(0.0f, g_kDiffusion * (si - g_saturations[i + dz]) + g_kDiffusionGravity * si * thetas0.y);
			sSum += deltasin0.y;
		}
		if (thetas0.z <= 1.0){
			deltasin0.z = getMax(0.0f, g_kDiffusion * (si - g_saturations[i - dy]) + g_kDiffusionGravity * si * thetas0.z);
			sSum += deltasin0.z;
		}

		if (thetas1.x <= 1.0){
			deltasin1.x = getMax(0.0f, g_kDiffusion * (si - g_saturations[i + dy]) + g_kDiffusionGravity * si * thetas1.x);
			sSum += deltasin1.x;
		}
		if (thetas1.y <= 1.0){
			deltasin1.y = getMax(0.0f, g_kDiffusion * (si - g_saturations[i - dx]) + g_kDiffusionGravity * si * thetas1.y);
			sSum += deltasin1.y;
		}
		if (thetas1.z <= 1.0){
			deltasin1.z = getMax(0.0f, g_kDiffusion * (si - g_saturations[i + dx]) + g_kDiffusionGravity * si * thetas1.z);
			sSum += deltasin1.z;
		}

		float normFac = 1.0;
		if (sSum > si){
			normFac = si / sSum;
		}

		if (thetas0.x <= 1.0){
			deltas[i] += -normFac * deltasin0.x;
			deltas[i - dz] += normFac * deltasin0.x;//* ai/an = 1.0
		}
		if (thetas0.y <= 1.0){
			deltas[i] += -normFac * deltasin0.y;
			deltas[i + dz] += normFac * deltasin0.y;//* ai/an = 1.0
		}
		if (thetas0.z <= 1.0){
			deltas[i] += -normFac * deltasin0.z;
			deltas[i - dy] += normFac * deltasin0.z;//* ai/an = 1.0
		}
		if (thetas1.x <= 1.0){
			deltas[i] += -normFac * deltasin1.x;
			deltas[i + dy] += normFac * deltasin1.x;//* ai/an = 1.0
		}
		if (thetas1.y <= 1.0){
			deltas[i] += -normFac * deltasin1.y;
			deltas[i - dx] += normFac * deltasin1.y;//* ai/an = 1.0
		}
		if (thetas1.z <= 1.0){
			deltas[i] += -normFac * deltasin1.z;
			deltas[i + dx] += normFac * deltasin1.z;//* ai/an = 1.0
		}

		//printf("%d %f\n", i, deltas[i]);
		//printf("%f %f %f %f %f %f\n", thetas0.x, thetas0.y, thetas0.z, thetas1.x, thetas1.y, thetas1.z); 
		//printf("%f %f %f %f %f %f\n", deltasin0.x, deltasin0.y, deltasin0.z, deltasin1.x, deltasin1.y, deltasin1.z);

	}

	for (int i = 0; i < numCube; i++){
		g_saturations[i] += deltas[i];
		if (g_saturations[i] < 0){
			if (g_saturations[i] == -0.0){
				printf("mark\n");
			}
			g_saturations[i] = 0;
		}
	}
}

void DiffuseClothPoint(){		//solid

	int numPoint = g_numPoints;

	int dx = 1;
	int dy = g_dx;
	int dz = g_dx * g_dy;

	vector<float> deltas;
	deltas.resize(numPoint);

	for (int i = 0; i < numPoint; i++){
		float sSum = 0;
		float si = g_saturations[i];
		Vec3 thetas0 = g_thetas[i * 2];
		Vec3 thetas1 = g_thetas[i * 2 + 1];
		Vec3 deltasin0 = Vector3(0.0, 0.0, 0.0);
		Vec3 deltasin1 = Vector3(0.0, 0.0, 0.0);

		if (thetas0.x <= 1.0){
			deltasin0.x = getMax(0.0f, g_kDiffusion * (si - g_saturations[i - dz]) + g_kDiffusionGravity * si * thetas0.x);
			sSum += deltasin0.x;
		}
		if (thetas0.y <= 1.0){
			deltasin0.y = getMax(0.0f, g_kDiffusion * (si - g_saturations[i + dz]) + g_kDiffusionGravity * si * thetas0.y);
			sSum += deltasin0.y;
		}
		if (thetas0.z <= 1.0){
			deltasin0.z = getMax(0.0f, g_kDiffusion * (si - g_saturations[i - dy]) + g_kDiffusionGravity * si * thetas0.z);
			sSum += deltasin0.z;
		}

		if (thetas1.x <= 1.0){
			deltasin1.x = getMax(0.0f, g_kDiffusion * (si - g_saturations[i + dy]) + g_kDiffusionGravity * si * thetas1.x);
			sSum += deltasin1.x;
		}
		if (thetas1.y <= 1.0){
			deltasin1.y = getMax(0.0f, g_kDiffusion * (si - g_saturations[i - dx]) + g_kDiffusionGravity * si * thetas1.y);
			sSum += deltasin1.y;
		}
		if (thetas1.z <= 1.0){
			deltasin1.z = getMax(0.0f, g_kDiffusion * (si - g_saturations[i + dx]) + g_kDiffusionGravity * si * thetas1.z);
			sSum += deltasin1.z;
		}

		float normFac = 1.0;
		if (sSum > si){
			normFac = si / sSum;
		}

		if (thetas0.x <= 1.0){
			deltas[i] += -normFac * deltasin0.x;
			deltas[i - dz] += normFac * deltasin0.x;//* ai/an = 1.0
		}
		if (thetas0.y <= 1.0){
			deltas[i] += -normFac * deltasin0.y;
			deltas[i + dz] += normFac * deltasin0.y;//* ai/an = 1.0
		}
		if (thetas0.z <= 1.0){
			deltas[i] += -normFac * deltasin0.z;
			deltas[i - dy] += normFac * deltasin0.z;//* ai/an = 1.0
		}
		if (thetas1.x <= 1.0){
			deltas[i] += -normFac * deltasin1.x;
			deltas[i + dy] += normFac * deltasin1.x;//* ai/an = 1.0
		}
		if (thetas1.y <= 1.0){
			deltas[i] += -normFac * deltasin1.y;
			deltas[i - dx] += normFac * deltasin1.y;//* ai/an = 1.0
		}
		if (thetas1.z <= 1.0){
			deltas[i] += -normFac * deltasin1.z;
			deltas[i + dx] += normFac * deltasin1.z;//* ai/an = 1.0
		}

	}

	for (int i = 0; i < numPoint; i++){
		g_saturations[i] += deltas[i];
		if (g_saturations[i] < 0){
			if (g_saturations[i] == -0.0){
				printf("mark\n");
			}
			g_saturations[i] = 0;
		}
	}
}


//complex


void CalculateTriangleCentersComples(){
	if (g_triangleCenters.size() == 0)
		g_triangleCenters.resize(g_numTriangles);

	for (int i = 0; i < g_numTriangles; i++){
		Vec3 points = g_trianglePoints[i];
		Vec3 position0 = g_mesh->m_positions[int(points.x)];
		Vec3 position1 = g_mesh->m_positions[int(points.y)];
		Vec3 position2 = g_mesh->m_positions[int(points.z)];

		g_triangleCenters[i] = position1 + (position0 - position1 + position2 - position1) / 3;

	}
}
Vec3 calculateCosThetaComplex(int index){
	float t1 = 10.0, t2 = 10.0, t3 = 10.0;
	Vec3 neighbours = g_triangleNeighbours[index];
	if (neighbours.x >= 0){
		Vec3 dir = g_triangleCenters[int(neighbours.x)] - g_triangleCenters[index];
		Vec3 gDir = Vec3(0.0, -1.0, 0.0);
		t1 = (-dir.y) / (sqrt(sqr(dir.x) + sqr(dir.y) + sqr(dir.z)));
	}
	if (neighbours.y >= 0){
		Vec3 dir = g_triangleCenters[int(neighbours.y)] - g_triangleCenters[index];
		Vec3 gDir = Vec3(0.0, -1.0, 0.0);
		t2 = (-dir.y) / (sqrt(sqr(dir.x) + sqr(dir.y) + sqr(dir.z)));
	}
	if (neighbours.z >= 0){
		Vec3 dir = g_triangleCenters[int(neighbours.z)] - g_triangleCenters[index];
		Vec3 gDir = Vec3(0.0, -1.0, 0.0);
		t3 = (-dir.y) / (sqrt(sqr(dir.x) + sqr(dir.y) + sqr(dir.z)));
	}
	return Vec3(t1, t2, t3);
}
void CalculateThetasComplex(){
	if (g_thetas.size() == 0)
		g_thetas.resize(g_numTriangles);

	for (int i = 0; i < g_numTriangles; i++){
		g_thetas[i] = calculateCosThetaComplex(i);
	}
}
void DiffuseClothComplex(){
	vector<float> deltas;
	deltas.resize(g_numTriangles);

	for (int i = 0; i < g_numTriangles; i++){
		float sSum = 0;
		float si = g_saturations[i];
		Vec3 thetas = g_thetas[i];
		Vec3 neighbours = g_triangleNeighbours[i];
		Vec3 deltasin = Vector3(0.0, 0.0, 0.0);

		if (thetas.x <= 1.0){
			deltasin.x = getMax(0.0f, g_kDiffusion * (si - g_saturations[int(neighbours.x)]) + g_kDiffusionGravity * si * thetas.x);
			sSum += deltasin.x;
		}
		if (thetas.y <= 1.0){
			deltasin.y = getMax(0.0f, g_kDiffusion * (si - g_saturations[int(neighbours.y)]) + g_kDiffusionGravity * si * thetas.y);
			sSum += deltasin.y;
		}
		if (thetas.z <= 1.0){
			deltasin.z = getMax(0.0f, g_kDiffusion * (si - g_saturations[int(neighbours.z)]) + g_kDiffusionGravity * si * thetas.z);
			sSum += deltasin.z;
		}

		float normFac = 1.0;
		if (sSum > si){
			normFac = si / sSum;
		}

		if (thetas.x <= 1.0){
			deltas[i] += -normFac * deltasin.x;
			deltas[int(neighbours.x)] += normFac * deltasin.x;//* ai/an = 1.0
		}
		if (thetas.y <= 1.0){
			deltas[i] += -normFac * deltasin.y;
			deltas[int(neighbours.y)] += normFac * deltasin.y;//* ai/an = 1.0
		}
		if (thetas.z <= 1.0){
			deltas[i] += -normFac * deltasin.z;
			deltas[int(neighbours.z)] += normFac * deltasin.z;//* ai/an = 1.0
		}
	}

	for (int i = 0; i < g_numTriangles; i++){
		g_saturations[i] += deltas[i];
		if (g_saturations[i] < 0){
			if (g_saturations[i] == -0.0){
				printf("mark\n");
			}
			g_saturations[i] = 0;
		}

	}
}


/*dripping*/

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
		g_absorbable[activeCount] = true;
		activeCount++;

	}


}
void CreateParticleCube(int index, int &activeCount){

	Vec3 emitterDir = Vec3(0.0f, -1.0f, 0.0f);
	Vec3 emitterRight = Vec3(-1.0f, 0.0f, 0.0f);

	//position

	int z = index / ((g_dx - 1) * (g_dy - 1));
	int tmp = index % ((g_dx - 1) * (g_dy - 1));
	int y = tmp / (g_dx - 1);
	int x = tmp % (g_dx - 1);

	//printf("%d, %d %d %d\n", index, x, y, z);

	int dx = 1;
	int dy = g_dx;
	int dz = g_dx * g_dy;

	Vec3 aPos = Vec3(g_positions[x * dx + y * dy + z * dz]);
	Vec3 bPos = Vec3(g_positions[x * dx + y * dy + z * dz + dx]);
	Vec3 cPos = Vec3(g_positions[x * dx + y * dy + z * dz + dy]);;
	Vec3 dPos = Vec3(g_positions[x * dx + y * dy + z * dz + dx + dy]);;

	Vec3 centerPos = (aPos + dPos) / 2;

	int a = rand() % 101;
	int b = rand() % (101 - a);
	int c = rand() % (101 - a - b);
	int d = rand() % (101 - a - b - c);
	Vec3 emitterPos = (centerPos * (101 - a - b - c - d) + aPos * a + bPos * b + cPos * c + dPos * d) / 100.0;
	emitterPos += emitterDir * g_params.mCollisionDistance * 2.0;

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
		g_absorbable[activeCount] = true;
		activeCount++;

	}
}
void CreateParticlePoint(int index, int &activeCount){
	Vec3 emitterDir = Vec3(0.0f, -1.0f, 0.0f);
	Vec3 emitterRight = Vec3(-1.0f, 0.0f, 0.0f);

	//position

	int z = index / (g_dx * g_dy);
	int tmp = index % (g_dx * g_dy);
	int y = tmp / g_dx;
	int x = tmp % g_dx;

//	printf("%d, %d %d %d\n", index, x, y, z);

	int dx = 1;
	int dy = g_dx;
	int dz = g_dx * g_dy;

	Vec3 centerPos = g_positions[index];

//	printf("%d: %f %f %f\n",index, centerPos.x, centerPos.y, centerPos.z);

	Vec3 aPos = Vec3(0.0, 0.0, 0.0);
	Vec3 bPos = Vec3(0.0, 0.0, 0.0);
	Vec3 cPos = Vec3(0.0, 0.0, 0.0);
	Vec3 dPos = Vec3(0.0, 0.0, 0.0);
	if (x > 0) {
		aPos = Vec3(g_positions[index - dx]) - centerPos;
		int tmp = index - 1;
//		printf("left %d: %f %f %f\n", tmp, g_positions[tmp].x, g_positions[tmp].y, g_positions[tmp].z);
	}
	if (x < g_dx - 1) {
		bPos = Vec3(g_positions[index + dx]) - centerPos;
//		printf("right: %f %f %f\n", g_positions[index + dx].x, g_positions[index + dx].y, g_positions[index + dx].z);
	}
	if (y > 0) {
		cPos = Vec3(g_positions[index - dy]) - centerPos;
//		printf("back: %f %f %f\n", g_positions[index - dy].x, g_positions[index - dy].y, g_positions[index - dy].z);
	}
	if (y < g_dy - 1) {
		dPos = Vec3(g_positions[index + dy]) - centerPos;
//		printf("front: %f %f %f\n", g_positions[index + dy].x, g_positions[index + dy].y, g_positions[index + dy].z);
	}


	int a = rand() % 51;
	int b = rand() % 51;
	int c = rand() % 51;
	int d = rand() % 51;
	Vec3 emitterPos = centerPos + (aPos * a + bPos * b + cPos * c + dPos * d) / 100;
	emitterPos += emitterDir * g_params.mCollisionDistance * 2.0;

//	printf("%f %f %f\n\n", emitterPos.x, emitterPos.y, emitterPos.z);

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
		g_absorbable[activeCount] = true;
		activeCount++;

	}
}

void Dripping(){

	if (g_dripBuffer.size() == 0)
		g_dripBuffer.resize(g_numTriangles);

	int triangleNum = (g_dx - 1) * (g_dy - 1) * 2;

	int activeCount = flexGetActiveCount(g_flex);
//	printf("after drip: %d ", activeCount);

	float maxSaturation = g_maxSaturation * g_kMaxAbsorption;

	for (int i = 0; i < triangleNum; i++){
		//printf("%f ", g_saturations[i]);
		if (g_saturations[i] > maxSaturation){
			float m = g_saturations[i] - maxSaturation;
			g_dripBuffer[i] += m;
			while (g_dripBuffer[i] > g_mDrip){

				//printf("buffer %d: %f\n", i, g_dripBuffer[i]);

				CreateParticle(i, activeCount);

				g_dripBuffer[i] -= g_mDrip;
			}
			g_saturations[i] = maxSaturation;
		}
	}

	//printf("\n");
	//printf("after %d \n\n", activeCount);

	flexSetActive(g_flex, &g_activeIndices[0], activeCount, eFlexMemoryHost);
}
void DrippingCube(){
	if (g_dripBuffer.size() == 0)
		g_dripBuffer.resize(g_numCubes);

	int cubeNum = (g_dx - 1) * (g_dy - 1);

	int activeCount = flexGetActiveCount(g_flex);

	float maxSaturation = g_maxSaturation * g_kMaxAbsorption;

	for (int i = 0; i < cubeNum; i++){
		//printf("%f ", g_saturations[i]);
		if (g_saturations[i] > maxSaturation){

			float m = g_saturations[i] - maxSaturation;
			g_dripBuffer[i] += m;
			while (g_dripBuffer[i] > g_mDrip){

				CreateParticleCube(i, activeCount);

				g_dripBuffer[i] -= g_mDrip;
			}
			g_saturations[i] = maxSaturation;
			//printf("%d %f\n", i, g_saturations[i]);
		}
	}

	//printf("\n");
	//printf(" %d \n", activeCount);

	flexSetActive(g_flex, &g_activeIndices[0], activeCount, eFlexMemoryHost);

}
void DrippingPoint(){
	if (g_dripBuffer.size() == 0)
		g_dripBuffer.resize(g_numPoints);

	int pointsNum = g_dx * g_dy;

	int activeCount = flexGetActiveCount(g_flex);

	float maxSaturation = g_maxSaturation * g_kMaxAbsorption;

	for (int i = 0; i < pointsNum; i++){
		//printf("%f ", g_saturations[i]);
		if (g_saturations[i] > maxSaturation){

			float m = g_saturations[i] - maxSaturation;
			g_dripBuffer[i] += m;
			while (g_dripBuffer[i] > g_mDrip){

				CreateParticlePoint(i, activeCount);

				g_dripBuffer[i] -= g_mDrip;
			}
			g_saturations[i] = maxSaturation;
			//printf("%d %f\n", i, g_saturations[i]);
		}
	}

	//printf("\n");
	//printf(" %d \n", activeCount);

	flexSetActive(g_flex, &g_activeIndices[0], activeCount, eFlexMemoryHost);

}


void StandardizationNormal(){
	if (g_normals.size() == 0){
		g_normals.resize(g_numTriangles);
	}
//	else return;

	for (int i = 0; i < g_numTriangles; i++){
		Vec3 points = g_trianglePoints[i];
		Vec3 normal = (g_mesh->m_normals[int(points.x)] + g_mesh->m_normals[int(points.y)] + g_mesh->m_normals[int(points.z)]) / 3;
		float k = sqrt(sqr(normal.x) + sqr(normal.y) + sqr(normal.z));

		g_normals[i] = Vec4(normal / k, 1.0);

	}
}
bool drippable(int idx){
	Vec3 down = Vec3(0.0, -1.0, 0.0);
	Vec3 normal = g_mesh->m_normals[idx];

	int i = (-normal.y) / 1;
}
void CreateParticleComplex(int index, int &activeCount){

	Vec3 points = g_trianglePoints[index];
	Vec3 aPos = g_mesh->m_positions[int(points.x)];
	Vec3 bPos = g_mesh->m_positions[int(points.y)];
	Vec3 cPos = g_mesh->m_positions[int(points.z)];
	Vec3 centerPos = g_triangleCenters[index];

	int a = rand() % 101;
	int b = rand() % (101 - a);
	int c = rand() % (101 - a - b);
	Vec3 emitterPos = (centerPos * (101 - a - b - c) + aPos * a + bPos * b + cPos * c) / 100.0;

	//Vec3 normal = g_mesh->m_normals[index];
	//float k = sqrt(sqr(normal.x) + sqr(normal.y) + sqr(normal.z));
	//normal = normal / k;

	//emitterPos += normal * sqrt(g_params.mSolidRestDistance);

	//printf("(%f, %f, %f)\t", aPos.x, aPos.y, aPos.z);
	//printf("(%f, %f, %f)\t", bPos.x, bPos.y, bPos.z);
	//printf("(%f, %f, %f)\n", cPos.x, cPos.y, cPos.z);
	//printf("(%f, %f, %f)\t", centerPos.x, centerPos.y, centerPos.z);
	//printf("(%f, %f, %f)\n", normal.x, normal.y, normal.z);
	//printf("(%f, %f, %f)\n\n", emitterPos.x, emitterPos.y, emitterPos.z);

	float r;
	int phase;

	if (g_params.mFluid)
	{
		r = g_params.mFluidRestDistance;
		phase = flexMakePhase(0, eFlexPhaseSelfCollide | eFlexPhaseFluid);
		//phase = flexMakePhase(0, eFlexPhaseFluid);
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
		g_emitTime[activeCount] = 10;
		activeCount++;
	}
}
void DrippingComplex(){
	if (g_dripBuffer.size() == 0)
		g_dripBuffer.resize(g_numTriangles);

//		colorFlag = -1;
	StandardizationNormal();

	int activeCount = flexGetActiveCount(g_flex);
	//	printf("after drip: %d ", activeCount);

	float maxSaturation = g_maxSaturation * g_kMaxAbsorption;

	for (int i = 0; i < g_numTriangles; i++){
		//printf("%f ", g_saturations[i]);
		if (g_normals[i].y <= -0.7 && g_saturations[i] > maxSaturation){
		//if (g_saturations[i] > maxSaturation){
			float m = g_saturations[i] - maxSaturation;
			g_dripBuffer[i] += m;
//			colorFlag = i;
			while (g_dripBuffer[i] > g_mDrip){

				//printf("buffer %d: %f\n", i, g_dripBuffer[i]);

				CreateParticleComplex(i, activeCount);

				g_dripBuffer[i] -= g_mDrip;
			}
			g_saturations[i] = maxSaturation;
//			break;
		}
	}

	//printf("\n");
	//printf("after %d \n\n", activeCount);

	flexSetActive(g_flex, &g_activeIndices[0], activeCount, eFlexMemoryHost);
}