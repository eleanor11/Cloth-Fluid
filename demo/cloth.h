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

#include <set>
#include <vector>
#include <map>
#include <algorithm>
#include <functional>
#include <numeric>

namespace
{
	// calculate the inclusive prefix sum, equivalent of std::partial_sum
	template <typename T>
	void prefixSum(T first, T last, T dest)
	{
		if (first != last)
		{	
			*(dest++) = *(first++);
			for (; first != last; ++first, ++dest)
				*dest = *(dest-1) + *first;
		}
	}

	template <typename T>
	void gatherAdjacencies(std::vector<int>& valency, std::vector<int>& adjacencies, 
		const int* indices, int numIndices)
	{
		// count number of edges per vertex
		for(int i=0; i<numIndices; i++)
			valency[indices[i]] += 2;

		prefixSum(valency.begin(), valency.end(), valency.begin());
		adjacencies.resize(valency.back());



		// gather adjacent vertices
		for(int i=0; i<numIndices; i+=3)
		{
			const int* tIt = indices+i;

			for(int j=0; j<3; ++j)
			{
				adjacencies[--valency[tIt[j]]] = tIt[(j+1)%3];
				adjacencies[--valency[tIt[j]]] = tIt[(j+2)%3];
			}
		}
	}

	
	struct MeshEdge
	{
		MeshEdge() : mStretching(0.0f), mBending(0.0f), mShearing(0.0f) {}

		void classify()
		{
			mStretching += 0.1f;
		}

		// classify v0-v2 edge based on alternative v0-v1-v2 path 
		void classify(const Vec4& v0, const Vec4& v1, const Vec4& v2) const
		{
			const Vec3& p0 = reinterpret_cast<const Vec3&>(v0);
			const Vec3& p1 = reinterpret_cast<const Vec3&>(v1);
			const Vec3& p2 = reinterpret_cast<const Vec3&>(v2);

			float area = Length(Cross(p1-p0,p2-p1));
			// triangle height / base length
			// 1.0 = quad edge, 0.2 = quad diagonal + quad edge, 
			float ratio = area / LengthSq(p2-p0);

			// 0.5 = quad diagonal
			const_cast<MeshEdge*>(this)->mShearing += Max(0.0f, 0.15f - fabsf(0.45f - ratio));
			// 0.0 = collinear points
			const_cast<MeshEdge*>(this)->mBending += Max(0.0f, 0.1f - ratio) * 3;
		}

		float mStretching;
		float mBending;
		float mShearing;
	};

	typedef std::pair<int, int> Pair;
	typedef std::pair<Pair, int> Entry;
	
	// maintain heap status after elements have been pushed (heapify)
	template<typename T>
	void pushHeap(std::vector<T> &heap, const T &value)
	{
		heap.pushBack(value);
		T* begin = heap.begin();
		T* end = heap.end();

		if (end <= begin)
			return;
	
		int current = int(end - begin) - 1;
		while (current > 0)
		{
			const int parent = (current - 1) / 2;
			if (!(begin[parent] < begin[current]))
				break;

			std::swap(begin[parent], begin[current]);
			current = parent;
		}
	}

	// pop one element from the heap
	template<typename T>
	T popHeap(std::vector<T> &heap)
	{
		T* begin = heap.begin();
		T* end = heap.end();

		std::swap(begin[0], end[-1]); // exchange elements

		// shift down
		end--;

		int current = 0;
		while (begin + (current * 2 + 1) < end)
		{
			int child = current * 2 + 1;
			if (begin + child + 1 < end && begin[child] < begin[child + 1])
				++child;

			if (!(begin[current] < begin[child]))
				break;

			std::swap(begin[current], begin[child]);
			current = child;
		}

		return heap.popBack();
	}
}

class ClothMesh
{
public:

	struct Edge
	{
		int vertices[2];
		int tris[2];
		
		int stretchConstraint;
		int bendingConstraint;


		Edge(int a, int b)
		{
			assert(a != b);

			vertices[0] = Min(a, b);
			vertices[1] = Max(a, b);

			tris[0] = -1;
			tris[1] = -1;
			
			stretchConstraint = -1;
			bendingConstraint = -1;
		}

		bool Contains(int index)
		{
			return (vertices[0] == index) || (vertices[1] == index);
		}

		void Replace(int oldIndex, int newIndex)
		{
			if (vertices[0] == oldIndex)
				vertices[0] = newIndex;
			else if (vertices[1] == oldIndex)
				vertices[1] = newIndex;
			else
				assert(0);
		}

		void RemoveTri(int index)
		{
			if (tris[0] == index)
				tris[0] = -1;
			else if (tris[1] == index)
				tris[1] = -1;
			else
				assert(0);
		}

		bool AddTri(int index)
		{
			if (tris[0] == -1)
			{
				tris[0] = index;
				return true;
			}
			else if (tris[1] == -1)
			{
				// check tri not referencing same edge
				if (index == tris[0])
					return false;		
				else
				{
					tris[1] = index;
					return true;
				}
			}
			else
				return false;
		}

		bool operator < (const Edge& rhs) const
		{
			if (vertices[0] != rhs.vertices[0])
				return vertices[0] < rhs.vertices[0];
			else
				return vertices[1] < rhs.vertices[1];
		}
	};

	struct Triangle
	{
		Triangle(int a, int b, int c)
		{
			assert(a != b && a != c);
			assert(b != c);

			vertices[0] = a;
			vertices[1] = b;
			vertices[2] = c;
		}

		bool Contains(int v)
		{
			if (vertices[0] == v ||
				vertices[1] == v ||
				vertices[2] == v)
				return true;
			else
				return false;
		}

		void ReplaceEdge(int oldIndex, int newIndex)
		{
			for (int i=0; i < 3; ++i)
			{
				if (edges[i] == oldIndex)
				{
					edges[i] = newIndex;
					return;
				}

			}
			assert(0);
		}

		void ReplaceVertex(int oldIndex, int newIndex)
		{
			for (int i=0; i < 3; ++i)
			{
				if (vertices[i] == oldIndex)
				{
					vertices[i] = newIndex;
					return;
				}
			}

			assert(0);
		}

		int GetOppositeVertex(int v0, int v1)
		{
			for (int i=0; i < 3; ++i)
			{
				if (vertices[i] != v0 && vertices[i] != v1)
					return vertices[i];
			}

			assert(0);
			return -1;
		}

		int vertices[3];
		int edges[3];

		// used during splitting
		int side;
	};

	ClothMesh(const Vec4* vertices, int numVertices,
			  const int* indices, int numIndices,
			  float stretchStiffness,
			  float bendStiffness, bool tearable=true)
	{
		mValid = false;

		mNumVertices = numVertices;

		if (tearable)
		{
			// tearable cloth uses a simple bending constraint model that allows easy splitting of vertices and remapping of constraints
			typedef std::set<Edge> EdgeSet;
			EdgeSet edges;

			// build unique edge list
			for (int i=0; i < numIndices; i += 3)
			{
				mTris.push_back(Triangle(indices[i+0], indices[i+1], indices[i+2]));

				const int triIndex = i/3;

				// breaking the rules
				Edge& e1 = const_cast<Edge&>(*edges.insert(Edge(indices[i+0], indices[i+1])).first);
				Edge& e2 = const_cast<Edge&>(*edges.insert(Edge(indices[i+1], indices[i+2])).first);
				Edge& e3 = const_cast<Edge&>(*edges.insert(Edge(indices[i+2], indices[i+0])).first);

				if (!e1.AddTri(triIndex))
					return;
				if (!e2.AddTri(triIndex))
					return;
				if (!e3.AddTri(triIndex))
					return;
			}

			// flatten set to array
			mEdges.assign(edges.begin(), edges.end());

			// second pass, set edge indices to faces
			for (int i=0; i < numIndices; i += 3)
			{
				int e1 = int(lower_bound(mEdges.begin(), mEdges.end(), Edge(indices[i+0], indices[i+1])) - mEdges.begin());
				int e2 = int(lower_bound(mEdges.begin(), mEdges.end(), Edge(indices[i+1], indices[i+2])) - mEdges.begin());
				int e3 = int(lower_bound(mEdges.begin(), mEdges.end(), Edge(indices[i+2], indices[i+0])) - mEdges.begin());


				if (e1 != e2 && e1 != e3 && e2 != e3)
				{
					const int triIndex = i/3;

					mTris[triIndex].edges[0] = e1;
					mTris[triIndex].edges[1] = e2;
					mTris[triIndex].edges[2] = e3;
				}
				else
				{
					// degenerate tri
					return;
				}
			}

			// generate distance constraints
			for (size_t i=0; i < mEdges.size(); ++i)
			{
				Edge& edge = mEdges[i];

				// stretch constraint along mesh edges
				edge.stretchConstraint = AddConstraint(vertices, edge.vertices[0], edge.vertices[1], stretchStiffness);

				const int t1 = edge.tris[0];
				const int t2 = edge.tris[1];

				// add bending constraint between connected tris
				if (t1 != -1 && t2 != -1 && bendStiffness > 0.0f)
				{
					int v1 = mTris[t1].GetOppositeVertex(edge.vertices[0], edge.vertices[1]);
					int v2 = mTris[t2].GetOppositeVertex(edge.vertices[0], edge.vertices[1]);
					edge.bendingConstraint = AddConstraint(vertices, v1, v2, bendStiffness);
				}
			}
		}
		else
		{

			for (int i=0; i < numIndices; i += 3)
				mTris.push_back(Triangle(indices[i+0], indices[i+1], indices[i+2]));

			// assemble points
			std::vector<Vec4> particles(vertices, vertices+numVertices);

			// build adjacent vertex list
			std::vector<int> valency(mNumVertices+1, 0);
			std::vector<int> adjacencies;
			gatherAdjacencies<int>(valency, adjacencies, indices, numIndices);

			// build unique neighbors from adjacencies
			std::vector<int> mark(valency.size(), 0);
			std::vector<int> neighbors; neighbors.reserve(adjacencies.size());
			for(int i=1, j=0; i< int(valency.size()); ++i)
			{
				for(; j<valency[i]; ++j)
				{
					int k = adjacencies[j];
					if(mark[k] != i)
					{
						mark[k] = i;
						neighbors.push_back(k);
					}
				}
				valency[i] = int(neighbors.size());
			}

			// build map of unique edges and classify
			std::map<Pair, MeshEdge> edges;
			for(int i=0; i<mNumVertices; ++i)
			{
				float wi = particles[i].w;
				// iterate all neighbors
				int jlast = valency[i+1];
				for(int j=valency[i]; j<jlast; ++j)
				{
					// add 1-ring edge
					int m = neighbors[j];
					if(wi + particles[m].w > 0.0f)
						edges[Pair(Min(i, m), Max(i, m))].classify();

					// iterate all neighbors of neighbor
					int klast = valency[m+1];
					for(int k=valency[m]; k<klast; ++k)
					{
						int n = neighbors[k];
						if(n != i && wi + particles[n].w > 0.0f)
						{
							// add 2-ring edge
							edges[Pair(Min(i, n), Max(i, n))].classify(
								particles[i], particles[m], particles[n]);
						}
					}
				}
			}

			// copy classified edges to constraints array
			// build histogram of constraints per vertex
			std::vector<Entry> constraints; 	
			constraints.reserve(edges.size());
			valency.resize(0); valency.resize(mNumVertices+1, 0);

			for(std::map<Pair, MeshEdge>::iterator eIt = edges.begin(); eIt != edges.end(); ++eIt)
			{
				const MeshEdge& edge = eIt->second;
				const Pair& pair = eIt->first;
				if((edge.mStretching + edge.mBending + edge.mShearing) > 0.0f)
				{	
					float stiffness = 0.0f;

					if(edge.mBending > Max(edge.mStretching, edge.mShearing))
					{
						stiffness = bendStiffness;
					}
					else if(edge.mShearing > Max(edge.mStretching, edge.mBending))
					{
						stiffness = stretchStiffness;
					}
					else 
					{
						stiffness = stretchStiffness;						
					}

					++valency[pair.first];
					++valency[pair.second];
					constraints.push_back(Entry(pair, 0));

					if (stiffness > 0.0f)
						AddConstraint(vertices, pair.first, pair.second, stiffness);
				}
			} 
		}

		// calculate rest volume
		mRestVolume = 0.0f;
		mConstraintScale = 0.0f;

		std::vector<Vec3> gradients(numVertices);

		for (int i=0; i < numIndices; i+=3)
		{
			Vec3 v1 = Vec3(vertices[indices[i+0]]);
			Vec3 v2 = Vec3(vertices[indices[i+1]]);
			Vec3 v3 = Vec3(vertices[indices[i+2]]);

			const Vec3 n = Cross(v2-v1, v3-v1);
			const float signedVolume = Dot(v1, n);

			mRestVolume += signedVolume;

			gradients[indices[i+0]] += n;
			gradients[indices[i+1]] += n;
			gradients[indices[i+2]] += n;
		}
		
		for (int i=0; i < numVertices; ++i)
			mConstraintScale += Dot(gradients[i], gradients[i]);

		mConstraintScale = 1.0f/mConstraintScale;

		mValid = true;

	}

	int AddConstraint(const Vec4* vertices, int a, int b, float stiffness, float give=0.0f)
	{
		int index = int(mConstraintRestLengths.size());

		mConstraintIndices.push_back(a);
		mConstraintIndices.push_back(b);

		const float restLength = Length(Vec3(vertices[a])-Vec3(vertices[b]));
			
		mConstraintRestLengths.push_back(restLength*(1.0f + give));
		mConstraintCoefficients.push_back(stiffness);

		return index;
	}

	bool SplitVertex(const Vec4* vertices, int newIndex, int index, Vec3 splitPlane)
	{
		std::vector<int> adjacentTris;

		float w = Dot(vertices[index], splitPlane);

		int leftCount = 0;
		int rightCount = 0;

		// classify all tris attached to the split vertex according 
		// to which side of the split plane their centroid lies on O(N)
		for (size_t i=0; i < mTris.size(); ++i)
		{
			Triangle& tri = mTris[i];

			if (tri.Contains(index))
			{
				const Vec4 centroid = (vertices[tri.vertices[0]] + vertices[tri.vertices[1]] + vertices[tri.vertices[2]])/3.0f;

				if (Dot(Vec3(centroid), splitPlane) < w)
				{
					tri.side = 1;

					++leftCount;
				}
				else
				{
					tri.side = 0;

					++rightCount;
				}

				adjacentTris.push_back(int(i));
			}
		}

		// if all tris on one side of split plane then do nothing
		if (leftCount == 0 || rightCount == 0)
			return false;

		// remap triangle indices
		for (size_t i=0; i < adjacentTris.size(); ++i)
		{
			const int triIndex = adjacentTris[i];

			Triangle& tri = mTris[triIndex];

			// tris on the negative side of the split plane are assigned the new index
			if (tri.side == 0)
			{
				tri.ReplaceVertex(index, newIndex);

				// update edges and constraints
				for (int e=0; e < 3; ++e)
				{
					Edge& edge = mEdges[tri.edges[e]];

					if (edge.Contains(index))
					{
						bool exposed = false;

						if (edge.tris[0] != -1 && edge.tris[1] != -1)
						{
							Triangle& t1 = mTris[edge.tris[0]];
							Triangle& t2 = mTris[edge.tris[1]];

							// Case 1: connected tris lie on opposite sides of the split plane
							// creating a new exposed edge, need to break bending constraint
							// and create new stretch constraint for exposed edge
							if (t1.side != t2.side)
							{
								exposed = true;

								// create new edge
								Edge newEdge(edge.vertices[0], edge.vertices[1]);
								newEdge.Replace(index, newIndex);
								newEdge.AddTri(triIndex);
								
								// remove neighbor from old edge
								edge.RemoveTri(triIndex);
					
								// replace bending constraint with stretch constraint
								assert(edge.bendingConstraint != -1);

								newEdge.stretchConstraint = edge.bendingConstraint;

								mConstraintIndices[newEdge.stretchConstraint*2+0] = newEdge.vertices[0];
								mConstraintIndices[newEdge.stretchConstraint*2+1] = newEdge.vertices[1];
								mConstraintCoefficients[newEdge.stretchConstraint] = mConstraintCoefficients[edge.stretchConstraint];
								mConstraintRestLengths[newEdge.stretchConstraint] = mConstraintRestLengths[edge.stretchConstraint];

								edge.bendingConstraint = -1;

								// don't access Edge& after this 
								tri.ReplaceEdge(tri.edges[e], int(mEdges.size()));
								mEdges.push_back(newEdge);
							}
						}

						if (!exposed)
						{
							// Case 2: both tris on same side of split plane or boundary edge, simply remap edge and constraint
							// may have processed this edge already so check that it still containts old vertex
							edge.Replace(index, newIndex);

							const int stretching = edge.stretchConstraint;
							if (mConstraintIndices[stretching*2+0] == index)
								mConstraintIndices[stretching*2+0] = newIndex;
							else if (mConstraintIndices[stretching*2+1] == index)
								mConstraintIndices[stretching*2+1] = newIndex;
							else
								assert(0);
						}

						//tri.edges[e].Replace(
					}
					else
					{
						// Case 3: tri is adjacent to split vertex but this edge is not connected to it
						// therefore there can be a bending constraint crossing this edge connected 
						// to vertex that needs to be remapped
						const int bending = edge.bendingConstraint;
						if (bending != -1)
						{
							if (mConstraintIndices[bending*2+0] == index)
								mConstraintIndices[bending*2+0] = newIndex;
							else if (mConstraintIndices[bending*2+1] == index)
								mConstraintIndices[bending*2+1] = newIndex;
						}
					}
				}
				
			}			
		}

		return true;
	}

	std::vector<int> mConstraintIndices;
	std::vector<float> mConstraintCoefficients;
	std::vector<float> mConstraintRestLengths;

	std::vector<Edge> mEdges;
	std::vector<Triangle> mTris;
	int mNumVertices;

	float mRestVolume;
	float mConstraintScale;

	bool mValid;
};
