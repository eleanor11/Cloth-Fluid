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
#include "shaders.h"

#include "core/shader.h"
#include "core/mesh.h"
#include "core/tga.h"

#define cudaCheck(x) { cudaError_t err = x; if (err != cudaSuccess) { printf("Cuda error: %d in %s at %s:%d\n", err, #x, __FILE__, __LINE__); assert(0); } }

static float gSpotMin = 0.95f;
static float gSpotMax = 1.0f;
float gShadowBias = 0.075f;

extern GLuint g_msaaFbo;

// fixes some banding artifacts with repeated blending during thickness and diffuse rendering
#define USE_HDR_DIFFUSE_BLEND 0

Colour gColors[] = 
{
	Colour(0.0f, 0.5f, 1.0f),
	Colour(0.797f, 0.354f, 0.000f),
	Colour(0.092f, 0.465f, 0.820f),
	Colour(0.000f, 0.349f, 0.173f),
	Colour(0.875f, 0.782f, 0.051f),
	Colour(0.000f, 0.170f, 0.453f),
	Colour(0.673f, 0.111f, 0.000f),
	Colour(0.612f, 0.194f, 0.394f) 
};

// vertex shader
const char *vertexPointShader = "#version 130\n"STRINGIFY(

uniform float pointRadius;  // point size in world space
uniform float pointScale;   // scale to calculate size in pixels

uniform mat4 lightTransform; 
uniform vec3 lightDir;
uniform vec3 lightDirView;

uniform vec4 colors[8];

uniform vec4 transmission;
uniform int mode;

//in int density;
in float density;
in int phase;
in vec4 velocity;

void main()
{
    // calculate window-space point size
	vec4 viewPos = gl_ModelViewMatrix*vec4(gl_Vertex.xyz, 1.0);

	gl_Position = gl_ModelViewProjectionMatrix * vec4(gl_Vertex.xyz, 1.0);
	gl_PointSize = -pointScale * (pointRadius / viewPos.z);

	gl_TexCoord[0] = gl_MultiTexCoord0;    
	gl_TexCoord[1] = lightTransform*vec4(gl_Vertex.xyz-lightDir*pointRadius*2.0, 1.0);
	gl_TexCoord[2] = gl_ModelViewMatrix*vec4(lightDir, 0.0);

	if (mode == 1)
	{
		// density visualization
		if (density < 0.0f)
			gl_TexCoord[3].xyz = mix(vec3(0.1, 0.1, 1.0), vec3(0.1, 1.0, 1.0), -density);
		else
			gl_TexCoord[3].xyz = mix(vec3(1.0, 1.0, 1.0), vec3(0.1, 0.2, 1.0), density);
	}
	else if (mode == 2)
	{
		gl_PointSize *= clamp(gl_Vertex.w*0.25, 0.0f, 1.0);

		gl_TexCoord[3].xyzw = vec4(clamp(gl_Vertex.w*0.05, 0.0f, 1.0));
	}
	else
	{
		gl_TexCoord[3].xyz = mix(colors[phase%8].xyz*2.0, vec3(1.0), 0.1);	
	}

	// mass visualization (shock propagation)
	//gl_TexCoord[3].xyz = mix(vec3(0.873, 0.111, 0.000), vec3(0.0, 0.5, 1.0), gl_Vertex.y/3.0f);

	gl_TexCoord[4].xyz = gl_Vertex.xyz;
	gl_TexCoord[5].xyz = viewPos.xyz;
}
);

// pixel shader for rendering points as shaded spheres
const char *fragmentPointShader = STRINGIFY(

uniform vec3 lightDir;
uniform vec3 lightPos;
uniform float spotMin;
uniform float spotMax;
uniform int mode;

uniform sampler2DShadow shadowTex;
uniform vec2 shadowTaps[12];
uniform float pointRadius;  // point size in world space

// sample shadow map
float shadowSample()
{
	vec3 pos = vec3(gl_TexCoord[1].xyz/gl_TexCoord[1].w);
	vec3 uvw = (pos.xyz*0.5)+vec3(0.5);

	// user clip
	if (uvw.x  < 0.0 || uvw.x > 1.0)
		return 0.0;
	if (uvw.y < 0.0 || uvw.y > 1.0)
		return 0.0;
	
	float s = 0.0;
	float radius = 0.002;

	for (int i=0; i < 8; i++)
	{
		s += shadow2D(shadowTex, vec3(uvw.xy + shadowTaps[i]*radius, uvw.z)).r;
	}

	s /= 8.0;
	return s;
}

float sqr(float x) { return x*x; }

void main()
{
    // calculate normal from texture coordinates
    vec3 normal;
    normal.xy = gl_TexCoord[0].xy*vec2(2.0, -2.0) + vec2(-1.0, 1.0);
    float mag = dot(normal.xy, normal.xy);
    if (mag > 1.0) discard;   // kill pixels outside circle
   	normal.z = sqrt(1.0-mag);

	if (mode == 2)
	{
		float alpha  = normal.z*gl_TexCoord[3].w;
		gl_FragColor.xyz = gl_TexCoord[3].xyz*alpha;
		gl_FragColor.w = alpha;
		return;
	}

    // calculate lighting
	float shadow = shadowSample();
	
	vec3 lVec = normalize(gl_TexCoord[4].xyz-(lightPos));
	//float attenuation = 1000.0f/dot(lVec, lVec);
	float attenuation = max(smoothstep(spotMin, spotMax, dot(lVec, lightDir)), 0.025);

	vec3 ambient = vec3(0., 0., 0.);
	vec3 diffuse = vec3(0.9, 0.9, 0.9);
	vec3 reflectance =  gl_TexCoord[3].xyz;
	vec3 h = normalize(-gl_TexCoord[2].xyz + gl_ModelViewMatrixInverse[2].xyz);
	
	vec3 Lo = diffuse*reflectance*max(0.0, sqr(-dot(gl_TexCoord[2].xyz, normal)*0.5 + 0.5))*max(0.2,shadow)*attenuation;

	gl_FragColor = vec4(pow(Lo, vec3(1.0/2.2)), 1.0);

	vec3 eyePos = gl_TexCoord[5].xyz + normal*pointRadius;//*2.0;
	vec4 ndcPos = gl_ProjectionMatrix * vec4(eyePos, 1.0);
	ndcPos.z /= ndcPos.w;
	gl_FragDepth = ndcPos.z*0.5 + 0.5;

//	gl_FragColor = vec4(diffuse + ambient, 1.0);
//	gl_FragColor = vec4(N.x, N.y, N.z, 1.0);
//	gl_FragColor = vec4(dFdx(pos.x), dFdx(pos.y), 0.0, 1.0);
//	gl_FragColor.xyz = (gl_TexCoord[1].xyz/gl_TexCoord[1].w)*0.5 + 0.5;
//	gl_FragColor.w = 1.0;
//	gl_FragColor.z = 0.0;
}


);

// vertex shader
const char *vertexShader = "#version 130\n"STRINGIFY(

uniform mat4 lightTransform; 
uniform vec3 lightDir;
uniform float bias;
uniform vec4 clipPlane;
uniform float expand;


void main()
{
	vec3 n = normalize(gl_Normal);

    // calculate window-space point size
	gl_Position = gl_ModelViewProjectionMatrix * vec4(gl_Vertex.xyz + expand*n, 1.0);

	gl_TexCoord[0].xyz = n;
	gl_TexCoord[1] = lightTransform*vec4(gl_Vertex.xyz + n*bias, 1.0);
	gl_TexCoord[2] = gl_ModelViewMatrix*vec4(lightDir, 0.0);
	gl_TexCoord[3].xyz = gl_Vertex.xyz;	
	gl_TexCoord[4] = gl_Color;
	gl_TexCoord[5] = gl_MultiTexCoord0;
	gl_TexCoord[6] = gl_SecondaryColor;
	gl_TexCoord[7] = gl_ModelViewMatrix*vec4(gl_Vertex.xyz, 1.0);

	gl_ClipDistance[0] = dot(clipPlane,vec4(gl_Vertex.xyz, 1.0));
}
);

// pixel shader for rendering points as shaded spheres
const char *fragmentShader = STRINGIFY(

uniform vec3 lightDir;
uniform vec3 lightPos;
uniform float spotMin;
uniform float spotMax;
uniform vec3 color;
uniform vec4 fogColor;

uniform sampler2DShadow shadowTex;
uniform vec2 shadowTaps[12];

uniform sampler2D tex;
uniform bool sky;

uniform bool grid;
uniform bool texture;

float sqr(float x) { return x*x; }


// sample shadow map
float shadowSample()
{
	vec3 pos = vec3(gl_TexCoord[1].xyz/gl_TexCoord[1].w);
	vec3 uvw = (pos.xyz*0.5)+vec3(0.5);

	// user clip
	if (uvw.x  < 0.0 || uvw.x > 1.0)
		return 1.0;
	if (uvw.y < 0.0 || uvw.y > 1.0)
		return 1.0;
	
	float s = 0.0;
	float radius = 0.002;

	const int numTaps = 12;

	for (int i=0; i < numTaps; i++)
	{
		s += shadow2D(shadowTex, vec3(uvw.xy + shadowTaps[i]*radius, uvw.z)).r;
		//s += texture2D(shadowTex, vec3(uvw.xy + shadowTaps[i]*radius, uvw.xy)).r > uvw.z;
	}

	s /= numTaps;
	return s;
}

float filterwidth(float2 v)
{
  float2 fw = max(abs(ddx(v)), abs(ddy(v)));
  return max(fw.x, fw.y);
}

float2 bump(float2 x) 
{
	return (floor((x)/2) + 2.f * max(((x)/2) - floor((x)/2) - .5f, 0.f)); 
}

float checker(float2 uv)
{
  float width = filterwidth(uv);
  float2 p0 = uv - 0.5 * width;
  float2 p1 = uv + 0.5 * width;
  
  float2 i = (bump(p1) - bump(p0)) / width;
  return i.x * i.y + (1 - i.x) * (1 - i.y);
}

void main()
{
    // calculate lighting
	float shadow = max(shadowSample(), 0.5);

	vec3 lVec = normalize(gl_TexCoord[3].xyz-(lightPos));
	float attenuation = max(smoothstep(spotMin, spotMax, dot(lVec, lightDir)), 0.05);
		
	vec3 n = gl_TexCoord[0].xyz;
	vec3 color = gl_TexCoord[4].xyz;
	
	if (!gl_FrontFacing)
	{
		color = gl_TexCoord[6].xyz;
		n *= -1.0f;
	}	
	

	if (grid && (n.y >0.995))
	{
		/*
		int grid = (int(floor(gl_TexCoord[3].x)) + int(floor(gl_TexCoord[3].y)) + int(floor(gl_TexCoord[3].z)))&1;
		if (grid)
			color *= 0.9;
			*/
		color *= 1.0 - 0.25*checker(float2(gl_TexCoord[3].x,  gl_TexCoord[3].z));
	}
	else if (grid && abs(n.z) > 0.995)
	{
		color *= 1.0 - 0.25*checker(float2(gl_TexCoord[3].y,  gl_TexCoord[3].x));
	}
	
	//vec3 marble = texture2D(marbleTex, 	gl_TexCoord[5].xy*200.0);
	//color *= marble;

	if (texture)
	{
		/*
		float edgeFactor = 1.0-gl_TexCoord[5].z;

		if (edgeFactor < 1.0)
		{
			//gl_FragColor = gl_TexCoord[5];
			color = texture2D(tex, gl_TexCoord[5].xy).xyz*color*edgeFactor;
		
			if (color.x < 0.2)
				discard;
		}*/
		color = texture2D(tex, gl_TexCoord[5].xy).xyz;
	}	
 
	
	// direct light term
	float wrap = 0.0;
	vec3 diffuse = color*vec3(1.0, 1.0, 1.0)*max(0.0, (-dot(lightDir, n)+wrap)/(1.0+wrap)*shadow)*attenuation;
	
	// wrap ambient term aligned with light dir
	vec3 light = vec3(0.03, 0.025, 0.025)*1.5;
	vec3 dark = vec3(0.025, 0.025, 0.03);
	vec3 ambient = 4.0*color*mix(dark, light, -dot(lightDir, n)*0.5 + 0.5)*attenuation;

	/*
	vec3 eye = normalize(gl_ModelViewMatrixInverse[3].xyz-gl_TexCoord[3].xyz);
	vec3 h = normalize(eye - lVec);
	vec3 specular = pow(dot(h, n), 60.0);
	*/

	vec3 fog = lerp(vec3(fogColor), diffuse + ambient, exp(gl_TexCoord[7].z*fogColor.w));

	gl_FragColor = vec4(pow(fog, vec3(1.0/2.2)), 1.0);				
});

void ShadowApply(GLint sprogram, Vec3 lightPos, Vec3 lightTarget, Matrix44 lightTransform, GLuint shadowTex)
{
	GLint uLightTransform = glGetUniformLocation(sprogram, "lightTransform");
	glUniformMatrix4fv(uLightTransform, 1, false, lightTransform);

	GLint uLightPos = glGetUniformLocation(sprogram, "lightPos");
	glUniform3fv(uLightPos, 1, lightPos);
	
	GLint uLightDir = glGetUniformLocation(sprogram, "lightDir");
	glUniform3fv(uLightDir, 1, Normalize(lightTarget-lightPos));

	GLint uBias = glGetUniformLocation(sprogram, "bias");
	glUniform1f(uBias, gShadowBias);

	const Vec2 taps[] = 
	{ 
		Vec2(-0.326212f,-0.40581f),Vec2(-0.840144f,-0.07358f),
		Vec2(-0.695914f,0.457137f),Vec2(-0.203345f,0.620716f),
		Vec2(0.96234f,-0.194983f),Vec2(0.473434f,-0.480026f),
		Vec2(0.519456f,0.767022f),Vec2(0.185461f,-0.893124f),
		Vec2(0.507431f,0.064425f),Vec2(0.89642f,0.412458f),
		Vec2(-0.32194f,-0.932615f),Vec2(-0.791559f,-0.59771f) 
	};
	
	GLint uShadowTaps = glGetUniformLocation(sprogram, "shadowTaps");
	glUniform2fv(uShadowTaps, 12, &taps[0].x);
	
	glEnable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, shadowTex);

}

void DrawPoints(GLuint positions, GLuint colors, int n, int offset, float radius, float screenWidth, float screenAspect, float fov, Vec3 lightPos, Vec3 lightTarget, Matrix44 lightTransform, GLuint shadowTex, bool showDensity, bool msaa)
{
	static int sprogram = -1;
	if (sprogram == -1)
	{
		sprogram = CompileProgram(vertexPointShader, fragmentPointShader);
	}

	if (sprogram)
	{
		glEnable(GL_POINT_SPRITE);
		glTexEnvi(GL_POINT_SPRITE, GL_COORD_REPLACE, GL_TRUE);
		glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
		//glDepthMask(GL_TRUE);
		glEnable(GL_DEPTH_TEST);

		// for some reason points in msaa buffers are half the size..
		float msaaScale = msaa?2.0f:1.0f;

		int mode = 0;
		if (showDensity)
			mode = 1;
		if (shadowTex == 0)
			mode = 2;		

		glVerify(glUseProgram(sprogram));
		glVerify(glUniform1f( glGetUniformLocation(sprogram, "pointRadius"), radius));
		glVerify(glUniform1f( glGetUniformLocation(sprogram, "pointScale"), msaaScale*screenWidth/screenAspect * (1.0f / (tanf(fov*0.5f)))));
		glVerify(glUniform1f( glGetUniformLocation(sprogram, "spotMin"), gSpotMin));
		glVerify(glUniform1f( glGetUniformLocation(sprogram, "spotMax"), gSpotMax));
		glVerify(glUniform1i( glGetUniformLocation(sprogram, "mode"), mode));
		glVerify(glUniform4fv( glGetUniformLocation(sprogram, "colors"), 8, (float*)&gColors[0].r));

		// set shadow parameters
		ShadowApply(sprogram, lightPos, lightTarget, lightTransform, shadowTex);

		glEnableClientState(GL_VERTEX_ARRAY);
		glBindBuffer(GL_ARRAY_BUFFER, positions);
		glVertexPointer(4, GL_FLOAT, 0, 0);

		int d = glGetAttribLocation(sprogram, "density");
		assert(d != -1);

		int p = glGetAttribLocation(sprogram, "phase");
		assert(p != -1);

		//int v = glGetAttribLocation(sprogram, "velocity");
		//assert(v != -1);

		glVerify(glEnableVertexAttribArray(d));
		glVerify(glBindBuffer(GL_ARRAY_BUFFER, colors));

		glVerify(glEnableVertexAttribArray(p));
		glVerify(glBindBuffer(GL_ARRAY_BUFFER, colors));

		//glVerify(glEnableVertexAttribArray(v));
		//glVerify(glBindBuffer(GL_ARRAY_BUFFER, colors));


		// only one array will be used, and is interpreted based on the visualization mode
		glVerify(glVertexAttribPointer(d, 1,  GL_FLOAT, GL_FALSE, 0, 0));	// densities
		glVerify(glVertexAttribIPointer(p, 1,  GL_INT, 0, 0));			// phases
//		glVerify(glVertexAttribPointer(v, 4,  GL_FLOAT, GL_FALSE, 0, 0));			// phases
		
		glVerify(glDrawArrays(GL_POINTS, offset, n));

		glVerify(glUseProgram(0));
		glVerify(glBindBuffer(GL_ARRAY_BUFFER, 0));
		glVerify(glDisableClientState(GL_VERTEX_ARRAY));	
		
		glVerify(glDisableVertexAttribArray(d));
		glVerify(glDisableVertexAttribArray(p));
		
		glDisable(GL_POINT_SPRITE);
		glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);		
	}
}
void DrawPlane(const Vec4& p);

static GLuint s_diffuseProgram = GLuint(-1);

void BindSolidShader(Vec3 lightPos, Vec3 lightTarget, Matrix44 lightTransform, GLuint shadowTex, float bias, Vec4 fogColor)
{

	if (s_diffuseProgram == GLuint(-1))
		s_diffuseProgram = CompileProgram(vertexShader, fragmentShader);


	if (s_diffuseProgram)
	{
		glDepthMask(GL_TRUE);
		glEnable(GL_DEPTH_TEST);		

		glUseProgram(s_diffuseProgram);
		glUniform1i(glGetUniformLocation(s_diffuseProgram, "grid"), 0);
		glUniform1f( glGetUniformLocation(s_diffuseProgram, "spotMin"), gSpotMin);
		glUniform1f( glGetUniformLocation(s_diffuseProgram, "spotMax"), gSpotMax);
		glUniform4fv( glGetUniformLocation(s_diffuseProgram, "fogColor"), 1, fogColor);

		// set shadow parameters
		ShadowApply(s_diffuseProgram, lightPos, lightTarget, lightTransform, shadowTex);
	}
}
void UnbindSolidShader()
{
	glActiveTexture(GL_TEXTURE1);
	glDisable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE0);

	glUseProgram(0);
}


void DrawPlanes(Vec4* planes, int n, float bias)
{
	// diffuse 		
	glColor3f(0.9f, 0.9f, 0.9f);

	GLint uBias = glGetUniformLocation(s_diffuseProgram, "bias");
	glUniform1f(uBias, 0.0f);
	GLint uGrid = glGetUniformLocation(s_diffuseProgram, "grid");
	glUniform1i(uGrid, 1);
	GLint uExpand = glGetUniformLocation(s_diffuseProgram, "expand");
	glUniform1f(uExpand, 0.0f);

	for (int i=0; i < n; ++i)
	{
		Vec4 p = planes[i];
		p.w -= bias;

		DrawPlane(p, false);
	}

	glUniform1i(uGrid, 0);
	glUniform1f(uBias, gShadowBias);

}


void DrawMesh(const Mesh* m, Vec3 color)
{ 
	if (m)
	{
		glVerify(glColor3fv(color));

		glVerify(glBindBuffer(GL_ARRAY_BUFFER, 0));
		glVerify(glEnableClientState(GL_NORMAL_ARRAY));
		glVerify(glEnableClientState(GL_VERTEX_ARRAY));

		glVerify(glNormalPointer(GL_FLOAT, sizeof(float)*3, &m->m_normals[0]));
		glVerify(glVertexPointer(3, GL_FLOAT, sizeof(float)*3, &m->m_positions[0]));		
		
		if (m->m_colours.size())
		{
			glVerify(glEnableClientState(GL_COLOR_ARRAY));
			glVerify(glColorPointer(4, GL_FLOAT, 0, &m->m_colours[0]));
		}

		glVerify(glDrawElements(GL_TRIANGLES, m->GetNumFaces()*3, GL_UNSIGNED_INT, &m->m_indices[0]));

		glVerify(glDisableClientState(GL_VERTEX_ARRAY));
		glVerify(glDisableClientState(GL_NORMAL_ARRAY));
		
		if (m->m_colours.size())
			glVerify(glDisableClientState(GL_COLOR_ARRAY));
	}
}


void DrawSky()
{
	static Mesh* s_sky;
	if (!s_sky)
	{
		s_sky = ImportMeshFromPly("data/sphere.ply");
		s_sky->Transform(ScaleMatrix(1000.0f));
	}

	glUniform1i(glGetUniformLocation(s_diffuseProgram, "sky"), 1);

	glDisable(GL_CULL_FACE);
	DrawMesh(s_sky, Vec3(0.0f));
	glEnable(GL_CULL_FACE);

	glUniform1i(glGetUniformLocation(s_diffuseProgram, "sky"), 0);
	
}

void DrawCloth(const Vec4* positions, const Vec4* normals, const float* uvs, const int* indices, int numTris, int numPositions, int colorIndex, float expand, bool twosided, bool smooth)
{ 
	if (!numTris)
		return;

	if (twosided)
		glDisable(GL_CULL_FACE);

	GLint program;
	glGetIntegerv(GL_CURRENT_PROGRAM, &program);

	if (program == GLint(s_diffuseProgram))
	{
		GLint uBias = glGetUniformLocation(s_diffuseProgram, "bias");
		glUniform1f(uBias, 0.0f);

		GLint uExpand = glGetUniformLocation(s_diffuseProgram, "expand");
		glUniform1f(uExpand, expand);
	}

	glColor3fv(gColors[colorIndex+1]*1.5f);
	glSecondaryColor3fv(gColors[colorIndex]*1.5f);

	glVerify(glEnableClientState(GL_VERTEX_ARRAY));
	glVerify(glEnableClientState(GL_NORMAL_ARRAY));

	glVerify(glVertexPointer(3, GL_FLOAT, sizeof(float)*4, positions));
	glVerify(glNormalPointer(GL_FLOAT, sizeof(float)*4, normals));
	
	glVerify(glDrawElements(GL_TRIANGLES, numTris*3, GL_UNSIGNED_INT, indices));

	glVerify(glDisableClientState(GL_VERTEX_ARRAY));
	glVerify(glDisableClientState(GL_NORMAL_ARRAY));

	if (twosided)
		glEnable(GL_CULL_FACE);

	if (program == GLint(s_diffuseProgram))
	{
		GLint uBias = glGetUniformLocation(s_diffuseProgram, "bias");
		glUniform1f(uBias, gShadowBias);

		GLint uExpand = glGetUniformLocation(s_diffuseProgram, "expand");
		glUniform1f(uExpand, 0.0f);
	}
}
void DrawClothColor(const Vec4* positions, const Vec4* colors, const Vec4* normals, const float* uvs, const int* indices, int numTris, int numPositions, int colorIndex, float expand, bool twosided, bool smooth)
{
	if (!numTris)
		return;

	if (twosided)
		glDisable(GL_CULL_FACE);

	GLint program;
	glGetIntegerv(GL_CURRENT_PROGRAM, &program);

	if (program == GLint(s_diffuseProgram))
	{
		GLint uBias = glGetUniformLocation(s_diffuseProgram, "bias");
		glUniform1f(uBias, 0.0f);

		GLint uExpand = glGetUniformLocation(s_diffuseProgram, "expand");
		glUniform1f(uExpand, expand);
	}

	//glColor3fv(gColors[colorIndex+1]*1.5f);
	//glColor3fv(Colour(1.0f, 1.0f, 1.0f));
	//glSecondaryColor3fv(gColors[colorIndex]*1.5f);
	//glSecondaryColor3fv(Colour(1.0f, 1.0f, 0.3f));

	glVerify(glEnableClientState(GL_VERTEX_ARRAY));
	glVerify(glEnableClientState(GL_NORMAL_ARRAY));
	glVerify(glEnableClientState(GL_COLOR_ARRAY));
	glVerify(glEnableClientState(GL_SECONDARY_COLOR_ARRAY));

	glVerify(glVertexPointer(3, GL_FLOAT, sizeof(float) * 4, positions));
	glVerify(glNormalPointer(GL_FLOAT, sizeof(float) * 4, normals));
	glVerify(glColorPointer(3, GL_FLOAT, sizeof(float) * 4, colors));
	glVerify(glSecondaryColorPointer(3, GL_FLOAT, sizeof(float) * 4, colors));

	glVerify(glDrawElements(GL_TRIANGLES, numTris * 3, GL_UNSIGNED_INT, indices));

	glVerify(glDisableClientState(GL_VERTEX_ARRAY));
	glVerify(glDisableClientState(GL_NORMAL_ARRAY));
	glVerify(glDisableClientState(GL_COLOR_ARRAY));
	glVerify(glDisableClientState(GL_SECONDARY_COLOR_ARRAY));

	if (twosided)
		glEnable(GL_CULL_FACE);

	if (program == GLint(s_diffuseProgram))
	{
		GLint uBias = glGetUniformLocation(s_diffuseProgram, "bias");
		glUniform1f(uBias, gShadowBias);

		GLint uExpand = glGetUniformLocation(s_diffuseProgram, "expand");
		glUniform1f(uExpand, 0.0f);
	}
}

void DrawRope(Vec4* positions, int* indices, int numIndices, float radius, int color)
{
	if (numIndices < 2)
		return;

	std::vector<Vec3> vertices;
	std::vector<Vec3> normals;
	std::vector<int> triangles;
	
	// flatten curve
	std::vector<Vec3> curve(numIndices);
	for (int i=0; i < numIndices; ++i)
		curve[i] = Vec3(positions[indices[i]]);

	const int resolution = 8;
	const int smoothing = 3;

	vertices.reserve(resolution*numIndices*smoothing);
	normals.reserve(resolution*numIndices*smoothing);
	triangles.reserve(numIndices*resolution*6*smoothing);

	Extrude(&curve[0], int(curve.size()), vertices, normals, triangles, radius, resolution, smoothing);
		
	glVerify(glDisable(GL_CULL_FACE));
	glVerify(glColor3fv(gColors[color%8]*1.5f));
	glVerify(glSecondaryColor3fv(gColors[color%8]*1.5f));

	glVerify(glEnableClientState(GL_VERTEX_ARRAY));
	glVerify(glEnableClientState(GL_NORMAL_ARRAY));

	glVerify(glVertexPointer(3, GL_FLOAT, sizeof(float)*3, &vertices[0]));
	glVerify(glNormalPointer(GL_FLOAT, sizeof(float)*3, &normals[0]));

	glVerify(glDrawElements(GL_TRIANGLES, GLsizei(triangles.size()), GL_UNSIGNED_INT, &triangles[0]));

	glVerify(glDisableClientState(GL_VERTEX_ARRAY));
	glVerify(glDisableClientState(GL_NORMAL_ARRAY));
	glVerify(glEnable(GL_CULL_FACE));

}

static const int kShadowResolution = 2048;

void ShadowCreate(GLuint& texture, GLuint& frameBuffer)
{
	glVerify(glGenFramebuffers(1, &frameBuffer));
	glVerify(glGenTextures(1, &texture));
	glVerify(glBindTexture(GL_TEXTURE_2D, texture));

	glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)); 
	glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)); 
	 
	// This is to allow usage of shadow2DProj function in the shader 
	glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE)); 
	glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL)); 
	glVerify(glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_INTENSITY)); 

	glVerify(glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, kShadowResolution, kShadowResolution, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, NULL));

	glVerify(glBindFramebuffer(GL_FRAMEBUFFER, frameBuffer));

	glVerify(glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, texture, 0));

}

void ShadowBegin(GLuint texture, GLuint frameBuffer)
{
	glVerify(glBindFramebuffer(GL_FRAMEBUFFER, frameBuffer));

	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClear(GL_DEPTH_BUFFER_BIT);
	glViewport(0, 0, kShadowResolution, kShadowResolution);

	// draw back faces (for teapot)
	glDisable(GL_CULL_FACE);
}

void ShadowEnd()
{
	glVerify(glBindFramebuffer(GL_FRAMEBUFFER, g_msaaFbo));

	glEnable(GL_CULL_FACE);
}

void ReflectCreate(GLuint& texture, int width, int height)
{
	// copy frame buffer to texture
	glVerify(glActiveTexture(GL_TEXTURE0));
	glVerify(glEnable(GL_TEXTURE_2D));

	glVerify(glGenTextures(1, &texture));
	glVerify(glBindTexture(GL_TEXTURE_2D, texture));

	glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)); 
	glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)); 	 
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
		
	glVerify(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL));
}

void ReflectDestroy(GLuint texture)
{
	glVerify(glDeleteTextures(1, &texture));
}


void ReflectBegin(Vec4 plane, int width, int height)
{
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glViewport(0, 0, width, height);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	Matrix44 scale = Matrix44::kIdentity;
	scale.columns[0][0] *= -2.0f;
	scale.columns[1][1] *= -2.0f;
	scale.columns[2][2] *= -2.0f;
	scale.columns[3][3] *= -2.0f;

	Matrix44 reflect = (scale*Outer(Vec4(plane.x, plane.y, plane.z, 0.0f), plane));
	reflect.columns[0][0] += 1.0f;
	reflect.columns[1][1] += 1.0f;
	reflect.columns[2][2] += 1.0f;
	reflect.columns[3][3] += 1.0f;

	glMultMatrixf(reflect);

	glVerify(glFrontFace(GL_CW));
	glVerify(glEnable(GL_CLIP_PLANE0));

	glVerify(glUniform4fv( glGetUniformLocation(s_diffuseProgram, "clipPlane"), 1, plane));
}

void ReflectEnd(GLuint texture, int width, int height)
{
	// copy frame buffer to texture
	glVerify(glActiveTexture(GL_TEXTURE0));
	glVerify(glEnable(GL_TEXTURE_2D));
	glVerify(glBindTexture(GL_TEXTURE_2D, texture));

	glVerify(glCopyTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 0, 0, width, height));
	
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	glVerify(glDisable(GL_CLIP_PLANE0));
	glVerify(glFrontFace(GL_CCW));

	glBindFramebuffer(GL_FRAMEBUFFER, g_msaaFbo);
}


//-----------------------------------------------------------------------------------------------------
// vertex shader

const char *vertexPointDepthShader = STRINGIFY(

uniform float pointRadius;  // point size in world space
uniform float pointScale;   // scale to calculate size in pixels

void main()
{
    // calculate window-space point size
	gl_Position = gl_ModelViewProjectionMatrix * vec4(gl_Vertex.xyz, 1.0);
	gl_PointSize = pointScale * (pointRadius / gl_Position.w);

	gl_TexCoord[0] = gl_MultiTexCoord0;    
	gl_TexCoord[1] = gl_ModelViewMatrix * vec4(gl_Vertex.xyz, 1.0);
}
);

// pixel shader for rendering points as shaded spheres
const char *fragmentPointDepthShader = STRINGIFY(

uniform float pointRadius;  // point size in world space

void main()
{
    // calculate normal from texture coordinates
    vec3 normal;
    normal.xy = gl_TexCoord[0].xy*vec2(2.0, -2.0) + vec2(-1.0, 1.0);
    float mag = dot(normal.xy, normal.xy);
    if (mag > 1.0) discard;   // kill pixels outside circle
   	normal.z = sqrt(1.0-mag);

	vec3 eyePos = gl_TexCoord[1].xyz + normal*pointRadius*2.0;
	vec4 ndcPos = gl_ProjectionMatrix * vec4(eyePos, 1.0);
	ndcPos.z /= ndcPos.w;

	gl_FragColor = vec4(eyePos.z, 1.0, 1.0, 1.0);
	gl_FragDepth = ndcPos.z*0.5 + 0.5;
}
);


// pixel shader for rendering points density
const char *fragmentPointThicknessShader = STRINGIFY(

void main()
{
    // calculate normal from texture coordinates
    vec3 normal;
    normal.xy = gl_TexCoord[0].xy*vec2(2.0, -2.0) + vec2(-1.0, 1.0);
    float mag = dot(normal.xy, normal.xy);
    if (mag > 1.0) discard;   // kill pixels outside circle
   	normal.z = sqrt(1.0-mag);

	gl_FragColor = vec4(normal.z*0.005);	
}
);

//--------------------------------------------------------
// Ellipsoid shaders
//
const char *vertexEllipsoidDepthShader = "#version 120\n"STRINGIFY(

// rotation matrix in xyz, scale in w
attribute vec4 q1;
attribute vec4 q2;
attribute vec4 q3;

// returns 1.0 for x==0.0 (unlike glsl)
float Sign(float x) { return x < 0.0 ? -1.0: 1.0; }

bool solveQuadratic(float a, float b, float c, out float minT, out float maxT)
{
	if (a == 0.0 && b == 0.0)
	{
		minT = maxT = 0.0;
		return false;
	}

	float discriminant = b*b - 4.0*a*c;

	if (discriminant < 0.0)
	{
		return false;
	}

	float t = -0.5*(b + Sign(b)*sqrt(discriminant));
	minT = t / a;
	maxT = c / t;

	if (minT > maxT)
	{
		float tmp = minT;
		minT = maxT;
		maxT = tmp;
	}

	return true;
}

float DotInvW(vec4 a, vec4 b) {	return a.x*b.x + a.y*b.y + a.z*b.z - a.w*b.w; }

void main()
{	
	vec3 worldPos = gl_Vertex.xyz;// - vec3(0.0, 0.1*0.25, 0.0);	// hack move towards ground to account for anisotropy

	// construct quadric matrix
	mat4 q;
	q[0] = vec4(q1.xyz*q1.w, 0.0);
	q[1] = vec4(q2.xyz*q2.w, 0.0);
	q[2] = vec4(q3.xyz*q3.w, 0.0);
	q[3] = vec4(worldPos, 1.0);

	// transforms a normal to parameter space (inverse transpose of (q*modelview)^-T)
	mat4 invClip = transpose(gl_ModelViewProjectionMatrix*q);

	// solve for the right hand bounds in homogenous clip space
	float a1 = DotInvW(invClip[3], invClip[3]);
	float b1 = -2.0f*DotInvW(invClip[0], invClip[3]);
	float c1 = DotInvW(invClip[0], invClip[0]);

	float xmin;
	float xmax;
 	solveQuadratic(a1, b1, c1, xmin, xmax);	

	// solve for the right hand bounds in homogenous clip space
	float a2 = DotInvW(invClip[3], invClip[3]);
	float b2 = -2.0f*DotInvW(invClip[1], invClip[3]);
	float c2 = DotInvW(invClip[1], invClip[1]); 

	float ymin;
	float ymax;
 	solveQuadratic(a2, b2, c2, ymin, ymax);

	gl_Position = vec4(worldPos.xyz, 1.0);
	gl_TexCoord[0] = vec4(xmin, xmax, ymin, ymax);

	// construct inverse quadric matrix (used for ray-casting in parameter space)
	mat4 invq;
	invq[0] = vec4(q1.xyz/q1.w, 0.0);
	invq[1] = vec4(q2.xyz/q2.w, 0.0);
	invq[2] = vec4(q3.xyz/q3.w, 0.0);
	invq[3] = vec4(0.0, 0.0, 0.0, 1.0);

	invq = transpose(invq);
	invq[3] = -(invq*gl_Position);

	// transform a point from view space to parameter space
	invq = invq*gl_ModelViewMatrixInverse;

	// pass down
	gl_TexCoord[1] = invq[0];
	gl_TexCoord[2] = invq[1];
	gl_TexCoord[3] = invq[2];
	gl_TexCoord[4] = invq[3];

	// compute ndc pos for frustrum culling in GS
	vec4 ndcPos = gl_ModelViewProjectionMatrix * vec4(worldPos.xyz, 1.0);
	gl_TexCoord[5] = ndcPos / ndcPos.w;
}
);

const char* geometryEllipsoidDepthShader = 
"#version 120\n"
"#extension GL_EXT_geometry_shader4 : enable\n"
STRINGIFY(
void main()
{
	vec3 pos = gl_PositionIn[0].xyz;
	vec4 bounds = gl_TexCoordIn[0][0];
	vec4 ndcPos = gl_TexCoordIn[0][5];

	// frustrum culling
	const float ndcBound = 1.0;
	if (ndcPos.x < -ndcBound) return;
	if (ndcPos.x > ndcBound) return;
	if (ndcPos.y < -ndcBound) return;
	if (ndcPos.y > ndcBound) return;

	float xmin = bounds.x;
	float xmax = bounds.y;
	float ymin = bounds.z;
	float ymax = bounds.w;

	// inv quadric transform
	gl_TexCoord[0] = gl_TexCoordIn[0][1];
	gl_TexCoord[1] = gl_TexCoordIn[0][2];
	gl_TexCoord[2] = gl_TexCoordIn[0][3];
	gl_TexCoord[3] = gl_TexCoordIn[0][4];

	gl_Position = vec4(xmin, ymax, 0.0, 1.0);
	EmitVertex();

	gl_Position = vec4(xmin, ymin, 0.0, 1.0);
	EmitVertex();

	gl_Position = vec4(xmax, ymax, 0.0, 1.0);
	EmitVertex();

	gl_Position = vec4(xmax, ymin, 0.0, 1.0);
	EmitVertex();
}
);

// pixel shader for rendering points as shaded spheres
const char *fragmentEllipsoidDepthShader = "#version 120\n"STRINGIFY(

uniform vec3 invViewport;
uniform vec3 invProjection;

float Sign(float x) { return x < 0.0 ? -1.0: 1.0; }

bool solveQuadratic(float a, float b, float c, out float minT, out float maxT)
{
	if (a == 0.0 && b == 0.0)
	{
		minT = maxT = 0.0;
		return true;
	}

	float discriminant = b*b - 4.0*a*c;

	if (discriminant < 0.0)
	{
		return false;
	}

	float t = -0.5*(b + Sign(b)*sqrt(discriminant));
	minT = t / a;
	maxT = c / t;

	if (minT > maxT)
	{
		float tmp = minT;
		minT = maxT;
		maxT = tmp;
	}

	return true;
}

float sqr(float x) { return x*x; }

void main()
{
	// transform from view space to parameter space
	mat4 invQuadric;
	invQuadric[0] = gl_TexCoord[0];
	invQuadric[1] = gl_TexCoord[1];
	invQuadric[2] = gl_TexCoord[2];
	invQuadric[3] = gl_TexCoord[3];

	vec4 ndcPos = vec4(gl_FragCoord.xy*invViewport.xy*vec2(2.0, 2.0) - vec2(1.0, 1.0), -1.0, 1.0);
	vec4 viewDir = gl_ProjectionMatrixInverse*ndcPos; 

	// ray to parameter space
	vec4 dir = invQuadric*vec4(viewDir.xyz, 0.0);
	vec4 origin = invQuadric[3];

	// set up quadratric equation
	float a = sqr(dir.x) + sqr(dir.y) + sqr(dir.z);// - sqr(dir.w);
	float b = dir.x*origin.x + dir.y*origin.y + dir.z*origin.z - dir.w*origin.w;
	float c = sqr(origin.x) + sqr(origin.y) + sqr(origin.z) - sqr(origin.w);

	float minT;
	float maxT;

	if (solveQuadratic(a, 2.0*b, c, minT, maxT))
	{
		vec3 eyePos = viewDir.xyz*minT;
		vec4 ndcPos = gl_ProjectionMatrix * vec4(eyePos, 1.0);
		ndcPos.z /= ndcPos.w;

		gl_FragColor = vec4(eyePos.z, 1.0, 1.0, 1.0);
		gl_FragDepth = ndcPos.z*0.5 + 0.5;

		return;
	}
	else
		discard;	

	gl_FragColor = vec4(0.5, 0.0, 0.0, 1.0);
}
);

//--------------------------------------------------------------------------------
// Composite shaders

const char* vertexPassThroughShader = STRINGIFY(

void main()
{
	gl_Position = vec4(gl_Vertex.xyz, 1.0);
	gl_TexCoord[0] = gl_MultiTexCoord0; 
}
);


const char* fragmentBlurDepthShader = 
"#extension GL_ARB_texture_rectangle : enable\n"
STRINGIFY(

uniform sampler2DRect depthTex;
uniform sampler2D thicknessTex;
uniform float blurRadiusWorld;
uniform float blurScale;
uniform float blurFalloff;
uniform vec2 invTexScale;

uniform bool debug;

float sqr(float x) { return x*x; }

void main()
{
    // eye-space depth of center sample
    float depth = texture2DRect(depthTex, gl_FragCoord.xy).x;
	float thickness = texture2D(thicknessTex, gl_TexCoord[0].xy).x;

	if (debug)
	{
		// do not blur
		gl_FragColor.x = depth;
		return;
	}

	// threshold on thickness to create nice smooth silhouettes
	if (depth == 0.0)//|| thickness < 0.02f)
	{
		gl_FragColor.x = 0.0;
		return;
	}

	/*
	float dzdx = dFdx(depth);
	float dzdy = dFdy(depth);

	// handle edge case
	if (max(abs(dzdx), abs(dzdy)) > 0.05)
	{
		dzdx = 0.0;
		dzdy = 0.0;

		gl_FragColor.x = depth;
		return;
	}
	*/

	float blurDepthFalloff = 5.5;//blurFalloff*mix(4.0, 1.0, thickness)/blurRadiusWorld*0.0375;	// these constants are just a re-scaling from some known good values

	float maxBlurRadius = 5.0;
	//float taps = min(maxBlurRadius, blurScale * (blurRadiusWorld / -depth));
	//vec2 blurRadius = min(mix(0.25, 2.0/blurFalloff, thickness) * blurScale * (blurRadiusWorld / -depth) / taps, 0.15)*invTexScale;
	
	//discontinuities between different tap counts are visible. to avoid this we 
	//use fractional contributions between #taps = ceil(radius) and floor(radius) 
	float radius = min(maxBlurRadius, blurScale * (blurRadiusWorld / -depth));
	float radiusInv = 1.0/radius;
	float taps = ceil(radius);
	float frac = taps - radius;

	float sum = 0.0;
    float wsum = 0.0;
	float count = 0.0;

    for(float y=-taps; y <= taps; y += 1.0)
	{
        for(float x=-taps; x <= taps; x += 1.0)
		{
			vec2 offset = vec2(x, y);

            float sample = texture2DRect(depthTex, gl_FragCoord.xy + offset).x;

			if (sample < -10000.0*0.5)
				continue;

            // spatial domain
            float r1 = length(vec2(x, y))*radiusInv;
			float w = exp(-(r1*r1));

			//float expectedDepth = depth + dot(vec2(dzdx, dzdy), offset);

            // range domain (based on depth difference)
            float r2 = (sample - depth) * blurDepthFalloff;
            float g = exp(-(r2*r2));

			//fractional radius contributions
			float wBoundary = step(radius, max(abs(x), abs(y)));
			float wFrac = 1.0 - wBoundary*frac;

			sum += sample * w * g * wFrac;
			wsum += w * g * wFrac;
			count += g * wFrac;
        }
    }

    if (wsum > 0.0) {
        sum /= wsum;
    }

	float blend = count/sqr(2.0*radius+1.0);
	gl_FragColor.x = mix(depth, sum, blend);
}
);

const char* fragmentCompositeShader = STRINGIFY(

uniform sampler2D tex;
uniform vec2 invTexScale;
uniform vec3 lightPos;
uniform vec3 lightDir;
uniform float spotMin;
uniform float spotMax;
uniform vec4 color;
uniform float ior;

uniform vec2 clipPosToEye;

uniform sampler2D reflectTex;
uniform sampler2DShadow shadowTex;
uniform vec2 shadowTaps[12];
uniform mat4 lightTransform;

uniform sampler2D thicknessTex;
uniform sampler2D sceneTex;

uniform bool debug;

// sample shadow map
float shadowSample(vec3 worldPos)
{
	vec4 pos = lightTransform*vec4(worldPos+lightDir*0.15, 1.0);
	pos /= pos.w;
	vec3 uvw = (pos.xyz*0.5)+vec3(0.5);

	// user clip
	if (uvw.x  < 0.0 || uvw.x > 1.0)
		return 1.0;
	if (uvw.y < 0.0 || uvw.y > 1.0)
		return 1.0;
	
	float s = 0.0;
	float radius = 0.002;

	for (int i=0; i < 8; i++)
	{
		s += shadow2D(shadowTex, vec3(uvw.xy + shadowTaps[i]*radius, uvw.z)).r;
	}

	s /= 8.0;
	return s;
}


vec3 viewportToEyeSpace(vec2 coord, float eyeZ)
{
	// find position at z=1 plane
	vec2 uv = (coord*2.0 - vec2(1.0))*clipPosToEye;

	return vec3(-uv*eyeZ, eyeZ);
}

vec3 srgbToLinear(vec3 c) { return pow(c, vec3(2.2)); }
vec3 linearToSrgb(vec3 c) { return pow(c, vec3(1.0/2.2)); }

float sqr(float x) { return x*x; }
float cube(float x) { return x*x*x; }

void main()
{
	float eyeZ = texture2D(tex, gl_TexCoord[0].xy).x;

	if (eyeZ == 0.0)
		discard;

	// reconstruct eye space pos from depth
	vec3 eyePos = viewportToEyeSpace(gl_TexCoord[0].xy, eyeZ);

	// finite difference approx for normals, can't take dFdx because
	// the one-sided difference is incorrect at shape boundaries
	vec3 zl = eyePos - viewportToEyeSpace(gl_TexCoord[0].xy - vec2(invTexScale.x, 0.0), texture2D(tex, gl_TexCoord[0].xy - vec2(invTexScale.x, 0.0)).x);
	vec3 zr = viewportToEyeSpace(gl_TexCoord[0].xy + vec2(invTexScale.x, 0.0), texture2D(tex, gl_TexCoord[0].xy + vec2(invTexScale.x, 0.0)).x) - eyePos;
	vec3 zt = viewportToEyeSpace(gl_TexCoord[0].xy + vec2(0.0, invTexScale.y), texture2D(tex, gl_TexCoord[0].xy + vec2(0.0, invTexScale.y)).x) - eyePos;
	vec3 zb = eyePos - viewportToEyeSpace(gl_TexCoord[0].xy - vec2(0.0, invTexScale.y), texture2D(tex, gl_TexCoord[0].xy - vec2(0.0, invTexScale.y)).x);
	
	vec3 dx = zl;
	vec3 dy = zt;

	if (abs(zr.z) < abs(zl.z))
		dx = zr;

	if (abs(zb.z) < abs(zt.z))
		dy = zb;

	//vec3 dx = dFdx(eyePos.xyz);
	//vec3 dy = dFdy(eyePos.xyz);

	vec4 worldPos = gl_ModelViewMatrixInverse*vec4(eyePos, 1.0);
	float shadow = shadowSample(worldPos.xyz);

	vec3 l = (gl_ModelViewMatrix*vec4(lightDir, 0.0)).xyz;
	vec3 v = -normalize(eyePos);
	
	vec3 n = normalize(cross(dx, dy));
	vec3 h = normalize(v + l);

	vec3 skyColor = vec3(0.1, 0.2, 0.4)*1.2;
	vec3 groundColor = vec3(0.1, 0.1, 0.2);
	
	float fresnel = 0.1 + (1.0 - 0.1)*cube(1.0-max(dot(n, v), 0.0));

	vec3 lVec = normalize(worldPos.xyz-lightPos);
	float attenuation = max(smoothstep(spotMin, spotMax, abs(dot(lVec, -lightDir))), 0.05);

	float ln = dot(l, n)*attenuation;

	vec3 rEye = reflect(-v, n).xyz;
	vec3 rWorld = (gl_ModelViewMatrixInverse*vec4(rEye, 0.0)).xyz;

	vec2 texScale = vec2(0.75, 1.0);	// to account for backbuffer aspect ratio (todo: pass in)

	float refractScale = ior*0.025;
	float reflectScale = ior*0.1;

	// attenuate refraction near ground (hack)
	refractScale *= smoothstep(0.1, 0.4, worldPos.y);
	
	vec2 refractCoord = gl_TexCoord[0].xy + n.xy*refractScale*texScale;	
	//vec2 refractCoord = gl_TexCoord[0].xy + refract(-v, n, 1.0/1.33)*refractScale*texScale;	

	// read thickness from refracted coordinate otherwise we get halos around objectsw
	float thickness = max(texture2D(thicknessTex, refractCoord).x, 0.3);
	
	//vec3 transmission = exp(-(vec3(1.0)-color.xyz)*thickness);
	vec3 transmission = (1.0-(1.0-color.xyz)*thickness*0.8)*color.w; 
	vec3 refract = texture2D(sceneTex, refractCoord).xyz*transmission;

	vec2 sceneReflectCoord = gl_TexCoord[0].xy - rEye.xy*texScale*reflectScale/eyePos.z;	
	vec3 sceneReflect = (texture2D(sceneTex, sceneReflectCoord).xyz)*shadow;
	vec3 planarReflect = texture2D(reflectTex, gl_TexCoord[0].xy).xyz;

	// fade out planar reflections above the ground
	vec3 reflect = mix(planarReflect, sceneReflect, smoothstep(0.05, 0.3, worldPos.y)) + mix(groundColor, skyColor, smoothstep(0.15, 0.25, rWorld.y)*shadow);
	
	// lighting
	vec3 diffuse = color.xyz*mix(vec3(0.29, 0.379, 0.59), vec3(1.0), (ln*0.5 + 0.5)*max(shadow, 0.4))*(1.0-color.w);
	vec3 specular = vec3(1.2*pow(max(dot(h, n), 0.0), 400.0));

	gl_FragColor.xyz = diffuse + (mix(refract, reflect, fresnel) + specular)*color.w;
	gl_FragColor.w = 1.0;

	if (debug)
		gl_FragColor = vec4(n*0.5 + vec3(0.5), 1.0);

	// write valid z
	vec4 clipPos = gl_ProjectionMatrix*vec4(0.0, 0.0, eyeZ, 1.0);
	clipPos.z /= clipPos.w;

	gl_FragDepth = clipPos.z*0.5 + 0.5;
}
);


struct FluidRenderer
{
	GLuint mDepthFbo;
	GLuint mDepthTex;
	GLuint mDepthSmoothTex;
	GLuint mSceneFbo;
	GLuint mSceneTex;
	GLuint mReflectTex;

	GLuint mThicknessFbo;
	GLuint mThicknessTex;

	GLuint mPointThicknessProgram;
	GLuint mPointDepthProgram;

	GLuint mEllipsoidThicknessProgram;
	GLuint mEllipsoidDepthProgram;

	GLuint mCompositeProgram;
	GLuint mDepthBlurProgram;

	int mSceneWidth;
	int mSceneHeight;
};

FluidRenderer* CreateFluidRenderer(uint32_t width, uint32_t height)
{
	FluidRenderer* renderer = new FluidRenderer();

	renderer->mSceneWidth = width;
	renderer->mSceneHeight = height;

	// scene depth texture
	glVerify(glGenTextures(1, &renderer->mDepthTex));
	glVerify(glBindTexture(GL_TEXTURE_RECTANGLE_ARB, renderer->mDepthTex));

	glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)); 
	glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)); 	 
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
	glVerify(glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_LUMINANCE32F_ARB, width, height, 0, GL_LUMINANCE, GL_FLOAT, NULL));

	// smoothed depth texture
	glVerify(glGenTextures(1, &renderer->mDepthSmoothTex));
	glVerify(glBindTexture(GL_TEXTURE_2D, renderer->mDepthSmoothTex));

	glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)); 
	glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)); 	 
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
	glVerify(glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE32F_ARB, width, height, 0, GL_LUMINANCE, GL_FLOAT, NULL));

	// scene copy
	glVerify(glGenTextures(1, &renderer->mSceneTex));
	glVerify(glBindTexture(GL_TEXTURE_2D, renderer->mSceneTex));

	glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)); 
	glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)); 	 
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
	glVerify(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL));

	glVerify(glGenFramebuffers(1, &renderer->mSceneFbo));
	glVerify(glBindFramebuffer(GL_FRAMEBUFFER, renderer->mSceneFbo));
	glVerify(glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, renderer->mSceneTex, 0));


	// frame buffer
	glVerify(glGenFramebuffers(1, &renderer->mDepthFbo));
	glVerify(glBindFramebuffer(GL_FRAMEBUFFER, renderer->mDepthFbo));
	glVerify(glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_RECTANGLE_ARB, renderer->mDepthTex, 0));
	
	GLuint zbuffer;
	glVerify(glGenRenderbuffers(1, &zbuffer));
	glVerify(glBindRenderbuffer(GL_RENDERBUFFER, zbuffer));
	glVerify(glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height));
	glVerify(glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, zbuffer));

	glVerify(glDrawBuffer(GL_COLOR_ATTACHMENT0));
	glVerify(glReadBuffer(GL_COLOR_ATTACHMENT0));

	GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	assert(status == GL_FRAMEBUFFER_COMPLETE);
	glBindFramebuffer(GL_FRAMEBUFFER, g_msaaFbo);
	
	// reflect texture
	glVerify(glGenTextures(1, &renderer->mReflectTex));
	glVerify(glBindTexture(GL_TEXTURE_2D, renderer->mReflectTex));

	glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)); 
	glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)); 	 
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
	glVerify(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL));

	// thickness texture
	const int thicknessWidth = width;
	const int thicknessHeight = height;

	glVerify(glGenTextures(1, &renderer->mThicknessTex));
	glVerify(glBindTexture(GL_TEXTURE_2D, renderer->mThicknessTex));

	glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)); 
	glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)); 	 
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));

#if USE_HDR_DIFFUSE_BLEND	
	glVerify(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, thicknessWidth, thicknessHeight, 0, GL_RGBA, GL_FLOAT, NULL));
#else
	glVerify(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, thicknessWidth, thicknessHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL));
#endif
		
	// thickness buffer
	glVerify(glGenFramebuffers(1, &renderer->mThicknessFbo));
	glVerify(glBindFramebuffer(GL_FRAMEBUFFER, renderer->mThicknessFbo));
	glVerify(glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, renderer->mThicknessTex, 0));
	
	GLuint thickz;
	glVerify(glGenRenderbuffers(1, &thickz));
	glVerify(glBindRenderbuffer(GL_RENDERBUFFER, thickz));
	glVerify(glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, thicknessWidth, thicknessHeight));
	glVerify(glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, thickz));
	
	status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	assert(status == GL_FRAMEBUFFER_COMPLETE);
	glBindFramebuffer(GL_FRAMEBUFFER, g_msaaFbo);

	// compile shaders
	renderer->mPointDepthProgram = CompileProgram(vertexPointDepthShader, fragmentPointDepthShader);
	renderer->mPointThicknessProgram = CompileProgram(vertexPointDepthShader, fragmentPointThicknessShader);

	//renderer->mEllipsoidThicknessProgram = CompileProgram(vertexEllipsoidDepthShader, fragmentEllipsoidThicknessShader);
	renderer->mEllipsoidDepthProgram = CompileProgram(vertexEllipsoidDepthShader, fragmentEllipsoidDepthShader, geometryEllipsoidDepthShader);

	renderer->mCompositeProgram = CompileProgram(vertexPassThroughShader, fragmentCompositeShader);
	renderer->mDepthBlurProgram = CompileProgram(vertexPassThroughShader, fragmentBlurDepthShader);

	return renderer;
}

void DestroyFluidRenderer(FluidRenderer* renderer)
{
	glVerify(glDeleteFramebuffers(1, &renderer->mSceneFbo));
	glVerify(glDeleteFramebuffers(1, &renderer->mDepthFbo));
	glVerify(glDeleteTextures(1, &renderer->mDepthTex));
	glVerify(glDeleteTextures(1, &renderer->mDepthSmoothTex));
	glVerify(glDeleteTextures(1, &renderer->mSceneTex));

	glVerify(glDeleteFramebuffers(1, &renderer->mThicknessFbo));
	glVerify(glDeleteTextures(1, &renderer->mThicknessTex));
}

FluidRenderBuffers CreateFluidRenderBuffers(int numFluidParticles, int numDiffuseParticles)
{
	FluidRenderBuffers buffers;
	buffers.mNumFluidParticles = numFluidParticles;
	buffers.mNumDiffuseParticles = numDiffuseParticles;

	// write only
	cudaGraphicsRegisterFlags flags = cudaGraphicsRegisterFlagsWriteDiscard;

	// vbos
	glVerify(glGenBuffers(1, &buffers.mPositionVBO));
	glVerify(glBindBuffer(GL_ARRAY_BUFFER, buffers.mPositionVBO));
	glVerify(glBufferData(GL_ARRAY_BUFFER, sizeof(float)*4*numFluidParticles, 0, GL_DYNAMIC_DRAW));

	cudaCheck(cudaGraphicsGLRegisterBuffer(&buffers.mPositionRes, buffers.mPositionVBO, flags));

	// density
	glVerify(glGenBuffers(1, &buffers.mDensityVBO));
	glVerify(glBindBuffer(GL_ARRAY_BUFFER, buffers.mDensityVBO));
	glVerify(glBufferData(GL_ARRAY_BUFFER, sizeof(int)*numFluidParticles, 0, GL_DYNAMIC_DRAW));

	cudaCheck(cudaGraphicsGLRegisterBuffer(&buffers.mDensityRes, buffers.mDensityVBO, flags));


	for (int i=0; i < 3; ++i)
	{
		glVerify(glGenBuffers(1, &buffers.mAnisotropyVBO[i]));
		glVerify(glBindBuffer(GL_ARRAY_BUFFER, buffers.mAnisotropyVBO[i]));
		glVerify(glBufferData(GL_ARRAY_BUFFER, sizeof(float)*4*numFluidParticles, 0, GL_DYNAMIC_DRAW));

		cudaCheck(cudaGraphicsGLRegisterBuffer(&buffers.mAnisotropyRes[i], buffers.mAnisotropyVBO[i], flags));	
	}

	if (numDiffuseParticles)
	{
		glVerify(glGenBuffers(1, &buffers.mDiffusePositionVBO));
		glVerify(glBindBuffer(GL_ARRAY_BUFFER, buffers.mDiffusePositionVBO));
		glVerify(glBufferData(GL_ARRAY_BUFFER, sizeof(float)*4*numDiffuseParticles, 0, GL_DYNAMIC_DRAW));
		glVerify(glBindBuffer(GL_ARRAY_BUFFER, 0));

		glVerify(glGenBuffers(1, &buffers.mDiffuseVelocityVBO));
		glVerify(glBindBuffer(GL_ARRAY_BUFFER, buffers.mDiffuseVelocityVBO));
		glVerify(glBufferData(GL_ARRAY_BUFFER, sizeof(float)*4*numDiffuseParticles, 0, GL_DYNAMIC_DRAW));
		glVerify(glBindBuffer(GL_ARRAY_BUFFER, 0));

		glVerify(glGenBuffers(1, &buffers.mDiffuseIndicesIBO));
		glVerify(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffers.mDiffuseIndicesIBO));
		glVerify(glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int)*numDiffuseParticles, 0, GL_DYNAMIC_DRAW));
		glVerify(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));

		cudaCheck(cudaGraphicsGLRegisterBuffer(&buffers.mDiffusePositionRes, buffers.mDiffusePositionVBO, flags));
		cudaCheck(cudaGraphicsGLRegisterBuffer(&buffers.mDiffuseVelocityRes, buffers.mDiffuseVelocityVBO, flags));
		cudaCheck(cudaGraphicsGLRegisterBuffer(&buffers.mDiffuseIndicesRes, buffers.mDiffuseIndicesIBO, flags));
	}

	return buffers;
}

void DestroyFluidRenderBuffers(FluidRenderBuffers buffers)
{
	cudaCheck(cudaGraphicsUnregisterResource(buffers.mPositionRes));
	cudaCheck(cudaGraphicsUnregisterResource(buffers.mDensityRes));
	
	for (int i=0; i < 3; ++i)
		cudaCheck(cudaGraphicsUnregisterResource(buffers.mAnisotropyRes[i]));

	if (buffers.mNumDiffuseParticles)
	{
		cudaCheck(cudaGraphicsUnregisterResource(buffers.mDiffusePositionRes));
		cudaCheck(cudaGraphicsUnregisterResource(buffers.mDiffuseVelocityRes));
		cudaCheck(cudaGraphicsUnregisterResource(buffers.mDiffuseIndicesRes));
	}

	glDeleteBuffers(1, &buffers.mPositionVBO);
	glDeleteBuffers(3, buffers.mAnisotropyVBO);
	glDeleteBuffers(1, &buffers.mDensityVBO);

	if (buffers.mNumDiffuseParticles)
	{
		glDeleteBuffers(1, &buffers.mDiffusePositionVBO);
		glDeleteBuffers(1, &buffers.mDiffuseVelocityVBO);
		glDeleteBuffers(1, &buffers.mDiffuseIndicesIBO);
	}
}

void RenderFullscreenQuad()
{
	glColor3f(1.0f, 1.0f, 1.0f);
	glBegin(GL_QUADS);

	glTexCoord2f(0.0f, 0.0f);
	glVertex2f(-1.0f, -1.0f);

	glTexCoord2f(1.0f, 0.0f);
	glVertex2f(1.0f, -1.0f);

	glTexCoord2f(1.0f, 1.0f);
	glVertex2f(1.0f, 1.0f);

	glTexCoord2f(0.0f, 1.0f);
	glVertex2f(-1.0f, 1.0f);

	glEnd();
}

extern Mesh* g_mesh;
void DrawConvexes();

void RenderEllipsoids(FluidRenderer* render, GLuint positions, GLuint q1, GLuint q2, GLuint q3, GLuint densities, int n, int offset, float radius, float screenWidth, float screenAspect, float fov, Vec3 lightPos, Vec3 lightTarget, Matrix44 lightTransform, GLuint shadowTex, GLuint reflectTex, Vec4 color, float blur, float ior, bool debug)
{
	// resolve msaa back buffer to texture
	glVerify(glBindFramebuffer(GL_READ_FRAMEBUFFER_EXT, g_msaaFbo));
	glVerify(glBindFramebuffer(GL_DRAW_FRAMEBUFFER_EXT, render->mSceneFbo));
	glVerify(glBlitFramebuffer(0, 0, GLsizei(screenWidth), GLsizei(screenWidth/screenAspect), 0, 0, GLsizei(screenWidth), GLsizei(screenWidth/screenAspect), GL_COLOR_BUFFER_BIT, GL_LINEAR));

	//thickness texture
	glVerify(glBindFramebuffer(GL_FRAMEBUFFER, render->mThicknessFbo));
	glVerify(glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, render->mThicknessTex, 0));
	glVerify(glDrawBuffer(GL_COLOR_ATTACHMENT0));

	glViewport(0, 0, GLsizei(screenWidth), GLsizei(screenWidth/screenAspect));
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClear(GL_DEPTH_BUFFER_BIT);

	glDepthMask(GL_TRUE);
	glDisable(GL_CULL_FACE);

	if (g_mesh)
		DrawMesh(g_mesh, Vec3(1.0f));

	DrawConvexes();

	glClear(GL_COLOR_BUFFER_BIT);

	glTexEnvi(GL_POINT_SPRITE, GL_COORD_REPLACE, GL_TRUE);
	glEnable(GL_POINT_SPRITE);
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_ONE, GL_ONE);
	glDepthMask(GL_FALSE);

	// make sprites larger to get smoother thickness texture
	const float thicknessScale = 4.0f; 

	glUseProgram(render->mPointThicknessProgram);
	glUniform1f( glGetUniformLocation(render->mPointThicknessProgram, "pointRadius"), thicknessScale*radius);
	glUniform1f( glGetUniformLocation(render->mPointThicknessProgram, "pointScale"), screenWidth/screenAspect * (1.0f / (tanf(fov*0.5f))));

	glEnableClientState(GL_VERTEX_ARRAY);			
	glBindBuffer(GL_ARRAY_BUFFER, positions);
	glVertexPointer(3, GL_FLOAT, sizeof(float)*4, (void*)(offset*sizeof(float)*4));

	glDrawArrays(GL_POINTS, 0, n);

	glUseProgram(0);
	glDisableClientState(GL_VERTEX_ARRAY);	
	glDisable(GL_POINT_SPRITE);
	glDisable(GL_BLEND);
		
	// depth texture
	glVerify(glBindFramebuffer(GL_FRAMEBUFFER, render->mDepthFbo));
	glVerify(glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_RECTANGLE_ARB, render->mDepthTex, 0));
	glVerify(glDrawBuffer(GL_COLOR_ATTACHMENT0));
	
	// draw points
	//glTexEnvi(GL_POINT_SPRITE, GL_COORD_REPLACE, GL_TRUE);
	glDisable(GL_POINT_SPRITE);
	glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);
	glEnable(GL_DEPTH_TEST);
	glDepthMask(GL_TRUE);

	glViewport(0, 0, int(screenWidth), int(screenWidth/screenAspect));
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	const float viewHeight = tanf(fov/2.0f);

	glUseProgram(render->mEllipsoidDepthProgram);
	glUniform3fv( glGetUniformLocation(render->mEllipsoidDepthProgram, "invViewport"), 1, Vec3(1.0f/screenWidth, screenAspect/screenWidth, 1.0f));
	glUniform3fv( glGetUniformLocation(render->mEllipsoidDepthProgram, "invProjection"), 1, Vec3(screenAspect*viewHeight, viewHeight, 1.0f));

	glEnableClientState(GL_VERTEX_ARRAY);
	glBindBuffer(GL_ARRAY_BUFFER, positions);
	glVertexPointer(3, GL_FLOAT, sizeof(float)*4, 0);//(void*)(offset*sizeof(float)*4));

	// ellipsoid eigenvectors
	int s1 = glGetAttribLocation(render->mEllipsoidDepthProgram, "q1");
	glEnableVertexAttribArray(s1);
	glBindBuffer(GL_ARRAY_BUFFER, q1);
	glVertexAttribPointer(s1, 4, GL_FLOAT, GL_FALSE, 0, 0);// (void*)(offset*sizeof(float)*4));

	int s2 = glGetAttribLocation(render->mEllipsoidDepthProgram, "q2");
	glEnableVertexAttribArray(s2);
	glBindBuffer(GL_ARRAY_BUFFER, q2);
	glVertexAttribPointer(s2, 4, GL_FLOAT, GL_FALSE, 0, 0);//(void*)(offset*sizeof(float)*4));

	int s3 = glGetAttribLocation(render->mEllipsoidDepthProgram, "q3");
	glEnableVertexAttribArray(s3);
	glBindBuffer(GL_ARRAY_BUFFER, q3);
	glVertexAttribPointer(s3, 4, GL_FLOAT, GL_FALSE, 0, 0);// (void*)(offset*sizeof(float)*4));
	
	glVerify(glDrawArrays(GL_POINTS, offset, n));

	glUseProgram(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableVertexAttribArray(s1);
	glDisableVertexAttribArray(s2);
	glDisableVertexAttribArray(s3);

	glDisable(GL_POINT_SPRITE);

	//---------------------------------------------------------------
	// blur

	glDisable(GL_DEPTH_TEST);
	glDepthMask(GL_FALSE);

	glVerify(glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, render->mDepthSmoothTex, 0));
	glUseProgram(render->mDepthBlurProgram);
	
	glActiveTexture(GL_TEXTURE0);
	glEnable(GL_TEXTURE_RECTANGLE_ARB);	
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, render->mDepthTex);

	glActiveTexture(GL_TEXTURE1);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, render->mThicknessTex);

	glVerify(glUniform1f( glGetUniformLocation(render->mDepthBlurProgram, "blurRadiusWorld"), radius*0.5f));	// blur half the radius by default
	glVerify(glUniform1f( glGetUniformLocation(render->mDepthBlurProgram, "blurScale"), screenWidth/screenAspect * (1.0f / (tanf(fov*0.5f)))));
	glVerify(glUniform2fv( glGetUniformLocation(render->mDepthBlurProgram, "invTexScale"), 1, Vec2(1.0f/screenAspect, 1.0f)));
	glVerify(glUniform1f( glGetUniformLocation(render->mDepthBlurProgram, "blurFalloff"),  blur));
	glVerify(glUniform1i( glGetUniformLocation(render->mDepthBlurProgram, "depthTex"), 0));
	glVerify(glUniform1i( glGetUniformLocation(render->mDepthBlurProgram, "thicknessTex"), 1));
	glVerify(glUniform1i(glGetUniformLocation(render->mDepthBlurProgram, "debug"), debug));

	glVerify(RenderFullscreenQuad());

	glActiveTexture(GL_TEXTURE0);
	glDisable(GL_TEXTURE_RECTANGLE_ARB);	

	//---------------------------------------------------------------
	// composite with scene

	glVerify(glBindFramebuffer(GL_FRAMEBUFFER, g_msaaFbo));
	glVerify(glEnable(GL_DEPTH_TEST));
	glVerify(glDepthMask(GL_TRUE));
	glVerify(glDisable(GL_BLEND));
	glVerify(glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA));

	glVerify(glUseProgram(render->mCompositeProgram));	

	glVerify(glUniform2fv(glGetUniformLocation(render->mCompositeProgram, "invTexScale"), 1, Vec2(1.0f/screenWidth, screenAspect/screenWidth)));
	glVerify(glUniform2fv(glGetUniformLocation(render->mCompositeProgram, "clipPosToEye"), 1, Vec2(tanf(fov*0.5f)*screenAspect, tanf(fov*0.5f))));
	glVerify(glUniform4fv(glGetUniformLocation(render->mCompositeProgram, "color"), 1, color));
	glVerify(glUniform1f(glGetUniformLocation(render->mCompositeProgram, "ior"),  ior));
	glVerify(glUniform1f(glGetUniformLocation(render->mCompositeProgram, "spotMin"), gSpotMin));
	glVerify(glUniform1f(glGetUniformLocation(render->mCompositeProgram, "spotMax"), gSpotMax));
	glVerify(glUniform1i(glGetUniformLocation(render->mCompositeProgram, "debug"), debug));

	glVerify(glUniform3fv(glGetUniformLocation(render->mCompositeProgram, "lightPos"), 1, lightPos));
	glVerify(glUniform3fv(glGetUniformLocation(render->mCompositeProgram, "lightDir"), 1, -Normalize(lightTarget-lightPos)));
	glVerify(glUniformMatrix4fv(glGetUniformLocation(render->mCompositeProgram, "lightTransform"), 1, false, lightTransform));
	
	const Vec2 taps[] = 
	{ 
		Vec2(-0.326212f,-0.40581f),Vec2(-0.840144f,-0.07358f),
		Vec2(-0.695914f,0.457137f),Vec2(-0.203345f,0.620716f),
		Vec2(0.96234f,-0.194983f),Vec2(0.473434f,-0.480026f),
		Vec2(0.519456f,0.767022f),Vec2(0.185461f,-0.893124f),
		Vec2(0.507431f,0.064425f),Vec2(0.89642f,0.412458f),
		Vec2(-0.32194f,-0.932615f),Vec2(-0.791559f,-0.59771f) 
	};
	
	glVerify(glUniform2fv(glGetUniformLocation(render->mCompositeProgram, "shadowTaps"), 12, &taps[0].x));

	// smoothed depth tex
	glVerify(glActiveTexture(GL_TEXTURE0));
	glVerify(glEnable(GL_TEXTURE_2D));
	glVerify(glBindTexture(GL_TEXTURE_2D, render->mDepthSmoothTex));

	// shadow tex
	glVerify(glActiveTexture(GL_TEXTURE1));
	glVerify(glEnable(GL_TEXTURE_2D));
	glVerify(glBindTexture(GL_TEXTURE_2D, shadowTex));

	// thickness tex
	glVerify(glActiveTexture(GL_TEXTURE2));
	glVerify(glEnable(GL_TEXTURE_2D));
	glVerify(glBindTexture(GL_TEXTURE_2D, render->mThicknessTex));

	// scene tex
	glVerify(glActiveTexture(GL_TEXTURE3));
	glVerify(glEnable(GL_TEXTURE_2D));
	glVerify(glBindTexture(GL_TEXTURE_2D, render->mSceneTex));

	// reflection tex
	glVerify(glActiveTexture(GL_TEXTURE5));
	glVerify(glEnable(GL_TEXTURE_2D));
	glVerify(glBindTexture(GL_TEXTURE_2D, reflectTex));	

	glVerify(glUniform1i(glGetUniformLocation(render->mCompositeProgram, "tex"), 0));
	glVerify(glUniform1i(glGetUniformLocation(render->mCompositeProgram, "shadowTex"), 1));
	glVerify(glUniform1i(glGetUniformLocation(render->mCompositeProgram, "thicknessTex"), 2));
	glVerify(glUniform1i(glGetUniformLocation(render->mCompositeProgram, "sceneTex"), 3));
	glVerify(glUniform1i(glGetUniformLocation(render->mCompositeProgram, "reflectTex"), 5));

	// -- end shadowing
	
	// ignores projection matrices
	glVerify(RenderFullscreenQuad());


	// reset state
	glActiveTexture(GL_TEXTURE5);
	glDisable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE3);
	glDisable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE2);
	glDisable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE1);
	glDisable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE0);
	glDisable(GL_TEXTURE_2D);


	glEnable(GL_DEPTH_TEST);
	glDepthMask(GL_TRUE);
	glDisable(GL_BLEND);
}

//------------------------------------------------------------------------------
// Diffuse Shading

const char *vertexDiffuseShader = STRINGIFY(

uniform float pointRadius;  // point size in world space
uniform float pointScale;   // scale to calculate size in pixels
uniform vec3 lightPos;
uniform vec3 lightDir;
uniform float spotMin;
uniform float spotMax;
uniform vec4 color;

void main()
{
	vec3 worldPos = gl_Vertex.xyz;// - vec3(0.0, 0.1*0.25, 0.0);	// hack move towards ground to account for anisotropy;
	vec4 eyePos = gl_ModelViewMatrix * vec4(worldPos, 1.0);

	gl_Position = gl_ProjectionMatrix * eyePos;
	//gl_Position.z -= 0.0025;	// bias above fluid surface

    // calculate window-space point size
	gl_PointSize = pointRadius * (pointScale / gl_Position.w);

	vec3 lVec = normalize(worldPos-lightPos);
	float attenuation = max(smoothstep(spotMin, spotMax, dot(lVec, lightDir)), 0.2);

	gl_TexCoord[0] = gl_MultiTexCoord0;    
	gl_TexCoord[1] = vec4(worldPos, gl_Vertex.w);
	gl_TexCoord[2] = eyePos;

	gl_TexCoord[3].xyz = gl_ModelViewMatrix*vec4(gl_MultiTexCoord1.xyz, 0.0);
	gl_TexCoord[3].w = attenuation;

	gl_TexCoord[4].xyzw = color;

	// hack to color different emitters 
	if (gl_MultiTexCoord1.w == 2.0)
		gl_TexCoord[4].xyzw = vec4(0.85, 0.65, 0.65, color.w);
	else if (gl_MultiTexCoord1.w == 1.0)
		gl_TexCoord[4].xyzw = vec4(0.65, 0.85, 0.65, color.w);

	// compute ndc pos for frustrum culling in GS
	vec4 ndcPos = gl_ModelViewProjectionMatrix * vec4(worldPos.xyz, 1.0);
	gl_TexCoord[5] = ndcPos / ndcPos.w;
}
);




const char *geometryDiffuseShader = 
"#version 120\n"
"#extension GL_EXT_geometry_shader4 : enable\n"
STRINGIFY(

uniform float pointScale;  // point size in world space
uniform float motionBlurScale;
uniform float diffusion;
uniform vec3 lightDir;

void main()
{
	vec4 ndcPos = gl_TexCoordIn[0][5];

	// frustrum culling
	const float ndcBound = 1.0;
	if (ndcPos.x < -ndcBound) return;
	if (ndcPos.x > ndcBound) return;
	if (ndcPos.y < -ndcBound) return;
	if (ndcPos.y > ndcBound) return;

	float velocityScale = 1.0;

	vec3 v = gl_TexCoordIn[0][3].xyz*velocityScale;
	vec3 p = gl_TexCoordIn[0][2].xyz;
		
	// billboard in eye space
	vec3 u = vec3(0.0, pointScale, 0.0);
	vec3 l = vec3(pointScale, 0.0, 0.0);
	
	// increase size based on life
	float lifeFade = mix(1.0f+diffusion, 1.0, min(1.0, gl_TexCoordIn[0][1].w*0.25f));
	u *= lifeFade;
	l *= lifeFade;

	//lifeFade = 1.0;

	float fade = 1.0/(lifeFade*lifeFade);
	float vlen = length(v)*motionBlurScale;

	if (vlen > 0.5)
	{
		float len = max(pointScale, vlen*0.016);
		fade = min(1.0, 2.0/(len/pointScale));

		u = normalize(v)*max(pointScale, vlen*0.016);	// assume 60hz
		l = normalize(cross(u, vec3(0.0, 0.0, -1.0)))*pointScale;
	}	
	
	{
		
		gl_TexCoord[1] = gl_TexCoordIn[0][1];	// vertex world pos (life in w)
		gl_TexCoord[2] = gl_TexCoordIn[0][2];	// vertex eye pos
		gl_TexCoord[3] = gl_TexCoordIn[0][3];	// vertex velocity in view space
		gl_TexCoord[3].w = fade;
		gl_TexCoord[4] = gl_ModelViewMatrix*vec4(lightDir, 0.0);
		gl_TexCoord[4].w = gl_TexCoordIn[0][3].w; // attenuation
		gl_TexCoord[5].xyzw = gl_TexCoordIn[0][4].xyzw;	// color

		float zbias = 0.0f;//0.00125*2.0;

        gl_TexCoord[0] = vec4(0.0, 1.0, 0.0, 0.0);
        gl_Position = gl_ProjectionMatrix * vec4(p + u - l, 1.0);
		gl_Position.z -= zbias;
        EmitVertex();
		
		gl_TexCoord[0] = vec4(0.0, 0.0, 0.0, 0.0);
        gl_Position = gl_ProjectionMatrix * vec4(p - u - l, 1.0);
		gl_Position.z -= zbias;
        EmitVertex();

		gl_TexCoord[0] = vec4(1.0, 1.0, 0.0, 0.0);
        gl_Position = gl_ProjectionMatrix * vec4(p + u + l, 1.0);
		gl_Position.z -= zbias;
        EmitVertex();

		gl_TexCoord[0] = vec4(1.0, 0.0, 0.0, 0.0);
        gl_Position = gl_ProjectionMatrix * vec4(p - u + l, 1.0);
		gl_Position.z -= zbias;
        EmitVertex();
    }
}
);

const char *fragmentDiffuseShader = STRINGIFY(

float sqr(float x) { return x*x; }
float cube(float x) { return x*x*x; }

uniform sampler2D depthTex;
uniform sampler2D noiseTex;
uniform vec2 invViewport;
uniform vec4 color;
uniform bool front;
uniform bool shadow;

//uniform sampler2DShadow shadowTex;
uniform sampler2D shadowTex;
uniform vec2 shadowTaps[12];
uniform mat4 lightTransform;
uniform vec3 lightDir;
uniform float inscatterCoefficient;
uniform float outscatterCoefficient;

// sample shadow map
float shadowSample(vec3 worldPos)
{
	vec4 pos = lightTransform*vec4(worldPos-lightDir*0.0, 1.0);
	pos /= pos.w;
	vec3 uvw = (pos.xyz*0.5)+vec3(0.5);

	// user clip
	if (uvw.x  < 0.0 || uvw.x > 1.0)
		return 0.0;
	if (uvw.y < 0.0 || uvw.y > 1.0)
		return 0.0;
	
	float s = 0.0;
	float radius = 0.001;
	
	/*
	const int numTaps = 6;
	for (int i=0; i < numTaps; i++)
	{
		s += shadow2D(shadowTex, vec3(uvw.xy + shadowTaps[i]*radius, uvw.z)).r;
	}

	s /= float(numTaps);
	*/

	const int numTaps = 6;
	for (int i=0; i < numTaps; i++)
	{
		s += 1.0-clamp((uvw.z-texture2D(shadowTex, uvw.xy + shadowTaps[i]*radius).x)*200.0, 0.0, 1.0);
	}
	s /= float(numTaps);

	return s;
}


void main()
{
	float attenuation = gl_TexCoord[4].w;
	float lifeFade = min(1.0, gl_TexCoord[1].w*0.125);

    // calculate normal from texture coordinates
    vec3 normal;
    normal.xy = gl_TexCoord[0].xy*vec2(2.0, 2.0) + vec2(-1.0, -1.0);
    float mag = dot(normal.xy, normal.xy);
    if (mag > 1.0) discard;   // kill pixels outside circle
   	normal.z = 1.0-mag;

	// two pass (depth peeled), second pass only has to re-draw particles in front of the fluid surface
	if (front)
	{
		float fluidDepth = texture2D(depthTex, gl_FragCoord.xy*invViewport);
		if (fluidDepth >= 0.0)
			discard;
	}
	
	/*
	float fluidDepth = texture2D(depthTex, gl_FragCoord.xy*invViewport);
	float particleDepth = gl_TexCoord[2].z;
	
    if (fluidDepth < 0.0 && fluidDepth > particleDepth)
    {
            vec4 ndc = gl_ProjectionMatrix*vec4(0.0, 0.0, fluidDepth+0.05, 1.0);
            ndc.z /= ndc.w;

            gl_FragDepth = ndc.z*0.5 + 0.5;
    }
	else
	{
		// if particle in front of fluid then don't fade
		depthFade = 1.0;
        gl_FragDepth = gl_FragCoord.z;
	}
	*/

	// shadowing
	float s = 1.0;
	if (shadow)
		s = shadowSample(gl_TexCoord[1].xyz);// + normal*0.1);

	// wrapped diffuse
	float ln = 1.0; //dot(normal, -gl_TexCoord[4].xyz);//*0.25 + 0.75;

	// attenuate lighting based on depth (out scattering due to water)
	vec3 scatteringCoefficient = vec3(0.09, 0.1, 0.1);
	//vec3 scatteringCoefficient = vec3(1.0) - color.xyz*1.5;
	float lightDepth = 1.0f-s;

	// how much light reaches this particle (approximate based on shadow map value)
	vec3 lightTransmission = exp(-lightDepth*scatteringCoefficient*12.0);

	/*
	gl_FragColor.xyz = s;
	gl_FragColor.w = 1.0;
	return;
	*/

	// ambient color
	vec3 dark = vec3(0.05, 0.035, 0.035);	
	vec3 light = color.xyz*1.25;
	vec3 l = mix(dark, light, ln)*attenuation;
	vec3 c = lightTransmission*l;

	float velocityFade = gl_TexCoord[3].w;
	//float alpha = smoothstep(0.0, 1.0, normal.z)*lifeFade*velocityFade;
	float alpha = lifeFade*velocityFade*sqr(normal.z);//*max(texture2D(noiseTex, gl_TexCoord[0].xy).x*2.5f, 0.0);

	if (shadow)
	{
		// good for gases
		float outscatter = alpha*outscatterCoefficient;
		vec3 inscatter = c*alpha*inscatterCoefficient;

		inscatter *= gl_TexCoord[5].xyz*gl_TexCoord[5].w;
		outscatter *= gl_TexCoord[5].w;

		gl_FragColor = vec4(inscatter, outscatter);		
	}
	else
	{
		// good for foam
		float outscatter = alpha;
		vec3 inscatter = vec3(1.0)*outscatter;

		gl_FragColor = vec4(inscatter, outscatter);
	}
}
);

void RenderDiffuse(FluidRenderer* render, GLuint positions, GLuint velocities, GLuint indices, int n, float radius, float screenWidth, float screenAspect, float fov, Vec4 color, Vec3 lightPos, Vec3 lightTarget, Matrix44 lightTransform, GLuint shadowTex, float motionBlur, float inscatter, float outscatter, bool shadow, bool front)
{
	static int sprogram = -1;
	if (sprogram == -1)
		sprogram = CompileProgram(vertexDiffuseShader, fragmentDiffuseShader, geometryDiffuseShader);

	int thicknessScale = 1;

	if (sprogram)
	{
#if USE_HDR_DIFFUSE_BLEND
	
		{
			glVerify(glBindFramebuffer(GL_READ_FRAMEBUFFER, g_msaaFbo));
			glVerify(glBindFramebuffer(GL_DRAW_FRAMEBUFFER, render->mThicknessFbo));
			glVerify(glBlitFramebuffer(0, 0, render->mSceneWidth, render->mSceneHeight, 0, 0, render->mSceneWidth/thicknessScale, render->mSceneHeight/thicknessScale, GL_DEPTH_BUFFER_BIT, GL_NEAREST));

			glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
			glClear(GL_COLOR_BUFFER_BIT);		
		}
#endif

		glEnable(GL_POINT_SPRITE);
		glTexEnvi(GL_POINT_SPRITE, GL_COORD_REPLACE, GL_TRUE);
		glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
		glDepthMask(GL_FALSE);
		glEnable(GL_DEPTH_TEST);
		glEnable(GL_BLEND);
		glDisable(GL_CULL_FACE);
		glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);

		
		glUseProgram(sprogram);
		glUniform1f( glGetUniformLocation(sprogram, "motionBlurScale"), motionBlur);
		glUniform1f( glGetUniformLocation(sprogram, "diffusion"), 1.0f);
		glUniform1f( glGetUniformLocation(sprogram, "pointScale"), radius*1.0f);
		glUniform1f( glGetUniformLocation(sprogram, "pointRadius"), screenWidth / float(thicknessScale) / (2.0f*screenAspect*tanf(fov*0.5f)));		
		glUniform2fv( glGetUniformLocation(sprogram, "invViewport"), 1, Vec2(1.0f/screenWidth, screenAspect/screenWidth));
		glUniform4fv( glGetUniformLocation(sprogram, "color"), 1, color);
		glUniform1i( glGetUniformLocation(sprogram, "tex"), 0);
		glUniform1f( glGetUniformLocation(sprogram, "inscatterCoefficient"), inscatter);
		glUniform1f( glGetUniformLocation(sprogram, "outscatterCoefficient"), outscatter);

		GLint uLightTransform = glGetUniformLocation(sprogram, "lightTransform");
		glUniformMatrix4fv(uLightTransform, 1, false, lightTransform);

		GLint uLightPos = glGetUniformLocation(sprogram, "lightPos");
		glUniform3fv(uLightPos, 1, lightPos);
	
		GLint uLightDir = glGetUniformLocation(sprogram, "lightDir");
		glUniform3fv(uLightDir, 1, Normalize(lightTarget-lightPos));

		glUniform1f( glGetUniformLocation(sprogram, "spotMin"), gSpotMin);
		glUniform1f( glGetUniformLocation(sprogram, "spotMax"), gSpotMax);

		const Vec2 taps[] = 
		{ 
			Vec2(-0.326212f,-0.40581f),Vec2(-0.840144f,-0.07358f),
			Vec2(-0.695914f,0.457137f),Vec2(-0.203345f,0.620716f),
			Vec2(0.96234f,-0.194983f),Vec2(0.473434f,-0.480026f),
			Vec2(0.519456f,0.767022f),Vec2(0.185461f,-0.893124f),
			Vec2(0.507431f,0.064425f),Vec2(0.89642f,0.412458f),
			Vec2(-0.32194f,-0.932615f),Vec2(-0.791559f,-0.59771f) 
		};
	
		glVerify(glUniform2fv(glGetUniformLocation(sprogram, "shadowTaps"), 12, &taps[0].x));
		glVerify(glUniform1i(glGetUniformLocation(sprogram, "noiseTex"), 2));
		glVerify(glUniform1i(glGetUniformLocation(sprogram, "shadowTex"), 1));
		glVerify(glUniform1i(glGetUniformLocation(sprogram, "depthTex"), 0));
		glVerify(glUniform1i(glGetUniformLocation(sprogram, "front"), front));
		glVerify(glUniform1i(glGetUniformLocation(sprogram, "shadow"), shadow));

		// noise tex
		//glActiveTexture(GL_TEXTURE2);
		//glEnable(GL_TEXTURE_2D);
		//glBindTexture(GL_TEXTURE_2D, noiseTex);

		// shadow tex
		glActiveTexture(GL_TEXTURE1);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, shadowTex);
		glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE)); 
		//glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL)); 


		glActiveTexture(GL_TEXTURE0);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, render->mDepthSmoothTex);

		glClientActiveTexture(GL_TEXTURE1);
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		glVerify(glBindBuffer(GL_ARRAY_BUFFER, velocities));
		glTexCoordPointer(4, GL_FLOAT, sizeof(float)*4, 0);

		glEnableClientState(GL_VERTEX_ARRAY);			
		glBindBuffer(GL_ARRAY_BUFFER, positions);
		glVertexPointer(4, GL_FLOAT, sizeof(float)*4, 0);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indices);

		glDrawElements(GL_POINTS, n, GL_UNSIGNED_INT, 0);

		glUseProgram(0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glDisableClientState(GL_VERTEX_ARRAY);	
		glDisableClientState(GL_TEXTURE_COORD_ARRAY);	
		glDisable(GL_POINT_SPRITE);
		glDisable(GL_BLEND);
		glDepthMask(GL_TRUE);

		glVerify(glActiveTexture(GL_TEXTURE2));
		glVerify(glDisable(GL_TEXTURE_2D));
		glVerify(glActiveTexture(GL_TEXTURE1));
		glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE)); 
		glVerify(glDisable(GL_TEXTURE_2D));
		glVerify(glActiveTexture(GL_TEXTURE0));
		glVerify(glDisable(GL_TEXTURE_2D));

#if USE_HDR_DIFFUSE_BLEND
		
			{
			glVerify(glBindFramebuffer(GL_FRAMEBUFFER, g_msaaFbo));
			glVerify(glViewport(0, 0, int(screenWidth), int(screenWidth/screenAspect)));

			//glClear(GL_COLOR_BUFFER_BIT);
			glUseProgram(0);
			glDisable(GL_DEPTH_TEST);
			glEnable(GL_BLEND);
			glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
			glDepthMask(GL_FALSE);
			glDisable(GL_CULL_FACE);
			
			glVerify(glActiveTexture(GL_TEXTURE0));
			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, render->mThicknessTex);

			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			glLoadIdentity();
		
			glMatrixMode(GL_PROJECTION);
			glPushMatrix();
			glLoadIdentity();
			gluOrtho2D(-1.0f, 1.0f, -1.0f, 1.0);

			RenderFullscreenQuad();

			glMatrixMode(GL_MODELVIEW);
			glPopMatrix();
		
			glMatrixMode(GL_PROJECTION);
			glPopMatrix();

			glDepthMask(GL_TRUE);
		}
#endif

	}
}

