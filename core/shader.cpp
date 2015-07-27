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
#include "shader.h"

#include "types.h"
#include "maths.h"
#include "platform.h"
#include "tga.h"

#include <stdarg.h>
#include <stdio.h>

#define WITH_GLEW

void GlslPrintShaderLog(GLuint obj)
{
#if !PLATFORM_IOS
	int infologLength = 0;
	int charsWritten  = 0;
	char *infoLog;
	
	GLint result;
	glGetShaderiv(obj, GL_COMPILE_STATUS, &result);

	// only print log if compile fails
	if (result == GL_FALSE)
	{
		glGetShaderiv(obj, GL_INFO_LOG_LENGTH,&infologLength);
	
		if (infologLength > 1)
		{
			infoLog = (char *)malloc(infologLength);
			glGetShaderInfoLog(obj, infologLength, &charsWritten, infoLog);
			printf("%s\n",infoLog);
			free(infoLog);
		}
	}
#endif
}

void glAssert(const char* msg, long line, const char* file)
{
	struct glError 
	{
		GLenum code;
		const char* name;
	};

	static const glError errors[] = {	{GL_NO_ERROR, "No Error"},
										{GL_INVALID_ENUM, "Invalid Enum"},
										{GL_INVALID_VALUE, "Invalid Value"},
										{GL_INVALID_OPERATION, "Invalid Operation"}
#if OGL1
										,{GL_STACK_OVERFLOW, "Stack Overflow"},
										{GL_STACK_UNDERFLOW, "Stack Underflow"},
										{GL_OUT_OF_MEMORY, "Out Of Memory"}
#endif
									};

	GLenum e = glGetError();

	if (e == GL_NO_ERROR)
	{
		return;
	}
	else
	{
		const char* errorName = "Unknown error";

		// find error message
		for (uint32_t i=0; i < sizeof(errors)/sizeof(glError); i++)
		{
			if (errors[i].code == e)
			{
				errorName = errors[i].name;
			}
		}

		printf("OpenGL: %s - error %s in %s at line %d\n", msg, errorName, file, int(line));
		assert(0);
	}
}

void PreProcessShader(const char* filename, std::string& source)
{
	// load source
	FILE* f = fopen(filename, "r");

	if (!f)
	{
		printf("Could not open shader file for reading: %s\n", filename);
		return;
	}

	// add lines one at a time handling include files recursively
	while (!feof(f))
	{
		char buf[1024];

		if (fgets(buf, 1024, f) != NULL)
		{	
			// test for #include
			if (strncmp(buf, "#include", 8) == 0)
			{	
				const char* begin = strchr(buf, '\"');
				const char* end = strrchr(buf, '\"');

				if (begin && end && (begin != end))
				{
					// lookup file relative to current file
					PreProcessShader((StripFilename(filename) + std::string(begin+1, end)).c_str(), source);
				}
			}
			else
			{
				// add line to output
				source += buf;
			}
		}
	}

	fclose(f);
}

GLuint CompileProgramFromFile(const char *vertexPath, const char *fragmentPath)
{
	std::string vsource;
	PreProcessShader(vertexPath, vsource);

	std::string fsource;
	PreProcessShader(fragmentPath, fsource);

	return CompileProgram(vsource.c_str(), fsource.c_str());
}

GLuint CompileProgram(const char *vsource, const char *fsource, const char* gsource)
{

	GLuint vertexShader = GLuint(-1);
	GLuint geometryShader = GLuint(-1); 
	GLuint fragmentShader = GLuint(-1); 

	GLuint program = glCreateProgram();

	if (vsource)
	{
		vertexShader = glCreateShader(GL_VERTEX_SHADER);
		glShaderSource(vertexShader, 1, &vsource, 0);
		glCompileShader(vertexShader);
		GlslPrintShaderLog(vertexShader);
		glAttachShader(program, vertexShader);
	}

	if (fsource)
	{
		fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
		glShaderSource(fragmentShader, 1, &fsource, 0);
		glCompileShader(fragmentShader);
		GlslPrintShaderLog(fragmentShader);
		glAttachShader(program, fragmentShader);
	}

	if (gsource)
	{
		geometryShader = glCreateShader(GL_GEOMETRY_SHADER);
		glShaderSource(geometryShader, 1, &gsource, 0);
		glCompileShader(geometryShader);
		GlslPrintShaderLog(geometryShader);

		// hack, force billboard gs mode
		glAttachShader(program, geometryShader);
		glProgramParameteriEXT ( program, GL_GEOMETRY_VERTICES_OUT_EXT, 4 ) ; 
		glProgramParameteriEXT ( program, GL_GEOMETRY_INPUT_TYPE_EXT, GL_POINTS ) ; 
		glProgramParameteriEXT ( program, GL_GEOMETRY_OUTPUT_TYPE_EXT, GL_TRIANGLE_STRIP ) ; 
	}

	glLinkProgram(program);

	// check if program linked
	GLint success = 0;
	glGetProgramiv(program, GL_LINK_STATUS, &success);

	if (!success) {
		char temp[256];
		glGetProgramInfoLog(program, 256, 0, temp);
		printf("Failed to link program:\n%s\n", temp);
		glDeleteProgram(program);
		program = 0;
	}
	else
	{
		printf("Created shader program: %d\n", program);
	}

	return program;
}

#if _WIN32

GLuint CompileProgram(const char *vsource, const char* csource, const char* esource, const char* fsource)
{
	GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
	GLuint controlShader = glCreateShader(GL_TESS_CONTROL_SHADER);
	GLuint evaluationShader = glCreateShader(GL_TESS_EVALUATION_SHADER);
	GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

	glShaderSource(vertexShader, 1, &vsource, 0);
	glShaderSource(controlShader, 1, &csource, 0);
	glShaderSource(evaluationShader, 1, &esource, 0);
	glShaderSource(fragmentShader, 1, &fsource, 0);
	
	glCompileShader(vertexShader);
	GlslPrintShaderLog(vertexShader);
	
	glCompileShader(controlShader);
	GlslPrintShaderLog(controlShader);

	glCompileShader(evaluationShader);
	GlslPrintShaderLog(evaluationShader);

	glCompileShader(fragmentShader);
	GlslPrintShaderLog(fragmentShader);

	GLuint program = glCreateProgram();

	glAttachShader(program, vertexShader);
	glAttachShader(program, controlShader);
	glAttachShader(program, evaluationShader);
	glAttachShader(program, fragmentShader);

	glLinkProgram(program);

	// check if program linked
	GLint success = 0;
	glGetProgramiv(program, GL_LINK_STATUS, &success);

	if (!success) {
		char temp[256];
		glGetProgramInfoLog(program, 256, 0, temp);
		printf("Failed to link program:\n%s\n", temp);
		glDeleteProgram(program);
		program = 0;
	}
	else
	{
		printf("Created shader program: %d\n", program);
	}

	return program;
}

#endif

void DrawPlane(const Vec4& p, bool color)
{
	Vec3 u, v;
	BasisFromVector(Vec3(p.x, p.y, p.z), &u, &v);

	Vec3 c = Vec3(p.x, p.y, p.z)*-p.w;
	
	glBegin(GL_QUADS);

	if (color)
		glColor3fv(p*0.5f + Vec4(0.5f, 0.5f, 0.5f, 0.5f));

	float kSize = 200.0f;

	// draw a grid of quads, otherwise z precision suffers
	for (int x=-3; x <= 3; ++x)
	{
		for (int y=-3; y <= 3; ++y)
		{
			Vec3 coff = c + u*float(x)*kSize*2.0f + v*float(y)*kSize*2.0f;

			glTexCoord2f(1.0f, 1.0f);
			glNormal3f(p.x, p.y, p.z);
			glVertex3fv(coff + u*kSize + v*kSize);

			glTexCoord2f(0.0f, 1.0f);
			glNormal3f(p.x, p.y, p.z);
			glVertex3fv(coff - u*kSize + v*kSize);

			glTexCoord2f(0.0f, 0.0f);
			glNormal3f(p.x, p.y, p.z);
			glVertex3fv(coff - u*kSize - v*kSize);

			glTexCoord2f(1.0f, 0.0f);
			glNormal3f(p.x, p.y, p.z);
			glVertex3fv(coff + u*kSize - v*kSize);
		}
	}

	glEnd();

}

void DrawStringA(int x, int y, const char* s)
{
	glRasterPos2d(x, y);
	while (*s)
	{
		glutBitmapCharacter(GLUT_BITMAP_8_BY_13, *s);
		++s;
	}
}

void DrawStringS(int x, int y, const char* s)
{
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glTranslatef(float(x), float(y), 0.0f);
	
	// make it match GLUT_BITMAP_8_BY_13
	const float kSize = 8.0f/104.76f;

	glScalef(kSize, kSize, kSize);

#if _APPLE_
	while (*s)
	{
		glutStrokeCharacter(GLUT_STROKE_ROMAN, *s);
		++s;
	}
#else
	glutStrokeString(GLUT_STROKE_MONO_ROMAN, (const unsigned char*)s);	
#endif

	glPopMatrix();
}


void DrawString(int x, int y, const char* s, ...)
{
	char buf[2048];

	va_list args;

	va_start(args, s);
	vsnprintf(buf, 2048, s, args);
	va_end(args);

	DrawStringA(x ,y, buf);
}

void DrawStringStroked(int x, int y, Vec3 color, const char* s, ...)
{
	char buf[2048];

	va_list args;

	va_start(args, s);
	vsnprintf(buf, 2048, s, args);
	va_end(args);

	glColor3fv(color);
	DrawStringS(x ,y, buf);
}

void DrawFrustum(const Mat44& projToWorld)
{
	// transform corner points back to world space
	Point3 corners[] = { 
		Point3(-1.0f, 1.0f, 1.0f),
		Point3(1.0f, 1.0f, 1.0f),
		Point3(1.0f, -1.0f, 1.0f),
		Point3(-1.0f, -1.0f, 1.0f),

		Point3(-1.0f, 1.0f, -1.0f),
		Point3(1.0f, 1.0f, -1.0f),
		Point3(1.0f, -1.0f, -1.0f),
		Point3(-1.0f, -1.0f, -1.0f) };


	glDisable(GL_BLEND);
	glBegin(GL_LINE_STRIP);
	glColor3f(0.0f, 1.0f, 0.0f);

	for (int i=0; i < 4; ++i)
	{
		Point3 p = projToWorld*corners[i];
		glVertex3fv(p);
	}

	glVertex3fv(projToWorld*corners[0]);

	glEnd();

	glBegin(GL_LINE_STRIP);
	glColor3f(0.0f, 1.0f, 0.0f);

	for (int i=4; i < 8; ++i)
	{
		Point3 p = projToWorld*corners[i];
		glVertex3fv(p);
	}

	glVertex3fv(projToWorld*corners[4]);

	glEnd();
}

void DrawBoundingBox(const Vec3& lower, const Vec3& upper)
{
	glBegin(GL_LINES);
	// base
	glVertex3fv(lower);
	glVertex3fv(Vec3(upper.x, lower.y, lower.z));

	glVertex3fv(Vec3(upper.x, lower.y, lower.z));
	glVertex3fv(Vec3(upper.x, lower.y, upper.z));

	glVertex3fv(Vec3(upper.x, lower.y, upper.z));
	glVertex3fv(Vec3(lower.x, lower.y, upper.z));

	glVertex3fv(Vec3(lower.x, lower.y, upper.z));
	glVertex3fv(lower);

	// top
	glVertex3fv(Vec3(lower.x, upper.y, lower.z));
	glVertex3fv(Vec3(upper.x, upper.y, lower.z));

	glVertex3fv(Vec3(upper.x, upper.y, lower.z));
	glVertex3fv(Vec3(upper.x, upper.y, upper.z));

	glVertex3fv(Vec3(upper.x, upper.y, upper.z));
	glVertex3fv(Vec3(lower.x, upper.y, upper.z));

	glVertex3fv(Vec3(lower.x, upper.y, upper.z));
	glVertex3fv(Vec3(lower.x, upper.y, lower.z));


	// struts
	glVertex3fv(Vec3(lower.x, upper.y, lower.z));
	glVertex3fv(Vec3(lower.x, lower.y, lower.z));

	glVertex3fv(Vec3(upper.x, upper.y, lower.z));
	glVertex3fv(Vec3(upper.x, lower.y, lower.z));

	glVertex3fv(Vec3(upper.x, upper.y, upper.z));
	glVertex3fv(Vec3(upper.x, lower.y, upper.z));

	glVertex3fv(Vec3(lower.x, upper.y, upper.z));
	glVertex3fv(Vec3(lower.x, lower.y, upper.z));

	glEnd();

}


GLuint LoadCubeTexture(const char* baseName)
{

	TgaImage img;
	std::string base(baseName);
	
	GLuint texture;
	glVerify(glActiveTexture(GL_TEXTURE0));
    glVerify(glEnable(GL_TEXTURE_CUBE_MAP));
    glVerify(glGenTextures(1, &texture));
    glVerify(glBindTexture(GL_TEXTURE_CUBE_MAP, texture));
	glVerify(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_GENERATE_MIPMAP, GL_TRUE));
    glVerify(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
    glVerify(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR)); 
    glVerify(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    glVerify(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
    glVerify(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE));

	if (TgaLoad((base + "_bk.tga").c_str(), img))
		glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_Z, 0, GL_RGBA, img.m_width, img.m_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, img.m_data);

	if (TgaLoad((base + "_ft.tga").c_str(), img))
		glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, 0, GL_RGBA, img.m_width, img.m_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, img.m_data);

	if (TgaLoad((base + "_lf.tga").c_str(), img))
		glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_X, 0, GL_RGBA, img.m_width, img.m_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, img.m_data);

	if (TgaLoad((base + "_rt.tga").c_str(), img))
		glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X, 0, GL_RGBA, img.m_width, img.m_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, img.m_data);

	if (TgaLoad((base + "_dn.tga").c_str(), img))
		glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_Y, 0, GL_RGBA, img.m_width, img.m_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, img.m_data);

	if (TgaLoad((base + "_up.tga").c_str(), img))
		glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, 0, GL_RGBA, img.m_width, img.m_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, img.m_data);

	//...
	glVerify(glDisable(GL_TEXTURE_CUBE_MAP));

	return texture;
}

void Extrude(const Vec3* points, int numPoints, std::vector<Vec3>& vertices, std::vector<Vec3>& normals, std::vector<int>& triangles, float radius, int resolution, int smoothing)
{
	if (numPoints < 2)
		return;

	Vec3 u, v;
	Vec3 w = SafeNormalize(Vec3(points[1])-Vec3(points[0]), Vec3(0.0f, 1.0f, 0.0f));

	BasisFromVector(w, &u, &v);

	Matrix44 frame;
	frame.SetCol(0, Vec4(u.x, u.y, u.z, 0.0f));
	frame.SetCol(1, Vec4(v.x, v.y, v.z, 0.0f));
	frame.SetCol(2, Vec4(w.x, w.y, w.z, 0.0f));
	frame.SetCol(3, Vec4(0.0f, 0.0f, 0.0f, 1.0f));	

	for (int i=0; i < numPoints -1; ++i)
	{
		Vec3 next;

		if (i < numPoints -1)			
			next = Normalize(Vec3(points[i+1])-Vec3(points[i-1]));
		else
			next = Normalize(Vec3(points[i])-Vec3(points[i-1]));

		int a = Max(i-1, 0);
		int b = i;
		int c = Min(i+1, numPoints -1);
		int d = Min(i+2, numPoints -1);

		Vec3 p1 = Vec3(points[b]);
		Vec3 p2 = Vec3(points[c]);
		Vec3 m1 = 0.5f*(Vec3(points[c]) - Vec3(points[a]));
		Vec3 m2 = 0.5f*(Vec3(points[d]) - Vec3(points[b]));		

		// ensure last segment handled correctly
		int segments = (i < numPoints-2)?smoothing:smoothing+1;

		for (int s=0; s < segments; ++s)
		{
			Vec3 pos = HermiteInterpolate(p1, p2, m1, m2, s/float(smoothing));
			Vec3 dir = Normalize(HermiteTangent(p1, p2, m1, m2, s/float(smoothing)));

			Vec3 cur = frame.GetAxis(2);
			const float angle = acosf(Dot(cur, dir));

			// if parallel then don't need to do anything
			if (fabsf(angle) > 0.001f)
				frame = RotationMatrix(angle, SafeNormalize(Cross(cur, dir)))*frame;

			int startIndex = vertices.size();

			for (int c=0; c < resolution; ++c)
			{			
				float angle = k2Pi / resolution;

				// transform position and normal to world space
				Vec4 v = frame*Vec4(cosf(angle*c), sinf(angle*c), 0.0f, 0.0f);
			
				vertices.push_back(Vec3(v)*radius + pos);
				normals.push_back(Vec3(v));
			}

			// output triangles
			if (startIndex != 0)
			{
				for (int i=0; i < resolution; ++i)
				{
					int curIndex = startIndex + i;
					int nextIndex = startIndex + (i+1)%resolution; 

					triangles.push_back(curIndex);
					triangles.push_back(curIndex-resolution);
					triangles.push_back(nextIndex-resolution);
				
					triangles.push_back(nextIndex-resolution);
					triangles.push_back(nextIndex);
					triangles.push_back(curIndex);
				}	
			}
		}
	}
}
