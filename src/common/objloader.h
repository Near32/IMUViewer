#ifndef OBJLOADER_H
#define OBJLOADER_H

#include <iostream>
#include <vector>
#include <cstdlib>
#include <stdio.h>
#include <string.h>

namespace glm
{
	typedef struct vec3
	{
		float x;
		float y;
		float z;	
	} vec3;
	
	typedef struct vec2
	{
		float x;
		float y;	
	} vec2;

};
bool loadOBJ( const char* path, std::vector<glm::vec3>& out_vertices, std::vector<glm::vec2>& out_uvs, std::vector<glm::vec3>& out_normals);
#endif
