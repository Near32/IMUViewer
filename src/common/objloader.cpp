#include "objloader.h"

bool loadOBJ( const char* path, std::vector<glm::vec3>& out_vertices, std::vector<glm::vec2>& out_uvs, std::vector<glm::vec3>& out_normals)
{
	std::vector<unsigned int> vertexIndices, uvIndices, normalIndices;
	std::vector<glm::vec3> temp_vertices;
	std::vector<glm::vec2> temp_uvs;
	std::vector<glm::vec3> temp_normals;
	
	FILE* file = fopen(path,"r");
	
	if(file == NULL)
	{
		std::cerr << "Impossible d'ouvrir le fichier obj : " << std::string(path) << std::endl;
		return false;
	}
	
	while(1)
	{
		char line[128];
		
		int res = fscanf(file,"%s", line);
		
		if(res == EOF)
			break;
			
		//else :
		if( strcmp(line,"v") == 0)
		{
			//then it is a vertex :
			glm::vec3 vertex;
			fscanf(file, "%f %f %f\n", &vertex.x, &vertex.y, &vertex.z);
			temp_vertices.push_back(vertex);
		}
		else if( strcmp(line,"vt") == 0)
		{
			//then it is an uv :
			glm::vec2 uv;
			fscanf(file, "%f %f\n", &uv.x, &uv.y);
			temp_uvs.push_back(uv);
		}
		else if(strcmp(line,"vn") == 0)
		{
			//then it is a normal :
			glm::vec3 normal;
			fscanf(file, "%f %f %f\n", &normal.x, &normal.y, &normal.z);
			temp_normals.push_back(normal);
		}
		else if(strcmp(line,"f") == 0)
		{
			//then it is a face:
			std::string v1,v2,v3;
			unsigned int vertexIndex[3], uvIndex[3], normalIndex[3];
    		int matches = fscanf(file, "%d %d %d\n", &vertexIndex[0], &vertexIndex[1], &vertexIndex[2]);
    		
    		if (matches != 3)
    		{
        		std::cerr << "File can't be read by our simple parser." << std::endl;
        		return false;
    		}
    		
    		vertexIndices.push_back(vertexIndex[0]);
    		vertexIndices.push_back(vertexIndex[1]);
    		vertexIndices.push_back(vertexIndex[2]);
    		
    		//TODO : UVS + NORMALS
    	}	
		
		
	
	}
	
	
	//indexation :
	
	for(int i=0;i<vertexIndices.size();i++)
	{
		out_vertices.push_back( (glm::vec3)temp_vertices[ vertexIndices[i]-1 ] );
		//TODO : UVS + NORMALS
	}
	
	fclose(file);
	
	return true;
} 
