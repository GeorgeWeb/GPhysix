#pragma once
#include "Common.h"
#include "OBJloader.h"
#include "Shader.h"

#include <string>
#include <vector>

namespace GPhysix
{
	class Vertex
	{
		public:
			Vertex() : m_coord(glm::vec3()) {}
			Vertex(const glm::vec3& coord) : m_coord(coord) {}
			
			inline glm::vec3& getCoord() { return m_coord; }
			
			inline friend bool operator==(Vertex& t_lhs, Vertex& t_rhs);
			inline friend bool operator!=(Vertex& t_lhs, Vertex& t_rhs) { return !(t_lhs == t_rhs); }
			
			inline bool operator<(const Vertex& other) const
			{
				return m_coord.x != other.m_coord.x ? m_coord.x < other.m_coord.x
					 : m_coord.y != other.m_coord.y ? m_coord.y < other.m_coord.y
					 : m_coord.z < other.m_coord.z;
			}

		private:
			glm::vec3 m_coord;
	};

	enum MeshBufferPositions
	{
		POSITION_VB,
		TEXCOORD_VB,
		NORMAL_VB,
		INDEX_VB
	};

	class Mesh
	{
		public:
			enum MeshType
			{
				TRIANGLE,
				QUAD,
				CUBE
			};

			// default constructor creates a horizontal plane or dimensions 1x1 centered on the origin
			// default constructor creates a horizontal plane or dimensions 1 x 1 centered on the origin
			Mesh();
			// construct a mesh from a type
			Mesh(MeshType type);
			// create mesh from a .obj file
			Mesh(const std::string& fileName);
			// create a mesh from an array of vertices and normals
			Mesh(float[]);
			
			virtual ~Mesh() { }

			// getModel computes the model matrix any time it is required
			glm::vec3 getPos() const { return getTranslate()[3]; }
			glm::mat4 getModel() const { return getTranslate() * getRotate() * getScale(); }
			glm::mat4 getTranslate() const { return m_translate; }
			glm::mat4 getRotate() const { return m_rotate; }
			glm::mat4 getScale() const { return m_scale; }
			glm::vec3 getScaleVec() const { return glm::vec3(m_scale[0].x, m_scale[1].y, m_scale[2].z); }

			Shader getShader() const { return m_shader; }
			GLuint getVertexArrayObject() const { return m_vertexArrayObject; }
			GLuint getVertexBuffer() const { return m_vertexBuffer; }
			GLuint getNormalBuffer() const { return m_normalBuffer; }

			unsigned int getNumIndices() const { return m_numIndices; }
			const std::vector<Vertex>& getVertices() { return m_vertices; }

			// set position of mesh center to specified 3D position vector
			inline void setPos(const glm::vec3 &position)
			{
				m_translate[3][0] = position[0];
				m_translate[3][1] = position[1];
				m_translate[3][2] = position[2];
			}
			// set i_th coordinate of mesh center to float p (x: i=0, y: i=1, z: i=2)
			inline void setPos(int i, float p) { m_translate[3][i] = p; }

			// rotate mesh by mat4
			inline void setRotateMat4(const glm::mat4& t_mat4) { m_rotate = t_mat4; }

			// allocate shader to mesh
			inline void setShader(const Shader &shader) { m_shader = shader; m_shader.Use(); }

			/*
			** INITIALISATION AND UTILITY METHODS
			*/

			// create vector of unique vertices (no duplicates) from vector of all mesh vertices
			void createUniqueVertices();

			// initialise transform matrices to identity
			void initTransform();
			// create mesh from vertices
			void initMesh(Vertex* vertices, glm::vec3* normals);
			// create mesh from model (typically loaded from a file)
			void InitMesh(const IndexedModel& model);
			// load .obj file
			void loadOBJ(const char* path, std::vector <glm::vec3>& out_vertices, 
				std::vector <glm::vec2>& out_uvs, std::vector <glm::vec3>& out_normals);
			
			// calculate normals from array of vertices
			//void calculateNormals();

			/*
			** TRANSFORMATION METHODS
			*/

			// translate mesh by a vector
			inline void translate(const glm::vec3& vect) { m_translate = glm::translate(m_translate, vect); }
			// rotate mesh by a vector
			inline void rotate(const float& angle, const glm::vec3& vect) { m_rotate = glm::rotate(m_rotate, angle, vect); }
			// scale mesh by a vector
			inline void scale(const glm::vec3& vect) { m_scale = glm::scale(m_scale, vect); }
			
			inline glm::vec3 calculateNormals() {
				return glm::vec3(1.0f);
			}

			GLuint m_vertexArrayObject;
			GLuint m_vertexBuffer;
			GLuint m_normalBuffer;

		private:
			/*
			enum
			{
				POSITION_VB,
				NUM_BUFFERS
			};
			*/
	
			//GLuint m_vertexArrayBuffers[NUM_BUFFERS];
			glm::mat4 m_translate;
			glm::mat4 m_rotate;
			glm::mat4 m_scale;

			unsigned int m_numIndices;
			std::vector<Vertex> m_vertices;

			Shader m_shader;
	};
}