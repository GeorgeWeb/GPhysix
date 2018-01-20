#pragma once

// Std. Includes
#include <string>
#include <time.h>
#include <functional>

// Other Libs
#include "SOIL2/SOIL2.h"

// project includes
#include "Application.h"
#include "Shader.h"
#include "Force.h"
#include "Particle.h"
#include "RigidBody.h"

// MASS_SPRINGS, RIGID_BODY, DOMINO
#define DOMINO

namespace GPhysix
{
	namespace Time
	{
		static constexpr float deltaTime = 0.003f;
	}
	
	template<class BodyType>
	class Simulation { };

	// Particle-specific designed Simulation class
	template<>
	class Simulation<Particle>
	{
		private:
			Application m_app;

			float m_accumulator;
			GLfloat m_currentTime;

		public:
			Simulation();
			~Simulation() { }
	
			inline Application &getApp() { return m_app; };
			inline float &getAccumultor() { return m_accumulator; };
			inline const bool &IsRunning() { return !glfwWindowShouldClose(m_app.getWindow()); }
			
			inline void Draw(const Mesh &mesh) { m_app.draw(mesh); }
			inline void Draw(Particle &particle) { m_app.draw(particle.getMesh()); }

			void BeginFrames();
			void EndFrames();
			void Exit();
		
			void CreateParticle(Particle &particle, const glm::vec3 &translation);
			bool inBoundaries(const Mesh &particleMesh, const Mesh &boundingBox, const int coordIndex);
			void CollisionDetection(const Mesh &boundingBox, Particle &particle);
			void CollisionDetectionForCloth(const Mesh &boundingBox, Particle &particle);
	};

	// RigidBody-specific designed Simulation class
	template<>
	class Simulation<RigidBody>
	{
		private:
			Application m_app;

			float m_accumulator;
			const float m_timeMultiplier = 0.25f;
			GLfloat m_currentTime;
		
		public:
			Simulation();
			~Simulation() { }
		
			inline Application &getApp() { return m_app; };
			inline float &getAccumultor() { return m_accumulator; };
			inline const bool &IsRunning() { return !glfwWindowShouldClose(m_app.getWindow()); }

			inline void Draw(const Mesh& mesh) { m_app.draw(mesh); }
			inline void Draw(RigidBody* rigBody) { m_app.draw(rigBody->getMesh()); }

			void BeginFrames();
			void EndFrames();
			void Exit();
	};
}