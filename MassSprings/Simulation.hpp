#pragma once

// Std. Includes
#include <string>
#include <time.h>
#include <array>
#include <algorithm>
#include <functional>

// Other Libs
#include "SOIL2/SOIL2.h"

// project includes
#include "../Application.h"
#include "../Shader.h"
#include "../Particle.h"
#include "../Gravity.h"
#include "../Drag.h"
#include "../Hook.h"

namespace GPhysix
{
	class Simulation
	{
		private:
			Application m_app;

			float m_accumulator;
			GLfloat m_currentTime;
		public:
			static constexpr float deltaTime = 0.0005f;

			Simulation();
			~Simulation();
		public:
			inline Application &getApp() { return m_app; };
			inline float &getAccumultor() { return m_accumulator; };
			inline const bool &IsRunning() { return !glfwWindowShouldClose(m_app.getWindow()); }
			
			inline void Draw(const Mesh &mesh) { m_app.draw(mesh); }
			inline void Draw(Particle &particle) { m_app.draw(particle.getMesh()); }

			void BeginFrames();
			void EndFrames();
			void Exit();
		public:
			void CreateParticle(Particle &particle, const glm::vec3 &translation);
			void IntegrateSI(Particle &particle, const float deltaTime);
			void IntegrateEX(Particle &particle, const float deltaTime);
			bool inBoundaries(const Mesh &particle, const Mesh &boundingBox, const int coordIndex);
			void CollisionDetection(const Mesh &boundingBox, Particle &particle);
			void CollisionDetectionForCloth(const Mesh &boundingBox, Particle &particle);
	};
}