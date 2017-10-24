#include "Simulation.h"

using namespace GPhysix;

Simulation<Particle>::Simulation() : 
	m_app(Application::Application()),
	m_accumulator(0.0f),
	m_currentTime(static_cast<GLfloat>(glfwGetTime()))
{
	// initialize application
	m_app.initRender();
	Application::camera.setCameraPosition(glm::vec3(0.0f, 5.0f, 10.0f));
}

void Simulation<Particle>::CreateParticle(Particle& particle, const glm::vec3& translation)
{
	// set particle properties
	particle.scale(glm::vec3(1.f, 1.f, 1.f));
	particle.translate(translation);
	particle.setVel(glm::vec3(0.0f, 1.0f, 0.0f));
	particle.getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag"));
}

void Simulation<Particle>::IntegrateSI(Particle& particle)
{
	particle.getVel() += particle.getAcc() * Time::deltaTime;
	particle.getPos() += particle.getVel() * Time::deltaTime;
}

void Simulation<Particle>::IntegrateEX(Particle& particle)
{
	particle.getPos() += particle.getVel() * Time::deltaTime;
	particle.getVel() += particle.getAcc() * Time::deltaTime;
}

bool Simulation<Particle>::inBoundaries(const Mesh& particleMesh, const Mesh& boundingBox, const int coordIndex)
{
	bool minCheck = particleMesh.getPos()[coordIndex] >= boundingBox.getPos()[coordIndex] + .5f;
	bool maxCheck = particleMesh.getPos()[coordIndex] <= boundingBox.getPos()[coordIndex] + boundingBox.getScale()[coordIndex][coordIndex];

	return minCheck && maxCheck;
}

void Simulation<Particle>::CollisionDetection(const Mesh& boundingBox, Particle& particle)
{
	// Loop thruogh all sides of the particle object
	for (int i = 0; i < 3; i++)
	{
		// SIMD optimized simple-AABB collsiion detection
		if (!inBoundaries(particle.getMesh(), boundingBox, i))
		{
			// Change particle's direction to its opposite direction vector
			particle.getVel()[i] *= -0.8f; ///> TODO: Implement friction force instead
			// Minimums check to stabilize the particle
			particle.getPos()[i] = (particle.getPos()[i] < boundingBox.getPos()[i] + .5f)
				? boundingBox.getPos()[i] + .5f : particle.getPos()[i];
			// Maximums check to stabilize the particle
			particle.getPos()[i] = (particle.getPos()[i] > boundingBox.getPos()[i] + boundingBox.getScale()[i][i])
				? boundingBox.getPos()[i] + boundingBox.getScale()[i][i] : particle.getPos()[i];
		}
	}
}

void Simulation<Particle>::CollisionDetectionForCloth(const Mesh &boundingBox, Particle &particle)
{
	// Loop thruogh all sides of the particle object
	for (int i = 0; i < 3; i++)
	{
		// SIMD optimized simple-AABB collsiion detection
		if (!inBoundaries(particle.getMesh(), boundingBox, i))
		{
			// Change particle's direction to its opposite direction vector
			particle.getVel()[i] *= -.9f; ///> TODO: Implement friction force instead
			// Minimums check to stabilize the particle
			if (particle.getPos()[i] < boundingBox.getPos()[i] + .5f)
			{
				particle.getPos()[i] = boundingBox.getPos()[i] + .5f;
				particle.getVel() *= -.9f; ///> TODO: Implement friction force instead
			}
		}
	}
}

void Simulation<Particle>::Exit()
{
	m_app.terminate();
}

void Simulation<Particle>::BeginFrames()
{
	m_app.clear();

	// update frames
	GLfloat newTime = static_cast<GLfloat>(glfwGetTime());
	GLfloat frameTime = newTime - m_currentTime;
	m_currentTime = newTime;
	m_accumulator += frameTime;
	m_accumulator *= 1.0f;
	// manage interaction
	m_app.doMovement(Time::deltaTime);
}

void Simulation<Particle>::EndFrames()
{
	m_app.display();
}