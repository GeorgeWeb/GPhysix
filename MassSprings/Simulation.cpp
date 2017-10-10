#include "Simulation.hpp"

using namespace GPhysix;

Simulation::Simulation() : 
	m_app(Application::Application()),
	m_accumulator(0.0f),
	m_currentTime(static_cast<GLfloat>(glfwGetTime()))
{
	// initialize application
	m_app.initRender();
	Application::camera.setCameraPosition(glm::vec3(0.0f, 10.0f, 30.0f));
}

Simulation::~Simulation()
{
}

void Simulation::CreateParticle(Particle &particle, const glm::vec3 &translation)
{
	// set particle properties
	particle.scale(glm::vec3(3.f, 3.f, 3.f));
	particle.translate(translation);
	particle.setVel(glm::vec3(0.0f, 1.0f, 0.0f));
	particle.getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag"));
}

void Simulation::IntegrateSI(Particle &particle, const float deltaTime)
{
	particle.getVel() += particle.getAcc() * deltaTime;
	particle.getPos() += particle.getVel() * deltaTime;
}

void Simulation::IntegrateEX(Particle &particle, const float deltaTime)
{
	particle.getPos() += particle.getVel() * deltaTime;
	particle.getVel() += particle.getAcc() * deltaTime;
}

bool Simulation::inBoundaries(const Mesh &particle, const Mesh & boundingBox, const int & coordIndex)
{
	bool minCheck = particle.getPos()[coordIndex] >= boundingBox.getPos()[coordIndex] + .5f;
	bool maxCheck = particle.getPos()[coordIndex] <= boundingBox.getPos()[coordIndex] + boundingBox.getScale()[coordIndex][coordIndex];

	return minCheck && maxCheck;
}

void Simulation::CollisionDetection(const Mesh &boundingBox, Particle &particle)
{
	// Loop thruogh all sides of the particle object
	for (int i = 0; i < 3; i++)
	{
		// SIMD optimized simple-AABB collsiion detection
		if (!inBoundaries(particle.getMesh(), boundingBox, i))
		{
			// Change particle's direction to its opposite direction vector
			particle.getVel()[i] *= -1.0f;
			// Minimums check to stabilize the particle
			particle.getPos()[i] = (particle.getPos()[i] < boundingBox.getPos()[i] + .5f)
				? boundingBox.getPos()[i] + .5f : particle.getPos()[i];
			// Maximums check to stabilize the particle
			particle.getPos()[i] = (particle.getPos()[i] > boundingBox.getPos()[i] + boundingBox.getScale()[i][i])
				? boundingBox.getPos()[i] + boundingBox.getScale()[i][i] : particle.getPos()[i];
		}
	}
}

void Simulation::Exit()
{
	m_app.terminate();
}

void Simulation::BeginFrames()
{
	m_app.clear();

	// update frames
	GLfloat newTime = (GLfloat)glfwGetTime();
	GLfloat frameTime = newTime - m_currentTime;
	m_currentTime = newTime;
	m_accumulator += frameTime;

	// manage interaction
	m_app.doMovement(deltaTime);
}

void Simulation::EndFrames()
{
	m_app.display();
}

