#include "Simulation.h"

using namespace GPhysix;

Simulation<RigidBody>::Simulation() :
	m_app(Application::Application()),
	m_accumulator(0.0f),
	m_currentTime(static_cast<GLfloat>(glfwGetTime()))
{
	// initialize application
	m_app.initRender();
	Application::camera.setCameraPosition(glm::vec3(0.0f, 5.0f, 10.0f));
}

void Simulation<RigidBody>::Integrate(RigidBody& rigBody)
{
	rigBody.getVel() += rigBody.getAcc() * Time::deltaTime;
	rigBody.getPos() += rigBody.getVel() * Time::deltaTime;
	
}

void Simulation<RigidBody>::Exit()
{
	m_app.terminate();
}

void Simulation<RigidBody>::BeginFrames()
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

void Simulation<RigidBody>::EndFrames()
{
	m_app.display();
}