// Std. Includes
#include <string>
#include <time.h>
#include <array>
#include <algorithm>
#include <functional>

// Other Libs
#include "SOIL2/SOIL2.h"

// project includes
#include "Application.h"
#include "Shader.h"
#include "Particle.h"

using namespace GPhysix;

// _Task_Eval, _Task_Blow
#define _Task_Blow

#pragma region HELPER FUNCTIONS PROTOTYPES
void CreateApp(Application &app);
void IntegrateSI(Particle &particle, const float deltaTime);
void IntegrateEX(Particle &particle, const float deltaTime);
bool inBoundaries(const Mesh &particle, const Mesh &boundingBox, const int &coordIndex);
void CollisionDetection(const Mesh &boundingBox, std::array<Particle, 3> &particles);
void Evaluation(std::array<Particle, 3> &particles, float &accumulator, const float deltaTime);
void BlowDryer(std::array<Particle, 3> &particles, const Mesh &forceField, float &accumulator, const float deltaTime, DynamicForce *wind);
#pragma endregion

// main function
int main()
{

#pragma region SETUP
	// create application
	Application app = Application::Application();
	CreateApp(app);
	
	// create ground plane
	Mesh plane = Mesh::Mesh();
	plane.scale(glm::vec3(20.0f, 20.0f, 20.0f));
	plane.setShader(Shader("resources/shaders/core.vert", "resources/shaders/core.frag"));

	// create non-visual , no need for shaders
	// collision boundary
	Mesh boundingBox = Mesh::Mesh();
	boundingBox.scale(glm::vec3(20.0f, 20.0f, 20.0f));
	boundingBox.translate(glm::vec3(-11.0f, .0f, -11.0f));
	// trigger force field
	Mesh forceField = Mesh::Mesh();
	forceField.scale(glm::vec3(7.5f, 7.5f, 7.5f));
	forceField.translate(glm::vec3(-5.0f, .0f, -5.0f));

#ifdef _Task_Blow
		/* DRAG
		// drag force
		DynamicForce *drag = new DynamicForce();
		*/
		// wind force
		DynamicForce *wind = new DynamicForce();
		wind->setForce(glm::vec3(.0005f, .005f, .0005f));
#endif
	
	// time
	const float deltaTime = .0005f;
	float accumulator = .0f;
	GLfloat currentTime = (GLfloat)glfwGetTime();
#pragma endregion

#pragma region CREATE PARTICLES
	// create particles
	std::array<Particle, 3> particles;
	float span = 0.0f;
	for (auto &particle : particles)
	{
		particle.scale(glm::vec3(3.f, 3.f, 3.f));
#ifdef _Task_Eval
		particle.translate(glm::vec3(span, plane.getPos().y + 2.0f, 0.0f));
		particle.setVel(glm::vec3(.0f, .0f, .0f));
#elif defined (_Task_Blow)
		particle.translate(glm::vec3(span, plane.getPos().y + 2.5f, 0.0f));
		particle.setVel(glm::vec3(.25f, .5f, .125f));
#endif
		particle.getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag"));
		span += 2.0f;
	}
#pragma endregion

	// Game loop
	while (!glfwWindowShouldClose(app.getWindow()))
	{

#pragma region TIMING
		// SET frame time
		GLfloat newTime = (GLfloat)glfwGetTime();
		GLfloat frameTime = newTime - currentTime;
		currentTime = newTime;
		accumulator += frameTime;
		// END fFrame time
#pragma endregion

#pragma region INTERACTION
		// Manage interaction
		app.doMovement(deltaTime);
#pragma endregion

#pragma region ASSIGNMENT - Deliverables
#ifdef _Task_Eval
		Evaluation(particles, accumulator, deltaTime);
#elif defined (_Task_Blow)
		BlowDryer(particles, forceField, accumulator, deltaTime, wind);
#endif
#pragma endregion
		
		// Collision detection used for all tasks
		CollisionDetection(boundingBox, particles);

#pragma region RENDER
		// clear buffer
		app.clear();
		// draw groud plane
		app.draw(plane);
		// draw particles
#ifdef _Task_Eval
		for (auto &particle : particles) app.draw(particle.getMesh());
#elif defined (_Task_Blow)
		app.draw(particles[0].getMesh());
#endif
		app.display();
#pragma endregion
	
	}

	// close the application
	app.terminate();
	
	return EXIT_SUCCESS;
}


#pragma region HELPER FUNCTION DEFINITIONS/IMPLEMENTATIONS

void CreateApp(Application &app)
{
	app.initRender();
	Application::camera.setCameraPosition(glm::vec3(0.0f, 15.0f, 45.0f));
}

void IntegrateSI(Particle &particle, const float deltaTime)
{
	particle.getVel() += particle.getAcc() * deltaTime;
	particle.getPos() += particle.getVel() * deltaTime;
}

void IntegrateEX(Particle &particle, const float deltaTime)
{
	particle.getPos() += particle.getVel() * deltaTime;
	particle.getVel() += particle.getAcc() * deltaTime;
}

bool inBoundaries(const Mesh &particle, const Mesh &boundingBox, const int &coordIndex)
{
	bool minCheck = particle.getPos()[coordIndex] >= boundingBox.getPos()[coordIndex] + .5f;
	bool maxCheck = particle.getPos()[coordIndex] <= boundingBox.getPos()[coordIndex] + boundingBox.getScale()[coordIndex][coordIndex];

	return minCheck && maxCheck;
}

void CollisionDetection(const Mesh &boundingBox, std::array<Particle, 3> &particles)
{
	std::for_each(particles.begin(), particles.end() - 1, [&](Particle &particle)
	{
		// Loop 0..3 for all sides of the particle object
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
	});
}

void Evaluation(std::array<Particle, 3> &particles, float &accumulator, const float deltaTime)
{
	auto applyInitialForce = [](Particle &particle)
	{
		particle.dForce.setForce(particle.getMass() * particle.dForce.getGravity());
		particle.setAcc(particle.dForce.getForce() / particle.getMass());
	};

	// Calculate initial force & set acceleration
	std::for_each(particles.begin(), particles.end() - 1, applyInitialForce);

	// Integration
	while (accumulator >= deltaTime)
	{
		IntegrateEX(particles[0], deltaTime);
		IntegrateSI(particles[1], deltaTime);
		particles[0].move(particles[0].getVel(), particles[0].getPos());
		particles[1].move(particles[1].getVel(), particles[1].getPos());
		accumulator -= deltaTime;
	}
}

void BlowDryer(std::array<Particle, 3> &particles, const Mesh &forceField, float &accumulator, const float deltaTime, DynamicForce *wind)
{
	auto applyInitialForce = [](Particle &particle)
	{
		particle.dForce.setForce(particle.getMass() * particle.dForce.getGravity());
		particle.setAcc(particle.dForce.getForce() / particle.getMass());

		// Work out Drag - NOT USED
		/* DRAG
		GLfloat dotVel = glm::dot(particles[i].getVel(), particles[i].getVel());
		glm::vec3 dragVec = (1.25f * .5f * .01f * dotVel * particles[i].getVel() / sqrt(dotVel));
		drag->setForce(-dragVec * particles[i].getMass());
		particles[i].addForce(drag);
		*/
	};
	
	// Calculate initial force & set acceleration
	std::for_each(particles.begin(), particles.end() - 2, applyInitialForce);

	// Integration
	while (accumulator >= deltaTime)
	{
		std::for_each(particles.begin(), particles.end() - 2, [=](Particle &particle)
		{
			IntegrateSI(particle, deltaTime);
			particle.move(particle.getVel(), particle.getPos());
		});
		accumulator -= deltaTime;
	}
	
	// Force field trigger
	std::for_each(particles.begin(), particles.end() - 2, [&](Particle &particle)
	{
		if (	// Check Y-axis
			(particle.getPos().y >= forceField.getPos().y && particle.getPos().y <= forceField.getScale()[1].y)
			&&	// Check X-axis
			(particle.getPos().x >= forceField.getPos().x - .5f * (forceField.getScale()[1].y - forceField.getPos().y) &&
				particle.getPos().x <= forceField.getScale()[0].x + .5f * (particle.getPos().y - forceField.getPos().y))
			&&	// Check Z-axis
			(particle.getPos().z >= forceField.getPos().z - .5f * (forceField.getScale()[1].y - forceField.getPos().y) &&
				particle.getPos().z <= forceField.getScale()[0].z + .5f * (particle.getPos().y - forceField.getPos().y))
			)
		{
			particle.getVel().y += wind->getForce().y - (wind->getForce().y * (particle.getPos().y - forceField.getPos().y) * .1f) * 0.0001f;
			particle.getVel().x += -(wind->getForce().x - ((forceField.getScale()[0].x + forceField.getPos().x) * 0.1f) - particle.getPos().x) * 0.0001f;
			particle.getVel().z += -(wind->getForce().z - ((forceField.getScale()[0].z + forceField.getPos().z) * 0.1f) - particle.getPos().z) * 0.0001f;
		}
	});
#pragma endregion
}

#pragma endregion