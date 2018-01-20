#include "Simulation.h"
#include <memory>

#ifdef RIGID_BODY

// Only define the following tasks: TASK1, TASK2
#define TASK2
// TASK3 is gonna run anyway if TASK1 is not defined
// TASK4 is dependent on a boolean applyFriction in TASK3

using namespace GPhysix;

std::unique_ptr<Simulation<RigidBody>> sim;
// GLOBAL DEBUG FLAGS/VARIABLES
bool isImpulseApplied = false;
bool isPrinted = false;
bool isCollisionStatPrinted = false;

[[deprecated("Do not use, yet. I will need to re-evaluate it!")]]
glm::vec3 toWorld(glm::vec3 vect, const glm::mat4& model)
{
	auto vect4D = glm::vec4(vect, 1.0f);
	vect4D = model * vect4D;

	vect.x = vect4D.x / vect4D.w;
	vect.y = vect4D.y / vect4D.w;
	vect.z = vect4D.z / vect4D.w;

	return vect;
}
// ----------------------------- //
// HELPER FUNCTIONS DECLARATIONS //
// ----------------------------- //
void handleStaticCollision(RigidBody* rb, Vertex collisionPoint, float e, bool applyFriction = false);
std::vector<Vertex> detectCollisionWithPlane(RigidBody* rb, glm::vec3 plane);
Vertex calcCollisionPoint(std::vector<Vertex> collidingVertices, RigidBody* rb);
void applyImpulse(RigidBody* rb, glm::vec3& J, glm::vec3& appPoint, double duration = 0);
// TODO: add applyFriction function

void Update(const std::vector<RigidBody*>& rigidBodies, glm::vec3 plane)
{
	// time-step and movement
	while (sim->getAccumultor() >= Time::deltaTime)
	{
		// manage interaction
		sim->getApp().doMovement(Time::deltaTime);

		auto& rb = rigidBodies[0];

		// calculate acceleration based on forces
		#ifndef TASK1
		rb->setAcc(rb->applyForces(rb->getPos(), rb->getVel()));
		#endif

		// Integrate position, velocity
		rb->integrate(Time::deltaTime);

		// Inegrate rotation
		rb->setAngVel(rb->getAngVel() + Time::deltaTime * rb->getAngAcc());
		glm::mat3 angVelSkew = glm::matrixCross3(rb->getAngVel());
		glm::mat3 R = glm::mat3(rb->getRotate());
		R += Time::deltaTime * angVelSkew * R;
		R = glm::orthonormalize(R);
		rb->setRotate(glm::mat4(R));

		#ifdef TASK1
		auto J = glm::vec3(-5.0f, 0.0f, 0.0f);
		auto applicationPoint = glm::vec3(rb->getPos() - glm::vec3(1.0f, 1.0f, 0.0f));
		applyImpulse(rb, J, applicationPoint, 2.0f);
		#else
		// ------------------ //
		// Collison detection //
		// ------------------ //
		auto collidingVertices = detectCollisionWithPlane(rb, plane);
		bool hasCollision = (collidingVertices.size() > 0);

		if (hasCollision)
		{
			// translate up on y axis by the lowest vertex on the y axis
			Vertex lowestVertex = collidingVertices[0].getCoord();
			for (Vertex vert : collidingVertices)
			{
				if (vert.getCoord().y < lowestVertex.getCoord().y)
					lowestVertex = vert;
			}

			// hacky 'empirical' fix for position
			glm::vec3 displacement = glm::vec3(0.0f);
			displacement.y = glm::abs(lowestVertex.getCoord().y);
			rb->translate(/*Time::deltaTime * */displacement);

			// get average point of collision from the colliding vertices
			Vertex collisionPoint = calcCollisionPoint(collidingVertices, rb);

			if (glm::length(rb->getVel()) + glm::length(rb->getAngVel()) < 0.1f)
			{
				rb->getVel() = glm::vec3(0.0f);
				rb->getAngVel() = glm::vec3(0.0f);
				rb->getAcc() = glm::vec3(0.0f);
				displacement.y = 0.0f;
			}

			// handle the collision on response
			handleStaticCollision(rb, collisionPoint, 0.8f, true);

			#ifdef TASK2
			if (!isCollisionStatPrinted)
			{
				for (Vertex vert : collidingVertices)
				{
					std::cout << "Collision detected in: " << glm::to_string(vert.getCoord()) << std::endl;
				}
				std::cout << "\nApplied collision impulse in: " << glm::to_string(collisionPoint.getCoord()) << std::endl << std::endl;

				isCollisionStatPrinted = true;
			}
			#endif
		}
		#endif

		// reset frames accumulator
		sim->getAccumultor() -= Time::deltaTime;
	}
}

void Render(const std::vector<Mesh>& meshes)
{
	for (const auto& mesh : meshes) sim->Draw(mesh);
}

auto main(void) -> int
{
	sim = std::make_unique<Simulation<RigidBody>>();

	// create ground plane
	Mesh plane;
	plane.scale(glm::vec3(100.0f, 1.0f, 100.0f));
	plane.setShader(Shader("resources/shaders/core.vert", "resources/shaders/core.frag"));
	
	// create rigid body cube from mesh
	auto cube = RigidBody(Mesh(Mesh::CUBE));
	cube.getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag"));
	cube.setMass(2.0f);
	cube.scale(glm::vec3(1.0f, 3.0f, 1.0f));
	cube.translate(glm::vec3(0.0f, 10.0f, 0.0f));
	#ifdef TASK1
	cube.setVel(glm::vec3(2.0f, 0.0f, 0.0f));
	cube.setAngVel(glm::vec3(0.0f, 0.0f, 0.0f));
	#else
	cube.setVel(glm::vec3(2.0f, 0.0f, 0.0f));
	cube.setAngVel(glm::vec3(0.1f, 0.1f, 0.1f));
	#endif
	// Output the inverse intertia
	std::cout << "Inverse inertia: " << glm::to_string(cube.getInvInertia()) << "\n\n";

	// Create forces
	Gravity g;
	// add forces to the cube rigid body
	cube.addForce(&g);

	// game loop
	while (sim->IsRunning())
	{
		sim->BeginFrames();
		
		Update({ &cube }, plane.getPos());
		Render({ plane, cube.getMesh() });

		sim->EndFrames();
	}

	// close the program
	sim->Exit();
	
	return EXIT_SUCCESS;
}

// ---------------------------- //
// HELPER FUNCTIONS DEFINITIONS //
// ---------------------------- //
void handleStaticCollision(RigidBody* rb, Vertex collisionPoint, float e, bool applyFriction)
{
	glm::vec3 r = collisionPoint.getCoord() - rb->getPos();
	glm::vec3 vr = rb->getVel() + glm::cross(rb->getAngVel(), r);
	glm::vec3 n = glm::normalize(glm::vec3(0.0f, 1.0f, 0.0f));
	glm::vec3 jCollision = (-(1 + e) * vr * n) / (pow(rb->getMass(), -1) + n * glm::cross((rb->getInvInertia() * (glm::cross(r, n))), r));

	// apply collision
	applyImpulse(rb, jCollision, r + rb->getPos());

	if (applyFriction)
	{
		glm::vec3 vt = vr - glm::dot(vr, n) * n;
		constexpr float mu = 0.1f;
		glm::vec3 jFriction = -mu * glm::length(jCollision) * glm::normalize(vt);
		
		if (glm::length(jCollision) > 0.0f)
		{
			// apply friction
			applyImpulse(rb, jFriction, r + rb->getPos());
		}
	}
}

std::vector<Vertex> detectCollisionWithPlane(RigidBody* rb, glm::vec3 plane)
{
	std::vector<Vertex> collidingVertices;

	// Check if it collides with plane
	for (Vertex vert : rb->getMesh().getVertices())
	{
		glm::vec3 edge = glm::mat3(rb->getMesh().getModel()) * vert.getCoord() + rb->getPos();
		if (edge.y < plane.y)
		{
			collidingVertices.push_back(edge);
		}
	}

	return collidingVertices;
}

Vertex calcCollisionPoint(std::vector<Vertex> collidingVertices, RigidBody* rb)
{
	glm::vec3 verticesSum = glm::vec3(0.0f);
	for (auto ver : collidingVertices)
	{
		verticesSum += ver.getCoord();
	}
	return Vertex(verticesSum / collidingVertices.size());
}

void applyImpulse(RigidBody* rb, glm::vec3& J, glm::vec3& appPoint, double duration)
{
	if (duration > 0)
	{
		// apply impulse over time
		if (glfwGetTime() > duration && !isImpulseApplied)
		{
			rb->setVel(rb->getVel() + J / rb->getMass());
			rb->setAngVel(rb->getAngVel() + rb->getInvInertia() * glm::cross(appPoint - rb->getPos(), J));

			isImpulseApplied = true;
		}
	}
	else
	{
		// simply apply impulse
		rb->setVel(rb->getVel() + J / rb->getMass());
		rb->setAngVel(rb->getAngVel() + rb->getInvInertia() * glm::cross(appPoint - rb->getPos(), J));
	}
}

#endif