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
void handleStaticCollision(RigidBody* rb, Vertex& collisionPoint, float e, bool applyFriction = false);
std::vector<Vertex> detectCollisionWithPlane(RigidBody* rb, const glm::vec3& plane);
Vertex calcCollisionPoint(std::vector<Vertex> collidingVertices, RigidBody* rb);
void applyImpulse(RigidBody* rb, glm::vec3& J, glm::vec3& appPoint, double duration = 0);
// TODO: add applyFriction function

void Update(const std::vector<RigidBody*>& rigidBodies, const glm::vec3& plane, const glm::vec3& boundary)
{
	// time-step and movement
	while (sim->getAccumultor() >= Time::deltaTime)
	{
		for (auto rb : rigidBodies)
		{
			// calculate acceleration based on forces
			#ifndef TASK1
			rb->setAcc(rb->applyForces(rb->getPos(), rb->getVel()));
			#endif

			// Integrate position, velocity
			rb->integrate(Time::deltaTime);
			
			// move the rigid body by its velocity
			rb->move();

			// Inegrate rotation
			rb->setAngVel(rb->getAngVel() + Time::deltaTime * rb->getAngAcc());
			auto angVelSkew = glm::matrixCross3(rb->getAngVel());
			auto R = glm::mat3(rb->getRotate());
			R += Time::deltaTime * angVelSkew * R;
			R = glm::orthonormalize(R);
			rb->setRotate(glm::mat4(R));

			#ifdef TASK1
			auto j = glm::vec3(-4.0f, 0.0f, 0.0f);
			auto applicationPoint = glm::vec3(rb->getPos() - glm::vec3(1.0f, 1.0f, 0.0f));
			applyImpulse(rb, j, applicationPoint, 2.0f);
			#else
			// ------------------ //
			// Collison detection //
			// ------------------ //
			auto collidingVertices = detectCollisionWithPlane(rb, plane);
			bool hasCollision = collidingVertices.size() > 0;

			// if collision is detected canRespond will be set true and can be used to check for collision
			if (hasCollision)
			{
				// check if the rigid body is in the boundaries
				if (rb->getPos().x < (0.5f * boundary.x + 2.5f) && rb->getPos().x > -(0.5f * boundary.x + 2.5f)
					&& rb->getPos().z < (0.5f * boundary.z + 2.5f) && rb->getPos().z > -(0.5f * boundary.z + 2.5f))
				{
					// Hacky fix of position
					auto lowestVertex = collidingVertices[0];
					for (auto vert : collidingVertices)
					{
						if (vert.getCoord().y < lowestVertex.getCoord().y)
						{
							lowestVertex = vert;
						}
					}

					glm::vec3 displacement = glm::vec3(0.0f);
					displacement.y = glm::abs(lowestVertex.getCoord().y);
					rb->translate(displacement);

					Vertex collisionPoint = calcCollisionPoint(collidingVertices, rb);
					// handle the collision on response
					handleStaticCollision(rb, collisionPoint, 0.7f, true);

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
			}
			#endif

		}

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
	plane.scale(glm::vec3(50.0f, 1.0f, 50.0f));
	plane.setShader(Shader("resources/shaders/core.vert", "resources/shaders/core.frag"));
	
	// create rigid body cube from mesh
	auto cube = RigidBody(Mesh(Mesh::CUBE));
	cube.getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag"));
	cube.setMass(2.0f);
	cube.scale(glm::vec3(1.0f, 3.0f, 1.0f));
	cube.translate(glm::vec3(0.0f, 10.0f, 0.0f));
	cube.setVel(glm::vec3(0.0f, 0.0f, 0.0f));
	cube.setAngVel(glm::vec3(0.0f, 0.0f, 0.5f));

	// Create forces
	Gravity g;
	// add forces to the cube rigid body
	cube.addForce(&g);

	// create a global var to store the point of collision between the cube and the plane
	glm::vec3 collisionPoint = glm::vec3(0.0f);

	// game loop
	while (sim->IsRunning())
	{
		sim->BeginFrames();
		
		Update({ &cube }, plane.getPos(), plane.getScaleVec());
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
void handleStaticCollision(RigidBody* rb, Vertex& collisionPoint, float e, bool applyFriction)
{
	auto n = glm::normalize(glm::vec3(0.0f, 1.0f, 0.0f));
	auto r = collisionPoint.getCoord() - rb->getPos();
	auto rVel = rb->getVel() + glm::cross(rb->getAngVel(), r);
	auto jCollision = glm::vec3(0.0f);
	auto num = -(1.0f + e) * rVel * n;
	auto denom = glm::pow(rb->getMass(), -1) + (n * glm::cross(rb->getInvInertia() * glm::cross(r, n), r));
	jCollision = num / denom;

	// apply collision impulse
	applyImpulse(rb, jCollision, r);
	
	// applyFriction = false; // Debug
	if (applyFriction)
	{
		/*
		// coeff. of friction
		float u = 0.25f;
		// glm::vec3 t = glm::orthonormalize(relVel, n);
		// jt from gregory's lecture:
		//float jt = jCollision.x * n.x + jCollision.y * n.y + jCollision
		glm::vec3 tangent = r - n * glm::dot(r, n);
		tangent = glm::normalize(tangent);

		glm::vec3 jFriction;
		if (abs(jt) < glm::length(jCollision) * u)
			jFriction = jt * rV;
		else
			jFriction = -jt * rV * u;

		std::cout << glm::to_string(relVel) << std::endl;

		// apply friction impulse
		applyImpulse(rb, -jFriction, r);
		*/
	}
}

std::vector<Vertex> detectCollisionWithPlane(RigidBody* rb, const glm::vec3& plane)
{
	std::vector<Vertex> collidingVertices;

	for (auto vert : rb->getMesh().getVertices())
	{
		auto vert2World = glm::mat3(rb->getMesh().getModel()) * vert.getCoord() + rb->getPos();

		// Check if a point colides
		if (vert2World.y < plane.y)
		{
			collidingVertices.push_back(vert2World);
		}
	}

	return collidingVertices;
}

Vertex calcCollisionPoint(std::vector<Vertex> collidingVertices, RigidBody* rb)
{
	auto verticesSum = glm::vec3(0.0f);
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