// Std. Includes
#include <string>
#include <time.h>

#include "Simulation.h"
#include <memory>

#ifdef DOMINO

using namespace GPhysix;

// simulation obj
std::unique_ptr<Simulation<RigidBody>> sim;
// forces
Gravity g = Gravity(glm::vec3(0.0f, -0.1f, 0.0f));
// flags
bool isImpulseApplied = false;
bool canRespondStatic = false;
bool canRespondDynamic = false;

// integration: returns the position difference from the acceleration and a timestep
void integratePos(RigidBody* rb);
// integration: returns the rotation difference from the acceleration and a timestep
void integrateRot(RigidBody* rb);
// Collision helper functions
void handleStaticCollision(RigidBody* rb, Vertex collisionPoint, float e, bool applyFriction = false);
std::vector<Vertex> detectCollisionWithPlane(RigidBody* rb, glm::vec3 plane);
Vertex calcCollisionPoint(std::vector<Vertex> collidingVertices, RigidBody* rb);
void applyImpulse(RigidBody* rb, glm::vec3& J, glm::vec3& appPoint, double duration = 0);

void handleCollisions(const std::vector<RigidBody*>& rigidBodies)
{
	// Do static rigid body vs (e.g.)plane collisions
	for (size_t i = 0; i < rigidBodies.size(); i++)
	{
		auto& rb = rigidBodies[i];

		auto collisionData = rb->canCollideStatic();
		canRespondStatic = collisionData.getHasIntersection();
		if (canRespondStatic)
		{
			std::cout << "A static collision was detected.\n";
			rb->setVel(glm::vec3(0.0f));
			rb->setAngVel(glm::vec3(0.0f));
			rb->getPos().y = 1.0f;
		}
	}

	// Do dynamic rigid body vs rigid body collisions
	for (size_t i = 0; i < rigidBodies.size(); i++)
	{
		auto& rb1 = rigidBodies[i];
		for (size_t j = i + 1; j < rigidBodies.size(); j++)
		{
			auto& rb2 = rigidBodies[j];

			// perform intersection check
			auto collisionData = rb1->canCollideDynamic(rb2);
			canRespondDynamic = collisionData.getHasIntersection();
			if (canRespondDynamic)
			{
				std::cout << "A dynamic collision was detected.\n";
				rb1->setVel(-1.0f * rb1->getVel());
				rb2->setVel(-1.0f * rb2->getVel());
				rb1->setAngVel(-1.0f * rb1->getAngVel());
				rb2->setAngVel(-1.0f * rb2->getAngVel());
			}
		}
	}
}

void Update(const std::vector<RigidBody*>& rigidBodies, glm::vec3 plane)
{
	// time-step and movement
	while (sim->getAccumultor() >= Time::deltaTime)
	{
		// manage interaction
		sim->getApp().doMovement(Time::deltaTime);

		if (!sim->getApp().pauseSimulation)
		{
			for (auto rb : rigidBodies)
			{
				//Integration (position)
				integratePos(rb);
				// integration (rotation)
				integrateRot(rb);

				// Collison handling (detection & response)
				handleCollisions(rigidBodies);
			}
		}
		// reset frames accumulator & update time
		sim->getAccumultor() -= Time::deltaTime;
	}
}

void Render(const std::vector<Mesh>& meshes)
{
	for (const auto& mesh : meshes) sim->Draw(mesh);
}

// main function
int main()
{
	sim = std::make_unique<Simulation<RigidBody>>();

	// define mesh types
	Mesh quadMesh = Mesh::Mesh(Mesh::QUAD);
	Mesh cubeMesh = Mesh::Mesh(Mesh::CUBE);
	Mesh sphereMesh = Mesh::Mesh("resources/models/sphere1.obj");

	// create environment
	Mesh plane = cubeMesh;
	plane.scale(glm::vec3(50.f, 1.0f, 50.f));
	plane.setShader(Shader("resources/shaders/physics.vert", "resources/shaders/transp.frag"));
	
	// rigid body set up
	// BODY 1
	RigidBody cube1 = RigidBody();
	cube1.setMesh(sphereMesh);
	cube1.getMesh().setShader(Shader("resources/shaders/physics.vert", "resources/shaders/physics.frag"));
	cube1.setMass(1.0f);
	cube1.translate(glm::vec3(-10.0f, 5.0f, 0.0f));
	cube1.setVel(glm::vec3(0.5f, 0.0f, 0.0f));
	cube1.setAngVel(glm::vec3(0.0f, 0.0f, -0.5f));
	cube1.addCollider(TYPE::SPHERE);
	// add forces to Rigid body
	cube1.addForce(&g);
	
	// BODY 2
	RigidBody cube2 = RigidBody();
	cube2.setMesh(cubeMesh);
	cube2.getMesh().setShader(Shader("resources/shaders/physics.vert", "resources/shaders/physics.frag"));
	cube2.setMass(1.0f);
	cube2.translate(glm::vec3(10.0f, 5.0f, 0.0f));
	cube2.setVel(glm::vec3(-0.5f, 0.0f, 0.0f));
	//cube2.setAngVel(glm::vec3(0.0f, 0.0f, 0.0f));
	cube2.setAngVel(glm::vec3(0.2f, 0.0f, 0.3f));
	cube2.addCollider(TYPE::OBB);
	// add forces to Rigid body
	cube2.addForce(&g);

	// BODY 3
	RigidBody cube3 = RigidBody();
	cube3.setMesh(cubeMesh);
	cube3.getMesh().setShader(Shader("resources/shaders/physics.vert", "resources/shaders/physics.frag"));
	cube3.setMass(1.0f);
	cube3.translate(glm::vec3(0.0f, 3.0f, 0.0f));
	cube3.setVel(glm::vec3(0.0f, 0.0f, 0.0f));
	cube3.setAngVel(glm::vec3(0.1f, 0.2f, 0.3f));
	cube3.addCollider(TYPE::OBB);
	// add forces to Rigid body
	cube3.addForce(&g);
	
	// game loop
	while (sim->IsRunning())
	{
		sim->BeginFrames();

		Update({ &cube1, &cube2, &cube3 }, plane.getPos());
		Render({ plane, cube1.getMesh(), cube2.getMesh(), cube3.getMesh() });

		sim->EndFrames();
	}

	// close the program
	sim->Exit();

	return EXIT_SUCCESS;
}

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
		constexpr float mu = 0.006f;
		glm::vec3 jFriction = -mu * glm::length(jCollision) * glm::normalize(vt);

		if (glm::length(jCollision) > 0.0f)
		{
			// apply friction
			applyImpulse(rb, jFriction, r + rb->getPos());
		}
	}
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

#pragma region INTEGRATION METHODS
void integratePos(RigidBody* rb)
{
	rb->setAcc(rb->applyForces(rb->getPos(), rb->getVel()));
	rb->setVel(rb->getVel() + Time::deltaTime * rb->getAcc());
	glm::vec3 dPos = Time::deltaTime * rb->getVel();
	rb->translate(dPos);
}

// integration: returns the rotation difference from the acceleration and a timestep
void integrateRot(RigidBody* rb)
{
	rb->setAngVel(rb->getAngVel() + Time::deltaTime * rb->getAngAcc());
	glm::mat3 angVelSkew = glm::matrixCross3(rb->getAngVel());
	// create 3x3 rotation matrix from rb rotation matrix
	glm::mat3 R = glm::mat3(rb->getRotate());
	// update rotation matrix
	R += Time::deltaTime * angVelSkew * R;
	R = glm::orthonormalize(R);
	rb->setRotate(glm::mat4(R));
}
#pragma endregion

#endif