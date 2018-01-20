// project includes
#include "Simulation.h"

// Std. Includes
#include <string>
#include <time.h>
#include <memory>
#include <set>

#ifdef DOMINO

using namespace GPhysix;

// simulation obj
std::unique_ptr<Simulation<RigidBody>> sim;
bool canRespondStatic = false;
bool canRespondDynamic = false;

// integration: returns the position difference from the acceleration and a timestep
void integratePos(RigidBody* rb);
// integration: returns the rotation difference from the acceleration and a timestep
void integrateRot(RigidBody* rb);
// static collision with plain helper functions
std::set<Vertex> collidingVerts(float yCoordinate, RigidBody &rb);
void collisionResponce(RigidBody &rb, Mesh plane, std::set<Vertex>collidingVertices);
void respond(RigidBody& rb1, RigidBody& rb2, CollisionManifold data);
void handleCollisions(const std::vector<RigidBody*>& rigidBodies, Mesh plane);

void Update(const std::vector<RigidBody*>& rigidBodies, Mesh plane)
{
	// time-step and movement
	while (sim->getAccumultor() >= Time::deltaTime)
	{
		// manage interaction
		sim->getApp().doMovement(Time::deltaTime);

		if (!sim->getApp().pauseSimulation)
		{
			// Collison handling (detection & response)
			handleCollisions(rigidBodies, plane);

			for (auto rb : rigidBodies)
			{
				//Integration (position)
				integratePos(rb);
				// integration (rotation)
				integrateRot(rb);
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

	// create ground plane
	Mesh plane = Mesh::Mesh(Mesh::QUAD);
	//Mesh plane = Mesh::Mesh("resources/models/plane10.obj");
	plane.setShader(Shader("resources/shaders/physics.vert", "resources/shaders/transp.frag"));
	plane.scale(glm::vec3(20.0f, 1.0f, 20.0f));
	plane.translate(glm::vec3(0.0f, 0.0f, 0.0f));
	
	// forces
	Gravity g = Gravity();

	// BODY 1
	RigidBody rb1 = RigidBody();
	rb1.setMesh(cubeMesh);
	rb1.getMesh().setShader(Shader("resources/shaders/physics.vert", "resources/shaders/physics.frag"));
	rb1.setMass(2.0f);
	rb1.translate(glm::vec3(0.0f, 2.0f, 0.0f));
	rb1.setVel(glm::vec3(0.0f, 0.0f, 0.0f));
	rb1.scale(glm::vec3(0.25f, 2.0f, 1.0f));
	//rb1.setAngVel(glm::vec3(0.0f, 0.0f, 0.0f));
	rb1.setAngVel(glm::vec3(0.0f, 0.0f, 0.0f));
	rb1.addCollider(TYPE::OBB);
	// add forces to Rigid body
	rb1.addForce(&g);

	// BODY 2
	RigidBody rb2 = RigidBody();
	rb2.setMesh(cubeMesh);
	rb2.getMesh().setShader(Shader("resources/shaders/physics.vert", "resources/shaders/physics.frag"));
	rb2.setMass(2.0f);
	rb2.translate(glm::vec3(2.0f, 2.0f, 0.0f));
	rb2.setVel(glm::vec3(0.0f, 0.0f, 0.0f));
	rb2.scale(glm::vec3(0.25f, 2.0f, 1.0f));
	//rb2.setAngVel(glm::vec3(0.0f, 0.0f, 0.0f));
	rb2.setAngVel(glm::vec3(0.0f, 0.0f, 0.0f));
	rb2.addCollider(TYPE::OBB);
	// add forces to Rigid body
	rb2.addForce(&g);

	// BODY 3
	RigidBody rb3 = RigidBody();
	rb3.setMesh(cubeMesh);
	rb3.getMesh().setShader(Shader("resources/shaders/physics.vert", "resources/shaders/physics.frag"));
	rb3.setMass(1.0f);
	rb3.translate(glm::vec3(-2.0f, 2.0f, 0.0f));
	rb3.setVel(glm::vec3(0.0f, 0.0f, 0.0f));
	rb3.scale(glm::vec3(0.25f, 2.0f, 1.0f));
	//rb2.setAngVel(glm::vec3(0.0f, 0.0f, 0.0f));
	rb3.setAngVel(glm::vec3(0.0f, 0.0f, 0.0f));
	rb3.addCollider(TYPE::OBB);
	// add forces to Rigid body
	rb3.addForce(&g);

	// BODY 4 -SHPERE
	RigidBody rb4 = RigidBody();
	rb4.setMesh(sphereMesh);
	rb4.getMesh().setShader(Shader("resources/shaders/physics.vert", "resources/shaders/physics.frag"));
	rb4.setMass(1.0f);
	rb4.translate(glm::vec3(4.0f, 5.0f, 0.0f));
	rb4.setVel(glm::vec3(-6.0f, 0.0f, 0.0f));
	//rb4.setAngVel(glm::vec3(0.0f, 0.0f, 0.0f));
	rb4.setAngVel(glm::vec3(0.0f, 0.0f, 0.0f));
	rb4.addCollider(TYPE::SPHERE);
	// add forces to Rigid body
	rb4.addForce(&g);

	// game loop
	while (sim->IsRunning())
	{
		sim->BeginFrames();

		Update({ &rb1, &rb2, &rb3, &rb4 }, plane);
		Render({ plane, rb1.getMesh(), rb2.getMesh(), rb3.getMesh(), rb4.getMesh() });

		sim->EndFrames();
	}

	// close the program
	sim->Exit();

	return EXIT_SUCCESS;
}

void handleCollisions(const std::vector<RigidBody*>& rigidBodies, Mesh plane)
{
	for (size_t i = 0; i < rigidBodies.size(); i++)
	{
		auto& rb1 = rigidBodies[i];

		auto collisionData = rb1->canCollideStatic();
		canRespondStatic = collisionData.getHasIntersection();
		if (canRespondStatic)
		{
			// Do static rigid body vs plane collisions
			std::set<Vertex> collidingVertices = collidingVerts(plane.getPos().y, *rb1);
			bool collisionOccurs = collidingVertices.size() > 0;
			if (collisionOccurs)
				collisionResponce(*rb1, plane, collidingVertices);
		}

		// Do dynamic rigid body vs rigid body collisions
		for (size_t j = i + 1; j < rigidBodies.size(); j++)
		{
			auto& rb2 = rigidBodies[j];

			// perform intersection check
			auto intersectData = rb1->canCollide(rb2);
			if (intersectData.getHasIntersection())
				respond(*rb1, *rb2, intersectData);
		}
	}
}


void respond(RigidBody& rb1, RigidBody& rb2, CollisionManifold data)
{
	float invMass1 = 1.0f / rb1.getMass();
	float invMass2 = 1.0f / rb2.getMass();
	float invMassSum = invMass1 + invMass2;
	if (invMassSum == 0.0f)
		return; ///> both objects have infinate mass!

	if (glm::length(rb1.getVel()) + glm::length(rb1.getAngVel()) < 0.2f)
	{
		rb1.getVel() = glm::vec3(0.0f);
		rb1.setAngVel(glm::vec3(0.0f));
		rb1.getAcc() = glm::vec3(0.0f);

	}

	if (glm::length(rb2.getVel()) + glm::length(rb2.getAngVel()) < 0.2f)
	{
		rb2.getVel() = glm::vec3(0.0f);
		rb2.setAngVel(glm::vec3(0.0f));
		rb2.getAcc() = glm::vec3(0.0f);
	}

	//local points of contact
	glm::vec3 sumPoints;
	int count = 0;
	for (int i = 0; i < data.getContacts().size(); i++)
	{
		sumPoints += data.getContacts()[i];
		count++;
	}

	sumPoints = sumPoints / count;
	glm::vec3 r1 = sumPoints - rb1.getPos();
	glm::vec3 r2 = sumPoints - rb2.getPos();

	glm::mat4 i1 = rb1.getInvInertia();
	glm::mat4 i2 = rb2.getInvInertia();

	// Relative velocity
	glm::vec3 relativeVel = (rb2.getVel() + glm::cross(rb2.getAngVel(), r2)) - (rb1.getVel() + glm::cross(rb1.getAngVel(), r1));
	// Relative collision normal
	glm::vec3 relativeNorm = data.getNormal();
	relativeNorm = glm::normalize(relativeNorm);

	if (glm::dot(relativeVel, relativeNorm) > 0.0f)
		return;

	float e = 0.01f;
	float numerator = (-(1.0f + e)* glm::dot(relativeVel, relativeNorm));
	float d1 = invMassSum;
	glm::vec3 d2 = glm::cross((glm::vec3(glm::vec4(glm::cross(r1, relativeNorm), 1.0f) * i1)), r1);
	glm::vec3 d3 = glm::cross((glm::vec3(glm::vec4(glm::cross(r2, relativeNorm), 1.0f) * i2)), r2);
	float denominator = d1 + glm::dot(relativeNorm, d2 + d3);
	float j = (denominator == 0.0f) ? 0.0f : numerator / denominator;
	
	if (data.getContacts().size() > 0 && j != 0.0f)
		j /= (float)data.getContacts().size();

	glm::vec3 impulse = relativeNorm * j;

	auto halfDepth = data.getDepth() / 2.0f;
	rb2.translate(halfDepth*relativeNorm);

	if (glm::length(impulse) > 0.0f)
	{
		rb1.getVel() = rb1.getVel() - impulse * invMass1;
		rb2.getVel() = rb2.getVel() + impulse * invMass2;
		rb1.setAngVel(rb1.getAngVel() - (glm::vec3(glm::vec4(glm::cross(r1, impulse), 0.0f) * i1)));
		rb2.setAngVel(rb2.getAngVel() + (glm::vec3(glm::vec4(glm::cross(r2, impulse), 0.0f) * i2)));
	}

	glm::vec3 t = relativeVel - (relativeNorm * glm::dot(relativeVel, relativeNorm));

	if (CMP(glm::length2(t), 0.0f))
		return;

	t = glm::normalize(t);

	numerator = -glm::dot(relativeVel, t);
	d1 = invMassSum;
	d2 = glm::cross(glm::vec3(glm::vec4(glm::cross(r1, t), 1.0f)* i1), r1);
	d3 = glm::cross(glm::vec3(glm::vec4(glm::cross(r2, t), 1.0f)* i2), r2);
	denominator = d1 + glm::dot(t, d2 + d3);

	if (denominator == 0.0f)
		return;
	
	float jt = numerator / denominator;
	if (data.getContacts().size() > 0.0f &&jt != 0.0f)
		jt /= (float)data.getContacts().size();
	
	if (CMP(jt, 0.0f))
		return;
	
	float friction = 0.3f;
	if (jt> j * friction)
		jt = j * friction;
	else if (jt< -j * friction)
		jt = -j * friction;
	
	glm::vec3 tangentImpuse = t * jt;
	glm::vec3 vt = relativeVel - glm::dot(relativeVel, relativeNorm) * relativeNorm;
	float mu = 0.5f;
	glm::vec3 jFriction = -mu * glm::length(j) * glm::normalize(vt);

	if (glm::length(jFriction) > 0.0f)
	{
		rb1.setVel(rb1.getVel() - (jFriction / rb1.getMass()));
		rb1.setAngVel(rb1.getAngVel() - (rb1.getInvInertia() * glm::cross(r1, jFriction)));
		rb2.setVel(rb2.getVel() + (jFriction / rb2.getMass()));
		rb2.setAngVel(rb2.getAngVel() + (rb2.getInvInertia() * glm::cross(r2, jFriction)));
	}
}
std::set<Vertex> collidingVerts(float yCoordinate, RigidBody &rb)
{
	std::set<Vertex> collidingVertices;

	// Check if it collides with plane
	for (auto v : rb.getMesh().getVertices())
	{
		// check the y coord of: localCoord * sclae + worldCoord
		// note: with opengl you need to use mat * vec
		glm::vec3 wCoord = glm::mat3(rb.getMesh().getModel()) * v.getCoord() + rb.getPos();
		if (wCoord.y < yCoordinate)
		{
			collidingVertices.insert(wCoord);
		}
	}

	return collidingVertices;
}

void collisionResponce(RigidBody &rb, Mesh plane, std::set<Vertex>collidingVertices)
{
	//std::cout << "collides" << std::endl;
	Vertex lowestVertex;
	// translate up on y axis by the lowest vertex on the y axis
	for (auto v : collidingVertices)
		lowestVertex = v;
	
	for (auto v : collidingVertices)
	{
		if (v.getCoord().y < lowestVertex.getCoord().y)
			lowestVertex = v;
	}

	glm::vec3 displacement = glm::vec3(0.0f);
	displacement.y = glm::abs(lowestVertex.getCoord().y);
	rb.translate(displacement);
	if (glm::length(rb.getVel()) + glm::length(rb.getAngVel()) < 0.05f)
	{
		rb.getVel() = glm::vec3(0.0f);
		rb.setAngVel(glm::vec3(0.0f));
		rb.getAcc() = glm::vec3(0.0f);
		displacement.y = 0.0f;
	}

	glm::vec3 sum = glm::vec3(0.0f);
	for (auto v : collidingVertices)
		sum += v.getCoord();
	Vertex averageCollidingPoint = Vertex(sum / collidingVertices.size());

	//Impulse
	glm::vec3 r = averageCollidingPoint.getCoord() - rb.getPos();
	glm::vec3 relVel = rb.getVel() + glm::cross(rb.getAngVel(), r);
	glm::vec3 n = glm::normalize(glm::vec3(0.0f, 1.0f, 0.0f));
	float e = 0.2f;
	glm::vec3 j = (-(1 + e) * relVel * n) / (pow(rb.getMass(), -1) + n * glm::cross((rb.getInvInertia() * (glm::cross(r, n))), r));
	// apply j
	if (glm::length(j) > 0.0f)
	{
		rb.setVel(rb.getVel() + (j / rb.getMass()));
		rb.setAngVel(rb.getAngVel() + (rb.getInvInertia() * glm::cross(r, j)));
	}
	// friction
	glm::vec3 vt = relVel - glm::dot(relVel, n) * n;
	float mu = 0.5f;
	glm::vec3 jFriction = -mu * glm::length(j) * glm::normalize(vt);

	if (glm::length(j) > 0)
	{
		rb.setVel(rb.getVel() + (jFriction / rb.getMass()));
		rb.setAngVel(rb.getAngVel() + (rb.getInvInertia() * glm::cross(r, jFriction)));
	}
}

#pragma region INTEGRATION METHODS (POSITION AND ROTATION)

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