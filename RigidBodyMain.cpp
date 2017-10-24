#include "Simulation.h"

#ifdef RIGID_BODY

using namespace GPhysix;

static Simulation<RigidBody> sim;

void Update(std::vector<RigidBody>& rigidBodies)
{
	// apply forces to the rigid body to calc. acceleration
	for (auto rigBody : rigidBodies)
		rigBody.setAcc(rigBody.applyForces(rigBody.getPos(), rigBody.getVel()));
	// time-step & movement
	while (sim.getAccumultor() >= Time::deltaTime)
	{
		for (auto rigBody : rigidBodies)
		{
			// integrate Pos, Vel
			sim.Integrate(rigBody);
			// move particles
			rigBody.move(rigBody.getVel(), rigBody.getPos());
			// Rotation
			glm::vec3& dRot = rigBody.getAngVel() * Time::deltaTime;
			if (glm::dot(dRot, dRot) > 0) rigBody.rotate(sqrt(glm::dot(dRot, dRot)), dRot);
		}

		// recalculate frames
		sim.getAccumultor() -= Time::deltaTime;
	}
}

void Render(std::vector<Mesh>& meshes)
{
	for(const auto& mesh : meshes) sim.Draw(mesh);
}

auto main(void) -> int
{
	// global shader for all drawable objects
	auto shader = Shader("resources/shaders/core.vert", "resources/shaders/core.frag");

	// create ground plane
	auto plane = Mesh();
	plane.scale(glm::vec3(20.0f, 20.0f, 20.0f));
	plane.setShader(shader);

	// create rigid body cube from mesh
	auto cube = RigidBody(Mesh(Mesh::CUBE));
	cube.getMesh().setShader(shader);

	// cube motion values
	cube.translate(glm::vec3(0.0f, 5.0f, 0.0f));
	cube.setVel(glm::vec3(0.0f, 7.0f, 0.0f));
	cube.setAngVel(glm::vec3(0.0f, 2.0f, 0.0f));

	// Create forces
	Gravity g = Gravity();
	// add forces to the cube rigid body
	cube.addForce(&g);

	// create rigid body vector out of all rigid bodies in the scene
	std::vector<RigidBody> rigidBodies { cube };
	// create meshes vector out of all meshes in the scene
	std::vector<Mesh> meshes { plane, cube.getMesh() };
	// game loop
	while (sim.IsRunning())
	{
		sim.BeginFrames();

		// TODO: Use Lambdas in Simulation for render and update and abstract this while!!!
		Update(rigidBodies);
		Render(meshes);

		sim.EndFrames();
	}

	// close the program
	sim.Exit();

	return EXIT_SUCCESS;
}

#endif