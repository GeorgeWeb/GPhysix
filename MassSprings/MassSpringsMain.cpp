#if 1

#include "Simulation.hpp"

using namespace GPhysix;

static Simulation sim;

Mesh plane;
Mesh boundingBox;
std::array<Particle, 10> particles;

Gravity grav;
Drag drag;

std::array<Hook, 9> hooks;
std::array<Hook, 9> reverseHooks;

void Update()
{
	// apply forces to particles to calc. acceleration
	for (int i = 1; i < particles.size() - 1; i++)
		particles[i].setAcc(particles[i].applyForces(particles[i].getPos(), particles[i].getVel()));

	// time-step & movement
	while (sim.getAccumultor() >= sim.deltaTime)
	{
		for (int i = 1; i < particles.size() - 1; i++)
		{
			// integrate Pos, Vel
			sim.IntegrateSI(particles[i], sim.deltaTime);
			// move particles
			particles[i].move(particles[i].getVel(), particles[i].getPos());
		}
		// recalculate frames
		sim.getAccumultor() -= sim.deltaTime;
	}

	// Apply collision to particles - ignore the first(stationary) particle
	std::for_each(particles.begin() + 1, particles.end(), [&](Particle &particle) { sim.CollisionDetection(boundingBox, particle); });
}

void ChainOfFive(bool damp);
void ChainOfTen(bool damp, bool collision);
/*
void HorizontalCloth(bool collision);
void VerticalCloth(bool collision);
*/

void Render()
{
	// draw groud plane
	sim.Draw(plane);
	// draw particles
	std::for_each(particles.begin(), particles.end(), [&](auto &particle) { sim.Draw(particle.getMesh()); });
}

int main()
{
	// create ground plane
	plane = Mesh::Mesh();
	plane.scale(glm::vec3(20.0f, 20.0f, 20.0f));
	plane.setShader(Shader("resources/shaders/core.vert", "resources/shaders/core.frag"));

	// create non-visual , no need for shaders
	// collision boundary
	boundingBox = Mesh::Mesh();
	boundingBox.scale(glm::vec3(20.0f, 20.0f, 20.0f));
	boundingBox.translate(glm::vec3(-11.0f, .0f, -11.0f));
	
	/*** FORCES **/
	grav = Gravity::Gravity();
	drag = Drag::Drag();

	float ks, kd, rest; 
	ks = 10.0f;
	kd = 5.0;
	rest = 0.05f;
	
	for (auto &hook : hooks)
	{
		hook = Hook::Hook(ks, kd, rest);
	}
	
	for (auto &revHook : reverseHooks)
	{
		revHook = Hook::Hook(ks, kd, rest);
	}

	// create particles
	size_t partCount = 0;
	float span = 0.0f;
	std::for_each(particles.begin(), particles.end(), [&](Particle &particle)
	{
		// create each particle with default values
		sim.CreateParticle(particle, glm::vec3(span, plane.getPos().y + 7.5f, 0.0f));
		// add force values to particles (ignore the first one - stationary)
		if (partCount > 0)
		{
			particle.addForce(&grav);
			particle.addForce(&drag);
		}
		// update count & span
		span++;
		partCount++;
	});
	
	/*** TODO: USE ITERATORS LATER WHEN REFACTORING!!! ***/
	// add pair of bodies(particles) to each hook
	for (int i = 0; i < hooks.size(); i++)
		hooks[i].setBodies(std::make_pair(&particles[i], &particles[i + 1]));
	// add pair of bodies(particles) to each reverse hook
	for (int i = reverseHooks.size() - 1; i >= 0; i--)
		reverseHooks[i].setBodies(std::make_pair(&particles[i + 1], &particles[i]));
	// add hook forces to particles
	for (int i = 1; i < particles.size(); i++)
	{
		particles[i].addForce(&hooks[i - 1]);
		if (i < particles.size() - 1) particles[i].addForce(&reverseHooks[i]);
	}
	
	// run the simulation program
	// game loop
	while (sim.IsRunning())
	{	
		sim.BeginFrames();
		
		// TODO: Use Lambdas in Simulation for render and update and abstract this while!!!
		Update();
		Render();
		
		sim.EndFrames();
	}

	// close the program
	sim.Exit();

	return EXIT_SUCCESS;
}

#endif

void ChainOfFive(bool damp)
{
}

void ChainOfTen(bool damp, bool collision)
{
}
