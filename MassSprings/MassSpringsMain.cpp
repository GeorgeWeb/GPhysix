#include "Simulation.hpp"

using namespace GPhysix;

// Chain5, Chain10, Flag, Trampoline
#define Flag

static Simulation sim;

#ifdef Chain5
constexpr size_t particleSize = 5;
constexpr size_t hookSize = 4;

std::array<Particle, particleSize> particles;
std::array<Hook, hookSize> hooks;
std::array<Hook, hookSize> reverseHooks;

#elif defined(Chain10)
constexpr size_t particleSize = 10;
constexpr size_t hookSize = 9;

std::array<Particle, particleSize> particles;
std::array<Hook, hookSize> hooks;
std::array<Hook, hookSize> reverseHooks;

#elif (defined Flag) || (defined Trampoline)
constexpr size_t particleSize = 10;

std::array<std::array<Particle, particleSize>, particleSize> particles;
std::array<std::array<Hook, 9>, 10> leftHooks;
std::array<std::array<Hook, 9>, 10> rightHooks;
std::array<std::array<Hook, 10>, 9> downHooks;
std::array<std::array<Hook, 10>, 9> upHooks;

#else
// TODO: Error it somehow
#endif

Gravity grav;
Drag drag;

Mesh plane;
Mesh boundingBox;

/*** HELPER FUNCTION PROTOTYPES ***/
void CreateHookForces(const float &ks, const float &kd, const float &rest);
void LinkHooksWithParticles();
void UpdateChainOfFive(bool damp);
void UpdateChainOfTen(bool damp, bool collision);
void UpdateTrampoline(bool collision, bool friction);
void UpdateFlag(bool collision, bool wind);
/*** !HELPER FUNCTION PROTOTYPES ***/

void Update()
{
#ifdef Chain5

	UpdateChainOfFive(false); ///> TODO: Manipulate dampening based on bool param (Use enum instead of bool)

#elif defined (Chain10)

	UpdateChainOfTen(true, true); ///> TODO: Manipulate dampening & collision based on bool param (Use enum instead of bool)

#elif defined (Flag)

	UpdateFlag(true, true); ///> TODO: Manipulate collision & wind based on bool params (Use enum instead of bool)

#elif defined (Trampoline)

	UpdateTrampoline(true, true); ///> TODO: Manipulate collision & friction based on bool params (Use enum instead of bool)

#else
	// TODO: Error it somehow
	return;
#endif		
}


void Render()
{
#ifndef Chain5
	// draw groud plane mesh
	sim.Draw(plane);
#endif

#if (defined Chain5) || (defined Chain10)
	// draw 1d particles
	std::for_each(particles.begin(), particles.end(), [&](Particle &particle)
	{
		sim.Draw(particle);
	});
#elif (defined Trampoline) || (defined Flag)
	// draw 2d particles
	std::for_each(particles.begin(), particles.end(), [&](std::array<Particle, particleSize> &particleRows)
	{
		std::for_each(particleRows.begin(), particleRows.end(), [&](Particle &particle)
		{
			sim.Draw(particle);
		});
	});
#else
	// TODO: Error it somehow
	return;
#endif
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
	#ifndef Chain10
	boundingBox.translate(glm::vec3(-20.0f, .0f, -20.0f));
	#endif
	/*** FORCES **/
	grav = Gravity::Gravity();
	drag = Drag::Drag();

	float ks = 5.0f, kd = 9.0f, rest = 0.5f;

	CreateHookForces(ks, kd, rest);

	#if (defined Chain5) || (defined Chain10)
	size_t partCount = 0;
	float spanX = 0.0f;
	// create 1d particles and add 1d hook forces to them
	std::for_each(particles.begin(), particles.end(), [&](Particle &particle)
	{
		// create each particle with default values
		sim.CreateParticle(particle, glm::vec3(spanX, plane.getPos().y + 7.5f, 0.0f));
		// add force values to particles (ignore the first one - stationary)
		if (partCount > 0)
		{
			particle.addForce(&grav);
			particle.addForce(&drag);
		}
		// update 'x-pos' span
		spanX++;
		// update particle index counter
		partCount++;
	});

	#elif (defined Trampoline) || (defined Flag)
	size_t partRowCount = 0;
	size_t partColCount = 0;
	float spanX = 1.0f;
	float spanZ = 1.0f;
	// Create 2d particles and add 2d hook forces to them
	std::for_each(particles.begin(), particles.end(), [&](std::array<Particle, particleSize> &particleRows)
	{
		std::for_each(particleRows.begin(), particleRows.end(), [&](Particle &particle)
		{
			// create each particle with default values
			sim.CreateParticle(particle, glm::vec3(-5.0f + spanX, plane.getPos().y + 7.5f, spanZ));
			// add force values to particles (ignore the first one - stationary)
			particle.addForce(&grav);
			particle.addForce(&drag);
			// update 'x-pos' span
			spanX++;
			// update particle index counter
			partColCount++;
		});
		spanZ--;
		spanX = 1.0f;
		partRowCount++;
	});
	#else	
	// TODO: Error it somehow
	return;
	#endif

	LinkHooksWithParticles();

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

void CreateHookForces(const float &ks, const float &kd, const float &rest)
{
	#if (defined Chain5) || (defined Chain10)
	// create 1d hook forces
	// forward hook forces
	for (auto &hook : hooks)
		hook = Hook::Hook(ks, kd, rest);
	// reverse/back hook forces
	for (auto &revHook : reverseHooks)
		revHook = Hook::Hook(ks, kd, rest);
	#elif (defined Trampoline) || (defined Flag)
	// create 2d hook forces

	// LEFT->RIGHT HOOK FORCES
	std::for_each(leftHooks.begin(), leftHooks.end(), [&](std::array<Hook, 9> &hooks)
	{
		std::for_each(hooks.begin(), hooks.end(), [&](Hook &hook)
		{
			hook = Hook::Hook(ks, kd, rest * 0.1f);
		});
	});
	// !LEFT->RIGHT HOOK FORCES

	// RIGHT->LEFT HOOK FORCES
	std::for_each(rightHooks.begin(), rightHooks.end(), [&](std::array<Hook, 9> &hooks)
	{
		std::for_each(hooks.begin(), hooks.end(), [&](Hook &hook)
		{
			hook = Hook::Hook(ks, kd, rest * 0.1f);
		});
	});
	// !RIGHT->LEFT HOOK FORCES

	// DOWN->UP HOOK FORCES
	std::for_each(downHooks.begin(), downHooks.end(), [&](std::array<Hook, 10> &hooks)
	{
		std::for_each(hooks.begin(), hooks.end(), [&](Hook &hook)
		{
			hook = Hook::Hook(ks, kd, rest * 2.0f);
		});
	});
	// !DOWN->UP HOOK FORCES

	// UP->DOWN HOOK FORCES
	std::for_each(upHooks.begin(), upHooks.end(), [&](std::array<Hook, 10> &hooks)
	{
		std::for_each(hooks.begin(), hooks.end(), [&](Hook &hook)
		{
			hook = Hook::Hook(ks, kd, rest * 2.0f);
		});
	});
	// !UP->DOWN HOOK FORCES
	#else
	// TODO: Error it somehow
	return;
	#endif
}

void LinkHooksWithParticles()
{
	#if (defined Chain5) || (defined Chain10)
	// TODO: USE ITERATORS LATER WHEN REFACTORING!!!
	// set bodies(particles) to each hook
	for (int i = 0; i < hooks.size(); i++)
		hooks[i].setBodies(std::make_pair(&particles[i], &particles[i + 1]));
	// set bodies(particles) to each reverse hook
	for (int i = reverseHooks.size() - 1; i >= 0; i--)
		reverseHooks[i].setBodies(std::make_pair(&particles[i + 1], &particles[i]));
	// link up&down hook forces to particles
	for (int i = 1; i < particles.size(); i++)
	{
		particles[i].addForce(&hooks[i - 1]);
		if (i < particles.size() - 1) particles[i].addForce(&reverseHooks[i]);
	}
	#elif (defined Trampoline) || (defined Flag)
	int row, col;
	// LEFT->RIGHT HOOK FORCES
	for (row = 0; row < 10; row++)
	{
		for (col = 0; col < 9; col++)
		{
			leftHooks[row][col].setBodies({ &particles[row][col], &particles[row][col + 1] });
		}
	}
	for (row = 0; row < 10; row++)
	{
		for (col = 1; col < 10; col++)
		{
			particles[row][col].addForce(&leftHooks[row][col - 1]);
		}
	}
	// !LEFT->RIGHT HOOK FORCES
	// RIGHT->LEFT HOOK FORCES
	for (row = 0; row < 10; row++)
	{
		for (col = 8; col >= 0; col--)
		{
			rightHooks[row][col].setBodies({ &particles[row][col + 1], &particles[row][col] });
		}
	}
	for (row = 0; row < 10; row++)
	{
		for (col = 0; col < 9; col++)
		{
			particles[row][col].addForce(&rightHooks[row][col]);
		}
	}
	// !RIGHT->LEFT HOOK FORCES
	
	// DOWN->UP HOOK FORCES
	for (col = 0; col < 10; col++)
	{
		for (row = 8; row >= 0; row--)
		{
			downHooks[row][col].setBodies({ &particles[row + 1][col], &particles[row][col] });
		}
	}
	for (col = 0; col < 10; col++)
	{
		for (row = 0; row < 9; row++)
		{
			particles[row][col].addForce(&downHooks[row][col]);
		}
	}
	// !DOWN->UP HOOK FORCES
	
	// UP->DOWN HOOK FORCES
	for (col = 0; col < 10; col++)
	{
		for (row = 0; row < 9; row++)
		{
			upHooks[row][col].setBodies({ &particles[row][col], &particles[row + 1][col] });
		}
	}
	for (col = 0; col < 10; col++)
	{
		for (row = 1; row < 10; row++)
		{
			particles[row][col].addForce(&upHooks[row - 1][col]);
		}
	}
	// !UP->DOWN HOOK FORCES
	#else
		// TODO: Error it somehow
		return;
	#endif
}

void UpdateChainOfFive(bool damp)
{
#ifdef Chain5
	// apply forces to particles to calc. acceleration
	for (int i = 1; i < particles.size(); i++)
		particles[i].setAcc(particles[i].applyForces(particles[i].getPos(), particles[i].getVel()));

	// time-step & movement
	while (sim.getAccumultor() >= sim.deltaTime)
	{
		for (int i = 1; i < particles.size(); i++)
		{
			// integrate Pos, Vel
			sim.IntegrateSI(particles[i], sim.deltaTime);
			// move particles
			particles[i].move(particles[i].getVel(), particles[i].getPos());
		}
		// recalculate frames
		sim.getAccumultor() -= sim.deltaTime;
	}
#endif
}

void UpdateChainOfTen(bool damp, bool collision)
{
#ifdef Chain10
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
#endif
}

void UpdateTrampoline(bool collision, bool friction)
{
#ifdef Trampoline
	// apply forces to particles to calc. acceleration
	std::for_each(particles.begin() + 1, particles.end() - 1, [&](std::array<Particle, particleSize> &particleRows)
	{
		std::for_each(particleRows.begin() + 1, particleRows.end() - 1, [&](Particle &particle)
		{
			particle.setAcc(particle.applyForces(particle.getPos(), particle.getVel()));
		});
	});

	// time-step & movement
	while (sim.getAccumultor() >= sim.deltaTime)
	{
		// apply forces to particles to calc. acceleration
		std::for_each(particles.begin() + 1, particles.end() - 1, [&](std::array<Particle, particleSize> &particleRows)
		{
			std::for_each(particleRows.begin() + 1, particleRows.end() - 1, [&](Particle &particle)
			{
				// integrate Pos, Vel
				sim.IntegrateSI(particle, sim.deltaTime);
				// move particles
				particle.move(particle.getVel(), particle.getPos());
			});
		});
		// recalculate frames
		sim.getAccumultor() -= sim.deltaTime;
	}
	
	// Apply collision to the non-static(non-velocity) particles
	std::for_each(particles.begin(), particles.end(), [&](std::array<Particle, particleSize> &particleRows)
	{
		std::for_each(particleRows.begin(), particleRows.end(), [&](Particle &particle)
		{
			sim.CollisionDetectionForCloth(boundingBox, particle);
		});
	});
	
#endif
}

void UpdateFlag(bool collision, bool wind)
{
#ifdef Flag
	// apply forces to particles to calc. acceleration
	std::for_each(particles.begin(), particles.end(), [&](std::array<Particle, particleSize> &particleRows)
	{
		std::for_each(particleRows.begin() + 1, particleRows.end(), [&](Particle &particle)
		{
			particle.setAcc(particle.applyForces(particle.getPos(), particle.getVel()));
		});
	});

	// time-step & movement
	while (sim.getAccumultor() >= sim.deltaTime)
	{
		// apply forces to particles to calc. acceleration
		std::for_each(particles.begin(), particles.end() , [&](std::array<Particle, particleSize> &particleRows)
		{
			std::for_each(particleRows.begin() + 1, particleRows.end(), [&](Particle &particle)
			{
				// integrate Pos, Vel
				sim.IntegrateSI(particle, sim.deltaTime);
				// move particles
				particle.move(particle.getVel(), particle.getPos());
			});
		});
		// recalculate frames
		sim.getAccumultor() -= sim.deltaTime;
	}
	// Apply collision to the non-static(non-velocity) particles
	std::for_each(particles.begin(), particles.end(), [&](std::array<Particle, particleSize> &particleRows)
	{
		std::for_each(particleRows.begin(), particleRows.end(), [&](Particle &particle)
		{
			sim.CollisionDetectionForCloth(boundingBox, particle);
		});
	});
#endif
}
