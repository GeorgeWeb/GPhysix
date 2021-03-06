#include "Simulation.h"

// Std. Includes
#include <random>

using namespace GPhysix;

#ifdef MASS_SPRINGS
// Chain5, Chain10, Trampoline, Flag
#define Trampoline

static Simulation<Particle> sim;

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
#endif

Gravity grav;
Drag drag;
Aero aero;
Wind wind;

Mesh plane;
Mesh boundingBox;

/*** HELPER FUNCTION PROTOTYPES ***/
void CreateHookForces(const float ks, const float kd, const float rest);
void AddAerodynamicForce();
void LinkHooksWithParticles();
void UpdateChainOfFive();
void UpdateChainOfTen();
void UpdateTrampoline();
void UpdateFlag();
/*** !HELPER FUNCTION PROTOTYPES ***/

void Update()
{
#ifdef Chain5

	UpdateChainOfFive(); ///> TODO: Manipulate dampening based on bool param (Use enum instead of bool)

#elif defined (Chain10)

	UpdateChainOfTen(); ///> TODO: Manipulate dampening & collision based on bool param (Use enum instead of bool)

#elif defined (Flag)

	UpdateFlag(); ///> TODO: Manipulate collision & wind based on bool params (Use enum instead of bool)

#elif defined (Trampoline)

	UpdateTrampoline(); ///> TODO: Manipulate collision & friction based on bool params (Use enum instead of bool)

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

auto main(void) -> int
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
	boundingBox.translate(glm::vec3(-20.0f, 0.0f, -20.0f));
	#else
	boundingBox.translate(glm::vec3(-10.0f, 0.0f, -10.0f));
	#endif
	/*** FORCES **/
	drag.setDrag(glm::vec3(1.25f, .5f, .5f));
	aero.setAero(1.25f, 1.1f);
	wind.setWind(glm::vec3(6.0f, 4.0f, 2.0f));

	#if (defined Chain5) || (defined Chain10)
	size_t partCount = 0;
	float spanX = -4.0f;
	// create 1d particles and add 1d hook forces to them
	std::for_each(particles.begin(), particles.end(), [&](Particle &particle)
	{
		// create each particle with default values
		#ifdef Chain5 
		sim.CreateParticle(particle, glm::vec3(spanX + 2.0f, plane.getPos().y + 7.5f, 0.0f));
		#else
		sim.CreateParticle(particle, glm::vec3(spanX, plane.getPos().y + 5.0f, 0.0f));
		#endif
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
	float spanX = 3.0f;
	float spanZ = 1.0f;
	// Create 2d particles and add 2d hook forces to them
	std::for_each(particles.begin(), particles.end(), [&](std::array<Particle, particleSize> &particleRows)
	{
		std::for_each(particleRows.begin(), particleRows.end(), [&](Particle &particle)
		{
			// create each particle with default values
			#ifdef Trampoline
			sim.CreateParticle(particle, glm::vec3(-5.0f + spanX, plane.getPos().y + 7.5f, spanZ));
			#elif defined (Flag)
			sim.CreateParticle(particle, glm ::vec3(-5.0f + spanX, plane.getPos().y + 10.0f, spanZ));
			#endif
			// add force values to particles (ignore the first one - stationary)
			particle.addForce(&grav);
			particle.addForce(&drag);
			particle.addForce(&wind);
			// update 'x-pos' span
			spanX++;
			// update particle index counter
			partColCount++;
		});
		spanZ -= 1.0f;
		spanX = 3.0f;
		partRowCount++;
	});
	#else	
	// TODO: Error it somehow
	return;
	#endif
	float ks, kd, rest;
#ifdef Trampoline
	ks = 9.0f; kd = 2.0f; rest = .5f;
#elif defined (Flag)
	ks = 25.0f; kd = 15.0f; rest = .5f;
#elif defined (Chain10)
	ks = 10.0f; kd = 20.0f; rest = .5f;
#else
	ks = 50.0f; kd = 0.0f; rest = 1.0f;
#endif
	
	// Add Hook forces to particles
	CreateHookForces(ks, kd, rest);
	LinkHooksWithParticles();

	// Link Aerodyamic drag force with particles
	AddAerodynamicForce();

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

void CreateHookForces(const float ks, const float kd, const float rest)
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
			hook = Hook::Hook(ks, kd, rest);
		});
	});
	// !LEFT->RIGHT HOOK FORCES

	// RIGHT->LEFT HOOK FORCES
	std::for_each(rightHooks.begin(), rightHooks.end(), [&](std::array<Hook, 9> &hooks)
	{
		std::for_each(hooks.begin(), hooks.end(), [&](Hook &hook)
		{
			hook = Hook::Hook(ks, kd, rest);
		});
	});
	// !RIGHT->LEFT HOOK FORCES

	// DOWN->UP HOOK FORCES
	std::for_each(downHooks.begin(), downHooks.end(), [&](std::array<Hook, 10> &hooks)
	{
		std::for_each(hooks.begin(), hooks.end(), [&](Hook &hook)
		{
			hook = Hook::Hook(ks, kd, rest);
		});
	});
	// !DOWN->UP HOOK FORCES

	// UP->DOWN HOOK FORCES
	std::for_each(upHooks.begin(), upHooks.end(), [&](std::array<Hook, 10> &hooks)
	{
		std::for_each(hooks.begin(), hooks.end(), [&](Hook &hook)
		{
			hook = Hook::Hook(ks, kd, rest);
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

void AddAerodynamicForce()
{
#ifdef Flag
	// add wind force to the aerodynamic drag force
	aero.addWind(&wind);
	for (size_t row = 0; row < particleSize - 2; row++)
	{
		for (size_t col = 0; row < particleSize - 2; row++)
		{
			// First triangle
			aero.setTriangle(&particles[row][col], &particles[row + 1][col], &particles[row][col + 1]);
			particles[row][col].addForce(&aero);
			particles[row + 1][col].addForce(&aero);
			particles[row][col + 1].addForce(&aero);
			// Second triangle
			aero.setTriangle(&particles[row + 1][col + 1], &particles[row + 1][col], &particles[row][col + 1]);
			particles[row + 1][col + 1].addForce(&aero);
			particles[row + 1][col].addForce(&aero);
			particles[row][col + 1].addForce(&aero);
		}
	}
#endif
}

void UpdateChainOfFive()
{
	#ifdef Chain5
	// apply forces to particles to calc. acceleration
	for (int i = 1; i < particles.size(); i++)
		particles[i].setAcc(particles[i].applyForces(particles[i].getPos(), particles[i].getVel()));

	// time-step & movement
	while (sim.getAccumultor() >= Time::deltaTime)
	{
		for (int i = 1; i < particles.size(); i++)
		{
			// integrate Pos, Vel
			sim.IntegrateSI(particles[i]);
			// move particles
			particles[i].move(particles[i].getVel(), particles[i].getPos());
		}
		// recalculate frames
		sim.getAccumultor() -= Time::deltaTime;
	}
	#endif
}

void UpdateChainOfTen()
{
	#ifdef Chain10
	// apply forces to particles to calc. acceleration
	for (int i = 1; i < particles.size() - 1; i++)
		particles[i].setAcc(particles[i].applyForces(particles[i].getPos(), particles[i].getVel()));

	// time-step & movement
	while (sim.getAccumultor() >= Time::deltaTime)
	{
		for (int i = 1; i < particles.size() - 1; i++)
		{
			// integrate Pos, Vel
			sim.IntegrateSI(particles[i]);
			// move particles
			particles[i].move(particles[i].getVel(), particles[i].getPos());
		}
		// recalculate frames
		sim.getAccumultor() -= Time::deltaTime;
	}

	// Apply collision to particles - ignore the first(stationary) particle
	std::for_each(particles.begin() + 1, particles.end(), [&](Particle &particle) { sim.CollisionDetection(boundingBox, particle); });
	#endif
}

void UpdateTrampoline()
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
	while (sim.getAccumultor() >= Time::deltaTime)
	{
		// apply forces to particles to calc. acceleration
		std::for_each(particles.begin() + 1, particles.end() - 1, [&](std::array<Particle, particleSize> &particleRows)
		{
			std::for_each(particleRows.begin() + 1, particleRows.end() - 1, [&](Particle &particle)
			{
				// integrate Pos, Vel
				sim.IntegrateSI(particle);
				// move particles
				particle.move(particle.getVel(), particle.getPos());
			});
		});
		// recalculate frames
		sim.getAccumultor() -= Time::deltaTime;
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

void UpdateFlag()
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
	while (sim.getAccumultor() >= Time::deltaTime)
	{
		// apply forces to particles to calc. acceleration
		std::for_each(particles.begin(), particles.end() , [&](std::array<Particle, particleSize> &particleRows)
		{
			std::for_each(particleRows.begin() + 1, particleRows.end(), [&](Particle &particle)
			{
				// integrate Pos, Vel
				sim.IntegrateSI(particle);
				// move particles
				particle.move(particle.getVel(), particle.getPos());
			});
		});
		// recalculate frames
		sim.getAccumultor() -= Time::deltaTime;
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
#endif