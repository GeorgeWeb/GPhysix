#include "Particle.h"

using namespace GPhysix;

Particle::Particle() : Body()
{
	setMesh(Mesh::Mesh());
	scale(glm::vec3(.1f, .1f, .1f));
	rotate((GLfloat)M_PI_2, glm::vec3(1.0f, 0.0f, 0.0f));
	
	// set dynamic values
	setAcc(glm::vec3(0.0f, 0.0f, 0.0f));
	setVel(glm::vec3(0.0f, 0.0f, 0.0f));
	
	// physical properties
	setMass(1.0f);
	setCor(1.0f);
}