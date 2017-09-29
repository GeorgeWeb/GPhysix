#include "DynamicForce.hpp"

using namespace GPhysix;

DynamicForce::DynamicForce()
{
	
}

DynamicForce::~DynamicForce()
{
	
}

void DynamicForce::plugIn(IForce *newForce)
{
	m_force += newForce->getForce();
}

void GPhysix::DynamicForce::setForce(const glm::vec3 &newForce)
{
	m_force = newForce;
}
