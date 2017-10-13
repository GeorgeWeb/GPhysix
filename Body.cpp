#include "Body.h"
#include <algorithm>

using namespace GPhysix;

Body::Body()
{ }

Body::~Body()
{
	/*
	if (!getForces().empty())
	{
		auto deleteForce = [](auto *force)
		{
			if (force != nullptr)
			{
				try
				{
					delete force;
					force = nullptr;
				}
				catch(Exception &e)
				{

				}
			}
		};
		std::for_each(getForces().begin(), getForces().end(), deleteForce);
	}
	*/
}

void Body::addForce(Force *force)
{
	m_forces.push_back(force);
}

const glm::vec3 &GPhysix::Body::applyForces(const glm::vec3 &pos, const glm::vec3 &vel)
{
	// init accumulator
	glm::vec3 fAccumulator = glm::vec3(0.0f);
	// sum/accum forces
	for (auto &f : m_forces) fAccumulator += f->apply(getMass(), pos, vel);
	// return force/mass
	return fAccumulator / getMass();
}

/* TRANSFORMATION METHODS */
void Body::translate(const glm::vec3 &vect)
{
	m_pos = m_pos + vect;
	m_mesh.translate(vect);
}

void Body::rotate(const float &angle, const glm::vec3 &vect)
{
	m_mesh.rotate(angle, vect);
}

void Body::scale(const glm::vec3 &vect)
{
	m_mesh.scale(vect);
}