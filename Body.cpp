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
	glm::vec3 fAccumulator = glm::vec3(0.0f);

	auto applyForceToEachBody = [&](auto *force)
	{ 
		fAccumulator += force->apply(m_mass, pos, vel);
	};

	std::for_each(m_forces.begin(), m_forces.end(), applyForceToEachBody);

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