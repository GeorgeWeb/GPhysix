#include "Force.h"
#include "Body.h"

using namespace GPhysix;

Gravity::Gravity()
{ }

Gravity::Gravity(const glm::vec3 &gravity) : m_gravity(gravity)
{ }

inline const glm::vec3 &Gravity::apply(float mass, const glm::vec3 &pos, const glm::vec3 &vel)
{
	return mass * m_gravity;
}