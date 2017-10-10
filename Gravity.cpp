#include "Gravity.h"
#include "Body.h"

using namespace GPhysix;

Gravity::Gravity()
{ }

Gravity::Gravity(const glm::vec3 &gravity) : m_gravity(gravity)
{ }

Gravity::~Gravity()
{ }

inline const glm::vec3 &Gravity::apply(const float &mass, const glm::vec3 &pos, const glm::vec3 &vel)
{
	return mass * getGravity();
}