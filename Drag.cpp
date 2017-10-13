#include "Force.h"
#include "Body.h"

using namespace GPhysix;

Drag::Drag()
{ }

Drag::Drag(const glm::vec3 &drag) : m_drag(drag)
{ }

inline const glm::vec3 &Drag::apply(float mass, const glm::vec3 &pos, const glm::vec3 &vel)
{
	// TODO: check for division by 0
	return -(m_drag[0] * m_drag[1] * m_drag[2] * glm::dot(vel, vel) * vel / sqrt(glm::dot(vel, vel)));
}
