#include "Drag.h"
#include "Body.h"

using namespace GPhysix;

Drag::Drag()
{ }

Drag::Drag(const glm::vec3 &drag) : m_drag(drag)
{ }

Drag::~Drag()
{ }

inline const glm::vec3 &Drag::apply(const float &mass, const glm::vec3 &pos, const glm::vec3 &vel)
{
	// TODO: check for division by 0
	return -(1.25f * .5f * .05f * glm::dot(vel, vel) * vel / sqrt(glm::dot(vel, vel)));
}
