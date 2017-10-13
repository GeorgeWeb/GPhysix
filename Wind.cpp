#include "Force.h"

using namespace GPhysix;

Wind::Wind()
{ }

Wind::Wind(const glm::vec3 & wind) : m_wind(wind)
{ }

inline const glm::vec3 &Wind::apply(float mass, const glm::vec3 &pos, const glm::vec3 &vel)
{
	return m_wind;
}