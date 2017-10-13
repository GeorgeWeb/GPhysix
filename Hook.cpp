#include "Force.h"
#include "Body.h"

using namespace GPhysix;

Hook::Hook()
{ }

Hook::Hook(const float ks, const float kd, const float rest)
	: m_ks(ks), m_kd(kd), m_rest(rest) { }

Hook::~Hook()
{ }

const glm::vec3 &Hook::apply(float mass, const glm::vec3 &pos, const glm::vec3 &vel)
{
	float l = glm::distance(m_bodies.second->getPos(), m_bodies.first->getPos()); ///> sqrt(second->pos[i]^2 - first->pos[i])...
	glm::vec3 e = (m_bodies.second->getPos() - m_bodies.first->getPos()) / l; ///> normalized (unit) vector
	float v1 = glm::dot(e, m_bodies.first->getVel());
	float v2 = glm::dot(e, m_bodies.second->getVel());
	float Fsd = -m_ks * (m_rest - l) - m_kd * (v1 - v2);
	
	return  -(Fsd * e);
}
