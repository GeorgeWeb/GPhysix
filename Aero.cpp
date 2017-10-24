#include "Force.h"
#include "Particle.h"

using namespace GPhysix;

Aero::Aero()
{ }

Aero::Aero(const float km, const float kd)
	: m_km(km), m_kd(kd)
{ }

Aero::~Aero()
{ }

inline const glm::vec3 &Aero::apply(float mass, const glm::vec3 &pos, const glm::vec3 &vel)
{
	// vSurface = v1 + v2 + v3
	glm::vec3 vSurf = (m_Triangle[0]->getVel() + m_Triangle[1]->getVel() + m_Triangle[2]->getVel()) / 3.0f;
	glm::vec3 vAir = m_wind->getWind();
	// v = vSurface - vAir
	glm::vec3 v = vSurf - vAir;

	// cross product of the diference between the position vectors of the triangle
	glm::vec3 crossTri = glm::cross((m_Triangle[1]->getPos() - m_Triangle[0]->getPos()), (m_Triangle[2]->getPos() - m_Triangle[0]->getPos()));
	// initial area: ao = 1/2 * || (r2 - r1)x(r3 - r1) ||
	float a0 = 0.5f * glm::length(crossTri);
	// the normal of the surface: n = (r2 - r1)x(r3 - r1) / || (r2 - r1)x(r3 - r1) ||
	glm::vec3 n = glm::normalize(crossTri);
	// the area of the surface: a = a0 * (v . n) / || v ||
	float a = (a0 * (glm::dot(v, n))) / glm::length(v);
	
	// the aerodynamic force: fAero = 1/2 * q * || v ||^2 * Cd * a * n
	glm::vec3 fAero = 0.5f * m_km * (glm::length(v) * glm::length(v)) * m_kd * a * n;
	// get the medium of fAero
	return -fAero / 3.0f;
}