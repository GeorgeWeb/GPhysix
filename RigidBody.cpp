#include "RigidBody.h"
#include <iostream>

using namespace GPhysix;

RigidBody::RigidBody(const Mesh &mesh)
{
	setMesh(mesh);
}

void RigidBody::move()
{
	setVel(this->getVel());
	setPos(this->getPos()); 
}

void RigidBody::updateInvInertia()
{
	float h = m_mesh.getScaleVec().x * 2.0f;
	float d = m_mesh.getScaleVec().z * 2.0f;
	float w = m_mesh.getScaleVec().y * 2.0f;

	glm::mat3 inertia = glm::mat3(0.0f);
	inertia[0][0] = m_mass / 12.0f * (h * h + d * d);
	inertia[1][1] = m_mass / 12.0f * (w * w + d * d);
	inertia[2][2] = m_mass / 12.0f * (w * w + h * h);

	setInvInertia(glm::inverse(inertia));
}
