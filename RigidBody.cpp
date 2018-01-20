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
	float w = m_mesh.getScaleVec().x * 2.0f;
	float h = m_mesh.getScaleVec().y * 2.0f;
	float d = m_mesh.getScaleVec().z * 2.0f;

	glm::mat3 inertia = glm::mat3(0.0f);
	inertia[0][0] = m_mass / 12.0f * (h * h + d * d);
	inertia[1][1] = m_mass / 12.0f * (w * w + d * d);
	inertia[2][2] = m_mass / 12.0f * (w * w + h * h);
	
	setInvInertia(glm::inverse(inertia));
}

CollisionManifold RigidBody::canCollideStatic()
{
	if (m_collider.getType() == TYPE::OBB)
	{
		return m_collider.intersect(getOrientedBoxCollider(), Collider::getPlaneCollider());
	}
	if (m_collider.getType() == TYPE::SPHERE)
	{
		return m_collider.intersect(getSphereCollider(), Collider::getPlaneCollider());
	}
	else
	{
		std::cout << "Error: Some of these STATIC collisions are not implemented yet.\n";
		return CollisionManifold();
	}
}


CollisionManifold RigidBody::canCollide(RigidBody* other)
{
	if (m_collider.getType() == TYPE::OBB && other->getCollider().getType() == TYPE::OBB)
	{
		return m_collider.intersect(getOrientedBoxCollider(), other->getOrientedBoxCollider());
	}
	else if (m_collider.getType() == TYPE::OBB && other->getCollider().getType() == TYPE::SPHERE)
	{
		return m_collider.intersect(other->getSphereCollider(), getOrientedBoxCollider());
	}
	else if (m_collider.getType() == TYPE::SPHERE && other->getCollider().getType() == TYPE::SPHERE)
	{
		return m_collider.intersect(getSphereCollider(), other->getSphereCollider());
	}
	else if (m_collider.getType() == TYPE::SPHERE && other->getCollider().getType() == TYPE::OBB)
	{
		return m_collider.intersect(getSphereCollider(), other->getOrientedBoxCollider());
	}
	else
	{
		std::cout << "Error: Some of these DYNAMIC collisions are not implemented yet.\n";
		return CollisionManifold();
	}

}

OBB RigidBody::getOrientedBoxCollider()
{
	auto edgeLen = getScaleVec();
	return OBB(getPos(), glm::mat3(getRotate()), edgeLen);
}

BoundingSphere RigidBody::getSphereCollider()
{
	auto R = (getScaleVec().x + getScaleVec().y + getScaleVec().z) / 3.0f;
	return BoundingSphere(getPos(), R);
}
