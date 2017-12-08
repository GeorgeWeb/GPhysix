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

IntersectData RigidBody::canCollideStatic()
{
	if (m_collider.getType() == TYPE::OBB)
	{
		return m_collider.intersect(getOrientedBoxCollider(), getPlaneCollider());
	}
	else if (m_collider.getType() == TYPE::AABB)
	{
		return m_collider.intersect(getAxisAlignedBoxCollider(), getPlaneCollider());
	}
	else if (m_collider.getType() == TYPE::SPHERE)
	{
		return m_collider.intersect(getSphereCollider(), getPlaneCollider());
	}
	else
	{
		std::cout << "Error: A collision between these shapes is not implemented yet.\n";
		return IntersectData(false, .0f);
	}
}

IntersectData RigidBody::canCollideDynamic(RigidBody* other)
{
	if (m_collider.getType() == TYPE::AABB && other->getCollider().getType() == TYPE::AABB)
	{
		return m_collider.intersect(getAxisAlignedBoxCollider(), other->getAxisAlignedBoxCollider());
	}
	else if (m_collider.getType() == TYPE::AABB && other->getCollider().getType() == TYPE::OBB)
	{
		return m_collider.intersect(getAxisAlignedBoxCollider(), other->getOrientedBoxCollider());
	}
	else if (m_collider.getType() == TYPE::OBB && other->getCollider().getType() == TYPE::OBB)
	{
		return m_collider.intersect(getOrientedBoxCollider(), other->getOrientedBoxCollider());
	}
	else if (m_collider.getType() == TYPE::SPHERE && other->getCollider().getType() == TYPE::SPHERE)
	{
		return m_collider.intersect(getSphereCollider(), other->getSphereCollider());
	}
	else if (m_collider.getType() == TYPE::SPHERE && other->getCollider().getType() == TYPE::AABB)
	{
		return m_collider.intersect(getSphereCollider(), other->getAxisAlignedBoxCollider());
	}
	else if (m_collider.getType() == TYPE::SPHERE && other->getCollider().getType() == TYPE::OBB)
	{
		return m_collider.intersect(getSphereCollider(), other->getOrientedBoxCollider());
	}
	else
	{
		std::cout << "Error: A collision between these shapes is not implemented yet.\n";
		return IntersectData(false, .0f);
	}
}

Plane RigidBody::getPlaneCollider()
{
	return Plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f);
}

AABB RigidBody::getAxisAlignedBoxCollider()
{
	glm::vec3 min, max;

	// get the zero-th coordinate vector of each vertex in world position
	// and set it as min and max value to begin comparison on.
	Vertex initVert = *(m_mesh.getVertices().cbegin());
	min = glm::mat3(m_mesh.getModel()) * initVert.getCoord() + getPos();
	max = glm::mat3(m_mesh.getModel()) * initVert.getCoord() + getPos();

	for (auto vertex : m_mesh.getVertices())
	{
		// get coordinate vector of each vertex in world position
		glm::vec3 coord = glm::mat3(m_mesh.getModel()) * vertex.getCoord() + getPos();

		// find minimum X,Y,Z vertex coordinate points
		if (coord.x < min.x) min.x = coord.x;
		if (coord.y < min.y) min.y = coord.y;
		if (coord.z < min.z) min.z = coord.z;

		// find maximum X,Y,Z vertex coordinate points
		if (coord.x > max.x) max.x = coord.x;
		if (coord.y > max.y) max.y = coord.y;
		if (coord.z > max.z) max.z = coord.z;
	}

	return AABB(min, max);
}

OBB RigidBody::getOrientedBoxCollider()
{
	auto halfEdgeLen = .9 * getScaleVec();
	return OBB(getPos(), glm::mat3(getRotate()), halfEdgeLen);
}

BoundingSphere RigidBody::getSphereCollider()
{
	auto R = (getScaleVec().x + getScaleVec().y + getScaleVec().z) / 3.0f;
	return BoundingSphere(getPos(), R);
}
