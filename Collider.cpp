#include "Collider.h"

using namespace GPhysix;

IntersectData Collider::intersect(AABB aabb1, AABB aabb2)
{
	glm::vec3 distances1 = aabb2.getMinExtents() - aabb1.getMaxExtents();
	glm::vec3 distances2 = aabb1.getMinExtents() - aabb2.getMaxExtents();

	glm::vec3 distances = glm::max(distances1, distances2);
	std::array<float, 3> distElements = { distances.x, distances.y, distances.z };
	float maxDistance = *(std::max_element(distElements.begin(), distElements.end()));

	return IntersectData(maxDistance < 0, maxDistance);
}

IntersectData Collider::intersect(AABB aabb, OBB obb)
{
	SAT sat;

	const float* o = glm::value_ptr(obb.getOrientation());
	glm::vec3 test[15] =
	{
		glm::vec3(1, 0, 0),				///> AABB axis 1
		glm::vec3(0, 1, 0),				///> AABB axis 2
		glm::vec3(0, 0, 1),				///> AABB axis 3
		glm::vec3(o[0], o[1], o[2]),	///> OBB axis 1
		glm::vec3(o[3], o[4], o[5]),	///> OBB axis 2
		glm::vec3(o[6], o[7], o[8])		///> OBB axis 3
	};
	
	// Fill out rest of axis
	for (int i = 0; i < 3; i++)
	{
		test[6 + i * 3 + 0] = glm::cross(test[i], test[0]);
		test[6 + i * 3 + 1] = glm::cross(test[i], test[1]);
		test[6 + i * 3 + 2] = glm::cross(test[i], test[2]);
	}
	
	for (int i = 0; i < 15; i++)
	{
		if (!sat.overlapOnAxis(aabb, obb, test[i]))
			return IntersectData(false, test[i]);
	}
	return IntersectData(true, test[8]);
}

IntersectData Collider::intersect(AABB aabb, Plane plane)
{
	glm::vec3 normal = glm::normalize(plane.getNormal());
	auto position = aabb.getCenter() + aabb.getMaxExtents();
	float pLen = aabb.getSize().x * fabsf(normal.x) + aabb.getSize().y * fabsf(normal.y) + aabb.getSize().z * fabsf(normal.z);
	float dot = glm::dot(normal, position);
	float dist = dot - plane.getDistance();

	return IntersectData(fabsf(dist) <= pLen, fabsf(dist));
}

IntersectData Collider::intersect(OBB obb1, OBB obb2)
{
	SAT sat;

	const float* o1 = glm::value_ptr(obb1.getOrientation());
	const float* o2 = glm::value_ptr(obb2.getOrientation());

	glm::vec3 test[15] =
	{
		glm::vec3(o1[0], o1[1], o1[2]),
		glm::vec3(o1[3], o1[4], o1[5]),
		glm::vec3(o1[6], o1[7], o1[8]),
		glm::vec3(o2[0], o2[1], o2[2]),
		glm::vec3(o2[3], o2[4], o2[5]),
		glm::vec3(o2[6], o2[7], o2[8])
	};

	// Fill out rest of axis
	for (int i = 0; i < 3; i++)
	{
		test[6 + i * 3 + 0] = glm::cross(test[i], test[0]);
		test[6 + i * 3 + 1] = glm::cross(test[i], test[1]);
		test[6 + i * 3 + 2] = glm::cross(test[i], test[2]);
	}

	for (int i = 0; i < 15; i++)
	{
		if (!sat.overlapOnAxis(obb1, obb2, test[i]))
			return IntersectData(false, test[i]);
	}

	// TODO: sum all axis and get the average(mid) one ...

	return IntersectData(true, test[0]);
}

IntersectData Collider::intersect(OBB obb, Plane plane)
{
	// Local variables for readability only
	const float* o = glm::value_ptr(obb.getOrientation());
	// rotation / orientation
	glm::vec3 rot[] = 
	{ 
		glm::vec3(o[0], o[1], o[2]),
		glm::vec3(o[3], o[4], o[5]),
		glm::vec3(o[6], o[7], o[8])
	};
	
	glm::vec3 normal = glm::normalize(plane.getNormal());
	float pLen = obb.getSize().x * fabsf(glm::dot(normal, rot[0])) + obb.getSize().y * fabsf(glm::dot(normal, rot[1])) + obb.getSize().z * fabsf(glm::dot(normal, rot[2]));
	
	float dot = glm::dot(normal, obb.getPosition());
	float dist = dot - plane.getDistance();

	return IntersectData(fabsf(dist) <= pLen, fabsf(dist));
}

IntersectData Collider::intersect(BoundingSphere sphere1, BoundingSphere sphere2)
{
	float radiusDistance = sphere1.getRadius() + sphere2.getRadius();
	float centerDistance = glm::length(sphere2.getCenter() - sphere1.getCenter());
	float distance = centerDistance - radiusDistance;

	return IntersectData(centerDistance < radiusDistance, distance);
}

IntersectData Collider::intersect(BoundingSphere sphere, AABB aabb)
{
	glm::vec3 cp = closestPoint(aabb, sphere.getCenter());
	float distSq = glm::length2(sphere.getCenter() - cp);
	float radiusSq = sphere.getRadius() * sphere.getRadius();

	return IntersectData(distSq < radiusSq, distSq);
}

IntersectData Collider::intersect(BoundingSphere sphere, OBB obb)
{
	glm::vec3 cp = closestPoint(obb, sphere.getCenter());
	float distSq = glm::length2(sphere.getCenter() - cp);
	float radiusSq = sphere.getRadius() * sphere.getRadius();

	return IntersectData(distSq < radiusSq, distSq);
}

IntersectData Collider::intersect(BoundingSphere sphere, Plane plane)
{
	glm::vec3 normal = glm::normalize(plane.getNormal());
	float distanceFromSphereCenter = glm::dot(normal, sphere.getCenter()) - plane.getDistance();
	float distanceFromSphere = distanceFromSphereCenter - sphere.getRadius();
	// ...
	auto radius = sphere.getRadius();
	bool sphereCenterInRange = distanceFromSphereCenter > -radius && distanceFromSphereCenter < radius;
	
	return IntersectData(sphereCenterInRange, normal * distanceFromSphere);
}

glm::vec3 Collider::closestPoint(AABB aabb, const glm::vec3& point)
{
	glm::vec3 result = point;
	glm::vec3 min = aabb.getMinExtents();
	glm::vec3 max = aabb.getMaxExtents();

	result.x = (result.x<min.x) ? min.x : result.x;
	result.y = (result.y<min.x) ? min.y : result.y;
	result.z = (result.z<min.x) ? min.z : result.z;

	result.x = (result.x>max.x) ? max.x : result.x;
	result.y = (result.y>max.x) ? max.y : result.y;
	result.z = (result.z>max.x) ? max.z : result.z;

	return result;
}

glm::vec3 GPhysix::Collider::closestPoint(OBB obb, const glm::vec3& point)
{
	glm::vec3 result = obb.getPosition();
	glm::vec3 dir = point - obb.getPosition();

	for (int i = 0; i < 3; i++)
	{
		const float* orientation = &(glm::value_ptr(obb.getOrientation()))[i * 3];
		glm::vec3 axis(orientation[0], orientation[1], orientation[2]);
		float distance = glm::dot(dir, axis);
		if (distance > obb.getSize()[i])
			distance = obb.getSize()[i];

		if (distance < -obb.getSize()[i])
			distance = -obb.getSize()[i];

		result = result + (axis * distance);
	}
	return result;
}

glm::vec3 Collider::closestPoint(BoundingSphere sphere, const glm::vec3& point)
{
	glm::vec3 sphereToPoint = point - sphere.getCenter();
	sphereToPoint = glm::normalize(sphereToPoint);
	sphereToPoint = sphereToPoint * sphere.getRadius();

	return sphereToPoint + sphere.getCenter();
}
