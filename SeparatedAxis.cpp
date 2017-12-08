#include "Collider.h"

using namespace GPhysix;

Interval SAT::getInterval(AABB aabb, const glm::vec3 axis)
{
	glm::vec3 i = aabb.getMinExtents();
	glm::vec3 a = aabb.getMaxExtents();

	glm::vec3 vertex[8] =
	{
		glm::vec3(i.x, a.y, a.z),
		glm::vec3(i.x, a.y, i.z),
		glm::vec3(i.x, i.y, a.z),
		glm::vec3(i.x, i.y, i.z),
		glm::vec3(a.x, a.y, a.z),
		glm::vec3(a.x, a.y, i.z),
		glm::vec3(a.x, i.y, a.z),
		glm::vec3(a.x, i.y, i.z)
	};

	// projecting each vertex onto the provided axes, and
	// store the min and max vertices in an interval structure.
	Interval result;
	result.min = result.max = glm::dot(axis, vertex[0]);
	for (int i = 1; i < 8; i++)
	{
		float projection = glm::dot(axis, vertex[i]);
		result.min = (projection < result.min) ? projection : result.min;
		result.max = (projection > result.max) ? projection : result.max;
	}

	return result;
}

Interval SAT::getInterval(OBB obb, const glm::vec3 axis)
{
	glm::vec3 vertex[8];
	// find the center, extents, and axis of the OBB
	glm::vec3 C = obb.getPosition();
	glm::vec3 E = obb.getSize();
	const float* o = glm::value_ptr(obb.getOrientation());
	// OBB Axis
	glm::vec3 A[] =
	{
		glm::vec3(o[0], o[1], o[2]),
		glm::vec3(o[3], o[4], o[5]),
		glm::vec3(o[6], o[7], o[8])
	};

	// use the center, extents, and local axis to find the actual vertices
	vertex[0] = C + A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
	vertex[1] = C - A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
	vertex[2] = C + A[0] * E[0] - A[1] * E[1] + A[2] * E[2];
	vertex[3] = C + A[0] * E[0] + A[1] * E[1] - A[2] * E[2];
	vertex[4] = C - A[0] * E[0] - A[1] * E[1] - A[2] * E[2];
	vertex[5] = C + A[0] * E[0] - A[1] * E[1] - A[2] * E[2];
	vertex[6] = C - A[0] * E[0] + A[1] * E[1] - A[2] * E[2];
	vertex[7] = C - A[0] * E[0] - A[1] * E[1] + A[2] * E[2];

	// project each vertex onto the provided axes and
	// store the min and max projection in an interval structure
	Interval result;
	result.min = result.max = glm::dot(axis, vertex[0]);
	for (int i = 1; i < 8; i++)
	{
		float projection = glm::dot(axis, vertex[i]);
		result.min = (projection < result.min) ? projection : result.min;
		result.max = (projection > result.max) ? projection : result.max;
	}

	return result;
}

bool SAT::overlapOnAxis(AABB aabb, OBB obb, const glm::vec3 axis)
{
	Interval a = getInterval(aabb, axis);
	Interval b = getInterval(obb, axis);
	return (b.min <= a.max) && (a.min <= b.max);
}


bool SAT::overlapOnAxis(OBB obb1, OBB obb2, const glm::vec3 axis)
{
	Interval a = getInterval(obb1, axis);
	Interval b = getInterval(obb2, axis);
	return (b.min <= a.max) && (a.min <= b.max);
}

const glm::vec3& SAT::getMin(OBB obb)
{
	glm::vec3 p1 = obb.getPosition();
	glm::vec3 p2 = obb.getPosition() + obb.getSize();

	return glm::vec3(fminf(p1.x, p2.x), fminf(p1.y, p2.y), fminf(p1.z, p2.z));
}

const glm::vec3& SAT::getMax(OBB obb)
{
	glm::vec3 p1 = obb.getPosition();
	glm::vec3 p2 = obb.getPosition() + obb.getSize();

	return glm::vec3(fmaxf(p1.x, p2.x), fmaxf(p1.y, p2.y), fmaxf(p1.z, p2.z));
}