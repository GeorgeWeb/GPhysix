#include "Collider.h"

using namespace GPhysix;


CollisionManifold Collider::intersect(OBB obb, Plane plane)
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

	glm::vec3 intersection;
	std::vector<Line> edges;
	edges = getEdges(obb);
	std::vector<glm::vec3> resultVecs;
	int count;
	glm::vec3 result;
	Plane p = Plane();

	for (int j = 0; j < edges.size(); ++j)
	{
		if (clipToPlane(p, edges[j], &intersection))
			resultVecs.push_back(intersection);
	}

	return CollisionManifold(fabsf(dist) <= pLen, plane.getNormal(), pLen - dist, resultVecs);
}


CollisionManifold Collider::intersect(OBB obb1, OBB obb2) 
{
	CollisionManifold result = CollisionManifold();

	const float* o1 = value_ptr(obb1.getOrientation());
	const float* o2 = value_ptr(obb2.getOrientation());

	// face axis
	glm::vec3 test[15] =
	{
		glm::vec3(o1[0], o1[1], o1[2]),
		glm::vec3(o1[3], o1[4], o1[5]),
		glm::vec3(o1[6], o1[7], o1[8]),
		glm::vec3(o2[0], o2[1], o2[2]),
		glm::vec3(o2[3], o2[4], o2[5]),
		glm::vec3(o2[6], o2[7], o2[8])
	};

	// fill out rest of axis
	for (int i = 0; i< 3; ++i)
	{
		test[6 + i * 3 + 0] = glm::cross(test[i], test[0]);
		test[6 + i * 3 + 1] = glm::cross(test[i], test[1]);
		test[6 + i * 3 + 2] = glm::cross(test[i], test[2]);
	}

	glm::vec3* hitNormal = 0;
	bool shouldFlip;

	for (int i = 0; i< 15; ++i)
	{
		if (glm::length2(test[i])< 0.001f)
			continue;

		float depth = penetrationDepth(obb1, obb2, test[i], &shouldFlip);

		if (depth <= 0.0f)
			return result;
		
		else if (depth < result.getDepth())
		{
			if (shouldFlip)
				test[i] = test[i] * -1.0f;
			
			result.setDepth(depth);
			hitNormal = &test[i];
		}
	}

	if (hitNormal == 0)
		return result;
	
	glm::vec3 axis = glm::normalize(*hitNormal);
	std::vector<glm::vec3> c1 = clipEdgesToOBB(getEdges(obb2), obb1);
	std::vector<glm::vec3> c2 = clipEdgesToOBB(getEdges(obb1), obb2);
	std::vector<glm::vec3> resultContacts;
	resultContacts.insert(resultContacts.end(), c1.begin(), c1.end());
	resultContacts.insert(resultContacts.end(), c2.begin(), c2.end());

	Interval i = getInterval(obb1, axis);
	float distance = (i.max - i.min)* 0.5f - result.getDepth() * 0.5f;
	glm::vec3 pointOnPlane = obb1.getPosition() + (axis * distance);
	for (int i = resultContacts.size() - 1; i >= 0; --i)
	{
		glm::vec3 contact = resultContacts[i];
		resultContacts[i] = contact + (axis * glm::dot(axis, pointOnPlane - contact));
	}

	for (int i = resultContacts.size() - 1; i >= 0; --i)
	{
		glm::vec3 contact = resultContacts[i];
		resultContacts[i] = contact + (axis *glm::dot(axis, pointOnPlane - contact));
		for (int j = resultContacts.size() - 1; j > i; --j)
		{
			if (glm::length2(resultContacts[j] - resultContacts[i]) < 0.001f)
			{
				resultContacts.erase(resultContacts.begin() + j);
				break;
			}
		}
	}

	result.setHasIntersection(true);
	result.setContacts(resultContacts);
	result.setNormal(axis);

	return result;

}

CollisionManifold Collider::intersect(BoundingSphere sphere1, BoundingSphere sphere2)
{
	CollisionManifold result = CollisionManifold();

	float r = sphere1.getRadius() + sphere2.getRadius();

	glm::vec3 d = sphere2.getCenter() - sphere1.getCenter();
	if (glm::length2(d) - r * r > 0 || glm::length2(d) == 0.0f)
		return result;

	d = glm::normalize(d);

	result.setHasIntersection(true);
	result.setNormal(d);
	result.setDepth(fabsf(glm::length(d) - r) * 0.5f);
	// distance to intersection point
	float dtp = sphere1.getRadius() - result.getDepth();
	glm::vec3 contact = sphere1.getCenter() + d * dtp;
	result.setContact(contact);

	return result;
}

CollisionManifold Collider::intersect(BoundingSphere sphere, OBB obb)
{
	CollisionManifold result;

	glm::vec3 cp = closestPoint(obb, sphere.getCenter());
	
	// if the point is outside the sphere, the sphere and OBB do not intersect.Return false, as shown:
	float distanceSq = glm::length2(cp - sphere.getCenter());
	float radiusSq = sphere.getRadius() * sphere.getRadius();
	if (distanceSq > radiusSq)
	{
		result = CollisionManifold();
		return result;
	}

	glm::vec3 normal;
	if (CMP(distanceSq, 0.0f))
	{
		float mSq = glm::length2(cp - obb.getPosition());
		if (CMP(mSq, 0.0f))
		{
			result = CollisionManifold();
			return result;
		}

		// closest point is at the center of the sphere
		normal = glm::normalize(cp - obb.getPosition());
	}
	else
	{
		normal = glm::normalize(sphere.getCenter() - cp);
	}

	std::vector<glm::vec3> resultContact;
	// fill out the intersection result
	glm::vec3 outp = sphere.getCenter() - normal * sphere.getRadius();
	float dist = glm::length(cp - outp);
	resultContact.push_back(cp + (outp - cp) * 0.5f);

	result.setHasIntersection(true);
	result.setContacts(resultContact);
	result.setNormal(normal);
	result.setDepth(dist * 0.5f);

	return result;
}

CollisionManifold Collider::intersect(BoundingSphere sphere, Plane plane)
{
	glm::vec3 normal = glm::normalize(plane.getNormal());
	float distanceFromSphereCenter = glm::dot(normal, sphere.getCenter()) - plane.getDistance();
	float distanceFromSphere = distanceFromSphereCenter - sphere.getRadius();
	// ...
	auto radius = sphere.getRadius();
	bool sphereCenterInRange = distanceFromSphereCenter > -radius && distanceFromSphereCenter < radius;

	return CollisionManifold(sphereCenterInRange, distanceFromSphere);
}

glm::vec3 Collider::closestPoint(OBB obb, const glm::vec3& point)
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


bool Collider::pointInOBB(glm::vec3 point, OBB obb)
{
	glm::vec3 dir = point - obb.getPosition();
	for (int i = 0; i < 3; ++i) {
		const float* orientation = &(value_ptr(obb.getOrientation()))[i * 3];
		glm::vec3 axis(orientation[0], orientation[1], orientation[2]);

		float distance = glm::dot(dir, axis);

		if (distance >obb.getSize()[i])
			return false;

		if (distance < -obb.getSize()[i])
			return false;
	}

	return true;
}


glm::vec3 Collider::closestPoint(BoundingSphere sphere, const glm::vec3& point)
{
	glm::vec3 sphereToPoint = point - sphere.getCenter();
	sphereToPoint = glm::normalize(sphereToPoint);
	sphereToPoint = sphereToPoint * sphere.getRadius();

	return sphereToPoint + sphere.getCenter();
}


Interval Collider::getInterval(OBB obb, const glm::vec3 axis)
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

std::vector<glm::vec3> Collider::getVertices(OBB obb)
{
	std::vector<glm::vec3> v;
	v.resize(8);

	glm::vec3 C = obb.getPosition(); // OBB Center
	glm::vec3 E = obb.getSize(); // OBB Extents
	const float* o = glm::value_ptr(obb.getOrientation());
	glm::vec3 A[] = { // OBB Axis
		glm::vec3(o[0], o[1], o[2]),
		glm::vec3(o[3], o[4], o[5]),
		glm::vec3(o[6], o[7], o[8]),
	};


	v[0] = C + A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
	v[1] = C - A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
	v[2] = C + A[0] * E[0] - A[1] * E[1] + A[2] * E[2];
	v[3] = C + A[0] * E[0] + A[1] * E[1] - A[2] * E[2];
	v[4] = C - A[0] * E[0] - A[1] * E[1] - A[2] * E[2];
	v[5] = C + A[0] * E[0] - A[1] * E[1] - A[2] * E[2];
	v[6] = C - A[0] * E[0] + A[1] * E[1] - A[2] * E[2];
	v[7] = C - A[0] * E[0] - A[1] * E[1] + A[2] * E[2];

	return v;

}

std::vector<Line> Collider::getEdges(OBB  obb)
{
	std::vector<Line> result;
	result.reserve(12);

	//gets vertices
	auto v = getVertices(obb);

	// Indices of edge-vertices (as pairs)
	int index[][2] =
	{
		{ 6,1 },{ 6,3 },{ 6,4 },{ 2,7 },{ 2,5 },{ 2,0 },
		{ 0,1 },{ 0,3 },{ 7,1 },{ 7,4 },{ 4,5 },{ 5,3 }
	};

	//loop trough index array and construct edges
	for (int j = 0; j < 12; ++j)
		result.push_back(Line(v[index[j][0]], v[index[j][1]]));

	return result;
}

std::vector<Plane> Collider::getPlanes(OBB  obb)
{
	glm::vec3 c = obb.getPosition(); // OBB Center
	glm::vec3 e = obb.getSize(); // OBB Extents
	const float* o = glm::value_ptr(obb.getOrientation());
	// OBB Axis
	glm::vec3 a[] = 
	{
		glm::vec3(o[0], o[1], o[2]),
		glm::vec3(o[3], o[4], o[5]),
		glm::vec3(o[6], o[7], o[8]),
	};

	std::vector<Plane> result;
	result.resize(6);

	//construct a plane for each face using a point  and the normal of the face
	result[0] = Plane(a[0], glm::dot(a[0], (c + a[0] * e.x)));
	result[1] = Plane(a[0] * -1.0f, -glm::dot(a[0], (c - a[0] * e.x)));
	result[2] = Plane(a[1], glm::dot(a[1], (c + a[1] * e.y)));
	result[3] = Plane(a[1] * -1.0f, -glm::dot(a[1], (c - a[1] * e.y)));
	result[4] = Plane(a[2], glm::dot(a[2], (c + a[2] * e.z)));
	result[5] = Plane(a[2] * -1.0f, -glm::dot(a[2], (c - a[2] * e.z)));

	return result;
}

bool Collider::clipToPlane(Plane  plane, const Line & line, glm::vec3 * outPoint)
{
	//Find if it intersects with the plane
	auto normal = glm::normalize(plane.getNormal());
	glm::vec3 ab = line.end - line.start;
	float nAB = glm::dot(normal, ab);

	if (CMP(nAB, 0.0f))
		return	false;

	//	Find where it intersects with the plane
	float	nA = glm::dot(normal, line.start);
	float	t = (plane.getDistance() - nA) / nAB;

	//if intersects return the point of intersection and true
	if (t >= 0.0f && t <= 1.0f) 
	{
		if (outPoint != 0)
			*outPoint = line.start + ab * t;

		return true;
	}

	return	false;
}

std::vector<glm::vec3> Collider::clipEdgesToOBB(const std::vector<Line>& edges, OBB obb)
{
	std::vector<glm::vec3>	result;
	result.reserve(edges.size());
	glm::vec3 intersection;
	std::vector<Plane>&	planes = getPlanes(obb);
	int count = 0;
	for (int i = 0; i < planes.size(); ++i)
	{
		for (int j = 0; j < edges.size(); ++j)
		{
			if (clipToPlane(planes[i], edges[j], &intersection) && pointInOBB(intersection, obb))
				result.push_back(intersection);
		}
	}

	return result;
}

float Collider::penetrationDepth(OBB o1, OBB  o2, const glm::vec3& axis, bool* outShouldFlip)
{
	Interval i1 = getInterval(o1, glm::normalize(axis));
	Interval i2 = getInterval(o2, glm::normalize(axis));

	if (!((i2.min <= i1.max) && (i1.min <= i2.max)))
		return 0.0f; /// no penerattion

	//legth of intervals
	float len1 = i1.max - i1.min;
	float len2 = i2.max - i2.min;

	//max and min point of intervals
	float min = fminf(i1.min, i2.min);
	float max = fmaxf(i1.max, i2.max);

	float length = max - min;

	//flips collision normal if needed
	if (outShouldFlip != 0)
		*outShouldFlip = (i2.min < i1.min);

	return (len1 + len2) - length;
}