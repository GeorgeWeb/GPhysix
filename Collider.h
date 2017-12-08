#pragma once

#include "IntersectData.h"
#include "Geometry.h"
#include <algorithm>
#include <cmath>
#include <array>

namespace GPhysix
{
	typedef struct SAT
	{
		const glm::vec3& getMin(OBB obb);
		const glm::vec3& getMax(OBB obb);
		Interval getInterval(AABB aabb, const glm::vec3 axis);
		Interval getInterval(OBB obb, const glm::vec3 axis);
		bool overlapOnAxis(AABB aabb, OBB obb, const glm::vec3 axis);
		bool overlapOnAxis(OBB obb1, OBB obb2, const glm::vec3 axis);
	} SAT;

	enum class TYPE
	{
		SPHERE,
		AABB,
		OBB,
		PLANE,
	};

	class Collider
	{
		public:
			Collider() { };
			Collider(const TYPE& type) : m_type(type) {}

			inline void setType(TYPE type) { m_type = type; }
			inline const TYPE& getType() const { return m_type; }

			// AABBs with :
			IntersectData intersect(AABB aabb1, AABB aabb2);
			IntersectData intersect(AABB aabb, OBB obb);
			IntersectData intersect(AABB aabb, Plane plane);
			
			// OBBs with :
			IntersectData intersect(OBB obb1, OBB obb2);
			IntersectData intersect(OBB obb, Plane plane);

			// BoundingSpheres with :
			IntersectData intersect(BoundingSphere sphere1, BoundingSphere sphere2);
			IntersectData intersect(BoundingSphere sphere, AABB aabb);
			IntersectData intersect(BoundingSphere sphere, OBB obb);
			IntersectData intersect(BoundingSphere sphere, Plane plane);

		//private:
			glm::vec3 closestPoint(AABB aabb, const glm::vec3& point);
			glm::vec3 closestPoint(OBB obb, const glm::vec3& point);
			glm::vec3 closestPoint(BoundingSphere sphere, const glm::vec3& point);

		private:
			TYPE m_type;
	};
}