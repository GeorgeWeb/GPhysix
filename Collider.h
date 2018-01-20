#ifndef COLLIDER_H
#define COLLIDER_H

#include "CollisionManifold.h"
#include "Geometry.h"
#include <algorithm>
#include <cmath>
#include <array>

namespace GPhysix
{
	enum class TYPE
	{
		SPHERE,
		OBB
	};

	class Collider
	{
		public:
			Collider() { };
			Collider(const TYPE& type) : m_type(type) {}

			static inline Plane getPlaneCollider() { return Plane(); }

			inline void setType(TYPE type) { m_type = type; }
			inline const TYPE& getType() const { return m_type; }

			// OBBs with :
			CollisionManifold intersect(OBB obb1, OBB obb2);
			CollisionManifold intersect(OBB obb, Plane plane);

			// BoundingSpheres with :
			CollisionManifold intersect(BoundingSphere sphere1, BoundingSphere sphere2);
			CollisionManifold intersect(BoundingSphere sphere, OBB obb);
			CollisionManifold intersect(BoundingSphere sphere, Plane plane);

		private:
			TYPE m_type;

			/*
			** SAT helper functions:
			*/
			Interval getInterval(OBB obb, const glm::vec3 axis);
			bool pointInOBB(glm::vec3 point, OBB obb);
			glm::vec3 closestPoint(OBB obb, const glm::vec3& point);
			glm::vec3 closestPoint(BoundingSphere sphere, const glm::vec3& point);

			std::vector<glm::vec3> getVertices(OBB obb);
			std::vector<Line> getEdges(OBB obb);
			std::vector<Plane> getPlanes(OBB obb);
			bool clipToPlane(Plane plane, const Line& line, glm::vec3* outPoint);
			std::vector<glm::vec3> clipEdgesToOBB(const std::vector<Line>& edges, OBB obb);

			float penetrationDepth(OBB obb1, OBB obb2, const glm::vec3& axis, bool* outShouldFlip);
	};
}

#endif !COLLIDER_H