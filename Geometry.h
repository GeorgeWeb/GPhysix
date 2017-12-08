#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "IntersectData.h"

namespace GPhysix
{
	class Geometry
	{
		protected:
			Geometry() = default;
	};

	class Plane
	{
		public:
			Plane(const glm::vec3 normal, float distance) :
				m_normal(normal),
				m_distance(distance)
			{ }

			Plane normalized() { return Plane(m_normal / glm::length(m_normal), m_distance / glm::length(m_normal)); }

			inline const glm::vec3& getNormal() { return m_normal; }
			inline float getDistance() { return m_distance; }

		private:
			glm::vec3 m_normal;
			float m_distance;
	};
	
	class AABB : public Geometry
	{
	public:
		AABB(const glm::vec3& minExtents, const glm::vec3& maxExtents) :
			m_minExtents(minExtents),
			m_maxExtents(maxExtents)
		{ }

		inline const glm::vec3& getMinExtents() { return m_minExtents; }
		inline const glm::vec3& getMaxExtents() { return m_maxExtents; }

		inline const glm::vec3& getSize() { return m_maxExtents - m_minExtents; }
		inline const glm::vec3& getPosition() { return m_minExtents + (0.5f * getSize()); }

	private:
		glm::vec3 m_minExtents;
		glm::vec3 m_maxExtents;
	};

	class OBB : public Geometry
	{
		public:
			OBB(const glm::vec3& position, const glm::mat3& orientation, const glm::vec3& size) :
				m_position(position), m_orientation(orientation), m_size(size)
			{}

			const glm::vec3& getPosition() { return m_position; }
			const glm::mat3& getOrientation() { return m_orientation; }
			const glm::vec3& getSize() { return m_size; }

		private:
			glm::vec3 m_position;
			glm::mat3 m_orientation;
			glm::vec3 m_size;
	};

	class BoundingSphere : public Geometry
	{
		public:
			BoundingSphere(const glm::vec3& center, float radius) :
				m_center(center),
				m_radius(radius)
			{}

			inline const glm::vec3& getCenter() { return m_center; }
			inline float getRadius() { return m_radius; }
		private:
			glm::vec3 m_center;
			float m_radius;
	};
}

#endif !GEOMETRY_H