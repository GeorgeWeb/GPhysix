#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "CollisionManifold.h"

namespace GPhysix
{
	typedef struct Interval
	{
		float min;
		float max;
	} Interval;

	class Geometry
	{
		protected:
			Geometry() = default;
	};

	class Line : Geometry
	{
		public:
			Line() : Geometry() {}
			Line(const glm::vec3& s, const glm::vec3& e) : start(s), end(e) {}
			
			glm::vec3 start;
			glm::vec3 end;
	};

	class Plane
	{
		public:
			Plane() : m_normal(glm::vec3(0.0f, 1.0f, 0.0f)), m_distance(0.0f) {}

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