#ifndef COLLISION_MANIFOLD_H
#define COLLISION_MANIFOLD_H

#include "Common.h"
#include <vector>

namespace GPhysix
{
	// TODO: Remove what is temp.

	class CollisionManifold
	{
		public:
			// def. non-valued construct / used if no collision...
			CollisionManifold() :
				m_hasIntersection(false),
				m_distance(0.0f),
				m_normal(glm::vec3(0.0f, 0.0f, 1.0f)),
				m_depth(FLT_MAX),
				m_contacts(std::vector<glm::vec3>())
			{}
			
			// constructor used for simple detections with minimal information
			CollisionManifold(const bool hasIntersection, const float distance) :
				m_hasIntersection(hasIntersection),
				m_distance(distance) 
			{}

			// the manifold valued construct. to be used for collision response.
			CollisionManifold(const bool hasIntersection, glm::vec3 normal, const float depth, const std::vector<glm::vec3>& contacts) :
				m_hasIntersection(hasIntersection),
				m_normal(normal),
				m_depth(depth),
				m_contacts(contacts)
			{}
			
			// getters
			inline const bool getHasIntersection() const { return m_hasIntersection; }
			inline const float getDistance() const { return m_distance; }
			inline const glm::vec3 getNormal() const { return m_normal; }
			inline const float getDepth() const { return m_depth; }
			inline const std::vector<glm::vec3> getContacts() const { return m_contacts; }
			
			// setters
			inline void setHasIntersection(bool hasIntersection) { m_hasIntersection = hasIntersection; }
			inline void setDistance(const float distance) { m_distance = distance; }
			inline void setNormal(const glm::vec3& normal) { m_normal = normal; }
			inline void setDepth(const float depth) { m_depth = depth; }
			
			inline void setContacts(std::vector<glm::vec3> contacts) { m_contacts = contacts; }
			inline void setContact(const glm::vec3& contact) { m_contacts.push_back(contact); }
			inline void updateContact(const glm::vec3& contact, const int i) { m_contacts[i] = contact; }
			inline void removeContact(const int i) { m_contacts.erase(m_contacts.begin() + i); }

		private:
			bool m_hasIntersection;
			float m_distance;
			glm::vec3 m_normal;
			float m_depth;
			std::vector<glm::vec3> m_contacts;
	};
}

#endif