#pragma once
// Math constraints
#define _USE_MATH_DEFINES
#include <cmath>
#include "Body.h"
#include "Collider.h"

namespace GPhysix
{
	class RigidBody : public Body
	{
		private:
			float m_density;
			glm::mat3 m_invInertia; ///> inverse inertia
			glm::vec3 m_angVel;		///> angular velocity
			glm::vec3 m_angAcc;		///> angular acceleration
			Collider m_collider;

		public:
			RigidBody() { };
			RigidBody(const Mesh& mesh);
			~RigidBody() { };
			
			void move() override;

			inline void setAngVel(const glm::vec3& omega) { m_angVel = omega; }
			inline void setAngAcc(const glm::vec3& alpha) { m_angAcc = alpha; }
			inline void setInvInertia(const glm::mat3& invInertia) { m_invInertia = invInertia; }
			void setMass(float mass) { m_mass = mass; updateInvInertia(); }
			void scale(glm::vec3 vect) { m_mesh.scale(vect); updateInvInertia(); }
			void setRotate(const glm::mat4& tmat4) { m_mesh.setRotateMat4(tmat4); }

			glm::vec3& getAngVel() { return m_angVel; }
			const glm::vec3& getAngAcc() { return m_angAcc; }
			const glm::mat3& getInvInertia() { return m_invInertia; }

			inline const glm::vec3& getScaleVec() const { return glm::vec3(getScale()[0].x, getScale()[1].y, getScale()[2].z); }
			void updateInvInertia();
			
			Collider getCollider() { return m_collider; }
			void addCollider(TYPE type) { m_collider.setType(type); }

			// COLLISION DETECTION FUNCTION
			IntersectData canCollideStatic();
			IntersectData canCollideDynamic(RigidBody* other);

		// CREATE COLLISION SHAPES FROM THE OBJECT
		private:
			Plane getPlaneCollider();
			AABB getAxisAlignedBoxCollider();
			OBB getOrientedBoxCollider();
			BoundingSphere getSphereCollider();
	};
}