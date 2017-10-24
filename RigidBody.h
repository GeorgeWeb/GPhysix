#pragma once
// Math constraints
#define _USE_MATH_DEFINES
#include <cmath>
#include "Body.h"

namespace GPhysix
{
	class RigidBody : public Body
	{
		private:
			float m_density;
			glm::mat3 m_invInertia; ///> inverse inertia
			glm::vec3 m_angVel;		///> angular velocity
			glm::vec3 m_angAcc;		///> angular acceleration

		public:
			RigidBody() { };
			RigidBody(const Mesh& mesh);
			~RigidBody() { };

			inline void setAngVel(const glm::vec3& omega) { m_angVel = omega; }
			inline void setAngAcc(const glm::vec3& alpha) { m_angAcc = alpha; }
			inline void setInvInertia(const glm::mat3& invInertia) { m_invInertia = invInertia; }

			const glm::vec3& getAngVel() { return m_angVel; }
			const glm::vec3& getAngAcc() { return m_angAcc; }
			const glm::mat3& getInvInertia() { return m_invInertia; }

			void scale(const glm::vec3& vect);
	};
}