#pragma once
#include "Force.h"
#include <utility>

namespace GPhysix
{
	class Hook : public Force
	{
		private:
			float m_ks;		///> spring stiffness
			float m_kd;		///> damping coefficient
			float m_rest;	///> spring rest length

			std::pair<Body*, Body*> m_bodies;

		public:
			Hook();
			Hook(const float &ks, const float &kd, const float &rest);

			const std::pair<Body*, Body*> &getBodies() const { return m_bodies; }
			
			inline void setBodies(std::pair<Body*, Body*> bodies) { m_bodies = bodies; }

			const glm::vec3 &apply(const float &mass, const glm::vec3 &pos, const glm::vec3 &vel) override;
	};
}