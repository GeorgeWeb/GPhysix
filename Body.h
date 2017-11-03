#pragma once
#include "Mesh.h"
#include "Force.h"

namespace GPhysix
{
	class Body
	{
		protected:
			Mesh m_mesh; ///> mesh used to represent the body
		
			float m_mass; ///> mass
			float m_cor; ///> coefficient of restitution
		
		private:
			glm::vec3 m_acc; ///> acceleration
			glm::vec3 m_vel; ///> velocity
			glm::vec3 m_pos; ///> position

			std::vector<Force*> m_forces;
		public:
			Body() { }
			~Body() { };

			/*
			** GET METHODS
			*/
			// mesh
			Mesh &getMesh() { return m_mesh; }
			// force
			inline const std::vector<Force*> &getForces() const { return m_forces; }
			// transform matrices
			glm::mat4 getTranslate() const { return m_mesh.getTranslate(); }
			glm::mat4 getRotate() const { return m_mesh.getRotate(); }
			glm::mat4 getScale() const { return m_mesh.getScale(); }
			// dynamic variables
			glm::vec3 &getAcc() { return m_acc; }
			glm::vec3 &getVel() { return m_vel; }
			glm::vec3 &getPos() { return m_pos; }
			// physical properties
			const float &getMass() const { return m_mass; }
			const float &getCor() const { return m_cor; }
		
			/*
			** SET METHODS
     		*/
			// mesh
			void setMesh(Mesh m) { m_mesh = m; }
			// force
			void addForce(Force *force);
			const glm::vec3 &applyForces(const glm::vec3 &pos, const glm::vec3 &vel);
			// dynamic variables
			void setAcc(const glm::vec3 &vect) { m_acc = vect; }
			void setVel(const glm::vec3 &vect) { m_vel = vect; }
			void setVel(const int &i, const float &v) { m_vel[i] = v; } ///> set the ith coordinate of the velocity vector
			void setPos(const glm::vec3 &vect) { m_pos = vect; m_mesh.setPos(vect); }
			void setPos(const int &i, const float &p) { m_pos[i] = p; m_mesh.setPos(i, p); } ///> set the ith coordinate the position vector
			
			// physical properties
			void setCor(float cor) { m_cor = cor; }
			void setMass(float mass) { m_mass = mass; }

			// transformation methods
			void translate(const glm::vec3& vect);
			void rotate(const float& angle, const glm::vec3& vect);
			void scale(const glm::vec3& vect);
			
			void integrate(const float deltaTime);
			virtual void move();
	};
}