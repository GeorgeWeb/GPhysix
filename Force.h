#pragma once
#include "../Common.h"
#include <iostream>
#include <array>

namespace GPhysix
{
	class Body;

	class Force
	{
	public:
		Force() {}
		virtual ~Force() {}

		virtual const glm::vec3 &apply(float mass, const glm::vec3 &pos, const glm::vec3 &vel) = 0;
	};

	class Gravity : public Force
	{
	private:
		glm::vec3 m_gravity = glm::vec3(0.0f, -9.8f, 0.0f);

	public:
		Gravity();
		Gravity(const glm::vec3 &gravity);
		virtual ~Gravity() {}

		inline const glm::vec3 &getGravity() const { return m_gravity; }
		inline void setGravity(glm::vec3 gravity) { m_gravity = gravity; }

		inline const glm::vec3 &apply(float mass, const glm::vec3 &pos, const glm::vec3 &vel) override;
	};

	class Drag : public Force
	{
	private:
		glm::vec3 m_drag;

	public:
		Drag();
		Drag(const glm::vec3 &drag);
		virtual ~Drag() {}

		inline const glm::vec3 &getDrag() const { return m_drag; }
		inline void setDrag(const glm::vec3 &drag) { m_drag = drag; }

		inline const glm::vec3 &apply(float mass, const glm::vec3 &pos, const glm::vec3 &vel);
	};

	class Hook : public Force
	{
	private:
		float m_ks;		///> spring stiffness
		float m_kd;		///> damping coefficient
		float m_rest;	///> spring rest length

		std::pair<Body*, Body*> m_bodies;

	public:
		Hook();
		Hook(const float ks, const float kd, const float rest);
		virtual ~Hook();

		const std::pair<Body*, Body*> &getBodies() const { return m_bodies; }
		inline void setBodies(std::pair<Body*, Body*> bodies) { m_bodies = bodies; }

		const glm::vec3 &apply(const float mass, const glm::vec3 &pos, const glm::vec3 &vel) override;
	};

	class Wind : public Force
	{
		private:
			glm::vec3 m_wind;
		public:
			Wind();
			Wind(const glm::vec3 &wind);
			virtual ~Wind() {}

			glm::vec3 getWind() const { return m_wind; }
			void setWind(glm::vec3 wind) { m_wind = wind; }

			inline const glm::vec3 &apply(float mass, const glm::vec3 &pos, const glm::vec3 &vel) override;
	};

	class Particle;

	class Aero : public Force
	{
	private:
		float m_km; ///> medium coefficient
		float m_kd;	///> drag coefficient

		std::array<Particle*, 3> m_Triangle;
		Wind *m_wind;
	public:
		Aero();
		Aero(const float km, const float kd);
		virtual ~Aero();

		inline const std::array<Particle*, 3> getTriangle() const { return m_Triangle; }
		
		inline void setAero(const float km, const float kd) { m_km = km; m_kd = kd; }
		inline void setTriangle(Particle* part1, Particle *part2, Particle *part3) { m_Triangle = { part1, part2, part3 }; }

		inline void addWind(Wind *wind) { m_wind = wind; }

		inline const glm::vec3 &apply(float mass, const glm::vec3 &pos, const glm::vec3 &vel);
	};
}