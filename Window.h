#ifndef WINDOW_H
#define WINDOW_H
#ifdef PRAGMA_ONCE
#pragma once
#endif // !PRAGMA_ONCE

#include "Common.h"
#include <memory>

namespace GPhysix
{
	struct GLFWwindowDestroyer
	{
		void operator()(GLFWwindow* windowPtr)
		{
			glfwDestroyWindow(windowPtr);
		}
	};

	using smartWindow = std::unique_ptr<GLFWwindow, GLFWwindowDestroyer>;

	class Window
	{
		private:
			const char *m_Title;
			unsigned int m_Width, m_Height;
			smartWindow m_Window;
		public:
			Window(const char *name, const unsigned int &width, const unsigned int &height);
			~Window();
			const bool &closed() const;
			void update() const;
		private:
			void init();
	};
}
#endif // !WINDOW_H