#include "Window.h"
#include <iostream>

using namespace GPhysix;

Window::Window(const char *name, const unsigned int &width, const unsigned int &height)
	: m_Title(name), m_Width(width), m_Height(height)
{ 
	init();
}

Window::~Window()
{
	// TODO: ...
}

void Window::init()
{
	glfwInit();

	// initialze smart_Window with raw glfwWindow creator as a construction argument
	m_Window.reset(glfwCreateWindow(m_Width, m_Height, m_Title, nullptr, nullptr));
	
	// check if the window is created without any issues
	if (!m_Window && !m_Window.get())
	{
		std::cerr << "Failed to create GLFW window!" << std::endl;
		return;
	}

	// set the current window in use
	glfwMakeContextCurrent(m_Window.get());
	glewInit();
}

const bool &GPhysix::Window::closed() const
{
	return glfwWindowShouldClose(m_Window.get());
}

void Window::update() const
{
	glfwPollEvents();
	glfwSwapBuffers(m_Window.get());
}
