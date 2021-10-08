/*
 * JoystickToggle.cpp
 *
 *  Created on: Feb 17, 2017
 *      Author: Kevin
 */
#include "JoystickToggle.h"


template <class T>
JoystickToggle<T>::JoystickToggle(Joystick *stick, int button, T *i, std::function<void(T*)> func):
		m_stick(stick),
		m_button(button),
		m_i(i),
        toggle_event(func),
		m_buttonPrevious(false),
		m_buttonCurrent(false)
{

}

template <class T>
void JoystickToggle<T>::Run()
{
	m_buttonCurrent = m_stick->GetRawButton(m_button);
	if(m_buttonCurrent && m_buttonCurrent != m_buttonPrevious)
	{
        toggle_event(m_i);
	}
	m_buttonPrevious = m_buttonCurrent;
	return;
}

template <class T>
JoystickToggle<T>::~JoystickToggle()
{
	delete m_stick;
	delete m_i;

}
