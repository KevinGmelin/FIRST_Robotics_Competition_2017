/*
 * JoystickToggle.h
 *
 *  Created on: Feb 17, 2017
 *      Author: Kevin
 */

#ifndef SRC_JOYSTICKTOGGLE_H_
#define SRC_JOYSTICKTOGGLE_H_

#include "WPILib.h"
#include <functional>

template <class T>
class JoystickToggle {
	Joystick *m_stick;
	int m_button;
	T *m_i;
	std::function<void(T*)> toggle_event;

	bool m_buttonPrevious;
	bool m_buttonCurrent;

public:
	JoystickToggle(Joystick *stick, int button, T *i, std::function<void(T*)> func);

	void Run();
	~JoystickToggle();

};

template class JoystickToggle<int>;
template class JoystickToggle<bool>;
template class JoystickToggle<double>;

#endif /* SRC_JOYSTICKTOGGLE_H_ */
