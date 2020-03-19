#ifndef _MOTOR_H
#define _MOTOR_H

#include "Arduino.h"

class FuntechStepper {
	public:
		FuntechStepper(int pin_enable, int pin_direction, int pin_step, int direction);
		void enable(void);
		void disable(void);
		void setDirection(int direction);
		void step(void);
		long getSteps(void);
		void resetSteps(void);

	private:
		int pin_enable;
		int pin_direction;
		int pin_step;
		int direction;
		long _steps;
		int _increment_step;
};

enum { FTS_FORWARD = 0, FTS_BACKWARD = 1 };

#endif // _MOTOR_H
