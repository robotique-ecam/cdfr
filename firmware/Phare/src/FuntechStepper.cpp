#include "FuntechStepper.h"

FuntechStepper::FuntechStepper(int pin_enable, int pin_direction, int pin_step, int direction) {
	this->pin_enable = pin_enable;
	this->pin_direction = pin_direction;
	this->pin_step = pin_step;
	this->direction = direction;

	pinMode(this->pin_enable, OUTPUT);
	pinMode(this->pin_direction, OUTPUT);
	pinMode(this->pin_step, OUTPUT);

	digitalWrite(this->pin_enable, HIGH);
	digitalWrite(this->pin_direction, direction);
	digitalWrite(this->pin_step, LOW);

	this->disable();
	this->setDirection(this->direction);
	this->resetSteps();
}

void FuntechStepper::enable(void) {
	digitalWrite(this->pin_enable, LOW);
}

void FuntechStepper::disable(void) {
	digitalWrite(this->pin_enable, HIGH);
}

void FuntechStepper::setDirection(int direction) {
	digitalWrite(this->pin_direction, this->direction ^ direction);
	this->_increment_step = -2 * direction + 1;
	//this->resetSteps();
}

void FuntechStepper::step(void) {
	digitalWrite(this->pin_step, HIGH);
	digitalWrite(this->pin_step, LOW);
	this->_steps += this->_increment_step;
}

long FuntechStepper::getSteps(void) {
	return this->_steps;
}

void FuntechStepper::resetSteps(void) {
	this->_steps = 0;
}
