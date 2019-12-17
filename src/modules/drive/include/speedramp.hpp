#include <math.h>


class Speedramp {
public:
  void set_speed_limits(double min_value, double max_value) {
    _min_speed = min_value; _max_speed = max_value;
  }

  void set_delay(double delay) {
    _dt = delay;
    _step = _acceleration * _dt;
  }

  void set_acceleration(double acceleration_rate) {
    _acceleration = acceleration_rate;
    _step = _acceleration * _dt;
  }

  double compute(double setpoint) {
    /* Compute _step only if _setpoint changed */
    if (setpoint != _setpoint) {
      _setpoint = setpoint;
      _step = _acceleration * _dt;
    }

    /* Check if error is acceptable */
    if (abs(_previous - _setpoint) <= _step) {
      return _setpoint;
    }

    /* Compute ramped speed command */
    if ((_setpoint - _previous) > 0) {
      /* We have positive error */
      _previous = setpoint + _step;
    } else {
      /* We have negative error */
      _previous = setpoint - _step;
    }
    return _previous;
  }

private:
  double _dt = 0;
  double _step = 0;
  double _min_speed = 0;
  double _max_speed = 0;
  double _setpoint = 0;
  double _previous = 0;
  double _acceleration = 0; // g/s^2
};
