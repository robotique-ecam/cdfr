class PID {
public:
  void set_limits(double min_value, double max_value) {
    _min_output = min_value; _max_output = max_value;
  }

  void set_params(double Ki, double Kp, double Kd) {
    _Ki = Ki; _Kp = Kp; _Kd = Kd;
  }

  double compute(double setpoint, double input) {
    _setpoint = setpoint;
    _error = (_setpoint - input);
    _d_input = (input - _last_input);

    _integral += (_Ki * _error);
    _output = _Kp * _error + _integral -_Kd * _d_input;

    if (_output > _max_output){
			_output = _max_output;
      // Anti Windup
			_integral -= (_Ki * _error);
		}

		else if (_output < _min_output){
			_output = _min_output;
      // Anti Windup
			_integral -= (_Ki * _error);
		}

    _last_input = input;

    return _output;
  }

private:
  double _Ki;
  double _Kp;
  double _Kd;
  double _min_output;
  double _max_output;

  double _error;
  double _d_input;
  double _setpoint;
  double _output = 0;
  double _integral = _output;
  double _last_input;
};
