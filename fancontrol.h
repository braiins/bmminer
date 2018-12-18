#include "pid_controller.h"

enum fancontrol_mode {
	FANCTRL_MANUAL,
	FANCTRL_AUTO,
	FANCTRL_EMERGENCY,
};

struct fancontrol {
	int initializing;
	enum fancontrol_mode mode;
	double setpoint_deg;
	int requested_fan_duty;	
	int fan_duty;
	double started, last_calc;
	FILE *log;
	PIDControl pid;
};
