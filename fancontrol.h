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


void fancontrol_init(struct fancontrol *fc);
int fancontrol_calculate(struct fancontrol *fc, int temp_ok, double temp);
void fancontrol_setmode_auto(struct fancontrol *fc, double setpoint_deg);
void fancontrol_setmode_manual(struct fancontrol *fc, int fan_duty);
void fancontrol_setmode_emergency(struct fancontrol *fc);
