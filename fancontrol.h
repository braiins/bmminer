#include "pid_controller.h"
#include "temp-def.h"

/* keep in sync with fancontrol_mode_name */
enum fancontrol_mode {
	FANCTRL_EMERGENCY,
	FANCTRL_AUTO,
	FANCTRL_MANUAL,
};

#define FANCTRL_MAX_LOG_AGE (24*3600)

extern const char *fancontrol_mode_name[];

struct fancontrol {
	int initializing;
	enum fancontrol_mode mode;
	double setpoint_deg;
	int requested_fan_duty;
	int fan_duty;
	double started, last_calc;
	double last_dt, last_temp;
	double log_started;
	FILE *log;
	PIDControl pid;
};

void fancontrol_init(struct fancontrol *fc);
int fancontrol_calculate(struct fancontrol *fc, int temp_ok, double temp);
void fancontrol_setmode_auto(struct fancontrol *fc, double setpoint_deg);
void fancontrol_setmode_manual(struct fancontrol *fc, int fan_duty);
void fancontrol_setmode_emergency(struct fancontrol *fc);
