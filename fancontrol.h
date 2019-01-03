#include "pid_controller.h"

/* Temperature limits (model specific?) */
#define DANGEROUS_TEMP		95
#define HOT_TEMP		90
#define DEFAULT_TARGET_TEMP     75
#define MIN_TEMP 		1

/* keep in sync with fancontrol_mode_name */
enum fancontrol_mode {
	FANCTRL_EMERGENCY,
	FANCTRL_AUTO,
	FANCTRL_MANUAL,
};

extern const char *fancontrol_mode_name[];

struct fancontrol {
	int initializing;
	enum fancontrol_mode mode;
	double setpoint_deg;
	int requested_fan_duty;
	int fan_duty;
	double started, last_calc;
	double last_dt, last_temp;
	FILE *log;
	PIDControl pid;
};

void fancontrol_init(struct fancontrol *fc);
int fancontrol_calculate(struct fancontrol *fc, int temp_ok, double temp);
void fancontrol_setmode_auto(struct fancontrol *fc, double setpoint_deg);
void fancontrol_setmode_manual(struct fancontrol *fc, int fan_duty);
void fancontrol_setmode_emergency(struct fancontrol *fc);
