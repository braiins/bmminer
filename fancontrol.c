#include "config.h"

#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "compat.h"
#include "miner.h"
#include "util.h"

#include "fancontrol.h"

/* PID constants */
#define PID_KP 20
#define PID_KI 0.05
#define PID_KD 0.1

/* Temperature limits (model specific?) */
#define DANGEROUS_TEMP		110
#define HOT_TEMP		95
#define DEFAULT_TARGET_TEMP     75

#define FAN_DUTY_MAX 		100
/* do not go lower than 60% duty cycle during warmup */
#define FAN_DUTY_MIN_WARMUP 	60
#define FAN_DUTY_MIN 		10

#define WARMUP_PERIOD_SEC	(6*60)


static double
cgtime_float(void)
{
	struct timeval tv;
	cgtime(&tv);
	return tv.tv_sec + (double)tv.tv_usec / 1e6;
}


static void
fanlog(struct fancontrol *fc, const char *fmt, ...)
{
	va_list ap;
	char buf[128];
	struct timeval tv;
	struct tm tm;

	if (fc->log == 0)
		return;

	cgtime(&tv);
	localtime_r(&tv.tv_sec, &tm);
	strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm);
	fprintf(fc->log, "%s ", buf);

	va_start(ap, fmt);
	vfprintf(fc->log, fmt, ap);
	va_end(ap);

	fprintf(fc->log, "\n");
	fflush(fc->log);
}

void
fancontrol_setmode_auto(struct fancontrol *fc, double setpoint_deg)
{
	fanlog(fc, "setmode(auto): target %lf", setpoint_deg);
	fc->mode = FANCTRL_AUTO;
	fc->setpoint_deg = setpoint_deg;
	PIDSetpointSet(&fc->pid, setpoint_deg);
}

void
fancontrol_setmode_manual(struct fancontrol *fc, int fan_duty)
{
	fanlog(fc, "setmode(manual): fan_duty=%d", fan_duty);
	fc->mode = FANCTRL_MANUAL;
	fc->requested_fan_duty = fan_duty;
}

void
fancontrol_setmode_emergency(struct fancontrol *fc)
{
	fanlog(fc, "setmode(emergency)");
	fc->mode = FANCTRL_EMERGENCY;
}

int
fancontrol_calculate(struct fancontrol *fc, int temp_ok, double temp)
{
	double now = cgtime_float();	
	double dt = now - fc->last_calc;
	double runtime = now - fc->started;

	fanlog(fc, "input: temp_ok=%d temp=%.2lf mode=%d init=%d setpoint=%.2lf fan_duty=%d",
		temp_ok, temp,
		fc->mode, fc->initializing,
		fc->setpoint_deg, fc->requested_fan_duty);

	/* check if temperature was measured ok */
	if (temp_ok) {
		/* we are past initialization */
		if (fc->initializing) {
			fc->initializing = 0;
		}
		/* is temperature meaningful? */
		if (temp <= 0.01) {
			/* maybe no temperature sensors? */
			fc->mode = FANCTRL_EMERGENCY;
			fanlog(fc, "temperature zero");
		}
		/* is temperature dangerous? (safety valve) */
		if (temp >= DANGEROUS_TEMP) {
			fc->mode = FANCTRL_EMERGENCY;
			fanlog(fc, "temperature too hot");
		}
	} else {
		/* temperature was not measured */
		if (fc->initializing) {
			/* you mean - not _yet_ measured? */
		} else {
			/* fuck */
			fc->mode = FANCTRL_EMERGENCY;
			fanlog(fc, "temperature not measured");
		}
	}

	/* only allow to lower fan duty by 1% a second */
	int new_min_duty = fc->fan_duty - ceil(dt * 1);
	new_min_duty = MAX(FAN_DUTY_MIN, new_min_duty);
	/* are we still warming up? */
	if (runtime < WARMUP_PERIOD_SEC) {
		/* do not run fans too low when warming up */
		new_min_duty = MAX(new_min_duty, FAN_DUTY_MIN_WARMUP);
	}
	/* set PID limits */
	PIDOutputLimitsSet(&fc->pid, new_min_duty, FAN_DUTY_MAX);

	/* calculate fan_duty for given mode */
	double fan_duty = FAN_DUTY_MAX;
	if (fc->initializing) {
		fan_duty = FAN_DUTY_MAX;
	} else if (fc->mode == FANCTRL_AUTO) {
		/* feed temperature to PID */
		PIDInputSet(&fc->pid, temp);
		PIDCompute(&fc->pid, dt);
		fan_duty = PIDOutputGet(&fc->pid);
	} else if (fc->mode == FANCTRL_MANUAL) {
		/* output requested fan speed */
		fan_duty = fc->requested_fan_duty;
	} else {
		/* emergency or other: run fans on full */
		fan_duty = FAN_DUTY_MAX;
	}
	/* remember what was set */
	fc->last_calc = now;
	fc->fan_duty = fan_duty;

	fanlog(fc, "output: fan_duty=%d dt=%.2lf mode=%d min_duty=%d",
		fc->fan_duty, dt, fc->mode, new_min_duty);

	return fan_duty;
}

int
fancontrol_init(struct fancontrol *fc)
{
	fc->log = fopen("/tmp/fancontrol.log", "w");
	fanlog(fc, "PID initializing");

	fc->initializing = 1;
	fc->started = cgtime_float();
	fc->last_calc = fc->started;
	fc->requested_fan_duty = fc->fan_duty = FAN_DUTY_MAX;

	PIDInit(&fc->pid, PID_KP, PID_KI, PID_KD, FAN_DUTY_MIN_WARMUP, FAN_DUTY_MAX, AUTOMATIC, REVERSE);
	fancontrol_setmode_auto(fc, DEFAULT_TARGET_TEMP);
}
