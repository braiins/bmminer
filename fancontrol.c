#include "config.h"

#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "compat.h"
#include "miner.h"
#include "util.h"

#include "fancontrol.h"

/* PID constants */
#define PID_KP 5
#define PID_KI 0.03
#define PID_KD 0.015

#define FAN_DUTY_MAX 		100
/* do not go lower than 60% duty cycle during warmup */
#define FAN_DUTY_MIN_WARMUP 	60
#define FAN_DUTY_MIN 		10
/* at this fan duty the temperature should be stable at some */
/* sensible (non-dangerous) value */
#define FAN_MIDPOINT		70
/* the fan is allowed to fall only at this rate (PWM% per second) */
/* this smoothes the settling curve considerably */
#define PWM_FALL_RATE_SEC	0.125

#define WARMUP_PERIOD_SEC	(60*2)


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
	int too_hot = 0;

	fanlog(fc, "input: temp_ok=%d temp=%.2lf mode=%d init=%d setpoint=%.2lf req_fan_duty=%d",
		temp_ok, temp,
		fc->mode, fc->initializing,
		fc->setpoint_deg, fc->requested_fan_duty);

	/* check if temperature was measured ok */
	if (temp_ok && temp >= MIN_TEMP) {
		/* we are past initialization */
		if (fc->initializing) {
			fc->initializing = 0;
			dt = 1;
		}
		/* is temperature dangerous? (safety valve) */
		if (temp >= DANGEROUS_TEMP) {
			fanlog(fc, "temperature dangerous, shutting down");
			fprintf(stderr, "\n\nTemperature DANGEROUS, Shutting Down\n\n");
			exit(1);
		}
		if (temp >= HOT_TEMP) {
			fanlog(fc, "temperature very hot, turning on fans");
			fc->fan_duty = FAN_DUTY_MAX;
			too_hot = 1;
		}
	} else {
		/* temperature was not measured */
		if (fc->initializing) {
			/* not _yet_ measured? */
			fanlog(fc, "no temperature yet");
		} else {
			/* assume it's too hot */
			too_hot = 1;
			fanlog(fc, "temperature not measured");
		}
	}


	/* calculate fan_duty for given mode */
	double fan_duty = FAN_DUTY_MAX;
	if (too_hot) {
		/* full power to fans */
		fan_duty = FAN_DUTY_MAX;
	} else if (fc->mode == FANCTRL_AUTO) {
		if (fc->initializing) {
			/* full power to fans */
			fan_duty = FAN_DUTY_MAX;
		} else {
			/* keep fan running during warmup period */
			int min_duty = FAN_DUTY_MIN;
			if (runtime < WARMUP_PERIOD_SEC)
				min_duty = FAN_DUTY_MIN_WARMUP;
			/* set PID limits */
			PIDOutputLimitsSet(&fc->pid, min_duty, FAN_DUTY_MAX);
			/* feed temperature to PID */
			PIDInputSet(&fc->pid, temp);
			PIDCompute(&fc->pid, dt);
			fan_duty = PIDOutputGet(&fc->pid);
		}
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

	fanlog(fc, "output: fan_duty=%d dt=%.2lf mode=%d",
		fc->fan_duty, dt, fc->mode);

	return fan_duty;
}

void
fancontrol_init(struct fancontrol *fc)
{
	fc->log = fopen("/tmp/fancontrol.log", "w");
	fanlog(fc, "PID initializing");

	fc->initializing = 1;
	fc->started = cgtime_float();
	fc->last_calc = fc->started;
	fc->requested_fan_duty = fc->fan_duty = FAN_DUTY_MAX;

	PIDInit(&fc->pid, PID_KP, PID_KI, PID_KD, FAN_DUTY_MIN_WARMUP, FAN_DUTY_MAX, FAN_MIDPOINT, AUTOMATIC, REVERSE);
	fancontrol_setmode_auto(fc, DEFAULT_TARGET_TEMP);
}
