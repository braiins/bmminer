#include "config.h"

#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "compat.h"
#include "miner.h"
#include "util.h"

#include "sensors.h"
#include "driver-btm-c5.h"

#define CHIP_ID_TO_ADDR(x) (((x) - 1) * 4)
#define I2C_SCAN_LOG_NAME "/tmp/i2c_scan.log"

static FILE *i2c_scan_log;

/*
 * Fake chip temperature from PCB temperature - the difference is about 15
 * degrees and the temperature takes longer to propagate, so the PID
 * controller oscillates a bit.
 */
static float
local_to_remote(float int_temp)
{
	return int_temp + 15;
}

/*
 * Sensor chip implementation
 */

enum {
	TMP451_REG_R_LOCAL_T	= 0x00,
	TMP451_REG_R_REMOTE_T	= 0x01,
	TMP451_REG_R_CONFIG	= 0x03,
	TMP451_REG_W_CONFIG	= 0x09,
	  TMP451_CONFIG_RANGE	= 0x04,
	TMP451_REG_RW_OFFSET	= 0x11,
	TMP451_REG_R_REMOTE_FRAC = 0x10,
	TMP451_REG_R_LOCAL_FRAC	= 0x15,
};

static int
tmp451_init(struct sensor *sensor)
{
	int ret;

	/* set extended mode */
	ret = i2c_write2(&sensor->dev, TMP451_REG_W_CONFIG, TMP451_CONFIG_RANGE, TMP451_REG_R_CONFIG);
	if (ret < 0)
		return ret;

	/* zero offset */
	i2c_write(&sensor->dev, TMP451_REG_RW_OFFSET, 0);
	if (ret < 0)
		return ret;

	return 0;
}

/* this is for sensor configured in _extended_ mode:
 * temperature 0..255 with offset 0x40 (zero is 64)
 */
static inline float
tmp451_make_temp(uint8_t whole, uint8_t fract)
{
	return (int)whole - 0x40 + fract / 256.0;
}

static int
tmp451_read_temp(struct sensor *sensor, struct temp *temp)
{
	int ret;
	uint8_t local, remote;
	uint8_t local_fract, remote_fract;

	/* read temperature registers */
	ret = i2c_read(&sensor->dev, TMP451_REG_R_LOCAL_T, &local);
	if (ret < 0)
		return ret;

	ret = i2c_read(&sensor->dev, TMP451_REG_R_REMOTE_T, &remote);
	if (ret < 0)
		return ret;

	local_fract = 0;
	remote_fract = 0;

	/* if this sensor provides meaningful fractional part, read it */
	if (sensor->ops->has_fract) {
		ret = i2c_read(&sensor->dev, TMP451_REG_R_REMOTE_FRAC, &remote_fract);
		if (ret < 0)
			return ret;

		ret = i2c_read(&sensor->dev, TMP451_REG_R_LOCAL_FRAC, &local_fract);
		if (ret < 0)
			return ret;
	}

	/* put temperatures together */
	temp->local = tmp451_make_temp(local, local_fract);

	/* broken-off remote sensor? */
	if (remote == 0) {
		/* from tmp451 documentation:
		 *
		 * SENSOR FAULT
		 *
		 * The TMP451 can sense a fault at the D+ input resulting from
		 * incorrect diode connection. The TMP451 can also sense an
		 * open circuit. Short-circuit conditions return a value
		 * of –64°C.
		 */
		applog(LOG_NOTICE, "chain %d has no remote temperature, fixing", sensor->dev.chain);
		temp->remote = local_to_remote(temp->local);
	} else {
		temp->remote = tmp451_make_temp(remote, remote_fract);
	}

	return 0;
}

static int
nct218_read_temp(struct sensor *sensor, struct temp *temp)
{
	int ret;
	uint8_t local;

	/* read local temperature */
	ret = i2c_read(&sensor->dev, 0x00, &local);
	if (ret < 0)
		return ret;

	/* put temperatures together */
	temp->local = tmp451_make_temp(local, 0);

	/* fake remote temperature */
	temp->remote = local_to_remote(temp->local);

	return 0;
}

static struct sensor_ops tmp451_chip = {
	.name = "TMP451",
	.manufacturer_id = 0x55,
	.init = tmp451_init,
	.read_temp = tmp451_read_temp,
	.has_fract = 0,
};

static struct sensor_ops adt7461_chip = {
	.name = "ADT7461",
	.manufacturer_id = 0x41,
	.init = tmp451_init,
	.read_temp = tmp451_read_temp,
	.has_fract = 0,
};


static struct sensor_ops nct218_chip = {
	.name = "NCT218",
	.manufacturer_id = 0x1a,
	.init = tmp451_init,
	.read_temp = nct218_read_temp,
	.has_fract = 0,
};

/*
 * Sensor probing and management
 */

/*
 * checks if byte is some sort of 0xff, 0x7f, 0x3f...
 * right-aligned string of zeros
 */
static int
is_i2c_garbage_byte(uint8_t b)
{
	return (b & (b + 1)) == 0;
}

static int
probe_sensor_addr(struct sensor *sensor)
{
	int ret;
	uint8_t man_id;

	ret = i2c_read(&sensor->dev, 0xfe, &man_id);
	if (ret < 0)
		return ret;

	if (man_id == 0x55) {
		/* TMP451 */
		sensor->ops = &tmp451_chip;
		return 0;
	} else if (man_id == 0x41) {
		/* ADT7461 */
		sensor->ops = &adt7461_chip;
		return 0;
	} else if (man_id == 0x1a) {
		/* NCT218 */
		sensor->ops = &nct218_chip;
		return 0;
	} else {
		/* not found */
		if (!is_i2c_garbage_byte(man_id)) {
			applog(LOG_NOTICE, "there's probably unsupported sensor at chain=%d, i2c_addr=%02x with man_id=%02x",
				sensor->dev.chain, sensor->dev.i2c_addr, man_id);
		}
		return -1;
	}
}

static void
dump_i2c_device(FILE *fw, struct i2c_dev *dev)
{
	uint8_t reg = 0xfe;

	fprintf(fw, "chain %d: found device on chip_addr=%02x, i2c_addr=%02x\n",
		dev->chain, dev->chip_addr, dev->i2c_addr);

	fprintf(fw, "regs from %02x:", reg);
	for (int i = 0; i < 32; i++) {
		uint8_t data;
		int ret;
		ret = i2c_read(dev, reg, &data);
		if (ret < 0)
			fprintf(fw, " XX");
		else
			fprintf(fw, " %02x", data);
		reg++;
	}
	fprintf(fw, "\n");
}

static void
scan_i2c_sensors(int chain, int bus)
{
	struct i2c_dev dev;

	/* open log file */
	if (i2c_scan_log == NULL) {
		i2c_scan_log = fopen(I2C_SCAN_LOG_NAME, "w");
		if (i2c_scan_log == NULL) {
			applog(LOG_ERR, "cannot open log file %s", I2C_SCAN_LOG_NAME);
			return;
		}
	}

	/* notify user this will take long */
	applog(LOG_NOTICE, "chain %d: running i2c scan, this may take up to 30 minutes per chain", chain);
	applog(LOG_NOTICE, "(if you don't want to do this, use --no-sensor-scan parameter)");

	/* make device for this sensor */
	for (int chip_id = CHAIN_ASIC_NUM; chip_id > 0; chip_id--) {
		fprintf(i2c_scan_log, "chain %d: scanning chip %d\n", chain, chip_id);
		for (int i2c_addr = 8*2; i2c_addr < 124*2; i2c_addr += 2) {
			int ret;
			uint8_t man_id;

			/* make device */
			i2c_makedev(&dev, chain, bus, CHIP_ID_TO_ADDR(chip_id), i2c_addr);

			ret = i2c_start_dev(&dev);
			if (ret < 0)
				continue;

			ret = i2c_read(&dev, 0xfe, &man_id);
			if (ret < 0)
				continue;
			if (is_i2c_garbage_byte(man_id))
				continue;

			applog(LOG_NOTICE, "chain %d: found device man_id=%02x on chip=%d, i2c_addr=%02x",
				chain, man_id, chip_id, i2c_addr);
			/* dump it */
			dump_i2c_device(i2c_scan_log, &dev);
		}
		fflush(i2c_scan_log);
	}
}

static int probe_chip_addrs[] = {
	CHIP_ID_TO_ADDR(62),
};
static int probe_i2c_addrs[] = {
	0x98,
	0x9a,
	0x9c,
};

int
probe_sensors(int chain, int bus, struct sensor *sensors, int max_sensors)
{
	int ret;
	int n = 0;

#if 0
	applog(LOG_NOTICE, "probing sensors: chain=%d max_sensors=%d",
		chain, max_sensors);
#endif
	for (int i = 0; i < ARRAY_SIZE(probe_chip_addrs); i++) {
		for (int j = 0; j < ARRAY_SIZE(probe_i2c_addrs); j++) {
			struct sensor *sensor = &sensors[n];

			/* make device for this sensor */
			i2c_makedev(&sensor->dev, chain, bus, probe_chip_addrs[i], probe_i2c_addrs[j]);

			/* try to start i2c bus for this device */
			ret = i2c_start_dev(&sensor->dev);
			if (ret < 0)
				continue;

			/* try to probe it */
			ret = probe_sensor_addr(sensor);
			if (ret < 0)
				continue;

			/* we found one */
			applog(LOG_NOTICE, "chain %d: found sensor %s at chip_addr=%02x, i2c_addr=%02x",
				chain, sensor->ops->name,
				sensor->dev.chip_addr,
				sensor->dev.i2c_addr);

			/* ok, this sensor has been probed */
			n++;
			if (n >= max_sensors)
				goto done;
		}
	}
	if (n == 0) {
		applog(LOG_WARNING, "chain %d: no sensors found!", chain);
		/* beware the double negative */
		if (!opt_no_sensor_scan)
			scan_i2c_sensors(chain, bus);
	}
done:
	return n;
}

int
sensor_read_temp(struct sensor *sensor, struct temp *temp)
{
	int ret;
	ret = i2c_start_dev(&sensor->dev);
	if (ret < 0)
		return ret;
	return sensor->ops->read_temp(sensor, temp);
#if 0
	if (ret < 0) {
		applog(LOG_NOTICE, "chain %d: failed reading temperature",
			sensor->dev.chain);
	} else {
		applog(LOG_NOTICE, "chain %d: read temp local=%f remote=%f",
			sensor->dev.chain, temp->local, temp->remote);
	}
	return ret;
#endif
}
