/*
 * Fake I2C driver
 */

struct i2c_dev {
	int chain, bus, chip_addr;
	int i2c_addr;
};

int i2c_read(struct i2c_dev *dev, uint8_t reg, uint8_t *data);
int i2c_write(struct i2c_dev *dev, uint8_t reg, uint8_t data);
int i2c_start_dev(struct i2c_dev *dev);
int i2c_write2(struct i2c_dev *dev, uint8_t reg, uint8_t data, uint8_t reg_read);

static inline void i2c_makedev(struct i2c_dev *dev, int chain, int bus, int chip_addr, int i2c_addr)
{
	dev->chain = chain;
	dev->bus = bus;
	dev->chip_addr = chip_addr;
	dev->i2c_addr = i2c_addr;
}

/*
 * Sensors and temperatures
 */

struct temp {
	float local, remote;
};

struct sensor;
struct sensor_ops {
	const char *name;
	uint8_t manufacturer_id;
	int (*init)(struct sensor *sensor);
	int (*read_temp)(struct sensor *sensor, struct temp *temp);
	int has_fract;
};

struct sensor {
	struct i2c_dev dev;
	struct sensor_ops *ops;
};

int probe_sensors(int chain, int bus, struct sensor *sensors, int max_sensors);

#define sensor_init(sens) (sens)->ops->init(sens)
int sensor_read_temp(struct sensor *sensor, struct temp *temp);

#define ZERO_TEMP {.local = 0, .remote = 0}

static inline void max_temp(struct temp *max, struct temp *temp)
{
	max->local = MAX(max->local, temp->local);
	max->remote = MAX(max->remote, temp->remote);
}

/*
 * Debugging
 */

#define SENSOR_DEBUG
#ifdef SENSOR_DEBUG
#define sensor_log(a...) applog(LOG_NOTICE, a)
#else
#define sensor_log(a...)
#endif
