#ifndef ZEPHYR_DRIVERS_BUS_485_H_
#define ZEPHYR_DRIVERS_BUS_485_H_

#include <zephyr/drivers/spi.h>

/* Client management API */
int bus_485_register_client(const struct device *dev, void *client);
int bus_485_unregister_client(const struct device *dev, void *client);

#endif /* ZEPHYR_DRIVERS_BUS_485_H_ */