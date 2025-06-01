#ifndef SPI_BUS_MANAGER_H
#define SPI_BUS_MANAGER_H

int32_t bus485_lock(const struct device * dev);
int32_t bus485_release(const struct device * dev);
int32_t bus485_send(const struct device * dev,
                    const uint8_t * buffer,
                    uint32_t count);
int32_t bus485_recv(const struct device * dev,
                    uint8_t * buffer,
                    uint32_t buffer_size,
                    uint32_t timeout_ms);
int32_t bus485_flush(const struct device * dev);
int32_t bus485_set_baudrate(const struct device * dev,
                            uint32_t baudrate);


#endif /* SPI_BUS_MANAGER_H */