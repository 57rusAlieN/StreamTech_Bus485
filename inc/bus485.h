#ifndef ZEPHYR_DRIVERS_BUS_485_H_
#define ZEPHYR_DRIVERS_BUS_485_H_

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

struct rs485_config {
    const struct device *uart_dev;
    struct gpio_dt_spec de_re_gpio;
    uint32_t pre_delay_us;
    uint32_t post_delay_us;
    uart_irq_callback_user_data_t callback;  // Пользовательский callback
    void *user_data;                        // Данные для callback
};

struct rs485_data {
    struct uart_config uart_cfg;
    struct k_sem tx_sem;                    // Семафор для синхронизации TX
    struct k_sem rx_sem;                    // Семафор для синхронизации RX
    uint8_t *rx_buf;                        // Буфер приема
    size_t rx_buf_len;                      // Длина буфера приема
    size_t rx_received;                     // Получено байт
};
struct rs485_driver_api {
    int (*lock)(k_timeout_t timeout);
    void (*release)(void);
    int (*send)(const uint8_t *data, size_t len);
    int (*recv)(uint8_t *buf, size_t len, k_timeout_t timeout);
    void (*flush)(void);
    int (*set_baudrate)(uint32_t baudrate);
};

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

#endif /* ZEPHYR_DRIVERS_BUS_485_H_ */