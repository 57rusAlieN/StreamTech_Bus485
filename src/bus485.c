#include "bus485.h"
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bus485, CONFIG_BUS485_LOG_LEVEL);

/* Структура данных драйвера */
struct bus485_data
{
    struct k_sem bus_sem;
    const struct device *uart;
    struct gpio_dt_spec de_re;
    uint32_t current_baudrate;
    uint32_t pre_delay_us;
    uint32_t post_delay_us;
};

/* API реализации */
static int32_t bus485_lock(const struct device *dev)
{
    struct bus485_data *data = dev->data;

    if (k_sem_take(&data->bus_sem, K_MSEC(100)))
    {
        LOG_ERR("Bus acquisition timeout");
        return -EBUSY;
    }

    return 0;
}

static int32_t bus485_release(const struct device *dev)
{
    struct bus485_data *data = dev->data;
    k_sem_give(&data->bus_sem);
    return 0;
}

static int32_t bus485_send(const struct device *dev,
                           const uint8_t *buffer, uint32_t count)
{
    struct bus485_data *data = dev->data;
    int32_t sent = 0;

    if (k_sem_count_get(&data->bus_sem) != 0)
    {
        LOG_ERR("Bus not locked");
        return -EACCES;
    }

    /* Активация передатчика */
    gpio_pin_set_dt(&data->de_re, 1);
    k_usleep(data->pre_delay_us);

    /* Отправка данных */
    for (; sent < count; sent++)
    {
        uart_poll_out(data->uart, buffer[sent]);
    }

    /* Деактивация передатчика */
    k_usleep(data->post_delay_us);
    gpio_pin_set_dt(&data->de_re, 0);

    return sent;
}

static int32_t bus485_recv(const struct device *dev,
                           uint8_t *buffer, uint32_t buffer_size,
                           uint32_t timeout_ms)
{
    struct bus485_data *data = dev->data;
    int32_t received = 0;
    
    if (k_sem_count_get(&data->bus_sem) != 0)
    {
        LOG_ERR("Bus not locked");
        return -EACCES;
    }
    
    int64_t end_time = k_uptime_get() + timeout_ms;

    /* Убедимся, что передатчик выключен */
    gpio_pin_set_dt(&data->de_re, 0);

    while (received < buffer_size && k_uptime_get() < end_time)
    {
        if (uart_poll_in(data->uart, &buffer[received]) == 0)
        {
            received++;
        }
        else
        {
            k_msleep(1);
        }
    }

    return (received > 0) ? received : -ETIMEDOUT;
}

static int32_t bus485_flush(const struct device *dev)
{
    struct bus485_data *data = dev->data;
    uint8_t dummy;

    if (k_sem_count_get(&data->bus_sem) != 0)
    {
        LOG_ERR("Bus not locked");
        return -EACCES;
    }

    while (uart_poll_in(data->uart, &dummy) == 0)
    {
        /* Читаем и отбрасываем все данные */
    }

    return 0;
}

static int32_t bus485_set_baudrate(const struct device *dev, uint32_t baudrate)
{
    struct bus485_data *data = dev->data;
    struct uart_config cfg;
    int err;

    if (k_sem_count_get(&data->bus_sem) != 0)
    {
        LOG_ERR("Bus not locked");
        return -EACCES;
    }

    err = uart_config_get(data->uart, &cfg);
    if (err)
    {
        return err;
    }

    cfg.baudrate = baudrate;
    err = uart_configure(data->uart, &cfg);
    if (!err)
    {
        data->current_baudrate = baudrate;
    }

    return err;
}

/* Инициализация драйвера */
static int bus485_init(const struct device *dev)
{
    struct bus485_data *data = dev->data;

    if (!device_is_ready(data->uart))
    {
        LOG_ERR("UART device not ready");
        return -ENODEV;
    }

    if (!device_is_ready(data->de_re.port))
    {
        LOG_ERR("GPIO device not ready");
        return -ENODEV;
    }

    gpio_pin_configure_dt(&data->de_re, GPIO_OUTPUT_INACTIVE);
    k_sem_init(&data->bus_sem, 1, 1);

    LOG_INF("BUS485 initialized at %u baud", data->current_baudrate);
    return 0;
}

/* Devicetree макросы */
#define DT_DRV_COMPAT custom_bus485

static const struct bus485_driver_api bus485_api = {
    .lock = bus485_lock,
    .release = bus485_release,
    .send = bus485_send,
    .recv = bus485_recv,
    .flush = bus485_flush,
    .set_baudrate = bus485_set_baudrate,
};

#define BUS485_DEFINE(n)                                    \
    static struct bus485_data bus485_data_##n = {           \
        .uart = DEVICE_DT_GET(DT_INST_PHANDLE(n, uart)),    \
        .de_re = GPIO_DT_SPEC_INST_GET(n, de_re_gpios),     \
        .current_baudrate = DT_INST_PROP(n, current_speed), \
        .pre_delay_us = DT_INST_PROP(n, pre_delay_us),      \
        .post_delay_us = DT_INST_PROP(n, post_delay_us),    \
    };                                                      \
                                                            \
    DEVICE_DT_INST_DEFINE(n,                                \
                          bus485_init,                      \
                          NULL,                             \
                          &bus485_data_##n,                 \
                          NULL,                             \
                          POST_KERNEL,                      \
                          CONFIG_BUS485_INIT_PRIORITY,      \
                          &bus485_api); // <--- Регистрация API

DT_INST_FOREACH_STATUS_OKAY(BUS485_DEFINE)
