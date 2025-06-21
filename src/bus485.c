#include "bus485.h"
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bus485, CONFIG_BUS485_LOG_LEVEL);

/* Конфигурация драйвера */
struct bus485_config 
{
    struct uart_config uart_cfg;
    struct gpio_dt_spec de_re;
    
    /* Настройки */
    uint32_t current_baudrate;
    uint32_t pre_delay_us;
    uint32_t post_delay_us;
};

/* Структура данных драйвера */
struct bus485_data
{
    /* Аппаратные ресурсы */
    const struct device *uart;

    /* Синхронизация */
    struct k_sem tx_sem;
    struct k_sem rx_sem;
    struct k_sem bus_sem;

    /* Буферы */
    uint8_t *rx_buf;
    uint32_t rx_buf_size;
    volatile uint32_t rx_received;
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
    const struct bus485_config *cfg = dev->config;

    /* Захватываем право на передачу */
    if (k_sem_take(&data->tx_sem, K_MSEC(100))) {
        return -EBUSY;
    }

    /* Активируем передатчик */
    gpio_pin_set_dt(&cfg->de_re, 1);
    k_usleep(cfg->pre_delay_us);

    /* Включаем прерывания передачи */
    uart_irq_tx_enable(data->uart);

    /* Отправка данных через FIFO */
    for (uint32_t i = 0; i < count; i++) {
        while (!uart_irq_tx_ready(data->uart)) {
            k_yield();
        }
        uart_fifo_fill(data->uart, &buffer[i], 1);
    }

    /* Ждем завершения передачи */
    while (!uart_irq_tx_complete(data->uart)) {
        k_yield();
    }

    /* Деактивируем передатчик */
    k_usleep(cfg->post_delay_us);
    gpio_pin_set_dt(&cfg->de_re, 0);
    uart_irq_tx_disable(data->uart);

    k_sem_give(&data->tx_sem);
    return count;
}

static int32_t bus485_recv(const struct device *dev,
                           uint8_t *buffer, uint32_t buffer_size,
                           uint32_t timeout_ms)
{
    struct bus485_data *data = dev->data;
    const struct bus485_config *cfg = dev->config;

    /* Настраиваем буфер приема */
    data->rx_buf = buffer;
    data->rx_buf_size = buffer_size;
    data->rx_received = 0;

    /* Убедимся, что передатчик выключен */
    gpio_pin_set_dt(&cfg->de_re, 0);

    /* Ждем данные с таймаутом */
    if (k_sem_take(&data->rx_sem, K_MSEC(timeout_ms))) {
        return -ETIMEDOUT;
    }

    return data->rx_received;
}

static int32_t bus485_flush(const struct device *dev)
{
    struct bus485_data *data = dev->data;
    uint8_t dummy;

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

    err = uart_config_get(data->uart, &cfg);
    if (err)
    {
        return err;
    }

    cfg.baudrate = baudrate;
    return uart_configure(data->uart, &cfg);
}

/* Обработчик прерываний UART */
static void uart_isr(const struct device *uart_dev, void *user_data)
{
    const struct device *bus_dev = user_data;
    struct bus485_data *data = bus_dev->data;
    uint8_t byte;

    /* Обработка приема */
    while (uart_irq_update(uart_dev) && uart_irq_rx_ready(uart_dev)) {
        if (uart_fifo_read(uart_dev, &byte, 1) == 1) {
            if (data->rx_received < data->rx_buf_size) {
                data->rx_buf[data->rx_received++] = byte;
            }
        }
    }

    /* Сигнализируем о получении данных */
    k_sem_give(&data->rx_sem);
}

/* Инициализация драйвера */
static int bus485_init(const struct device *dev)
{

    const struct bus485_config *cfg = dev->config;
    struct bus485_data *data = dev->data;
    int ret;

    /* Инициализация GPIO */
    if (!gpio_is_ready_dt(&cfg->de_re)) {
        LOG_ERR("DE/RE GPIO not ready");
        return -ENODEV;
    }
    gpio_pin_configure_dt(&cfg->de_re, GPIO_OUTPUT_INACTIVE);

    /* Инициализация UART */
    ret = uart_configure(data->uart, &cfg->uart_cfg);
    if (ret) {
        LOG_ERR("UART config failed: %d", ret);
        return ret;
    }

    /* Инициализация семафоров */
    k_sem_init(&data->tx_sem, 1, 1);
    k_sem_init(&data->rx_sem, 0, 1);
    k_sem_init(&data->bus_sem, 1, 1);

    /* Регистрация обработчика прерываний */
    uart_irq_callback_user_data_set(data->uart, uart_isr, (void *)dev);
    uart_irq_rx_enable(data->uart);

    LOG_INF("BUS485 initialized (baud: %d)", cfg->uart_cfg.baudrate);
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
    static struct bus485_data bus485_data_##n;              \
                                                            \
    static const struct bus485_config bus485_config_##n = { \
        .uart = DEVICE_DT_GET(DT_INST_PHANDLE(n, uart)),    \
        .de_re = GPIO_DT_SPEC_INST_GET(n, de_re_gpios),     \
        .current_baudrate = DT_INST_PROP(n, current_speed), \
        .pre_delay_us = DT_INST_PROP(n, pre_delay_us),      \
        .post_delay_us = DT_INST_PROP(n, post_delay_us),    \
        .uart_cfg = { \
            .baudrate = DT_INST_PROP(n, current_speed), \
            .parity = UART_CFG_PARITY_NONE, \
            .stop_bits = UART_CFG_STOP_BITS_1, \
            .data_bits = UART_CFG_DATA_BITS_8, \
            .flow_ctrl = UART_CFG_FLOW_CTRL_NONE \
        } \
    };                                                      \
                                                            \
    DEVICE_DT_INST_DEFINE(n,                                \
                          bus485_init,                      \
                          NULL,                             \
                          &bus485_data_##n,                 \
                          &bus485_config_##n,             \
                          POST_KERNEL,                      \
                          CONFIG_BUS485_INIT_PRIORITY,      \
                          &bus485_api); // <--- Регистрация API

DT_INST_FOREACH_STATUS_OKAY(BUS485_DEFINE)
