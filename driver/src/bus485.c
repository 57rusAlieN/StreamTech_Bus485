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
    struct k_sem bus_sem;

    /* Очередь приема */
    struct k_fifo rx_fifo;
    uint8_t rx_buf[CONFIG_BUS485_RX_QUEUE_SIZE];
    uint16_t rx_head;
    uint16_t rx_tail;
    struct k_mutex rx_mutex;
};

/* API реализации */
int32_t bus485_lock(const struct device *dev)
{
    struct bus485_data *data = dev->data;

    if (k_sem_take(&data->bus_sem, K_MSEC(100)))
    {
        LOG_ERR("Bus acquisition timeout");
        return -EBUSY;
    }

    return 0;
}

int32_t bus485_release(const struct device *dev)
{
    struct bus485_data *data = dev->data;
    k_sem_give(&data->bus_sem);
    return 0;
}

int32_t bus485_send(const struct device *dev,
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

int32_t bus485_recv(const struct device *dev,
                           uint8_t *buffer, uint32_t buffer_size,
                           uint32_t timeout_ms)
{
    struct bus485_data *data = dev->data;
    const struct bus485_config *cfg = dev->config;

    /* Настраиваем буфер приема */
    // data->rx_buf = buffer;
    // data->rx_buf_size = buffer_size;
    uint32_t rx_received = 0;

    /* Убедимся, что передатчик выключен */
    gpio_pin_set_dt(&cfg->de_re, 0);

    /* Читаем данные из очереди с таймаутом */
    while (rx_received < buffer_size) {
        if (k_mutex_lock(&data->rx_mutex, K_MSEC(timeout_ms))) return −EAGAIN;
        
        if (data->rx_tail != data->rx_head) {
            buffer[rx_received++] = data->rx_buf[data->rx_tail];
            data->rx_tail = (data->rx_tail + 1) % CONFIG_BUS485_RX_QUEUE_SIZE;
            k_mutex_unlock(&data->rx_mutex);
        } else {
            /// Очередь пуста
            k_mutex_unlock(&data->rx_mutex);
            break;
        }
    }
    return rx_received;
}

int32_t bus485_flush(const struct device *dev)
{
    struct bus485_data *data = dev->data;
    uint8_t dummy;

    while (uart_poll_in(data->uart, &dummy) == 0)
    {
        /* Читаем и отбрасываем все данные */
    }

    /* Сбрасываем очередь */
    data->rx_head = 0;
    data->rx_tail = 0;

    return 0;
}

int32_t bus485_set_baudrate(const struct device *dev, uint32_t baudrate)
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
        if (uart_fifo_read(uart_dev, &byte, 1) == 1){
            /* Блокируем очередь */
            k_mutex_lock(&data->rx_mutex, K_FOREVER);
            uint16_t next_head = (data->rx_head + 1) % CONFIG_BUS485_RX_QUEUE_SIZE;
            
            if (next_head != data->rx_tail) {
                data->rx_buf[data->rx_head] = byte;
                data->rx_head = next_head;
            }
            /* Сигнализируем о получении данных */
            k_mutex_unlock(&data->rx_mutex);
        }
    }
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
    k_sem_init(&data->bus_sem, 1, 1);

    /* Инициализация очереди приема */
    k_fifo_init(&data->rx_fifo);
    k_mutex_init(&data->rx_mutex);
    data->rx_head = 0;
    data->rx_tail = 0;

    /* Регистрация обработчика прерываний */
    uart_irq_callback_user_data_set(data->uart, uart_isr, (void *)dev);
    uart_irq_rx_enable(data->uart);

    LOG_INF("BUS485 initialized (baud: %d)", cfg->uart_cfg.baudrate);
    return 0;
}

/* Devicetree макросы */
#define DT_DRV_COMPAT custom_bus485
#define BUS485_DRV_COMPAT custom_bus485


const struct bus485_driver_api bus485_api = {
    .lock = bus485_lock,
    .release = bus485_release,
    .send = bus485_send,
    .recv = bus485_recv,
    .flush = bus485_flush,
    .set_baudrate = bus485_set_baudrate,
};

#ifdef BUS485_USE_MULTIPLE_INSTANCE
/// Multiple instances
#define BUS485_NODE DT_NODELABEL(bus485)
#define BUS485_DEFINE(n)                                    \
    struct bus485_data bus485_data_##n;              \
                                                            \
    const struct bus485_config bus485_config_##n = { \
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
    DEVICE_DT_INST_DEFINE(n,             \
                        bus485_init,                      \
                        NULL,                             \
                        &bus485_data_##n,                 \
                        &bus485_config_##n,               \
                        POST_KERNEL,                      \
                        CONFIG_BUS485_INIT_PRIORITY,      \
                        &bus485_api); // <--- Регистрация API

DT_INST_FOREACH_STATUS_OKAY(BUS485_DEFINE)

#else
/// Single instance
#define BUS485_NODE DT_PATH(bus485)
struct bus485_data bus485_data;
const struct bus485_config bus485_config = { 
    //.de_re = GPIO_DT_SPEC_GET(DT_PATH(bus485), de_re_gpios),     
    .current_baudrate = DT_PROP(BUS485_NODE, current_speed), 
    .pre_delay_us = DT_PROP(BUS485_NODE, pre_delay_us),      
    .post_delay_us = DT_PROP(BUS485_NODE, post_delay_us),    
    .uart_cfg = { 
        .baudrate = DT_PROP(BUS485_NODE, current_speed), 
        .parity = UART_CFG_PARITY_NONE, 
        .stop_bits = UART_CFG_STOP_BITS_1, 
        .data_bits = UART_CFG_DATA_BITS_8, 
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE 
    } 
};                                                      

DEVICE_DT_DEFINE(BUS485_NODE,             \
                    bus485_init,                      \
                    NULL,                             \
                    &bus485_data,                 \
                    &bus485_config,               \
                    POST_KERNEL,                      \
                    CONFIG_BUS485_INIT_PRIORITY,      \
                    &bus485_api); // <--- Регистрация API
#endif // BUS485_USE_MULTIPLE_INSTANCE

