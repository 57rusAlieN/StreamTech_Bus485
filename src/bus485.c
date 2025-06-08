// #include "bus485.h"

// int32_t bus485_lock(const struct device *dev)
// {
//     struct spi_context *ctx = dev->data;

//     if (!ctx->lock_initialized)
//     {
//         return -ENODEV;
//     }

//     /* Рекурсивный захват тем же владельцем */
//     if (ctx->lock.current_owner == requester)
//     {
//         atomic_inc(&ctx->lock.owner_count);
//         return 0;
//     }

//     /* Попытка захватить семафор */
//     if (k_sem_take(&ctx->lock.sem, K_NO_WAIT) == 0)
//     {
//         ctx->lock.current_owner = requester;
//         atomic_set(&ctx->lock.owner_count, 1);
//         return 0;
//     }

//     /* Если шина занята, добавляем в очередь ожидания */
//     struct k_fifo *waitq = &ctx->lock.waiters;
//     struct spi_waiter_node *node = k_malloc(sizeof(*node));
//     if (!node)
//     {
//         return -ENOMEM;
//     }

//     node->requester = requester;
//     k_fifo_put(waitq, node);

//     /* Ожидание семафора с таймаутом */
//     int ret = k_sem_take(&ctx->lock.sem, timeout);
//     if (ret != 0)
//     {
//         /* Удаляем из очереди при таймауте */
//         struct spi_waiter_node *tmp;
//         while ((tmp = k_fifo_get(waitq, K_NO_WAIT)) != NULL)
//         {
//             if (tmp->requester == requester)
//             {
//                 k_free(tmp);
//                 break;
//             }
//             k_fifo_put(waitq, tmp);
//         }
//         return ret;
//     }

//     ctx->lock.current_owner = requester;
//     atomic_set(&ctx->lock.owner_count, 1);
//     return 0;
// }

// int32_t bus485_release(const struct device *dev)
// {
//     struct spi_context *ctx = dev->data;

//     if (!ctx->lock_initialized)
//     {
//         return -ENODEV;
//     }

//     if (ctx->lock.current_owner != requester)
//     {
//         return -EPERM;
//     }

//     /* Уменьшаем счетчик рекурсивных захватов */
//     if (atomic_dec(&ctx->lock.owner_count) > 1)
//     {
//         return 0;
//     }

//     /* Передаем управление следующему в очереди */
//     struct spi_waiter_node *next = k_fifo_get(&ctx->lock.waiters, K_NO_WAIT);
//     if (next != NULL)
//     {
//         ctx->lock.current_owner = next->requester;
//         atomic_set(&ctx->lock.owner_count, 1);
//         k_free(next);
//     }
//     else
//     {
//         ctx->lock.current_owner = NULL;
//         atomic_set(&ctx->lock.owner_count, 0);
//     }

//     k_sem_give(&ctx->lock.sem);
//     return 0;
// }

// int32_t bus485_send(const struct device *dev,
//                     const uint8_t *buffer,
//                     uint32_t count)
// {
//     return 0;
// }

// int32_t bus485_recv(const struct device *dev,
//                     uint8_t *buffer,
//                     uint32_t buffer_size,
//                     uint32_t timeout_ms)
// {
//     return 0;
// }
// int32_t bus485_flush(const struct device *dev)
// {
//     return 0;
// }
// int32_t bus485_set_baudrate(const struct device *dev,
//                             uint32_t baudrate)
// {
//     return 0;
// }
#include "bus485.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(rs485, CONFIG_RS485_LOG_LEVEL);

/* Конфигурация из devicetree */
#define UART_NODE        DT_NODELABEL(uart0)
#define DE_RE_GPIO_NODE  DT_NODELABEL(rs485_gpio)

static const struct device *const uart_dev = DEVICE_DT_GET(UART_NODE);
static const struct gpio_dt_spec de_re_gpio = GPIO_DT_SPEC_GET(DE_RE_GPIO_NODE, gpios);

static struct {
    struct k_mutex bus_lock;
    bool initialized;
} rs485_ctx;

int32_t bus485_init(void)
{
    if (rs485_ctx.initialized) {
        return 0;
    }

    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART device not ready");
        return -ENODEV;
    }

    if (!device_is_ready(de_re_gpio.port)) {
        LOG_ERR("GPIO device not ready");
        return -ENODEV;
    }

    gpio_pin_configure_dt(&de_re_gpio, GPIO_OUTPUT_INACTIVE);
    k_mutex_init(&rs485_ctx.bus_lock);

    rs485_ctx.initialized = true;
    LOG_INF("RS485 initialized");
    return 0;
}

int32_t bus485_lock(k_timeout_t timeout)
{
    if (!rs485_ctx.initialized) {
        return -ENODEV;
    }

    if (k_mutex_lock(&rs485_ctx.bus_lock, timeout) != 0) {
        return -EBUSY;
    }

    return 0;
}

int32_t bus485_release(void)
{
    if (rs485_ctx.initialized) {
        k_mutex_unlock(&rs485_ctx.bus_lock);
    }
    return 0;
}

int32_t bus485_send(const uint8_t *data, size_t len)
{
    if (!rs485_ctx.initialized) {
        return -ENODEV;
    }

    if (k_mutex_lock(&rs485_ctx.bus_lock, K_NO_WAIT) != 0) {
        return -EBUSY;
    }

    // Активация линии передачи
    gpio_pin_set_dt(&de_re_gpio, 1);
    k_usleep(CONFIG_RS485_PRE_DELAY_US); // Пред-задержка

    int sent = 0;
    for (; sent < len; sent++) {
        uart_poll_out(uart_dev, data[sent]);
    }

    k_usleep(CONFIG_RS485_POST_DELAY_US); // Пост-задержка
    gpio_pin_set_dt(&de_re_gpio, 0);

    k_mutex_unlock(&rs485_ctx.bus_lock);
    return sent;
}

int32_t bus485_recv(uint8_t *buf, size_t len, k_timeout_t timeout)
{
    if (!rs485_ctx.initialized) {
        return -ENODEV;
    }

    int64_t end_time = k_uptime_get() + timeout.ticks;
    int received = 0;

    // Убедимся, что передатчик выключен
    gpio_pin_set_dt(&de_re_gpio, 0);

    while (received < len && k_uptime_get() < end_time) {
        if (uart_poll_in(uart_dev, &buf[received]) == 0) {
            received++;
        } else {
            k_msleep(1);
        }
    }

    return received > 0 ? received : -ETIMEDOUT;
}

int32_t bus485_flush(void)
{
    if (!rs485_ctx.initialized) return;

    uint8_t dummy;
    while (uart_poll_in(uart_dev, &dummy) == 0) {
        // Читаем и отбрасываем все данные
    }
    return 0;
}

int32_t bus485_set_baudrate(uint32_t baudrate)
{
    if (!rs485_ctx.initialized) {
        return -ENODEV;
    }

    struct uart_config cfg;
    int ret = uart_config_get(uart_dev, &cfg);
    if (ret != 0) {
        return ret;
    }

    cfg.baudrate = baudrate;
    return uart_configure(uart_dev, &cfg);
}

static const struct rs485_driver_api rs485_api = {
    .lock = bus485_lock,
    .release = bus485_release,
    .send = bus485_send,
    .recv = bus485_recv,
    .flush = bus485_flush,
    .set_baudrate = bus485_set_baudrate,
};

#define DT_DRV_COMPAT bus485_mutex

// #define RS485_INIT(n) \
//     static const struct rs485_config rs485_cfg_##n = { \
//         .uart_dev = DEVICE_DT_GET(DT_INST_PHANDLE(n, uart)), \
//         .de_re_gpio = GPIO_DT_SPEC_INST_GET(n, de_re_gpios), \
//         .pre_delay_us = DT_INST_PROP(n, pre_delay_us), \
//         .post_delay_us = DT_INST_PROP(n, post_delay_us), \
//         .callback = NULL, \
//         .user_data = NULL, \
//     }; \
//     \
//     static struct rs485_data rs485_data_##n; \
//     \
//     DEVICE_DT_INST_DEFINE(n, \
//         bus485_init, \
//         NULL, \
//         &rs485_data_##n, \
//         &rs485_cfg_##n, \
//         POST_KERNEL, \
//         CONFIG_RS485_INIT_PRIORITY, \
//         &rs485_driver_api); /* <-- Регистрация API */

// DT_INST_FOREACH_STATUS_OKAY(RS485_INIT)


#define RS485_DEFINE(n) \
    static struct rs485_data rs485_data_##n = { \
        .uart = DEVICE_DT_GET(DT_INST_PHANDLE(n, uart)), \
        .de_re = GPIO_DT_SPEC_INST_GET(n, de_re_gpios), \
        .pre_delay = DT_INST_PROP(n, pre_delay_us), \
        .post_delay = DT_INST_PROP(n, post_delay_us), \
    }; \
    DEVICE_DT_INST_DEFINE(n, \
        bus485_init, \
        NULL, \
        &rs485_data_##n, \
        NULL, \
        POST_KERNEL, \
        CONFIG_RS485_INIT_PRIORITY, \
        &rs485_driver_api);  /* <-- Регистрация API */

DT_INST_FOREACH_STATUS_OKAY(RS485_DEFINE)