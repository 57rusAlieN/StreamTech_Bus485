#include "bus485.h"
#include "manager.h"

struct bus485_data {
    struct spi_bus_lock lock;
    const struct spi_driver_api *original_api;
};

static int bus485_init(const struct device *dev)
{
    struct bus485_data *data = dev->data;
    
    /* Инициализация механизма блокировки */
    spi_bus_lock_init(&data->lock, 
                     CONFIG_SPI_EXTENDED_LOCK_TIMEOUT_MS);
    
    return 0;
}

static int bus485_transceive(const struct device *dev,
                            const struct spi_config *config,
                            const struct spi_buf_set *tx_bufs,
                            const struct spi_buf_set *rx_bufs,
                            void *user_data)
{
    struct bus485_data *data = dev->data;
    int ret;
    
    /* Захват шины */
    ret = spi_bus_lock_take(&data->lock, user_data);
    if (ret != 0) {
        return ret;
    }
    
    /* Вызов оригинального API */
    ret = data->original_api->transceive(dev, config, tx_bufs, rx_bufs);
    
    /* Освобождение шины */
    spi_bus_lock_release(&data->lock, user_data);
    
    return ret;
}

/* Регистрация расширенного драйвера */
#define SPI_EXT_DEVICE(n) \
    static struct bus485_data bus485_data_##n; \
    static int bus485_init_##n(const struct device *dev) { \
        bus485_data_##n.original_api = \
            (const struct spi_driver_api *)dev->api; \
        return bus485_init(dev); \
    } \
    DEVICE_DT_DEFINE(DT_INST(n, vendor_bus485), \
                    bus485_init_##n, NULL, \
                    &bus485_data_##n, NULL, \
                    POST_KERNEL, CONFIG_SPI_INIT_PRIORITY, \
                    &(struct spi_driver_api){ \
                        .transceive = bus485_transceive, \
                    });

DT_INST_FOREACH_STATUS_OKAY(SPI_EXT_DEVICE)