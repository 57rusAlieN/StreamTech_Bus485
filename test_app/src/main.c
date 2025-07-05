#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include <bus485.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define THREAD_STACK_SIZE 1024
#define THREAD_PRIORITY   5

/* Устройство RS485 из Devicetree */
#define BUS485_NODE DT_NODELABEL(bus485)
static const struct device *bus485_dev = DEVICE_DT_GET(BUS485_NODE);

/* Данные для отправки каждым потоком */
static uint8_t thread1_data[] = {0xAA, 0xBB, 0xCC};
static uint8_t thread2_data[] = {0x11, 0x22, 0x33};
static uint8_t thread3_data[] = {0x55, 0x66, 0x77};

/* Функция потока */
void bus485_thread(void *delay_ms, void *data_ptr, void *size_ptr)
{
    uint32_t delay = *(uint32_t *)delay_ms;
    uint8_t *data = (uint8_t *)data_ptr;
    uint32_t size = *(uint32_t *)size_ptr;
    int ret;

    while (1) {
        LOG_INF("Поток %p пытается захватить шину (задержка %d мс)", 
               k_current_get(), delay);

        /* Захват шины с таймаутом */
        ret = bus485_lock(bus485_dev);
        if (ret != 0) {
            LOG_ERR("Ошибка захвата шины: %d", ret);
            k_msleep(delay);
            continue;
        }

        /* Критическая секция - работа с шиной */
        LOG_INF("Поток %p захватил шину", k_current_get());
        
        /* Отправка данных */
        ret = bus485_send(bus485_dev, data, size);
        if (ret < 0) {
            LOG_ERR("Ошибка отправки: %d", ret);
        } else {
            LOG_INF("Отправлено %d байт", ret);
        }

        /* Искусственная задержка внутри критической секции */
        k_msleep(delay);

        /* Освобождение шины */
        bus485_release(bus485_dev);
        LOG_INF("Поток %p освободил шину", k_current_get());

        k_msleep(delay * 2);
    }
}

/* Создание потоков */
K_THREAD_DEFINE(thread1_id, THREAD_STACK_SIZE,
               bus485_thread, (void *)100, thread1_data, (void *)sizeof(thread1_data),
               THREAD_PRIORITY, 0, 0);

K_THREAD_DEFINE(thread2_id, THREAD_STACK_SIZE,
               bus485_thread, (void *)200, thread2_data, (void *)sizeof(thread2_data),
               THREAD_PRIORITY, 0, 0);

K_THREAD_DEFINE(thread3_id, THREAD_STACK_SIZE,
               bus485_thread, (void *)300, thread3_data, (void *)sizeof(thread3_data),
               THREAD_PRIORITY, 0, 0);

int main(void)
{
    if (!device_is_ready(bus485_dev)) {
        LOG_ERR("Устройство RS485 не готово!");
        return -1;
    }

    LOG_INF("Демонстрация конкурентного доступа к RS485 запущена");
	return 0;
}