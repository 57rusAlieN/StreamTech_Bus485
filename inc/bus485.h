#ifndef ZEPHYR_DRIVERS_BUS485_H_
#define ZEPHYR_DRIVERS_BUS485_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif
struct bus485_driver_api {
    int32_t (*lock)(const struct device *dev);
    int32_t (*release)(const struct device *dev);
    int32_t (*send)(const struct device *dev, const uint8_t *buffer, uint32_t count);
    int32_t (*recv)(const struct device *dev, uint8_t *buffer, uint32_t buffer_size, uint32_t timeout_ms);
    int32_t (*flush)(const struct device *dev);
    int32_t (*set_baudrate)(const struct device *dev, uint32_t baudrate);
};

/**
 * @brief Захват эксклюзивного доступа к шине
 * @param dev Указатель на устройство RS485
 * @return 0 при успехе, -EBUSY при занятой шине, -EIO при ошибке
 */
static int32_t bus485_lock(const struct device *dev);

/**
 * @brief Освобождение шины
 * @param dev Указатель на устройство RS485
 * @return 0 при успехе, -EIO при ошибке
 */
static int32_t bus485_release(const struct device *dev);

/**
 * @brief Отправка данных по шине
 * @param dev Указатель на устройство RS485
 * @param buffer Буфер с данными для отправки
 * @param count Размер данных в байтах
 * @return Количество отправленных байт или код ошибки
 */
static int32_t bus485_send(const struct device *dev, const uint8_t *buffer, uint32_t count);

/**
 * @brief Прием данных с шины
 * @param dev Указатель на устройство RS485
 * @param buffer Буфер для записи данных
 * @param buffer_size Максимальный размер буфера
 * @param timeout_ms Таймаут ожидания в миллисекундах
 * @return Количество принятых байт или код ошибки
 */
static int32_t bus485_recv(const struct device *dev, uint8_t *buffer, 
                   uint32_t buffer_size, uint32_t timeout_ms);

/**
 * @brief Очистка буферов приема/передачи
 * @param dev Указатель на устройство RS485
 * @return 0 при успехе, код ошибки при неудаче
 */
static int32_t bus485_flush(const struct device *dev);

/**
 * @brief Установка скорости обмена
 * @param dev Указатель на устройство RS485
 * @param baudrate Новая скорость (например, 9600, 115200)
 * @return 0 при успехе, код ошибки при неудаче
 */
static int32_t bus485_set_baudrate(const struct device *dev, uint32_t baudrate);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_BUS485_H_ */