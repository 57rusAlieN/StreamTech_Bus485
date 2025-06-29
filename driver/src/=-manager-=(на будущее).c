#include "manager.h"

struct spi_waiter
{
    sys_snode_t node;
    const void *requester;
};

void spi_bus_lock_init(struct spi_bus_lock *lock, uint32_t timeout_ms)
{
    k_sem_init(&lock->sem, 1, 1);
    k_fifo_init(&lock->waiters);
    atomic_set(&lock->owner_count, 0);
    lock->current_owner = NULL;
    lock->timeout_ms = timeout_ms;
}

int32_t bus485_lock(const struct device *dev)
{
    struct spi_context *ctx = dev->data;

    if (!ctx->lock_initialized)
    {
        return -ENODEV;
    }

    /* Рекурсивный захват тем же владельцем */
    if (ctx->lock.current_owner == requester)
    {
        atomic_inc(&ctx->lock.owner_count);
        return 0;
    }

    /* Попытка захватить семафор */
    if (k_sem_take(&ctx->lock.sem, K_NO_WAIT) == 0)
    {
        ctx->lock.current_owner = requester;
        atomic_set(&ctx->lock.owner_count, 1);
        return 0;
    }

    /* Если шина занята, добавляем в очередь ожидания */
    struct k_fifo *waitq = &ctx->lock.waiters;
    struct spi_waiter_node *node = k_malloc(sizeof(*node));
    if (!node)
    {
        return -ENOMEM;
    }

    node->requester = requester;
    k_fifo_put(waitq, node);

    /* Ожидание семафора с таймаутом */
    int ret = k_sem_take(&ctx->lock.sem, timeout);
    if (ret != 0)
    {
        /* Удаляем из очереди при таймауте */
        struct spi_waiter_node *tmp;
        while ((tmp = k_fifo_get(waitq, K_NO_WAIT)) != NULL)
        {
            if (tmp->requester == requester)
            {
                k_free(tmp);
                break;
            }
            k_fifo_put(waitq, tmp);
        }
        return ret;
    }

    ctx->lock.current_owner = requester;
    atomic_set(&ctx->lock.owner_count, 1);
    return 0;
}

int32_t bus485_release(const struct device *dev)
{
    struct spi_context *ctx = dev->data;

    if (!ctx->lock_initialized)
    {
        return -ENODEV;
    }

    if (ctx->lock.current_owner != requester)
    {
        return -EPERM;
    }

    /* Уменьшаем счетчик рекурсивных захватов */
    if (atomic_dec(&ctx->lock.owner_count) > 1)
    {
        return 0;
    }

    /* Передаем управление следующему в очереди */
    struct spi_waiter_node *next = k_fifo_get(&ctx->lock.waiters, K_NO_WAIT);
    if (next != NULL)
    {
        ctx->lock.current_owner = next->requester;
        atomic_set(&ctx->lock.owner_count, 1);
        k_free(next);
    }
    else
    {
        ctx->lock.current_owner = NULL;
        atomic_set(&ctx->lock.owner_count, 0);
    }

    k_sem_give(&ctx->lock.sem);
    return 0;
}

int32_t bus485_send(const struct device *dev,ё
                    const uint8_t *buffer,
                    uint32_t count)
{
    return 0;
}

int32_t bus485_recv(const struct device *dev,
                    uint8_t *buffer,
                    uint32_t buffer_size,
                    uint32_t timeout_ms)
{
    return 0;
}
int32_t bus485_flush(const struct device *dev)
{
    return 0;
}
int32_t bus485_set_baudrate(const struct device *dev,
                            uint32_t baudrate)
{
    return 0;
}
