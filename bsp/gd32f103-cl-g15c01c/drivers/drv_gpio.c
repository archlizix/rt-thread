/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2020-12-27     iysheng           first version
 * 2021-01-01     iysheng           support exti interrupt
 */

#include <board.h>
#include "drv_gpio.h"

#ifdef RT_USING_PIN

#if defined(GPIOG)
#define __GD32_PORT_MAX 7u
#elif defined(GPIOF)
#define __GD32_PORT_MAX 6u
#elif defined(GPIOE)
#define __GD32_PORT_MAX 5u
#elif defined(GPIOD)
#define __GD32_PORT_MAX 4u
#elif defined(GPIOC)
#define __GD32_PORT_MAX 3u
#elif defined(GPIOB)
#define __GD32_PORT_MAX 2u
#elif defined(GPIOA)
#define __GD32_PORT_MAX 1u
#else
#define __GD32_PORT_MAX 0u
#error Unsupported GD32 GPIO peripheral.
#endif

#define PIN_GDPORT_MAX __GD32_PORT_MAX

static const struct pin_irq_map pin_irq_map[] =
{
#if defined(SOC_SERIES_GD32F1)
    {GPIO_PIN_0, EXTI0_IRQn},
    {GPIO_PIN_1, EXTI1_IRQn},
    {GPIO_PIN_2, EXTI2_IRQn},
    {GPIO_PIN_3, EXTI3_IRQn},
    {GPIO_PIN_4, EXTI4_IRQn},
    {GPIO_PIN_5, EXTI5_9_IRQn},
    {GPIO_PIN_6, EXTI5_9_IRQn},
    {GPIO_PIN_7, EXTI5_9_IRQn},
    {GPIO_PIN_8, EXTI5_9_IRQn},
    {GPIO_PIN_9, EXTI5_9_IRQn},
    {GPIO_PIN_10, EXTI10_15_IRQn},
    {GPIO_PIN_11, EXTI10_15_IRQn},
    {GPIO_PIN_12, EXTI10_15_IRQn},
    {GPIO_PIN_13, EXTI10_15_IRQn},
    {GPIO_PIN_14, EXTI10_15_IRQn},
    {GPIO_PIN_15, EXTI10_15_IRQn},
#else
#error "Unsupported soc series"
#endif
};

static struct rt_pin_irq_hdr pin_irq_hdr_tab[] =
{
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
};
static uint32_t pin_irq_enable_mask = 0;

#define ITEM_NUM(items) sizeof(items) / sizeof(items[0])

static rt_base_t gd32_pin_get(const char *name)
{
    rt_base_t pin = 0;
    int hw_port_num, hw_pin_num = 0;
    int i, name_len;

    name_len = rt_strlen(name);

    if ((name_len < 4) || (name_len >= 6))
    {
        return -RT_EINVAL;
    }
    if ((name[0] != 'P') || (name[2] != '.'))
    {
        return -RT_EINVAL;
    }

    if ((name[1] >= 'A') && (name[1] <= 'Z'))
    {
        hw_port_num = (int)(name[1] - 'A');
    }
    else
    {
        return -RT_EINVAL;
    }

    for (i = 3; i < name_len; i++)
    {
        hw_pin_num *= 10;
        hw_pin_num += name[i] - '0';
    }

    pin = PIN_NUM(hw_port_num, hw_pin_num);

    return pin;
}

static void gd32_pin_write(rt_device_t dev, rt_base_t pin, rt_base_t value)
{
    uint32_t gpio_port;
    uint32_t gpio_pin;

    if (PIN_PORT(pin) < PIN_GDPORT_MAX)
    {
        gpio_port = PIN_GDPORT(pin);
        gpio_pin = PIN_GDPIN(pin);

        gpio_bit_write(gpio_port, gpio_pin, (bit_status)value);
    }
}

static int gd32_pin_read(rt_device_t dev, rt_base_t pin)
{
    uint32_t gpio_port;
    uint32_t gpio_pin;
    int value = RESET;

    if (PIN_PORT(pin) < PIN_GDPORT_MAX)
    {
        gpio_port = PIN_GDPORT(pin);
        gpio_pin = PIN_GDPIN(pin);
        value = gpio_input_bit_get(gpio_port, gpio_pin);
    }

    return value;
}

static void gd32_pin_mode(rt_device_t dev, rt_base_t pin, rt_base_t mode)
{
    if (PIN_PORT(pin) >= PIN_GDPORT_MAX)
    {
		return;
    }

    uint32_t gpio_port = PIN_GDPORT(pin);
    uint32_t gpio_pin = PIN_GDPIN(pin);

	uint32_t mode_gd;
    switch (mode)
    {
        case PIN_MODE_OUTPUT:
            mode_gd = GPIO_MODE_OUT_PP;
            break;
        case PIN_MODE_INPUT:
            mode_gd = GPIO_MODE_IN_FLOATING;
            break;
        case PIN_MODE_INPUT_PULLUP:
            mode_gd = GPIO_MODE_IPD;
            break;
        case PIN_MODE_INPUT_PULLDOWN:
            mode_gd = GPIO_MODE_IPU;
            break;
        case PIN_MODE_OUTPUT_OD:
            mode_gd = GPIO_MODE_OUT_OD;
            break;
        default:
            mode_gd = GPIO_MODE_IN_FLOATING;
            break;
    }

    gpio_init(gpio_port, mode_gd, GPIO_OSPEED_2MHZ, gpio_pin);
}

rt_inline rt_int32_t bit2bitno(rt_uint32_t bit)
{
    int i;
    for (i = 0; i < 32; i++)
    {
        if ((0x01 << i) == bit)
        {
            return i;
        }
    }
    return -1;
}

rt_inline const struct pin_irq_map *get_pin_irq_map(uint32_t pinbit)
{
    rt_int32_t mapindex = bit2bitno(pinbit);
    if (mapindex < 0 || mapindex >= ITEM_NUM(pin_irq_map))
    {
        return RT_NULL;
    }
    return &pin_irq_map[mapindex];
};

static rt_err_t gd32_pin_attach_irq(struct rt_device *device, rt_int32_t pin,
                                     rt_uint32_t mode, void (*hdr)(void *args), void *args)
{
    rt_base_t level;
    rt_int32_t irqindex = -1;

    if (PIN_PORT(pin) >= PIN_GDPORT_MAX)
    {
        return -RT_ENOSYS;
    }

    irqindex = bit2bitno(PIN_GDPIN(pin));
    if (irqindex < 0 || irqindex >= ITEM_NUM(pin_irq_map))
    {
        return RT_ENOSYS;
    }

    level = rt_hw_interrupt_disable();
    if (pin_irq_hdr_tab[irqindex].pin == pin &&
        pin_irq_hdr_tab[irqindex].hdr == hdr &&
        pin_irq_hdr_tab[irqindex].mode == mode &&
        pin_irq_hdr_tab[irqindex].args == args)
    {
        rt_hw_interrupt_enable(level);
        return RT_EOK;
    }
    if (pin_irq_hdr_tab[irqindex].pin != -1)
    {
        rt_hw_interrupt_enable(level);
        return RT_EBUSY;
    }
    pin_irq_hdr_tab[irqindex].pin = pin;
    pin_irq_hdr_tab[irqindex].hdr = hdr;
    pin_irq_hdr_tab[irqindex].mode = mode;
    pin_irq_hdr_tab[irqindex].args = args;
    rt_hw_interrupt_enable(level);

    return RT_EOK;
}

static rt_err_t gd32_pin_dettach_irq(struct rt_device *device, rt_int32_t pin)
{
    rt_base_t level;
    rt_int32_t irqindex = -1;

    if (PIN_PORT(pin) >= PIN_GDPORT_MAX)
    {
        return -RT_ENOSYS;
    }

    irqindex = bit2bitno(PIN_GDPIN(pin));
    if (irqindex < 0 || irqindex >= ITEM_NUM(pin_irq_map))
    {
        return RT_ENOSYS;
    }

    level = rt_hw_interrupt_disable();
    if (pin_irq_hdr_tab[irqindex].pin == -1)
    {
        rt_hw_interrupt_enable(level);
        return RT_EOK;
    }
    pin_irq_hdr_tab[irqindex].pin = -1;
    pin_irq_hdr_tab[irqindex].hdr = RT_NULL;
    pin_irq_hdr_tab[irqindex].mode = 0;
    pin_irq_hdr_tab[irqindex].args = RT_NULL;
    rt_hw_interrupt_enable(level);

    return RT_EOK;
}

static rt_err_t gd32_pin_irq_enable(struct rt_device *device, rt_base_t pin,
                                     rt_uint32_t enabled)
{
    const struct pin_irq_map *irqmap;
    rt_base_t level;
    rt_int32_t irqindex = -1;

    if (PIN_PORT(pin) >= PIN_GDPORT_MAX)
    {
        return -RT_ENOSYS;
    }

    uint32_t exti_line = PIN_GDPIN(pin);
    uint32_t exti_mode = EXTI_INTERRUPT;
    if (enabled == PIN_IRQ_ENABLE)
    {
        irqindex = bit2bitno(exti_line);
        if (irqindex < 0 || irqindex >= ITEM_NUM(pin_irq_map))
        {
            return -RT_ENOSYS;
        }

        level = rt_hw_interrupt_disable();
        if (pin_irq_hdr_tab[irqindex].pin == -1)
        {
            rt_hw_interrupt_enable(level);
            return -RT_ENOSYS;
        }

        irqmap = &pin_irq_map[irqindex];

		uint32_t exti_trig;
        switch (pin_irq_hdr_tab[irqindex].mode)
        {
        case PIN_IRQ_MODE_RISING:
            exti_trig = EXTI_TRIG_RISING;
            break;
        case PIN_IRQ_MODE_FALLING:
            exti_trig = EXTI_TRIG_FALLING;
            break;
        case PIN_IRQ_MODE_RISING_FALLING:
            exti_trig = EXTI_TRIG_BOTH;
            break;
        default:
            break;
        }

		/* enable the AF clock */
		rcu_periph_clock_enable(RCU_AF);

	    /* enable and set interrupt priority */
		nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);
        nvic_irq_enable(irqmap->irqno, 5U, 0U);

		/* connect key EXTI line to key GPIO pin */
		gpio_exti_source_select(PIN_PORT(pin), PIN_NO(pin));

        /* configure EXTI line */
		exti_init((exti_line_enum)(exti_line), (exti_mode_enum)exti_mode, (exti_trig_type_enum)exti_trig);
		exti_interrupt_flag_clear((exti_line_enum)(exti_line));

        rt_hw_interrupt_enable(level);
    }
    else if (enabled == PIN_IRQ_DISABLE)
    {
		irqmap = get_pin_irq_map(exti_line);
        if (irqmap == RT_NULL)
        {
            return -RT_EINVAL;
        }
        nvic_irq_disable(irqmap->irqno);
    }
    else
    {
        return -RT_ENOSYS;
    }

    return RT_EOK;
}
const static struct rt_pin_ops _gd32_pin_ops =
{
    gd32_pin_mode,
    gd32_pin_write,
    gd32_pin_read,
    gd32_pin_attach_irq,
    gd32_pin_dettach_irq,
    gd32_pin_irq_enable,
    gd32_pin_get,
};

rt_inline void pin_irq_hdr(int irqno)
{
    if (pin_irq_hdr_tab[irqno].hdr)
    {
        pin_irq_hdr_tab[irqno].hdr(pin_irq_hdr_tab[irqno].args);
    }
}

/**
  * @brief  This function handles EXTI interrupt request.
  * @param  exti_line: Specifies the pins connected EXTI line
  * @retval none
  */
void gd32_pin_exti_irqhandler(uint16_t exti_line)
{
    if(RESET != exti_interrupt_flag_get((exti_line_enum)(1 << exti_line)))
    {
        pin_irq_hdr(exti_line);
        exti_interrupt_flag_clear((exti_line_enum)(1 << exti_line));
    }
}

void EXTI0_IRQHandler(void)
{
    rt_interrupt_enter();
    gd32_pin_exti_irqhandler(GPIO_PIN_0);
    rt_interrupt_leave();
}

void EXTI1_IRQHandler(void)
{
    rt_interrupt_enter();
    gd32_pin_exti_irqhandler(GPIO_PIN_1);
    rt_interrupt_leave();
}

void EXTI2_IRQHandler(void)
{
    rt_interrupt_enter();
    gd32_pin_exti_irqhandler(GPIO_PIN_2);
    rt_interrupt_leave();
}

void EXTI3_IRQHandler(void)
{
    rt_interrupt_enter();
    gd32_pin_exti_irqhandler(GPIO_PIN_3);
    rt_interrupt_leave();
}

void EXTI4_IRQHandler(void)
{
    rt_interrupt_enter();
    gd32_pin_exti_irqhandler(GPIO_PIN_4);
    rt_interrupt_leave();
}

void EXTI5_9_IRQHandler(void)
{
    rt_interrupt_enter();
    gd32_pin_exti_irqhandler(GPIO_PIN_5);
    gd32_pin_exti_irqhandler(GPIO_PIN_6);
    gd32_pin_exti_irqhandler(GPIO_PIN_7);
    gd32_pin_exti_irqhandler(GPIO_PIN_8);
    gd32_pin_exti_irqhandler(GPIO_PIN_9);
    rt_interrupt_leave();
}

void EXTI10_15_IRQHandler(void)
{
    rt_interrupt_enter();
    gd32_pin_exti_irqhandler(GPIO_PIN_10);
    gd32_pin_exti_irqhandler(GPIO_PIN_11);
    gd32_pin_exti_irqhandler(GPIO_PIN_12);
    gd32_pin_exti_irqhandler(GPIO_PIN_13);
    gd32_pin_exti_irqhandler(GPIO_PIN_14);
    gd32_pin_exti_irqhandler(GPIO_PIN_15);
    rt_interrupt_leave();
}

int rt_hw_pin_init(void)
{
#if defined(GPIOG)
    rcu_periph_clock_enable(RCU_GPIOG);
#endif
#if defined(GPIOF)
    rcu_periph_clock_enable(RCU_GPIOF);
#endif
#if defined(GPIOE)
    rcu_periph_clock_enable(RCU_GPIOE);
#endif
#if defined(GPIOD)
    rcu_periph_clock_enable(RCU_GPIOD);
#endif
#if defined(GPIOC)
    rcu_periph_clock_enable(RCU_GPIOC);
#endif
#if defined(GPIOB)
    rcu_periph_clock_enable(RCU_GPIOB);
#endif
#if defined(GPIOA)
    rcu_periph_clock_enable(RCU_GPIOA);
#endif
    rcu_periph_clock_enable(RCU_AF);
    return rt_device_pin_register("pin", &_gd32_pin_ops, RT_NULL);
}
INIT_BOARD_EXPORT(rt_hw_pin_init);
#endif /* RT_USING_PIN */

