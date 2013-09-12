/*
 * MCP2210 software IRQ controller
 *
 * Copyright (c) 2013 Daniel Santos <daniel.santos@pobox.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */


#include <linux/irqdomain.h>
//#include <linux/gpio.h>
#include <linux/irq.h>

#include "mcp2210.h"
#include "mcp2210-debug.h"


#ifdef CONFIG_MCP2210_IRQ

static int complete_poll(struct mcp2210_cmd *cmd, void *context);
static int mcp2210_irq_map(struct irq_domain *domain, unsigned int irq,
			   irq_hw_number_t hwirq);

static const struct irq_domain_ops mcp2210_irq_domain_ops = {
	.map	= mcp2210_irq_map,
	.xlate	= irq_domain_xlate_twocell,
};


static inline struct mcp2210_device *to_dev(struct gpio_chip *chip)
{
	return container_of(chip, struct mcp2210_device, gpio);
}


int mcp2210_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	struct mcp2210_device *dev = to_dev(chip);
	return irq_create_mapping(dev->irq_domain, offset);
}

static void mcp2210_irq_mask(struct irq_data *data)
{
	struct mcp2210_device *dev = irq_data_get_irq_chip_data(data);

	mcp2210_debug();
}

static void mcp2210_irq_unmask(struct irq_data *data)
{
	struct mcp2210_device *dev = irq_data_get_irq_chip_data(data);

	mcp2210_debug();
}

static int mcp2210_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct mcp2210_device *dev = irq_data_get_irq_chip_data(data);

	mcp2210_debug();
#if 0
	if (type & IRQ_TYPE_EDGE_RISING)
		adnp->irq_rise[reg] |= BIT(pos);
	else
		adnp->irq_rise[reg] &= ~BIT(pos);

	if (type & IRQ_TYPE_EDGE_FALLING)
		adnp->irq_fall[reg] |= BIT(pos);
	else
		adnp->irq_fall[reg] &= ~BIT(pos);

	if (type & IRQ_TYPE_LEVEL_HIGH)
		adnp->irq_high[reg] |= BIT(pos);
	else
		adnp->irq_high[reg] &= ~BIT(pos);

	if (type & IRQ_TYPE_LEVEL_LOW)
		adnp->irq_low[reg] |= BIT(pos);
	else
		adnp->irq_low[reg] &= ~BIT(pos);
#endif
	return 0;
}

static void mcp2210_irq_bus_lock(struct irq_data *data)
{
	struct mcp2210_device *dev = irq_data_get_irq_chip_data(data);

	mcp2210_debug();

	mutex_lock(&dev->irq_lock);
}

static void mcp2210_irq_bus_unlock(struct irq_data *data)
{
	struct mcp2210_device *dev = irq_data_get_irq_chip_data(data);

	mcp2210_debug();
	mutex_unlock(&dev->irq_lock);
}

static struct irq_chip mcp2210_irq_chip = {
	.name		     = "gpio-mcp2210",
	.irq_mask	     = mcp2210_irq_mask,
	.irq_unmask	     = mcp2210_irq_unmask,
	.irq_set_type	     = mcp2210_irq_set_type,
	.irq_bus_lock	     = mcp2210_irq_bus_lock,
	.irq_bus_sync_unlock = mcp2210_irq_bus_unlock,
};

int mcp2210_irq_probe(struct mcp2210_device *dev)
{
	uint i;
	mutex_init(&dev->irq_lock);

	dev->nr_irqs = 0;
	for (i = 0; i < 9; ++i) {
		struct mcp2210_pin_config *pin = &dev->config->pins[i];

		if (!pin->has_irq)
			continue;

		switch (pin->mode) {
		case MCP2210_PIN_DEDICATED:
			if (i != 6)
				return -EINVAL;

			dev->s.poll_intr = 1;
			break;

		case MCP2210_PIN_GPIO:
			dev->s.poll_gpio = 1;
			break;
		};

		if (pin->irq + 1> dev->nr_irqs)
			dev->nr_irqs = pin->irq + 1;
	}

	BUG_ON(dev->nr_irqs > 7);


#ifdef CONFIG_MCP2210_GPIO
	if (dev->s.poll_gpio) {
		ctl_cmd_init(dev, &dev->cmd_poll_gpio,
			     MCP2210_CMD_GET_PIN_VALUE, 0, NULL, 0, false);
		dev->cmd_poll_gpio.head.complete = complete_poll;
		mcp2210_add_cmd(&dev->cmd_poll_gpio.head, false);
	}
#endif /* CONFIG_MCP2210_GPIO */

	if (dev->s.poll_intr) {
		ctl_cmd_init(dev, &dev->cmd_poll_intr,
			     MCP2210_CMD_GET_INTERRUPTS, 0, NULL, 0, false);
		dev->cmd_poll_intr.head.complete = complete_poll;
		mcp2210_add_cmd(&dev->cmd_poll_intr.head, false);
	}

	return 0;
}

void mcp2210_irq_remove(struct mcp2210_device *dev)
{
}

void _mcp2210_irq_do_gpio(struct mcp2210_device *dev, u16 old_val, u16 new_val)
{
}

void _mcp2210_irq_do_intr_counter(struct mcp2210_device *dev, u16 count)
{
	if (dev->s.chip_settings.pin_mode[6] != MCP2210_PIN_DEDICATED)
		return;
}

static int mcp2210_irq_map(struct irq_domain *domain, unsigned int irq,
			   irq_hw_number_t hwirq)
{
	irq_set_chip_data(irq, domain->host_data);
	irq_set_chip(irq, &mcp2210_irq_chip);
	irq_set_nested_thread(irq, true);

#ifdef CONFIG_ARM
	set_irq_flags(irq, IRQF_VALID);
#else
	irq_set_noprobe(irq);
#endif

	return 0;
}


static int complete_poll(struct mcp2210_cmd *cmd_head, void *context)
{
	struct mcp2210_device *dev = cmd_head->dev;
	struct mcp2210_cmd_ctl *cmd = (void*)cmd_head;
	int enabled;
	unsigned long interval;
	unsigned long now = jiffies;

	mcp2210_debug();

	if (dev->dead)
		return -EINPROGRESS;

	if (cmd->req.cmd == MCP2210_CMD_GET_PIN_VALUE) {
		enabled = dev->s.poll_gpio;
		interval = dev->config->poll_gpio_usecs;
		dev->s.last_poll_gpio = now;
	} else {
		enabled = dev->s.poll_intr;
		interval = dev->config->poll_intr_usecs;
		dev->s.last_poll_intr = now;
	}

	if (!enabled)
		return -EINPROGRESS;

	if (1) {
		unsigned long interval_j = usecs_to_jiffies(interval);
		unsigned long next = dev->eps[EP_OUT].submit_time + interval_j;
		cmd->head.delayed = 1;

		if (jiffdiff(next, now) < 0) {
			mcp2210_warn("poll interval collapse, restarting universe");
			next = jiffies + interval_j;
		}
		cmd->head.delay_until = next;

#if 0
		mcp2210_debug("interval_j: %lu, submit_time: %lu, next: %lu, jiffies: %lu",
			      interval_j, dev->eps[EP_OUT].submit_time, next, jiffies);
#endif
	}

	cmd->head.state = MCP2210_STATE_NEW;

	mcp2210_add_cmd(cmd_head, false);

	return -EINPROGRESS; /* tell process_commands not to free us */
}

#endif /* CONFIG_MCP2210_IRQ */
