/*
 *
 * Copyright (C) 2007 IC Nexus Co., LTD.
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 */

/*
 * 2007/07/16: initial
 *
 *
 * Author: Hardy Weng <hardy.weng@icnexus.com.tw>
 *
 */


#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <asm/io.h>
#include <asm/cacheflush.h>
#include <asm/mach-nxc2600/nxc2600.h>
#include <asm/mach-nxc2600/gpio.h>
#include <linux/platform_device.h>

static const char gpio_name[] = "NXC2600_GPIO";

#define GPIO_PIN(_pin)                  ((_pin)*0x1000000)

#define GPIO_MODE_MASK                  (0x003f0000)
#define GPIO_MODE_INVERT_MASK		(0x00800000)
#define GPIO_DIR_MASK                   (0x00400000)
#define GPIO_FUNC_MASK                  (0x0000ffff)

typedef struct _GPIO_DRV GPIO_DRV;

struct _GPIO
{
        GPIO_DRV *drv;
        struct device *dev;
        uint32_t pin;
        uint32_t mode;
        uint32_t dir;
        uint32_t func;
	uint32_t inv;
};

struct _GPIO_DRV
{
        GPIO gpios[MAX_GPIOS];
	spinlock_t lock;
        uint8_t *regs;
        struct device           *dev;
        struct resource         *irq;
        struct resource         *ioarea;
};

static struct _GPIO_DRV *drv = NULL;

static void 
gpio_set_operation( GPIO *gpio )
{
	//int gp = gpio->pin/32;
	//int pin = gpio->pin%32;
	spin_lock_irq(&(gpio->drv->lock));
	switch(gpio->func)
	{
		case GPIO_FUNC_NORMAL:
			{

			uint32_t mode;	
			uint32_t dir = (gpio->dir&GPIO_DIR_IN);

			if(dir == GPIO_DIR_IN)
			{
				switch( gpio->mode&GPIO_MODE_MASK )
        			{
			                case GPIO_MODE_IRQ_LOW_LEVEL:
			                        mode = NXC2600_GPIO_IRQ_TRIGGER_LOW_LEVEL;
                        			break;
			                case GPIO_MODE_IRQ_HIGH_LEVEL:
                        			mode = NXC2600_GPIO_IRQ_TRIGGER_HIGH_LEVEL;
			                        break;
			                case GPIO_MODE_IRQ_FALLING:
			                        mode = NXC2600_GPIO_IRQ_TRIGGER_FALLING_EDGE;
			                        break;
			                case GPIO_MODE_IRQ_RISING:
			                        mode = NXC2600_GPIO_IRQ_TRIGGER_RISING_EDGE;
                        			break;
			                case GPIO_MODE_NORMAL:
			                default:
			                        mode = NXC2600_GPIO_IRQ_TRIGGER_NO_IRQ;
						break;
        				}

				nxc2600_set_gpio_input_mode( gpio->pin, 0, mode);
			}
			else
			{
				nxc2600_set_gpio_output_mode( gpio->pin, 0 );
			}

			}
			break;
		case GPIO_FUNC_CIM:
			nxc2600_gpio_vs_cim(0);
			break;
		case GPIO_FUNC_DMA:
			nxc2600_gpio_vs_dma(0);
			break;
		case GPIO_FUNC_UART3_CTSRTS:
			nxc2600_gpio_vs_uart3_ctsrts(0);
			break;
		case GPIO_FUNC_UART3:
			nxc2600_gpio_vs_uart3(0);
			break;
		case GPIO_FUNC_UART1:
			nxc2600_gpio_vs_uart1(0);
                        break;
		case GPIO_FUNC_OHCI_HCD:
			nxc2600_gpio_vs_ohci_hcd(0);
                        break;
		case GPIO_FUNC_PS2:
			nxc2600_gpio_vs_ps2(0);
                        break;
		case GPIO_FUNC_MSC:
			nxc2600_gpio_vs_msc(0);
			break;
		case GPIO_FUNC_PRT:
			//nxc2600_gpio_vs_prt(0);
			break;
		case GPIO_FUNC_LCD:
			nxc2600_gpio_vs_lcd(0);
			break;
		case GPIO_FUNC_SCC0:
			nxc2600_gpio_vs_scc0(0);
                        break;
		case GPIO_FUNC_SCC1:
			nxc2600_gpio_vs_scc1(0);
			break;
		case GPIO_FUNC_AC97:
			nxc2600_gpio_vs_ac97(0);
                        break;
		case GPIO_FUNC_SSI:
			nxc2600_gpio_vs_ssi(0);
                        break;
		case GPIO_FUNC_NAND:
			nxc2600_gpio_vs_nand(0);
			break;
		case GPIO_FUNC_DCS1:
			nxc2600_gpio_vs_dcs1(0);
                        break;
		case GPIO_FUNC_CS1:
			nxc2600_gpio_vs_cs1(0);
                        break;
                case GPIO_FUNC_CS2:
			nxc2600_gpio_vs_cs2(0);
                        break;
                case GPIO_FUNC_CS3:
			nxc2600_gpio_vs_cs3(0);
                        break;
                case GPIO_FUNC_CS4:
			nxc2600_gpio_vs_cs4(0);
                        break;
                case GPIO_FUNC_CS5:
			nxc2600_gpio_vs_cs5(0);
                        break;
                case GPIO_FUNC_PCMCIA:
			nxc2600_gpio_vs_pcmcia(0);
                        break;
		case GPIO_FUNC_PWM0:
			nxc2600_gpio_vs_pwm0(0);
                        break;
		case GPIO_FUNC_PWM1:
			nxc2600_gpio_vs_pwm1(0);
                        break;
		case GPIO_FUNC_UART2:
			nxc2600_gpio_vs_uart2(0);
                        break;
		case GPIO_FUNC_MAC:
			nxc2600_gpio_vs_mac(0);
                        break;
		case GPIO_FUNC_UART0:
			nxc2600_gpio_vs_uart0(0);
                        break;
		default:
			break;
	}
	spin_unlock_irq(&gpio->drv->lock);

}


GPIO* 
gpio_request( uint32_t pin,
	      uint32_t operation,
	      struct device *dev )
{
	uint32_t mode = operation & GPIO_MODE_MASK;
	uint32_t dir = operation & GPIO_DIR_MASK;
	uint32_t func = operation & GPIO_FUNC_MASK;
	uint32_t inv = operation & GPIO_MODE_INVERT_MASK;
	if( pin == GPIO_PIN_MASK ) pin = (operation&GPIO_PIN_MASK)/0x1000000;
	if( pin >= MAX_GPIOS ) return NULL;
	if( dev == NULL ) return NULL;
	spin_lock_irq(&(drv->lock));
	if( drv->gpios[pin].dev != NULL)
	{
		printk("gpio pin %d is in used.\n",pin);
		goto err;
	}

	if( func == GPIO_FUNC_NORMAL )
	{
		switch(dir)
		{
			case GPIO_DIR_IN:
			case GPIO_DIR_OUT:
				break;
			default: goto err;
		}
		switch( mode )
		{
			case GPIO_MODE_NORMAL:
				break;
			case GPIO_MODE_IRQ_LOW_LEVEL:
			case GPIO_MODE_IRQ_HIGH_LEVEL:
			case GPIO_MODE_IRQ_FALLING:
			case GPIO_MODE_IRQ_RISING:
				if( dir != GPIO_DIR_IN ) goto err;
				break;
			default: goto err;
		}
	}
	else
	{
#if 0
		switch( func = (GPIO_PIN(pin)|operation)&(GPIO_PIN_MASK|GPIO_FUNC_MASK) )
		{
			default:
				printk("gpio: not support func [%x] at this pin %d\n",func, pin);
				goto err;
		}
#else

#endif
	}

	drv->gpios[pin].dev = dev;
	drv->gpios[pin].mode = mode;
	drv->gpios[pin].dir = dir;
	drv->gpios[pin].func = func;
	drv->gpios[pin].inv = inv;
	spin_unlock_irq(&(drv->lock));
	gpio_set_operation(&(drv->gpios[pin]));
	return &(drv->gpios[pin]);
err:
	spin_unlock_irq(&(drv->lock));
	return NULL;
}

void 
gpio_release( GPIO *gpio )
{
	if( gpio == NULL ) return;
	if( gpio->pin >= MAX_GPIOS ) return;

	spin_lock_irq(&gpio->drv->lock);
	gpio->mode   = gpio->drv->gpios[gpio->pin].mode;
	gpio->dir    = gpio->drv->gpios[gpio->pin].dir;
	gpio->func   = gpio->drv->gpios[gpio->pin].func;
	spin_unlock_irq(&gpio->drv->lock);

	gpio_set_operation(gpio);
	gpio->dev = NULL;

}

int 
gpio_set_out( GPIO *gpio,
	      int hilow )
{

	if( gpio->dir == GPIO_DIR_IN )
	{
		gpio->dir = GPIO_DIR_OUT;
		gpio_set_operation(gpio);
	}

	hilow = (hilow)? 1:0;
	spin_lock_irq(&gpio->drv->lock);
	nxc2600_set_gpio_output(gpio->pin,hilow);
	spin_unlock_irq(&gpio->drv->lock);

	return 0;
}

int 
gpio_get_in( GPIO *gpio )
{
	uint32_t data = 0;

	if( gpio->dir == GPIO_DIR_OUT )
	{
		gpio->dir = GPIO_DIR_IN;
		gpio_set_operation(gpio);
	}
	spin_lock_irq(&gpio->drv->lock);
	data = nxc2600_get_gpio_input(gpio->pin);
	spin_unlock_irq(&gpio->drv->lock);
	return (data)? 1:0;
}

EXPORT_SYMBOL(gpio_set_out);
EXPORT_SYMBOL(gpio_get_in);
EXPORT_SYMBOL(gpio_release);
EXPORT_SYMBOL(gpio_request);

#define show_gpio(pin) \
static ssize_t show_gpio##pin(struct device *d, \
                              struct device_attribute *attr,  \
			      char *buf) \
{ \
	if( drv->gpios[ pin ].func != GPIO_FUNC_NORMAL ) return sprintf(buf, "-1"); \
	return sprintf(buf, "%d\n", gpio_get_in(&(drv->gpios[ pin ]))); \
}

show_gpio(0)
show_gpio(1)
show_gpio(2)
show_gpio(3)
show_gpio(4)
show_gpio(5)
show_gpio(6)
show_gpio(7)
show_gpio(8)
show_gpio(9)
show_gpio(10)
show_gpio(11)
show_gpio(12)
show_gpio(13)
show_gpio(16)
show_gpio(17)
show_gpio(21)
show_gpio(23)
show_gpio(24)
show_gpio(25)
show_gpio(26)
show_gpio(27)
show_gpio(28)
show_gpio(29)
show_gpio(32)
show_gpio(33)
show_gpio(34)
show_gpio(35)
show_gpio(36)
show_gpio(37)
show_gpio(38)
show_gpio(39)
show_gpio(40)
show_gpio(41)
show_gpio(42)
show_gpio(43)
show_gpio(44)
show_gpio(45)
show_gpio(46)
show_gpio(47)
show_gpio(48)
show_gpio(49)
show_gpio(50)
show_gpio(51)
show_gpio(52)
show_gpio(53)
show_gpio(54)
show_gpio(55)
show_gpio(56)
show_gpio(57)
show_gpio(58)
show_gpio(59)
show_gpio(60)
show_gpio(61)
show_gpio(62)
show_gpio(63)
show_gpio(64)
show_gpio(65)
show_gpio(66)
show_gpio(67)
show_gpio(68)
show_gpio(69)
show_gpio(70)
show_gpio(71)
show_gpio(72)
show_gpio(73)
show_gpio(74)
show_gpio(75)
show_gpio(76)
show_gpio(77)
show_gpio(78)
show_gpio(79)
show_gpio(80)
show_gpio(81)
show_gpio(82)
show_gpio(83)
show_gpio(84)
show_gpio(85)
show_gpio(86)
show_gpio(87)
show_gpio(88)
show_gpio(89)
show_gpio(90)
show_gpio(91)
show_gpio(92)
show_gpio(93)
show_gpio(94)
show_gpio(95)
show_gpio(96)
show_gpio(97)
show_gpio(98)
show_gpio(99)
show_gpio(100)
show_gpio(101)
show_gpio(102)
show_gpio(103)
show_gpio(104)
show_gpio(105)
show_gpio(106)
show_gpio(107)
show_gpio(108)
show_gpio(109)
show_gpio(110)
show_gpio(111)
show_gpio(112)
show_gpio(113)
show_gpio(114)
show_gpio(115)
show_gpio(116)
show_gpio(117)
show_gpio(118)
show_gpio(119)
show_gpio(120)
show_gpio(121)
show_gpio(122)
show_gpio(123)
show_gpio(124)
show_gpio(125)
show_gpio(126)
show_gpio(127)

#define store_gpio(pin) \
static ssize_t store_gpio##pin(struct device *d, \
                               struct device_attribute *attr, \
                               const char *buf, \
                               size_t count) \
{ \
	uint32_t val = simple_strtoul(buf, NULL, 10); \
	if( drv->gpios[ pin ].func != GPIO_FUNC_NORMAL ) return -1; \
	gpio_set_out(&(drv->gpios[ pin ]), val); \
	return count; \
}

store_gpio(0)
store_gpio(1)
store_gpio(2)
store_gpio(3)
store_gpio(4)
store_gpio(5)
store_gpio(6)
store_gpio(7)
store_gpio(8)
store_gpio(9)
store_gpio(10)
store_gpio(11)
store_gpio(12)
store_gpio(13)
store_gpio(16)
store_gpio(17)
store_gpio(21)
store_gpio(23)
store_gpio(24)
store_gpio(25)
store_gpio(26)
store_gpio(27)
store_gpio(28)
store_gpio(29)
store_gpio(32)
store_gpio(33)
store_gpio(34)
store_gpio(35)
store_gpio(36)
store_gpio(37)
store_gpio(38)
store_gpio(39)
store_gpio(40)
store_gpio(41)
store_gpio(42)
store_gpio(43)
store_gpio(44)
store_gpio(45)
store_gpio(46)
store_gpio(47)
store_gpio(48)
store_gpio(49)
store_gpio(50)
store_gpio(51)
store_gpio(52)
store_gpio(53)
store_gpio(54)
store_gpio(55)
store_gpio(56)
store_gpio(57)
store_gpio(58)
store_gpio(59)
store_gpio(60)
store_gpio(61)
store_gpio(62)
store_gpio(63)
store_gpio(64)
store_gpio(65)
store_gpio(66)
store_gpio(67)
store_gpio(68)
store_gpio(69)
store_gpio(70)
store_gpio(71)
store_gpio(72)
store_gpio(73)
store_gpio(74)
store_gpio(75)
store_gpio(76)
store_gpio(77)
store_gpio(78)
store_gpio(79)
store_gpio(80)
store_gpio(81)
store_gpio(82)
store_gpio(83)
store_gpio(84)
store_gpio(85)
store_gpio(86)
store_gpio(87)
store_gpio(88)
store_gpio(89)
store_gpio(90)
store_gpio(91)
store_gpio(92)
store_gpio(93)
store_gpio(94)
store_gpio(95)
store_gpio(96)
store_gpio(97)
store_gpio(98)
store_gpio(99)
store_gpio(100)
store_gpio(101)
store_gpio(102)
store_gpio(103)
store_gpio(104)
store_gpio(105)
store_gpio(106)
store_gpio(107)
store_gpio(108)
store_gpio(109)
store_gpio(110)
store_gpio(111)
store_gpio(112)
store_gpio(113)
store_gpio(114)
store_gpio(115)
store_gpio(116)
store_gpio(117)
store_gpio(118)
store_gpio(119)
store_gpio(120)
store_gpio(121)
store_gpio(122)
store_gpio(123)
store_gpio(124)
store_gpio(125)
store_gpio(126)
store_gpio(127)

static DEVICE_ATTR(gpio0, S_IRUSR|S_IWUSR, show_gpio0, store_gpio0);
static DEVICE_ATTR(gpio1, S_IRUSR|S_IWUSR, show_gpio1, store_gpio1);
static DEVICE_ATTR(gpio2, S_IRUSR|S_IWUSR, show_gpio2, store_gpio2);
static DEVICE_ATTR(gpio3, S_IRUSR|S_IWUSR, show_gpio3, store_gpio3);
static DEVICE_ATTR(gpio4, S_IRUSR|S_IWUSR, show_gpio4, store_gpio4);
static DEVICE_ATTR(gpio5, S_IRUSR|S_IWUSR, show_gpio5, store_gpio5);
static DEVICE_ATTR(gpio6, S_IRUSR|S_IWUSR, show_gpio6, store_gpio6);
static DEVICE_ATTR(gpio7, S_IRUSR|S_IWUSR, show_gpio7, store_gpio7);
static DEVICE_ATTR(gpio8, S_IRUSR|S_IWUSR, show_gpio8, store_gpio8);
static DEVICE_ATTR(gpio9, S_IRUSR|S_IWUSR, show_gpio9, store_gpio9);
static DEVICE_ATTR(gpio10, S_IRUSR|S_IWUSR, show_gpio10, store_gpio10);
static DEVICE_ATTR(gpio11, S_IRUSR|S_IWUSR, show_gpio11, store_gpio11);
static DEVICE_ATTR(gpio12, S_IRUSR|S_IWUSR, show_gpio12, store_gpio12);
static DEVICE_ATTR(gpio13, S_IRUSR|S_IWUSR, show_gpio13, store_gpio13);
static DEVICE_ATTR(gpio16, S_IRUSR|S_IWUSR, show_gpio16, store_gpio16);
static DEVICE_ATTR(gpio17, S_IRUSR|S_IWUSR, show_gpio17, store_gpio17);
static DEVICE_ATTR(gpio21, S_IRUSR|S_IWUSR, show_gpio21, store_gpio21);
static DEVICE_ATTR(gpio23, S_IRUSR|S_IWUSR, show_gpio23, store_gpio23);
static DEVICE_ATTR(gpio24, S_IRUSR|S_IWUSR, show_gpio24, store_gpio24);
static DEVICE_ATTR(gpio25, S_IRUSR|S_IWUSR, show_gpio25, store_gpio25);
static DEVICE_ATTR(gpio26, S_IRUSR|S_IWUSR, show_gpio26, store_gpio26);
static DEVICE_ATTR(gpio27, S_IRUSR|S_IWUSR, show_gpio27, store_gpio27);
static DEVICE_ATTR(gpio28, S_IRUSR|S_IWUSR, show_gpio28, store_gpio28);
static DEVICE_ATTR(gpio29, S_IRUSR|S_IWUSR, show_gpio29, store_gpio29);
static DEVICE_ATTR(gpio32, S_IRUSR|S_IWUSR, show_gpio32, store_gpio32);
static DEVICE_ATTR(gpio33, S_IRUSR|S_IWUSR, show_gpio33, store_gpio33);
static DEVICE_ATTR(gpio34, S_IRUSR|S_IWUSR, show_gpio34, store_gpio34);
static DEVICE_ATTR(gpio35, S_IRUSR|S_IWUSR, show_gpio35, store_gpio35);
static DEVICE_ATTR(gpio36, S_IRUSR|S_IWUSR, show_gpio36, store_gpio36);
static DEVICE_ATTR(gpio37, S_IRUSR|S_IWUSR, show_gpio37, store_gpio37);
static DEVICE_ATTR(gpio38, S_IRUSR|S_IWUSR, show_gpio38, store_gpio38);
static DEVICE_ATTR(gpio39, S_IRUSR|S_IWUSR, show_gpio39, store_gpio39);
static DEVICE_ATTR(gpio40, S_IRUSR|S_IWUSR, show_gpio40, store_gpio40);
static DEVICE_ATTR(gpio41, S_IRUSR|S_IWUSR, show_gpio41, store_gpio41);
static DEVICE_ATTR(gpio42, S_IRUSR|S_IWUSR, show_gpio42, store_gpio42);
static DEVICE_ATTR(gpio43, S_IRUSR|S_IWUSR, show_gpio43, store_gpio43);
static DEVICE_ATTR(gpio44, S_IRUSR|S_IWUSR, show_gpio44, store_gpio44);
static DEVICE_ATTR(gpio45, S_IRUSR|S_IWUSR, show_gpio45, store_gpio45);
static DEVICE_ATTR(gpio46, S_IRUSR|S_IWUSR, show_gpio46, store_gpio46);
static DEVICE_ATTR(gpio47, S_IRUSR|S_IWUSR, show_gpio47, store_gpio47);
static DEVICE_ATTR(gpio48, S_IRUSR|S_IWUSR, show_gpio48, store_gpio48);
static DEVICE_ATTR(gpio49, S_IRUSR|S_IWUSR, show_gpio49, store_gpio49);
static DEVICE_ATTR(gpio50, S_IRUSR|S_IWUSR, show_gpio50, store_gpio50);
static DEVICE_ATTR(gpio51, S_IRUSR|S_IWUSR, show_gpio51, store_gpio51);
static DEVICE_ATTR(gpio52, S_IRUSR|S_IWUSR, show_gpio52, store_gpio52);
static DEVICE_ATTR(gpio53, S_IRUSR|S_IWUSR, show_gpio53, store_gpio53);
static DEVICE_ATTR(gpio54, S_IRUSR|S_IWUSR, show_gpio54, store_gpio54);
static DEVICE_ATTR(gpio55, S_IRUSR|S_IWUSR, show_gpio55, store_gpio55);
static DEVICE_ATTR(gpio56, S_IRUSR|S_IWUSR, show_gpio56, store_gpio56);
static DEVICE_ATTR(gpio57, S_IRUSR|S_IWUSR, show_gpio57, store_gpio57);
static DEVICE_ATTR(gpio58, S_IRUSR|S_IWUSR, show_gpio58, store_gpio58);
static DEVICE_ATTR(gpio59, S_IRUSR|S_IWUSR, show_gpio59, store_gpio59);
static DEVICE_ATTR(gpio60, S_IRUSR|S_IWUSR, show_gpio60, store_gpio60);
static DEVICE_ATTR(gpio61, S_IRUSR|S_IWUSR, show_gpio61, store_gpio61);
static DEVICE_ATTR(gpio62, S_IRUSR|S_IWUSR, show_gpio62, store_gpio62);
static DEVICE_ATTR(gpio63, S_IRUSR|S_IWUSR, show_gpio63, store_gpio63);
static DEVICE_ATTR(gpio64, S_IRUSR|S_IWUSR, show_gpio64, store_gpio64);
static DEVICE_ATTR(gpio65, S_IRUSR|S_IWUSR, show_gpio65, store_gpio65);
static DEVICE_ATTR(gpio66, S_IRUSR|S_IWUSR, show_gpio66, store_gpio66);
static DEVICE_ATTR(gpio67, S_IRUSR|S_IWUSR, show_gpio67, store_gpio67);
static DEVICE_ATTR(gpio68, S_IRUSR|S_IWUSR, show_gpio68, store_gpio68);
static DEVICE_ATTR(gpio69, S_IRUSR|S_IWUSR, show_gpio69, store_gpio69);
static DEVICE_ATTR(gpio70, S_IRUSR|S_IWUSR, show_gpio70, store_gpio70);
static DEVICE_ATTR(gpio71, S_IRUSR|S_IWUSR, show_gpio71, store_gpio71);
static DEVICE_ATTR(gpio72, S_IRUSR|S_IWUSR, show_gpio72, store_gpio72);
static DEVICE_ATTR(gpio73, S_IRUSR|S_IWUSR, show_gpio73, store_gpio73);
static DEVICE_ATTR(gpio74, S_IRUSR|S_IWUSR, show_gpio74, store_gpio74);
static DEVICE_ATTR(gpio75, S_IRUSR|S_IWUSR, show_gpio75, store_gpio75);
static DEVICE_ATTR(gpio76, S_IRUSR|S_IWUSR, show_gpio76, store_gpio76);
static DEVICE_ATTR(gpio77, S_IRUSR|S_IWUSR, show_gpio77, store_gpio77);
static DEVICE_ATTR(gpio78, S_IRUSR|S_IWUSR, show_gpio78, store_gpio78);
static DEVICE_ATTR(gpio79, S_IRUSR|S_IWUSR, show_gpio79, store_gpio79);
static DEVICE_ATTR(gpio80, S_IRUSR|S_IWUSR, show_gpio80, store_gpio80);
static DEVICE_ATTR(gpio81, S_IRUSR|S_IWUSR, show_gpio81, store_gpio81);
static DEVICE_ATTR(gpio82, S_IRUSR|S_IWUSR, show_gpio82, store_gpio82);
static DEVICE_ATTR(gpio83, S_IRUSR|S_IWUSR, show_gpio83, store_gpio83);
static DEVICE_ATTR(gpio84, S_IRUSR|S_IWUSR, show_gpio84, store_gpio84);
static DEVICE_ATTR(gpio85, S_IRUSR|S_IWUSR, show_gpio85, store_gpio85);
static DEVICE_ATTR(gpio86, S_IRUSR|S_IWUSR, show_gpio86, store_gpio86);
static DEVICE_ATTR(gpio87, S_IRUSR|S_IWUSR, show_gpio87, store_gpio87);
static DEVICE_ATTR(gpio88, S_IRUSR|S_IWUSR, show_gpio88, store_gpio88);
static DEVICE_ATTR(gpio89, S_IRUSR|S_IWUSR, show_gpio89, store_gpio89);
static DEVICE_ATTR(gpio90, S_IRUSR|S_IWUSR, show_gpio90, store_gpio90);
static DEVICE_ATTR(gpio91, S_IRUSR|S_IWUSR, show_gpio91, store_gpio91);
static DEVICE_ATTR(gpio92, S_IRUSR|S_IWUSR, show_gpio92, store_gpio92);
static DEVICE_ATTR(gpio93, S_IRUSR|S_IWUSR, show_gpio93, store_gpio93);
static DEVICE_ATTR(gpio94, S_IRUSR|S_IWUSR, show_gpio94, store_gpio94);
static DEVICE_ATTR(gpio95, S_IRUSR|S_IWUSR, show_gpio95, store_gpio95);
static DEVICE_ATTR(gpio96, S_IRUSR|S_IWUSR, show_gpio96, store_gpio96);
static DEVICE_ATTR(gpio97, S_IRUSR|S_IWUSR, show_gpio97, store_gpio97);
static DEVICE_ATTR(gpio98, S_IRUSR|S_IWUSR, show_gpio98, store_gpio98);
static DEVICE_ATTR(gpio99, S_IRUSR|S_IWUSR, show_gpio99, store_gpio99);
static DEVICE_ATTR(gpio100, S_IRUSR|S_IWUSR, show_gpio100, store_gpio100);
static DEVICE_ATTR(gpio101, S_IRUSR|S_IWUSR, show_gpio101, store_gpio101);
static DEVICE_ATTR(gpio102, S_IRUSR|S_IWUSR, show_gpio102, store_gpio102);
static DEVICE_ATTR(gpio103, S_IRUSR|S_IWUSR, show_gpio103, store_gpio103);
static DEVICE_ATTR(gpio104, S_IRUSR|S_IWUSR, show_gpio104, store_gpio104);
static DEVICE_ATTR(gpio105, S_IRUSR|S_IWUSR, show_gpio105, store_gpio105);
static DEVICE_ATTR(gpio106, S_IRUSR|S_IWUSR, show_gpio106, store_gpio106);
static DEVICE_ATTR(gpio107, S_IRUSR|S_IWUSR, show_gpio107, store_gpio107);
static DEVICE_ATTR(gpio108, S_IRUSR|S_IWUSR, show_gpio108, store_gpio108);
static DEVICE_ATTR(gpio109, S_IRUSR|S_IWUSR, show_gpio109, store_gpio109);
static DEVICE_ATTR(gpio110, S_IRUSR|S_IWUSR, show_gpio110, store_gpio110);
static DEVICE_ATTR(gpio111, S_IRUSR|S_IWUSR, show_gpio111, store_gpio111);
static DEVICE_ATTR(gpio112, S_IRUSR|S_IWUSR, show_gpio112, store_gpio112);
static DEVICE_ATTR(gpio113, S_IRUSR|S_IWUSR, show_gpio113, store_gpio113);
static DEVICE_ATTR(gpio114, S_IRUSR|S_IWUSR, show_gpio114, store_gpio114);
static DEVICE_ATTR(gpio115, S_IRUSR|S_IWUSR, show_gpio115, store_gpio115);
static DEVICE_ATTR(gpio116, S_IRUSR|S_IWUSR, show_gpio116, store_gpio116);
static DEVICE_ATTR(gpio117, S_IRUSR|S_IWUSR, show_gpio117, store_gpio117);
static DEVICE_ATTR(gpio118, S_IRUSR|S_IWUSR, show_gpio118, store_gpio118);
static DEVICE_ATTR(gpio119, S_IRUSR|S_IWUSR, show_gpio119, store_gpio119);
static DEVICE_ATTR(gpio120, S_IRUSR|S_IWUSR, show_gpio120, store_gpio120);
static DEVICE_ATTR(gpio121, S_IRUSR|S_IWUSR, show_gpio121, store_gpio121);
static DEVICE_ATTR(gpio122, S_IRUSR|S_IWUSR, show_gpio122, store_gpio122);
static DEVICE_ATTR(gpio123, S_IRUSR|S_IWUSR, show_gpio123, store_gpio123);
static DEVICE_ATTR(gpio124, S_IRUSR|S_IWUSR, show_gpio124, store_gpio124);
static DEVICE_ATTR(gpio125, S_IRUSR|S_IWUSR, show_gpio125, store_gpio125);
static DEVICE_ATTR(gpio126, S_IRUSR|S_IWUSR, show_gpio126, store_gpio126);
static DEVICE_ATTR(gpio127, S_IRUSR|S_IWUSR, show_gpio127, store_gpio127);





static struct attribute *gpio_sysfs_entries[] = 
{
        &dev_attr_gpio0.attr,
	&dev_attr_gpio1.attr,
	&dev_attr_gpio2.attr,
	&dev_attr_gpio3.attr,
	&dev_attr_gpio4.attr,
	&dev_attr_gpio5.attr,
	&dev_attr_gpio6.attr,
	&dev_attr_gpio7.attr,
	&dev_attr_gpio8.attr,
	&dev_attr_gpio9.attr,
        &dev_attr_gpio10.attr,
        &dev_attr_gpio11.attr,
        &dev_attr_gpio12.attr,
        &dev_attr_gpio13.attr,
        &dev_attr_gpio16.attr,
        &dev_attr_gpio17.attr,
        &dev_attr_gpio21.attr,
        &dev_attr_gpio23.attr,
        &dev_attr_gpio24.attr,
        &dev_attr_gpio25.attr,
        &dev_attr_gpio26.attr,
        &dev_attr_gpio27.attr,
        &dev_attr_gpio28.attr,
        &dev_attr_gpio29.attr,
        &dev_attr_gpio32.attr,
        &dev_attr_gpio33.attr,
        &dev_attr_gpio34.attr,
        &dev_attr_gpio35.attr,
        &dev_attr_gpio36.attr,
        &dev_attr_gpio37.attr,
        &dev_attr_gpio38.attr,
        &dev_attr_gpio39.attr,
        &dev_attr_gpio40.attr,
        &dev_attr_gpio41.attr,
        &dev_attr_gpio42.attr,
        &dev_attr_gpio43.attr,
        &dev_attr_gpio44.attr,
        &dev_attr_gpio45.attr,
        &dev_attr_gpio46.attr,
        &dev_attr_gpio47.attr,
        &dev_attr_gpio48.attr,
        &dev_attr_gpio49.attr,
        &dev_attr_gpio50.attr,
        &dev_attr_gpio51.attr,
        &dev_attr_gpio52.attr,
        &dev_attr_gpio53.attr,
        &dev_attr_gpio54.attr,
        &dev_attr_gpio55.attr,
        &dev_attr_gpio56.attr,
        &dev_attr_gpio57.attr,
        &dev_attr_gpio58.attr,
        &dev_attr_gpio59.attr,
        &dev_attr_gpio60.attr,
        &dev_attr_gpio61.attr,
        &dev_attr_gpio62.attr,
        &dev_attr_gpio63.attr,
        &dev_attr_gpio64.attr,
        &dev_attr_gpio65.attr,
        &dev_attr_gpio66.attr,
        &dev_attr_gpio67.attr,
        &dev_attr_gpio68.attr,
        &dev_attr_gpio69.attr,
        &dev_attr_gpio70.attr,
        &dev_attr_gpio71.attr,
        &dev_attr_gpio72.attr,
        &dev_attr_gpio73.attr,
        &dev_attr_gpio74.attr,
        &dev_attr_gpio75.attr,
        &dev_attr_gpio76.attr,
        &dev_attr_gpio77.attr,
        &dev_attr_gpio78.attr,
        &dev_attr_gpio79.attr,
        &dev_attr_gpio80.attr,
        &dev_attr_gpio81.attr,
        &dev_attr_gpio82.attr,
        &dev_attr_gpio83.attr,
        &dev_attr_gpio84.attr,
        &dev_attr_gpio85.attr,
        &dev_attr_gpio86.attr,
        &dev_attr_gpio87.attr,
        &dev_attr_gpio88.attr,
        &dev_attr_gpio89.attr,
        &dev_attr_gpio90.attr,
        &dev_attr_gpio91.attr,
        &dev_attr_gpio92.attr,
        &dev_attr_gpio93.attr,
        &dev_attr_gpio94.attr,
        &dev_attr_gpio95.attr,
        &dev_attr_gpio96.attr,
        &dev_attr_gpio97.attr,
        &dev_attr_gpio98.attr,
        &dev_attr_gpio99.attr,
        &dev_attr_gpio100.attr,
        &dev_attr_gpio101.attr,
        &dev_attr_gpio102.attr,
        &dev_attr_gpio103.attr,
        &dev_attr_gpio104.attr,
        &dev_attr_gpio105.attr,
        &dev_attr_gpio106.attr,
        &dev_attr_gpio107.attr,
        &dev_attr_gpio108.attr,
        &dev_attr_gpio109.attr,
        &dev_attr_gpio110.attr,
        &dev_attr_gpio111.attr,
        &dev_attr_gpio112.attr,
        &dev_attr_gpio113.attr,
        &dev_attr_gpio114.attr,
        &dev_attr_gpio115.attr,
        &dev_attr_gpio116.attr,
        &dev_attr_gpio117.attr,
        &dev_attr_gpio118.attr,
        &dev_attr_gpio119.attr,
        &dev_attr_gpio120.attr,
        &dev_attr_gpio121.attr,
        &dev_attr_gpio122.attr,
        &dev_attr_gpio123.attr,
        &dev_attr_gpio124.attr,
        &dev_attr_gpio125.attr,
        &dev_attr_gpio126.attr,
        &dev_attr_gpio127.attr,
        NULL
};

static struct attribute_group gpio_attribute_group = {
        .name = gpio_name,
        .attrs = gpio_sysfs_entries,
};

static struct miscdevice gpio_miscdev =
{
        .minor = MISC_DYNAMIC_MINOR,
        .name = gpio_name,
};

static struct resource gpio_resource =
{
        .name = gpio_name,
        .start = CPHYSADDR(NXC2600_GPIO),
        .end   = CPHYSADDR(NXC2600_GPIO) + 0xc0 - 1,
        .flags = IORESOURCE_MEM
};

static struct platform_device gpio_device =
{
        .name  = gpio_name,
        .id    = 0,
        .num_resources = 1,
        .resource = &gpio_resource
};



static void 
gpio_free(GPIO_DRV *drv)
{
        if (drv->regs != NULL)
        {
                iounmap(drv->regs);
                drv->regs = NULL;
        }

        if (drv->ioarea != NULL)
	{
                release_resource(drv->ioarea);
                kfree(drv->ioarea);
                drv->ioarea = NULL;
        }
	kfree(drv);
	drv = NULL;
}

#define res_size(_r) (((_r)->end - (_r)->start) + 1)
static int 
gpio_probe(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct resource *res;
	int ret = 0;
	int i;

	if( drv != NULL )
	{
		printk(KERN_ERR "gpio: already initial.");
		return -EBUSY;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if (res == NULL) {
                printk(KERN_ERR "gpio: no IO resource.\n");
                ret = -ENOENT;
                goto out;
        }

	if( res->start != NXC2600_GPIO ) 
	{
		return -ENOENT;
	}

	drv = kmalloc(sizeof(GPIO_DRV), GFP_KERNEL);
	if(drv == NULL)
	{
		printk(KERN_ERR "gpio: out of memory.\n");
		return -ENOMEM;
	}
	memset(drv, 0, sizeof(*drv));
        drv->ioarea = request_mem_region(res->start, 
					  res_size(res),
                                          "gpio");

        if (drv->ioarea == NULL) {
                printk(KERN_ERR "gpio: request IO failed.\n");
                ret = -ENXIO;
                goto out;
        }

        drv->regs = (uint8_t*)ioremap(res->start,  res_size(res));

        if (drv->regs == NULL) {
                printk(KERN_ERR "gpio: map IO failed.\n");
                ret = -ENXIO;
                goto out;
        }


	spin_lock_init(&(drv->lock));

	for( i = 0; i < MAX_GPIOS; i++)
        {
		drv->gpios[i].dev    = NULL;
		drv->gpios[i].pin    = i;
		drv->gpios[i].drv    = drv;
		drv->gpios[i].mode   = GPIO_MODE_NORMAL;
		drv->gpios[i].func   = GPIO_FUNC_NORMAL;
		drv->gpios[i].dir    = GPIO_DIR_IN;
#ifdef CONFIG_NXC2620_CIM
		if(i>=0 && i<12)
		{
			drv->gpios[i].func = GPIO_FUNC_CIM;
			if(i!=0) continue;
		}
#endif
#ifdef CONFIG_NXC2600_DMA
		if(i==12 || i==13 || i==26 || i==27)
		{
			drv->gpios[i].func = GPIO_FUNC_DMA;
			if(i!=12) continue;
		}
#endif
#ifdef CONFIG_NXC2600_UART3
		if(i==16 || i==21
#ifdef CONFIG_NXC2600_UART3_CTSRTS
		   || i==17 || i==22
#endif 
			)
		{
			drv->gpios[i].func = GPIO_FUNC_UART3;
			if(i!=16) continue;
		}
#endif
#ifdef CONFIG_NXC2600_UART1
		if(i==24 || i==25)
		{
			drv->gpios[i].func = GPIO_FUNC_UART1;
			if(i!=24) continue;
		}
#endif
#ifdef CONFIG_USB_OHCI_HCD_NXC2600
		if(i==28 || i==29)
		{
			drv->gpios[i].func = GPIO_FUNC_OHCI_HCD;
			if(i!=28) continue;
		}
#endif
#ifdef CONFIG_NXC2600_PS2
		if(i==32 || i==33)
		{
			drv->gpios[i].func = GPIO_FUNC_PS2;
			if(i!=32) continue;
		}
#endif
#ifdef CONFIG_MMC_NXC2620
		if(i>=34 && i<40)
		{
			drv->gpios[i].func = GPIO_FUNC_MSC;
			if(i!=34) continue;
		}
#endif
#ifdef CONFIG_NXC2610_PRT
		if((i>=34 && i<40) || (i>=96 &&i<111))
		{
			drv->gpios[i].func = GPIO_FUNC_PRT;
			if(i!=34) continue;
		}
#endif
#ifdef CONFIG_FB_NXC2600
		if(i>=40 && i<64)
		{
			drv->gpios[i].func = GPIO_FUNC_LCD;
			if(i!=40) continue;
		}
#endif
#ifdef CONFIG_NXC2600_SCC0
		if(i==64 || i==66)
		{
			drv->gpios[i].func = GPIO_FUNC_SCC0;
			if(i!=64) continue;
		}
#endif
#ifdef CONFIG_NXC2600_SCC1
		if(i==65 || i==67)
		{
			 drv->gpios[i].func = GPIO_FUNC_SCC1;
			if(i!=65) continue;
		}
#endif
#ifdef CONFIG_SND_NXC2600_AC97
		if(i==69 || i==70 || i==71 || i==77 || i==78)
		{
			drv->gpios[i].func = GPIO_FUNC_AC97;
			if(i!=69) continue;
		}
#endif
#ifdef CONFIG_NXC2600_SSI
		if(i>=72 && i<77)
		{
			drv->gpios[i].func = GPIO_FUNC_SSI;
			if(i!=72) continue;
		}
#endif
#ifdef CONFIG_MTD_NAND_NXC2600
		if(i==79 || i==80 || i==81 || i==85)
		{
			drv->gpios[i].func = GPIO_FUNC_NAND;
			if(i!=79) continue;
		}
#endif
#ifdef CONFIG_NXC2600_DCS1
		if(i==82)
			drv->gpios[i].func = GPIO_FUNC_DCS1;
#endif
#if defined(CONFIG_NXC2600_CS1) || defined(CONFIG_MFD_SM501)
		if(i==83)
			drv->gpios[i].func = GPIO_FUNC_CS1;
#endif
#ifdef CONFIG_NXC2600_CS2
		if(i==84)
			drv->gpios[i].func = GPIO_FUNC_CS2;
#endif
#ifdef CONFIG_NXC2600_CS3
		if(i==85)
			drv->gpios[i].func = GPIO_FUNC_CS3;
#endif
#ifdef CONFIG_NXC2600_CS4
		if(i==86)
			drv->gpios[i].func = GPIO_FUNC_CS4;
#endif
#ifdef CONFIG_NXC2600_CS5
		if(i==87)
			drv->gpios[i].func = GPIO_FUNC_CS5;
#endif
#ifdef CONFIG_NXC2600_PCMCIA
		if(i>=88 && i<94 && i!=89)
		{
			drv->gpios[i].func = GPIO_FUNC_PCMCIA;
			if(i!=88) continue;
		}
#endif
#ifdef CONFIG_NXC2600_PWM0
		if(i==94)
			drv->gpios[i].func = GPIO_FUNC_PWM0;
#endif
#ifdef CONFIG_NXC2600_PWM1
		if(i==95)
			drv->gpios[i].func = GPIO_FUNC_PWM1;
#endif
#ifdef CONFIG_NXC2600_UART2
		if(i==111 || i==125)
		{
			drv->gpios[i].func = GPIO_FUNC_UART2;
			if(i!=111) continue;
		}
#endif
#ifdef CONFIG_NXC2620_MAC
		if(i>=112 && i<125)
		{
			drv->gpios[i].func = GPIO_FUNC_MAC;
			if(i!=112) continue;
		}
#endif
#ifdef CONFIG_NXC2600_UART0
		if(i==126 || i==127)
		{
			drv->gpios[i].func = GPIO_FUNC_UART0;
			if(i!=126) continue;
		}
#endif
		if(drv->gpios[i].func   != GPIO_FUNC_NORMAL)
		gpio_set_operation(&(drv->gpios[i]));
        }

	dev_set_drvdata(dev, drv);

	ret = misc_register(&gpio_miscdev);
	if(ret)
	{
		printk(KERN_ERR " misc_register retruns %d\n", ret);
                ret = -EBUSY;
                goto out;
	}

        ret = sysfs_create_group(&(pdev->dev.kobj),
                                 &gpio_attribute_group);
        if (ret)
	{
                printk(KERN_ERR "failed to create sysfs device attributes\n");
		misc_deregister(&gpio_miscdev);
                sysfs_remove_group(&(pdev->dev.kobj),
                                   &gpio_attribute_group);
                goto out;
        }

	printk("NXC2600 gpio initialized\n");
out:
	if(ret < 0) gpio_free(drv);
	return ret;
}

static int 
gpio_remove(struct device *dev)
{
	GPIO_DRV *gpio = dev_get_drvdata(dev);
	gpio_free(gpio);
	dev_set_drvdata(dev, NULL);
	return 0;
}

static struct device_driver 
gpio_driver = 
{
        .name           = gpio_name,
        .bus            = &platform_bus_type,
        .probe          = gpio_probe,
        .remove         = gpio_remove,
};


static int __init 
gpio_init(void)
{
	int err;
        err = driver_register(&gpio_driver);
	if(err) return err;

        platform_device_register(&gpio_device);

	return 0;
}

static void __exit 
gpio_exit(void)
{
	platform_device_unregister(&gpio_device);
        driver_unregister(&gpio_driver);
}


module_init(gpio_init);
module_exit(gpio_exit);

MODULE_DESCRIPTION("NXC2600 GPIO driver");
MODULE_AUTHOR("Hardy Weng <hardy.weng@icnexus.com.tw>");
MODULE_LICENSE("GPL");
