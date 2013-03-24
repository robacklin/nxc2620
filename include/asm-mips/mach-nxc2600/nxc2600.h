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
 *   NXC2600 Registers and Operations
 *
 * 2007/07/16: initial
 *
 *
 * Author: Hardy Weng <hardy.weng@icnexus.com.tw>
 *
 */

#ifndef _NXC2600_H_
#define _NXC2600_H_


#ifndef CONFIG_NXC2600_CPU_CLOCK
#define CONFIG_NXC2600_CPU_CLOCK 336000000
#endif

#ifndef CONFIG_NXC2600_EXTAL_CLOCK
#define CONFIG_NXC2600_EXTAL_CLOCK	3686400
#endif

#ifndef IO_BASE
#define IO_BASE	(0xa0000000)
#endif

#ifndef CPHYSADDR
#define	CPHYSADDR(addr)		(((uint32_t)(addr))&0x1fffffff)
#endif

#ifndef uint32_t
#define uint32_t	unsigned int
#endif

#ifndef uint16_t
#define	uint16_t	unsigned short
#endif

#ifndef uint8_t
#define uint8_t		unsigned char
#endif

#define REG32(addr)				(*((volatile uint32_t*)(((uint32_t)(addr))|IO_BASE)))
#define REG16(addr)				(*((volatile uint16_t*)(((uint32_t)(addr))|IO_BASE)))
#define REG8(addr)				(*((volatile uint8_t*)(((uint32_t)(addr))|IO_BASE)))


#define NXC2600_IRQ_I2C				(1)
#define NXC2600_IRQ_KBC				(2)
#define NXC2600_IRQ_UPRT			(3)
#define NXC2600_IRQ_UART3			(6)
#define NXC2600_IRQ_UART2			(7)
#define NXC2600_IRQ_UART1			(8)
#define NXC2600_IRQ_UART0			(9)
#define NXC2600_IRQ_SCC1			(10)
#define NXC2600_IRQ_SCC0			(11)
#define NXC2600_IRQ_UDC				(12)
#define NXC2600_IRQ_UHC				(13)
#define NXC2600_IRQ_MSC				(14)
#define NXC2600_IRQ_RTC				(15)
#define NXC2600_IRQ_SSI				(17)
#define NXC2600_IRQ_CIM				(18)
#define NXC2600_IRQ_ETH				(19)
#define NXC2600_IRQ_AIC				(20)
#define NXC2600_IRQ_DMAC			(21)
#define NXC2600_IRQ_OST2			(22)
#define NXC2600_IRQ_OST1			(23)
#define NXC2600_IRQ_OST0			(24)
#define NXC2600_IRQ_GPIO_G3			(25)
#define NXC2600_IRQ_GPIO_G2			(26)
#define NXC2600_IRQ_GPIO_G1			(27)
#define NXC2600_IRQ_GPIO_G0			(28)
#define NXC2600_IRQ_LCD				(30)



#define	NXC2600_INTC_SR				(0x10001000)
#define		NXC2600_INTC_SR_I2C			(0x00000002)
#define         NXC2600_INTC_SR_KBC                     (0x00000004)
#define         NXC2600_INTC_SR_UPRT                    (0x00000008)
#define         NXC2600_INTC_SR_UART3                   (0x00000040)
#define         NXC2600_INTC_SR_UART2                   (0x00000080)
#define         NXC2600_INTC_SR_UART1                   (0x00000100)
#define         NXC2600_INTC_SR_UART0                   (0x00000200)
#define         NXC2600_INTC_SR_SCC1                    (0x00000400)
#define         NXC2600_INTC_SR_SCC0                    (0x00000800)
#define         NXC2600_INTC_SR_UDC                     (0x00001000)
#define         NXC2600_INTC_SR_UHC                     (0x00002000)
#define         NXC2600_INTC_SR_MSC                     (0x00004000)
#define         NXC2600_INTC_SR_RTC                     (0x00008000)
#define         NXC2600_INTC_SR_SSI                     (0x00020000)
#define         NXC2600_INTC_SR_CIM                     (0x00040000)
#define         NXC2600_INTC_SR_ETH                     (0x00080000)
#define         NXC2600_INTC_SR_AIC                     (0x00100000)
#define         NXC2600_INTC_SR_DMAC                    (0x00200000)
#define         NXC2600_INTC_SR_OST2                    (0x00400000)
#define         NXC2600_INTC_SR_OST1                    (0x00800000)
#define         NXC2600_INTC_SR_OST0                    (0x01000000)
#define         NXC2600_INTC_SR_GPIO3                   (0x02000000)
#define         NXC2600_INTC_SR_GPIO2                   (0x04000000)
#define         NXC2600_INTC_SR_GPIO1                   (0x08000000)
#define         NXC2600_INTC_SR_GPIO0                   (0x10000000)
#define         NXC2600_INTC_SR_LCD                     (0x40000000)
#define NXC2600_INTC_PR				(0x10001010)
#define 	NXC2600_INTC_PR_I2C			(0x00000002)
#define 	NXC2600_INTC_PR_KBC			(0x00000004)
#define 	NXC2600_INTC_PR_UPRT			(0x00000008)
#define 	NXC2600_INTC_PR_UART3			(0x00000040)
#define 	NXC2600_INTC_PR_UART2			(0x00000080)
#define 	NXC2600_INTC_PR_UART1			(0x00000100)
#define 	NXC2600_INTC_PR_UART0			(0x00000200)
#define 	NXC2600_INTC_PR_SCC1			(0x00000400)
#define 	NXC2600_INTC_PR_SCC0			(0x00000800)
#define 	NXC2600_INTC_PR_UDC			(0x00001000)
#define 	NXC2600_INTC_PR_UHC			(0x00002000)
#define 	NXC2600_INTC_PR_MSC			(0x00004000)
#define 	NXC2600_INTC_PR_RTC			(0x00008000)
#define 	NXC2600_INTC_PR_SSI			(0x00020000)
#define 	NXC2600_INTC_PR_CIM			(0x00040000)
#define 	NXC2600_INTC_PR_ETH			(0x00080000)
#define 	NXC2600_INTC_PR_AIC			(0x00100000)
#define 	NXC2600_INTC_PR_DMAC			(0x00200000)
#define 	NXC2600_INTC_PR_OST2			(0x00400000)
#define 	NXC2600_INTC_PR_OST1			(0x00800000)
#define 	NXC2600_INTC_PR_OST0			(0x01000000)
#define 	NXC2600_INTC_PR_GPIO3			(0x02000000)
#define 	NXC2600_INTC_PR_GPIO2			(0x04000000)
#define 	NXC2600_INTC_PR_GPIO1			(0x08000000)
#define 	NXC2600_INTC_PR_GPIO0			(0x10000000)
#define 	NXC2600_INTC_PR_LCD			(0x40000000)
#define NXC2600_INTC_MR				(0x10001004)
#define NXC2600_INTC_MSR			(0x10001008)
#define NXC2600_INTC_MCR			(0x1000100c)

#define NXC2600_SBA_CNTL			(0x13000000)
#define		NXC2600_SBA_CNTL_USB_PORT0_HOST			(0x00000080)
#define		NXC2600_SBA_CNTL_WT_MASK			(0x0000000f)
#define		NXC2600_SBA_CNTL_WT_CIM_LCDC_DMAC_ETHC_UHC_CPU	(0x00000000)
#define		NXC2600_SBA_CNTL_WT_CIM_LCDC_ETHC_DMAC_UHC_CPU	(0x00000001)
#define		NXC2600_SBA_CNTL_WT_CIM_LCDC_UHC_DMAC_ETHC_CPU	(0x00000002)
#define		NXC2600_SBA_CNTL_WT_LCDC_CIM_DMAC_ETHC_UHC_CPU	(0x00000003)
#define		NXC2600_SBA_CNTL_WT_LCDC_CIM_ETHC_DMAC_UHC_CPU	(0x00000004)
#define		NXC2600_SBA_CNTL_WT_LCDC_CIM_UHC_DMAC_ETHC_CPU	(0x00000005)
#define		NXC2600_SBA_CNTL_WT_DMAC_CIM_ETHC_LCDC_UHC_CPU	(0x00000006)
#define		NXC2600_SBA_CNTL_WT_DMAC_CIM_UHC_LCDC_ETHC_CPU	(0x00000007)
#define		NXC2600_SBA_CNTL_WT_DMAC_LCDC_CIM_ETHC_UHC_CPU	(0x00000008)
#define		NXC2600_SBA_CNTL_WT_ETHC_CIM_LCDC_DMAC_UHC_CPU	(0x00000009)
#define		NXC2600_SBA_CNTL_WT_ETHC_LCDC_CIM_DMAC_UHC_CPU	(0x0000000a)
#define		NXC2600_SBA_CNTL_WT_ETHC_UHC_CIM_LCDC_DMAC_CPU	(0x0000000b)
#define		NXC2600_SBA_CNTL_WT_UHC_CIM_LCDC_DMAC_ETHC_CPU	(0x0000000c)
#define		NXC2600_SBA_CNTL_WT_UHC_LCDC_CIM_DMAC_ETHC_CPU	(0x0000000d)
#define		NXC2600_SBA_CNTL_WT_CPU_CIM_LCDC_DMAC_ETHC_UHC	(0x0000000e)
#define		NXC2600_SBA_CNTL_WT_CPU_LCDC_CIM_DMAC_ETHC_UHC	(0x0000000f)

#define NXC2600_GPIO_BASE_IRQ(gp)		(32+32*(gp))
#define NXC2600_GPIO				(0x10010000)
#define NXC2600_GPIO_DATA(gp)			(0x10010000+0x30*(gp))
#define NXC2600_GPIO_DIR(gp)			(0x10010004+0x30*(gp))
#define NXC2600_GPIO_PE(gp)			(0x1001000c+0x30*(gp))
#define NXC2600_GPIO_ITL(gp)			(0x10010018+0x30*(gp))
#define NXC2600_GPIO_ITH(gp)			(0x1001001c+0x30*(gp))
#define NXC2600_GPIO_IE(gp)			(0x10010020+0x30*(gp))
#define NXC2600_GPIO_IM(gp)			(0x10010024+0x30*(gp))
#define NXC2600_GPIO_IRQ(gp)			(0x10010028+0x30*(gp))
#define NXC2600_GPIO_SL(gp)			(0x10010010+0x30*(gp))
#define NXC2600_GPIO_SH(gp)			(0x10010014+0x30*(gp))
#define		NXC2600_GPIO_SL_CIM_MASK		(0x00ffffff)
#define 	NXC2600_GPIO_SL_CIM			(0x00555555)
#define		NXC2600_GPIO_SL_DMA_MASK		(0x0f000000)
#define 	NXC2600_GPIO_SL_DMA			(0x05000000)
#define		NXC2600_GPIO_SH_DMA_MASK		(0x00f00000)
#define 	NXC2600_GPIO_SH_DMA			(0x00500000)
#define		NXC2600_GPIO_SH_UART3_MASK		(0x00000c03)
#define 	NXC2600_GPIO_SH_UART3			(0x00000401)
#define		NXC2600_GPIO_SH_UART3_CTSRTS_MASK	(0x0000c00c)
#define 	NXC2600_GPIO_SH_UART3_CTSRTS		(0x00004004)
#define		NXC2600_GPIO_SH_UART1_MASK		(0x000f0000)
#define 	NXC2600_GPIO_SH_UART1			(0x00050000)
#define		NXC2600_GPIO_SH_OHCI_HCD_MASK		(0x0f000000)
#define 	NXC2600_GPIO_SH_OHCI_HCD		(0x05000000)
#define		NXC2600_GPIO_SL_PS2_MASK		(0x0000000f)
#define 	NXC2600_GPIO_SL_PS2			(0x00000005)
#define		NXC2620_GPIO_SL_MSC_MASK		(0x0000fff0)
#define 	NXC2620_GPIO_SL_MSC			(0x00005550)
#define		NXC2610_GPIO_SL_PRT_1_MASK		(0x0000fff0)
#define 	NXC2610_GPIO_SL_PRT_1			(0x00005550)
#define		NXC2610_GPIO_SL_PRT_2_MASK		(0x3fffffff)
#define 	NXC2610_GPIO_SL_PRT_2			(0x15555555)
#define		NXC2600_GPIO_SL_LCD_MASK		(0xffff0000)
#define 	NXC2600_GPIO_SL_LCD			(0x55550000)
#define		NXC2600_GPIO_SH_LCD_MASK		(0xffffffff)

#define 	NXC2600_GPIO_SH_LCD			(0x556a5555)
#define		NXC2600_GPIO_SL_SCC0_MASK		(0x00000033)
#define 	NXC2600_GPIO_SL_SCC0			(0x00000011)
#define		NXC2600_GPIO_SL_SCC1_MASK		(0x000000cc)
#define 	NXC2600_GPIO_SL_SCC1			(0x00000044)
#define		NXC2600_GPIO_SL_AC97_MASK		(0x3c00fc00)
#define 	NXC2600_GPIO_SL_AC97			(0x24005400)
#define		NXC2600_GPIO_SL_I2S_MASTER_MASK		(0x3c00f300)
#define		NXC2600_GPIO_SL_I2S_MASTER		(0x28005100)
#define		NXC2600_GPIO_SL_I2S_SLAVE_MASK		(0x3c00f300)
#define		NXC2600_GPIO_SL_I2S_SLAVE		(0x14005100)
#define		NXC2600_GPIO_SL_SSI_MASK		(0x03ff0000)
#define 	NXC2600_GPIO_SL_SSI			(0x01550000)
#define		NXC2600_GPIO_SL_NAND_MASK		(0xc0000000)
#define 	NXC2600_GPIO_SL_NAND			(0x40000000)
#define		NXC2600_GPIO_SH_NAND_MASK		(0x0000000f)
#define 	NXC2600_GPIO_SH_NAND			(0x00000005)
#define		NXC2600_GPIO_SH_DCS1_MASK		(0x00000030)
#define 	NXC2600_GPIO_SH_DCS1			(0x00000010)
#define		NXC2600_GPIO_SH_CS1_MASK		(0x000000c0)
#define 	NXC2600_GPIO_SH_CS1			(0x00000040)
#define		NXC2600_GPIO_SH_CS2_MASK		(0x00000300)
#define 	NXC2600_GPIO_SH_CS2			(0x00000100)
#define		NXC2600_GPIO_SH_CS3_MASK		(0x00000c00)
#define 	NXC2600_GPIO_SH_CS3			(0x00000400)
#define		NXC2600_GPIO_SH_CS4_MASK		(0x00003000)
#define 	NXC2600_GPIO_SH_CS4			(0x00001000)
#define		NXC2600_GPIO_SH_CS5_MASK		(0x0000c000)
#define 	NXC2600_GPIO_SH_CS5			(0x00004000)
#ifdef	CONFIG_NXC2600_PCMCIA_USE_BVD2
#define	NXC2600_GPIO_BVD2	(0x00040000)
#define	NXC2600_GPIO_BVD2_MASK	(0x000c0000)
#else
#define	NXC2600_GPIO_BVD2	(0x00000000)
#define	NXC2600_GPIO_BVD2_MASK	(0x00000000)
#endif
#ifdef	CONFIG_NXC2600_PCMCIA_USE_PCE1
#define	NXC2600_GPIO_PCE1	(0x00100000)
#define	NXC2600_GPIO_PCE1_MASK	(0x00300000)
#else
#define	NXC2600_GPIO_PCE1	(0x00000000)
#define	NXC2600_GPIO_PCE1_MASK	(0x00000000)
#endif
#ifdef  CONFIG_NXC2600_PCMCIA_USE_PCE2
#define NXC2600_GPIO_PCE2       (0x04000000)
#define	NXC2600_GPIO_PCE2_MASK	(0x0c000000)
#else
#define NXC2600_GPIO_PCE2       (0x00000000)
#define NXC2600_GPIO_PCE2_MASK  (0x00000000)
#endif
#ifdef	CONFIG_NXC2600_PCMCIA_USE_INPACK
#define	NXC2600_GPIO_INPACK	(0x00010000)
#define	NXC2600_GPIO_INPACK_MASK (0x00030000)
#else
#define	NXC2600_GPIO_INPACK	(0x00000000)
#define	NXC2600_GPIO_INPACK_MASK (0x00000000)
#endif
#ifdef	CONFIG_NXC2600_PCMCIA_USE_IOIS16
#define	NXC2600_GPIO_IOIS16	(0x01000000)
#define	NXC2600_GPIO_IOIS16_MASK (0x03000000)
#else
#define	NXC2600_GPIO_IOIS16	(0x00000000)
#define	NXC2600_GPIO_IOIS16_MASK (0x00000000)
#endif
#ifdef CONFIG_NXC2600_PCMCIA_USE_PSKTSEL
#define	NXC2600_GPIO_PSKTSEL	(0x00400000)
#define	NXC2600_GPIO_PSKTSEL_MASK (0x00c00000)
#else
#define	NXC2600_GPIO_PSKTSEL	(0x00000000)
#define	NXC2600_GPIO_PSKTSEL_MASK (0x00000000)
#endif
#define		NXC2600_GPIO_SH_PCMCIA_MASK		(NXC2600_GPIO_PSKTSEL_MASK|NXC2600_GPIO_IOIS16_MASK|NXC2600_GPIO_INPACK_MASK|NXC2600_GPIO_PCE2_MASK|NXC2600_GPIO_PCE1_MASK|NXC2600_GPIO_BVD2_MASK)
#define 	NXC2600_GPIO_SH_PCMCIA			(NXC2600_GPIO_PSKTSEL|NXC2600_GPIO_IOIS16|NXC2600_GPIO_INPACK|NXC2600_GPIO_PCE2|NXC2600_GPIO_PCE1|NXC2600_GPIO_BVD2)
#define		NXC2600_GPIO_SH_PWM0_MASK		(0x30000000)
#define 	NXC2600_GPIO_SH_PWM0			(0x10000000)
#define		NXC2600_GPIO_SH_PWM1_MASK		(0xc0000000)
#define 	NXC2600_GPIO_SH_PWM1			(0x40000000)
#define		NXC2600_GPIO_SL_UART2_MASK		(0xc0000000)
#define 	NXC2600_GPIO_SL_UART2			(0x40000000)
#define		NXC2600_GPIO_SH_UART2_MASK		(0x0c000000)
#define 	NXC2600_GPIO_SH_UART2			(0x04000000)
#define		NXC2600_GPIO_SH_MAC_MASK		(0x03ffffff)
#define 	NXC2600_GPIO_SH_MAC			(0x01555555)
#define		NXC2600_GPIO_SH_UART0_MASK		(0xf0000000)
#define 	NXC2600_GPIO_SH_UART0			(0x50000000)
#define NXC2600_GPIO_SL0_DEFAULT		(NXC2600_GPIO_SL_DMA)
#define NXC2600_GPIO_SH0_DEFAULT		(NXC2600_GPIO_SH_DMA|NXC2600_GPIO_SH_UART3)
#define NXC2600_GPIO_SL1_DEFAULT		(NXC2600_GPIO_SL_LCD)
#define NXC2600_GPIO_SH1_DEFAULT		(NXC2600_GPIO_SH_LCD)
#define NXC2600_GPIO_SL2_DEFAULT		(NXC2600_GPIO_SL_NAND)
#define NXC2600_GPIO_SH2_DEFAULT		(NXC2600_GPIO_SH_NAND|NXC2600_GPIO_SH_DCS1|NXC2600_GPIO_SH_CS3)
#define NXC2600_GPIO_SL3_DEFAULT		(0)
#define NXC2600_GPIO_SH3_DEFAULT		(NXC2600_GPIO_SH_MAC|NXC2600_GPIO_SH_UART0)

#define NXC2600_GPIO_NIRQ(gpio)		(NXC2600_GPIO_BASE_IRQ(0)+gpio)

#ifndef __ASSEMBLY__


/* GPIO operations */

static __inline__ void
nxc2600_set_gpio_output( uint32_t pin,
			 int high )
{
	if( high )
		REG32(NXC2600_GPIO_DATA(pin/32)) |= (1<<(pin%32));
	else
		REG32(NXC2600_GPIO_DATA(pin/32)) &= ~(1<<(pin%32));	
}

static __inline__ void
nxc2600_set_gpio_gp_output( uint32_t gp,
                        uint32_t mask,
                        uint32_t outputs )
{
        REG32(NXC2600_GPIO_DATA(gp)) |= (outputs&mask);
        REG32(NXC2600_GPIO_DATA(gp)) &= ~((~outputs)&mask);
}

static __inline__ int
nxc2600_get_gpio_input( uint32_t pin )
{
	return ((REG32(NXC2600_GPIO_DATA(pin/32))&(1<<(pin%32)))? 1:0);
}

static __inline__ uint32_t
nxc2600_get_gpio_gp_input( uint32_t gp,
                           uint32_t mask )
{
        return REG32(NXC2600_GPIO_DATA(gp))&mask;
}

static __inline__ void
nxc2600_set_gpio_output_mode( uint32_t pin,
			      int pullupdown )
{
	uint32_t g,bit;

	g = pin / 32;
	pin %= 32;
	bit = (1<<pin);

	REG32(NXC2600_GPIO_IE(g)) &= ~bit;
	REG32(NXC2600_GPIO_DIR(g)) |= bit;

	if( pin < 16 )
		REG32(NXC2600_GPIO_SL(g)) &= ~(3<<(pin<<1));
	else
		REG32(NXC2600_GPIO_SH(g)) &= ~(3<<((pin-16)<<1));

	REG32(NXC2600_GPIO_PE(g)) = (REG32(NXC2600_GPIO_PE(g))&~bit) | ((pullupdown)? bit:0);

}

#define	NXC2600_GPIO_IRQ_TRIGGER_NO_IRQ		(-1)
#define NXC2600_GPIO_IRQ_TRIGGER_LOW_LEVEL	(0)
#define NXC2600_GPIO_IRQ_TRIGGER_HIGH_LEVEL	(1)
#define NXC2600_GPIO_IRQ_TRIGGER_FALLING_EDGE	(2)
#define	NXC2600_GPIO_IRQ_TRIGGER_RISING_EDGE	(3)

static __inline__ void
nxc2600_set_gpio_input_mode( uint32_t pin,
			     int pullupdown,
			     int irq_trigger )
{
	uint32_t g,bit;

        g = pin / 32;
        pin %= 32;
	bit = (1<<pin);

        REG32(NXC2600_GPIO_IE(g)) &= ~bit;
        REG32(NXC2600_GPIO_DIR(g)) &= ~bit;

	if( pin < 16 )
	{
                REG32(NXC2600_GPIO_SL(g)) &= ~(3<<(pin<<1));
		REG32(NXC2600_GPIO_ITL(g)) = (REG32(NXC2600_GPIO_ITL(g))&~(3<<(pin<<1))) | ((irq_trigger>=0)? (((uint32_t)irq_trigger)<<(pin<<1)):0);
	}
        else
	{
                REG32(NXC2600_GPIO_SH(g)) &= ~(3<<((pin-16)<<1));
		REG32(NXC2600_GPIO_ITH(g)) = (REG32(NXC2600_GPIO_ITH(g))&~(3<<((pin-16)<<1))) | ((irq_trigger>=0)? (((uint32_t)irq_trigger)<<((pin-16)<<1)):0);
	}

	REG32(NXC2600_GPIO_PE(g)) = (REG32(NXC2600_GPIO_PE(g))&~bit) | ((pullupdown)? bit:0);

	REG32(NXC2600_GPIO_IE(g)) = (REG32(NXC2600_GPIO_IE(g))&~bit) | ((irq_trigger>=0)? bit:0);
}

static __inline__ void
nxc2600_gpio_vs_uart0( int gpio_mode )
{
	REG32(NXC2600_GPIO_SH(3)) = (REG32(NXC2600_GPIO_SH(3))&~NXC2600_GPIO_SH_UART0_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SH_UART0);
}

static __inline__ void
nxc2600_gpio_vs_mac( int gpio_mode )
{
	REG32(NXC2600_GPIO_SH(3)) = (REG32(NXC2600_GPIO_SH(3))&~NXC2600_GPIO_SH_MAC_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SH_MAC);
}

static __inline__ void
nxc2600_gpio_vs_uart2( int gpio_mode )
{
	REG32(NXC2600_GPIO_SH(3)) = (REG32(NXC2600_GPIO_SH(3))&~NXC2600_GPIO_SH_UART2_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SH_UART2);
	REG32(NXC2600_GPIO_SL(3)) = (REG32(NXC2600_GPIO_SL(3))&~NXC2600_GPIO_SL_UART2_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SL_UART2);
}

static __inline__ void
nxc2600_gpio_vs_pwm1( int gpio_mode )
{
	REG32(NXC2600_GPIO_SH(2)) = (REG32(NXC2600_GPIO_SH(2))&~NXC2600_GPIO_SH_PWM1_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SH_PWM1);
}

static __inline__ void
nxc2600_gpio_vs_pwm0( int gpio_mode )
{
	REG32(NXC2600_GPIO_SH(2)) = (REG32(NXC2600_GPIO_SH(2))&~NXC2600_GPIO_SH_PWM0_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SH_PWM0);
}

static __inline__ void
nxc2600_gpio_vs_pcmcia( int gpio_mode )
{
	REG32(NXC2600_GPIO_SH(2)) = (REG32(NXC2600_GPIO_SH(2))&~NXC2600_GPIO_SH_PCMCIA_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SH_PCMCIA);
}

static __inline__ void
nxc2600_gpio_vs_cs5( int gpio_mode )
{
	REG32(NXC2600_GPIO_SH(2)) = (REG32(NXC2600_GPIO_SH(2))&~NXC2600_GPIO_SH_CS5_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SH_CS5);
}

static __inline__ void
nxc2600_gpio_vs_cs4( int gpio_mode )
{
        REG32(NXC2600_GPIO_SH(2)) = (REG32(NXC2600_GPIO_SH(2))&~NXC2600_GPIO_SH_CS4_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SH_CS4);
}

static __inline__ void
nxc2600_gpio_vs_cs3( int gpio_mode )
{
        REG32(NXC2600_GPIO_SH(2)) = (REG32(NXC2600_GPIO_SH(2))&~NXC2600_GPIO_SH_CS3_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SH_CS3);
}

static __inline__ void
nxc2600_gpio_vs_cs2( int gpio_mode )
{
        REG32(NXC2600_GPIO_SH(2)) = (REG32(NXC2600_GPIO_SH(2))&~NXC2600_GPIO_SH_CS2_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SH_CS2);
}

static __inline__ void
nxc2600_gpio_vs_cs1( int gpio_mode )
{
        REG32(NXC2600_GPIO_SH(2)) = (REG32(NXC2600_GPIO_SH(2))&~NXC2600_GPIO_SH_CS1_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SH_CS1);
}

static __inline__ void
nxc2600_gpio_vs_dcs1( int gpio_mode )
{
        REG32(NXC2600_GPIO_SH(2)) = (REG32(NXC2600_GPIO_SH(2))&~NXC2600_GPIO_SH_DCS1_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SH_DCS1);
}

static __inline__ void
nxc2600_gpio_vs_nand( int gpio_mode )
{
	REG32(NXC2600_GPIO_SH(2)) = (REG32(NXC2600_GPIO_SH(2))&~NXC2600_GPIO_SH_NAND_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SH_NAND);
	REG32(NXC2600_GPIO_SL(2)) = (REG32(NXC2600_GPIO_SL(2))&~NXC2600_GPIO_SL_NAND_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SL_NAND);

}

static __inline__ void
nxc2600_gpio_vs_ssi( int gpio_mode )
{
	REG32(NXC2600_GPIO_SL(2)) = (REG32(NXC2600_GPIO_SL(2))&~NXC2600_GPIO_SL_SSI_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SL_SSI);
}

static __inline__ void
nxc2600_gpio_vs_ac97( int gpio_mode )
{
	REG32(NXC2600_GPIO_SL(2)) = (REG32(NXC2600_GPIO_SL(2))&~NXC2600_GPIO_SL_AC97_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SL_AC97);
}

static __inline__ void
nxc2600_gpio_vs_i2s_master( int gpio_mode )
{
	REG32(NXC2600_GPIO_SL(2)) = (REG32(NXC2600_GPIO_SL(2))&~NXC2600_GPIO_SL_I2S_MASTER_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SL_I2S_MASTER);

}

static __inline__ void
nxc2600_gpio_vs_i2s_slave( int gpio_mode )
{
	REG32(NXC2600_GPIO_SL(2)) = (REG32(NXC2600_GPIO_SL(2))&~NXC2600_GPIO_SL_I2S_SLAVE_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SL_I2S_SLAVE);
}

static __inline__ void
nxc2600_gpio_vs_scc1( int gpio_mode )
{
	REG32(NXC2600_GPIO_SL(2)) = (REG32(NXC2600_GPIO_SL(2))&~NXC2600_GPIO_SL_SCC1_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SL_SCC1);
}

static __inline__ void
nxc2600_gpio_vs_scc0( int gpio_mode )
{
        REG32(NXC2600_GPIO_SL(2)) = (REG32(NXC2600_GPIO_SL(2))&~NXC2600_GPIO_SL_SCC0_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SL_SCC0);
}

static __inline__ void
nxc2600_gpio_vs_lcd( int gpio_mode )
{
	REG32(NXC2600_GPIO_SH(1)) = (REG32(NXC2600_GPIO_SH(1))&~NXC2600_GPIO_SH_LCD_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SH_LCD);
	REG32(NXC2600_GPIO_SL(1)) = (REG32(NXC2600_GPIO_SL(1))&~NXC2600_GPIO_SL_LCD_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SL_LCD);

}

static __inline__ void
nxc2600_gpio_vs_msc( int gpio_mode )
{
	REG32(NXC2600_GPIO_SL(1)) = (REG32(NXC2600_GPIO_SL(1))&~NXC2620_GPIO_SL_MSC_MASK) | ((gpio_mode)? 0:NXC2620_GPIO_SL_MSC);
}

static __inline__ void
nxc2600_gpio_vs_ps2( int gpio_mode )
{
	REG32(NXC2600_GPIO_SL(1)) = (REG32(NXC2600_GPIO_SL(1))&~NXC2600_GPIO_SL_PS2_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SL_PS2);
}

static __inline__ void
nxc2600_gpio_vs_ohci_hcd( int gpio_mode )
{
#if defined(CONFIG_USB_OHCI_HCD_NXC2600_EXT_CLK)
	REG32(NXC2600_GPIO_SH(0)) = (REG32(NXC2600_GPIO_SH(0))&~NXC2600_GPIO_SH_OHCI_HCD_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SH_OHCI_HCD);
	nxc2600_set_gpio_input_mode( 29, 0, 0 );
#endif
}

static __inline__ void
nxc2600_gpio_vs_uart1( int gpio_mode )
{
	REG32(NXC2600_GPIO_SH(0)) = (REG32(NXC2600_GPIO_SH(0))&~NXC2600_GPIO_SH_UART1_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SH_UART1);
}

static __inline__ void
nxc2600_gpio_vs_uart3_ctsrts( int gpio_mode )
{
	REG32(NXC2600_GPIO_SH(0)) = (REG32(NXC2600_GPIO_SH(0))&~NXC2600_GPIO_SH_UART3_CTSRTS_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SH_UART3_CTSRTS);
}

static __inline__ void
nxc2600_gpio_vs_uart3( int gpio_mode )
{
	REG32(NXC2600_GPIO_SH(0)) = (REG32(NXC2600_GPIO_SH(0))&~NXC2600_GPIO_SH_UART3_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SH_UART3);
}

static __inline__ void
nxc2600_gpio_vs_dma( int gpio_mode )
{
	REG32(NXC2600_GPIO_SH(0)) = (REG32(NXC2600_GPIO_SH(0))&~NXC2600_GPIO_SH_DMA_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SH_DMA);
	REG32(NXC2600_GPIO_SL(0)) = (REG32(NXC2600_GPIO_SL(0))&~NXC2600_GPIO_SL_DMA_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SL_DMA);
}

static __inline__ void
nxc2600_gpio_vs_cim( int gpio_mode )
{
	REG32(NXC2600_GPIO_SL(0)) = (REG32(NXC2600_GPIO_SL(0))&~NXC2600_GPIO_SL_CIM_MASK) | ((gpio_mode)? 0:NXC2600_GPIO_SL_CIM);
}

#endif



#define NXC2600_OST_CR(n)			(0x10002018+0x20*(n))
#define 	NXC2600_OST_CR_CKC_PCLK_DIV4		(0x0000)
#define 	NXC2600_OST_CR_CKC_PCLK_DIV16		(0x0001)
#define 	NXC2600_OST_CR_CKC_PCLK_DIV64		(0x0002)
#define 	NXC2600_OST_CR_CKC_PCLK_DIV256		(0x0003)
#define 	NXC2600_OST_CR_CKC_RTCCLK		(0x0004)
#define 	NXC2600_OST_CR_CKC_EXTAL		(0x0005)
#define 	NXC2600_OST_CR_UIE			(0x0020)
#define 	NXC2600_OST_CR_UF			(0x0040)
#define 	NXC2600_OST_CR_SF			(0x0080)

#define NXC2600_OST_DR(n)			(0x10002010+0x20*(n))
#define NXC2600_OST_CNT(n)			(0x10002014+0x20*(n))
#define NXC2600_OST_ER				(0x10002000)
#define 	NXC2600_OST_ER_OTE(n)			(1<<(n))

#ifndef __ASSEMBLY__

static __inline__ void
nxc2600_stop_system_timer(void)
{
	REG8(NXC2600_OST_ER) &= ~NXC2600_OST_ER_OTE(0);
}

static __inline__ void
nxc2600_start_system_timer( uint32_t hz,
			    uint32_t count_start )
{
	REG8(NXC2600_OST_ER) &= ~NXC2600_OST_ER_OTE(0);
	REG16(NXC2600_OST_CR(0)) = 0;
        REG32(NXC2600_OST_DR(0)) = ((hz)? (CONFIG_NXC2600_EXTAL_CLOCK/hz):0xffffffff);

        REG32(NXC2600_OST_CNT(0)) = ((count_start)? count_start:((hz)? (CONFIG_NXC2600_EXTAL_CLOCK/hz):0xffffffff));

	REG16(NXC2600_OST_CR(0)) = NXC2600_OST_CR_UIE|NXC2600_OST_CR_CKC_EXTAL;

	while(REG16(NXC2600_OST_CR(0))&NXC2600_OST_CR_SF);

	REG8(NXC2600_OST_ER) |= NXC2600_OST_ER_OTE(0);

}

static __inline__ void
nxc2600_reset_system_timer( void )
{
	REG8(NXC2600_OST_ER) &= ~NXC2600_OST_ER_OTE(0);
	REG32(NXC2600_OST_CNT(0)) = REG32(NXC2600_OST_DR(0));
	REG8(NXC2600_OST_ER) |= NXC2600_OST_ER_OTE(0);
}

static __inline__ uint32_t
nxc2600_get_system_timer_counter( void )
{
	return (REG32(NXC2600_OST_DR(0))-REG32(NXC2600_OST_CNT(0)));
}

#endif

#define NXC2600_UART(uart)			(0x10030000|((uart)*0x1000))
#define NXC2600_UART_LSR(uart)			(NXC2600_UART(uart)|0x14)
#define 	NXC2600_UART_LSR_TEMP			(0x40)
#define 	NXC2600_UART_LSR_THRE			(0x20)
#define 	NXC2600_UART_LSR_DRY			(0x01)
#define NXC2600_UART_THR(uart)			NXC2600_UART(uart)
#define NXC2600_UART_RBR(uart)			NXC2600_UART(uart)
#define NXC2600_UART_LCR(uart)			(NXC2600_UART(uart)|0x0c)
#define 	NXC2600_UART_LCR_WLS(wls)		((wls)-5) /* 5-8 */
#define 	NXC2600_UART_LCR_SBLS(stop)		(((stop)-1)<<2) /* 1, 2 */
#define 	NXC2600_UART_LCR_PARE			(0x08)
#define 	NXC2600_UART_LCR_PARM_ODD		(0x00)
#define 	NXC2600_UART_LCR_PARM_EVEN		(0x10)
#define 	NXC2600_UART_LCR_STPAR			(0x20)
#define 	NXC2600_UART_LCR_SBK			(0x40)
#define 	NXC2600_UART_LCR_DLAB			(0x80)
#define NXC2600_UART_DLLR(uart)			NXC2600_UART(uart)
#define NXC2600_UART_DLHR(uart)			(NXC2600_UART(uart)|0x04)
#define NXC2600_UART_IER(uart)			(NXC2600_UART(uart)|0x04)
#define NXC2600_UART_FCR(uart)			(NXC2600_UART(uart)|0x08)
#define 	NXC2600_UART_FCR_UUE			(0x10)
#define 	NXC2600_UART_FCR_FME			(0x01)
#define 	NXC2600_UART_FCR_RFRT			(0x02)
#define 	NXC2600_UART_FCR_TFRT			(0x04)
#define 	NXC2600_UART_FCR_DME			(0x08)
#define 	NXC2600_UART_FCR_RDTR_1			(0x00)
#define 	NXC2600_UART_FCR_RDTR_4			(0x40)
#define 	NXC2600_UART_FCR_RDTR_8			(0x80)
#define 	NXC2600_UART_FCR_RDTR_15		(0xc0)
#define	NXC2600_UART_MCR(uart)			(NXC2600_UART(uart)|0x10)
#define		NXC2600_UART_MCR_ENABLE_MODEM		(0x80)
#define		NXC2600_UART_MCR_LOOP			(0x10)
#define		NXC2600_UART_MCR_RTS			(0x02)
#define	NXC2600_UART_MSR(uart)			(NXC2600_UART(uart)|0x18)
#define		NXC2600_UART_MSR_CTS			(0x10)
#define		NXC2600_UART_MSR_CCTS			(0x01)
#define	NXC2600_UART_ISR(uart)			(NXC2600_UART(uart)|0x20)
#define		NXC2600_UART_ISR_XMITIR			(0x01)
#define		NXC2600_UART_ISR_RCVEIR			(0x02)
#define		NXC2600_UART_ISR_XMODE			(0x04)
#define		NXC2600_UART_ISR_TDPL			(0x08)
#define		NXC2600_UART_ISR_RDPL			(0x10)

#ifndef CONFIG_NXC2600_DEFAULT_DEBUG_PORT
#define CONFIG_NXC2600_DEFAULT_DEBUG_PORT		(0)
#endif

#define NXC2600_CS0				(0x1c000000)
#define NXC2600_CS0_FLASH_SIZE			(0x00800000)

#define NXC2600_RTC_CR				(0x10003000)
#define		NXC2600_RTC_CR_RTCEN			(0x00000001)
#define		NXC2600_RTC_CR_AE			(0x00000004)
#define		NXC2600_RTC_CR_AIE			(0x00000008)
#define		NXC2600_RTC_CR_AF			(0x00000010)
#define		NXC2600_RTC_CR_1HZIE			(0x00000020)
#define		NXC2600_RTC_CR_1HZ			(0x00000040)
#define NXC2600_RTC_SR				(0x10003004)
#define NXC2600_RTC_SAR				(0x1000300c)
#define	NXC2600_RTC_GR				(0x10003018)
#define		NXC2600_RTC_GR_DIV(div)			((div)&0x0000ffff)
#define		NXC2600_RTC_GR_ADC(adc)			(((adc)&0x000003ff)<<16)
#define		NXC2600_RTC_GR_LOCK			(0x80000000)

	
#define NXC2600_EMC_BCR				(0x13010000)
#define 	NXC2600_EMC_BCR_BRE			(0x00000002)

#define NXC2600_SM_CR0				(0x13010010)
#define NXC2600_SM_CR1				(0x13010014)
#define NXC2600_SM_CR2				(0x13010018)
#define NXC2600_SM_CR3				(0x1301001c)
#define NXC2600_SM_CR4				(0x13010020)
#define NXC2600_SM_CR5				(0x13010024)
#define 	NXC2600_SM_CR_TYPE_MASK			(0x00000001)
#define 		NXC2600_SM_CR_TYPE_NORMAL		(0x00000000)
#define 		NXC2600_SM_CR_TYPE_BURST		(0x00000001)
#define 	NXC2600_SM_CR_BL_MASK			(0x00000006)
#define 		NXC2600_SM_CR_BL_CA_4			(0x00000000)
#define 		NXC2600_SM_CR_BL_CA_8			(0x00000002)
#define 		NXC2600_SM_CR_BL_CA_16			(0x00000004)
#define 		NXC2600_SM_CR_BL_CA_32			(0x00000006)
#define 	NXC2600_SM_CR_BCM_MASK			(0x00000008)
#define 		NXC2600_SM_CR_BCM_NORMAL		(0x00000000)
#define 		NXC2600_SM_CR_BCM_BYTE_CTRL		(0x00000008)
#define 	NXC2600_SM_CR_BW_MASK			(0x000000c0)
#define 		NXC2600_SM_CR_BW_BITS_8			(0x00000000)
#define 		NXC2600_SM_CR_BW_BITS_16		(0x00000040)
#define 		NXC2600_SM_CR_BW_BITS_32		(0x00000080)
#define 	NXC2600_SM_CR_TAS_MASK			(0x00000700)
#define 		NXC2600_SM_CR_TAS(cycles)		((0x7&cycles)<<8)
#define 	NXC2600_SM_CR_TAH_MASK			(0x00007000)
#define 		NXC2600_SM_CR_TAH(cycles)		((0x7&cycles)<<12)
#define 	NXC2600_SM_CR_TBP_MASK			(0x000f0000)
#define 		NXC2600_SM_CR_TBP_0			(0x00000000)
#define 		NXC2600_SM_CR_TBP_1			(0x00010000)
#define 		NXC2600_SM_CR_TBP_2			(0x00020000)
#define 		NXC2600_SM_CR_TBP_3			(0x00030000)
#define 		NXC2600_SM_CR_TBP_4			(0x00040000)
#define 		NXC2600_SM_CR_TBP_5			(0x00050000)
#define 		NXC2600_SM_CR_TBP_6			(0x00060000)
#define 		NXC2600_SM_CR_TBP_7			(0x00070000)
#define 		NXC2600_SM_CR_TBP_8			(0x00080000)
#define 		NXC2600_SM_CR_TBP_9			(0x00090000)
#define 		NXC2600_SM_CR_TBP_10			(0x000a0000)
#define 		NXC2600_SM_CR_TBP_12			(0x000b0000)
#define 		NXC2600_SM_CR_TBP_15			(0x000c0000)
#define 		NXC2600_SM_CR_TBP_20			(0x000d0000)
#define 		NXC2600_SM_CR_TBP_25			(0x000e0000)
#define 		NXC2600_SM_CR_TBP_31			(0x000f0000)
#define 	NXC2600_SM_CR_TAW_MASK			(0x00f00000)
#define 		NXC2600_SM_CR_TAW_0			(0x00000000)
#define 		NXC2600_SM_CR_TAW_1			(0x00100000)
#define 		NXC2600_SM_CR_TAW_2			(0x00200000)
#define 		NXC2600_SM_CR_TAW_3			(0x00300000)
#define 		NXC2600_SM_CR_TAW_4			(0x00400000)
#define 		NXC2600_SM_CR_TAW_5			(0x00500000)
#define 		NXC2600_SM_CR_TAW_6			(0x00600000)
#define 		NXC2600_SM_CR_TAW_7			(0x00700000)
#define 		NXC2600_SM_CR_TAW_8			(0x00800000)
#define 		NXC2600_SM_CR_TAW_9			(0x00900000)
#define 		NXC2600_SM_CR_TAW_10			(0x00a00000)
#define 		NXC2600_SM_CR_TAW_12			(0x00b00000)
#define 		NXC2600_SM_CR_TAW_15			(0x00c00000)
#define 		NXC2600_SM_CR_TAW_20			(0x00d00000)
#define 		NXC2600_SM_CR_TAW_25			(0x00e00000)
#define			NXC2600_SM_CR_TAW_31			(0x00f00000)
#define 	NXC2600_SM_CR_STRV_MASK			(0x0f000000)
#define 		NXC2600_SM_CR_STRV(cycles)		((0xf&(cycles))<<24)

#define NXC2600_SM_ACR0				(0x13010030)
#define NXC2600_SM_ACR1				(0x13010034)
#define NXC2600_SM_ACR2				(0x13010038)
#define NXC2600_SM_ACR3				(0x1301003c)
#define NXC2600_SM_ACR4				(0x13010040)
#define NXC2600_SM_ACR5				(0x13010044)



#ifndef __ASSEMBLY__

static __inline__ void
nxc2600_sm_init( void )
{
}

static __inline__ void
nxc2600_sm_nor_boot_flash_init(uint32_t addr)
{
	REG32(NXC2600_SM_CR0) = 0x0fff7740;
	REG32(NXC2600_SM_ACR0) = ((addr>>16)&0x0000ff00)|0x000000ff;
}

#endif



#define NXC2600_SDRAM_DIV	(3)
#define NXC2600_SDRAM_CLOCK	(CONFIG_NXC2600_CPU_CLOCK/NXC2600_SDRAM_DIV)
#ifndef CONFIG_NXC2600_SDRAM_ROW
#define	CONFIG_NXC2600_SDRAM_ROW	(13)
#endif
#ifndef CONFIG_NXC2600_SDRAM_COL
#define	CONFIG_NXC2600_SDRAM_COL	(9)
#endif
#ifndef CONFIG_NXC2600_SDRAM_CL
#define	CONFIG_NXC2600_SDRAM_CL		(3)
#endif
#ifndef CONFIG_NXC2600_SDRAM_TRAS
#define CONFIG_NXC2600_SDRAM_TRAS	(45)
#endif
#ifndef CONFIG_NXC2600_SDRAM_RCD
#define	CONFIG_NXC2600_SDRAM_RCD	(20)
#endif
#ifndef CONFIG_NXC2600_SDRAM_TPC
#define	CONFIG_NXC2600_SDRAM_TPC	(20)
#endif
#ifndef CONFIG_NXC2600_SDRAM_TRWL
#define	CONFIG_NXC2600_SDRAM_TRWL	(7)
#endif
#ifndef CONFIG_NXC2600_SDRAM_TREF
#define	CONFIG_NXC2600_SDRAM_TREF	(7812)
#endif

#define NXC2600_SDRAM_DMCR			(0x13010080)
#define 	NXC2600_SDRAM_DMCR_TCL_MASK		(0x00000003)
#define 		NXC2600_SDRAM_DMCR_TCL_INHIBIT		(0x00000000)
#define 		NXC2600_SDRAM_DMCR_TCL_CYCLES_2		(0x00000001)
#define 		NXC2600_SDRAM_DMCR_TCL_CYCLES_3		(0x00000002)
#define 	NXC2600_SDRAM_DMCR_TRC_MASK		(0x0000001c)
#define 		NXC2600_SDRAM_DMCR_TRC(cycles)		(((((cycles)+1)/2)&0x7)<<2) /* 1,3,5,7,9,11,13,15 */
#define 	NXC2600_SDRAM_DMCR_TRWL_MASK		(0x00000060)
#define 		NXC2600_SDRAM_DMCR_TRWL(cycles)		((((cycles))&0x3)<<5) /* 1-4 */
#define 	NXC2600_SDRAM_DMCR_TPC_MASK		(0x00000700)
#define 		NXC2600_SDRAM_DMCR_TPC(cycles)		((((cycles))&0x7)<<8) /* 1-8 */
#define 	NXC2600_SDRAM_DMCR_RCD_MASK		(0x00001800)
#define 		NXC2600_SDRAM_DMCR_RCD(delay)		((((delay))&0x3)<<11) /* 1-4 */
#define 	NXC2600_SDRAM_DMCR_TRAS_MASK		(0x0000e000)
#define 		NXC2600_SDRAM_DMCR_TRAS(time)		((((time)-4)&0x7)<<13) /* 4-11 */
#define 	NXC2600_SDRAM_DMCR_EPIN_MASK		(0x00020000)
#define 		NXC2600_SDRAM_DMCR_EPIN_DESERTED	(0x00000000)
#define 		NXC2600_SDRAM_DMCR_EPIN_ASSERTED	(0x00020000)
#define 	NXC2600_SDRAM_DMCR_PDM_MASK		(0x00040000)
#define 		NXC2600_SDRAM_DMCR_PDM_NON_POWER_DOWN	(0x00000000)
#define 		NXC2600_SDRAM_DMCR_PDM_POWER_DOWN	(0x00040000)
#define 	NXC2600_SDRAM_DMCR_BA_MASK		(0x00080000)
#define 		NXC2600_SDRAM_DMCR_BA_1_BIT_BANK	(0x00000000)
#define 		NXC2600_SDRAM_DMCR_BA_2_BIT_BANK	(0x00080000)
#define 	NXC2600_SDRAM_DMCR_RA_MASK		(0x00300000)
#define 		NXC2600_SDRAM_DMCR_RA(row)		((((row)-11)&0x3)<<20) /* 11-13 */
#define 	NXC2600_SDRAM_DMCR_MRSET_MASK		(0x00800000)
#define 		NXC2600_SDRAM_DMCR_MRSET_MODE_SETTING	(0x00800000)
#define 		NXC2600_SDRAM_DMCR_MRSET_PRE_CHARGE	(0x00000000)
#define 	NXC2600_SDRAM_DMCR_RFSH_MASK		(0x01000000)
#define 		NXC2600_SDRAM_DMCR_RFSH_NO_REFRESH	(0x00000000)
#define 		NXC2600_SDRAM_DMCR_RFSH_REFRESH		(0x01000000)
#define 	NXC2600_SDRAM_DMCR_RMODE_MASK		(0x02000000)
#define 		NXC2600_SDRAM_DMCR_RMODE_AUTO_REFRESH	(0x00000000)
#define 		NXC2600_SDRAM_DMCR_RMODE_SELF_REFRESH	(0x02000000)
#define 	NXC2600_SDRAM_DMCR_CA_MASK		(0x1c000000)
#define 		NXC2600_SDRAM_DMCR_CA(col)		((((col)-8)&0x7)<<26) /* 8-12 */
#define 		NXC2600_SDRAM_DMCR_BW_MASK		(0x80000000)
#define 		NXC2600_SDRAM_DMCR_BW_32_BITS		(0x00000000)
#define 		NXC2600_SDRAM_DMCR_BW_16_BITS		(0x80000000)
#define NXC2600_SDRAM_DMCR_DEFAULT		(NXC2600_SDRAM_DMCR_BW_32_BITS| \
						 NXC2600_SDRAM_DMCR_BA_2_BIT_BANK| \
						 NXC2600_SDRAM_DMCR_EPIN_ASSERTED| \
						 ((CONFIG_NXC2600_SDRAM_CL==2)? NXC2600_SDRAM_DMCR_TCL_CYCLES_2:NXC2600_SDRAM_DMCR_TCL_CYCLES_3))

#define NXC2600_SDRAM_SDMR(bank)		(0x1301a000+(0x1000*bank))
#define 	NXC2600_SDRAM_SDMR_BL_1			(0x000)
#define 	NXC2600_SDRAM_SDMR_BL_2			(0x001)
#define 	NXC2600_SDRAM_SDMR_BL_4			(0x002)
#define 	NXC2600_SDRAM_SDMR_BL_8			(0x003)
#define 	NXC2600_SDRAM_SDMR_BT_SEQUENTIAL	(0x000)
#define 	NXC2600_SDRAM_SDMR_BT_INTERLEAVE	(0x008)
#define 	NXC2600_SDRAM_SDMR_CAS(latency)		(((latency)&0x3)<<4) /* 1-3 */
#define 	NXC2600_SDRAM_SDMR_OM_NORMAL		(0x000)
#define 	NXC2600_SDRAM_SDMR_WBM_BURST		(0x000)
#define 	NXC2600_SDRAM_SDMR_WBM_SIGNLE		(0x100)
#define NXC2600_SDRAM_SDMR_DEFAULT		(NXC2600_SDRAM_SDMR_BL_4| \
						 NXC2600_SDRAM_SDMR_BT_SEQUENTIAL| \
						 NXC2600_SDRAM_SDMR_CAS(((CONFIG_NXC2600_SDRAM_CL==2)?2:3))| \
						 NXC2600_SDRAM_SDMR_OM_NORMAL)


#define NXC2600_SDRAM_RTCSR			(0x13010084)
#define 	NXC2600_SDRAM_RTCSR_CKS_DISABLE		(0x00000000)
#define 	NXC2600_SDRAM_RTCSR_CKS_CKO_DIV_4	(0x00000001)
#define 	NXC2600_SDRAM_RTCSR_CKS_CKO_DIV_16	(0x00000002)
#define 	NXC2600_SDRAM_RTCSR_CKS_CKO_DIV_64	(0x00000003)
#define 	NXC2600_SDRAM_RTCSR_CKS_CKO_DIV_256	(0x00000004)
#define 	NXC2600_SDRAM_RTCSR_CKS_CKO_DIV_1024	(0x00000005)
#define 	NXC2600_SDRAM_RTCSR_CKS_CKO_DIV_2048	(0x00000006)
#define 	NXC2600_SDRAM_RTCSR_CKS_CKO_DIV_4096	(0x00000007)
#define 	NXC2600_SDRAM_RTCSR_CMF_NOT_MATCH	(0x00000000)
#define 	NXC2600_SDRAM_RTCSR_CMF_MATCH		(0x00000080)

#define NXC2600_SDRAM_RTCNT			(0x13010088)
#define 	NXC2600_SDRAM_RTCNT_RTCNT(c)		((c)&0xff)

#define NXC2600_SDRAM_RTCOR			(0x1301008c)
#define 	NXC2600_SDRAM_RTCOR_RTCOR(c)		((c)&0xff)

#define NXC2600_SDRAM_AR(bank)                  (0x13010090+(4*bank))

#ifndef __ASSEMBLY__

static __inline__  int
nxc2600_sdram_init( int banks,
                    int col,
                    int row,
                    unsigned int size_of_bank )
{
        register unsigned int dmcr, sdmode, tmp, ns;

        nxc2600_gpio_vs_dcs1((banks>1)? 0:1);
	

        REG32(NXC2600_EMC_BCR) = NXC2600_EMC_BCR_BRE;
        REG16(NXC2600_SDRAM_RTCSR) = NXC2600_SDRAM_RTCSR_CKS_DISABLE;
	udelay(400);
        REG16(NXC2600_SDRAM_RTCOR) = 0;
        REG16(NXC2600_SDRAM_RTCNT) = 0;

        tmp = size_of_bank/0x1000000;
	REG32(NXC2600_SDRAM_AR(0)) = 0x000020f0|tmp;
        REG32(NXC2600_SDRAM_AR(1)) = 0x00002000|tmp|(tmp<<8)|((banks>1)?0xf0:0);

        dmcr = NXC2600_SDRAM_DMCR_DEFAULT|NXC2600_SDRAM_DMCR_CA(col)|NXC2600_SDRAM_DMCR_RA(row);


        ns = 1000000000 / NXC2600_SDRAM_CLOCK;

        tmp = CONFIG_NXC2600_SDRAM_TRAS/ns;
        if (tmp < 4)
                tmp = 4;
        else if (tmp > 11)
                tmp = 11;
        dmcr |= NXC2600_SDRAM_DMCR_TRAS(tmp);

        tmp = CONFIG_NXC2600_SDRAM_RCD/ns;
        if (tmp > 3)
                tmp = 3;
        dmcr |= NXC2600_SDRAM_DMCR_RCD(tmp);

        tmp = CONFIG_NXC2600_SDRAM_TPC/ns;
        if (tmp > 7)
                tmp = 7;
        dmcr |= NXC2600_SDRAM_DMCR_TPC(tmp);

        tmp = CONFIG_NXC2600_SDRAM_TRWL/ns;
        if (tmp > 3)
                tmp = 3;
        dmcr |= NXC2600_SDRAM_DMCR_TRWL(tmp);

        tmp = (CONFIG_NXC2600_SDRAM_TRAS + CONFIG_NXC2600_SDRAM_TPC)/ns;
        if (tmp > 14)
                tmp = 14;
        dmcr |= NXC2600_SDRAM_DMCR_TRC(tmp);

        sdmode = NXC2600_SDRAM_SDMR_DEFAULT;

        sdmode <<= 2;

        /* First, precharge phase */
        REG32(NXC2600_SDRAM_DMCR) = dmcr;
        udelay(500);

        /* Set refresh registers */
        tmp = CONFIG_NXC2600_SDRAM_TREF/ns;
        tmp = tmp/64 + 1;
        if (tmp > 0xff)
                tmp = 0xff;

        REG16(NXC2600_SDRAM_RTCOR) = tmp;
        REG16(NXC2600_SDRAM_RTCSR) = NXC2600_SDRAM_RTCSR_CKS_CKO_DIV_64;

        /* precharge all chip-selects */
        REG8(NXC2600_SDRAM_SDMR(0)|sdmode) = 0;
	if(banks>1)
        REG8(NXC2600_SDRAM_SDMR(1)|sdmode) = 0;

        /* wait for precharge, > 500us */
        udelay(500);

        /* enable refresh and set SDRAM mode */
        REG32(NXC2600_SDRAM_DMCR) = dmcr |
                                    NXC2600_SDRAM_DMCR_MRSET_MODE_SETTING |
                                    NXC2600_SDRAM_DMCR_RFSH_REFRESH;

        /* write sdram mode register for each chip-select */
        REG8(NXC2600_SDRAM_SDMR(0)|sdmode) = 0;
	if(banks>1)
        REG8(NXC2600_SDRAM_SDMR(1)|sdmode) = 0;
	udelay(500);
        return 0;
}

#endif




#define NXC2600_CGM_CCR0			(0x10000000)
#define 	NXC2600_CGM_CCR0_CDIV_MASK		(0x0000000f)
#define 		NXC2600_CGM_CCR0_CDIV_1			(0x00000000)
#define 		NXC2600_CGM_CCR0_CDIV_1_2		(0x00000001)
#define 		NXC2600_CGM_CCR0_CDIV_1_3		(0x00000002)
#define 		NXC2600_CGM_CCR0_CDIV_1_4		(0x00000003)
#define 		NXC2600_CGM_CCR0_CDIV_1_6		(0x00000004)
#define 		NXC2600_CGM_CCR0_CDIV_1_8		(0x00000005)
#define 		NXC2600_CGM_CCR0_CDIV_1_12		(0x00000006)
#define 		NXC2600_CGM_CCR0_CDIV_1_16		(0x00000007)
#define 		NXC2600_CGM_CCR0_CDIV_1_24		(0x00000008)
#define 		NXC2600_CGM_CCR0_CDIV_1_32		(0x00000009)
#define 	NXC2600_CGM_CCR0_HDIV_MASK		(0x000000f0)
#define 		NXC2600_CGM_CCR0_HDIV_1			(0x00000000)
#define 		NXC2600_CGM_CCR0_HDIV_1_2		(0x00000010)
#define 		NXC2600_CGM_CCR0_HDIV_1_3		(0x00000020)
#define 		NXC2600_CGM_CCR0_HDIV_1_4		(0x00000030)
#define 		NXC2600_CGM_CCR0_HDIV_1_6		(0x00000040)
#define 		NXC2600_CGM_CCR0_HDIV_1_8		(0x00000050)
#define 		NXC2600_CGM_CCR0_HDIV_1_12		(0x00000060)
#define 		NXC2600_CGM_CCR0_HDIV_1_16		(0x00000070)
#define 		NXC2600_CGM_CCR0_HDIV_1_24		(0x00000080)
#define 		NXC2600_CGM_CCR0_HDIV_1_32		(0x00000090)
#define 	NXC2600_CGM_CCR0_PDIV_MASK		(0x00000f00)
#define 		NXC2600_CGM_CCR0_PDIV_1			(0x00000000)
#define 		NXC2600_CGM_CCR0_PDIV_1_2		(0x00000100)
#define 		NXC2600_CGM_CCR0_PDIV_1_3		(0x00000200)
#define 		NXC2600_CGM_CCR0_PDIV_1_4		(0x00000300)
#define 		NXC2600_CGM_CCR0_PDIV_1_6		(0x00000400)
#define 		NXC2600_CGM_CCR0_PDIV_1_8		(0x00000500)
#define 		NXC2600_CGM_CCR0_PDIV_1_12		(0x00000600)
#define 		NXC2600_CGM_CCR0_PDIV_1_16		(0x00000700)
#define 		NXC2600_CGM_CCR0_PDIV_1_24		(0x00000800)
#define 		NXC2600_CGM_CCR0_PDIV_1_32		(0x00000900)
#define 	NXC2600_CGM_CCR0_LDIV_MASK		(0x0000f000)
#define 		NXC2600_CGM_CCR0_LDIV(div)		(((div)-1)<<12)	/* 1-16 */
#define 	NXC2600_CGM_CCR0_MDIV_MASK		(0x000f0000)
#define 		NXC2600_CGM_CCR0_MDIV_1			(0x00000000)
#define 		NXC2600_CGM_CCR0_MDIV_1_2		(0x00010000)
#define 		NXC2600_CGM_CCR0_MDIV_1_3		(0x00020000)
#define 		NXC2600_CGM_CCR0_MDIV_1_4		(0x00030000)
#define 		NXC2600_CGM_CCR0_MDIV_1_6		(0x00040000)
#define 		NXC2600_CGM_CCR0_MDIV_1_8		(0x00050000)
#define 		NXC2600_CGM_CCR0_MDIV_1_12		(0x00060000)
#define 		NXC2600_CGM_CCR0_MDIV_1_16		(0x00070000)
#define 		NXC2600_CGM_CCR0_MDIV_1_24		(0x00080000)
#define 		NXC2600_CGM_CCR0_MDIV_1_32		(0x00090000)
#define 	NXC2600_CGM_CCR0_CE_UPDATE		(0x00100000)
#define 	NXC2600_CGM_CCR0_CE_PLL_DISABLE		(0x00000000)
#define 	NXC2600_CGM_CCR0_SCLKOEN_ENABLE		(0x00400000)
#define 	NXC2600_CGM_CCR0_SCLKOEN_DISABLE	(0x00000000)
#define 	NXC2600_CGM_CCR0_MCS_24_576MHZ		(0x01000000)
#define 	NXC2600_CGM_CCR0_MCS_19_1692MHZ		(0x00000000)
#define 	NXC2600_CGM_CCR0_UDIV_MASK		(0x0e000000)
#define 		NXC2600_CGM_CCR0_UDIV(div)		((div-1)<<25) /* 1-8 */
#define 	NXC2600_CGM_CCR0_UCS_OHCI_HCD_CLK	(0x10000000)
#define 	NXC2600_CGM_CCR0_UCS_OHCI_HCD_PLL	(0x00000000)
#define 	NXC2600_CGM_CCR0_I2CS_PLL_DIV_2		(0x20000000)
#define 	NXC2600_CGM_CCR0_I2CS_PLL_DIV_1		(0x00000000)
#define 	NXC2600_CGM_CCR0_LCS_LCD_PCLK		(0x40000000)
#define 	NXC2600_CGM_CCR0_LCS_LCD_DIV		(0x00000000)
#define 	NXC2600_CGM_CCR0_SCS_SSI_USBCLK		(0x80000000)
#define 	NXC2600_CGM_CCR0_SCS_SSI_EXTAL		(0x00000000)

#if defined(CONFIG_USB_OHCI_HCD_NXC2600_EXT_CLK)
#define	OHCI_HCD_CLOCK_SOURCE		NXC2600_CGM_CCR0_UCS_OHCI_HCD_CLK
#else
#if (CONFIG_NXC2600_CPU_CLOCK%48000000) != 0
#error PLL/CPU clock must be N*48000000
#endif
#define	OHCI_HCD_CLOCK_SOURCE		(NXC2600_CGM_CCR0_UCS_OHCI_HCD_PLL|NXC2600_CGM_CCR0_UDIV(CONFIG_NXC2600_CPU_CLOCK/48000000))
#endif

#define NXC2600_CGM_CCR0_DEFAULT		(OHCI_HCD_CLOCK_SOURCE|	\
						 NXC2600_CGM_CCR0_SCLKOEN_ENABLE| \
						 NXC2600_CGM_CCR0_MDIV_1_3| \
						 NXC2600_CGM_CCR0_LDIV(3)| \
						 NXC2600_CGM_CCR0_PDIV_1_3| \
						 NXC2600_CGM_CCR0_HDIV_1_3| \
						 NXC2600_CGM_CCR0_CDIV_1)

#define NXC2600_CGM_PCR				(0x10000010)
#define 	NXC2600_CGM_PCR_PLLST(st)		(0xff&st)
#define 	NXC2600_CGM_PCR_PLLEN			(0x00000100)
#define 	NXC2600_CGM_PCR_PLLBP			(0x00000200)
#define 	NXC2600_CGM_PCR_PLLS_MASK		(0x00000400) /*Read Only*/
#define 	NXC2600_CGM_PCR_PLLOD_DIV1		(0x00000000)
#define 	NXC2600_CGM_PCR_PLLOD_DIV2		(0x00010000)
#define 	NXC2600_CGM_PCR_PLLOD_DIV4		(0x00030000)
#define 	NXC2600_CGM_PCR_PLLN(n)			((n&0x1f)<<18)
#define 	NXC2600_CGM_PCR_PLLM(m)			((m&0x1ff)<<23)
#define NXC2600_CGM_PCR_DEFAULT			(NXC2600_CGM_PCR_PLLM(CONFIG_NXC2600_CPU_CLOCK*2/CONFIG_NXC2600_EXTAL_CLOCK+1-2)| \
						 NXC2600_CGM_PCR_PLLN(0)| \
						 NXC2600_CGM_PCR_PLLOD_DIV1| \
						 NXC2600_CGM_PCR_PLLEN| \
						 NXC2600_CGM_PCR_PLLST(0x20))			

#define NXC2600_CGM_CCR1			(0x10000060)
#define		NXC2600_CGM_CCR1_PXDIV_MASK		(0x000001ff)
#define		NXC2600_CGM_CCR1_PXDIV(pxdiv)		(((pxdiv)-1)&NXC2600_CGM_CCR1_PXDIV_MASK)

#define NXC2600_CGM_SSI				(0x10000074)
#define	NXC2600_CGM_SSI_MASK				(0x0000000f)


#ifndef __ASSEMBLY__

static __inline__ void
nxc2600_cpu_idle(void)
{
        REG32(NXC2600_CGM_PCR) = (REG32(NXC2600_CGM_PCR)&~NXC2600_CGM_PCR_PLLEN)|NXC2600_CGM_PCR_PLLBP;
}

static __inline__ void
nxc2600_cpu_max_speed(void)
{
        REG32(NXC2600_CGM_PCR) = (REG32(NXC2600_CGM_PCR)&~NXC2600_CGM_PCR_PLLBP)|NXC2600_CGM_PCR_PLLEN;
}

static __inline__ void
nxc2600_set_clock( uint32_t bits )
{
	REG32(NXC2600_CGM_CCR0); 
}

static __inline__ uint32_t
nxc2600_get_pllout( void )
{
	uint32_t pcr = REG32(NXC2600_CGM_PCR);

	if( !( pcr & NXC2600_CGM_PCR_PLLEN) )
		return CONFIG_NXC2600_EXTAL_CLOCK;

	return CONFIG_NXC2600_CPU_CLOCK;

}

static __inline__ void
nxc2600_set_pix_clock( uint32_t clock )
{
	REG32(NXC2600_CGM_CCR1) = NXC2600_CGM_CCR1_PXDIV(nxc2600_get_pllout()/clock);
}

static __inline__ void
nxc2600_set_lcd_clock( uint32_t clock )
{
	uint32_t div;
        unsigned int encode[10] = {1,2,3,4,6,8,12,16,24,32};
	clock = nxc2600_get_pllout()/clock;
        for(div=0;div<10;div++)
                if (clock < encode[div])
                        break;

	REG32(NXC2600_CGM_CCR0) = (REG32(NXC2600_CGM_CCR0)&~NXC2600_CGM_CCR0_LDIV_MASK) | NXC2600_CGM_CCR0_LDIV(div) | NXC2600_CGM_CCR0_CE_UPDATE;

}

static __inline__ void
nxc2600_set_msc_clock( uint32_t clock )
{
	if(clock==19169200)
		REG32(NXC2600_CGM_CCR0) &= ~NXC2600_CGM_CCR0_MCS_24_576MHZ;
	else
		REG32(NXC2600_CGM_CCR0) |= NXC2600_CGM_CCR0_MCS_24_576MHZ;

	REG32(NXC2600_CGM_CCR0) |= NXC2600_CGM_CCR0_CE_UPDATE;
}

static __inline__ void
nxc2600_set_ssi_clock( uint32_t clock )
{

	REG32(NXC2600_CGM_CCR0) |= NXC2600_CGM_CCR0_SCS_SSI_USBCLK;

        REG32(NXC2600_CGM_CCR0) |= NXC2600_CGM_CCR0_CE_UPDATE;


}

#endif

#define NXC2600_PMC				(0x10000004)
#define	NXC2600_PMC_LCR				(0x10000004)
#define		NXC2600_PMC_LCR_
#define	NXC2600_CDR				(0x10000020)
#define		NXC2600_CDR_UART0			(0x00000001)
#define		NXC2600_CDR_UART1			(0x00000002)
#define		NXC2600_CDR_UART2			(0x00000004)
#define		NXC2600_CDR_OST				(0x00000008)
#define		NXC2600_CDR_DMAC			(0x00000020)
#define		NXC2600_CDR_UHC				(0x00000040)
#define		NXC2600_CDR_LCD				(0x00000080)
#define		NXC2600_CDR_I2C				(0x00000100)
#define		NXC2600_CDR_AIC_PCLK			(0x00000200)
#define		NXC2600_CDR_PWM0			(0x00000400)
#define		NXC2600_CDR_PWM1			(0x00000800)
#define		NXC2600_CDR_SSI				(0x00001000)
#define		NXC2600_CDR_MSC				(0x00002000)
#define		NXC2600_CDR_SCC				(0x00004000)
#define		NXC2600_CDR_AIC_BITCLK			(0x00040000)
#define		NXC2600_CDR_UART3			(0x00100000)
#define		NXC2600_CDR_EMAC			(0x00200000)
#define		NXC2600_CDR_KBC				(0x00400000)
#define		NXC2600_CDR_CIM				(0x00800000)
#define		NXC2600_CDR_UDC				(0x01000000)
#define		NXC2600_CDR_UPRT			(0x02000000)

#ifndef __ASSEMBLY__
static __inline__ void
nxc2600_disable_clock(uint32_t bits)
{
	REG32(NXC2600_CDR) |= (bits);
}
static __inline__ void
nxc2600_enable_clock(uint32_t bits)
{
	REG32(NXC2600_CDR) &= ~(bits);
}
#endif

#define NXC2600_PMU_LCR				(0x10000004)
#define		NXC2600_PMU_LCR_LPM_MASK		(0x00000003)
#define		NXC2600_PMU_LCR_LPM_IDLE		(0x00000000)
#define		NXC2600_PMU_LCR_LPM_SLEEP		(0x00000001)
#define		NXC2600_PMU_LCR_LPM_HIBERNATE		(0x00000002)
#define		NXC2600_PMU_LCR_DOZE			(0x00000004)
#define		NXC2600_PMU_LCR_DUTE(div)		(((div)&0x1f)<<3)

#ifndef __ASSEMBLY__
static __inline__ void
nxc2600_setup_idle_mode(void)
{
	REG32(NXC2600_PMU_LCR) = NXC2600_PMU_LCR_LPM_IDLE;
}

static __inline__ void
nxc2600_idle_mode(void)
{
	nxc2600_stop_system_timer();
	REG32(NXC2600_PMU_LCR) = NXC2600_PMU_LCR_LPM_IDLE;
	__asm__("       .set    mips3                   \n"
                "       wait                            \n"
                "       .set    mips0                   \n");
	nxc2600_reset_system_timer();
}

static __inline__ void
nxc2600_sleep_mode(void)
{
        nxc2600_stop_system_timer();
        REG32(NXC2600_PMU_LCR) = NXC2600_PMU_LCR_LPM_SLEEP;
        __asm__("       .set    mips3                   \n"
                "       wait                            \n"
                "       .set    mips0                   \n");
        nxc2600_reset_system_timer();
}

static __inline__ void
nxc2600_dute_mode(uint32_t div)
{
	REG32(NXC2600_PMU_LCR) = NXC2600_PMU_LCR_DOZE|NXC2600_PMU_LCR_DUTE(div);
}

#endif


/* NXC2620 MAC */
#define NXC2620_MAC				(0x13101000)
#define NXC2620_MAC_BMR				(0x13101000)
#define		NXC2620_MAC_BMR_SWR			(0x00000001)
#define		NXC2620_MAC_BMR_DSL(dsl)		(((dsl)&0x1f)<<2)
#define		NXC2620_MAC_BMR_BLE_BIG_ENDIAN		(0x00000080)
#define		NXC2620_MAC_BMR_PBL(pbl)		(((pbl)&0x3f)<<8)
#define		NXC2620_MAC_BMR_DBO_BIG_ENDIAN		(0x00100000)
#define NXC2620_MAC_TPDR			(0x13101004)
#define NXC2620_MAC_RPDR			(0x13101008)
#define NXC2620_MAC_RBAR			(0x1310100c)
#define NXC2620_MAC_TBAR			(0x13101010)
#define NXC2620_MAC_SR				(0x13101014)
#define		NXC2620_MAC_SR_TI			(0x00000001)
#define 	NXC2620_MAC_SR_TPS			(0x00000002)
#define		NXC2620_MAC_SR_TU			(0x00000004)
#define		NXC2620_MAC_SR_TJT			(0x00000008)
#define		NXC2620_MAC_SR_UNF			(0x00000020)
#define		NXC2620_MAC_SR_RI			(0x00000040)
#define		NXC2620_MAC_SR_RU			(0x00000080)
#define		NXC2620_MAC_SR_RPS			(0x00000100)
#define		NXC2620_MAC_SR_RWT			(0x00000200)
#define		NXC2620_MAC_SR_ETI			(0x00000400)
#define		NXC2620_MAC_SR_FBE			(0x00002000)
#define		NXC2620_MAC_SR_ERI			(0x00004000)
#define		NXC2620_MAC_SR_AIS			(0x00008000)
#define		NXC2620_MAC_SR_NIS			(0x00010000)
#define		NXC2620_MAC_SR_RS_MASK			(0x000e0000)
#define			NXC2620_MAC_SR_RS_STOPPED		(0x00000000)
#define			NXC2620_MAC_SR_RS_FETCHING		(0x00020000)
#define			NXC2620_MAC_SR_RS_CHECK			(0x00040000)
#define			NXC2620_MAC_SR_RS_WAITING		(0x00060000)
#define			NXC2620_MAC_SR_RS_SUSPENDED		(0x00080000)
#define			NXC2620_MAC_SR_RS_CLOSING		(0x000a0000)
#define			NXC2620_MAC_SR_RS_FLUSHING		(0x000c0000)
#define			NXC2620_MAC_SR_RS_QUEUING		(0x000e0000)
#define		NXC2620_MAC_SR_TS_MASK			(0x00700000)
#define			NXC2620_MAC_SR_TS_STOPPED		(0x00000000)
#define			NXC2620_MAC_SR_TS_FETCHING		(0x00100000)
#define			NXC2620_MAC_SR_TS_WAITING		(0x00200000)
#define			NXC2620_MAC_SR_TS_QUEUING		(0x00300000)
#define		NXC2620_MAC_SR_EB_MASK			(0x03800000)
#define			NXC2620_MAC_SR_EB_NO_ERR		(0x00000000)
#define			NXC2620_MAC_SR_EB_TX_ABORT		(0x00800000)
#define			NXC2620_MAC_SR_EB_RX_ABORT		(0x01000000)
#define NXC2620_MAC_OMR				(0x13101018)
#define		NXC2620_MAC_OMR_SR			(0x00000002)
#define		NXC2620_MAC_OMR_OSF			(0x00000004)
#define		NXC2620_MAC_OMR_ST			(0x00002000)
#define		NXC2620_MAC_OMR_SF			(0x00200000)
#define		NXC2620_MAC_OMR_TTM_MASK		(0x0040c000)
#define			NXC2620_MAC_OMR_TTM_100M_WORDS_32		(0x00000000)
#define			NXC2620_MAC_OMR_TTM_100M_WORDS_64		(0x00004000)
#define			NXC2620_MAC_OMR_TTM_100M_WORDS_128		(0x00008000)
#define			NXC2620_MAC_OMR_TTM_100M_WORDS_256		(0x0000c000)
#define			NXC2620_MAC_OMR_TTM_10M_WORDS_18		(0x00400000)
#define			NXC2620_MAC_OMR_TTM_10M_WORDS_24		(0x00404000)
#define			NXC2620_MAC_OMR_TTM_10M_WORDS_32		(0x00408000)
#define			NXC2620_MAC_OMR_TTM_10M_WORDS_40		(0x0040c000)
#define NXC2620_MAC_IER				(0x1310101c)
#define		NXC2620_MAC_IER_TI			(0x00000001)
#define		NXC2620_MAC_IER_TS			(0x00000002)
#define		NXC2620_MAC_IER_TU			(0x00000004)
#define		NXC2620_MAC_IER_TJ			(0x00000008)
#define		NXC2620_MAC_IER_UN			(0x00000020)
#define		NXC2620_MAC_IER_RI			(0x00000040)
#define		NXC2620_MAC_IER_RU			(0x00000080)
#define		NXC2620_MAC_IER_RS			(0x00000100)
#define		NXC2620_MAC_IER_RWE			(0x00000200)
#define		NXC2620_MAC_IER_ET			(0x00000400)
#define		NXC2620_MAC_IER_FBE			(0x00002000)
#define		NXC2620_MAC_IER_ERE			(0x00004000)
#define		NXC2620_MAC_IER_AI			(0x00008000)
#define		NXC2620_MAC_IER_NI			(0x00010000)
#define NXC2620_MAC_MFC				(0x13101020)
#define		NXC2620_MAC_MFC_MFC_MASK		(0x0000ffff)
#define			NXC2620_MAC_MFC_MFC(mfc)		((mfc)&0xffff)
#define		NXC2620_MAC_MFC_FOC_MASK		(0x0ffe0000)
#define			NXC2620_MAC_MFC_FOC(foc)		(((foc)&0xffe)<<17)
#define NXC2620_MAC_HTAR			(0x13101050)
#define NXC2620_MAC_HRAR			(0x13101054)
#define NXC2620_MAC_CR				(0x13100000)
#define		NXC2620_MAC_CR_RE			(0x00000004)
#define		NXC2620_MAC_CR_TE			(0x00000008)
#define		NXC2620_MAC_CR_DC			(0x00000020)
#define		NXC2620_MAC_CR_BOLTM_MASK		(0x000000c0)
#define			NXC2620_MAC_CR_BOLTM_10			(0x00000000)
#define			NXC2620_MAC_CR_BOLTM_8			(0x00000040)
#define			NXC2620_MAC_CR_BOLTM_4			(0x00000080)
#define			NXC2620_MAC_CR_BOLTM_1			(0x000000c0)
#define		NXC2620_MAC_CR_ASTP			(0x00000100)
#define		NXC2620_MAC_CR_DTRY			(0x00000400)
#define		NXC2620_MAC_CR_DBF			(0x00000800)
#define		NXC2620_MAC_CR_LCC			(0x00001000)
#define		NXC2620_MAC_CR_HP			(0x00002000)
#define		NXC2620_MAC_CR_HO			(0x00008000)
#define		NXC2620_MAC_CR_PB			(0x00010000)
#define		NXC2620_MAC_CR_IF			(0x00020000)
#define		NXC2620_MAC_CR_PR			(0x00040000)
#define		NXC2620_MAC_CR_PM			(0x00080000)
#define		NXC2620_MAC_CR_FDM			(0x00100000)
#define		NXC2620_MAC_CR_OM_MASK			(0x00600000)
#define			NXC2620_MAC_CR_OM_NORMAL		(0x00000000)
#define			NXC2620_MAC_CR_OM_INTERNAL		(0x00200000)
#define			NXC2620_MAC_CR_OM_EXTERNAL		(0x00400000)
#define		NXC2620_MAC_CR_DRO			(0x00800000)
#define		NXC2620_MAC_CR_PS			(0x08000000)
#define		NXC2620_MAC_CR_HBD			(0x10000000)
#define		NXC2620_MAC_CR_RA			(0x80000000)
#define NXC2620_MAC_AHR				(0x13100004)
#define NXC2620_MAC_ALR				(0x13100008)
#define NXC2620_MAC_HTH				(0x1310000c)
#define NXC2620_MAC_HTL				(0x13100010)
#define NXC2620_MAC_MIA				(0x13100014)
#define		NXC2620_MAC_MIA_MB			(0x00000001)
#define		NXC2620_MAC_MIA_MW			(0x00000002)
#define		NXC2620_MAC_MIA_MR(mr)			(((mr)&0x1f)<<6)
#define		NXC2620_MAC_MIA_PA(pa)			(((pa)&0x1f)<<11)
#define NXC2620_MAC_MID				(0x13100018)
#define NXC2620_MAC_FCR				(0x1310001c)
#define		NXC2620_MAC_FCR_BUSY			(0x00000001)
#define		NXC2620_MAC_FCR_FCE			(0x00000002)
#define		NXC2620_MAC_FCR_PCF			(0x00000004)
#define		NXC2620_MAC_FCR_PAUSE(time)		(((time)&0xffff)<<16)
#define NXC2620_MAC_VTR1			(0x13100020)
#define NXC2620_MAC_VTR2			(0x13100024)
#define NXC2620_MAC_WKFR			(0x13100028)
#define NXC2620_MAC_PMBR			(0x1310002c)
#define		NXC2620_MAC_PMBR_MPE			(0x00000002)
#define		NXC2620_MAC_PMBR_RWPE			(0x00000004)
#define		NXC2620_MAC_PMBR_MPF			(0x00000020)
#define		NXC2620_MAC_PMBR_RWPF			(0x00000040)
#define		NXC2620_MAC_PMBR_GUPE			(0x00000200)

#ifndef __ASSEMBLY__
struct NXC2620_MAC_DMA_RX_DESC
{
        union
        {
                uint32_t        rdes0;
                struct
                {
                        uint32_t        __resv0:1;
                        uint32_t        ce:1;
                        uint32_t        db:1;
                        uint32_t        me:1;
                        uint32_t        wt:1;
                        uint32_t        ft:1;
                        uint32_t        cs:1;
                        uint32_t        ftl:1;
                        uint32_t        ls:1;
                        uint32_t        fs:1;
                        uint32_t        mf:1;
                        uint32_t        fr:1;
                        uint32_t        le:1;
                        uint32_t        __resv1:1;
                        uint32_t        de:1;
                        uint32_t        es:1;
                        uint32_t        fl:14;
                        uint32_t        ff:1;
                        uint32_t        own:1;
                };
        };

        union
        {
                uint32_t        rdes1;
                struct
                {
                        uint32_t        rbs1:11;
                        uint32_t        rbs2:11;
                        uint32_t        __resv2:2;
                        uint32_t        rch:1;
                        uint32_t        rer:1;
                        uint32_t        __resv3:6;
                };
        };

	uint32_t			buffer;
        uint32_t			next;
} __attribute__ ((packed));

struct NXC2620_MAC_DMA_TX_DESC
{
        union
        {
                uint32_t        tdes0;
                struct
                {
                        uint32_t        dfr:1;
                        uint32_t        uf:1;
                        uint32_t        ed:1;
                        uint32_t        cc:4;
                        uint32_t        hbf:1;
                        uint32_t        ec:1;
                        uint32_t        lc:1;
                        uint32_t        nc:1;
                        uint32_t        loc:1;
                        uint32_t        __resv0:3;
                        uint32_t        fa:1;
                        uint32_t        __resv1:15;
                        uint32_t        own:1;
                };
        };

        union
        {
                uint32_t        tdes1;
                struct
                {
                        uint32_t        tbs1:11;
                        uint32_t        tbs2:11;
                        uint32_t        __resv2:1;
                        uint32_t        dpd:1;
                        uint32_t        tch:1;
                        uint32_t        ter:1;
                        uint32_t        acd:1;
                        uint32_t        __resv3:2;
                        uint32_t        fs:1;
                        uint32_t        ls:1;
                        uint32_t        ioc:1;
                };
        };

        uint32_t                        buffer;
        uint32_t			next;
} __attribute__ ((packed));

#ifndef NXC2620_MAC_DMA_MAX_TX_DESC
#define NXC2620_MAC_DMA_MAX_TX_DESC 16
#endif

#ifndef NXC2620_MAC_DMA_MAX_RX_DESC
#define NXC2620_MAC_DMA_MAX_RX_DESC 32
#endif

struct NXC2620_MAC_DMA_DESC
{
	struct NXC2620_MAC_DMA_TX_DESC	tx[NXC2620_MAC_DMA_MAX_TX_DESC];
	struct NXC2620_MAC_DMA_RX_DESC  rx[NXC2620_MAC_DMA_MAX_RX_DESC];

	uint32_t tx_head;
	uint32_t tx_queued;
	void *tx_buffer[NXC2620_MAC_DMA_MAX_TX_DESC];

	uint32_t rx_head;
	uint32_t rx_finish;
	void *rx_buffer[NXC2620_MAC_DMA_MAX_RX_DESC];

	
};

static __inline__ void
nxc2620_mac_set_addr( unsigned char *addr )
{
	unsigned int a;
	a = addr[3];
	a = (a<<8)|addr[2];
	a = (a<<8)|addr[1];
	a = (a<<8)|addr[0];
	REG32(NXC2620_MAC_ALR) = a;
	a = addr[5];
	a = (a<<8)|addr[4];
	REG32(NXC2620_MAC_AHR) = a;
}

static __inline__ void
nxc2620_mac_get_addr( unsigned char *addr )
{
	unsigned int a;
	a = REG32(NXC2620_MAC_AHR);
	addr[4] = a&0xff;
	addr[5] = (a>>8)&0xff;
	a = REG32(NXC2620_MAC_ALR);
	addr[0] = a&0xff;
	addr[1] = (a>>8)&0xff;
	addr[2] = (a>>16)&0xff;
	addr[3] = (a>>24)&0xff;
}

static __inline__ int
nxc2620_mac_wait_for_mdio_finished( uint32_t timeout_msecs )
{
	if( timeout_msecs > 100000 ) timeout_msecs = 100000;
	while( REG32(NXC2620_MAC_MIA) & NXC2620_MAC_MIA_MB )
	{
		udelay(3);
		if(!timeout_msecs--) return -1;
	}
	return 0;
}

static __inline__ int
nxc2620_mac_mdio_write( uint32_t phy_id,
			uint32_t reg,
			uint32_t val )
{

	REG32(NXC2620_MAC_MID) = val;
	REG32(NXC2620_MAC_MIA) = NXC2620_MAC_MIA_PA(phy_id) | NXC2620_MAC_MIA_MR(reg) | NXC2620_MAC_MIA_MB | NXC2620_MAC_MIA_MW;
	return nxc2620_mac_wait_for_mdio_finished( 300 );
}

static __inline__ int
nxc2620_mac_mdio_read( uint32_t phy_id,
			uint32_t reg )
{
	REG32(NXC2620_MAC_MIA) = (NXC2620_MAC_MIA_PA(phy_id) | NXC2620_MAC_MIA_MR(reg) | NXC2620_MAC_MIA_MB);
	return (( nxc2620_mac_wait_for_mdio_finished( 300 ) )? -1:REG32(NXC2620_MAC_MID));
}

static __inline__ int
nxc2620_mac_search_phy_id( void )
{
        int phy_id;
        int data;
#ifndef MII_BMSR
#define MII_BMSR 0x01
#endif
        for(phy_id = 0; phy_id<32; phy_id++)
        {
                data = nxc2620_mac_mdio_read( phy_id, MII_BMSR );
                if( (data != 0xffff) && (data != 0x0000) )
			return phy_id;
        }
        return -1;
}

extern void*
nxc2620_mac_alloc_buffer( void **data,
			  uint32_t size );

extern void
nxc2620_mac_free_buffer( void *buf );

extern int
nxc2620_mac_rx( void *priv );

extern int
nxc2620_mac_tx( void *priv,
                uint32_t slot,
                uint32_t len );


static __inline__ void
nxc2620_mac_change_mode( int duplex,
                         int s100m )
{
	REG32(NXC2620_MAC_IER) = 0;
        REG32(NXC2620_MAC_OMR) = (REG32(NXC2620_MAC_OMR)&~(NXC2620_MAC_OMR_TTM_MASK)) | ((s100m)? NXC2620_MAC_OMR_TTM_100M_WORDS_32:NXC2620_MAC_OMR_TTM_10M_WORDS_18);

	REG32(NXC2620_MAC_MFC);
	REG32(NXC2620_MAC_BMR) = NXC2620_MAC_BMR_PBL(4);
	REG32(NXC2620_MAC_CR) = NXC2620_MAC_CR_DBF|NXC2620_MAC_CR_RE|NXC2620_MAC_CR_TE| ((duplex)? NXC2620_MAC_CR_FDM:0);


        REG32(NXC2620_MAC_IER) = NXC2620_MAC_IER_FBE|NXC2620_MAC_IER_RS|
                                 NXC2620_MAC_IER_RU|NXC2620_MAC_IER_RI|
                                 NXC2620_MAC_IER_TI|NXC2620_MAC_IER_AI|
                                 NXC2620_MAC_IER_TU|NXC2620_MAC_IER_NI|
				 NXC2620_MAC_IER_TS;

}

static __inline__ int
nxc2620_mac_init( void )
{
	unsigned char addr[6];

	nxc2620_mac_get_addr(addr);
	REG32(NXC2620_MAC_BMR) |= NXC2620_MAC_BMR_SWR;
	while(REG32(NXC2620_MAC_BMR) & NXC2620_MAC_BMR_SWR) udelay(100);
	nxc2620_mac_set_addr(addr);

	REG32(NXC2620_MAC_IER) = 0;
	REG32(NXC2620_MAC_OMR)	= NXC2620_MAC_OMR_SF|NXC2620_MAC_OMR_TTM_100M_WORDS_32|NXC2620_MAC_OMR_OSF;
	REG32(NXC2620_MAC_MFC);
	
	REG32(NXC2620_MAC_BMR) |= NXC2620_MAC_BMR_PBL(4);
	REG32(NXC2620_MAC_CR) = NXC2620_MAC_CR_DBF|NXC2620_MAC_CR_RE|NXC2620_MAC_CR_TE|NXC2620_MAC_CR_FDM;


	return 0;
}

static __inline__ void
nxc2620_mac_stop( void )
{
	REG32(NXC2620_MAC_IER) = 0;
}

static __inline__ int
nxc2620_mac_dma_desc_init( struct NXC2620_MAC_DMA_DESC *desc )
{
	int i;
	void *data;
	memset((void*)desc, 0, sizeof(struct NXC2620_MAC_DMA_DESC));
	for( i = 0; i < NXC2620_MAC_DMA_MAX_TX_DESC; i++ )
	{
		desc->tx[i].buffer = 0;
		desc->tx[i].own = 0;
		desc->tx[i].next = virt_to_phys(&(desc->tx[i+1]));
		desc->tx[i].tch = 0;
		desc->tx[i].ioc = 0;
		desc->tx_buffer[i] = NULL;
	}
	desc->tx[NXC2620_MAC_DMA_MAX_TX_DESC-1].next = virt_to_phys(&(desc->tx[0]));
	desc->tx[NXC2620_MAC_DMA_MAX_TX_DESC-1].ter = 1;
	desc->tx_head = 0;
	desc->tx_queued = 0;
	REG32(NXC2620_MAC_TBAR) = virt_to_phys(&(desc->tx[0]));

	for( i = 0; i < NXC2620_MAC_DMA_MAX_RX_DESC; i++ )
        {
		desc->rx_buffer[i] = nxc2620_mac_alloc_buffer( &data, 1536 );
		if( desc->rx_buffer[i] == NULL )
		{
			goto _rx_alloc_buffer_err;
		}
		desc->rx[i].buffer = virt_to_phys( data );
		desc->rx[i].rbs1 = 1536;
		desc->rx[i].next = virt_to_phys(&(desc->rx[i+1]));
                desc->rx[i].own = 1;
		//desc->rx[i].wt = 1;
        }
        desc->rx[NXC2620_MAC_DMA_MAX_RX_DESC-1].next = virt_to_phys(&(desc->rx[0]));
	desc->rx[NXC2620_MAC_DMA_MAX_RX_DESC-1].ls = 1;
	desc->rx[NXC2620_MAC_DMA_MAX_RX_DESC-1].rer = 1;
	desc->rx[0].fs = 1;
	desc->rx_head = 0;
	desc->rx_finish = 0;
	REG32(NXC2620_MAC_RBAR) = virt_to_phys( &(desc->rx[0]) );
	flush_cache_all();
	REG32(NXC2620_MAC_OMR) |= NXC2620_MAC_OMR_ST|NXC2620_MAC_OMR_SR;

        REG32(NXC2620_MAC_IER) = NXC2620_MAC_IER_FBE|NXC2620_MAC_IER_RS|
                                 NXC2620_MAC_IER_RU|NXC2620_MAC_IER_RI|
                                 NXC2620_MAC_IER_TI|NXC2620_MAC_IER_AI|
                                 NXC2620_MAC_IER_TU|NXC2620_MAC_IER_NI|
                                 NXC2620_MAC_IER_TS;

	REG32(NXC2620_MAC_RPDR) = 1;
	return 0;

_rx_alloc_buffer_err:
	for( i = 0; i < NXC2620_MAC_DMA_MAX_RX_DESC; i++ )
	{
		if( desc->rx_buffer[i] == NULL ) break;
		nxc2620_mac_free_buffer( desc->rx_buffer[i] );
	}

	return -1;
}

static __inline__ void
nxc2620_mac_dma_desc_free( struct NXC2620_MAC_DMA_DESC *desc )
{
	int i;
	REG32(NXC2620_MAC_IER) = 0;
	for( i = 0; i < NXC2620_MAC_DMA_MAX_TX_DESC; i++)
	{
		if( desc->tx_buffer[i] != NULL )
                {
                        nxc2620_mac_free_buffer( desc->tx_buffer[i] );
                        desc->tx_buffer[i] = NULL;
                }

		desc->tx[i].own = 0;
	}
	for( i = 0; i < NXC2620_MAC_DMA_MAX_RX_DESC; i++ )
        {
                if( desc->rx_buffer[i] != NULL )
		{
			nxc2620_mac_free_buffer( desc->rx_buffer[i] );
			desc->rx_buffer[i] = NULL;
		}
		desc->rx[i].own = 0;
        }
}

static __inline__ void
nxc2620_mac_rx_finished( struct NXC2620_MAC_DMA_DESC *desc )
{
	void *data;

	if(desc->rx[desc->rx_finish].own)
		return;
	desc->rx_buffer[desc->rx_finish] = nxc2620_mac_alloc_buffer( &data, 1536);

	if( desc->rx_buffer[desc->rx_finish] == NULL )
		return;

	desc->rx[desc->rx_finish].buffer = virt_to_phys( data );
        desc->rx[desc->rx_finish].rbs1 = 1536;
	desc->rx[desc->rx_finish].own = 1;
	
        desc->rx_finish++;
	if(desc->rx_finish>=NXC2620_MAC_DMA_MAX_RX_DESC)
                desc->rx_finish=0;

}

static __inline__ void
nxc2620_mac_rx_ready( void *priv,
		      struct NXC2620_MAC_DMA_DESC *desc )
{
	while( ! desc->rx[desc->rx_head].own )
	{

#if 0
		if( desc->rx[desc->rx_head].es )
			nxc2620_mac_rx_error( priv );
		else
#endif

		if(desc->rx_buffer[desc->rx_head]!=NULL)
			nxc2620_mac_rx( priv );

		nxc2620_mac_rx_finished(desc);

	        desc->rx_head++;
		if(desc->rx_head>=NXC2620_MAC_DMA_MAX_RX_DESC)
			desc->rx_head=0;

	}
	flush_cache_all();
}

static __inline__ int
nxc2620_mac_tx_done( void *priv,
		     struct NXC2620_MAC_DMA_DESC *desc )
{
	while( desc->tx_queued && !desc->tx[desc->tx_head].own )
	{

		nxc2620_mac_tx(priv, desc->tx_head, desc->tx[desc->tx_head].tbs1 );
                nxc2620_mac_free_buffer( desc->tx_buffer[desc->tx_head] );
		desc->tx_buffer[desc->tx_head] = NULL;
		desc->tx[desc->tx_head].buffer = 0;
		desc->tx[desc->tx_head].own = 0;
		desc->tx_queued--;
		desc->tx_head++;
		if(desc->tx_head>=NXC2620_MAC_DMA_MAX_TX_DESC)
			desc->tx_head = 0;
	}
	return 0;
}

static __inline__ int
nxc2620_mac_send( struct NXC2620_MAC_DMA_DESC *desc,
		  void *data,
		  uint32_t len )
{
	int slot;
	if(desc->tx_queued == NXC2620_MAC_DMA_MAX_TX_DESC)
	{
		printk("nxc2620 mac: queue is full.\n");
		return -1;
	}
	if(len>2047)
	{
		printk("nxc2620 mac: packet size is too long.\n");
		return -1;
	}

	slot = (desc->tx_head+desc->tx_queued);
	if(slot>=NXC2620_MAC_DMA_MAX_TX_DESC)
		slot-=NXC2620_MAC_DMA_MAX_TX_DESC;

	desc->tx[slot].buffer = virt_to_phys(data);
	desc->tx[slot].own = 0;
	desc->tx[slot].tbs1 = len;
	desc->tx[slot].ls = 1;
	desc->tx[slot].fs = 1;
	desc->tx[slot].own = 1;
	desc->tx_queued++;
	flush_cache_all();

	REG32(NXC2620_MAC_TPDR) = 1;

	return slot;
}

static __inline__ int
nxc2620_mac_dma_poll( void *priv,
		      struct NXC2620_MAC_DMA_DESC *desc )
{
	uint32_t sr;

	sr = REG32(NXC2620_MAC_SR);
	REG32(NXC2620_MAC_SR) = sr;

	if( sr & (NXC2620_MAC_SR_TI|NXC2620_MAC_SR_TU) )
                nxc2620_mac_tx_done( priv, desc );

	if( sr & (NXC2620_MAC_SR_RU|NXC2620_MAC_SR_RI) )
		nxc2620_mac_rx_ready( priv, desc );

	if( sr & NXC2620_MAC_SR_RU )
		REG32(NXC2620_MAC_RPDR) = 1;

	return 0;
}

#endif





#define NXC2600_OHCI_HCD			(0x13030000)


#define NXC2600_DMA(ch)			(0x13020000+(0x20*(ch)))
#define NXC2600_DMA_SA(ch)			(NXC2600_DMA(ch)+0x00)
#define	NXC2600_DMA_DA(ch)			(NXC2600_DMA(ch)+0x04)
#define	NXC2600_DMA_TC(ch)			(NXC2600_DMA(ch)+0x08)
#define	NXC2600_DMA_RT(ch)			(NXC2600_DMA(ch)+0x0c)
#define		NXC2600_DMA_RT_EXT_REQUEST		(0x00000000)
#define		NXC2600_DMA_RT_PCMCIA_OUT		(0x00000004)
#define		NXC2600_DMA_RT_PCMCIA_IN		(0x00000005)
#define		NXC2600_DMA_RT_AUTO_REQUEST		(0x00000008)
#define		NXC2600_DMA_RT_DES_OUT			(0x0000000a)
#define		NXC2600_DMA_RT_DES_IN			(0x0000000b)
#define		NXC2600_DMA_RT_UART3_TX			(0x0000000e)
#define		NXC2600_DMA_RT_UART3_RX			(0x0000000f)
#define		NXC2600_DMA_RT_UART2_TX			(0x00000010)
#define		NXC2600_DMA_RT_UART2_RX			(0x00000011)
#define		NXC2600_DMA_RT_UART1_TX			(0x00000012)
#define		NXC2600_DMA_RT_UART1_RX			(0x00000013)
#define 	NXC2600_DMA_RT_UART0_TX			(0x00000014)
#define		NXC2600_DMA_RT_UART0_RX			(0x00000015)
#define		NXC2600_DMA_RT_SSI_TX			(0x00000016)
#define		NXC2600_DMA_RT_SSI_RX			(0x00000017)
#define		NXC2600_DMA_RT_AIC_TX			(0x00000018)
#define		NXC2600_DMA_RT_AIC_RX			(0x00000019)
#define		NXC2600_DMA_RT_MSC_TX			(0x0000001a)
#define		NXC2600_DMA_RT_MSC_RX			(0x0000001b)
#define		NXC2600_DMA_RT_OST			(0x0000001c)
#define NXC2600_DMA_DCS(ch)                     (NXC2600_DMA(ch)+0x10)
#define		NXC2600_DMA_DCS_CTE			(0x00000001)
#define         NXC2600_DMA_DCS_TIE                     (0x00000002)
#define		NXC2600_DMA_DCS_HLT			(0x00000004)
#define		NXC2600_DMA_DCS_CT			(0x00000008)
#define		NXC2600_DMA_DCS_AR			(0x00000010)
#define		NXC2600_DMA_DCS_TM			(0x00000080)
#define		NXC2600_DMA_DCS_TSZ_MASK		(0x00000700)
#define		NXC2600_DMA_DCS_TSZ_32_BIT		(0x00000000)
#define		NXC2600_DMA_DCS_TSZ_8_BIT		(0x00000100)
#define		NXC2600_DMA_DCS_TSZ_16_BIT		(0x00000200)
#define		NXC2600_DMA_DCS_TSZ_16_BYTE		(0x00000300)
#define		NXC2600_DMA_DCS_TSZ_32_BYTE		(0x00000400)
#define		NXC2600_DMA_DCS_DP_MASK			(0x00003000)
#define		NXC2600_DMA_DCS_DP_32_BIT		(0x00000000)
#define		NXC2600_DMA_DCS_DP_8_BIT		(0x00001000)
#define		NXC2600_DMA_DCS_DP_16_BIT		(0x00002000)
#define		NXC2600_DMA_DCS_SP_MASK			(0x0000c000)
#define		NXC2600_DMA_DCS_SP_32_BIT		(0x00000000)
#define		NXC2600_DMA_DCS_SP_8_BIT		(0x00004000)
#define		NXC2600_DMA_DCS_SP_16_BIT		(0x00008000)
#define		NXC2600_DMA_DCS_RDIL_MASK		(0x000f0000)
#define		NXC2600_DMA_DCS_RDIL_0			(0x00000000)
#define		NXC2600_DMA_DCS_RDIL_2			(0x00010000)
#define		NXC2600_DMA_DCS_RDIL_4			(0x00020000)
#define		NXC2600_DMA_DCS_RDIL_8			(0x00030000)
#define		NXC2600_DMA_DCS_RDIL_12			(0x00040000)
#define		NXC2600_DMA_DCS_RDIL_16			(0x00050000)
#define		NXC2600_DMA_DCS_RDIL_20			(0x00060000)
#define		NXC2600_DMA_DCS_RDIL_24			(0x00070000)
#define		NXC2600_DMA_DCS_RDIL_28			(0x00080000)
#define		NXC2600_DMA_DCS_RDIL_32			(0x00090000)
#define		NXC2600_DMA_DCS_RDIL_48			(0x000a0000)
#define		NXC2600_DMA_DCS_RDIL_60			(0x000b0000)
#define		NXC2600_DMA_DCS_RDIL_64			(0x000c0000)
#define		NXC2600_DMA_DCS_RDIL_124		(0x000d0000)
#define		NXC2600_DMA_DCS_RDIL_128		(0x000e0000)
#define		NXC2600_DMA_DCS_RDIL_200		(0x000f0000)
#define		NXC2600_DMA_DCS_DAI			(0x00400000)
#define		NXC2600_DMA_DCS_SAI			(0x00800000)
#define		NXC2600_DMA_DCS_EOPM			(0x08000000)
#define		NXC2600_DMA_DCS_ERDM_MASK		(0x30000000)
#define		NXC2600_DMA_DCS_ERDM_LOW		(0x00000000)
#define		NXC2600_DMA_DCS_ERDM_FALLING		(0x10000000)
#define		NXC2600_DMA_DCS_ERDM_HIGH		(0x20000000)
#define		NXC2600_DMA_DCS_ERDM_RISING		(0x30000000)
#define		NXC2600_DMA_DCS_EACKM			(0x40000000)
#define		NXC2600_DMA_DCS_EACKS			(0x80000000)
#define NXC2600_DMA_IP                  (0x130200f8)
#define		NXC2600_DMA_IP_CIRQ(n)			(1<<(15-(n)))
#define	NXC2600_DMA_CTRL			(0x130200fc)
#define		NXC2600_DMA_CTRL_ENABLE			(0x00000001)
#define		NXC2600_DMA_CTRL_AR			(0x00000004)
#define		NXC2600_DMA_CTRL_HLT			(0x00000008)
#define		NXC2600_DMA_CTRL_PM_01234567		(0x00000000)
#define		NXC2600_DMA_CTRL_PM_02314675		(0x00000001)
#define		NXC2600_DMA_CTRL_PM_20136457		(0x00000002)
#define		NXC2600_DMA_CTRL_PM_ROUNDROBIN		(0x00000003)
#define		NXC2600_DMA_CTRL_PM(bits)		(((bits)&0x3)<<8)
#define NXC2600_DMA_BASE_IRQ            (32+4*32)

#ifndef __ASSEMBLY__

/* DMA operations */

#define	NXC2600_DMA_MAX_BYTE_COUNT	(0x00ffffff)

struct nxc2600_dma_mode
{
	uint32_t type;
	uint32_t addr;
	uint32_t mode;
	void (*set_packet) (struct nxc2600_dma_mode*);
};

#define NXC2600_DMA_MODE_MEMSET	\
	{	.type	=	NXC2600_DMA_RT_AUTO_REQUEST,	\
		.addr	=	0,	\
		.mode	=	NXC2600_DMA_DCS_TSZ_8_BIT |	\
				NXC2600_DMA_DCS_SP_8_BIT |	\
				NXC2600_DMA_DCS_DP_8_BIT |	\
				NXC2600_DMA_DCS_DAI |		\
				NXC2600_DMA_DCS_RDIL_0,		\
		.set_packet	=	NULL,	\
	}

#define	NXC2600_DMA_MODE_MEMCPY	\
	{	.type	=	NXC2600_DMA_RT_AUTO_REQUEST,	\
		.addr	=	0,	\
		.mode	=	NXC2600_DMA_DCS_TSZ_8_BIT |     \
                                NXC2600_DMA_DCS_SP_8_BIT |     \
                                NXC2600_DMA_DCS_DP_8_BIT |     \
                                NXC2600_DMA_DCS_DAI |           \
				NXC2600_DMA_DCS_SAI |		\
				NXC2600_DMA_DCS_EACKM |		\
                                NXC2600_DMA_DCS_RDIL_0,         \
		.set_packet	=	NULL,	\
        }

extern void nxc2600_aic_set_dma_mode_tx_packet(struct nxc2600_dma_mode*);
#define NXC2600_DMA_MODE_AIC_TX	\
	{       .type   =       NXC2600_DMA_RT_AIC_TX,		\
                .addr   =       NXC2600_AIC_DR,	\
		.mode   =       \
                                NXC2600_DMA_DCS_SAI |		\
				NXC2600_DMA_DCS_RDIL_0,		\
		.set_packet	=	nxc2600_aic_set_dma_mode_tx_packet, \
        }

extern void nxc2600_aic_set_dma_mode_rx_packet(struct nxc2600_dma_mode*);
#define NXC2600_DMA_MODE_AIC_RX \
        {       .type   =       NXC2600_DMA_RT_AIC_RX,          \
                .addr   =       NXC2600_AIC_DR,      \
                .mode   =       \
                                NXC2600_DMA_DCS_DAI |           \
                                NXC2600_DMA_DCS_RDIL_0,         \
		.set_packet	=	nxc2600_aic_set_dma_mode_rx_packet, \
        }

#define	NXC2600_DMA_MODE_MMC_TX	\
	{	.type	=	NXC2600_DMA_RT_MSC_TX,		\
		.addr	=	NXC2600_MMC_TXFIFO,		\
		.mode	=	NXC2600_DMA_DCS_TSZ_32_BIT |    \
				NXC2600_DMA_DCS_SAI | 		\
				NXC2600_DMA_DCS_DP_32_BIT |	\
				NXC2600_DMA_DCS_SP_32_BIT |	\
				NXC2600_DMA_DCS_RDIL_0|		\
				NXC2600_DMA_DCS_EACKM,		\
		.set_packet	=	NULL,			\
	}

#define NXC2600_DMA_MODE_MMC_RX \
        {       .type   =       NXC2600_DMA_RT_MSC_RX,          \
                .addr   =       NXC2600_MMC_RXFIFO,             \
                .mode   =       NXC2600_DMA_DCS_TSZ_32_BIT |	\
				NXC2600_DMA_DCS_DP_32_BIT |	\
				NXC2600_DMA_DCS_SP_32_BIT |	\
				NXC2600_DMA_DCS_DAI |           \
				NXC2600_DMA_DCS_RDIL_0,		\
                .set_packet     =      NULL,			\
        }

#define NXC2600_DMA_MODE_UART0_TX \
	{	.type	=	NXC2600_DMA_RT_UART0_TX,	\
		.addr	=	NXC2600_UART_THR(0),		\
		.mode	=	NXC2600_DMA_DCS_TSZ_8_BIT |	\
				NXC2600_DMA_DCS_SP_8_BIT |	\
				NXC2600_DMA_DCS_DP_8_BIT |	\
				NXC2600_DMA_DCS_SAI |		\
				NXC2600_DMA_DCS_RDIL_0,		\
		.set_packet	=	NULL,			\
	}

#define	NXC2600_DMA_MODE_UART0_RX \
	{       .type   =       NXC2600_DMA_RT_UART0_RX,        \
                .addr   =       NXC2600_UART_RBR(0),            \
                .mode   =       NXC2600_DMA_DCS_TSZ_8_BIT |     \
                                NXC2600_DMA_DCS_SP_8_BIT |      \
                                NXC2600_DMA_DCS_DP_8_BIT |      \
                                NXC2600_DMA_DCS_DAI |           \
                                NXC2600_DMA_DCS_RDIL_0,         \
                .set_packet     =       NULL,                   \
        }

#define NXC2600_DMA_MODE_UART1_TX \
        {       .type   =       NXC2600_DMA_RT_UART1_TX,        \
                .addr   =       NXC2600_UART_THR(1),            \
                .mode   =       NXC2600_DMA_DCS_TSZ_8_BIT |     \
                                NXC2600_DMA_DCS_SP_8_BIT |      \
                                NXC2600_DMA_DCS_DP_8_BIT |      \
                                NXC2600_DMA_DCS_SAI |           \
                                NXC2600_DMA_DCS_RDIL_0,         \
                .set_packet     =       NULL,                   \
        }

#define NXC2600_DMA_MODE_UART1_RX \
        {       .type   =       NXC2600_DMA_RT_UART1_RX,        \
                .addr   =       NXC2600_UART_RBR(1),            \
                .mode   =       NXC2600_DMA_DCS_TSZ_8_BIT |     \
                                NXC2600_DMA_DCS_SP_8_BIT |      \
                                NXC2600_DMA_DCS_DP_8_BIT |      \
                                NXC2600_DMA_DCS_DAI |           \
                                NXC2600_DMA_DCS_RDIL_0,         \
                .set_packet     =       NULL,                   \
        }

#define NXC2600_DMA_MODE_UART2_TX \
        {       .type   =       NXC2600_DMA_RT_UART2_TX,        \
                .addr   =       NXC2600_UART_THR(2),            \
                .mode   =       NXC2600_DMA_DCS_TSZ_8_BIT |     \
                                NXC2600_DMA_DCS_SP_8_BIT |      \
                                NXC2600_DMA_DCS_DP_8_BIT |      \
                                NXC2600_DMA_DCS_SAI |           \
                                NXC2600_DMA_DCS_RDIL_0,         \
                .set_packet     =       NULL,                   \
        }

#define NXC2600_DMA_MODE_UART2_RX \
        {       .type   =       NXC2600_DMA_RT_UART2_RX,        \
                .addr   =       NXC2600_UART_RBR(2),            \
                .mode   =       NXC2600_DMA_DCS_TSZ_8_BIT |     \
                                NXC2600_DMA_DCS_SP_8_BIT |      \
                                NXC2600_DMA_DCS_DP_8_BIT |      \
                                NXC2600_DMA_DCS_DAI |           \
                                NXC2600_DMA_DCS_RDIL_0,         \
                .set_packet     =       NULL,                   \
        }
#define NXC2600_DMA_MODE_UART3_TX \
        {       .type   =       NXC2600_DMA_RT_UART3_TX,        \
                .addr   =       NXC2600_UART_THR(3),            \
                .mode   =       NXC2600_DMA_DCS_TSZ_8_BIT |     \
                                NXC2600_DMA_DCS_SP_8_BIT |      \
                                NXC2600_DMA_DCS_DP_8_BIT |      \
                                NXC2600_DMA_DCS_SAI |           \
                                NXC2600_DMA_DCS_RDIL_0,         \
                .set_packet     =       NULL,                   \
        }

#define NXC2600_DMA_MODE_UART3_RX \
        {       .type   =       NXC2600_DMA_RT_UART3_RX,        \
                .addr   =       NXC2600_UART_RBR(3),            \
                .mode   =       NXC2600_DMA_DCS_TSZ_8_BIT |     \
                                NXC2600_DMA_DCS_SP_8_BIT |      \
                                NXC2600_DMA_DCS_DP_8_BIT |      \
                                NXC2600_DMA_DCS_DAI |           \
                                NXC2600_DMA_DCS_RDIL_0,         \
                .set_packet     =       NULL,                   \
        }

static __inline__ void
nxc2600_dma_init( void )
{
	REG32(NXC2600_DMA_CTRL) = NXC2600_DMA_CTRL_ENABLE |
                                  NXC2600_DMA_CTRL_PM(NXC2600_DMA_CTRL_PM_ROUNDROBIN);
}

static __inline__ void
nxc2600_dma_disable(uint32_t ch)
{
	if(!(REG32(NXC2600_DMA_DCS(ch))&NXC2600_DMA_DCS_CTE))
		return;
	REG32(NXC2600_DMA_DCS(ch)) &= ~NXC2600_DMA_DCS_CTE;

}

static __inline__ void
nxc2600_dma_set_mode( uint32_t ch,
		      uint32_t count,
		      const struct nxc2600_dma_mode *mode )
{
	static const int tsz[] = { 4, 1, 2, 16, 32 };

	count /= tsz[(mode->mode&NXC2600_DMA_DCS_TSZ_MASK)>>8];

	if(count<1) count=1;
	//REG32(NXC2600_DMA_DCS(ch)) = 0;
	REG32(NXC2600_DMA_TC(ch)) = count&0x00ffffff;
	REG32(NXC2600_DMA_DCS(ch)) = mode->mode;
	REG32(NXC2600_DMA_RT(ch)) = mode->type;
}

static __inline__ void
nxc2600_dma_set_src( uint32_t ch,
		     void *src )
{
	REG32(NXC2600_DMA_SA(ch)) = CPHYSADDR((uint32_t)src);
}

static __inline__ void
nxc2600_dma_set_dest( uint32_t ch,
		      void *dest )
{
	REG32(NXC2600_DMA_DA(ch)) = CPHYSADDR((uint32_t)dest);
}

static __inline__ void
nxc2600_dma_start_transfer( uint32_t ch )
{
	REG32(NXC2600_DMA_IP) &= ~NXC2600_DMA_IP_CIRQ(ch);
	REG32(NXC2600_DMA_DCS(ch)) = (REG32(NXC2600_DMA_DCS(ch))&~(NXC2600_DMA_DCS_CT|NXC2600_DMA_DCS_HLT|NXC2600_DMA_DCS_AR)) |
				    (NXC2600_DMA_DCS_CTE | NXC2600_DMA_DCS_TIE);
}

static __inline__ void
nxc2600_dma_cleanup_interrupt( uint32_t ch )
{
	uint32_t val = REG32(NXC2600_DMA_DCS(ch))&~(NXC2600_DMA_DCS_HLT|NXC2600_DMA_DCS_AR);
        REG32(NXC2600_DMA_IP) &= ~NXC2600_DMA_IP_CIRQ(ch);
	//REG32(NXC2600_DMA_DCS(ch)) &= ~NXC2600_DMA_DCS_TIE;
	if(val&NXC2600_DMA_DCS_CT)
	{
		val &= ~(NXC2600_DMA_DCS_CT|NXC2600_DMA_DCS_CTE); //~(NXC2600_DMA_DCS_TIE);
	}
	REG32(NXC2600_DMA_DCS(ch)) = val;
}

static __inline__ uint32_t
nxc2600_dma_get_current_transfer_count( uint32_t ch,
					uint32_t timeout_10usec )
{
	static const uint32_t tsz[] = { 4, 1, 2, 16, 32 };

	if(timeout_10usec==0)
		timeout_10usec = 1;

	while(REG32(NXC2600_DMA_TC(ch)) || !(REG32(NXC2600_DMA_DCS(ch))&NXC2600_DMA_DCS_CT))
        {
		if(!--timeout_10usec)
			break;
                udelay(10);
        }
	return (tsz[(REG32(NXC2600_DMA_DCS(ch))&NXC2600_DMA_DCS_TSZ_MASK)>>8]*REG32(NXC2600_DMA_TC(ch)));
}

#endif




#define	NXC2600_AIC				(0x10020000)
#define	NXC2600_AIC_FR				(0x10020000)
#define		NXC2600_AIC_FR_ENB			(0x00000001)
#define		NXC2600_AIC_FR_SYNCD			(0x00000002)
#define		NXC2600_AIC_FR_BCKD			(0x00000004)
#define		NXC2600_AIC_FR_RST			(0x00000008)
#define		NXC2600_AIC_FR_AUSEL			(0x00000010)
#define		NXC2600_AIC_FR_TFTH_MASK		(0x00000f00)
#define		NXC2600_AIC_FR_TFTH(value)		(((value)<<8)&NXC2600_AIC_FR_TFTH_MASK)
#define		NXC2600_AIC_FR_RFTH_MASK		(0x0000f000)
#define		NXC2600_AIC_FR_RFTH(value)		(((value)<<12)&NXC2600_AIC_FR_RFTH_MASK)
#define	NXC2600_AIC_CR				(0x10020004)
#define		NXC2600_AIC_CR_EREC			(0x00000001)
#define		NXC2600_AIC_CR_ERPL			(0x00000002)
#define		NXC2600_AIC_CR_ENLBF			(0x00000004)
#define		NXC2600_AIC_CR_ETFS			(0x00000008)
#define		NXC2600_AIC_CR_ERFS			(0x00000010)
#define		NXC2600_AIC_CR_ETUR			(0x00000020)
#define		NXC2600_AIC_CR_EROR			(0x00000040)
#define		NXC2600_AIC_CR_FLUSH			(0x00000100)
#define		NXC2600_AIC_CR_ASVTSU			(0x00000200)
#define		NXC2600_AIC_CR_ENDSW			(0x00000300)
#define		NXC2600_AIC_CR_M2S			(0x00000400)
#define		NXC2600_AIC_CR_TDMS			(0x00004000)
#define		NXC2600_AIC_CR_RDMS			(0x00008000)
#define		NXC2600_AIC_CR_ISS_MASK			(0x00070000)
#define		NXC2600_AIC_CR_ISS_8_BIT		(0x00000000)
#define		NXC2600_AIC_CR_ISS_16_BIT		(0x00010000)
#define		NXC2600_AIC_CR_ISS_18_BIT		(0x00020000)
#define		NXC2600_AIC_CR_ISS_20_BIT		(0x00030000)
#define		NXC2600_AIC_CR_ISS_24_BIT		(0x00040000)
#define		NXC2600_AIC_CR_OSS_MASK			(0x00380000)
#define		NXC2600_AIC_CR_OSS_8_BIT		(0x00000000)
#define		NXC2600_AIC_CR_OSS_16_BIT		(0x00080000)
#define		NXC2600_AIC_CR_OSS_18_BIT		(0x00100000)
#define		NXC2600_AIC_CR_OSS_20_BIT		(0x00180000)
#define		NXC2600_AIC_CR_OSS_24_BIT		(0x00200000)
#define	NXC2600_AIC_CR1				(0x10020008)
#define		NXC2600_AIC_CR1_XS_MASK			(0x000003ff)
#define		NXC2600_AIC_CR1_XS(slot)		(1<<((slot)-3))	
#define		NXC2600_AIC_CR1_RS_MASK			(0x03ff0000)
#define		NXC2600_AIC_CR1_RS(slot)		(1<<(16+((slot)-3)))
#define	NXC2600_AIC_CR2				(0x1002000c)
#define		NXC2600_AIC_CR2_SA			(0x00000001)
#define		NXC2600_AIC_CR2_SS			(0x00000002)
#define		NXC2600_AIC_CR2_SR			(0x00000004)
#define		NXC2600_AIC_CR2_SO			(0x00000008)
#define		NXC2600_AIC_CR2_IASS_MASK		(0x000000c0)
#define		NXC2600_AIC_CR2_IASS_20_BIT		(0x00000000)
#define		NXC2600_AIC_CR2_IASS_18_BIT		(0x00000040)
#define		NXC2600_AIC_CR2_IASS_16_BIT		(0x00000080)
#define		NXC2600_AIC_CR2_IASS_8_BIT		(0x000000c0)
#define		NXC2600_AIC_CR2_OASS_MASK		(0x00000300)
#define		NXC2600_AIC_CR2_OASS_20_BIT		(0x00000000)
#define		NXC2600_AIC_CR2_OASS_18_BIT		(0x00000100)
#define		NXC2600_AIC_CR2_OASS_16_BIT		(0x00000200)
#define		NXC2600_AIC_CR2_OASS_8_BIT		(0x00000300)
#define		NXC2600_AIC_CR2_ECADT			(0x00010000)
#define		NXC2600_AIC_CR2_ESADR			(0x00020000)
#define		NXC2600_AIC_CR2_ERSTO			(0x00040000)
#define	NXC2600_AIC_I2SCR			(0x10020010)
#define		NXC2600_AIC_I2SCR_AMSL			(0x00000001)
#define		NXC2600_AIC_I2SCR_WL_MASK		(0x0000000e)
#define		NXC2600_AIC_I2SCR_WL_24_BIT		(0x00000000)
#define		NXC2600_AIC_I2SCR_WL_20_BIT		(0x00000002)
#define		NXC2600_AIC_I2SCR_WL_18_BIT		(0x00000004)
#define		NXC2600_AIC_I2SCR_WL_16_BIT		(0x00000006)
#define		NXC2600_AIC_I2SCR_WL_8_BIT		(0x00000008)
#define		NXC2600_AIC_I2SCR_STPBK			(0x00001000)
#define	NXC2600_AIC_SR				(0x10020014)
#define		NXC2600_AIC_SRV1_TFL(level)		(((level)&0x00001f00)>>8)
#define		NXC2600_AIC_SRV1_RFL(level)		(((level)&0x1f000000)>>24)
#define		NXC2600_AIC_SRV2_TFS			(0x00000008)
#define		NXC2600_AIC_SRV2_RFS			(0x00000010)
#define		NXC2600_AIC_SRV2_TUR			(0x00000020)
#define		NXC2600_AIC_SRV2_ROR			(0x00000040)
#define		NXC2600_AIC_SRV2_TFL(level)		(((level)&0x00003f00)>>8)
#define		NXC2600_AIC_SRV2_RFL(level)		(((level)&0x3f000000)>>24)
#define	NXC2600_AIC_ACSR			(0x10020018)
#define		NXC2600_AIC_ACSR_CADT			(0x00010000)
#define		NXC2600_AIC_ACSR_SADR			(0x00020000)
#define		NXC2600_AIC_ACSR_RSTO			(0x00040000)
#define		NXC2600_AIC_ACSR_CLPM			(0x00080000)
#define		NXC2600_AIC_ACSR_CRDY			(0x00100000)
#define		NXC2600_AIC_ACSR_SLTERR			(0x00200000)
#define	NXC2600_AIC_I2SSR			(0x1002001c)
#define		NXC2600_AIC_I2SSR_BSY			(0x00000004)
#define	NXC2600_AIC_ACCAR			(0x10020020)
#define		NXC2600_AIC_ACCAR_CAR(car)		((car)&0x000fffff)
#define	NXC2600_AIC_ACCDR			(0x10020024)
#define		NXC2600_AIC_ACCDR_CDR(cdr)		((cdr)&0x000fffff)
#define	NXC2600_AIC_ACSAR			(0x10020028)
#define		NXC2600_AIC_ACSAR_SAR(sar)		((sar)&0x000fffff)
#define	NXC2600_AIC_ACSDR			(0x1002002c)
#define		NXC2600_AIC_ACSDR_SDR(sdr)		((sdr)&0x000fffff)
#define	NXC2600_AIC_I2SDIV			(0x10020030)
#define		NXC2600_AIC_I2SDIVV1_DV(dv)		((dv)&0x0000007f)
#define		NXC2600_AIC_I2SDIVV2_DV(dv)		((dv)&0x0000000f)
#define	NXC2600_AIC_DR				(0x10020034)
#define		NXC2600_AIC_DR_DATA(data)		((data)&0x00ffffff)
#ifndef NXC2600_AIC_EXTERN_IRQ_PIN
#define	NXC2600_AIC_EXTERN_IRQ_PIN	68
#endif
#ifndef NXC2600_AIC_EXTERN_IRQ
#define	NXC2600_AIC_EXTERN_IRQ	(NXC2600_GPIO_NIRQ(NXC2600_AIC_EXTERN_IRQ_PIN))
#endif

#ifndef __ASSEMBLY__

/* AIC operations */

static __inline__ void
nxc2600_aic_init( int ac97_mode )
{
	if(ac97_mode)
		nxc2600_gpio_vs_ac97(0);
	else
		nxc2600_gpio_vs_i2s_master(0);

	nxc2600_enable_clock(NXC2600_CDR_AIC_BITCLK);
	REG32(NXC2600_AIC_FR) &= ~NXC2600_AIC_FR_ENB;
	REG32(NXC2600_AIC_FR) |= NXC2600_AIC_FR_RST;
	//REG32(NXC2600_AIC_ACSR) = 0;
	while(REG32(NXC2600_AIC_FR)&NXC2600_AIC_FR_RST) udelay(10);
	REG32(NXC2600_AIC_CR) = 0;
	REG32(NXC2600_AIC_FR) = (REG32(NXC2600_AIC_FR)&~NXC2600_AIC_FR_AUSEL) |
				 NXC2600_AIC_FR_ENB | 
				 NXC2600_AIC_FR_BCKD | NXC2600_AIC_FR_SYNCD | 
				 ((ac97_mode)? 0:NXC2600_AIC_FR_AUSEL);
	
}

static __inline__ void
nxc2600_aic_stop( void )
{
	REG32(NXC2600_AIC_FR) &= ~NXC2600_AIC_FR_ENB;
	nxc2600_disable_clock(NXC2600_CDR_AIC_BITCLK);
}

static __inline__ void
nxc2600_aic_transmit_trigger( uint32_t value )
{
	REG32(NXC2600_AIC_FR) = (REG32(NXC2600_AIC_FR)&~NXC2600_AIC_FR_TFTH_MASK) | NXC2600_AIC_FR_TFTH(value);
}

static __inline__ void
nxc2600_aic_receive_trigger( uint32_t value )
{
	REG32(NXC2600_AIC_FR) = (REG32(NXC2600_AIC_FR)&~NXC2600_AIC_FR_RFTH_MASK) | NXC2600_AIC_FR_RFTH(value);
}

static __inline__ void
nxc2600_aic_ac97_warn_reset( void )
{
        REG32(NXC2600_AIC_CR2) |= NXC2600_AIC_CR2_SA;
        REG32(NXC2600_AIC_CR2) |= NXC2600_AIC_CR2_SS;
        udelay(10);
        REG32(NXC2600_AIC_CR2) &= ~NXC2600_AIC_CR2_SS;
        REG32(NXC2600_AIC_CR2) &= ~NXC2600_AIC_CR2_SA;
}

static __inline__ int
nxc2600_aic_wait_for_done( void )
{
	int timeout;
	for(timeout = 100000; timeout; timeout-- )
	{
		if(REG32(NXC2600_AIC_ACSR)&NXC2600_AIC_ACSR_CADT)
		{
			REG32(NXC2600_AIC_ACSR) &= ~NXC2600_AIC_ACSR_CADT;
			return 0;
		}
		udelay(10);
	}
	nxc2600_aic_ac97_warn_reset();
	return -1;
}
static __inline__ int
nxc2600_aic_wait_for_status_done( void )
{
	int timeout;
	for( timeout = 100000; timeout; timeout-- )
	{
		if(REG32(NXC2600_AIC_ACSR)&NXC2600_AIC_ACSR_SADR)
		{
			REG32(NXC2600_AIC_ACSR) &= ~NXC2600_AIC_ACSR_SADR;
			return 0;
		}
		if(REG32(NXC2600_AIC_ACSR)&NXC2600_AIC_ACSR_RSTO)
		{
			REG32(NXC2600_AIC_ACSR) &= ~NXC2600_AIC_ACSR_RSTO;
			return -2;
		}
		udelay(10);
	}
	return -1;
}

static __inline__ void
nxc2600_aic_ac97_cold_reset( void )
{
#if 0
	REG32(NXC2600_AIC_CR2) |= NXC2600_AIC_CR2_SA;
	REG32(NXC2600_AIC_CR2) &= ~NXC2600_AIC_CR2_SS;
	REG32(NXC2600_AIC_CR2) |= NXC2600_AIC_CR2_SR;
	udelay(2);
	REG32(NXC2600_AIC_CR2) &= ~NXC2600_AIC_CR2_SR;
        REG32(NXC2600_AIC_CR2) &= ~NXC2600_AIC_CR2_SA;
#else
	nxc2600_gpio_vs_ac97(1);
	nxc2600_set_gpio_output_mode(69,0);
	nxc2600_set_gpio_output_mode(70,0);
	nxc2600_set_gpio_output_mode(71,0);
	nxc2600_set_gpio_output_mode(78,0);
	nxc2600_set_gpio_output(70, 0);
	nxc2600_set_gpio_output(71, 0);
	nxc2600_set_gpio_output(78, 0);
	nxc2600_set_gpio_output(69, 0);
	udelay(20);
	nxc2600_set_gpio_output(69, 1);
	udelay(20);
	nxc2600_gpio_vs_ac97(0);
#endif
	udelay(300);
}

static __inline__ int
nxc2600_aic_ac97_write( uint32_t reg,
			uint32_t val )
{
#if 0
	udelay(2000);
	nxc2600_aic_ac97_warn_reset();
	udelay(2000);
#endif
	REG32(NXC2600_AIC_ACCAR) = reg << 12;
	REG32(NXC2600_AIC_ACCDR) = val << 4;
	if(nxc2600_aic_wait_for_done()) return -1;
	return 0;
}

static __inline__ int
nxc2600_aic_ac97_read( uint32_t reg )
{
	int data;
#if 0
	udelay(2000);
	nxc2600_aic_ac97_warn_reset();
        udelay(2000);
#endif
	REG32(NXC2600_AIC_ACCAR) = (1<<19) | (reg<<12);
	if(nxc2600_aic_wait_for_done()) return -1;
	if(nxc2600_aic_wait_for_status_done()) return -1;
	data = REG32(NXC2600_AIC_ACSDR);
	data >>= 4;
	return data;
}

static __inline__ void
nxc2600_aic_ac97_set_playback_mono(void)
{
	REG32(NXC2600_AIC_CR1) = (REG32(NXC2600_AIC_CR1)&~NXC2600_AIC_CR1_XS_MASK) | NXC2600_AIC_CR1_XS(4);
}

static __inline__ void
nxc2600_aic_ac97_set_playback_stereo(void)
{
        REG32(NXC2600_AIC_CR1) = (REG32(NXC2600_AIC_CR1)&~NXC2600_AIC_CR1_XS_MASK) | NXC2600_AIC_CR1_XS(4) | NXC2600_AIC_CR1_XS(3);
}

static __inline__ int
nxc2600_aic_ac97_set_playback_sample_bits( uint32_t bits )
{
	switch(bits)
	{
		case 8:
			bits = NXC2600_AIC_CR2_OASS_8_BIT;
			break;
		case 16:
			bits = NXC2600_AIC_CR2_OASS_16_BIT;
			break;
		case 18:
	                bits = NXC2600_AIC_CR2_OASS_18_BIT;
			break;
		case 20:
			bits = NXC2600_AIC_CR2_OASS_20_BIT;
			break;
		default:
			return -1;
	}
	REG32(NXC2600_AIC_CR)  &= ~NXC2600_AIC_CR_ERPL;
	REG32(NXC2600_AIC_CR2) = (REG32(NXC2600_AIC_CR2)&~NXC2600_AIC_CR2_OASS_MASK) | bits;
	return 0;
}

static __inline__ void
nxc2600_aic_ac97_start_playback(void)
{
        REG32(NXC2600_AIC_CR)  |= NXC2600_AIC_CR_ERPL|NXC2600_AIC_CR_TDMS;
}

static __inline__ void
nxc2600_aic_ac97_set_playback_mute(void)
{
	REG32(NXC2600_AIC_CR)  &= ~NXC2600_AIC_CR_ERPL;
}

static __inline__ void
nxc2600_aic_ac97_stop_playback(void)
{
	REG32(NXC2600_AIC_CR) &= ~(NXC2600_AIC_CR_ERPL|NXC2600_AIC_CR_TDMS);
}

static __inline__ void
nxc2600_aic_ac97_set_capture_mono(void)
{
        REG32(NXC2600_AIC_CR1) = (REG32(NXC2600_AIC_CR1)&~NXC2600_AIC_CR1_RS_MASK) | NXC2600_AIC_CR1_RS(4);
}

static __inline__ int
nxc2600_aic_ac97_set_capture_sample_bits( uint32_t bits )
{
        switch(bits)
        {
                case 8:
                        bits = NXC2600_AIC_CR2_IASS_8_BIT;
                        break;
                case 16:
                        bits = NXC2600_AIC_CR2_IASS_16_BIT;
                        break;
                case 18:
                        bits = NXC2600_AIC_CR2_IASS_18_BIT;
                        break;
                case 20:
                        bits = NXC2600_AIC_CR2_IASS_20_BIT;
                        break;
                default:
                        return -1;
        }
        REG32(NXC2600_AIC_CR)  &= ~NXC2600_AIC_CR_EREC;
        REG32(NXC2600_AIC_CR2) = (REG32(NXC2600_AIC_CR2)&~NXC2600_AIC_CR2_IASS_MASK) | bits;
        return 0;
}

static __inline__ void
nxc2600_aic_ac97_set_capture_stereo(void)
{
        REG32(NXC2600_AIC_CR1) = (REG32(NXC2600_AIC_CR1)&~NXC2600_AIC_CR1_RS_MASK) | NXC2600_AIC_CR1_RS(4) | NXC2600_AIC_CR1_RS(3);
}

static __inline__ void
nxc2600_aic_ac97_start_capture(void)
{
        REG32(NXC2600_AIC_CR)  |= NXC2600_AIC_CR_EREC|NXC2600_AIC_CR_RDMS;
}

static __inline__ void
nxc2600_aic_ac97_set_capture_mute(void)
{
	REG32(NXC2600_AIC_CR)  &= ~NXC2600_AIC_CR_EREC;
}

static __inline__ void
nxc2600_aic_ac97_stop_capture(void)
{
        REG32(NXC2600_AIC_CR) &= ~(NXC2600_AIC_CR_EREC|NXC2600_AIC_CR_RDMS);
}

static __inline__ void
nxc2600_aic_ac97_set_sample_format( uint32_t format )
{
	REG32(NXC2600_AIC_CR) = (REG32(NXC2600_AIC_CR)&NXC2600_AIC_CR_OSS_MASK)|
				(format&NXC2600_AIC_CR_OSS_MASK);			
}

static __inline__ void
nxc2600_aic_ac97_set_sample_rate( uint32_t rate )
{
}

#endif


#define NXC2600_I2C			(0x10042000)
#define	NXC2600_I2C_DR			(0x10042000)
#define	NXC2600_I2C_CR			(0x10042004)
#define		NXC2600_I2C_CR_ENABLE		(0x01)
#define		NXC2600_I2C_CR_NACK		(0x02)
#define		NXC2600_I2C_CR_STOP		(0x04)
#define		NXC2600_I2C_CR_START		(0x08)
#define		NXC2600_I2C_CR_IEN		(0x10)
#define	NXC2600_I2C_SR			(0x10042008)
#define		NXC2600_I2C_SR_ACKF		(0x01)
#define		NXC2600_I2C_SR_DRF		(0x02)
#define		NXC2600_I2C_SR_TEND		(0x04)
#define		NXC2600_I2C_SR_BUSY		(0x08)
#define		NXC2600_I2C_SR_STX		(0x10)
#define	NXC2600_I2C_GR			(0x1004200c)

#ifndef __ASSEMBLY__

/* I2C operations */

static __inline__ void
nxc2600_i2c_init( uint32_t clock )
{
	if( clock == 0 ) clock = 100000;
        if( clock > 400000 ) clock = 400000;
        nxc2600_enable_clock(NXC2600_CDR_I2C);
	REG8(NXC2600_I2C_CR) = NXC2600_I2C_CR_ENABLE;
        REG16(NXC2600_I2C_GR) = CONFIG_NXC2600_EXTAL_CLOCK / 16 / clock ;

	
}

static __inline__ int
nxc2600_i2c_wait_for_ack( void )
{
        int timeout = 1000;
        while((REG8(NXC2600_I2C_SR)&(NXC2600_I2C_SR_ACKF|NXC2600_I2C_SR_DRF)))
        {
                if(!timeout--)
                        return -1;
                udelay(1);
        }

        return 0;
}
static __inline__ int
nxc2600_i2c_read_byte_data( int last_one )
{
        int timeout = 1000;
        int data;

	if(last_one)
		REG8(NXC2600_I2C_CR) |= NXC2600_I2C_CR_NACK;
	else
        	REG8(NXC2600_I2C_CR) &= ~NXC2600_I2C_CR_NACK;
        while(!(REG8(NXC2600_I2C_SR)&NXC2600_I2C_SR_DRF))
        {
                if(!timeout--) return -1;
                udelay(1);
        }
	if(last_one)
	{
		REG8(NXC2600_I2C_CR) |= NXC2600_I2C_CR_STOP||NXC2600_I2C_CR_ENABLE;
	}
        data = REG8(NXC2600_I2C_DR);
       	REG8(NXC2600_I2C_SR) &= ~NXC2600_I2C_SR_DRF;
        return data;
}

static __inline__ int
nxc2600_i2c_write_byte_data( uint8_t data )
{
	int timeout = 1000;

	REG8(NXC2600_I2C_CR) |= NXC2600_I2C_CR_NACK;
        REG8(NXC2600_I2C_DR) = data;
	REG8(NXC2600_I2C_SR) |= NXC2600_I2C_SR_DRF;

	while(!(REG8(NXC2600_I2C_SR)&NXC2600_I2C_SR_TEND))
        {
                if(!timeout--)
                        return -1;
                udelay(1);
        }

        return nxc2600_i2c_wait_for_ack();
}

static __inline__ int
nxc2600_i2c_send_start( uint8_t addr,
                        int read )
{
	REG8(NXC2600_I2C_CR) &= ~NXC2600_I2C_CR_NACK;
	REG8(NXC2600_I2C_SR) &= ~NXC2600_I2C_SR_DRF;
	REG8(NXC2600_I2C_CR) |= NXC2600_I2C_CR_START|NXC2600_I2C_CR_ENABLE; // | NXC2600_I2C_CR_IEN;
        while(REG8(NXC2600_I2C_CR)&NXC2600_I2C_CR_START) udelay(1);
        return nxc2600_i2c_write_byte_data( (addr<<1)|((read)? 1:0));
}

static __inline__ void
nxc2600_i2c_send_stop( void )
{
        REG8(NXC2600_I2C_CR) |= NXC2600_I2C_CR_STOP|NXC2600_I2C_CR_ENABLE;
        while( REG8(NXC2600_I2C_CR)&NXC2600_I2C_CR_STOP ) udelay(1);
	while( REG8(NXC2600_I2C_SR)&NXC2600_I2C_SR_BUSY ) udelay(1);
	udelay(10);
}


static __inline__ int
nxc2600_i2c_write_data( uint8_t addr,
                        void *data,
                        int len )
{
        int i = -1;
        char *buf = data;
        for(i=0; i<len; i++)
        {
                if(nxc2600_i2c_write_byte_data( buf[i] ))
                        break;
        }
        return i;
}
static __inline__ int
nxc2600_i2c_read_data( uint8_t addr,
                        void *data,
                        int len )
{
        int i = -1;
        int tmp;
        char *buf = data;

        for(i=0; i<len; i++)
        {
                tmp = nxc2600_i2c_read_byte_data((i+1)==len);
                if( tmp <0 )
                        break;
                buf[i] = (uint8_t)tmp;
        }
        return i;
}


#endif


#define NXC2600BL_MAX_INTENSITY		(CONFIG_NXC2600_EXTAL_CLOCK/28800/2)

struct nxc2600_backlight 
{
	int powermode;
	int current_intensity;

	struct device *dev;
};


#define	NXC2600_PWM(n)				(0x10050000|(0x00001000*(n)))
#define NXC2600_PWM_CTR(n)			(NXC2600_PWM(n))
#define		NXC2600_PWM_CTR_PRESCALE_MASK		(0x3f)
#define		NXC2600_PWM_CTR_PRESCALE(val)		(((val)-1)&0x3f)
#define		NXC2600_PWM_CTR_SHUTDOWN_MASK		(0x40)
#define		NXC2600_PWM_CTR_GRACEFUL_SHUTDOWN	(0x00)
#define		NXC2600_PWM_CTR_ABRUPT_SHUTDOWN		(0x40)
#define		NXC2600_PWM_CTR_ENABLE			(0x80)
#define	NXC2600_PWM_PER(n)			(NXC2600_PWM(n)|0x00000004)
#define		NXC2600_PWM_PER_PERIOD_MASK		(0x03ff)
#define		NXC2600_PWM_PER_PERIOD(val)		(((val)-1)&0x03ff)
#define	NXC2600_PWM_DUT(n)			(NXC2600_PWM(n)|0x00000008)
#define		NXC2600_PWM_DUT_DUTY_MASK		(0x03ff)
#define		NXC2600_PWM_DUT_DUTY(duty)		((duty)&0x03ff)
#define		NXC2600_PWM_DUT_FDUTY			(0x0400)


#ifndef __ASSEMBLY__

/* PWM operations */

static __inline__ void
nxc2600_pwm_init( uint32_t n,
		  uint32_t clock )
{
	uint32_t duty = 0;
	if( clock < 28800 ) /* minimal 28800 Hz */
		return;
	if( clock )
	{
		duty = CONFIG_NXC2600_EXTAL_CLOCK/clock;
	}
	REG8(NXC2600_PWM_CTR(n)) = 0;
	REG16(NXC2600_PWM_DUT(n)) = NXC2600_PWM_DUT_DUTY(duty);
	REG16(NXC2600_PWM_PER(n)) = NXC2600_PWM_PER_PERIOD(duty*2);
}

static __inline__ void
nxc2600_pwm_enable( uint32_t n )
{
	REG8(NXC2600_PWM_CTR(n)) |= NXC2600_PWM_CTR_ENABLE;
}

static __inline__ void
nxc2600_pwm_disable( uint32_t n )
{
	REG8(NXC2600_PWM_CTR(n)) &= ~NXC2600_PWM_CTR_ENABLE;
}

#endif




#define NXC2600_LCD				(0x13050000)
#define	NXC2600_LCD_CFG				(0x13050000)
#define		NXC2600_LCD_CFG_MODE_MASK		(0x0000000f)
#define		NXC2600_LCD_CFG_MODE_TFT		(0x00000000)
#define		NXC2600_LCD_CFG_MODE_SHARP_HR_TFT	(0x00000001)
#define		NXC2600_LCD_CFG_MODE_CASIO_TFT		(0x00000002)
#define		NXC2600_LCD_CFG_MODE_SAMSUNG_A_TFT	(0x00000003)
#define		NXC2600_LCD_CFG_MODE_NON_INTER_CCIR656	(0x00000004)
#define		NXC2600_LCD_CFG_MODE_INTER_CCIR656	(0x00000006)
#define		NXC2600_LCD_CFG_MODE_SINGLE_COLOR_STN	(0x00000008)
#define		NXC2600_LCD_CFG_MODE_SINGLE_MONO_STN	(0x00000009)
#define		NXC2600_LCD_CFG_MODE_DUAL_COLOR_STN	(0x0000000a)
#define		NXC2600_LCD_CFG_MODE_DUAL_MONO_STN	(0x0000000b)
#define		NXC2600_LCD_CFG_PDW_MASK		(0x00000030)
#define		NXC2600_LCD_CFG_PDW_SIGNAL_0		(0x00000000)
#define		NXC2600_LCD_CFG_PDW_SIGNAL_1		(0x00000010)
#define		NXC2600_LCD_CFG_PDW_SIGNAL_3		(0x00000020)
#define		NXC2600_LCD_CFG_PDW_SIGNAL_7		(0x00000030)
#define		NXC2600_LCD_CFG_PDW_DUAL_MONO_2		(0x00000020)
#define		NXC2600_LCD_CFG_PDW_DUAL_MONO_3		(0x00000030)
#define		NXC2600_LCD_CFG_VSP			(0x00000100)
#define		NXC2600_LCD_CFG_DEP			(0x00000200)
#define		NXC2600_LCD_CFG_PCP			(0x00000400)
#define		NXC2600_LCD_CFG_HSP			(0x00000800)
#define		NXC2600_LCD_CFG_REVP			(0x00001000)
#define		NXC2600_LCD_CFG_SPLP			(0x00002000)
#define		NXC2600_LCD_CFG_CLSP			(0x00004000)
#define		NXC2600_LCD_CFG_PSP			(0x00008000)
#define		NXC2600_LCD_CFG_SYNDIR			(0x00010000)
#define		NXC2600_LCD_CFG_INVDAT			(0x00020000)
#define		NXC2600_LCD_CFG_PCLKM			(0x00040000)
#define		NXC2600_LCD_CFG_HSYNM			(0x00080000)
#define		NXC2600_LCD_CFG_REVM			(0x00100000)
#define		NXC2600_LCD_CFG_SPLM			(0x00200000)
#define		NXC2600_LCD_CFG_CLSM			(0x00400000)
#define		NXC2600_LCD_CFG_PSM			(0x00800000)
#define	NXC2600_LCD_VSYNC			(0x13050004)
#define		NXC2600_LCD_VSYNC_VPE_MASK		(0x000007ff)
#define		NXC2600_LCD_VSYNC_VPE(val)		((val)&0x000007ff)
#define		NXC2600_LCD_VSYNC_VPS_MASK		(0x07ff0000)
#define		NXC2600_LCD_VSYNC_VPS(val)		(((val)&0x000007ff)<<16)
#define	NXC2600_LCD_HSYNC			(0x13050008)
#define		NXC2600_LCD_HSYNC_HPE_MASK		(0x000007ff)
#define		NXC2600_LCD_HSYNC_HPE(val)		((val)&0x000007ff)
#define		NXC2600_LCD_HSYNC_HPS_MASK		(0x07ff0000)
#define		NXC2600_LCD_HSYNC_HPS(val)		(((val)&0x000007ff)<<16)
#define	NXC2600_LCD_VAT				(0x1305000c)
#define		NXC2600_LCD_VAT_VT_MASK			(0x000007ff)
#define		NXC2600_LCD_VAT_VT(val)			((val)&0x000007ff)
#define		NXC2600_LCD_VAT_HT_MASK			(0x07ff0000)
#define		NXC2600_LCD_VAT_HT(val)			(((val)&0x000007ff)<<16)
#define	NXC2600_LCD_DAH				(0x13050010)
#define		NXC2600_LCD_DAH_HDE_MASK		(0x000007ff)
#define		NXC2600_LCD_DAH_HDE(val)		((val)&0x000007ff)
#define		NXC2600_LCD_DAH_HDS_MASK		(0x07ff0000)
#define		NXC2600_LCD_DAH_HDS(val)		(((val)&0x000007ff)<<16)
#define	NXC2600_LCD_DAV				(0x13050014)
#define		NXC2600_LCD_DAV_VDE_MASK		(0x000007ff)
#define		NXC2600_LCD_DAV_VDE(val)		((val)&0x000007ff)
#define		NXC2600_LCD_DAV_VDS_MASK		(0x07ff0000)
#define		NXC2600_LCD_DAV_VDS(val)		(((val)&0x000007ff)<<16)
#define	NXC2600_LCD_PS				(0x13050018)
#define		NXC2600_LCD_PS_PSE_MASK			(0x000007ff)
#define		NXC2600_LCD_PS_PSE(val)			((val)&0x000007ff)
#define		NXC2600_LCD_PS_PSS_MASK			(0x07ff0000)
#define		NXC2600_LCD_PS_PSS(val)			(((val)&0x000007ff)<<16)
#define	NXC2600_LCD_CLS				(0x1305001c)
#define		NXC2600_LCD_CLS_CLSE_MASK		(0x000007ff)
#define		NXC2600_LCD_CLS_CLSE(val)		((val)&0x000007ff)
#define		NXC2600_LCD_CLS_CLSS_MASK		(0x07ff0000)
#define		NXC2600_LCD_CLS_CLSS(val)		(((val)&0x000007ff)<<16)
#define	NXC2600_LCD_SPL				(0x13050020)
#define		NXC2600_LCD_SPL_SPLE_MASK		(0x000007ff)
#define		NXC2600_LCD_SPL_SPLE(val)		((val)&0x000007ff)
#define		NXC2600_LCD_SPL_SPLS_MASK		(0x07ff0000)
#define		NXC2600_LCD_SPL_SPLS(val)		(((val)&0x000007ff)<<16)
#define	NXC2600_LCD_REV				(0x13050024)
#define		NXC2600_LCD_REV_REVE_MASK		(0x000007ff)
#define		NXC2600_LCD_REV_REVE(val)		((val)&0x000007ff)
#define		NXC2600_LCD_REV_REVS_MASK		(0x07ff0000)
#define		NXC2600_LCD_REV_REVS(val)		(((val)&0x000007ff)<<16)
#define	NXC2600_LCD_CTRL			(0x13050030)
#define		NXC2600_LCD_CTRL_BPP_MASK		(0x00000007)
#define		NXC2600_LCD_CTRL_BPP_1			(0x00000000)
#define		NXC2600_LCD_CTRL_BPP_2			(0x00000001)
#define		NXC2600_LCD_CTRL_BPP_4			(0x00000002)
#define		NXC2600_LCD_CTRL_BPP_8			(0x00000003)
#define		NXC2600_LCD_CTRL_BPP_16			(0x00000004)
#define		NXC2600_LCD_CTRL_ENABLE			(0x00000008)
#define		NXC2600_LCD_CTRL_DIS			(0x00000010)
#define		NXC2600_LCD_CTRL_PEDN			(0x00000020)
#define		NXC2600_LCD_CTRL_BEDN			(0x00000040)
#define		NXC2600_LCD_CTRL_QDM			(0x00000080)
#define		NXC2600_LCD_CTRL_LDDM			(0x00000100)
#define		NXC2600_LCD_CTRL_IFUM1			(0x00000200)
#define		NXC2600_LCD_CTRL_IFUM0			(0x00000400)
#define		NXC2600_LCD_CTRL_OFUM			(0x00000800)
#define		NXC2600_LCD_CTRL_SOFM			(0x00001000)
#define		NXC2600_LCD_CTRL_EOFM			(0x00002000)
#define		NXC2600_LCD_CTRL_PDD_MASK		(0x00ff0000)
#define		NXC2600_LCD_CTRL_PDD(count)		(((count)&0x000000ff)<<16)
#define		NXC2600_LCD_CTRL_FRC_MASK		(0x03000000)
#define		NXC2600_LCD_CTRL_FRC_16			(0x00000000)
#define		NXC2600_LCD_CTRL_FRC_4			(0x01000000)
#define		NXC2600_LCD_CTRL_FRC_2			(0x02000000)
#define		NXC2600_LCD_CTRL_OFUP			(0x04000000)
#define		NXC2600_LCD_CTRL_RGB_565		(0x00000000)
#define		NXC2600_LCD_CTRL_RGB_555		(0x08000000)
#define		NXC2600_LCD_CTRL_BST_MASK		(0x30000000)
#define		NXC2600_LCD_CTRL_BST_4			(0x00000000)
#define		NXC2600_LCD_CTRL_BST_8			(0x10000000)
#define		NXC2600_LCD_CTRL_BST_16			(0x20000000)
#define NXC2600_LCD_STATE			(0x13050034)
#define		NXC2600_LCD_STATE_LDD			(0x00000001)
#define		NXC2600_LCD_STATE_IFU1			(0x00000002)
#define		NXC2600_LCD_STATE_IFU0			(0x00000004)
#define		NXC2600_LCD_STATE_OUF			(0x00000008)
#define		NXC2600_LCD_STATE_SOF			(0x00000010)
#define		NXC2600_LCD_STATE_EOF			(0x00000020)
#define		NXC2600_LCD_STATE_QD			(0x00000080)
#define NXC2600_LCD_IID				(0x13050038)
#define	NXC2600_LCD_DA(n)			(0x13050040+0x10*(n))
#define	NXC2600_LCD_SA(n)			(0x13050044+0x10*(n))
#define	NXC2600_LCD_FID(n)			(0x13050048+0x10*(n))
#define	NXC2600_LCD_CMD(n)			(0x1305004c+0x10*(n))
#define		NXC2600_LCD_CMD_LEN(len)		((len)&0x00ffffff)
#define		NXC2600_LCD_CMD_PAL			(0x10000000)
#define		NXC2600_LCD_CMD_EOFINT			(0x40000000)
#define		NXC2600_LCD_CMD_SOFINT			(0x80000000)
#define	NXC2600_LCD_PALETTE_ENTRIES	(256)
#define	NXC2600_LCD_MAX_XRES		(1024)
#define	NXC2600_LCD_MAX_YRES		(1024)
#define	NXC2600_LCD_MAX_BPP		(16)

#ifndef __ASSEMBLY__

/* LCD operations */

struct nxc2600_panel
{
        const char name[25];

        /* panel timings */
        uint32_t ctrl;
	uint32_t cfg;
        uint32_t fclk;      /* frame clk */
        uint32_t hsw;       /* hsync width, in pclk */
        uint32_t vsw;       /* vsync width, in line count */
        uint32_t elw;       /* end of line, in pclk */
        uint32_t blw;       /* begin of line, in pclk */
        uint32_t efw;       /* end of frame, in line count */
        uint32_t bfw;       /* begin of frame, in line count */

        uint32_t        min_xres;
        uint32_t        max_xres;
        uint32_t        min_yres;
        uint32_t        max_yres;

        uint32_t        xres;
        uint32_t        yres;
        uint32_t        xpos;
        uint32_t        ypos;
        uint32_t        bpp;

};

#ifndef CONFIG_NXC2600_LCD_ON_GPIO_PIN
#define CONFIG_NXC2600_LCD_ON_GPIO_PIN	93
#endif

static __inline__ void
nxc2600_lcd_on(void)
{
        nxc2600_enable_clock(NXC2600_CDR_LCD);
        nxc2600_set_gpio_output(CONFIG_NXC2600_LCD_ON_GPIO_PIN,1);
        REG32(NXC2600_LCD_CTRL) |= NXC2600_LCD_CTRL_ENABLE;
	udelay(1000);

}

static __inline__ void
nxc2600_lcd_off(void)
{
	REG32(NXC2600_LCD_CTRL) &= ~NXC2600_LCD_CTRL_ENABLE;
        nxc2600_set_gpio_output(CONFIG_NXC2600_LCD_ON_GPIO_PIN,0);
        nxc2600_disable_clock(NXC2600_CDR_LCD);
}

static __inline__ void
nxc2600_lcd_init(struct nxc2600_panel *panel)
{
	uint32_t stn;
	uint32_t mode;
	uint32_t val,pclk;

	nxc2600_set_gpio_output_mode(CONFIG_NXC2600_LCD_ON_GPIO_PIN,0);

	REG32(NXC2600_LCD_CTRL) = panel->ctrl | NXC2600_LCD_CTRL_BST_16 | NXC2600_LCD_CTRL_OFUP;

	mode = panel->cfg & NXC2600_LCD_CFG_MODE_MASK;
	switch( mode )
	{
        	case NXC2600_LCD_CFG_MODE_SINGLE_COLOR_STN:
		    case NXC2600_LCD_CFG_MODE_SINGLE_MONO_STN:
	        case NXC2600_LCD_CFG_MODE_DUAL_COLOR_STN:
        	case NXC2600_LCD_CFG_MODE_DUAL_MONO_STN:
                	if ((mode == NXC2600_LCD_CFG_MODE_DUAL_MONO_STN) ||
	                    (mode == NXC2600_LCD_CFG_MODE_DUAL_COLOR_STN))
        	                stn = panel->max_yres >> 1;
 	                else
	                        stn = panel->max_yres;

	                REG32(NXC2600_LCD_VSYNC) = (0 << 16) | panel->vsw;
        	        REG32(NXC2600_LCD_HSYNC) = ((panel->blw+panel->max_xres) << 16) | (panel->blw+panel->max_xres+panel->hsw);

                	/* Screen setting */
	                REG32(NXC2600_LCD_VAT) = ((panel->blw + panel->max_xres + panel->hsw + panel->elw) << 16) | (stn + panel->vsw + panel->bfw + panel->efw);
	                REG32(NXC2600_LCD_DAH) = (panel->blw << 16) | (panel->blw + panel->max_xres);
                	REG32(NXC2600_LCD_DAV) = (0 << 16) | (stn);

	                /* AC BIAs signal */
        	        REG32(NXC2600_LCD_PS) = (0 << 16) | (stn+panel->vsw+panel->efw+panel->bfw);

                	break;

	        case NXC2600_LCD_CFG_MODE_TFT:
        	case NXC2600_LCD_CFG_MODE_SHARP_HR_TFT:
	        case NXC2600_LCD_CFG_MODE_CASIO_TFT:
        	case NXC2600_LCD_CFG_MODE_SAMSUNG_A_TFT:
	                REG32(NXC2600_LCD_VSYNC) = (0 << 16) | panel->vsw;
        	        REG32(NXC2600_LCD_DAV) = ((panel->vsw + panel->bfw) << 16) | (panel->vsw + panel->bfw + panel->max_yres);
                	REG32(NXC2600_LCD_VAT) = (((panel->blw + panel->max_xres + panel->elw + panel->hsw)) << 16) | (panel->vsw + panel->bfw + panel->max_yres + panel->efw);
	                REG32(NXC2600_LCD_HSYNC) = (0 << 16) | panel->hsw;
	                REG32(NXC2600_LCD_DAH) = ((panel->hsw + panel->blw) << 16) | (panel->hsw + panel->blw + panel->max_xres);
        	        break;
        }
        switch( mode )
	{
	        case NXC2600_LCD_CFG_MODE_SAMSUNG_A_TFT:
        	        panel->cfg |= NXC2600_LCD_CFG_REVP | NXC2600_LCD_CFG_SPLP;
                	break;
	        case NXC2600_LCD_CFG_MODE_SHARP_HR_TFT:
	                panel->cfg |= NXC2600_LCD_CFG_PSP;
        	        break;
	        case NXC2600_LCD_CFG_MODE_CASIO_TFT:
        	        break;
        }

        /* Configure the LCD panel */
        REG32(NXC2600_LCD_CFG) = panel->cfg;


        val = panel->fclk; /* frame clk */
        pclk = val * (panel->max_xres + panel->hsw + panel->elw + panel->blw) *
               (panel->max_yres + panel->vsw + panel->efw + panel->bfw); /* Pixclk */

        if ((mode == NXC2600_LCD_CFG_MODE_SINGLE_COLOR_STN) ||
            (mode == NXC2600_LCD_CFG_MODE_DUAL_COLOR_STN))
                pclk = (pclk * 3);

        if ((mode == NXC2600_LCD_CFG_MODE_SINGLE_COLOR_STN) ||
            (mode == NXC2600_LCD_CFG_MODE_DUAL_COLOR_STN) ||
            (mode == NXC2600_LCD_CFG_MODE_SINGLE_MONO_STN) ||
            (mode == NXC2600_LCD_CFG_MODE_DUAL_MONO_STN))                
		pclk = pclk >> ((panel->cfg & NXC2600_LCD_CFG_PDW_MASK) >> 4);

        if ((mode == NXC2600_LCD_CFG_MODE_DUAL_COLOR_STN) ||
            (mode == NXC2600_LCD_CFG_MODE_DUAL_MONO_STN))                
		pclk >>= 1;

	nxc2600_set_pix_clock(pclk);
	nxc2600_set_lcd_clock(pclk*4);

	switch( mode )
        {
                case NXC2600_LCD_CFG_MODE_SAMSUNG_A_TFT:
                {
                        unsigned int total, tp_s, tp_e, ckv_s, ckv_e;
                        unsigned int rev_s, rev_e, inv_s, inv_e;
                        total = panel->blw + panel->max_xres + panel->elw + panel->hsw;
                        tp_s = panel->blw + panel->max_xres + 1;
                        tp_e = tp_s + 1;
                        ckv_s = tp_s - pclk/(1000000000/4100);
                        ckv_e = tp_s + total;
                        rev_s = tp_s - 11;      /* -11.5 clk */
                        rev_e = rev_s + total;
                        inv_s = tp_s;
                        inv_e = inv_s + total;
                        REG32(NXC2600_LCD_CLS) = (tp_s << 16) | tp_e;
                        REG32(NXC2600_LCD_PS) = (ckv_s << 16) | ckv_e;
                        REG32(NXC2600_LCD_SPL) = (rev_s << 16) | rev_e;
                        REG32(NXC2600_LCD_REV) = (inv_s << 16) | inv_e;
                        break;
                }
                case NXC2600_LCD_CFG_MODE_SHARP_HR_TFT:
                {
                        unsigned int total, cls_s, cls_e, ps_s, ps_e;
                        unsigned int spl_s, spl_e, rev_s, rev_e;
                        total = panel->blw + panel->max_xres + panel->elw + panel->hsw;
                        spl_s = 1;
                        spl_e = spl_s + 1;
                        cls_s = 0;
                        cls_e = total - 60;     /* > 4us (pclk = 80ns) */
                        ps_s = cls_s;
                        ps_e = cls_e;
                        rev_s = total - 40;     /* > 3us (pclk = 80ns) */
                        rev_e = rev_s + total;
                        REG32(NXC2600_LCD_SPL) = (spl_s << 16) | spl_e;
                        REG32(NXC2600_LCD_CLS) = (cls_s << 16) | cls_e;
                        REG32(NXC2600_LCD_PS) = (ps_s << 16) | ps_e;
                        REG32(NXC2600_LCD_REV) = (rev_s << 16) | rev_e;
                        break;
                }
                case NXC2600_LCD_CFG_MODE_CASIO_TFT:
                        break;
	}

}

static __inline__ int
nxc2600_lcd_get_bpp( uint32_t bpp )
{
	uint32_t ctrl = ((bpp == -1)? REG32(NXC2600_LCD_CTRL):bpp);

	switch(ctrl&NXC2600_LCD_CTRL_BPP_MASK)
	{
		case NXC2600_LCD_CTRL_BPP_1:
			return 1;
		case NXC2600_LCD_CTRL_BPP_2:
			return 2;
		case NXC2600_LCD_CTRL_BPP_4:
			return 4;
		case NXC2600_LCD_CTRL_BPP_8:
			return 8;
		case NXC2600_LCD_CTRL_BPP_16:
			return 16;
	}
	return -1;
}
#endif









#define NXC2600_NAND_DATA_PORT		(IO_BASE|0x14000000)
#define NXC2600_NAND_CSR		(0x13010050)
#define		NXC2600_NAND_CSR_NFE		(0x00000001)
#define		NXC2600_NAND_CSR_FCE		(0x00000002)
#define		NXC2600_NAND_CSR_ECCE		(0x00000004)
#define		NXC2600_NAND_CSR_ERST		(0x00000008)
#define		NXC2600_NAND_CSR_BOOT_SEL_MASK	(0x00000070)
#define		NXC2600_NAND_CSR_BUS_WIDTH_16	(0x00000010)
#define		NXC2600_NAND_CSR_PAGE_SIZE_2048	(0x00000020)
#define		NXC2600_NAND_CSR_ROWADDRCYCLE_3	(0x00000040)
#define		NXC2600_NAND_CSR_RB		(0x00000080)
#define	NXC2600_NAND_ECC		(0x13010054)
#define		NXC2600_NAND_ECC_0_MASK		(0x000000ff)
#define		NXC2600_NAND_ECC_0(ecc)		((ecc)&0xff)
#define		NXC2600_NAND_ECC_1_MASK		(0x0000ff00)
#define		NXC2600_NAND_ECC_1(ecc)		((ecc)&0xff)<<8)
#define		NXC2600_NAND_ECC_2_MASK		(0x00ff0000)
#define		NXC2600_NAND_ECC_2(ecc)		((ecc)&0xff)<<16)

#ifndef NXC2600_NAND_DEFAULT_BUS_WIDTH
#define	NXC2600_NAND_DEFAULT_BUS_WIDTH	8
#endif
#ifndef	NXC2600_NAND_DEFAULT_PAGE_SIZE
#define	NXC2600_NAND_DEFAULT_PAGE_SIZE	2048
#endif
#ifndef	NXC2600_NAND_DEFAULT_ROWADDR_CYCLE
#define	NXC2600_NAND_DEFAULT_ROWADDR_CYCLE	2
#endif
#ifndef NXC2600_NAND_ECC_BLOCK
#define	NXC2600_NAND_ECC_BLOCK	256
#endif
#ifndef NXC2600_NAND_ECC_POS
#define	NXC2600_NAND_ECC_POS	4
#endif

#ifndef __ASSEMBLY__

/* NAND operations */

static __inline__ int
nxc2600_get_boot_bank( void )
{
	static int bank = -1;
	if(bank==-1)
		bank = ((REG32(NXC2600_NAND_CSR)&NXC2600_NAND_CSR_NFE)? 3:0);

	return bank;
}

static __inline__ void
nxc2600_nand_set_nce( int hilow )
{
	if(!hilow)
		REG32(NXC2600_NAND_CSR) |= NXC2600_NAND_CSR_FCE;
	else
		REG32(NXC2600_NAND_CSR) &= ~NXC2600_NAND_CSR_FCE;
}

static __inline__ void*
nxc2600_nand_set_cle( int hilow )
{
	if(hilow)
		return (void*)(NXC2600_NAND_DATA_PORT | 0x00040000);

	
	return (void*)(NXC2600_NAND_DATA_PORT & ~0x00040000);
}

static __inline__ void*
nxc2600_nand_set_ale( int hilow )
{
	if(hilow)
		return (void*)(NXC2600_NAND_DATA_PORT |  0x00080000);

	
	return (void*)(NXC2600_NAND_DATA_PORT & ~0x00080000);

}

static __inline__ void
nxc2600_nand_cmd( int cmd )
{
	REG8(nxc2600_nand_set_cle(1)) = cmd;
}

static __inline__ void
nxc2600_nand_addr( int addr )
{
	REG8(nxc2600_nand_set_ale(1)) = addr;
}

static __inline__ void
nxc2600_nand_ecc_enable(void)
{
	REG32(NXC2600_NAND_CSR) |= NXC2600_NAND_CSR_ECCE|NXC2600_NAND_CSR_ERST;
}

static __inline__ void
nxc2600_nand_ecc_disable(void)
{
	REG32(NXC2600_NAND_CSR) &= ~NXC2600_NAND_CSR_ECCE;
}

static __inline__ uint32_t
nxc2600_nand_get_ecc(void)
{
	return (REG32(NXC2600_NAND_ECC)&0x00ffffff);
}

static __inline__ int
nxc2600_nand_is_ready( void )
{
	return ((REG32(NXC2600_NAND_CSR)&NXC2600_NAND_CSR_RB)? 1:0);
}

static __inline__ int
nxc2600_nand_get_bus_width( void )
{
	if(nxc2600_get_boot_bank()==3)
		return ((REG32(NXC2600_NAND_CSR)&NXC2600_NAND_CSR_BUS_WIDTH_16)? 16:8);
	switch(NXC2600_NAND_DEFAULT_BUS_WIDTH)
	{
		case 8:
			REG32(NXC2600_NAND_CSR) &= ~NXC2600_NAND_CSR_BUS_WIDTH_16;
			break;
		case 16:
			REG32(NXC2600_NAND_CSR) |= NXC2600_NAND_CSR_BUS_WIDTH_16;
			break;
	}
	return NXC2600_NAND_DEFAULT_BUS_WIDTH;
}

static __inline__ int
nxc2600_nand_get_page_size( void )
{
	if(nxc2600_get_boot_bank()==3)
		return ((REG32(NXC2600_NAND_CSR)&NXC2600_NAND_CSR_PAGE_SIZE_2048)? 2048:512);
	switch(NXC2600_NAND_DEFAULT_PAGE_SIZE)
	{
		case 2048:
			REG32(NXC2600_NAND_CSR) |= NXC2600_NAND_CSR_PAGE_SIZE_2048;
			break;
		case 512:
			REG32(NXC2600_NAND_CSR) &= ~NXC2600_NAND_CSR_PAGE_SIZE_2048;
			break;
	}
	return NXC2600_NAND_DEFAULT_PAGE_SIZE;
}

static __inline__ int
nxc2600_nand_get_rowaddr_cycle( void )
{
	if(nxc2600_get_boot_bank()==3)
		return ((REG32(NXC2600_NAND_CSR)&NXC2600_NAND_CSR_ROWADDRCYCLE_3)? 3:2);
	switch(NXC2600_NAND_DEFAULT_ROWADDR_CYCLE)
	{
		case 2:
			REG32(NXC2600_NAND_CSR) &= ~NXC2600_NAND_CSR_ROWADDRCYCLE_3;
			break;
		case 3:
			REG32(NXC2600_NAND_CSR) |= NXC2600_NAND_CSR_ROWADDRCYCLE_3;
			break;
	}
	return NXC2600_NAND_DEFAULT_ROWADDR_CYCLE;
}

static __inline__ void
nxc2600_nand_init( void )
{
        nxc2600_get_boot_bank();
        nxc2600_gpio_vs_nand(0);
	nxc2600_nand_get_bus_width();
	nxc2600_nand_get_page_size();
	nxc2600_nand_get_rowaddr_cycle();
        REG32(NXC2600_NAND_CSR) |= NXC2600_NAND_CSR_NFE;
}

static __inline__ void
nxc2600_nand_disable(void)
{
	REG32(NXC2600_NAND_CSR) &= ~NXC2600_NAND_CSR_NFE;
}

#endif


#define	NXC2600_MMC				(0x10021000)
#define NXC2600_MMC_STRPCL			(0x10021000)
#define		NXC2600_MMC_STRPCL_CLOCK_MASK		(0x0003)
#define		NXC2600_MMC_STRPCL_CLOCK_START		(0x0002)
#define		NXC2600_MMC_STRPCL_CLOCK_STOP		(0x0001)
#define		NXC2600_MMC_STRPCL_START_OP		(0x0004)
#define		NXC2600_MMC_STRPCL_RESET		(0x0008)
#define		NXC2600_MMC_STRPCL_STOP_READWAIT	(0x0010)
#define		NXC2600_MMC_STRPCL_START_READWAIT	(0x0020)
#define		NXC2600_MMC_STRPCL_EXIT_TRANSFER	(0x0040)
#define		NXC2600_MMC_STRPCL_EXIT_MULTIPLE	(0x0080)
#define	NXC2600_MMC_STAT			(0x10021004)
#define		NXC2600_MMC_STAT_TIMEOUT_READ		(0x00000001)
#define		NXC2600_MMC_STAT_TIMEOUT_RES		(0x00000002)
#define		NXC2600_MMC_STAT_CRC_WRITE_ERR		(0x00000004)
#define		NXC2600_MMC_STAT_CRC_WRITE_NO_CRC	(0x00000008)
#define		NXC2600_MMC_STAT_CRC_READ_ERR		(0x00000010)
#define		NXC2600_MMC_STAT_CRC_RES_ERR		(0x00000020)
#define		NXC2600_MMC_STAT_DATA_FIFO_EMPTY	(0x00000040)
#define		NXC2600_MMC_STAT_DATA_FIFO_FULL		(0x00000080)
#define		NXC2600_MMC_STAT_CLK_EN			(0x00000100)
#define		NXC2600_MMC_STAT_IS_READWAIT		(0x00000200)
#define		NXC2600_MMC_STAT_DATA_FIFO_AFULL	(0x00000400)
#define		NXC2600_MMC_STAT_END_CMD_RES		(0x00000800)
#define		NXC2600_MMC_STAT_DATA_TRAN_DONE		(0x00001000)
#define		NXC2600_MMC_STAT_PRG_DONE		(0x00002000)
#define		NXC2600_MMC_STAT_SDIO_INT_ACTIVE	(0x00004000)
#define		NXC2600_MMC_STAT_IS_RESETTING		(0x00008000)
#define	NXC2600_MMC_CLKRT			(0x10021008)
#define		NXC2600_MMC_CLKRT_CLK_RATE_CLK_SRC	(0x0000)
#define		NXC2600_MMC_CLKRT_CLK_RATE_1_2		(0x0001)
#define		NXC2600_MMC_CLKRT_CLK_RATE_1_4		(0x0002)
#define		NXC2600_MMC_CLKRT_CLK_RATE_1_8		(0x0003)
#define		NXC2600_MMC_CLKRT_CLK_RATE_1_16		(0x0004)
#define		NXC2600_MMC_CLKRT_CLK_RATE_1_32		(0x0005)
#define		NXC2600_MMC_CLKRT_CLK_RATE_1_64		(0x0006)
#define		NXC2600_MMC_CLKRT_CLK_RATE_1_128	(0x0007)
#define	NXC2600_MMC_CMDAT			(0x1002100c)
#define		NXC2600_MMC_CMDAT_RESPONSE_FORMAT_MASK		(0x00000007)
#define		NXC2600_MMC_CMDAT_RESPONSE_FORMAT_R1_R1B	(0x00000001)
#define		NXC2600_MMC_CMDAT_RESPONSE_FORMAT_R2		(0x00000002)
#define		NXC2600_MMC_CMDAT_RESPONSE_FORMAT_R3		(0x00000003)
#define		NXC2600_MMC_CMDAT_RESPONSE_FORMAT_R4		(0x00000004)
#define		NXC2600_MMC_CMDAT_RESPONSE_FORMAT_R5		(0x00000005)
#define		NXC2600_MMC_CMDAT_RESPONSE_FORMAT_R6		(0x00000006)
#define		NXC2600_MMC_CMDAT_DATA_EN			(0x00000008)
#define		NXC2600_MMC_CMDAT_WRITE				(0x00000010)
#define		NXC2600_MMC_CMDAT_STREAM_BLOCK			(0x00000020)
#define		NXC2600_MMC_CMDAT_BUSY				(0x00000040)
#define		NXC2600_MMC_CMDAT_INIT				(0x00000080)
#define		NXC2600_MMC_CMDAT_DMA_EN			(0x00000100)
#define		NXC2600_MMC_CMDAT_BUS_WIDTH_MASK		(0x00000600)
#define		NXC2600_MMC_CMDAT_BUS_WIDTH_1BIT		(0x00000200)
#define		NXC2600_MMC_CMDAT_BUS_WIDTH_4BIT		(0x00000400)
#define		NXC2600_MMC_CMDAT_IO_ABORT			(0x00000800)
#define	NXC2600_MMC_RESTO			(0x10021010)
#define		NXC2600_MMC_RESTO_RES_TO(resto)		((resto)&0x00ff)
#define	NXC2600_MMC_RDTO			(0x10021014)
#define	NXC2600_MMC_BLKLEN			(0x10021018)
#define		NXC2600_MMC_BLKLEN_BLK_LEN(bytes)	((bytes)&0x0fff)
#define	NXC2600_MMC_NOB				(0x1002101c)
#define	NXC2600_MMC_SNOB			(0x10021020)
#define	NXC2600_MMC_IMASK			(0x10021024)
#define		NXC2600_MMC_IMASK_DATA_TRAN_DONE	(0x0001)
#define		NXC2600_MMC_IMASK_PRG_DONE		(0x0002)
#define		NXC2600_MMC_IMASK_END_CMD_RES		(0x0004)
#define		NXC2600_MMC_IMASK_RXFIFO_RD_REQ		(0x0020)
#define		NXC2600_MMC_IMASK_TXFIFO_WR_REQ		(0x0040)
#define		NXC2600_MMC_IMASK_SDIO			(0x0080)
#define	NXC2600_MMC_IREG			(0x10021028)
#define		NXC2600_MMC_IREG_DATA_TRAN_DONE		(0x0001)
#define		NXC2600_MMC_IREG_PRG_DONE		(0x0002)
#define		NXC2600_MMC_IREG_END_CMD_RES		(0x0004)
#define		NXC2600_MMC_IREG_RXFIFO_RD_REQ		(0x0020)
#define		NXC2600_MMC_IREG_TXFIFO_WR_REQ		(0x0040)
#define		NXC2600_MMC_IREG_SDIO			(0x0080)
#define	NXC2600_MMC_CMD				(0x1002102c)
#define		NXC2600_MMC_CMD_INDEX(index)		((index)&0x3f)
#define	NXC2600_MMC_ARG				(0x10021030)
#define	NXC2600_MMC_RES				(0x10021034)
#define	NXC2600_MMC_RXFIFO			(0x10021038)
#define	NXC2600_MMC_TXFIFO			(0x1002103c)
#define NXC2600_MMC_MAX_BLOCK_SIZE		(0x0fff)
#define	NXC2600_MMC_MAX_BLOCK_COUNT		(0xffff)

#ifndef __ASSEMBLY__

/* MMC/SD operations */

#ifndef CONFIG_NXC2600_MMC_POWER_ENABLE_PIN
#define	CONFIG_NXC2600_MMC_POWER_ENABLE_PIN	(91)
#endif
#ifndef CONFIG_NXC2600_MMC_HOTPLUG_PIN
#define	CONFIG_NXC2600_MMC_HOTPLUG_PIN	(90)
#endif
#define	NXC2600_MMC_HOTPLUG_IRQ		(NXC2600_GPIO_NIRQ(CONFIG_NXC2600_MMC_HOTPLUG_PIN))
static __inline__ void
nxc2600_mmc_init(void)
{
	nxc2600_gpio_vs_msc(0);
	nxc2600_set_gpio_output_mode(CONFIG_NXC2600_MMC_POWER_ENABLE_PIN,1);
}

static __inline__ uint32_t
nxc2600_mmc_get_response(uint32_t *buf,
			 int size)
{
	uint16_t resp;
	uint32_t tmp[5];
	switch(size)
	{
		case 1:
			resp = REG16(NXC2600_MMC_RES);
			tmp[0] = ((resp>>8)&0xff);
			tmp[1] = resp&0xff;
			resp = REG16(NXC2600_MMC_RES);
			tmp[2] = ((resp>>8)&0xff);
                        tmp[3] = resp&0xff;
                        resp = REG16(NXC2600_MMC_RES);
			tmp[4] = resp&0xff;
			buf[0] = (tmp[1] << 24) | 
				 (tmp[2] << 16) | 
				 (tmp[3] << 8) | 
				 (tmp[4]) ;
			break;
		case 4:
			{
			int i;
			uint32_t v,w1,w2,data;
			data = REG16(NXC2600_MMC_RES);
			v = data & 0xffff;
 			for (i = 0; i < 4; i++) 
			{
				data = REG16(NXC2600_MMC_RES);
				w1 = data & 0xffff;
				data = REG16(NXC2600_MMC_RES);
				w2 = data & 0xffff;
				buf[i] = v << 24 | w1 << 8 | w2 >> 8;
				v = w2;
			}
			}
	}
	return 0;
}

static __inline__ void
nxc2600_mmc_reset(void)
{
	REG16(NXC2600_MMC_STRPCL) |= NXC2600_MMC_STRPCL_RESET;
	while(REG32(NXC2600_MMC_STAT)&NXC2600_MMC_STAT_IS_RESETTING) udelay(10);
	REG16(NXC2600_MMC_RESTO) = 0x00ff;
	REG16(NXC2600_MMC_RDTO) = 0xffff;
}

static __inline__ void
nxc2600_mmc_enable_power(void)
{
	nxc2600_set_gpio_output(CONFIG_NXC2600_MMC_POWER_ENABLE_PIN,1);
}

static __inline__ void
nxc2600_mmc_disable_power(void)
{
	nxc2600_set_gpio_output(CONFIG_NXC2600_MMC_POWER_ENABLE_PIN,1);
}

static __inline__ int
nxc2600_mmc_is_card_inserted(void)
{
	return nxc2600_get_gpio_input(CONFIG_NXC2600_MMC_HOTPLUG_PIN);
}

static __inline__ void
nxc2600_mmc_stop_clock(void)
{
	REG16(NXC2600_MMC_STRPCL) = (REG16(NXC2600_MMC_STRPCL)&~NXC2600_MMC_STRPCL_CLOCK_MASK)|NXC2600_MMC_STRPCL_CLOCK_STOP;
	while(REG32(NXC2600_MMC_STAT)&NXC2600_MMC_STAT_CLK_EN) udelay(3);
}

static __inline__ void
nxc2600_mmc_start_clock(void)
{
        REG16(NXC2600_MMC_STRPCL) = (REG16(NXC2600_MMC_STRPCL)&~NXC2600_MMC_STRPCL_CLOCK_MASK)|NXC2600_MMC_STRPCL_CLOCK_START|NXC2600_MMC_STRPCL_START_OP;
	while(!(REG32(NXC2600_MMC_STAT)&NXC2600_MMC_STAT_CLK_EN)) udelay(3);
}

static __inline__ void
nxc2600_mmc_set_clock(int is_sd,
		      uint32_t rate)
{
	uint32_t clk;
	uint16_t clkrt;
	clk = ((is_sd)? 24000000: 20000000);
	clkrt = 0;
        while (rate < clk)
        {
                clkrt++;
                clk>>=1;
        }
	nxc2600_mmc_stop_clock();
	nxc2600_set_msc_clock(2457600);
	REG32(NXC2600_MMC_CLKRT) = clkrt;
}

static __inline__ void
nxc2600_mmc_disable_interrupt(uint32_t mask)
{
	REG16(NXC2600_MMC_IMASK) |= mask;
}

static __inline__ void
nxc2600_mmc_enable_interrupt(uint32_t unmask)
{
	REG16(NXC2600_MMC_IMASK) &= ~unmask;
}

static __inline__ void
nxc2600_mmc_clear_status(void)
{
	REG16(NXC2600_MMC_IREG) = 0xff;
}

static __inline__ int
nxc2600_mmc_send_cmd( uint8_t cmd,
		      uint32_t arg,
		      uint32_t cmdat )
{

	nxc2600_mmc_stop_clock();
	nxc2600_mmc_clear_status();
	REG8(NXC2600_MMC_CMD) = cmd;
	REG32(NXC2600_MMC_ARG) = arg;
	REG32(NXC2600_MMC_CMDAT) = cmdat;
	nxc2600_mmc_start_clock();
	return 0;
}

static __inline__ int
nxc2600_mmc_set_block_size(uint32_t size)
{
	if(size>2048)
		return -1;

	REG16(NXC2600_MMC_BLKLEN) = size;
	return 0;
}

static __inline__ int
nxc2600_mmc_set_block_count(uint32_t count)
{
	if(count>0xffff)
		return -1;
	REG16(NXC2600_MMC_NOB) = count;
	return 0;
}
#endif



#define NXC2600_UDC_EP_TYPE_CONTROL     (0x00000000)
#define NXC2600_UDC_EP_TYPE_ISO         (0x00000001)
#define NXC2600_UDC_EP_TYPE_BLUK        (0x00000002)
#define NXC2600_UDC_EP_TYPE_INTERRUPT   (0x00000003)
#define NXC2600_UDC_ICR(ep)             (0x13040000+0x20*(ep))
#define NXC2600_UDC_OCR0                (0x13040200)
#define NXC2600_UDC_OCR(ep)             (0x130402a0+0x20*((ep)-5))
#define         NXC2600_UDC_CR_STALL            (0x00000001)
#define         NXC2600_UDC_CR_FLUSH            (0x00000002)
#define         NXC2600_UDC_CR_SNOOP            (0x00000004)
#define         NXC2600_UDC_CR_TYPE_MASK        (0x00000030)
#define         NXC2600_UDC_CR_TYPE(type)       (((type)&0x3)<<4)
#define NXC2600_UDC_ISR(ep)             ((ep)? (0x13040024+0x20*((ep)-1)):0x13040004)
#define NXC2600_UDC_OSR(ep)             ((ep)? (0x130402a4+0x20*((ep)-5)):0x13040204)
#define         NXC2600_UDC_SR_PID_MASK         (0x0000000f)
#define         NXC2600_UDC_SR_PID(pid)         ((pid)&0xf)
#define         NXC2600_UDC_SR_OPS_MASK         (0x00000030)
#define		NXC2600_UDC_SR_OPS_NONE		(0x00000000)
#define		NXC2600_UDC_SR_OPS_RECEIVED	(0x00000010)
#define		NXC2600_UDC_SR_OPS_SETUP	(0x00000020)
#define         NXC2600_UDC_SR_IPS              (0x00000040)
#define         NXC2600_UDC_SR_RPS_MASK         (0x003ff800)
#define         NXC2600_UDC_SR_RPS(sr)         (((sr)>>11)&0x7ff)
#define NXC2600_UDC_IBSR(ep)            (0x13040008+0x20*(ep))
#define         NXC2600_UDC_IBSR_BS_MASK        (0x0000ffff)
#define         NXC2600_UDC_IBSR_BS(bs)         ((((bs)-1)&0x7)+1)
#define NXC2600_UDC_OFNR0               (0x13040208)
#define NXC2600_UDC_OFNR(ep)            (0x130402a8+0x20*((ep)-5))
#define         NXC2600_UDC_OFNR_FN_MASK        (0x00003fff)
#define         NXC2600_UDC_OFNR_FN(fn)         ((fn)&0x3fff)
#define NXC2600_UDC_IMPR(ep)            (0x1304000c+0x20*(ep))
#define NXC2600_UDC_OMPR0               (0x1304020c)
#define NXC2600_UDC_OMPR(ep)            (0x130402ac+0x20*((ep)-5))
#define         NXC2600_UDC_MPR_MPS_MASK        (0x0000ffff)
#define         NXC2600_UDC_MPR_MPS(mps)        ((((mps)-1)&0x3f)+1)
#define NXC2600_UDC_IPR                 (0x13040414)
#define         NXC2600_UDC_IPR_IIP_MASK        (0x0000ffff)
#define         NXC2600_UDC_IPR_IIP(iip)        (1<<(iip))
#define         NXC2600_UDC_IPR_OIP_MASK        (0xffff0000)
#define         NXC2600_UDC_IPR_OIP(oip)        ((1<<(oip))<<16)
#define NXC2600_UDC_IMR                 (0x13040418)
#define         NXC2600_UDC_IMR_IIM_MASK        (0x0000ffff)
#define         NXC2600_UDC_IMR_IIM(iim)        (1<<(iim))
#define         NXC2600_UDC_IMR_OIM_MASK        (0xffff0000)
#define         NXC2600_UDC_IMR_OIM(oim)        ((1<<(oim))<<16)
#define NXC2600_UDC_INFR(ep)            (0x13040504+0x4*(ep))
#define         NXC2600_UDC_INFR_EPN_MASK       (0x0000000f)
#define         NXC2600_UDC_INFR_EPN(epn)       ((epn)&0xf)
#define         NXC2600_UDC_INFR_EPD            (0x00000010)
#define         NXC2600_UDC_INFR_EPT_MASK       (0x00000060)
#define         NXC2600_UDC_INFR_EPT(type)      (((type)&0x3)<<5)
#define         NXC2600_UDC_INFR_ECGN_MASK      (0x00000780)
#define         NXC2600_UDC_INFR_ECGN(ecgn)     (((ecgn)&0xf)<<7)
#define         NXC2600_UDC_INFR_EIFN_MASK      (0x00007800)
#define         NXC2600_UDC_INFR_EIFN(eifn)     (((eifn)&0xf)<<11)
#define         NXC2600_UDC_INFR_EALTS_MASK     (0x00078000)
#define         NXC2600_UDC_INFR_EALTS(ealts)   (((ealts)&0xf)<<15)
#define         NXC2600_UDC_INFR_EMPS_MASK      (0x1ff80000)
#define         NXC2600_UDC_INFR_EMPS(emps)     (((emps)&0xfff)<<19)
#define NXC2600_UDC_DCFGR               (0x13040400)
#define         NXC2600_UDC_DCFGR_SPEED_MASK    (0x00000003)
#define         NXC2600_UDC_DCFGR_SPEED_HIGH    (0x00000000)
#define         NXC2600_UDC_DCFGR_SPEED_LOW     (0x00000002)
#define         NXC2600_UDC_DCFGR_SPEED_FULL    (0x00000003)
#define         NXC2600_UDC_DCFGR_RW            (0x00000004)
#define         NXC2600_UDC_DCFGR_SP            (0x00000008)
#define         NXC2600_UDC_DCFGR_SF            (0x00000010)
#define NXC2600_UDC_DCR                 (0x13040404)
#define         NXC2600_UDC_DCR_RESUME          (0x00000001)
#define NXC2600_UDC_DSR                 (0x13040408)
#define         NXC2600_UDC_DSR_CFG_MASK        (0x0000000f)
#define         NXC2600_UDC_DSR_CFG(num)        ((num)&0xf)
#define         NXC2600_UDC_DSR_INTF_MASK       (0x000000f0)
#define         NXC2600_UDC_DSR_INTF(num)       (((num)&0xf)<<4)
#define         NXC2600_UDC_DSR_ALT_MASK        (0x00000f00)
#define         NXC2600_UDC_DSR_ALT(alt)        (((alt)&0xf)<<8)
#define         NXC2600_UDC_DSR_SFN_MASK        (0xfffc0000)
#define         NXC2600_UDC_DSR_SFN(num)        (((num)&0x3fff)<<18)
#define NXC2600_UDC_DIMR                (0x13040410)
#define NXC2600_UDC_DIPR                (0x1304040c)
#define         NXC2600_UDC_DIPR_SC             (0x00000001)
#define         NXC2600_UDC_DIPR_SI             (0x00000002)
#define         NXC2600_UDC_DIPR_URST           (0x00000008)
#define         NXC2600_UDC_DIPR_USUS           (0x00000010)
#define         NXC2600_UDC_DIPR_SOF            (0x00000020)
#define NXC2600_UDC_SCAR                (0x13040500)
#define         NXC2600_UDC_SCAR_SCA(addr)      ((addr)&0xffff)
#define NXC2600_UDC_FIFO_UINT_SIZE              (4)
#define NXC2600_UDC_MAX_PACKET_SIZE             (64)
#define NXC2600_UDC_MIN_PACKET_SIZE             (8)
#define NXC2600_UDC_RXFIFO              (0x13040800)
#define NXC2600_UDC_TXFIFO(ep)          (0x13040840+((ep)?(32+(((ep)-1)*64)):0))
#define NXC2600_UDC_TCFM		(0x1304041c)
#define NXC2600_UDC_TXZLP		(0x13040420)

#ifndef __ASSEMBLY__

/*

 */
static __inline__ void
nxc2600_udc_init(void)
{
#if 1
        REG32(NXC2600_UDC_DCFGR) = NXC2600_UDC_DCFGR_SPEED_FULL|
                                   NXC2600_UDC_DCFGR_RW|
                                   NXC2600_UDC_DCFGR_SF;
        REG32(NXC2600_UDC_DCR) = 0;
        REG32(NXC2600_UDC_DIMR) = 0;
        REG32(NXC2600_UDC_IMR) = 0xffffffff;
	REG32(NXC2600_UDC_IPR) = 0xffffffff;
        REG32(NXC2600_UDC_SCAR) = NXC2600_UDC_SCAR_SCA(0xffff);

        REG32(NXC2600_UDC_OMPR0) = NXC2600_UDC_MPR_MPS(32);
        REG32(NXC2600_UDC_OCR0) = NXC2600_UDC_CR_TYPE(NXC2600_UDC_EP_TYPE_CONTROL);

        REG32(NXC2600_UDC_ICR(0)) = NXC2600_UDC_CR_FLUSH;
        REG32(NXC2600_UDC_ICR(0)) = NXC2600_UDC_CR_TYPE(NXC2600_UDC_EP_TYPE_CONTROL);

        REG32(NXC2600_UDC_INFR(0)) = NXC2600_UDC_INFR_EPT(NXC2600_UDC_EP_TYPE_CONTROL)|NXC2600_UDC_INFR_EPN(0)|NXC2600_UDC_INFR_ECGN(0xf);
        REG32(NXC2600_UDC_IBSR(0)) = NXC2600_UDC_IBSR_BS(32/4);

#endif
#if 1
	REG32(NXC2600_UDC_DIMR) = 0x32;
	REG32(NXC2600_UDC_DCFGR) = 0x17;
	REG32(NXC2600_UDC_DCR) = 0;

	REG32(NXC2600_UDC_ICR(0)) = (0<<4)|(1<<1);
	REG32(NXC2600_UDC_ICR(0)) = (0<<4);
	REG32(NXC2600_UDC_ICR(1)) = (3<<4)|(1<<1);
        REG32(NXC2600_UDC_ICR(1)) = (3<<4);
	REG32(NXC2600_UDC_ICR(2)) = (2<<4)|(1<<1);
        REG32(NXC2600_UDC_ICR(2)) = (2<<4);
	REG32(NXC2600_UDC_ICR(3)) = (2<<4)|(1<<1);
        REG32(NXC2600_UDC_ICR(3)) = (2<<4);
	REG32(NXC2600_UDC_ICR(4)) = (1<<4)|(1<<1);
        REG32(NXC2600_UDC_ICR(4)) = (1<<4);
	REG32(NXC2600_UDC_OCR0) = (0<<4);
	REG32(NXC2600_UDC_OCR(5)) = (2<<4);
	REG32(NXC2600_UDC_OCR(6)) = (2<<4);
	REG32(NXC2600_UDC_OCR(7)) = (1<<4);
	REG32(NXC2600_UDC_ISR(0)) = 0;
	REG32(NXC2600_UDC_ISR(1)) = 0;
	REG32(NXC2600_UDC_ISR(2)) = 0;
	REG32(NXC2600_UDC_ISR(3)) = 0;
	REG32(NXC2600_UDC_ISR(4)) = 0;
	REG32(NXC2600_UDC_OSR(0)) = 0;
	REG32(NXC2600_UDC_OSR(5)) = 0;
        REG32(NXC2600_UDC_OSR(6)) = 0;
        REG32(NXC2600_UDC_OSR(7)) = 0;
	REG32(NXC2600_UDC_IBSR(0)) = 32/4;
	REG32(NXC2600_UDC_IBSR(1)) = 64/4;
        REG32(NXC2600_UDC_IBSR(2)) = 64/4;
        REG32(NXC2600_UDC_IBSR(3)) = 64/4;
        REG32(NXC2600_UDC_IBSR(4)) = 64/4;
	REG32(NXC2600_UDC_IMPR(0)) = 32;
        REG32(NXC2600_UDC_IMPR(1)) = 64;
        REG32(NXC2600_UDC_IMPR(2)) = 64;
        REG32(NXC2600_UDC_IMPR(3)) = 64;
        REG32(NXC2600_UDC_IMPR(4)) = 64;
	REG32(NXC2600_UDC_OMPR0) = 32;
	REG32(NXC2600_UDC_OMPR(5)) = 64;
        REG32(NXC2600_UDC_OMPR(6)) = 64;
        REG32(NXC2600_UDC_OMPR(7)) = 64;

	REG32(NXC2600_UDC_INFR(0)) = (32<<19)|(0<<15)|(0<<11)|(1<<7)|(0<<5)|(0<<4)|(0<<0);
	REG32(NXC2600_UDC_INFR(1)) = (64<<19)|(0<<15)|(0<<11)|(1<<7)|(3<<5)|(1<<4)|(1<<0);
	REG32(NXC2600_UDC_INFR(2)) = (64<<19)|(0<<15)|(0<<11)|(1<<7)|(2<<5)|(1<<4)|(2<<0);
	REG32(NXC2600_UDC_INFR(3)) = (64<<19)|(0<<15)|(0<<11)|(1<<7)|(2<<5)|(1<<4)|(3<<0);
	REG32(NXC2600_UDC_INFR(4)) = (64<<19)|(0<<15)|(0<<11)|(1<<7)|(1<<5)|(1<<4)|(4<<0);
	REG32(NXC2600_UDC_INFR(5)) = (64<<19)|(0<<15)|(0<<11)|(1<<7)|(2<<5)|(0<<4)|(5<<0);
	REG32(NXC2600_UDC_INFR(6)) = (64<<19)|(0<<15)|(0<<11)|(1<<7)|(2<<5)|(0<<4)|(6<<0);
	REG32(NXC2600_UDC_INFR(7)) = (64<<19)|(0<<15)|(0<<11)|(1<<7)|(1<<5)|(0<<4)|(7<<0);
#endif
	REG32(NXC2600_UDC_SCAR) = 0xffff;
	REG32(NXC2600_UDC_IPR) = 0xffffffff;
	REG32(NXC2600_UDC_IMR) = 0;
}

static __inline__ int
nxc2600_udc_ep_init(uint32_t ep,
                    uint32_t packet_size,
                    uint32_t type)
{
        if(packet_size%4)
                return -1;
        if((packet_size<NXC2600_UDC_MIN_PACKET_SIZE) ||
           (packet_size>NXC2600_UDC_MAX_PACKET_SIZE))
                return -1;
        
        REG32(NXC2600_UDC_INFR(ep)) = (REG32(NXC2600_UDC_INFR(ep))&~(NXC2600_UDC_INFR_EPT_MASK|NXC2600_UDC_INFR_EMPS_MASK|NXC2600_UDC_INFR_EPD|NXC2600_UDC_INFR_EPN_MASK))|NXC2600_UDC_INFR_EPT(type)|((ep<5)?NXC2600_UDC_INFR_EPD:0)|NXC2600_UDC_INFR_EPN(ep);
	
        if(ep<5)
        {
                REG32(NXC2600_UDC_IBSR(ep)) = NXC2600_UDC_IBSR_BS(packet_size/4);
                REG32(NXC2600_UDC_IMPR(ep)) = NXC2600_UDC_MPR_MPS(packet_size);
                REG32(NXC2600_UDC_ICR(ep)) = NXC2600_UDC_CR_FLUSH;
                REG32(NXC2600_UDC_ICR(ep)) = (REG32(NXC2600_UDC_ICR(ep))|NXC2600_UDC_CR_TYPE_MASK)|NXC2600_UDC_CR_TYPE(type);
 //               REG32(NXC2600_UDC_IMR) &= ~NXC2600_UDC_IMR_IIM(ep);
        }
        else
        {
                REG32(NXC2600_UDC_OMPR(ep)) = NXC2600_UDC_MPR_MPS(packet_size);
                REG32(NXC2600_UDC_OCR(ep)) = (REG32(NXC2600_UDC_OCR(ep))|NXC2600_UDC_CR_TYPE_MASK)|NXC2600_UDC_CR_TYPE(type);
 //               REG32(NXC2600_UDC_IMR) &= ~NXC2600_UDC_IMR_OIM(ep);
        }
}

static __inline__ int
nxc2600_udc_ep_stop(uint32_t ep)
{
        if(ep<5)
        {
                //REG32(NXC2600_UDC_IMR) |= NXC2600_UDC_IPR_IIM(ep);
        }
        else
        {
                //REG32(NXC2600_UDC_IMR) |= NXC2600_UDC_IPR_OIM(ep);
        }
	return 0;
}

static __inline__ int
nxc2600_udc_ep_write_data(uint32_t ep,
			  void *buf,
			  int count)
{
	void *addr = (void*)NXC2600_UDC_TXFIFO(ep);
	if(count==0)
	{
		REG32(NXC2600_UDC_TXZLP) = 0;
		REG32(addr) = 0;
	}
	else
	{
	while(count>=4)
        {
                REG32(addr) = *(uint32_t*)buf;
                buf+=4;
                count-=4;
		addr+=4;
        }
	while(count>0)
	{
		REG8(addr) = *(uint8_t*)buf;
		buf++;
		count--;
	}
	}
	//REG32(NXC2600_UDC_ICR(ep)) |= NXC2600_UDC_CR_FLUSH;
        REG32(NXC2600_UDC_TCFM) = 0;
	return 0;
}

static __inline__ int
nxc2600_udc_ep_read_data(uint32_t ep,
			  void *buf,
			  int count)
{
        while(count>=4)
        {
                *(uint32_t*)buf = REG32(NXC2600_UDC_RXFIFO);
                buf += 4;
                count-=4;
        }
        while(count>0)
        {
                *(uint8_t*)buf = REG8(NXC2600_UDC_RXFIFO);
                buf++;
                count--;
        }

        //{uint32_t tmp=REG32(NXC2600_UDC_TCFM);}
	return 0;
}

#endif



#define NXC2600_SSI				(0x10043000)
#define NXC2600_SSI_DR                          (0x10043000)
#define         NXC2600_SSI_DR_DATA_MASK                (0x0000ffff)
#define         NXC2600_SSI_DR_DATA(data)               ((data)&NXC2600_SSI_DR_DATA_MASK)
#define         NXC2600_SSI_DR_GPC                      (0x00010000)
#define NXC2600_SSI_CR0                         (0x10043004)
#define         NXC2600_SSI_CR0_DISREV                  (0x0001)
#define         NXC2600_SSI_CR0_RFLUSH                  (0x0002)
#define         NXC2600_SSI_CR0_TFLUSH                  (0x0004)
#define         NXC2600_SSI_CR0_FSEL                    (0x0040)
#define         NXC2600_SSI_CR0_RFINC                   (0x0100)
#define         NXC2600_SSI_CR0_RFINE                   (0x0200)
#define         NXC2600_SSI_CR0_LOOP                    (0x0400)
#define         NXC2600_SSI_CR0_REIE                    (0x0800)
#define         NXC2600_SSI_CR0_TEIE                    (0x1000)
#define         NXC2600_SSI_CR0_RIE                     (0x2000)
#define         NXC2600_SSI_CR0_TIE                     (0x4000)
#define         NXC2600_SSI_CR0_SSIE                    (0x8000)
#define NXC2600_SSI_CR1                         (0x10043008)
#define         NXC2600_SSI_CR1_POL                     (0x00000001)
#define         NXC2600_SSI_CR1_PHA                     (0x00000002)
#define         NXC2600_SSI_CR1_FLEN_MASK               (0x000000f0)
#define         NXC2600_SSI_CR1_FLEN(len)               ((((len)-2)&0xf)<<4)
#define         NXC2600_SSI_CR1_RTRG_MASK               (0x00000300)
#define         NXC2600_SSI_CR1_RTRG_1                  (0x00000000)
#define         NXC2600_SSI_CR1_RTRG_4                  (0x00000100)
#define         NXC2600_SSI_CR1_RTRG_8                  (0x00000200)
#define         NXC2600_SSI_CR1_RTRG_14                 (0x00000300)
#define         NXC2600_SSI_CR1_TTRG_MASK               (0x00000c00)
#define         NXC2600_SSI_CR1_TTRG_1                  (0x00000000)
#define         NXC2600_SSI_CR1_TTRG_4                  (0x00000400)
#define         NXC2600_SSI_CR1_TTRG_8                  (0x00000800)
#define         NXC2600_SSI_CR1_TTRG_14                 (0x00000c00)
#define         NXC2600_SSI_CR1_MCOM_MASK               (0x0000f000)
#define         NXC2600_SSI_CR1_MCOM(bit)               (((bit)&4)<<12)
#define         NXC2600_SSI_CR1_FMAT_MASK               (0x00300000)
#define         NXC2600_SSI_CR1_FMAT_MOTO_SPI           (0x00000000)
#define         NXC2600_SSI_CR1_FMAT_TI_SSP             (0x00100000)
#define         NXC2600_SSI_CR1_FMAT_NATIONAL_1         (0x00200000)
#define         NXC2600_SSI_CR1_FMAT_NATIONAL_2         (0x00300000)
#define         NXC2600_SSI_CR1_MULTS                   (0x00400000)
#define         NXC2600_SSI_CR1_UNFIN                   (0x00800000)
#define         NXC2600_SSI_CR1_ITFRM                   (0x01000000)
#define         NXC2600_SSI_CR1_LFST                    (0x02000000)
#define         NXC2600_SSI_CR1_TCKFI_MASK              (0x0c000000)
#define         NXC2600_SSI_CR1_TCKFI(delay)            (((delay)&3)<<26)
#define         NXC2600_SSI_CR1_TFVCK_MASK              (0x30000000)
#define         NXC2600_SSI_CR1_TFVCK(delay)            (((delay)&3)<<28)
#define         NXC2600_SSI_CR1_FRMHL_MASK              (0xc0000000)
#define         NXC2600_SSI_CR1_FRMHL_CE_LOW_CE2_LOW    (0x00000000)
#define         NXC2600_SSI_CR1_FRMHL_CE_HIGH_CE2_LOW   (0x40000000)
#define         NXC2600_SSI_CR1_FRMHL_CE_LOW_CE2_HIGH   (0x80000000)
#define         NXC2600_SSI_CR1_FRMHL_CE_HIGH_CE2_HIGH  (0xc0000000)
#define NXC2600_SSI_SR                          (0x1004300c)
#define         NXC2600_SSI_SR_OVER                     (0x00000001)
#define         NXC2600_SSI_SR_UNDR                     (0x00000002)
#define         NXC2600_SSI_SR_RFHF                     (0x00000004)
#define         NXC2600_SSI_SR_TFHE                     (0x00000008)
#define         NXC2600_SSI_SR_RFE                      (0x00000010)
#define         NXC2600_SSI_SR_TFF                      (0x00000020)
#define         NXC2600_SSI_SR_BUSY                     (0x00000040)
#define         NXC2600_SSI_SR_END                      (0x00000080)
#define         NXC2600_SSI_SR_RFIFO_NUM(reg)           (((reg)&0x00001f00)>>8)
#define         NXC2600_SSI_SR_TFIFO_NUM(reg)           (((reg)&0x0003e000)>>13)
#define NXC2600_SSI_ITR                         (0x10043010)
#define         NXC2600_SSI_ITR_IVLTM_MASK              (0x7fff)
#define         NXC2600_SSI_ITR_IVLTM
#define         NXC2600_SSI_ITR_CNTCLK                  (0x8000)
#define NXC2600_SSI_ICR                         (0x10043014)
#define         NXC2600_SSI_ICR_ICC_MASK                (0x07)
#define         NXC2600_SSI_ICR_ICC(num)                ((num)&0x07)
#define NXC2600_SSI_GR                          (0x10043018)
#define         NXC2600_SSI_GR_CGV_MASK                 (0x00ff)
#define         NXC2600_SSI_GR_CGV(clk)                 ((clk)&0x00ff)

#ifndef __ASSEMBLY__

/* SSI operations */

static __inline__ void
nxc2600_ssi_disable(void)
{
	REG16(NXC2600_SSI_CR0) &= ~NXC2600_SSI_CR0_SSIE;
	REG32(NXC2600_SSI_SR) &= ~(NXC2600_SSI_SR_OVER|NXC2600_SSI_SR_UNDR);
	REG16(NXC2600_SSI_CR0) |= NXC2600_SSI_CR0_RFLUSH|NXC2600_SSI_CR0_TFLUSH||NXC2600_SSI_CR0_DISREV;

}

static __inline__ void
nxc2600_ssi_enable(void)
{
	REG16(NXC2600_SSI_CR0) |= NXC2600_SSI_CR0_SSIE; //|NXC2600_SSI_CR0_TIE|NXC2600_SSI_CR0_RIE|NXC2600_SSI_CR0_TEIE|NXC2600_SSI_CR0_REIE;

}

static __inline__ void
nxc2600_ssi_set_clock(uint32_t clock)
{
	REG16(NXC2600_SSI_CR0) |= NXC2600_SSI_CR0_SSIE;
        nxc2600_enable_clock(NXC2600_CDR_SSI);
	nxc2600_set_ssi_clock(clock);
        REG16(NXC2600_SSI_GR) = (REG16(NXC2600_SSI_GR)&~NXC2600_SSI_GR_CGV_MASK)|NXC2600_SSI_GR_CGV(48000000/2/clock-1);
}

#define NXC2600_SSI_FORMAT_MOTO_SPI     (NXC2600_SSI_CR1_FMAT_MOTO_SPI)
#define NXC2600_SSI_FORMAT_TI_SSP       (NXC2600_SSI_CR1_FMAT_TI_SSP)
#define NXC2600_SSI_FORMAT_NATIONAL_1   (NXC2600_SSI_CR1_FMAT_NATIONAL_1)
#define NXC2600_SSI_FORMAT_NATIONAL_2   (NXC2600_SSI_CR1_FMAT_NATIONAL_2)
#define NXC2600_SSI_MODE_CE     (0)
#define NXC2600_SSI_MODE_CE2    (1)
#define NXC2600_SSI_MODE_GPC    (2)

static __inline__ int
nxc2600_ssi_init(uint32_t mode,
                 uint32_t format,
                 uint32_t msb,
                 uint32_t data_bits,
                 uint32_t clock )
{
        uint32_t cr0,cr1;

        nxc2600_ssi_disable();
	REG32(NXC2600_SSI_CR1) &= ~NXC2600_SSI_CR1_UNFIN;
        REG32(NXC2600_SSI_SR) &= ~(NXC2600_SSI_SR_OVER|NXC2600_SSI_SR_UNDR);
        REG16(NXC2600_SSI_CR0) |= NXC2600_SSI_CR0_DISREV;
	nxc2600_ssi_set_clock( clock );
	nxc2600_gpio_vs_ssi(0);

        cr0 = REG16(NXC2600_SSI_CR0);

        cr1 = REG32(NXC2600_SSI_CR1)&~(NXC2600_SSI_CR1_LFST|NXC2600_SSI_CR1_FMAT_MASK|NXC2600_SSI_CR1_FLEN_MASK|NXC2600_SSI_CR1_TCKFI_MASK|NXC2600_SSI_CR1_TFVCK_MASK|NXC2600_SSI_CR1_POL|NXC2600_SSI_CR1_PHA|NXC2600_SSI_CR1_UNFIN|NXC2600_SSI_CR0_RFINE);

        cr1 |= NXC2600_SSI_CR1_FLEN(data_bits);
        if(!msb) cr1 |= NXC2600_SSI_CR1_LFST;
	else cr1 &= ~NXC2600_SSI_CR1_LFST;

        switch(format)
        {
                case NXC2600_SSI_FORMAT_MOTO_SPI:
                        cr1 |= NXC2600_SSI_CR1_FMAT_MOTO_SPI;
                        cr1 |= (NXC2600_SSI_CR1_TCKFI(3)|NXC2600_SSI_CR1_TFVCK(3));
			cr1 |= NXC2600_SSI_CR1_PHA|NXC2600_SSI_CR1_POL;
                        break;
                case NXC2600_SSI_FORMAT_TI_SSP:
                        cr1 &= ~NXC2600_SSI_CR1_UNFIN;
                        cr1 |= NXC2600_SSI_CR1_FMAT_TI_SSP;
                        break;
                case NXC2600_SSI_FORMAT_NATIONAL_1:
                        cr0 &= ~NXC2600_SSI_CR0_RFINE;
                        cr1 |= NXC2600_SSI_CR1_FMAT_NATIONAL_1;
                        cr1 |= (NXC2600_SSI_CR1_TCKFI(3)|NXC2600_SSI_CR1_TFVCK(3));

                        break;
                case NXC2600_SSI_FORMAT_NATIONAL_2:
                        cr0 &= ~NXC2600_SSI_CR0_RFINE;
                        cr1 |= NXC2600_SSI_CR1_FMAT_NATIONAL_2;
                        cr1 |= (NXC2600_SSI_CR1_TCKFI(3)|NXC2600_SSI_CR1_TFVCK(3));

                        break;

        }

	cr1 &= ~NXC2600_SSI_CR1_MULTS;
        switch(mode)
        {
                case NXC2600_SSI_MODE_GPC:
                        cr1 |= NXC2600_SSI_CR1_MULTS;
                case NXC2600_SSI_MODE_CE:
                        cr0 &= ~NXC2600_SSI_CR0_FSEL;
                        break;
                case NXC2600_SSI_MODE_CE2:
                        cr0 |= NXC2600_SSI_CR0_FSEL;
                        break;
                default:
                        return -1;
        }
        cr1 &= ~(NXC2600_SSI_CR1_TTRG_MASK|NXC2600_SSI_CR1_RTRG_MASK);
        cr1 |= NXC2600_SSI_CR1_RTRG_4|NXC2600_SSI_CR1_TTRG_4;
        REG16(NXC2600_SSI_ITR) = 0;
	REG16(NXC2600_SSI_CR0) = cr0;
        REG32(NXC2600_SSI_CR1) = cr1;

	return 0;
}

static __inline__ void
nxc2600_wait_busy(void)
{
	int timeout = 1000;
        while(REG32(NXC2600_SSI_SR)&NXC2600_SSI_SR_BUSY)
        {
                udelay(1);
                if(!timeout--)
                        break;
        }
}

static __inline__ uint32_t
nxc2600_ssi_read( uint8_t *data,
                  uint32_t size )
{
        uint32_t i;
        int timeout;

        if(data==NULL || size<=0)
                return 0;

	if(NXC2600_SSI_SR_RFIFO_NUM(REG32(NXC2600_SSI_SR)))
		REG16(NXC2600_SSI_CR0) |= NXC2600_SSI_CR0_RFLUSH;

	REG16(NXC2600_SSI_CR0) &= ~NXC2600_SSI_CR0_DISREV;

	REG32(NXC2600_SSI_CR1) |= NXC2600_SSI_CR1_UNFIN;

        for(i=0; i<size; i++)
        {

		timeout = 1000;
                while(NXC2600_SSI_SR_TFIFO_NUM(REG32(NXC2600_SSI_SR))>10)
                {
                        udelay(1);
                        if(!timeout--)
                                goto _tx_failed;

                }

		REG32(NXC2600_SSI_DR) = 0;

		if(i==size)
			REG32(NXC2600_SSI_CR1) &= ~NXC2600_SSI_CR1_UNFIN;

                if(REG32(NXC2600_SSI_SR)&NXC2600_SSI_SR_UNDR)
                        REG32(NXC2600_SSI_SR) &= ~NXC2600_SSI_SR_UNDR;

		 timeout = 1000;
                while(!NXC2600_SSI_SR_RFIFO_NUM(REG32(NXC2600_SSI_SR)))
                {
                        udelay(1);
                        if(!timeout--)
                                break;
                }

                data[i] = NXC2600_SSI_DR_DATA(REG32(NXC2600_SSI_DR));


        }
	nxc2600_wait_busy();
_tx_failed:

        return i;
}

static __inline__ uint32_t
nxc2600_ssi_write( uint8_t *data,
                   uint32_t size )
{
        uint32_t i = 0;
        int timeout;

        if(data==NULL || size<=0)
                return 0;
	REG32(NXC2600_SSI_CR1) |= NXC2600_SSI_CR1_UNFIN;

	REG32(NXC2600_SSI_DR) = NXC2600_SSI_DR_DATA(data[0]);


	timeout = 1000;
	while(!(REG32(NXC2600_SSI_SR)&NXC2600_SSI_SR_UNDR))
	{
		udelay(1);
		if(!timeout--)
			return -1;
	}

        for(i=1; i<size; i++)
        {
                timeout = 1000;
                while(NXC2600_SSI_SR_TFIFO_NUM(REG32(NXC2600_SSI_SR))>10) // &NXC2600_SSI_SR_TFF)
                {
                        udelay(1);
                        if(!timeout--)
				goto _tx_failed;

                }

                REG32(NXC2600_SSI_DR) = NXC2600_SSI_DR_DATA(data[i]);
		if(i==size)
			REG32(NXC2600_SSI_CR1) &= ~NXC2600_SSI_CR1_UNFIN;
		if(REG32(NXC2600_SSI_SR)&NXC2600_SSI_SR_UNDR)
			REG32(NXC2600_SSI_SR) &= ~(NXC2600_SSI_SR_UNDR);
        }

_tx_failed:
	nxc2600_wait_busy();

        return i;
}


#endif


#endif
