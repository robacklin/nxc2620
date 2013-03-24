
/*
 * Register bits and API for Wolfson WM97xx series of codecs
 */

#ifndef _LINUX_WM97XX_H
#define _LINUX_WM97XX_H

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/ac97_codec.h>
#include <sound/initval.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/input.h>	/* Input device layer */
#include <linux/platform_device.h>

/*
 * WM97xx AC97 Touchscreen registers
 */
#define AC97_WM97XX_DIGITISER1		0x76
#define AC97_WM97XX_DIGITISER2		0x78
#define AC97_WM97XX_DIGITISER_RD 	0x7a
#define AC97_WM9713_DIG1		0x74
#define AC97_WM9713_DIG2		AC97_WM97XX_DIGITISER1
#define AC97_WM9713_DIG3		AC97_WM97XX_DIGITISER2

/*
 * WM97xx register bits
 */
#define WM97XX_POLL		0x8000	/* initiate a polling measurement */
#define WM97XX_ADCSEL_X		0x1000	/* x coord measurement */
#define WM97XX_ADCSEL_Y		0x2000	/* y coord measurement */
#define WM97XX_ADCSEL_PRES	0x3000	/* pressure measurement */
#define WM97XX_ADCSEL_MASK	0x7000
#define WM97XX_COO		0x0800	/* enable coordinate mode */
#define WM97XX_CTC		0x0400	/* enable continuous mode */
#define WM97XX_CM_RATE_93	0x0000	/* 93.75Hz continuous rate */
#define WM97XX_CM_RATE_187	0x0100	/* 187.5Hz continuous rate */
#define WM97XX_CM_RATE_375	0x0200	/* 375Hz continuous rate */
#define WM97XX_CM_RATE_750	0x0300	/* 750Hz continuous rate */
#define WM97XX_CM_RATE_8K	0x00f0	/* 8kHz continuous rate */
#define WM97XX_CM_RATE_12K	0x01f0	/* 12kHz continuous rate */
#define WM97XX_CM_RATE_24K	0x02f0	/* 24kHz continuous rate */
#define WM97XX_CM_RATE_48K	0x03f0	/* 48kHz continuous rate */
#define WM97XX_CM_RATE_MASK	0x03f0
#define WM97XX_RATE(i)		(((i & 3) << 8) | ((i & 4) ? 0xf0 : 0))
#define WM97XX_DELAY(i)		((i << 4) & 0x00f0)	/* sample delay times */
#define WM97XX_DELAY_MASK	0x00f0
#define WM97XX_SLEN		0x0008	/* slot read back enable */
#define WM97XX_SLT(i)		((i - 5) & 0x7)	/* panel slot (5-11) */
#define WM97XX_SLT_MASK		0x0007
#define WM97XX_PRP_DETW		0x4000	/* detect on, digitise off, wake */
#define WM97XX_PRP_DET		0x8000	/* detect on, digitise off, no wake */
#define WM97XX_PRP_DET_DIG	0xc000	/* setect on, digitise on */
#define WM97XX_RPR		0x2000	/* wake up on pen down */
#define WM97XX_PEN_DOWN		0x8000	/* pen is down */
#define WM97XX_ADCSRC_MASK	0x7000	/* ADC source mask */

#define WM97XX_AUX_ID1		0x8001
#define WM97XX_AUX_ID2		0x8002
#define WM97XX_AUX_ID3		0x8003
#define WM97XX_AUX_ID4		0x8004


/* WM9712 Bits */
#define WM9712_45W		0x1000	/* set for 5-wire touchscreen */
#define WM9712_PDEN		0x0800	/* measure only when pen down */
#define WM9712_WAIT		0x0200	/* wait until adc is read before next sample */
#define WM9712_PIL		0x0100	/* current used for pressure measurement. set 400uA else 200uA */
#define WM9712_MASK_HI		0x0040	/* hi on mask pin (47) stops conversions */
#define WM9712_MASK_EDGE	0x0080	/* rising/falling edge on pin delays sample */
#define	WM9712_MASK_SYNC	0x00c0	/* rising/falling edge on mask initiates sample */
#define WM9712_RPU(i)		(i&0x3f)	/* internal pull up on pen detect (64k / rpu) */
#define WM9712_PD(i)		(0x1 << i)	/* power management */

/* WM9712 Registers */
#define AC97_WM9712_POWER	0x24
#define AC97_WM9712_REV		0x58

/* WM9705 Bits */
#define WM9705_PDEN		0x1000	/* measure only when pen is down */
#define WM9705_PINV		0x0800	/* inverts sense of pen down output */
#define WM9705_BSEN		0x0400	/* BUSY flag enable, pin47 is 1 when busy */
#define WM9705_BINV		0x0200	/* invert BUSY (pin47) output */
#define WM9705_WAIT		0x0100	/* wait until adc is read before next sample */
#define WM9705_PIL		0x0080	/* current used for pressure measurement. set 400uA else 200uA */
#define WM9705_PHIZ		0x0040	/* set PHONE and PCBEEP inputs to high impedance */
#define WM9705_MASK_HI		0x0010	/* hi on mask stops conversions */
#define WM9705_MASK_EDGE	0x0020	/* rising/falling edge on pin delays sample */
#define	WM9705_MASK_SYNC	0x0030	/* rising/falling edge on mask initiates sample */
#define WM9705_PDD(i)		(i & 0x000f)	/* pen detect comparator threshold */


/* WM9713 Bits */
#define WM9713_PDPOL		0x0400	/* Pen down polarity */
#define WM9713_POLL		0x0200	/* initiate a polling measurement */
#define WM9713_CTC		0x0100	/* enable continuous mode */
#define WM9713_ADCSEL_X		0x0002	/* X measurement */
#define WM9713_ADCSEL_Y		0x0004	/* Y measurement */
#define WM9713_ADCSEL_PRES	0x0008	/* Pressure measurement */
#define WM9713_COO		0x0001	/* enable coordinate mode */
#define WM9713_PDEN		0x0800	/* measure only when pen down */
#define WM9713_ADCSEL_MASK	0x00fe	/* ADC selection mask */
#define WM9713_WAIT		0x0200	/* coordinate wait */

/* AUX ADC ID's */
#define TS_COMP1		0x0
#define TS_COMP2		0x1
#define TS_BMON			0x2
#define TS_WIPER		0x3

/* ID numbers */
#define WM97XX_ID1		0x574d
#define WM9712_ID2		0x4c12
#define WM9705_ID2		0x4c05
#define WM9713_ID2		0x4c13

/* Codec GPIO's */
#define WM97XX_MAX_GPIO		16
#define WM97XX_GPIO_1		(1 << 1)
#define WM97XX_GPIO_2		(1 << 2)
#define WM97XX_GPIO_3		(1 << 3)
#define WM97XX_GPIO_4		(1 << 4)
#define WM97XX_GPIO_5		(1 << 5)
#define WM97XX_GPIO_6		(1 << 6)
#define WM97XX_GPIO_7		(1 << 7)
#define WM97XX_GPIO_8		(1 << 8)
#define WM97XX_GPIO_9		(1 << 9)
#define WM97XX_GPIO_10		(1 << 10)
#define WM97XX_GPIO_11		(1 << 11)
#define WM97XX_GPIO_12		(1 << 12)
#define WM97XX_GPIO_13		(1 << 13)
#define WM97XX_GPIO_14		(1 << 14)
#define WM97XX_GPIO_15		(1 << 15)


#define AC97_LINK_FRAME		21	/* time in uS for AC97 link frame */


/*---------------- Return codes from sample reading functions ---------------*/

/* More data is available; call the sample gathering function again */
#define RC_AGAIN			0x00000001
/* The returned sample is valid */
#define RC_VALID			0x00000002
/* The pen is up (the first RC_VALID without RC_PENUP means pen is down) */
#define RC_PENUP			0x00000004
/* The pen is down (RC_VALID implies RC_PENDOWN, but sometimes it is helpful
   to tell the handler that the pen is down but we don't know yet his coords,
   so the handler should not sleep or wait for pendown irq) */
#define RC_PENDOWN			0x00000008

/*
 * The wm97xx driver provides a private API for writing platform-specific
 * drivers.
 */

/* The structure used to return arch specific sampled data into */
struct wm97xx_data {
    int x;
    int y;
    int p;
};

/*
 * Codec GPIO status
 */
enum wm97xx_gpio_status {
    WM97XX_GPIO_HIGH,
    WM97XX_GPIO_LOW
};

/*
 * Codec GPIO direction
 */
enum wm97xx_gpio_dir {
    WM97XX_GPIO_IN,
    WM97XX_GPIO_OUT
};

/*
 * Codec GPIO polarity
 */
enum wm97xx_gpio_pol {
    WM97XX_GPIO_POL_HIGH,
    WM97XX_GPIO_POL_LOW
};

/*
 * Codec GPIO sticky
 */
enum wm97xx_gpio_sticky {
    WM97XX_GPIO_STICKY,
    WM97XX_GPIO_NOTSTICKY
};

/*
 * Codec GPIO wake
 */
enum wm97xx_gpio_wake {
    WM97XX_GPIO_WAKE,
    WM97XX_GPIO_NOWAKE
};

/*
 * Digitiser ioctl commands
 */
#define WM97XX_DIG_START	0x1
#define WM97XX_DIG_STOP		0x2
#define WM97XX_PHY_INIT		0x3
#define WM97XX_AUX_PREPARE	0x4
#define WM97XX_DIG_RESTORE	0x5

struct wm97xx;

extern struct wm97xx_codec_drv wm9705_codec;
extern struct wm97xx_codec_drv wm9712_codec;
extern struct wm97xx_codec_drv wm9713_codec;

/*
 * Codec driver interface - allows mapping to WM9705/12/13 and newer codecs
 */
struct wm97xx_codec_drv {
	u16 id;
	char *name;

	/* read 1 sample */
	int (*poll_sample) (struct wm97xx *, int adcsel, int *sample);

	/* read X,Y,[P] in poll */
	int (*poll_touch) (struct wm97xx *, struct wm97xx_data *);

	int (*acc_enable) (struct wm97xx *, int enable);
	void (*phy_init) (struct wm97xx *);
	void (*dig_enable) (struct wm97xx *, int enable);
	void (*dig_restore) (struct wm97xx *);
	void (*aux_prepare) (struct wm97xx *);
};


/* Machine specific and accelerated touch operations */
struct wm97xx_mach_ops {

	/* accelerated touch readback - coords are transmited on AC97 link */
	int acc_enabled;
	void (*acc_pen_up) (struct wm97xx *);
	int (*acc_pen_down) (struct wm97xx *);
	int (*acc_startup) (struct wm97xx *);
	void (*acc_shutdown) (struct wm97xx *);

	/* interrupt mask control - required for accelerated operation */
	void (*irq_enable) (struct wm97xx *, int enable);

	/* GPIO pin used for accelerated operation */
	int irq_gpio;

	/* pre and post sample - can be used to minimise any analog noise */
	void (*pre_sample) (int);  /* function to run before sampling */
	void (*post_sample) (int);  /* function to run after sampling */
};

struct wm97xx {
	u16 dig[3], id, gpio[6], misc;	/* Cached codec registers */
	u16 dig_save[3];		/* saved during aux reading */
	struct wm97xx_codec_drv *codec;	/* attached codec driver*/
	struct input_dev *input_dev;	/* touchscreen input device */
	struct snd_ac97 *ac97;		/* ALSA codec access */
	struct device *dev;		/* ALSA device */
	struct platform_device *battery_dev;
	struct platform_device *touch_dev;
	struct wm97xx_mach_ops *mach_ops;
	struct mutex codec_mutex;
	struct delayed_work ts_reader;  /* Used to poll touchscreen */
	unsigned long ts_reader_interval; /* Current interval for timer */
	unsigned long ts_reader_min_interval; /* Minimum interval */
	unsigned int pen_irq;		/* Pen IRQ number in use */
	struct workqueue_struct *ts_workq;
	struct work_struct pen_event_work;
	u16 acc_slot;			/* AC97 slot used for acc touch data */
	u16 acc_rate;			/* acc touch data rate */
	unsigned pen_is_down:1;		/* Pen is down */
	unsigned aux_waiting:1;		/* aux measurement waiting */
	unsigned pen_probably_down:1;	/* used in polling mode */
};

/*
 * Codec GPIO access (not supported on WM9705)
 * This can be used to set/get codec GPIO and Virtual GPIO status.
 */
enum wm97xx_gpio_status wm97xx_get_gpio(struct wm97xx *wm, u32 gpio);
void wm97xx_set_gpio(struct wm97xx *wm, u32 gpio,
			  enum wm97xx_gpio_status status);
void wm97xx_config_gpio(struct wm97xx *wm, u32 gpio,
				     enum wm97xx_gpio_dir dir,
				     enum wm97xx_gpio_pol pol,
				     enum wm97xx_gpio_sticky sticky,
				     enum wm97xx_gpio_wake wake);

/* codec AC97 IO access */
int wm97xx_reg_read(struct wm97xx *wm, u16 reg);
void wm97xx_reg_write(struct wm97xx *wm, u16 reg, u16 val);

/* aux adc readback */
int wm97xx_read_aux_adc(struct wm97xx *wm, u16 adcsel);

/* machine ops */
int wm97xx_register_mach_ops(struct wm97xx *, struct wm97xx_mach_ops *);
void wm97xx_unregister_mach_ops(struct wm97xx *);

//--------------------------------------------------------------------------
// WM9715 Register Address Definition

#define	WM_SW_RESET		0x00	// WM Reset Register.
#define	WM_OUT2_VOL		0x02	// WM L/ROUT2 Volume Register
#define WM_HEADPHONE_VOL	0x04	// WM Headphone Volume Register.
#define WM_MONO_VOL        	0x06	// WM MONOOUT Voulme Register.
#define WM_TONE_CTRL       	0x08	// WM DAC Tone Control Register.
#define WM_BEEP_INPUT		0x0A	// WM PCBEEP Input Register.
#define WM_PHONE_VOL		0x0C	// WM PHONE Volume Register.
#define	WM_MIC_VOL		0x0E	// WM MIC Volume Register.
#define WM_LINEIN_VOL		0x10	// WM LINEIN Volume Register.
#define WM_AUXDAC_VOL		0x12	// WM AUXDAC Volume/Routing Register.
#define WM_SKLETONE_VOL		0x14	// WM SKLETON Volume Register.
#define WM_OUT3_VOL		0x16	// WM OUT3 Volume Register.
#define	WM_DAC_VOL		0x18	// WM DAC Volume Register.
#define	WM_REC_SRC		0x1A	// WM Record Source Select Register.
#define	WM_REC_GAIN		0x1C	// WM Record Gain Register.
#define	WM_GP			0x20	// WM General Purpose Register.
#define WM_DAC3D_CTRL		0x22	// WM DAC 3D Control Register.
#define WM_POWERDOWN		0x24	// WM Powerdown Register.
#define	WM_POWERDOWN_CTRL	0x26	// WM Powerdown Control & Status Register.
#define	WM_EXT_AUDIO_ID		0x28	// WM Extended Audio ID Register.
#define	WM_EXT_AUDIO_CTRL	0x2A	// WM Extended Audio Status and Control Register.
#define	WM_DAC_SAMPLE_RATE	0x2C	// WM Audio DACs Sample Rate Register.
#define WM_AUXDAC_SAMPLE_RATE	0x2E	// WM AUXDAC Sample Rate Register.
#define	WM_ADC_SAMPLE_RATE	0x32	// WM Audio ADC Sample Rate Register.
#define WM_SPDIF_CTRL		0x3A	// WM SPDIF Control Register.
#define WM_PIN_CFG_1		0x4C	// WM Pin Configuration (1) Register(INTR Pin Enable).
#define WM_INTR_POLARITY	0x4E	// WM Interrupt Polarity Register(GENIRQ).
#define WM_INTR_STICKY		0x50	// WM Interrupt Sticky Register(GENIRQ).
#define WM_INTR_WAKEUP		0x52	// WM Interrupt Wake-up Register(GENIRQ).
#define WM_INTR_FLAG		0x54	// WM Interrupt Flags Register(GENIRQ).
#define WM_PIN_CFG_2		0x56	// WM Pin Configuration (2) Register(INTR Pin Enable).
#define WM_EXTRA_FUNC_1		0x58	// WM Additional Functions (1) Register.
#define WM_VENDOR_RSV_1		0x5A	// WM Vendor Reserved Register(vendor test).
#define WM_EXTRA_FUNC_2		0x5C	// WM Additional Functions (2) Register.
#define WM_VENDOR_RSV_2		0x5E	// WM Vendor Reserved Register(vendor test).
#define WM_ALC_CTRL		0x60	// WM ALC Control Register.
#define WM_ALC_GATE_CTRL	0x62	// WM ALC/Noise Gate Control Register.
#define WM_AUXDAC_INPUT_CTRL	0x64	// WM AUXDAC Input Control Register.
#define WM_VENDOR_RSV_3		0x66	// WM Vendor Reserved Register(don't touch).
#define WM_VENDOR_RSV_4		0x68	// WM Vendor Reserved Register(don't touch).
#define WM_VENDOR_RSV_5		0x6A	// WM Vendor Reserved Register(don't touch).
#define WM_VENDOR_RSV_6		0x6C	// WM Vendor Reserved Register(don't touch).
#define WM_VENDOR_RSV_7		0x6E	// WM Vendor Reserved Register(don't touch).
#define WM_VENDOR_RSV_8		0x70	// WM Vendor Reserved Register(don't touch).
#define WM_VENDOR_RSV_9		0x72	// WM Vendor Reserved Register(don't touch).
#define WM_VENDOR_RSV_10	0x74	// WM Vendor Reserved Register(don't touch).
#define	WM_DIGITISER_1		0x76	// WM Digitiser Register (1).
#define	WM_DIGITISER_2		0x78	// WM Digitiser Register (2).
#define	WM_DIGITISER_READ_BACK	0x7A	// WM Digitiser Read Back Register.
#define	WM_VENDOR_ID1		0x7C	// WM Vendor ID1 Regiser.
#define	WM_VENDOR_ID2		0x7E	// WM Vendor ID2 Regiser.


//--------------------------------------------------------------------------
// WM9715 Register Field Common Definition

// WM Version
#define WM_PD_REV_40		0x6174		// WM_SW_RESET
#define WM_PD_VENDOR_ID1	0x574D		// WM_VENDOR_ID1
#define WM_PD_VENDOR_ID2	0x4C12		// WM_VENDOR_ID2

// WM Feature Information
#define	WM_20BIT_ADC		( 1 << 9 )	// WM_SW_RESET bit 9
#define	WM_20BIT_DAC		( 1 << 7 )	// WM_SW_RESET bit 7
#define	WM_BASS_BOOST		( 1 << 5 )	// WM_SW_RESET bit 5

// WM Extended Feature Information
#define	WM_VRA			( 1 << 0 )	// WM_EXT_AUDIO_ID bit 0

// WM Audio Mute Definition
#define	WM_OUT2_MUTE		( 1 << 15 )	// WM_OUT2_VOL bit 15

// WM Record Source Select Information
#define	WM_RS_MIC		0x0000		// WM_REC_SRC
#define	WM_RS_LINE_IN		0x0404		// WM_REC_SRC

// WM Powerdown Control and Status
#define	WM_INTN_CLK_OFF		( 1 << 13 )	// WM_POWERDOWN_CTRL bit 13
#define	WM_EXTN_CLK_OFF		( 1 << 12 )	// WM_POWERDOWN_CTRL bit 12
#define	WM_VREF_OFF		( 1 << 11 )	// WM_POWERDOWN_CTRL bit 11
#define	WM_DAC_OFF		( 1 << 9 )	// WM_POWERDOWN_CTRL bit 9
#define	WM_ADC_OFF		( 1 << 8 )	// WM_POWERDOWN_CTRL bit 8
#define	WM_VREF_RDY		( 1 << 3 )	// WM_POWERDOWN_CTRL bit 3
#define	WM_DAC_RDY		( 1 << 1 )	// WM_POWERDOWN_CTRL bit 1
#define	WM_ADC_RDY		( 1 << 0 )	// WM_POWERDOWN_CTRL bit 0
#define	WM_READY		( WM_VREF_RDY | WM_DAC_RDY | WM_ADC_RDY )

// WM Extended Audio Control and Status
#define	WM_VRA_DIS		( 0 << 0 )	// WM_EXT_AUDIO_CTRL bit 0
#define	WM_VRA_EN		( 1 << 0 )	// WM_EXT_AUDIO_CTRL bit 0

#define	WM_SEN_DIS		( 0 << 2 )	// WM_EXT_AUDIO_CTRL bit 2
#define	WM_SEN_EN		( 1 << 2 )	// WM_EXT_AUDIO_CTRL bit 2


// WM Audio Sample Rate
#define	WM_SAMPLE_RATE_8K	8000
#define	WM_SAMPLE_RATE_11K	11025
#define	WM_SAMPLE_RATE_12K	12000
#define	WM_SAMPLE_RATE_16K	16000
#define	WM_SAMPLE_RATE_22K	22050
#define	WM_SAMPLE_RATE_24K	24000
#define	WM_SAMPLE_RATE_32K	32000
#define	WM_SAMPLE_RATE_44K	44100
#define	WM_SAMPLE_RATE_48K	48000

// WM INTR Pin Configuration
#define WM_GC2_GENIRQ_EN	(0 << 2)	// WM_PIN_CFG_1 bit 2
#define WM_GC2_GENIRQ_DIS	(1 << 2)	// WM_PIN_CFG_1 bit 2

#define WM_GC3_PENDOWN_EN	(0 << 3)	// WM_PIN_CFG_1 bit 3
#define WM_GC3_PENDOWN_DIS	(1 << 3)	// WM_PIN_CFG_1 bit 3

#define WM_GC4_ADCIRQ_EN	(0 << 4)	// WM_PIN_CFG_1 bit 4
#define WM_GC4_ADCIRQ_DIS	(1 << 4)	// WM_PIN_CFG_1 bit 4

#define WM_GC5_SPDIFOUT_EN	(0 << 5)	// WM_PIN_CFG_1 bit 5
#define WM_GC5_SPDIFOUT_DIS	(1 << 5)	// WM_PIN_CFG_1 bit 5

#define WM_GE2_GENIRQ_EN	(0 << 2)	// WM_PIN_CFG_2 bit 2
#define WM_GE2_GENIRQ_DIS	(1 << 2)	// WM_PIN_CFG_2 bit 2

#define WM_GE3_PENDOWN_EN	(0 << 3)	// WM_PIN_CFG_2 bit 3
#define WM_GE3_PENDOWN_DIS	(1 << 3)	// WM_PIN_CFG_2 bit 3

#define WM_GE4_ADCIRQ_EN	(0 << 4)	// WM_PIN_CFG_2 bit 4
#define WM_GE4_ADCIRQ_DIS	(1 << 4)	// WM_PIN_CFG_2 bit 4

#define WM_GE5_SPDIFOUT_EN	(0 << 5)	// WM_PIN_CFG_2 bit 5
#define WM_GE5_SPDIFOUT_DIS	(1 << 5)	// WM_PIN_CFG_2 bit 5


// WM GENIRQ Interrupt Control: WM_INTR_POLARITY, WM_INTR_STICKY, WM_INTR_WAKEUP, WM_INTR_FLAG
#define	WN_INTR_COMP_1		( 1 << 15 )	// bit 15
#define	WM_INTR_COMP_2		( 1 << 14 )	// bit 14
#define	WM_INTR_PENDOWN		( 1 << 13 )	// bit 13
#define	WM_INTR_AUXADC_DATA_READY ( 1 << 12 )	// bit 12
#define	WM_INTR_THERMAL_SENSOR	( 1 << 11 )	// bit 11

// WM Touch Screen Mode (Measurement Type)
#define WM_SLT5			(0 << 2)	// WM_DIGITISER_1 bit 0~2
#define WM_SLT6			(1 << 2)	// WM_DIGITISER_1 bit 0~2
#define WM_SLT7			(2 << 2)	// WM_DIGITISER_1 bit 0~2
#define WM_SLT8			(3 << 2)	// WM_DIGITISER_1 bit 0~2
#define WM_SLT9			(4 << 2)	// WM_DIGITISER_1 bit 0~2
#define WM_SLT10		(5 << 2)	// WM_DIGITISER_1 bit 0~2
#define WM_SLT11		(6 << 2)	// WM_DIGITISER_1 bit 0~2

#define WM_SLEN_REGISTER	(0 << 3)	// WM_DIGITISER_1 bit 3
#define WM_SLEN_SLOT		(1 << 3)	// WM_DIGITISER_1 bit 3

#define WM_ADC_DELAY_1		(0 << 7)	// WM_DIGITISER_1 bit 4~7
#define WM_ADC_DELAY_2		(1 << 7)	// WM_DIGITISER_1 bit 4~7
#define WM_ADC_DELAY_4		(2 << 7)	// WM_DIGITISER_1 bit 4~7
#define WM_ADC_DELAY_8		(3 << 7)	// WM_DIGITISER_1 bit 4~7
#define WM_ADC_DELAY_16		(4 << 7)	// WM_DIGITISER_1 bit 4~7
#define WM_ADC_DELAY_32		(5 << 7)	// WM_DIGITISER_1 bit 4~7
#define WM_ADC_DELAY_48		(6 << 7)	// WM_DIGITISER_1 bit 4~7
#define WM_ADC_DELAY_64		(7 << 7)	// WM_DIGITISER_1 bit 4~7
#define WM_ADC_DELAY_96		(8 << 7)	// WM_DIGITISER_1 bit 4~7
#define WM_ADC_DELAY_128	(9 << 7)	// WM_DIGITISER_1 bit 4~7
#define WM_ADC_DELAY_160	(10 << 7)	// WM_DIGITISER_1 bit 4~7
#define WM_ADC_DELAY_192	(11 << 7)	// WM_DIGITISER_1 bit 4~7
#define WM_ADC_DELAY_224	(12 << 7)	// WM_DIGITISER_1 bit 4~7
#define WM_ADC_DELAY_256	(13 << 7)	// WM_DIGITISER_1 bit 4~7
#define WM_ADC_DELAY_288	(14 << 7)	// WM_DIGITISER_1 bit 4~7
#define WM_ADC_DELAY_NO		(15 << 7)	// WM_DIGITISER_1 bit 4~7

#define WM_CR_00		(0 << 9)	// WM_DIGITISER_1 bit 8~9
#define WM_CR_01		(1 << 9)	// WM_DIGITISER_1 bit 8~9
#define WM_CR_10		(2 << 9)	// WM_DIGITISER_1 bit 8~9
#define WM_CR_11		(3 << 9)	// WM_DIGITISER_1 bit 8~9

#define WM_CTC_POLLING		(0 << 10)	// WM_DIGITISER_1 bit 10
#define WM_CTC_DMA		(1 << 10)	// WM_DIGITISER_1 bit 10

#define WM_COO_SINGLE		(0 << 11)	// WM_DIGITISER_1 bit 11
#define WM_COO_XY		(1 << 11)	// WM_DIGITISER_1 bit 11

#define WM_ADCSEL_NO_MEASURE	(0 << 14)	// WM_DIGITISER_1 bit 12~14
#define WM_ADCSEL_X_MEASURE	(1 << 14)	// WM_DIGITISER_1 bit 12~14
#define WM_ADCSEL_Y_MEASURE	(2 << 14)	// WM_DIGITISER_1 bit 12~14
#define WM_ADCSEL_PRES_MEASURE	(3 << 14)	// WM_DIGITISER_1 bit 12~14
#define WM_ADCSEL_COMP1_MEASURE	(4 << 14)	// WM_DIGITISER_1 bit 12~14
#define WM_ADCSEL_COMP2_MEASURE	(5 << 14)	// WM_DIGITISER_1 bit 12~14
#define WM_ADCSEL_BMON_MEASURE	(6 << 14)	// WM_DIGITISER_1 bit 12~14
#define WM_ADCSEL_WIPER_MEASURE	(7 << 14)	// WM_DIGITISER_1 bit 12~14

#define WM_POLL_INIT		(1 << 15)	// WM_DIGITISER_1 bit 15


// WM Touch Screen Control
#define WM_RPU_68k		(1 << 0)	// WM_DIGITISER_2 bit 0~5
#define WM_RPU_34k		(2 << 0)	// WM_DIGITISER_2 bit 0~5

#define WM_PIL_200uA		(0 << 8)	// WM_DIGITISER_2 bit 8
#define WM_PIL_400uA		(1 << 8)	// WM_DIGITISER_2 bit 8

#define WM_WAIT_OVERWRITE	(0 << 9)	// WM_DIGITISER_2 bit 9
#define WM_WAIT_HELD		(1 << 9)	// WM_DIGITISER_2 bit 9

#define WM_PDEN_ALWAYS		(0 << 11)	// WM_DIGITISER_2 bit 11
#define WM_PDEN_DWON		(1 << 11)	// WM_DIGITISER_2 bit 11

#define WM_45W_4		(0 << 12)	// WM_DIGITISER_2 bit 12
#define WM_45W_5		(1 << 12)	// WM_DIGITISER_2 bit 12

#define WM_RPR_ACLINK		(0 << 13)	// WM_DIGITISER_2 bit 13
#define WM_RPR_BOTH		(1 << 13)	// WM_DIGITISER_2 bit 13

#define WM_PRP_OFF		(0 << 15)	// WM_DIGITISER_2 bit 14~15
#define WM_PRP_WAKEUP		(1 << 15)	// WM_DIGITISER_2 bit 14~15
#define WM_PRP_NO_WAKEUP	(2 << 15)	// WM_DIGITISER_2 bit 14~15
#define WM_PRP_RUNNING		(3 << 15)	// WM_DIGITISER_2 bit 14~15


// WM Touch Screen Data Read Back Mask
#define WM_ADCD			0x0FFF		// WM_DIGITISER_READ_BACK bit 0~11
#define WM_ADCSRC		(3 << 14)	// WM_DIGITISER_READ_BACK bit 12~14
#define WM_PNDN			(1 << 15) 	// WM_DIGITISER_READ_BACK bit 15

#define WM_ADCSRC_NO_MEASURE	(0 << 14)	// WM_DIGITISER_READ_BACK bit 12~14 defines
#define WM_ADCSRC_X_MEASURE	(1 << 14)	// WM_DIGITISER_READ_BACK bit 12~14 defines
#define WM_ADCSRC_Y_MEASURE	(2 << 14)	// WM_DIGITISER_READ_BACK bit 12~14 defines
#define WM_ADCSRC_PRES_MEASURE	(3 << 14)	// WM_DIGITISER_READ_BACK bit 12~14 defines
#define WM_ADCSRC_COMP1_MEASURE	(4 << 14)	// WM_DIGITISER_READ_BACK bit 12~14 defines
#define WM_ADCSRC_COMP2_MEASURE	(5 << 14)	// WM_DIGITISER_READ_BACK bit 12~14 defines
#define WM_ADCSRC_BMON_MEASURE	(6 << 14)	// WM_DIGITISER_READ_BACK bit 12~14 defines
#define WM_ADCSRC_WIPER_MEASURE	(7 << 14) 	// WM_DIGITISER_READ_BACK bit 12~14 defines

#define WM_PNDN_UP		(0 << 15) 	// WM_DIGITISER_READ_BACK bit 15 defines
#define WM_PNDN_DOWN		(1 << 15) 	// WM_DIGITISER_READ_BACK bit 15 defines

#ifndef __MIPS_ASSEMBLER


#define TEST(reg, bit_mask)  (reg & bit_mask)

// The ID of the device which will use WM9715 in a UCHAR, so must be <=7
#define WM_DEV_TOUCH		1
#define WM_DEV_AUDIO		2
#define WM_DEV_BATTERY		3

typedef struct
{
	uint16_t  r02;	// OUT2 Volume
	uint16_t  r04;	// Headphone Volume
	uint16_t  r0E;	// MIC Volume
	uint16_t  r24;	// Powerdown
	uint16_t  r26;	// Powerdown Control/Status
	uint16_t  r76;	// Digitiser Reg 1
	uint16_t  r78;	// Digitiser Reg 2
	uint16_t  r7A;	// Digitiser Read Back
	uint16_t  rXX[8];	// Pads
} WM9715, *PWM9715;

#endif // __MIPS_ASSEMBLER

#endif
