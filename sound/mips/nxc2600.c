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
 * 2007/11/15:
 *	1. fix dead lock when stop audio.
 *
 * 2007/07/16: initial
 *
 *
 * Author: Hardy Weng <hardy.weng@icnexus.com.tw>
 * 
 */
/*
 * 2009/05/14:
 *      1. fix noise at last period
 *      2. enable playback volume
 *      3. enable capture function and recording volume   
 *      4. re-make alsa-lib to fix overrun and underrun issues
 * 
 *
 * Author: Marvin Tien <marvin.tien@icnexus.com.tw>
 *
 */

#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <sound/driver.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/dmaengine.h>
#include <linux/timer.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/ac97_codec.h>
#include <asm/delay.h>
#include <asm/cacheflush.h>
#include <asm/mach-nxc2600/nxc2600.h>
#include <linux/wm97xx.h>

struct timeval now;

struct audio_stream
{
	struct dma_client client;
        struct dma_chan *channel;
	dma_cookie_t dma_cookie;
	struct snd_pcm_substream *substream;
	spinlock_t dma_lock;

	uint32_t buffer_bytes;
	int buffer_pos;
	int chno;
	int capture;
	uint32_t buffer_total;
	int last_period;
	int periods;
};

struct snd_nxc2600
{
	struct snd_card *card;

	struct resource *ac97_res_port;
	spinlock_t ac97_lock;
	struct snd_ac97 *ac97;

	struct snd_pcm *pcm;
	struct audio_stream playback;
	struct audio_stream capture;

};

static int
nxc2600_dma_start(struct audio_stream *stream,
		  void *dma_area );

static void
nxc2600_snd_dma_callback(void *callback_param)
{
        struct audio_stream *stream = (struct audio_stream*)callback_param;
	struct snd_pcm_substream *substream = stream->substream;
        volatile dma_cookie_t *cookie = &stream->dma_cookie;
        volatile int *buffer_pos = &stream->buffer_pos;
        
        nxc2600_dma_cleanup_interrupt(stream->chno); //marvin+
        spin_lock_irq(&(stream->dma_lock));
       	*buffer_pos += stream->buffer_bytes;
	if(*buffer_pos>=stream->buffer_total)
		*buffer_pos = 0;

        if(*cookie>0)
        {
                *cookie = 0;

		stream->last_period++;
	        if(stream->last_period>=stream->periods)
        	        stream->last_period = 0;

        	if(substream->runtime->status->state==SNDRV_PCM_STATE_DRAINING && stream->substream->runtime->stop_threshold > 0) //marvin+
        	{ 
          	   nxc2600_dma_start(stream,substream->runtime->dma_area+stream->last_period*stream->buffer_bytes);
                   stream->substream->runtime->stop_threshold=0;
                   stream->channel->device->device_issue_pending(stream->channel);
         	}   
        	else
        	{ 
        	   nxc2600_dma_start(stream,substream->runtime->dma_area+stream->last_period*stream->buffer_bytes);
        	   stream->channel->device->device_issue_pending(stream->channel);
        	}   
        }
	spin_unlock_irq(&(stream->dma_lock));
	if(*cookie>0)
		snd_pcm_period_elapsed(substream);
}

static int
nxc2600_dma_stop(struct audio_stream *stream)
{
	uint32_t pos;
	int timeout = 100;
	volatile int *buffer_pos = &stream->buffer_pos;
	volatile  dma_cookie_t *cookie = &stream->dma_cookie;

	if(*cookie==0)
		return 0;

	*cookie = 0;
	pos = (stream->last_period+1)*stream->buffer_bytes;
	if(pos>=stream->buffer_total)
		pos=0;

	spin_unlock_irq(&(stream->dma_lock));

	while( pos!=*buffer_pos && timeout--) udelay(10);

	spin_lock_irq(&(stream->dma_lock));
	
	return 0;
}

static int
nxc2600_dma_start(struct audio_stream *stream,
		  void *dma_area )
{
	uint32_t src,dest;
        struct dma_async_tx_descriptor *tx;
        
	if(stream->capture)
	{
		src = CPHYSADDR(NXC2600_AIC_DR);
		dest = virt_to_phys((void*)dma_area);
	}
	else
	{
		src = virt_to_phys((void*)dma_area);
		dest = CPHYSADDR(NXC2600_AIC_DR);
	}

	tx = stream->channel->device->device_prep_dma_memcpy(stream->channel, stream->buffer_bytes, 0);

        if (!tx)
                return -ENOMEM;

        tx->ack = 1;
        tx->callback = nxc2600_snd_dma_callback;
        tx->callback_param = stream;
       	tx->tx_set_src(src, tx, 0);
	tx->tx_set_dest(dest, tx, 0);

	stream->dma_cookie = tx->tx_submit(tx);

	return ((stream->dma_cookie>0)? 0:-1);
}

static unsigned int rates[] = {8000, 11025, 16000, 22050, 32000, 44100, 48000 };
static struct snd_pcm_hw_constraint_list hw_constraints_rates = 
{
	.count	= ARRAY_SIZE(rates),
	.list	= rates,
	.mask	= 0,
};

static struct snd_pcm_hardware snd_nxc2600_hw =
{
	.info			= (SNDRV_PCM_INFO_INTERLEAVED | \
				   SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID),
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
        .rates                  = 
                                   (SNDRV_PCM_RATE_48000),
        .rate_min               = 48000,
	.rate_max		= 48000,
	.channels_min		= 1,
	.channels_max		= 2,
        .buffer_bytes_max       = 128*1024,
        .period_bytes_min       = 32,
        .period_bytes_max       = 32*1024,
        .periods_min            = 2,
        .periods_max            = 4,
        .fifo_size              = 16,
};

static int
snd_nxc2600_playback_open(struct snd_pcm_substream *substream)
{
	struct snd_nxc2600 *nxc2600 = substream->pcm->private_data;

	if(nxc2600->playback.channel == NULL)
		return -EBUSY;

	nxc2600->playback.substream = substream;
	substream->private_data = &nxc2600->playback;
	substream->runtime->hw = snd_nxc2600_hw;
                 
	return (snd_pcm_hw_constraint_list(substream->runtime, 0,
		SNDRV_PCM_HW_PARAM_RATE, &hw_constraints_rates) < 0);
}

static int
snd_nxc2600_capture_open(struct snd_pcm_substream *substream)
{
	struct snd_nxc2600 *nxc2600 = substream->pcm->private_data;

	if(nxc2600->capture.channel == NULL)
                return -EBUSY;

	nxc2600->capture.substream = substream;
	substream->private_data = &nxc2600->capture;
	substream->runtime->hw = snd_nxc2600_hw;

	return (snd_pcm_hw_constraint_list(substream->runtime, 0,
		SNDRV_PCM_HW_PARAM_RATE, &hw_constraints_rates) < 0);
}


static int
snd_nxc2600_hw_free(struct snd_pcm_substream *substream);

static int
snd_nxc2600_playback_close(struct snd_pcm_substream *substream)
{
	struct audio_stream *stream = substream->private_data;
	
	spin_lock_irq(&stream->dma_lock);
	nxc2600_dma_stop(stream);
	spin_unlock_irq(&stream->dma_lock);
	stream->substream = NULL;
	memset(substream->runtime->dma_area, 0, stream->buffer_total);
	REG32(NXC2600_AIC_CR) |= NXC2600_AIC_CR_FLUSH;
	nxc2600_aic_ac97_stop_playback();

	return 0;
}

static int
snd_nxc2600_capture_close(struct snd_pcm_substream *substream)
{
	struct audio_stream *stream = substream->private_data;

	spin_lock_irq(&stream->dma_lock);
	nxc2600_dma_stop(stream);
	spin_unlock_irq(&stream->dma_lock);
	stream->substream = NULL;
	memset(substream->runtime->dma_area, 0, stream->buffer_total);
	REG32(NXC2600_AIC_CR) |= NXC2600_AIC_CR_FLUSH;
	nxc2600_aic_ac97_stop_capture();

	return 0;
}

static int
snd_nxc2600_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *hw_params)
{
	struct audio_stream *stream = substream->private_data;
	int err;

	err = snd_pcm_lib_malloc_pages(substream,
				       params_buffer_bytes(hw_params));
	if (err < 0)
		return err;

	stream->buffer_total = params_period_bytes(hw_params)*params_periods(hw_params);
	stream->buffer_bytes = params_period_bytes(hw_params);
	stream->periods = params_periods(hw_params);

	return 0;
}

static int
snd_nxc2600_hw_free(struct snd_pcm_substream *substream)
{

	return snd_pcm_lib_free_pages(substream);
}

static __inline__ void
nxc2600_aic_ac97_enable_playback(void) //marvin+
{
	nxc2600_aic_ac97_write(WM_REC_SRC, 0x3000);
        nxc2600_aic_ac97_write(WM_REC_GAIN, 0x8000);
	nxc2600_aic_ac97_write(WM_MIC_VOL, 0x6808);
	nxc2600_aic_ac97_write(WM_TONE_CTRL, 0x0000);	
	nxc2600_aic_ac97_write(WM_OUT2_VOL, 0x0000);	
	nxc2600_aic_ac97_write(WM_OUT3_VOL, 0x0000);	
	nxc2600_aic_ac97_write(WM_HEADPHONE_VOL, 0x0000);	
	nxc2600_aic_ac97_write(WM_DAC_VOL, 0x2808);
}	
		
static int
snd_nxc2600_playback_prepare(struct snd_pcm_substream *substream)
{
	struct snd_nxc2600 *nxc2600 = substream->pcm->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct audio_stream *stream = substream->private_data;
	
        REG32(NXC2600_AIC_CR) |= NXC2600_AIC_CR_FLUSH;
        memset(substream->runtime->dma_area, 0, stream->buffer_total);
	if( nxc2600_aic_ac97_set_playback_sample_bits(16) )
		return -EINVAL;
	if( runtime->channels == 1)
	      nxc2600_aic_ac97_set_playback_mono();
	else
	      nxc2600_aic_ac97_set_playback_stereo();

	snd_ac97_set_rate(nxc2600->ac97, AC97_PCM_FRONT_DAC_RATE, runtime->rate);
	
        nxc2600_aic_ac97_enable_playback();
        
        nxc2600_aic_transmit_trigger(7);
        
        nxc2600_aic_ac97_stop_capture();
        
	nxc2600_aic_ac97_start_playback();
        
	return 0;
}

static __inline__ void
nxc2600_aic_ac97_enable_mic(void) //marvin+
{
	uint16_t usa=0;
	
	usa = (0x4800); 
	nxc2600_aic_ac97_write(WM_REC_SRC, usa);
	usa =  (0x0);
	nxc2600_aic_ac97_write(WM_REC_GAIN, usa);
	nxc2600_aic_ac97_write(WM_MIC_VOL, 0x08C0);	
	usa = nxc2600_aic_ac97_read(WM_DAC_VOL);
	usa &=  ~(0x1<<15);
	nxc2600_aic_ac97_write(WM_DAC_VOL, usa);
}

static int
snd_nxc2600_capture_prepare(struct snd_pcm_substream *substream)
{
	struct snd_nxc2600 *nxc2600 = substream->pcm->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct audio_stream *stream = substream->private_data;

        REG32(NXC2600_AIC_CR) |= NXC2600_AIC_CR_FLUSH;
        memset(substream->runtime->dma_area, 0, stream->buffer_total);
        if( nxc2600_aic_ac97_set_capture_sample_bits(16) )
            return -EINVAL;
        if( runtime->channels == 1)
              nxc2600_aic_ac97_set_capture_mono();
        else
              nxc2600_aic_ac97_set_capture_stereo();

	snd_ac97_set_rate(nxc2600->ac97, AC97_PCM_LR_ADC_RATE, runtime->rate);
		
        nxc2600_aic_ac97_enable_mic();
        
        nxc2600_aic_receive_trigger(15);

        nxc2600_aic_ac97_stop_playback();
        
	nxc2600_aic_ac97_start_capture();

	return 0;
}


static int
snd_nxc2600_trigger(struct snd_pcm_substream *substream, 
				int cmd)
{
	struct audio_stream *stream = substream->private_data;
	int err = 0;

	spin_lock_irq(&stream->dma_lock);
	switch (cmd)
	{
		case SNDRV_PCM_TRIGGER_START:
			stream->buffer_pos = 0;
			for(stream->last_period=0;
				stream->last_period<2;
				stream->last_period++)
			{
			err = nxc2600_dma_start(stream,substream->runtime->dma_area+stream->last_period*stream->buffer_bytes);
			if(err)
			{
				nxc2600_dma_stop(stream);
				break;
			}
			stream->channel->device->device_issue_pending(stream->channel);
			}
			stream->last_period--;
			break;
		case SNDRV_PCM_TRIGGER_STOP:
			err = nxc2600_dma_stop(stream);
			break;
		default:
			err = -EINVAL;
			break;
	}
	spin_unlock_irq(&stream->dma_lock);
	return err;
}

static int
snd_nxc2600_playback_trigger(struct snd_pcm_substream *substream,
                                int cmd)
{
	return snd_nxc2600_trigger(substream,cmd);
}

static int
snd_nxc2600_capture_trigger(struct snd_pcm_substream *substream,
                                int cmd)
{
	return snd_nxc2600_trigger(substream,cmd);
}

static snd_pcm_uframes_t
snd_nxc2600_pointer(struct snd_pcm_substream *substream)
{
	uint32_t count;
	struct audio_stream *stream = substream->private_data;
	spin_lock(&(stream->dma_lock));        
	count=bytes_to_frames(substream->runtime,
				stream->buffer_pos+
				stream->buffer_bytes-
				nxc2600_dma_get_current_transfer_count(stream->chno,0));        
	spin_unlock(&(stream->dma_lock));			
	if (count >= stream->substream->runtime->buffer_size)
	    count = 0;
	
	return count;     
}

static snd_pcm_uframes_t
snd_nxc2600_playback_pointer(struct snd_pcm_substream *substream)
{
	return snd_nxc2600_pointer(substream);
}

static snd_pcm_uframes_t
snd_nxc2600_capture_pointer(struct snd_pcm_substream *substream)
{
        return snd_nxc2600_pointer(substream);
}

static struct snd_pcm_ops snd_card_nxc2600_playback_ops = 
{
	.open			= snd_nxc2600_playback_open,
	.close			= snd_nxc2600_playback_close,
	.ioctl			= snd_pcm_lib_ioctl,
	.hw_params	        = snd_nxc2600_hw_params,
	.hw_free	        = snd_nxc2600_hw_free,
	.prepare		= snd_nxc2600_playback_prepare,
	.trigger		= snd_nxc2600_playback_trigger,
	.pointer		= snd_nxc2600_playback_pointer,
};

static struct snd_pcm_ops snd_card_nxc2600_capture_ops = 
{
	.open			= snd_nxc2600_capture_open,
	.close			= snd_nxc2600_capture_close,
	.ioctl			= snd_pcm_lib_ioctl,
	.hw_params	        = snd_nxc2600_hw_params,
	.hw_free	        = snd_nxc2600_hw_free,
	.prepare		= snd_nxc2600_capture_prepare,
	.trigger		= snd_nxc2600_capture_trigger,
	.pointer		= snd_nxc2600_capture_pointer,
};

extern int nxc2600_dma_get_chno(struct dma_chan *chan);
static enum dma_state_client
stream_dma_event( struct dma_client *client,
		    struct dma_chan *chan,
		    enum dma_state state)
{
	struct audio_stream *stream = (struct audio_stream *)client;
        enum dma_state_client ack = DMA_NAK;

        spin_lock_irq(&stream->dma_lock);
        switch (state) 
	{
        	case DMA_RESOURCE_AVAILABLE:
			if( stream->channel == NULL )
			{
                        	ack = DMA_ACK;
				stream->channel = chan;
				stream->chno = nxc2600_dma_get_chno(chan);
			}
                	break;
		case DMA_RESOURCE_REMOVED:
                        if( stream->channel == chan ) 
			{
                        	ack = DMA_ACK;
                        	stream->channel = NULL;
				stream->chno = -1;
			}
			break;
	        default:
        	        break;
        }
        spin_unlock_irq(&stream->dma_lock);

        return ack;
}

static int __devinit
snd_nxc2600_pcm_new(struct snd_nxc2600 *nxc2600)
{
	struct snd_pcm *pcm;
	int err;

	if ((err = snd_pcm_new(nxc2600->card, "NXC2600 AC97 PCM", 0, 1, 1, &pcm)) < 0)
		return err;

	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_CONTINUOUS,
		snd_dma_continuous_data(GFP_KERNEL), 128*1024, 128*1024);

	pcm->private_data = nxc2600;
	pcm->info_flags = 0;
	strcpy(pcm->name, "NXC2600 AC97 PCM");

	spin_lock_init(&nxc2600->playback.dma_lock);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
                &snd_card_nxc2600_playback_ops);
	nxc2600->playback.channel = NULL;
	nxc2600->playback.client.event_callback = stream_dma_event;
	nxc2600->playback.capture = 0;
        dma_cap_set(DMA_MEMCPY, nxc2600->playback.client.cap_mask);
        dma_async_client_register(&nxc2600->playback.client);
        dma_async_client_chan_request(&nxc2600->playback.client);


        spin_lock_init(&nxc2600->capture.dma_lock);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
                &snd_card_nxc2600_capture_ops);
	nxc2600->capture.client.event_callback = stream_dma_event;
	nxc2600->capture.channel = NULL;
	nxc2600->capture.capture = 1;
        dma_cap_set(DMA_MEMCPY, nxc2600->capture.client.cap_mask);
        dma_async_client_register(&nxc2600->capture.client);
        dma_async_client_chan_request(&nxc2600->capture.client);

	nxc2600->pcm = pcm;
	return 0;
}


static unsigned short
snd_nxc2600_ac97_read(struct snd_ac97 *ac97, unsigned short reg)
{
	struct snd_nxc2600 *nxc2600 = ac97->private_data;
	int data;

	spin_lock_irq(&nxc2600->ac97_lock);

	data = nxc2600_aic_ac97_read(reg);

	spin_unlock_irq(&nxc2600->ac97_lock);

	return data;

}


static void
snd_nxc2600_ac97_write(struct snd_ac97 *ac97, unsigned short reg, unsigned short val)
{
	struct snd_nxc2600 *nxc2600 = ac97->private_data;

	spin_lock_irq(&nxc2600->ac97_lock);

	if(nxc2600_aic_ac97_write( reg, val ))
		printk("AC97 write error\n");

	spin_unlock_irq(&nxc2600->ac97_lock);
}

static int __devinit
snd_nxc2600_ac97_new(struct snd_nxc2600 *nxc2600)
{
	int err;
	struct snd_ac97_bus *pbus;
	struct snd_ac97_template ac97;
 	static struct snd_ac97_bus_ops ops = 
	{
		.write = snd_nxc2600_ac97_write,
		.read = snd_nxc2600_ac97_read,
	};

	if ((nxc2600->ac97_res_port = request_mem_region(CPHYSADDR(NXC2600_AIC),
	       		0x40, "NXC2600 AC97")) == NULL) 
	{
		snd_printk(KERN_ERR "ALSA AC97: can't grap AC97 port\n");
		return -EBUSY;
	}

	spin_lock_init(&nxc2600->ac97_lock);
	if ((err = snd_ac97_bus(nxc2600->card, 0, &ops, nxc2600, &pbus)) < 0)
 		return err;

	memset(&ac97, 0, sizeof(ac97));
	ac97.private_data = nxc2600;
	if ((err = snd_ac97_mixer(pbus, &ac97, &nxc2600->ac97)) < 0)
		return err;

	return 0;
}

static void
snd_nxc2600_free(struct snd_card *card)
{
	struct snd_nxc2600 *nxc2600 = card->private_data;

	if (nxc2600->ac97_res_port)
		release_and_free_resource(nxc2600->ac97_res_port);


}

static struct snd_card *nxc2600_card;

static int __init
nxc2600_init(void)
{
	int err;
	struct snd_card *card;
	struct snd_nxc2600 *nxc2600;

	card = snd_card_new(-1, "AC97", THIS_MODULE, sizeof(struct snd_nxc2600));
	if (card == NULL)
		return -ENOMEM;

	card->private_free = snd_nxc2600_free;
	nxc2600 = card->private_data;
	nxc2600->card = card;

 	nxc2600->ac97_res_port = NULL;

	nxc2600_aic_init(1);
	nxc2600_aic_ac97_cold_reset();

	if ((err = snd_nxc2600_ac97_new(nxc2600)) < 0 )
	{
		snd_card_free(card);
		return err;
	}

	if ((err = snd_nxc2600_pcm_new(nxc2600)) < 0) {
		snd_card_free(card);
		return err;
	}

	strcpy(card->driver, "NXC2600-AC97");
	strcpy(card->shortname, "NXC2600-AC97");
	sprintf(card->longname, "ICNEXUS NXC2600-AC97 ALSA Driver");

	if ((err = snd_card_register(card)) < 0) 
	{
		snd_card_free(card);
		return err;
	}

	printk( KERN_INFO "ALSA AC97: Driver Initialized\n" );
	nxc2600_card = card;
	return 0;
}

static void __exit 
nxc2600_exit(void)
{
	snd_card_free(nxc2600_card);
	nxc2600_aic_stop();
}

module_init(nxc2600_init);
module_exit(nxc2600_exit);


MODULE_AUTHOR("Hardy Weng <hardy.weng@icnexus.com.tw>");
MODULE_DESCRIPTION("NXC2600 AC'97 ALSA Driver");
MODULE_LICENSE("GPL");

