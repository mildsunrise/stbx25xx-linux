/*
 * stbx25xx.c - driver for digital TV devices equipped with IBM STBx25xx SoC
 *
 * Copyright (C) 2009 Tomasz Figa <tomasz.figa@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/gpio.h>
#include <linux/of_platform.h>
#include "stbx25xx.h"


#ifdef CONFIG_DVB_STBx25xx_DEBUG
#define DEBSTATUS ""
#else
#define DEBSTATUS " (debugging is not enabled)"
#endif

int dvb_stbx25xx_debug;
module_param_named(debug, dvb_stbx25xx_debug,  int, 0644);
MODULE_PARM_DESC(debug, "set debug level (1=info,2=tuner,4=i2c,8=ts,16=sram,32=reg (|-able))." DEBSTATUS);
#undef DEBSTATUS

struct stbx25xx_dvb_dev stbx25xx_dvb_dev = {
	.owner		= THIS_MODULE,
};

extern int (*dvbdmx_disconnect_frontend)(struct dmx_demux *demux);
extern int (*dvbdmx_connect_frontend)(struct dmx_demux *demux,
				   struct dmx_frontend *frontend);

static int stbx25xx_dvb_init(struct stbx25xx_dvb_dev *dev)
{
	short num[1] = {-1};
	int ret = dvb_register_adapter(&dev->dvb_adapter,
				       "STBx25xx Digital TV device", dev->owner,
				       dev->dev, num);
	if (ret < 0) {
		err("error registering DVB adapter");
		return ret;
	}
	
	dev->dvb_adapter.priv = dev;
	
	dev->demux.dmx.capabilities = (DMX_TS_FILTERING | DMX_PES_FILTERING |
					DMX_SECTION_FILTERING | DMX_MEMORY_BASED_FILTERING | DMX_TS_DESCRAMBLING);
	dev->demux.priv		= dev;
	dev->demux.filternum	= STBx25xx_MAX_FEED;
	dev->demux.feednum	= STBx25xx_MAX_FEED;

	dev->demux.start_feed	= stbx25xx_demux_start_feed;
	dev->demux.stop_feed	= stbx25xx_demux_stop_feed;
	dev->demux.write_to_decoder = stbx25xx_demux_write_to_decoder;

	if ((ret = dvb_dmx_init(&dev->demux)) < 0) {
		err("dvb_dmx failed: error %d",ret);
		goto err_dmx;
	}
	
	// set overrides for clipmode
	dvbdmx_connect_frontend 	= dev->demux.dmx.connect_frontend;
	dev->demux.dmx.connect_frontend = stbx25xx_demux_connect_frontend;
	dvbdmx_disconnect_frontend 	= dev->demux.dmx.disconnect_frontend;
	dev->demux.dmx.disconnect_frontend = stbx25xx_demux_disconnect_frontend;
	dev->demux.dmx.get_stc		= stbx25xx_demux_get_stc;

	dev->hw_frontend.source = DMX_FRONTEND_0;

	dev->dmxdev.filternum		= dev->demux.feednum;
	dev->dmxdev.demux		= &dev->demux.dmx;
	dev->dmxdev.capabilities	= 0;
	if ((ret = dvb_dmxdev_init(&dev->dmxdev, &dev->dvb_adapter)) < 0) {
		err("dvb_dmxdev_init failed: error %d",ret);
		goto err_dmx_dev;
	}

	if ((ret = dev->demux.dmx.add_frontend(&dev->demux.dmx, &dev->hw_frontend)) < 0) {
		err("adding hw_frontend to dmx failed: error %d",ret);
		goto err_dmx_add_hw_frontend;
	}

/*
	dev->mem_frontend.source = DMX_MEMORY_FE;
	if ((ret = dev->demux.dmx.add_frontend(&dev->demux.dmx, &dev->mem_frontend)) < 0) {
		err("adding mem_frontend to dmx failed: error %d",ret);
		goto err_dmx_add_mem_frontend;
	}
*/

	if ((ret = dev->demux.dmx.connect_frontend(&dev->demux.dmx, &dev->hw_frontend)) < 0) {
		err("connect frontend failed: error %d",ret);
		goto err_connect_frontend;
	}

/*
	if ((ret = dvb_add_frontend_notifier(adapter, stbx25xx_demux_before_after_tune, NULL)) < 0) {
		err("adding frontend notifier failed: error %d", ret);
		goto error_frontend_notifier;
	}
*/

	dvb_net_init(&dev->dvb_adapter, &dev->dvbnet, &dev->demux.dmx);

	dev->init_state |= FC_STATE_DVB_INIT;
	return 0;

/*
error_frontend_notifier:
	dev->demux.dmx.disconnect_frontend(&dev->demux.dmx);
*/
err_connect_frontend:
	dev->demux.dmx.remove_frontend(&dev->demux.dmx,&dev->mem_frontend);
err_dmx_add_mem_frontend:
	dev->demux.dmx.remove_frontend(&dev->demux.dmx,&dev->hw_frontend);
err_dmx_add_hw_frontend:
	dvb_dmxdev_release(&dev->dmxdev);
err_dmx_dev:
	dvb_dmx_release(&dev->demux);
err_dmx:
	dvb_unregister_adapter(&dev->dvb_adapter);
	return ret;
}

static void stbx25xx_dvb_exit(struct stbx25xx_dvb_dev *dev)
{
	if (dev->init_state & FC_STATE_DVB_INIT) {
		dvb_net_release(&dev->dvbnet);

		dev->demux.dmx.close(&dev->demux.dmx);
		dev->demux.dmx.remove_frontend(&dev->demux.dmx,&dev->mem_frontend);
		dev->demux.dmx.remove_frontend(&dev->demux.dmx,&dev->hw_frontend);
		dvb_dmxdev_release(&dev->dmxdev);
		dvb_dmx_release(&dev->demux);
		dvb_unregister_adapter(&dev->dvb_adapter);

		deb_info("deinitialized dvb stuff\n");
	}
	dev->init_state &= ~FC_STATE_DVB_INIT;
}

static int stbx25xx_dvb_map_irqs(struct of_device *ofdev, struct stbx25xx_dvb_dev *dvb)
{
	struct device_node *np = ofdev->node;
	int irq, i;

	for(i=0; i<4; i++) {
		irq = irq_of_parse_and_map(np, i);
		if (irq == NO_IRQ) {
			dev_err(&ofdev->dev, "irq_of_parse_and_map failed\n");
			return NO_IRQ;
		}
		dvb->irq_num[i] = irq;
	}

	return 0;
}


static int stbx25xx_adapter_probe(struct of_device *dev, const struct of_device_id *match)
{
	int ret;
	struct stbx25xx_dvb_dev *dvb_dev = &stbx25xx_dvb_dev;
	
	gpio_direction_output(253, 0);

	platform_set_drvdata(dev, dvb_dev);
	
	dvb_dev->dev = &dev->dev;
	
	if ((ret = stbx25xx_dvb_map_irqs(dev, dvb_dev)) != 0) {
		err("IRQ mapping failed: error %d", ret);
		goto video_error;
	}
	
	if ((ret = stbx25xx_video_init(dvb_dev))) {
		err("Video initiailzation failed: error %d", ret);
		goto video_error;
	}
	
	if ((ret = stbx25xx_audio_init(dvb_dev))) {
		err("Audio initialization failed: error %d", ret);
		goto audio_error;
	}
	
	if ((ret = stbx25xx_demux_init(dvb_dev))) {
		err("Demux initialization failed: error %d", ret);
		goto demux_error;
	}

	if ((ret = stbx25xx_dvb_init(dvb_dev))) {
		err("DVB initialization failed: error %d", ret);
		goto dvb_error;
	}

	if ((ret = stbx25xx_frontend_init(dvb_dev))) {
		err("Front-end initialization failed: error %d", ret);
		goto frontend_error;
	}

	info("Initialization of hardware complete");

	return 0;

frontend_error:
	stbx25xx_dvb_exit(dvb_dev);
dvb_error:	
	stbx25xx_demux_exit(dvb_dev);
demux_error:
	stbx25xx_audio_exit(dvb_dev);
audio_error:
	stbx25xx_video_exit(dvb_dev);	
video_error:
	return ret;
}

static int stbx25xx_adapter_remove(struct of_device *dev)
{
	struct stbx25xx_dvb_dev *dvb_dev = platform_get_drvdata(dev);
	
	stbx25xx_frontend_exit(dvb_dev);
	stbx25xx_dvb_exit(dvb_dev);
	stbx25xx_demux_exit(dvb_dev);
	stbx25xx_audio_exit(dvb_dev);
	stbx25xx_video_exit(dvb_dev);
	
	return 0;
}

static struct of_device_id stbx25xx_adapter_matches[] = {
	{ .compatible	= "ibm,stbx25xx-dvb", },
	{},
};

static struct of_platform_driver stbx25xx_dvb_driver = {
	.match_table	= stbx25xx_adapter_matches,
	.probe		= stbx25xx_adapter_probe,
	.remove		= stbx25xx_adapter_remove,
	.driver		= {
		.name		= "stbx25xx-dvb",
	},
};

static int stbx25xx_adapter_module_init(void)
{
	of_register_platform_driver(&stbx25xx_dvb_driver);
	info(DRIVER_NAME " loaded successfully");
	return 0;
}

static void stbx25xx_adapter_module_cleanup(void)
{
	of_unregister_platform_driver(&stbx25xx_dvb_driver);
	info(DRIVER_NAME " unloaded successfully");
}

module_init(stbx25xx_adapter_module_init);
module_exit(stbx25xx_adapter_module_cleanup);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_NAME);
MODULE_LICENSE("GPL");
