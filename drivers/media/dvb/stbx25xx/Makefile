dvb_stbx25xx-y = stbx25xx.o stbx25xx_demux.o stbx25xx_tuner.o
dvb_stbx25xx-$(CONFIG_DVB_STBx25xx_AV) += stbx25xx_video.o stbx25xx_audio.o
dvb_stbx25xx-$(CONFIG_DVB_STBx25xx_OSD) += stbx25xx_osd.o
obj-$(CONFIG_DVB_STBx25xx) += dvb_stbx25xx.o

EXTRA_CFLAGS += -Idrivers/media/dvb/dvb-core/ -Idrivers/media/dvb/frontends/
EXTRA_CFLAGS += -Idrivers/media/common/tuners/
