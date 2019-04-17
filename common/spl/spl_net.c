/*
 * (C) Copyright 2000-2004
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * (C) Copyright 2012
 * Ilya Yanok <ilya.yanok@gmail.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <errno.h>
#include <spl.h>
#include <net.h>
#include <dm.h>

DECLARE_GLOBAL_DATA_PTR;

#ifndef CONFIG_SPL_LOAD_FIT_ADDRESS
#define CONFIG_SPL_LOAD_FIT_ADDRESS 0
#endif

static ulong spl_net_load_read(struct spl_load_info *load, ulong sector,
			       ulong count, void *buf)
{
	debug("%s: sector %lx, count %lx, buf %lx\n",
	      __func__, sector, count, (ulong)buf);
	memcpy(buf, (void *)(CONFIG_SPL_LOAD_FIT_ADDRESS + sector), count);
	return count;
}

int spl_net_load_image(const char *device)
{
	struct image_header *header;
	int rv;

	env_init();
	env_relocate();
	setenv("autoload", "yes");
	if (IS_ENABLED(CONFIG_SPL_LOAD_FIT))
		load_addr = CONFIG_SPL_LOAD_FIT_ADDRESS;
	else
		load_addr = CONFIG_SYS_TEXT_BASE - sizeof(struct image_header);

	rv = eth_initialize();
	if (rv == 0) {
		printf("No Ethernet devices found\n");
		return -ENODEV;
	}
	if (device)
		setenv("ethact", device);
	rv = net_loop(BOOTP);
	if (rv < 0) {
		printf("Problem booting with BOOTP\n");
		return rv;
	}

	header = (struct image_header *)load_addr;

	if (IS_ENABLED(CONFIG_SPL_LOAD_FIT) &&
	    image_get_magic(header) == FDT_MAGIC) {
		struct spl_load_info load;

		debug("Found FIT\n");
		load.bl_len = 1;
		load.read = spl_net_load_read;
		spl_load_simple_fit(&load, 0, header);
	} else {
		spl_parse_image_header((struct image_header *)load_addr);
	}

	return 0;
}
