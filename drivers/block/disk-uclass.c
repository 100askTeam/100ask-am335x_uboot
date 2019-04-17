/*
 * Copyright (c) 2015 Google, Inc
 * Written by Simon Glass <sjg@chromium.org>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <dm/uclass-internal.h>
#include <dm/device-internal.h>
#include <scsi.h>

int scsi_get_device(int index, struct udevice **devp)
{
	struct udevice *dev;
	int ret;

	ret = uclass_find_device(UCLASS_DISK, index, &dev);
	if (ret || !dev) {
		printf("%d device not found\n", index);
		return ret;
	}

	ret = device_probe(dev);
	if (ret) {
		error("device probe error\n");
		return ret;
	}

	*devp = dev;

	return ret;
}

void scsi_init(void)
{
	struct udevice *dev;
	int ret;

	ret = scsi_get_device(0, &dev);
	if (ret || !dev) {
		error("scsi device not found\n");
		return;
	}

	scsi_scan(1);
}

UCLASS_DRIVER(disk) = {
	.id		= UCLASS_DISK,
	.name		= "disk",
};
