/*
 * NAND uclass driver for NAND bus.
 *
 * (C) Copyright 2016
 *     Texas Instruments Incorporated, <www.ti.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <errno.h>
#include <nand.h>

DECLARE_GLOBAL_DATA_PTR;

#ifdef CONFIG_DM_NAND

nand_info_t *get_nand_dev_by_index(int idx)
{
	nand_info_t *nand;
	struct udevice *dev;
	int ret;

	ret = uclass_get_device(UCLASS_NAND, idx, &dev);
	if (ret) {
		debug("NAND device (%d) not found\n", idx);
		return NULL;
	}

	nand = (nand_info_t *)dev_get_uclass_priv(dev);
	if (!nand) {
		error("Nand device not ready\n");
		return NULL;
	}

	return nand;
}

static int nand_child_pre_probe(struct udevice *dev)
{
	nand_info_t *nand = dev_get_uclass_priv(dev);
	void *priv = dev_get_priv(dev);

	/*
	 * Store nand device priv pointer in nand_info so that
	 * it can be used by nand command
	 */
	nand->priv = priv;

	return 0;
}

UCLASS_DRIVER(nand) = {
	.id				= UCLASS_NAND,
	.name				= "nand",
	.flags				= DM_UC_FLAG_SEQ_ALIAS,
	.post_probe		= nand_child_pre_probe,
	.per_device_auto_alloc_size	= sizeof(nand_info_t),
};

#endif
