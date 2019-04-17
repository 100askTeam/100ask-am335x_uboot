/*
 * Copyright 2016 Texas Instruments, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#include <common.h>
#include <libfdt.h>
#include <fdt_support.h>
#include <malloc.h>

#include <asm/omap_common.h>
#include <asm/arch-omap5/sys_proto.h>

#ifdef CONFIG_TI_SECURE_DEVICE

/* Give zero values if not already defined */
#ifndef CONFIG_SECURE_BOOT_SRAM
#define CONFIG_SECURE_BOOT_SRAM (0)
#endif
#ifndef CONFIG_SECURE_RUN_SRAM
#define CONFIG_SECURE_RUN_SRAM (0)
#endif

static u32 hs_irq_skip[] = {
	8,	/* Secure violation reporting interrupt */
	15,	/* One interrupt for SDMA by secure world */
	118	/* One interrupt for Crypto DMA by secure world */
};

static int ft_hs_fixup_crossbar(void *fdt, bd_t *bd)
{
	const char *path;
	int offs;
	int ret;
	int len, i, old_cnt, new_cnt;
	u32 *temp;
	const u32 *p_data;

	/*
	 * Increase the size of the fdt
	 * so we have some breathing room
	 */
	ret = fdt_increase_size(fdt, 512);
	if (ret < 0) {
		printf("Could not increase size of device tree: %s\n",
		       fdt_strerror(ret));
		return ret;
	}

	/* Reserve IRQs that are used/needed by secure world */
	path = "/ocp/crossbar";
	offs = fdt_path_offset(fdt, path);
	if (offs < 0) {
		debug("Node %s not found.\n", path);
		return 0;
	}

	/* Get current entries */
	p_data = fdt_getprop(fdt, offs, "ti,irqs-skip", &len);
	if (p_data)
		old_cnt = len / sizeof(u32);
	else
		old_cnt = 0;

	new_cnt = sizeof(hs_irq_skip) /
				sizeof(hs_irq_skip[0]);

	/* Create new/updated skip list for HS parts */
	temp = malloc(sizeof(u32) * (old_cnt + new_cnt));
	for (i = 0; i < new_cnt; i++)
		temp[i] = cpu_to_fdt32(hs_irq_skip[i]);
	for (i = 0; i < old_cnt; i++)
		temp[i + new_cnt] = p_data[i];

	/* Blow away old data and set new data */
	fdt_delprop(fdt, offs, "ti,irqs-skip");
	ret = fdt_setprop(fdt, offs, "ti,irqs-skip",
			  temp,
			  (old_cnt + new_cnt) * sizeof(u32));
	free(temp);

	/* Check if the update worked */
	if (ret < 0) {
		printf("Could not add ti,irqs-skip property to node %s: %s\n",
		       path, fdt_strerror(ret));
		return ret;
	}

	return 0;
}

static int ft_hs_disable_rng(void *fdt, bd_t *bd)
{
	const char *path;
	int offs;
	int ret;

	/* Make HW RNG reserved for secure world use */
	path = "/ocp/rng";
	offs = fdt_path_offset(fdt, path);
	if (offs < 0) {
		debug("Node %s not found.\n", path);
		return 0;
	}
	ret = fdt_setprop_string(fdt, offs,
				 "status", "disabled");
	if (ret < 0) {
		printf("Could not add status property to node %s: %s\n",
		       path, fdt_strerror(ret));
		return ret;
	}
	return 0;
}

#if (CONFIG_SECURE_BOOT_SRAM != 0) || (CONFIG_SECURE_RUN_SRAM != 0)
static int ft_hs_fixup_sram(void *fdt, bd_t *bd)
{
	const char *path;
	int offs;
	int ret;
	u32 temp[2];

	/*
	 * Update SRAM reservations on secure devices. The OCMC RAM
	 * is always reserved for secure use from the start of that
	 * memory region
	 */
	path = "/ocp/ocmcram@40300000/sram-hs";
	offs = fdt_path_offset(fdt, path);
	if (offs < 0) {
		debug("Node %s not found.\n", path);
		return 0;
	}

	/* relative start offset */
	temp[0] = cpu_to_fdt32(0);
	/* reservation size */
	temp[1] = cpu_to_fdt32(max(CONFIG_SECURE_BOOT_SRAM,
				   CONFIG_SECURE_RUN_SRAM));
	fdt_delprop(fdt, offs, "reg");
	ret = fdt_setprop(fdt, offs, "reg", temp, 2 * sizeof(u32));
	if (ret < 0) {
		printf("Could not add reg property to node %s: %s\n",
		       path, fdt_strerror(ret));
		return ret;
	}

	return 0;
}
#else
static int ft_hs_fixup_sram(void *fdt, bd_t *bd) { return 0; }
#endif

#if (CONFIG_TI_SECURE_EMIF_TOTAL_REGION_SIZE != 0)
static int ft_hs_fixup_dram(void *fdt, bd_t *bd)
{
	const char *path, *subpath;
	int offs;
	u32 sec_mem_start = CONFIG_TI_SECURE_EMIF_REGION_START;
	u32 sec_mem_size = CONFIG_TI_SECURE_EMIF_TOTAL_REGION_SIZE;
	fdt64_t temp[2];

	/* If start address is zero, place at end of DRAM */
	if (0 == sec_mem_start)
		sec_mem_start =
			(CONFIG_SYS_SDRAM_BASE +
			(omap_sdram_size() - sec_mem_size));

	/* Delete any original secure_reserved node */
	path = "/reserved-memory/secure_reserved";
	offs = fdt_path_offset(fdt, path);
	if (offs >= 0)
		fdt_del_node(fdt, offs);

	/* Add new secure_reserved node */
	path = "/reserved-memory";
	offs = fdt_path_offset(fdt, path);
	if (offs < 0) {
		debug("Node %s not found\n", path);
		path = "/";
		subpath = "reserved-memory";
		offs = fdt_path_offset(fdt, path);
		offs = fdt_add_subnode(fdt, offs, subpath);
		if (offs < 0) {
			printf("Could not create %s%s node.\n", path, subpath);
			return 1;
		}
		path = "/reserved-memory";
		offs = fdt_path_offset(fdt, path);
	}

	subpath = "secure_reserved";
	offs = fdt_add_subnode(fdt, offs, subpath);
	if (offs < 0) {
		printf("Could not create %s%s node.\n", path, subpath);
		return 1;
	}

	temp[0] = cpu_to_fdt64(((u64)sec_mem_start));
	temp[1] = cpu_to_fdt64(((u64)sec_mem_size));
	fdt_setprop_string(fdt, offs, "compatible",
			   "ti,dra7-secure-memory");
	fdt_setprop_string(fdt, offs, "status", "okay");
	fdt_setprop(fdt, offs, "no-map", NULL, 0);
	fdt_setprop(fdt, offs, "reg", temp, sizeof(temp));

	return 0;
}
#else
static int ft_hs_fixup_dram(void *fdt, bd_t *bd) { return 0; }
#endif

static int ft_hs_add_tee(void *fdt, bd_t *bd)
{
	const char *path, *subpath;
	int offs;

	extern int tee_loaded;
	if (!tee_loaded)
		return 0;

	path = "/";
	offs = fdt_path_offset(fdt, path);

	subpath = "firmware";
	offs = fdt_add_subnode(fdt, offs, subpath);
	if (offs < 0) {
		printf("Could not create %s node.\n", subpath);
		return 1;
	}

	subpath = "optee";
	offs = fdt_add_subnode(fdt, offs, subpath);
	if (offs < 0) {
		printf("Could not create %s node.\n", subpath);
		return 1;
	}

	fdt_setprop_string(fdt, offs, "compatible", "linaro,optee-tz");
	fdt_setprop_string(fdt, offs, "method", "smc");

	return 0;
}

static void ft_hs_fixups(void *fdt, bd_t *bd)
{
	/* Check we are running on an HS/EMU device type */
	if (GP_DEVICE != get_device_type()) {
		if ((ft_hs_fixup_crossbar(fdt, bd) == 0) &&
		    (ft_hs_disable_rng(fdt, bd) == 0) &&
		    (ft_hs_fixup_sram(fdt, bd) == 0) &&
		    (ft_hs_fixup_dram(fdt, bd) == 0) &&
		    (ft_hs_add_tee(fdt, bd) == 0))
			return;
	} else {
		printf("ERROR: Incorrect device type (GP) detected!");
	}
	/* Fixup failed or wrong device type */
	hang();
}
#else
static void ft_hs_fixups(void *fdt, bd_t *bd)
{
}
#endif

#if defined(CONFIG_TARGET_DRA7XX_EVM) || defined(CONFIG_TARGET_AM57XX_EVM)
#define OPP_DSP_CLK_NUM	3
#define OPP_IVA_CLK_NUM	2
#define OPP_GPU_CLK_NUM	2

const char *dra7_opp_dsp_clk_names[OPP_DSP_CLK_NUM] = {
	"dpll_dsp_ck",
	"dpll_dsp_m2_ck",
	"dpll_dsp_m3x2_ck",
};

const char *dra7_opp_iva_clk_names[OPP_IVA_CLK_NUM] = {
	"dpll_iva_ck",
	"dpll_iva_m2_ck",
};

const char *dra7_opp_gpu_clk_names[OPP_GPU_CLK_NUM] = {
	"dpll_gpu_ck",
	"dpll_gpu_m2_ck",
};

/* DSPEVE voltage domain */
u32 dra7_opp_dsp_clk_rates[NUM_OPPS][OPP_DSP_CLK_NUM] = {
	{}, /*OPP_LOW */
	{600000000, 600000000, 400000000}, /* OPP_NOM */
	{700000000, 700000000, 466666667}, /* OPP_OD */
	{750000000, 750000000, 500000000}, /* OPP_HIGH */
};

/* IVA voltage domain */
u32 dra7_opp_iva_clk_rates[NUM_OPPS][OPP_IVA_CLK_NUM] = {
	{}, /* OPP_LOW */
	{1165000000, 388333334}, /* OPP_NOM */
	{860000000, 430000000}, /* OPP_OD */
	{1064000000, 532000000}, /* OPP_HIGH */
};

/* GPU voltage domain */
u32 dra7_opp_gpu_clk_rates[NUM_OPPS][OPP_GPU_CLK_NUM] = {
	{}, /* OPP_LOW */
	{1277000000, 425666667}, /* OPP_NOM */
	{1000000000, 500000000}, /* OPP_OD */
	{1064000000, 532000000}, /* OPP_HIGH */
};

static int ft_fixup_clocks(void *fdt, const char **names, u32 *rates, int num)
{
	int offs, node_offs, ret, i;
	uint32_t phandle;

	offs = fdt_path_offset(fdt, "/ocp/l4@4a000000/cm_core_aon@5000/clocks");
	if (offs < 0) {
		debug("Could not find cm_core_aon clocks node path offset : %s\n",
		      fdt_strerror(offs));
		return offs;
	}

	for (i = 0; i < num; i++) {
		node_offs = fdt_subnode_offset(fdt, offs, names[i]);
		if (node_offs < 0) {
			debug("Could not find clock sub-node %s: %s\n",
			      names[i], fdt_strerror(node_offs));
			return offs;
		}

		phandle = fdt_get_phandle(fdt, node_offs);
		if (!phandle) {
			debug("Could not find phandle for clock %s\n",
			      names[i]);
			return -1;
		}

		ret = fdt_setprop_u32(fdt, node_offs, "assigned-clocks",
				      phandle);
		if (ret < 0) {
			debug("Could not add assigned-clocks property to clock node %s: %s\n",
			      names[i], fdt_strerror(ret));
			return ret;
		}

		ret = fdt_setprop_u32(fdt, node_offs, "assigned-clock-rates",
				      rates[i]);
		if (ret < 0) {
			debug("Could not add assigned-clock-rates property to clock node %s: %s\n",
			      names[i], fdt_strerror(ret));
			return ret;
		}
	}

	return 0;
}

static void ft_opp_clock_fixups(void *fdt, bd_t *bd)
{
	const char **clk_names;
	u32 *clk_rates;
	int ret;

	if (!is_dra72x() && !is_dra7xx())
		return;

	/* fixup DSP clocks */
	clk_names = dra7_opp_dsp_clk_names;
	clk_rates = dra7_opp_dsp_clk_rates[get_voltrail_opp(VOLT_EVE)];
	ret = ft_fixup_clocks(fdt, clk_names, clk_rates, OPP_DSP_CLK_NUM);
	if (ret) {
		printf("ft_fixup_clocks failed for DSP voltage domain: %s\n",
		       fdt_strerror(ret));
		return;
	}

	/* fixup IVA clocks */
	clk_names = dra7_opp_iva_clk_names;
	clk_rates = dra7_opp_iva_clk_rates[get_voltrail_opp(VOLT_IVA)];
	ret = ft_fixup_clocks(fdt, clk_names, clk_rates, OPP_IVA_CLK_NUM);
	if (ret) {
		printf("ft_fixup_clocks failed for IVA voltage domain: %s\n",
		       fdt_strerror(ret));
		return;
	}

	/* fixup GPU clocks */
	clk_names = dra7_opp_gpu_clk_names;
	clk_rates = dra7_opp_gpu_clk_rates[get_voltrail_opp(VOLT_GPU)];
	ret = ft_fixup_clocks(fdt, clk_names, clk_rates, OPP_GPU_CLK_NUM);
	if (ret) {
		printf("ft_fixup_clocks failed for GPU voltage domain: %s\n",
		       fdt_strerror(ret));
		return;
	}
}
#else
static void ft_opp_clock_fixups(void *fdt, bd_t *bd) { }
#endif /* CONFIG_TARGET_DRA7XX_EVM || CONFIG_TARGET_AM57XX_EVM */

/*
 * Place for general cpu/SoC FDT fixups. Board specific
 * fixups should remain in the board files which is where
 * this function should be called from.
 */
void ft_cpu_setup(void *fdt, bd_t *bd)
{
	ft_hs_fixups(fdt, bd);
	ft_opp_clock_fixups(fdt, bd);
}
