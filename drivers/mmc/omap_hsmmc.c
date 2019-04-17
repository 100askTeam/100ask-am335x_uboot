/*
 * (C) Copyright 2008
 * Texas Instruments, <www.ti.com>
 * Sukumar Ghorai <s-ghorai@ti.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation's version 2 of
 * the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <config.h>
#include <common.h>
#include <malloc.h>
#include <memalign.h>
#include <mmc.h>
#include <part.h>
#include <i2c.h>
#include <twl4030.h>
#include <twl6030.h>
#include <palmas.h>
#include <asm/io.h>
#include <asm/arch/mmc_host_def.h>
#ifdef CONFIG_OMAP54XX
#include <asm/arch/mux_dra7xx.h>
#include <asm/arch/dra7xx_iodelay.h>
#endif
#if !defined(CONFIG_SOC_KEYSTONE)
#include <asm/gpio.h>
#include <asm/arch/sys_proto.h>
#endif
#include <dm.h>
#include <power/regulator.h>

DECLARE_GLOBAL_DATA_PTR;

/* simplify defines to OMAP_HSMMC_USE_GPIO */
#if (defined(CONFIG_OMAP_GPIO) && !defined(CONFIG_SPL_BUILD)) || \
	(defined(CONFIG_SPL_BUILD) && defined(CONFIG_SPL_GPIO_SUPPORT))
#define OMAP_HSMMC_USE_GPIO
#else
#undef OMAP_HSMMC_USE_GPIO
#endif

/* common definitions for all OMAPs */
#define SYSCTL_SRC	(1 << 25)
#define SYSCTL_SRD	(1 << 26)

#ifdef CONFIG_IODELAY_RECALIBRATION
struct omap_hsmmc_pinctrl_state {
	struct pad_conf_entry *padconf;
	int npads;
	struct iodelay_cfg_entry *iodelay;
	int niodelays;
};
#endif

struct omap_hsmmc_data {
	struct hsmmc *base_addr;
	struct mmc_config cfg;
	uint bus_width;
	uint clock;
#ifdef OMAP_HSMMC_USE_GPIO
#ifdef CONFIG_DM_MMC
	struct gpio_desc cd_gpio;	/* Change Detect GPIO */
	struct gpio_desc wp_gpio;	/* Write Protect GPIO */
	bool cd_inverted;
#else
	int cd_gpio;
	int wp_gpio;
#endif
#endif
#ifdef CONFIG_DM_MMC
	uint iov;
	uint timing;
	u8 controller_flags;
	struct omap_hsmmc_adma_desc *adma_desc_table;
	uint desc_slot;
	int node;
	char *version;
	struct udevice *vmmc_supply;
	struct udevice *vmmc_aux_supply;
	ushort last_cmd;
	uint signal_voltage;
#ifdef CONFIG_IODELAY_RECALIBRATION
	struct omap_hsmmc_pinctrl_state *default_pinctrl_state;
	struct omap_hsmmc_pinctrl_state *hs_pinctrl_state;
	struct omap_hsmmc_pinctrl_state *hs200_1_8v_pinctrl_state;
	struct omap_hsmmc_pinctrl_state *ddr_1_8v_pinctrl_state;
	struct omap_hsmmc_pinctrl_state *sdr104_pinctrl_state;
	struct omap_hsmmc_pinctrl_state *ddr50_pinctrl_state;
	struct omap_hsmmc_pinctrl_state *sdr50_pinctrl_state;
	struct omap_hsmmc_pinctrl_state *sdr25_pinctrl_state;
	struct omap_hsmmc_pinctrl_state *sdr12_pinctrl_state;
#endif
#endif
};

#ifdef CONFIG_DM_MMC
struct omap_hsmmc_adma_desc {
	u8 attr;
	u8 reserved;
	u16 len;
	u32 addr;
};

struct omap_mmc_of_data {
	u8 controller_flags;
};

#define ADMA_MAX_LEN	63488

/* Decriptor table defines */
#define ADMA_DESC_ATTR_VALID		BIT(0)
#define ADMA_DESC_ATTR_END		BIT(1)
#define ADMA_DESC_ATTR_INT		BIT(2)
#define ADMA_DESC_ATTR_ACT1		BIT(4)
#define ADMA_DESC_ATTR_ACT2		BIT(5)

#define ADMA_DESC_TRANSFER_DATA		ADMA_DESC_ATTR_ACT2
#define ADMA_DESC_LINK_DESC	(ADMA_DESC_ATTR_ACT1 | ADMA_DESC_ATTR_ACT2)
#endif

/* If we fail after 1 second wait, something is really bad */
#define MAX_RETRY_MS	1000
#define MMC_TIMEOUT_MS	20
#define OMAP_HSMMC_SUPPORTS_DUAL_VOLT		BIT(0)
#define OMAP_HSMMC_USE_ADMA			BIT(1)
#define OMAP_HSMMC_REQUIRE_IODELAY		BIT(2)

static int mmc_read_data(struct hsmmc *mmc_base, char *buf, unsigned int size);
static int mmc_write_data(struct hsmmc *mmc_base, const char *buf,
			unsigned int siz);
static void omap_hsmmc_start_clock(struct hsmmc *mmc_base);
static void omap_hsmmc_stop_clock(struct hsmmc *mmc_base);
static void mmc_reset_controller_fsm(struct hsmmc *mmc_base, u32 bit);

#if defined(OMAP_HSMMC_USE_GPIO) && !defined(CONFIG_DM_MMC)
static int omap_mmc_setup_gpio_in(int gpio, const char *label)
{
	int ret;

#ifndef CONFIG_DM_GPIO
	if (!gpio_is_valid(gpio))
		return -1;
#endif
	ret = gpio_request(gpio, label);
	if (ret)
		return ret;

	ret = gpio_direction_input(gpio);
	if (ret)
		return ret;

	return gpio;
}
#endif

static unsigned char mmc_board_init(struct mmc *mmc)
{
#if defined(CONFIG_OMAP34XX)
	t2_t *t2_base = (t2_t *)T2_BASE;
	struct prcm *prcm_base = (struct prcm *)PRCM_BASE;
	u32 pbias_lite;

	pbias_lite = readl(&t2_base->pbias_lite);
	pbias_lite &= ~(PBIASLITEPWRDNZ1 | PBIASLITEPWRDNZ0);
#ifdef CONFIG_TARGET_OMAP3_CAIRO
	/* for cairo board, we need to set up 1.8 Volt bias level on MMC1 */
	pbias_lite &= ~PBIASLITEVMODE0;
#endif
	writel(pbias_lite, &t2_base->pbias_lite);

	writel(pbias_lite | PBIASLITEPWRDNZ1 |
		PBIASSPEEDCTRL0 | PBIASLITEPWRDNZ0,
		&t2_base->pbias_lite);

	writel(readl(&t2_base->devconf0) | MMCSDIO1ADPCLKISEL,
		&t2_base->devconf0);

	writel(readl(&t2_base->devconf1) | MMCSDIO2ADPCLKISEL,
		&t2_base->devconf1);

	/* Change from default of 52MHz to 26MHz if necessary */
	if (!(mmc->cfg->host_caps & MMC_MODE_HS_52MHz))
		writel(readl(&t2_base->ctl_prog_io1) & ~CTLPROGIO1SPEEDCTRL,
			&t2_base->ctl_prog_io1);

	writel(readl(&prcm_base->fclken1_core) |
		EN_MMC1 | EN_MMC2 | EN_MMC3,
		&prcm_base->fclken1_core);

	writel(readl(&prcm_base->iclken1_core) |
		EN_MMC1 | EN_MMC2 | EN_MMC3,
		&prcm_base->iclken1_core);
#endif

#if defined(CONFIG_OMAP54XX) || defined(CONFIG_OMAP44XX)
	/* PBIAS config needed for MMC1 only */
	if (mmc->block_dev.devnum == 0)
		vmmc_pbias_config(LDO_VOLT_3V0);
#endif

	return 0;
}

void mmc_init_stream(struct hsmmc *mmc_base)
{
	ulong start;

	writel(readl(&mmc_base->con) | INIT_INITSTREAM, &mmc_base->con);

	writel(MMC_CMD0, &mmc_base->cmd);
	start = get_timer(0);
	while (!(readl(&mmc_base->stat) & CC_MASK)) {
		if (get_timer(0) - start > MAX_RETRY_MS) {
			printf("%s: timedout waiting for cc!\n", __func__);
			return;
		}
	}
	writel(CC_MASK, &mmc_base->stat)
		;
	writel(MMC_CMD0, &mmc_base->cmd)
		;
	start = get_timer(0);
	while (!(readl(&mmc_base->stat) & CC_MASK)) {
		if (get_timer(0) - start > MAX_RETRY_MS) {
			printf("%s: timedout waiting for cc2!\n", __func__);
			return;
		}
	}
	writel(readl(&mmc_base->con) & ~INIT_INITSTREAM, &mmc_base->con);
}

#ifdef CONFIG_DM_MMC
#ifdef CONFIG_IODELAY_RECALIBRATION
static void omap_hsmmc_set_timing(struct mmc *mmc)
{
	u32 val;
	struct hsmmc *mmc_base;
	struct omap_hsmmc_data *priv = (struct omap_hsmmc_data *)mmc->priv;
	struct omap_hsmmc_pinctrl_state *pinctrl_state;

	mmc_base = priv->base_addr;

	writel(readl(&mmc_base->con) & ~DDR, &mmc_base->con);
	omap_hsmmc_stop_clock(mmc_base);
	val = readl(&mmc_base->ac12);
	val &= ~AC12_UHSMC_MASK;
	switch (mmc->timing) {
	case MMC_TIMING_MMC_HS200:
		val |= AC12_UHSMC_SDR104;
		pinctrl_state = priv->hs200_1_8v_pinctrl_state;
		break;
	case MMC_TIMING_UHS_SDR104:
		val |= AC12_UHSMC_SDR104;
		pinctrl_state = priv->sdr104_pinctrl_state;
		break;
	case MMC_TIMING_UHS_DDR50:
		val |= AC12_UHSMC_DDR50;
		writel(readl(&mmc_base->con) | DDR, &mmc_base->con);
		pinctrl_state = priv->ddr50_pinctrl_state;
		break;
	case MMC_TIMING_UHS_SDR50:
		val |= AC12_UHSMC_SDR50;
		pinctrl_state = priv->sdr50_pinctrl_state;
		break;
	case MMC_TIMING_UHS_SDR25:
		val |= AC12_UHSMC_SDR25;
		pinctrl_state = priv->sdr25_pinctrl_state;
		break;
	case MMC_TIMING_UHS_SDR12:
		val |= AC12_UHSMC_SDR12;
		pinctrl_state = priv->sdr12_pinctrl_state;
		break;
	case MMC_TIMING_SD_HS:
	case MMC_TIMING_MMC_HS:
		val |= AC12_UHSMC_RES;
		pinctrl_state = priv->hs_pinctrl_state;
		break;
	case MMC_TIMING_MMC_DDR52:
		val |= AC12_UHSMC_RES;
		writel(readl(&mmc_base->con) | DDR, &mmc_base->con);
		pinctrl_state = priv->ddr_1_8v_pinctrl_state;
		break;
	default:
		val |= AC12_UHSMC_RES;
		pinctrl_state = priv->default_pinctrl_state;
		break;
	}
	writel(val, &mmc_base->ac12);

	if (priv->controller_flags & OMAP_HSMMC_REQUIRE_IODELAY) {
		if (pinctrl_state->iodelay)
			late_recalibrate_iodelay(pinctrl_state->padconf,
						 pinctrl_state->npads,
						 pinctrl_state->iodelay,
						 pinctrl_state->niodelays);
		else
			do_set_mux32((*ctrl)->control_padconf_core_base,
				     pinctrl_state->padconf,
				     pinctrl_state->npads);
	}

	omap_hsmmc_start_clock(mmc_base);
	priv->timing = mmc->timing;
}
#endif

static void omap_hsmmc_conf_bus_power(struct mmc *mmc, uint signal_voltage)
{
	struct hsmmc *mmc_base;
	struct omap_hsmmc_data *priv = (struct omap_hsmmc_data *)mmc->priv;
	u32 val;

	mmc_base = priv->base_addr;

	val = readl(&mmc_base->hctl) & ~SDVS_MASK;

	switch (signal_voltage) {
	case IOV_3V3:
		val |= SDVS_3V3;
		break;
	case IOV_3V0:
		val |= SDVS_3V0;
		break;
	case IOV_1V8:
		val |= SDVS_1V8;
		break;
	}

	writel(val, &mmc_base->hctl);
}

static int omap_hsmmc_card_busy_low(struct mmc *mmc)
{
	u32 val;
	int i;
	struct hsmmc *mmc_base;
	struct omap_hsmmc_data *priv = (struct omap_hsmmc_data *)mmc->priv;

	mmc_base = priv->base_addr;

	val = readl(&mmc_base->con);
	val &= ~CON_CLKEXTFREE;
	val |= CON_PADEN;
	writel(val, &mmc_base->con);

	/* By observation, card busy status reflects in 100 - 200us */
	for (i = 0; i < 5; i++) {
		val = readl(&mmc_base->pstate);
		if (!(val & (PSTATE_CLEV | PSTATE_DLEV)))
			return true;

		udelay(200);
	}

	return false;
}

static int omap_hsmmc_card_busy_high(struct mmc *mmc)
{
	int ret = true;
	u32 val;
	int i;
	struct hsmmc *mmc_base;
	struct omap_hsmmc_data *priv = (struct omap_hsmmc_data *)mmc->priv;

	mmc_base = priv->base_addr;

	val = readl(&mmc_base->con);
	val |= CON_CLKEXTFREE;
	writel(val, &mmc_base->con);

	/* By observation, card busy status reflects in 100 - 200us */
	for (i = 0; i < 5; i++) {
		val = readl(&mmc_base->pstate);
		if ((val & PSTATE_CLEV) && (val & PSTATE_DLEV)) {
			val = readl(&mmc_base->con);
			val &= ~(CON_CLKEXTFREE | CON_PADEN);
			writel(val, &mmc_base->con);
			ret = false;
			goto ret;
		}

		udelay(200);
	}

ret:
	return ret;
}

static int omap_hsmmc_card_busy(struct mmc *mmc)
{
	int ret;
	u32 val;
	struct hsmmc *mmc_base;
	struct omap_hsmmc_data *priv = (struct omap_hsmmc_data *)mmc->priv;

	mmc_base = priv->base_addr;

	if (priv->last_cmd != SD_CMD_SWITCH_UHS18V) {
		val = readl(&mmc_base->pstate);
		if (val & PSTATE_DLEV_DAT0)
			return true;
		return false;
	}

	val = readl(&mmc_base->ac12);
	if (val & AC12_V1V8_SIGEN)
		ret = omap_hsmmc_card_busy_high(mmc);
	else
		ret = omap_hsmmc_card_busy_low(mmc);

	return ret;
}

#ifdef CONFIG_DM_REGULATOR
static int omap_hsmmc_set_io_regulator(struct mmc *mmc, int uV)
{
	int ret;
	struct omap_hsmmc_data *priv = (struct omap_hsmmc_data *)mmc->priv;

	if (!priv->vmmc_aux_supply)
		return 0;

	ret = regulator_set_enable(priv->vmmc_aux_supply, false);
	if (ret && ret != -ENOSYS)
		return ret;

	ret = regulator_set_value(priv->vmmc_aux_supply, uV);
	if (ret)
		return ret;

	ret = regulator_set_enable(priv->vmmc_aux_supply, true);
	if (ret && ret != -ENOSYS)
		return ret;

	return 0;
}
#endif

static int omap_hsmmc_set_signal_voltage(struct mmc *mmc)
{
	u32 val;
	struct hsmmc *mmc_base;
	struct omap_hsmmc_data *priv = (struct omap_hsmmc_data *)mmc->priv;

	mmc_base = priv->base_addr;
	priv->signal_voltage = mmc->signal_voltage;

	if (mmc->signal_voltage == MMC_SIGNAL_VOLTAGE_330) {
		val = readl(&mmc_base->capa);
		if (!(val & VS30_3V0SUP))
			return -EOPNOTSUPP;

		omap_hsmmc_conf_bus_power(mmc, IOV_3V0);

		val = readl(&mmc_base->ac12);
		val &= ~AC12_V1V8_SIGEN;
		writel(val, &mmc_base->ac12);

#if defined(CONFIG_OMAP54XX) && defined(CONFIG_PALMAS_POWER)
#ifdef CONFIG_DM_REGULATOR
		return omap_hsmmc_set_io_regulator(mmc, 3000000);
#else
		vmmc_pbias_config(LDO_VOLT_3V0);
#endif
#endif
	} else if (mmc->signal_voltage == MMC_SIGNAL_VOLTAGE_180) {
		val = readl(&mmc_base->capa);
		if (!(val & VS18_1V8SUP))
			return -EOPNOTSUPP;

		omap_hsmmc_conf_bus_power(mmc, IOV_1V8);

		val = readl(&mmc_base->ac12);
		val |= AC12_V1V8_SIGEN;
		writel(val, &mmc_base->ac12);

#if defined(CONFIG_OMAP54XX) && defined(CONFIG_PALMAS_POWER)
#ifdef CONFIG_DM_REGULATOR
		return omap_hsmmc_set_io_regulator(mmc, 1800000);
#else
		vmmc_pbias_config(LDO_VOLT_1V8);
#endif
#endif
	} else {
		return -EOPNOTSUPP;
	}

	return 0;
}

static void omap_hsmmc_set_capabilities(struct mmc *mmc)
{
	struct hsmmc *mmc_base;
	struct omap_hsmmc_data *priv = (struct omap_hsmmc_data *)mmc->priv;
	u32 val;

	mmc_base = priv->base_addr;
	val = readl(&mmc_base->capa);

	if (priv->controller_flags & OMAP_HSMMC_SUPPORTS_DUAL_VOLT) {
		val |= (VS30_3V0SUP | VS18_1V8SUP);
		priv->iov = IOV_3V0;
	} else {
		switch (priv->iov) {
		case IOV_3V3:
			val |= VS33_3V3SUP;
			break;
		case IOV_3V0:
			val |= VS30_3V0SUP;
			break;
		case IOV_1V8:
			val |= VS18_1V8SUP;
			break;
		}
	}

	writel(val, &mmc_base->capa);
}

static void omap_hsmmc_disable_tuning(struct mmc *mmc)
{
	int val;
	struct hsmmc *mmc_base;

	mmc_base = ((struct omap_hsmmc_data *)mmc->priv)->base_addr;
	val = readl(&mmc_base->ac12);
	val &= ~(AC12_SCLK_SEL);
	writel(val, &mmc_base->ac12);

	val = readl(&mmc_base->dll);
	val &= ~(DLL_FORCE_VALUE | DLL_SWT);
	writel(val, &mmc_base->dll);
}

static void omap_hsmmc_set_dll(struct mmc *mmc, int count)
{
	int i;
	u32 val;
	struct hsmmc *mmc_base;

	mmc_base = ((struct omap_hsmmc_data *)mmc->priv)->base_addr;
	val = readl(&mmc_base->dll);
	val |= DLL_FORCE_VALUE;
	val &= ~(DLL_FORCE_SR_C_MASK << DLL_FORCE_SR_C_SHIFT);
	val |= (count << DLL_FORCE_SR_C_SHIFT);
	writel(val, &mmc_base->dll);

	val |= DLL_CALIB;
	writel(val, &mmc_base->dll);
	for (i = 0; i < 1000; i++) {
		if (readl(&mmc_base->dll) & DLL_CALIB)
			break;
	}
	val &= ~DLL_CALIB;
	writel(val, &mmc_base->dll);
}

static int omap_hsmmc_execute_tuning(struct mmc *mmc, uint opcode)
{
	struct hsmmc *mmc_base;

	u32 val;
	u8 cur_match, prev_match = 0;
	int ret;
	u32 phase_delay = 0;
	u32 start_window = 0, max_window = 0;
	u32 length = 0, max_len = 0;

	/* clock tuning is not needed for upto 52MHz */
	if (mmc->clock <= 52000000)
		return 0;

	mmc_base = ((struct omap_hsmmc_data *)mmc->priv)->base_addr;

	val = readl(&mmc_base->ac12);
	val |= AC12_V1V8_SIGEN;
	writel(val, &mmc_base->ac12);
	val = readl(&mmc_base->dll);
	val |= DLL_SWT;
	writel(val, &mmc_base->dll);
	while (phase_delay <= MAX_PHASE_DELAY) {
		omap_hsmmc_set_dll(mmc, phase_delay);

		cur_match = !mmc_send_tuning(mmc, opcode, NULL);

		if (cur_match) {
			if (prev_match) {
				length++;
			} else {
				start_window = phase_delay;
				length = 1;
			}
		}

		if (length > max_len) {
			max_window = start_window;
			max_len = length;
		}

		prev_match = cur_match;
		phase_delay += 4;
		udelay(100);
	val = readl(&mmc_base->dll);
	val &= ~DLL_FORCE_VALUE;
	writel(val, &mmc_base->dll);
	}

	if (!max_len) {
		ret = -EIO;
		goto tuning_error;
	}

	val = readl(&mmc_base->ac12);
	if (!(val & AC12_SCLK_SEL)) {
		ret = -EIO;
		goto tuning_error;
	}

	phase_delay = max_window + 4 * ((3 * max_len) >> 2);
	omap_hsmmc_set_dll(mmc, phase_delay);

	mmc_reset_controller_fsm(mmc_base, SYSCTL_SRD);
	mmc_reset_controller_fsm(mmc_base, SYSCTL_SRC);

	return 0;

tuning_error:

	omap_hsmmc_disable_tuning(mmc);
	mmc_reset_controller_fsm(mmc_base, SYSCTL_SRD);
	mmc_reset_controller_fsm(mmc_base, SYSCTL_SRC);

	return ret;
}

#ifdef CONFIG_DM_REGULATOR
static int omap_hsmmc_set_vdd(struct mmc *mmc, int enable)
{
	struct omap_hsmmc_data *priv = (struct omap_hsmmc_data *)mmc->priv;
	struct hsmmc *mmc_base = priv->base_addr;

	if (enable) {
		regulator_set_enable(priv->vmmc_supply, true);
		mmc_init_stream(mmc_base);
	} else {
		regulator_set_enable(priv->vmmc_supply, false);
	}

	return 0;
}
#endif
#endif

static void mmc_enable_irq(struct mmc *mmc, struct mmc_cmd *cmd)
{
	u32 irq_mask = INT_EN_MASK;
	struct hsmmc *mmc_base;

	mmc_base = ((struct omap_hsmmc_data *)mmc->priv)->base_addr;

	/*
	 * TODO: Errata i802 indicates only DCRC interrupts can occur during
	 * tuning procedure and DCRC should be disabled. But see occurences
	 * of DEB, CIE, CEB, CCRC interupts during tuning procedure. These
	 * interrupts occur along with BRR, so the data is actually in the
	 * buffer. It has to be debugged why these interrutps occur
	 */
	if (cmd && cmd->cmdidx == MMC_SEND_TUNING_BLOCK_HS200)
		irq_mask &= ~(IE_DEB | IE_DCRC | IE_CIE | IE_CEB | IE_CCRC);

	writel(irq_mask, &mmc_base->ie);
}

static int omap_hsmmc_init_setup(struct mmc *mmc)
{
	struct hsmmc *mmc_base;
	unsigned int reg_val;
	unsigned int dsor;
	ulong start;
#ifdef CONFIG_DM_MMC
	struct omap_hsmmc_data *priv = (struct omap_hsmmc_data *)mmc->priv;
#endif

	mmc_base = ((struct omap_hsmmc_data *)mmc->priv)->base_addr;
	mmc_board_init(mmc);

	writel(readl(&mmc_base->sysconfig) | MMC_SOFTRESET,
		&mmc_base->sysconfig);
	start = get_timer(0);
	while ((readl(&mmc_base->sysstatus) & RESETDONE) == 0) {
		if (get_timer(0) - start > MAX_RETRY_MS) {
			printf("%s: timedout waiting for cc2!\n", __func__);
			return TIMEOUT;
		}
	}
	writel(readl(&mmc_base->sysctl) | SOFTRESETALL, &mmc_base->sysctl);
	start = get_timer(0);
	while ((readl(&mmc_base->sysctl) & SOFTRESETALL) != 0x0) {
		if (get_timer(0) - start > MAX_RETRY_MS) {
			printf("%s: timedout waiting for softresetall!\n",
				__func__);
			return TIMEOUT;
		}
	}

#ifdef CONFIG_DM_MMC
	omap_hsmmc_set_capabilities(mmc);
	omap_hsmmc_conf_bus_power(mmc, priv->iov);
	reg_val = readl(&mmc_base->hl_hwinfo);
	if (reg_val & MADMA_EN)
		priv->controller_flags |= OMAP_HSMMC_USE_ADMA;
#else
	writel(DTW_1_BITMODE | SDBP_PWROFF | SDVS_3V0, &mmc_base->hctl);
	writel(readl(&mmc_base->capa) | VS30_3V0SUP | VS18_1V8SUP,
		&mmc_base->capa);
#endif

	reg_val = readl(&mmc_base->con) & RESERVED_MASK;

	writel(CTPL_MMC_SD | reg_val | WPP_ACTIVEHIGH | CDP_ACTIVEHIGH |
		MIT_CTO | DW8_1_4BITMODE | MODE_FUNC | STR_BLOCK |
		HR_NOHOSTRESP | INIT_NOINIT | NOOPENDRAIN, &mmc_base->con);

	dsor = 240;
	mmc_reg_out(&mmc_base->sysctl, (ICE_MASK | DTO_MASK | CEN_MASK),
		(ICE_STOP | DTO_15THDTO));
	mmc_reg_out(&mmc_base->sysctl, ICE_MASK | CLKD_MASK,
		(dsor << CLKD_OFFSET) | ICE_OSCILLATE);
	start = get_timer(0);
	while ((readl(&mmc_base->sysctl) & ICS_MASK) == ICS_NOTREADY) {
		if (get_timer(0) - start > MAX_RETRY_MS) {
			printf("%s: timedout waiting for ics!\n", __func__);
			return TIMEOUT;
		}
	}
	writel(readl(&mmc_base->sysctl) | CEN_ENABLE, &mmc_base->sysctl);

	writel(readl(&mmc_base->hctl) | SDBP_PWRON, &mmc_base->hctl);

	mmc_enable_irq(mmc, NULL);

#ifndef CONFIG_DM_REGULATOR
	mmc_init_stream(mmc_base);
#endif

	return 0;
}

/*
 * MMC controller internal finite state machine reset
 *
 * Used to reset command or data internal state machines, using respectively
 * SRC or SRD bit of SYSCTL register
 */
static void mmc_reset_controller_fsm(struct hsmmc *mmc_base, u32 bit)
{
	ulong start;

	mmc_reg_out(&mmc_base->sysctl, bit, bit);

	/*
	 * CMD(DAT) lines reset procedures are slightly different
	 * for OMAP3 and OMAP4(AM335x,OMAP5,DRA7xx).
	 * According to OMAP3 TRM:
	 * Set SRC(SRD) bit in MMCHS_SYSCTL register to 0x1 and wait until it
	 * returns to 0x0.
	 * According to OMAP4(AM335x,OMAP5,DRA7xx) TRMs, CMD(DATA) lines reset
	 * procedure steps must be as follows:
	 * 1. Initiate CMD(DAT) line reset by writing 0x1 to SRC(SRD) bit in
	 *    MMCHS_SYSCTL register (SD_SYSCTL for AM335x).
	 * 2. Poll the SRC(SRD) bit until it is set to 0x1.
	 * 3. Wait until the SRC (SRD) bit returns to 0x0
	 *    (reset procedure is completed).
	 */
#if defined(CONFIG_OMAP44XX) || defined(CONFIG_OMAP54XX) || \
	defined(CONFIG_AM33XX) || defined(CONFIG_AM43XX)
	if (!(readl(&mmc_base->sysctl) & bit)) {
		start = get_timer(0);
		/* To check why this bit is never set in DRA7xx */
		while (!(readl(&mmc_base->sysctl) & bit)) {
			if (get_timer(0) - start > MMC_TIMEOUT_MS)
				return;
		}
	}
#endif
	start = get_timer(0);
	while ((readl(&mmc_base->sysctl) & bit) != 0) {
		if (get_timer(0) - start > MAX_RETRY_MS) {
			printf("%s: timedout waiting for sysctl %x to clear\n",
				__func__, bit);
			return;
		}
	}
}

#ifdef CONFIG_DM_MMC
static int omap_hsmmc_adma_desc(struct mmc *mmc, char *buf, u16 len, bool end)
{
	struct omap_hsmmc_data *priv = (struct omap_hsmmc_data *)mmc->priv;
	struct omap_hsmmc_adma_desc *desc;
	u8 attr;

	desc = &priv->adma_desc_table[priv->desc_slot];

	attr = ADMA_DESC_ATTR_VALID | ADMA_DESC_TRANSFER_DATA;
	if (!end)
		priv->desc_slot++;
	else
		attr |= ADMA_DESC_ATTR_END;

	desc->len = len;
	desc->addr = (u32)buf;
	desc->reserved = 0;
	desc->attr = attr;

	return 0;
}

static int omap_hsmmc_prepare_adma_table(struct mmc *mmc, struct mmc_data *data)
{
	uint total_len = data->blocksize * data->blocks;
	uint desc_count = DIV_ROUND_UP(total_len, ADMA_MAX_LEN);
	struct omap_hsmmc_data *priv = (struct omap_hsmmc_data *)mmc->priv;
	int i = desc_count;
	char *buf;

	priv->desc_slot = 0;
	priv->adma_desc_table = (struct omap_hsmmc_adma_desc *)
				memalign(ARCH_DMA_MINALIGN, desc_count *
				sizeof(struct omap_hsmmc_adma_desc));

	if (data->flags & MMC_DATA_READ)
		buf = data->dest;
	else
		buf = (char *)data->src;

	while (--i) {
		omap_hsmmc_adma_desc(mmc, buf, ADMA_MAX_LEN, false);
		buf += ADMA_MAX_LEN;
		total_len -= ADMA_MAX_LEN;
	}

	omap_hsmmc_adma_desc(mmc, buf, total_len, true);

	flush_dcache_range((long)priv->adma_desc_table,
			   (long)priv->adma_desc_table +
			   ROUND(desc_count *
			   sizeof(struct omap_hsmmc_adma_desc),
			   ARCH_DMA_MINALIGN));
	return 0;
}

static void omap_hsmmc_prepare_data(struct mmc *mmc, struct mmc_data *data)
{
	struct hsmmc *mmc_base;
	struct omap_hsmmc_data *priv = (struct omap_hsmmc_data *)mmc->priv;
	u32 val;
	char *buf;

	mmc_base = ((struct omap_hsmmc_data *)mmc->priv)->base_addr;
	omap_hsmmc_prepare_adma_table(mmc, data);

	if (data->flags & MMC_DATA_READ)
		buf = data->dest;
	else
		buf = (char *)data->src;

	val = readl(&mmc_base->hctl);
	val |= DMA_SELECT;
	writel(val, &mmc_base->hctl);

	val = readl(&mmc_base->con);
	val |= DMA_MASTER;
	writel(val, &mmc_base->con);

	writel((u32)priv->adma_desc_table, &mmc_base->admasal);

	/* TODO: This shouldn't be required for read. However I don't seem
	 * to get valid data without this.
	 */
	flush_dcache_range((u32)buf,
			   (u32)buf +
			   ROUND(data->blocksize * data->blocks,
			   ARCH_DMA_MINALIGN));
}

static void omap_hsmmc_dma_cleanup(struct mmc *mmc)
{
	struct hsmmc *mmc_base;
	struct omap_hsmmc_data *priv = (struct omap_hsmmc_data *)mmc->priv;
	u32 val;

	mmc_base = ((struct omap_hsmmc_data *)mmc->priv)->base_addr;

	val = readl(&mmc_base->con);
	val &= ~DMA_MASTER;
	writel(val, &mmc_base->con);

	val = readl(&mmc_base->hctl);
	val &= ~DMA_SELECT;
	writel(val, &mmc_base->hctl);

	kfree(priv->adma_desc_table);
}
#endif

static int omap_hsmmc_send_cmd(struct mmc *mmc, struct mmc_cmd *cmd,
			struct mmc_data *data)
{
	struct hsmmc *mmc_base;
	unsigned int flags, mmc_stat;
	ulong start;
#ifdef CONFIG_DM_MMC
	struct omap_hsmmc_data *priv = (struct omap_hsmmc_data *)mmc->priv;
	priv->last_cmd = cmd->cmdidx;
#endif

	mmc_base = ((struct omap_hsmmc_data *)mmc->priv)->base_addr;

	if (cmd->cmdidx == MMC_CMD_STOP_TRANSMISSION)
		return 0;

	start = get_timer(0);
	while ((readl(&mmc_base->pstate) & (DATI_MASK | CMDI_MASK)) != 0) {
		if (get_timer(0) - start > MAX_RETRY_MS) {
			printf("%s: timedout waiting on cmd inhibit to clear\n",
					__func__);
			return TIMEOUT;
		}
	}
	writel(0xFFFFFFFF, &mmc_base->stat);
	start = get_timer(0);
	while (readl(&mmc_base->stat)) {
		if (get_timer(0) - start > MAX_RETRY_MS) {
			printf("%s: timedout waiting for STAT (%x) to clear\n",
				__func__, readl(&mmc_base->stat));
			return TIMEOUT;
		}
	}
	/*
	 * CMDREG
	 * CMDIDX[13:8]	: Command index
	 * DATAPRNT[5]	: Data Present Select
	 * ENCMDIDX[4]	: Command Index Check Enable
	 * ENCMDCRC[3]	: Command CRC Check Enable
	 * RSPTYP[1:0]
	 *	00 = No Response
	 *	01 = Length 136
	 *	10 = Length 48
	 *	11 = Length 48 Check busy after response
	 */
	/* Delay added before checking the status of frq change
	 * retry not supported by mmc.c(core file)
	 */
	if (cmd->cmdidx == SD_CMD_APP_SEND_SCR)
		udelay(50000); /* wait 50 ms */

	if (!(cmd->resp_type & MMC_RSP_PRESENT))
		flags = 0;
	else if (cmd->resp_type & MMC_RSP_136)
		flags = RSP_TYPE_LGHT136 | CICE_NOCHECK;
	else if (cmd->resp_type & MMC_RSP_BUSY)
		flags = RSP_TYPE_LGHT48B;
	else
		flags = RSP_TYPE_LGHT48;

	/* enable default flags */
	flags =	flags | (CMD_TYPE_NORMAL | CICE_NOCHECK | CCCE_NOCHECK |
			MSBS_SGLEBLK);
	flags &= ~(ACEN_ENABLE | BCE_ENABLE | DE_ENABLE);

	if (cmd->resp_type & MMC_RSP_CRC)
		flags |= CCCE_CHECK;
	if (cmd->resp_type & MMC_RSP_OPCODE)
		flags |= CICE_CHECK;

	if (data) {
		if ((cmd->cmdidx == MMC_CMD_READ_MULTIPLE_BLOCK) ||
			 (cmd->cmdidx == MMC_CMD_WRITE_MULTIPLE_BLOCK)) {
			flags |= (MSBS_MULTIBLK | BCE_ENABLE | ACEN_ENABLE);
			data->blocksize = 512;
			writel(data->blocksize | (data->blocks << 16),
							&mmc_base->blk);
		} else
			writel(data->blocksize | NBLK_STPCNT, &mmc_base->blk);

		if (data->flags & MMC_DATA_READ)
			flags |= (DP_DATA | DDIR_READ);
		else
			flags |= (DP_DATA | DDIR_WRITE);

#ifdef CONFIG_DM_MMC
		if ((priv->controller_flags & OMAP_HSMMC_USE_ADMA) &&
		    cmd->cmdidx != MMC_SEND_TUNING_BLOCK_HS200) {
			omap_hsmmc_prepare_data(mmc, data);
			flags |= DE_ENABLE;
		}
#endif
	}

	mmc_enable_irq(mmc, cmd);

	writel(cmd->cmdarg, &mmc_base->arg);
	udelay(20);		/* To fix "No status update" error on eMMC */
	writel((cmd->cmdidx << 24) | flags, &mmc_base->cmd);

	start = get_timer(0);
	do {
		mmc_stat = readl(&mmc_base->stat);
		if (get_timer(0) - start > MAX_RETRY_MS) {
			printf("%s : timeout: No status update\n", __func__);
			return TIMEOUT;
		}
	} while (!mmc_stat);

	if ((mmc_stat & IE_CTO) != 0) {
		mmc_reset_controller_fsm(mmc_base, SYSCTL_SRC);
		return TIMEOUT;
	} else if ((mmc_stat & ERRI_MASK) != 0)
		return -1;

	if (mmc_stat & CC_MASK) {
		writel(CC_MASK, &mmc_base->stat);
		if (cmd->resp_type & MMC_RSP_PRESENT) {
			if (cmd->resp_type & MMC_RSP_136) {
				/* response type 2 */
				cmd->response[3] = readl(&mmc_base->rsp10);
				cmd->response[2] = readl(&mmc_base->rsp32);
				cmd->response[1] = readl(&mmc_base->rsp54);
				cmd->response[0] = readl(&mmc_base->rsp76);
			} else
				/* response types 1, 1b, 3, 4, 5, 6 */
				cmd->response[0] = readl(&mmc_base->rsp10);
		}
	}

#ifdef CONFIG_DM_MMC
	if ((priv->controller_flags & OMAP_HSMMC_USE_ADMA) && data &&
	    cmd->cmdidx != MMC_SEND_TUNING_BLOCK_HS200) {
		if (mmc_stat & IE_ADMAE) {
			omap_hsmmc_dma_cleanup(mmc);
			return -1;
		}

		do {
			mmc_stat = readl(&mmc_base->stat);
			if (mmc_stat & TC_MASK) {
				writel(readl(&mmc_base->stat) | TC_MASK,
				       &mmc_base->stat);
				break;
			}
		} while (1);

		omap_hsmmc_dma_cleanup(mmc);
		return 0;
	}
#endif

	if (data && (data->flags & MMC_DATA_READ)) {
		mmc_read_data(mmc_base,	data->dest,
				data->blocksize * data->blocks);
	} else if (data && (data->flags & MMC_DATA_WRITE)) {
		mmc_write_data(mmc_base, data->src,
				data->blocksize * data->blocks);
	}
	return 0;
}

static int mmc_read_data(struct hsmmc *mmc_base, char *buf, unsigned int size)
{
	unsigned int *output_buf = (unsigned int *)buf;
	unsigned int mmc_stat;
	unsigned int count;

	/*
	 * Start Polled Read
	 */
	count = (size > MMCSD_SECTOR_SIZE) ? MMCSD_SECTOR_SIZE : size;
	count /= 4;

	while (size) {
		ulong start = get_timer(0);
		do {
			mmc_stat = readl(&mmc_base->stat);
			if (get_timer(0) - start > MAX_RETRY_MS) {
				printf("%s: timedout waiting for status!\n",
						__func__);
				return TIMEOUT;
			}
		} while (mmc_stat == 0);

		if ((mmc_stat & (IE_DTO | IE_DCRC | IE_DEB)) != 0)
			mmc_reset_controller_fsm(mmc_base, SYSCTL_SRD);

		if ((mmc_stat & ERRI_MASK) != 0)
			return 1;

		if (mmc_stat & BRR_MASK) {
			unsigned int k;

			writel(readl(&mmc_base->stat) | BRR_MASK,
				&mmc_base->stat);
			for (k = 0; k < count; k++) {
				*output_buf = readl(&mmc_base->data);
				output_buf++;
			}
			size -= (count*4);
		}

		if (mmc_stat & BWR_MASK)
			writel(readl(&mmc_base->stat) | BWR_MASK,
				&mmc_base->stat);

		if (mmc_stat & TC_MASK) {
			writel(readl(&mmc_base->stat) | TC_MASK,
				&mmc_base->stat);
			break;
		}
	}
	return 0;
}

static int mmc_write_data(struct hsmmc *mmc_base, const char *buf,
				unsigned int size)
{
	unsigned int *input_buf = (unsigned int *)buf;
	unsigned int mmc_stat;
	unsigned int count;

	/*
	 * Start Polled Write
	 */
	count = (size > MMCSD_SECTOR_SIZE) ? MMCSD_SECTOR_SIZE : size;
	count /= 4;

	while (size) {
		ulong start = get_timer(0);
		do {
			mmc_stat = readl(&mmc_base->stat);
			if (get_timer(0) - start > MAX_RETRY_MS) {
				printf("%s: timedout waiting for status!\n",
						__func__);
				return TIMEOUT;
			}
		} while (mmc_stat == 0);

		if ((mmc_stat & (IE_DTO | IE_DCRC | IE_DEB)) != 0)
			mmc_reset_controller_fsm(mmc_base, SYSCTL_SRD);

		if ((mmc_stat & ERRI_MASK) != 0)
			return 1;

		if (mmc_stat & BWR_MASK) {
			unsigned int k;

			writel(readl(&mmc_base->stat) | BWR_MASK,
					&mmc_base->stat);
			for (k = 0; k < count; k++) {
				writel(*input_buf, &mmc_base->data);
				input_buf++;
			}
			size -= (count*4);
		}

		if (mmc_stat & BRR_MASK)
			writel(readl(&mmc_base->stat) | BRR_MASK,
				&mmc_base->stat);

		if (mmc_stat & TC_MASK) {
			writel(readl(&mmc_base->stat) | TC_MASK,
				&mmc_base->stat);
			break;
		}
	}
	return 0;
}

static void omap_hsmmc_stop_clock(struct hsmmc *mmc_base)
{
	writel(readl(&mmc_base->sysctl) & ~CEN_ENABLE, &mmc_base->sysctl);
}

static void omap_hsmmc_start_clock(struct hsmmc *mmc_base)
{
	writel(readl(&mmc_base->sysctl) | CEN_ENABLE, &mmc_base->sysctl);
}

static void omap_hsmmc_set_clock(struct mmc *mmc)
{
	struct omap_hsmmc_data *priv_data = mmc->priv;
	struct hsmmc *mmc_base;
	unsigned int dsor = 0;
	ulong start;

	mmc_base = priv_data->base_addr;
	omap_hsmmc_stop_clock(mmc_base);

	/* TODO: Is setting DTO required here? */
	mmc_reg_out(&mmc_base->sysctl, (ICE_MASK | DTO_MASK),
		    (ICE_STOP | DTO_15THDTO));

	if (mmc->clock != 0) {
		dsor = DIV_ROUND_UP(MMC_CLOCK_REFERENCE * 1000000, mmc->clock);
		if (dsor > CLKD_MAX)
			dsor = CLKD_MAX;
	}

	mmc_reg_out(&mmc_base->sysctl, ICE_MASK | CLKD_MASK,
		    (dsor << CLKD_OFFSET) | ICE_OSCILLATE);

	start = get_timer(0);
	while ((readl(&mmc_base->sysctl) & ICS_MASK) == ICS_NOTREADY) {
		if (get_timer(0) - start > MAX_RETRY_MS) {
			printf("%s: timedout waiting for ics!\n", __func__);
			return;
		}
	}

	priv_data->clock = mmc->clock;
	omap_hsmmc_start_clock(mmc_base);
}

static void omap_hsmmc_set_bus_width(struct mmc *mmc)
{
	struct omap_hsmmc_data *priv_data = mmc->priv;
	struct hsmmc *mmc_base;

	mmc_base = priv_data->base_addr;
	/* configue bus width */
	switch (mmc->bus_width) {
	case 8:
		writel(readl(&mmc_base->con) | DTW_8_BITMODE,
			&mmc_base->con);
		break;

	case 4:
		writel(readl(&mmc_base->con) & ~DTW_8_BITMODE,
			&mmc_base->con);
		writel(readl(&mmc_base->hctl) | DTW_4_BITMODE,
			&mmc_base->hctl);
		break;

	case 1:
	default:
		writel(readl(&mmc_base->con) & ~DTW_8_BITMODE,
			&mmc_base->con);
		writel(readl(&mmc_base->hctl) & ~DTW_4_BITMODE,
			&mmc_base->hctl);
		break;
	}

	priv_data->bus_width = mmc->bus_width;
}

static int omap_hsmmc_set_ios(struct mmc *mmc)
{
	struct omap_hsmmc_data *priv_data = mmc->priv;
	struct hsmmc *mmc_base = priv_data->base_addr;
	int ret = 0;

	if (priv_data->bus_width != mmc->bus_width)
		omap_hsmmc_set_bus_width(mmc);

	if (priv_data->clock != mmc->clock)
		omap_hsmmc_set_clock(mmc);

	if (mmc->clk_disable)
		omap_hsmmc_stop_clock(mmc_base);
	else
		omap_hsmmc_start_clock(mmc_base);

#ifdef CONFIG_DM_MMC
#ifdef CONFIG_IODELAY_RECALIBRATION
	if (priv_data->timing != mmc->timing)
		omap_hsmmc_set_timing(mmc);
#endif
	if (priv_data->signal_voltage != mmc->signal_voltage)
		ret = omap_hsmmc_set_signal_voltage(mmc);
#endif

	return ret;
}

#ifdef OMAP_HSMMC_USE_GPIO
#ifdef CONFIG_DM_MMC
static int omap_hsmmc_getcd(struct mmc *mmc)
{
	struct omap_hsmmc_data *priv = mmc->priv;
	int value;

	value = dm_gpio_get_value(&priv->cd_gpio);
	/* if no CD return as 1 */
	if (value < 0)
		return 1;

	if (priv->cd_inverted)
		return !value;
	return value;
}

static int omap_hsmmc_getwp(struct mmc *mmc)
{
	struct omap_hsmmc_data *priv = mmc->priv;
	int value;

	value = dm_gpio_get_value(&priv->wp_gpio);
	/* if no WP return as 0 */
	if (value < 0)
		return 0;
	return value;
}
#else
static int omap_hsmmc_getcd(struct mmc *mmc)
{
	struct omap_hsmmc_data *priv_data = mmc->priv;
	int cd_gpio;

	/* if no CD return as 1 */
	cd_gpio = priv_data->cd_gpio;
	if (cd_gpio < 0)
		return 1;

	/* NOTE: assumes card detect signal is active-low */
	return !gpio_get_value(cd_gpio);
}

static int omap_hsmmc_getwp(struct mmc *mmc)
{
	struct omap_hsmmc_data *priv_data = mmc->priv;
	int wp_gpio;

	/* if no WP return as 0 */
	wp_gpio = priv_data->wp_gpio;
	if (wp_gpio < 0)
		return 0;

	/* NOTE: assumes write protect signal is active-high */
	return gpio_get_value(wp_gpio);
}
#endif
#endif

static const struct mmc_ops omap_hsmmc_ops = {
	.send_cmd	= omap_hsmmc_send_cmd,
	.set_ios	= omap_hsmmc_set_ios,
	.init		= omap_hsmmc_init_setup,
#ifdef OMAP_HSMMC_USE_GPIO
	.getcd		= omap_hsmmc_getcd,
	.getwp		= omap_hsmmc_getwp,
#endif
#ifdef CONFIG_DM_MMC
	.execute_tuning = omap_hsmmc_execute_tuning,
	.card_busy = omap_hsmmc_card_busy,
#ifdef CONFIG_DM_REGULATOR
	.set_vdd	= omap_hsmmc_set_vdd,
#endif
#endif
};

#ifndef CONFIG_DM_MMC
int omap_mmc_init(int dev_index, uint host_caps_mask, uint f_max, int cd_gpio,
		int wp_gpio)
{
	struct mmc *mmc;
	struct omap_hsmmc_data *priv_data;
	struct mmc_config *cfg;
	uint host_caps_val;

	priv_data = malloc(sizeof(*priv_data));
	if (priv_data == NULL)
		return -1;

	host_caps_val = MMC_MODE_4BIT | MMC_MODE_HS_52MHz | MMC_MODE_HS;

	switch (dev_index) {
	case 0:
		priv_data->base_addr = (struct hsmmc *)OMAP_HSMMC1_BASE;
		break;
#ifdef OMAP_HSMMC2_BASE
	case 1:
		priv_data->base_addr = (struct hsmmc *)OMAP_HSMMC2_BASE;
#if (defined(CONFIG_OMAP44XX) || defined(CONFIG_OMAP54XX) || \
	defined(CONFIG_DRA7XX) || \
	defined(CONFIG_AM43XX) || defined(CONFIG_SOC_KEYSTONE)) && \
		defined(CONFIG_HSMMC2_8BIT)
		/* Enable 8-bit interface for eMMC on OMAP4/5 or DRA7XX */
		host_caps_val |= MMC_MODE_8BIT;
#endif
		break;
#endif
#ifdef OMAP_HSMMC3_BASE
	case 2:
		priv_data->base_addr = (struct hsmmc *)OMAP_HSMMC3_BASE;
#if defined(CONFIG_DRA7XX) && defined(CONFIG_HSMMC3_8BIT)
		/* Enable 8-bit interface for eMMC on DRA7XX */
		host_caps_val |= MMC_MODE_8BIT;
#endif
		break;
#endif
	default:
		priv_data->base_addr = (struct hsmmc *)OMAP_HSMMC1_BASE;
		return 1;
	}
#ifdef OMAP_HSMMC_USE_GPIO
	/* on error gpio values are set to -1, which is what we want */
	priv_data->cd_gpio = omap_mmc_setup_gpio_in(cd_gpio, "mmc_cd");
	priv_data->wp_gpio = omap_mmc_setup_gpio_in(wp_gpio, "mmc_wp");
#endif

	cfg = &priv_data->cfg;

	cfg->name = "OMAP SD/MMC";
	cfg->ops = &omap_hsmmc_ops;

	cfg->voltages = MMC_VDD_32_33 | MMC_VDD_33_34 | MMC_VDD_165_195;
	cfg->host_caps = host_caps_val & ~host_caps_mask;

	cfg->f_min = 400000;

	if (f_max != 0)
		cfg->f_max = f_max;
	else {
		if (cfg->host_caps & MMC_MODE_HS) {
			if (cfg->host_caps & MMC_MODE_HS_52MHz)
				cfg->f_max = 52000000;
			else
				cfg->f_max = 26000000;
		} else
			cfg->f_max = 20000000;
	}

	cfg->b_max = CONFIG_SYS_MMC_MAX_BLK_COUNT;

#if defined(CONFIG_OMAP34XX)
	/*
	 * Silicon revs 2.1 and older do not support multiblock transfers.
	 */
	if ((get_cpu_family() == CPU_OMAP34XX) && (get_cpu_rev() <= CPU_3XX_ES21))
		cfg->b_max = 1;
#endif
	mmc = mmc_create(cfg, priv_data);
	if (mmc == NULL)
		return -1;

	return 0;
}
#else
#ifdef CONFIG_IODELAY_RECALIBRATION
static struct pad_conf_entry *
omap_hsmmc_get_pad_conf_entry(const fdt32_t *pinctrl, int count)
{
	int index = 0;
	struct pad_conf_entry *padconf;

	padconf = (struct pad_conf_entry *)malloc(sizeof(*padconf) * count);
	if (!padconf) {
		printf("failed to allocate memory\n");
		return 0;
	}

	while (index < count) {
		padconf[index].offset = fdt32_to_cpu(pinctrl[index]);
		padconf[index].val = fdt32_to_cpu(pinctrl[index + 1]);
		index += 2;
	}

	return padconf;
}

static struct iodelay_cfg_entry *
omap_hsmmc_get_iodelay_cfg_entry(const fdt32_t *pinctrl, int count)
{
	int index = 0;
	struct iodelay_cfg_entry *iodelay;

	iodelay = (struct iodelay_cfg_entry *)malloc(sizeof(*iodelay) * count);
	if (!iodelay) {
		printf("failed to allocate memory\n");
		return 0;
	}

	while (index < count) {
		iodelay[index].offset = fdt32_to_cpu(pinctrl[index]);
		iodelay[index].a_delay = fdt32_to_cpu(pinctrl[index + 1])
						      & 0xFFFF;
		iodelay[index].g_delay = (fdt32_to_cpu(pinctrl[index + 1])
					  & 0xFFFF0000) >> 16;
		index += 2;
	}

	return iodelay;
}

static const fdt32_t *omap_hsmmc_get_pinctrl_entry(uint32_t phandle, int *len)
{
	const void *fdt = gd->fdt_blob;
	int offset;
	const fdt32_t *pinctrl;

	offset = fdt_node_offset_by_phandle(fdt, phandle);
	if (offset < 0) {
		printf("failed to get pinctrl node %s.\n",
		       fdt_strerror(offset));
		return 0;
	}

	pinctrl = fdt_getprop(fdt, offset, "pinctrl-single,pins", len);
	if (!pinctrl) {
		printf("failed to get property pinctrl-single,pins\n");
		return 0;
	}

	return pinctrl;
}

static uint32_t omap_hsmmc_get_pad_conf_phandle(struct mmc *mmc,
						char *prop_name)
{
	const void *fdt = gd->fdt_blob;
	const __be32 *phandle;
	struct omap_hsmmc_data *priv = (struct omap_hsmmc_data *)mmc->priv;
	int node = priv->node;

	phandle = fdt_getprop(fdt, node, prop_name, NULL);
	if (!phandle) {
		printf("failed to get property %s\n", prop_name);
		return 0;
	}

	return fdt32_to_cpu(*phandle);
}

static uint32_t omap_hsmmc_get_iodelay_phandle(struct mmc *mmc,
					       char *prop_name)
{
	const void *fdt = gd->fdt_blob;
	const __be32 *phandle;
	struct omap_hsmmc_data *priv = (struct omap_hsmmc_data *)mmc->priv;
	int len;
	int count;
	int node = priv->node;

	phandle = fdt_getprop(fdt, node, prop_name, &len);
	if (!phandle) {
		printf("failed to get property %s\n", prop_name);
		return 0;
	}

	/* No manual mode iodelay values if count < 2 */
	count = len / sizeof(*phandle);
	if (count < 2)
		return 0;

	return fdt32_to_cpu(*(phandle + 1));
}


static struct pad_conf_entry *
omap_hsmmc_get_pad_conf(struct mmc *mmc, char *prop_name, int *npads)
{
	int len;
	int count;
	struct pad_conf_entry *padconf;
	uint32_t phandle;
	const fdt32_t *pinctrl;

	phandle = omap_hsmmc_get_pad_conf_phandle(mmc, prop_name);
	if (!phandle)
		return ERR_PTR(-EINVAL);

	pinctrl = omap_hsmmc_get_pinctrl_entry(phandle, &len);
	if (!pinctrl)
		return ERR_PTR(-EINVAL);

	count = len / sizeof(*pinctrl);
	padconf = omap_hsmmc_get_pad_conf_entry(pinctrl, count);
	if (!padconf)
		return ERR_PTR(-EINVAL);

	*npads = count;

	return padconf;
}

static struct iodelay_cfg_entry *
omap_hsmmc_get_iodelay(struct mmc *mmc, char *prop_name, int *niodelay)
{
	int len;
	int count;
	struct iodelay_cfg_entry *iodelay;
	uint32_t phandle;
	const fdt32_t *pinctrl;

	phandle = omap_hsmmc_get_iodelay_phandle(mmc, prop_name);
	/* Not all modes have manual mode iodelay values. So its not fatal */
	if (!phandle)
		return 0;

	pinctrl = omap_hsmmc_get_pinctrl_entry(phandle, &len);
	if (!pinctrl)
		return ERR_PTR(-EINVAL);

	count = len / sizeof(*pinctrl);
	iodelay = omap_hsmmc_get_iodelay_cfg_entry(pinctrl, count);
	if (!iodelay)
		return ERR_PTR(-EINVAL);

	*niodelay = count;

	return iodelay;
}

static struct omap_hsmmc_pinctrl_state *
omap_hsmmc_get_pinctrl_by_mode(struct mmc *mmc, char *mode)
{
	int index;
	int npads = 0;
	int niodelays = 0;
	const void *fdt = gd->fdt_blob;
	struct omap_hsmmc_data *priv = (struct omap_hsmmc_data *)mmc->priv;
	int node = priv->node;
	char prop_name[11];
	struct omap_hsmmc_pinctrl_state *pinctrl_state;

	pinctrl_state = (struct omap_hsmmc_pinctrl_state *)
			 malloc(sizeof(*pinctrl_state));
	if (!pinctrl_state) {
		printf("failed to allocate memory\n");
		return 0;
	}

	index = fdt_find_string(fdt, node, "pinctrl-names", mode);
	if (index < 0)
		goto err_pinctrl_state;

	sprintf(prop_name, "pinctrl-%d", index);

	pinctrl_state->padconf = omap_hsmmc_get_pad_conf(mmc, prop_name,
							 &npads);
	if (IS_ERR(pinctrl_state->padconf))
		goto err_pinctrl_state;
	pinctrl_state->npads = npads;

	pinctrl_state->iodelay = omap_hsmmc_get_iodelay(mmc, prop_name,
							&niodelays);
	if (IS_ERR(pinctrl_state->iodelay))
		goto err_padconf;
	pinctrl_state->niodelays = niodelays;

	return pinctrl_state;

err_padconf:
	kfree(pinctrl_state->padconf);

err_pinctrl_state:
	kfree(pinctrl_state);
	return 0;
}

#define OMAP_HSMMC_SETUP_PINCTRL(capmask, mode)				\
	do {								\
		struct omap_hsmmc_pinctrl_state *s = NULL;		\
		char str[20];						\
		if (!(cfg->host_caps & capmask))			\
			break;						\
									\
		if (priv->version) {					\
			sprintf(str, "%s-%s", #mode, priv->version);	\
			s = omap_hsmmc_get_pinctrl_by_mode(mmc, str);	\
		}							\
									\
		if (!s)							\
			s = omap_hsmmc_get_pinctrl_by_mode(mmc, #mode);	\
									\
		if (!s) {						\
			printf("no pinctrl for %s\n", #mode);		\
			cfg->host_caps &= ~(capmask);			\
		} else {						\
			priv->mode##_pinctrl_state = s;			\
		}							\
	} while (0)

static int omap_hsmmc_get_pinctrl_state(struct mmc *mmc)
{
	struct omap_hsmmc_data *priv = (struct omap_hsmmc_data *)mmc->priv;
	struct mmc_config *cfg = &priv->cfg;
	struct omap_hsmmc_pinctrl_state *default_pinctrl;

	if (!(priv->controller_flags & OMAP_HSMMC_REQUIRE_IODELAY))
		return 0;

	default_pinctrl = omap_hsmmc_get_pinctrl_by_mode(mmc, "default");
	if (!default_pinctrl) {
		printf("no pinctrl state for default mode\n");
		return -EINVAL;
	}
	priv->default_pinctrl_state = default_pinctrl;

	OMAP_HSMMC_SETUP_PINCTRL(MMC_MODE_HS200, hs200_1_8v);
	OMAP_HSMMC_SETUP_PINCTRL(MMC_MODE_DDR_52MHz, ddr_1_8v);
	OMAP_HSMMC_SETUP_PINCTRL(MMC_MODE_HS, hs);
	OMAP_HSMMC_SETUP_PINCTRL(MMC_MODE_UHS_SDR104, sdr104);
	OMAP_HSMMC_SETUP_PINCTRL(MMC_MODE_UHS_DDR50, ddr50);
	OMAP_HSMMC_SETUP_PINCTRL(MMC_MODE_UHS_SDR50, sdr50);
	OMAP_HSMMC_SETUP_PINCTRL(MMC_MODE_UHS_SDR25, sdr25);
	OMAP_HSMMC_SETUP_PINCTRL(MMC_MODE_UHS_SDR12, sdr12);

	return 0;
}
#endif

static int omap_hsmmc_ofdata_to_platdata(struct udevice *dev)
{
	struct omap_hsmmc_data *priv = dev_get_priv(dev);
	const void *fdt = gd->fdt_blob;
	int node = dev->of_offset;
	struct mmc_config *cfg;
	int ret;

	priv->base_addr = map_physmem(dev_get_addr(dev), sizeof(struct hsmmc *),
				      MAP_NOCACHE);
	priv->node = node;
	cfg = &priv->cfg;

	ret = mmc_of_parse(fdt, node, cfg);
	if (ret < 0)
		return ret;

	cfg->host_caps |= MMC_MODE_HS_52MHz | MMC_MODE_HS;
	cfg->f_min = 400000;
	cfg->voltages = MMC_VDD_32_33 | MMC_VDD_33_34 | MMC_VDD_165_195;
	cfg->b_max = CONFIG_SYS_MMC_MAX_BLK_COUNT;
	if (fdtdec_get_bool(fdt, node, "ti,dual-volt"))
		priv->controller_flags |= OMAP_HSMMC_SUPPORTS_DUAL_VOLT;
	priv->iov = fdtdec_get_int(fdt, node, "iov", 1800000);

#ifdef OMAP_HSMMC_USE_GPIO
	priv->cd_inverted = fdtdec_get_bool(fdt, node, "cd-inverted");
#endif

	return 0;
}

__weak int platform_fixup_disable_uhs_mode(void)
{
	return 0;
}

static int omap_hsmmc_platform_fixup(struct mmc *mmc)
{
	struct omap_hsmmc_data *priv = (struct omap_hsmmc_data *)mmc->priv;
	struct mmc_config *cfg = &priv->cfg;

	priv->version = NULL;

	if (platform_fixup_disable_uhs_mode()) {
		priv->version = "rev11";
		cfg->host_caps &= ~(MMC_MODE_HS200 | MMC_MODE_UHS_SDR104
				    | MMC_MODE_UHS_SDR50);
	}

	return 0;
}

static int omap_hsmmc_probe(struct udevice *dev)
{
	struct mmc_uclass_priv *upriv = dev_get_uclass_priv(dev);
	struct omap_hsmmc_data *priv = dev_get_priv(dev);
	struct mmc_config *cfg;
	struct mmc *mmc;
#ifdef CONFIG_IODELAY_RECALIBRATION
	int ret;
	struct omap_mmc_of_data *data;
#endif

	cfg = &priv->cfg;
	cfg->name = "OMAP SD/MMC";
	cfg->ops = &omap_hsmmc_ops;

	mmc = mmc_create(cfg, priv);
	if (mmc == NULL)
		return -1;

	omap_hsmmc_platform_fixup(mmc);

#ifdef CONFIG_DM_REGULATOR
	device_get_supply_regulator(dev, "vmmc-supply", &priv->vmmc_supply);
	device_get_supply_regulator(dev, "vmmc_aux-supply",
				    &priv->vmmc_aux_supply);
#endif
#ifdef OMAP_HSMMC_USE_GPIO
	gpio_request_by_name(dev, "cd-gpios", 0, &priv->cd_gpio, GPIOD_IS_IN);
	gpio_request_by_name(dev, "wp-gpios", 0, &priv->wp_gpio, GPIOD_IS_IN);
#endif

#ifdef CONFIG_IODELAY_RECALIBRATION
	data  = (struct omap_mmc_of_data *)dev_get_driver_data(dev);
	if (data && data->controller_flags & OMAP_HSMMC_REQUIRE_IODELAY)
		priv->controller_flags |= OMAP_HSMMC_REQUIRE_IODELAY;

	ret = omap_hsmmc_get_pinctrl_state(mmc);
	if (ret < 0)
		return ret;
#endif

	upriv->mmc = mmc;

	return 0;
}

static const struct omap_mmc_of_data dra7_mmc_of_data = {
	.controller_flags = OMAP_HSMMC_REQUIRE_IODELAY,
};


static const struct udevice_id omap_hsmmc_ids[] = {
	{ .compatible = "ti,omap3-hsmmc" },
	{ .compatible = "ti,omap4-hsmmc" },
	{ .compatible = "ti,am33xx-hsmmc" },
	{ .compatible = "ti,dra7-hsmmc", .data = (ulong)&dra7_mmc_of_data },
	{ }
};

U_BOOT_DRIVER(omap_hsmmc) = {
	.name	= "omap_hsmmc",
	.id	= UCLASS_MMC,
	.of_match = omap_hsmmc_ids,
	.ofdata_to_platdata = omap_hsmmc_ofdata_to_platdata,
	.probe	= omap_hsmmc_probe,
	.priv_auto_alloc_size = sizeof(struct omap_hsmmc_data),
};
#endif
