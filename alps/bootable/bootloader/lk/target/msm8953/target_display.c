/* Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


#include <debug.h>
#include <string.h>
#include <smem.h>
#include <err.h>
#include <msm_panel.h>
#include <mipi_dsi.h>
#include <pm8x41.h>
#include <pm8x41_wled.h>
#include <qpnp_wled.h>
#include <board.h>
#include <mdp5.h>
#include <scm.h>
#include <regulator.h>
#include <platform/clock.h>
#include <platform/gpio.h>
#include <platform/iomap.h>
#include <target/display.h>
#include <mipi_dsi_autopll_thulium.h>
#include <qtimer.h>
#include <platform.h>
//add by guchong
#include <platform/gpio.h>
#include <i2c_qup.h>
#include <blsp_qup.h>

#include "include/panel.h"
#include "include/display_resource.h"
#include "gcdb_display.h"

#define MAX_POLL_READS 15
#define POLL_TIMEOUT_US 1000
#define STRENGTH_SIZE_IN_BYTES	10
#define REGULATOR_SIZE_IN_BYTES	5
#define LANE_SIZE_IN_BYTES		20
/*---------------------------------------------------------------------------*/
/* GPIO configuration                                                        */
/*---------------------------------------------------------------------------*/
static struct gpio_pin reset_gpio = {
  "msmgpio", 61, 3, 1, 0, 1
};

#if defined(SIMCOM_LK_NANO)
//static struct gpio_pin lcdpw_gpio = {
//  "msmgpio", 0, 3, 1, 0, 1
//};
static struct gpio_pin bklt_gpio = {
  "pmi8952_gpios", 1, 3, 1, 0, 1
};

static struct gpio_pin enable_gpio = {
  "msmgpio", 48, 3, 1, 0, 1
};
#else
static struct gpio_pin bkl_gpio = {
  "msmgpio", 59, 3, 1, 0, 1
};

static struct gpio_pin enable_gpio = {
  "msmgpio", 12, 3, 1, 0, 1
};
#endif

#if defined(SIMCOM_LK_NANO)
static struct gpio_pin tc_vdd_pwr_enable_gpio = {
  "msmgpio", 19, 3, 1, 0, 1
};

static struct gpio_pin tc_vdds_gpio = {
  "msmgpio", 20, 3, 1, 0, 1
};

static struct gpio_pin tc_reset_gpio = {
  "msmgpio", 24, 3, 1, 0, 1
};

//static struct gpio_pin clk_gpio = {
//  "pm8953_gpios", 2, 3, 1, 0, 1
//};
#endif

#define VCO_DELAY_USEC 1000
#define GPIO_STATE_LOW 0
#define GPIO_STATE_HIGH 2
#define RESET_GPIO_SEQ_LEN 3
#define PMIC_WLED_SLAVE_ID 3
#if defined(SIMCOM_LK_NANO)
#define TC358762_I2C_ADDRESS		0x0B

struct tc358762_reg_table {
	const uint16_t reg;
	const uint16_t h_val;
	const uint16_t l_val;
};
//add by guchong
extern void gpio_tlmm_config(uint32_t gpio, uint8_t func,
			uint8_t dir, uint8_t pull,
			uint8_t drvstr, uint32_t enable);

static struct tc358762_reg_table tc358762_reg_t[]={
#if 1// sync event
	{0x047C, 0x0000, 0x0000},
	{0x0210, 0x0000, 0x0007},
	{0x0164, 0x0000, 0x0022},
	{0x0168, 0x0000, 0x0022},
	{0x0144, 0x0000, 0x0000},
	{0x0148, 0x0000, 0x0000},
	{0x0114, 0x0000, 0x0005},

	{0x0450, 0x0000, 0x0062},
	{0x0454, 0x0000, 0x0122},

	{0x0420, 0x0000, 0x0153},//LCD CTRL
	{0x0424, 0x0028, 0x0004},
	{0x0428, 0x00FD, 0x0280},
	{0x042c, 0x0015, 0x0001},
	{0x0430, 0x001E, 0x01E0},
	{0x0434, 0x0000, 0x0001},
	{0x0464, 0x0000, 0x040F},
	
	{0x0104, 0x0000, 0x0001},
	{0x0204, 0x0000, 0x0001},

	{0x0468, 0x0000, 0x0004},
	{0x0470, 0x3823, 0x0000},

#else //sync pulse
	{0x047C, 0x0000, 0x0000},
	{0x0210, 0x0000, 0x0007},
	{0x0164, 0x0000, 0x001F},
	{0x0168, 0x0000, 0x001F},
	{0x0144, 0x0000, 0x0000},
	{0x0148, 0x0000, 0x0000},
	{0x0114, 0x0000, 0x0005},

	{0x0450, 0x0000, 0x0062},
	{0x0454, 0x0000, 0x0122},

	{0x0420, 0x0000, 0x0151},//LCD CTRL
	{0x0464, 0x0000, 0x040F},
	
	{0x0104, 0x0000, 0x0001},
	{0x0204, 0x0000, 0x0001},

	{0x0468, 0x0000, 0x0004},
	{0x0470, 0x3823, 0x0000},
#endif
};
#endif

static uint32_t dsi_pll_lock_status(uint32_t pll_base, uint32_t off,
	uint32_t bit)
{
	uint32_t cnt, status;

	/* check pll lock first */
	for (cnt = 0; cnt < MAX_POLL_READS; cnt++) {
		status = readl(pll_base + off);
		dprintf(SPEW, "%s: pll_base=%x cnt=%d status=%x\n",
				__func__, pll_base, cnt, status);
		status &= BIT(bit); /* bit 5 */
		if (status)
			break;
		udelay(POLL_TIMEOUT_US);
	}

	return status;
}

static uint32_t dsi_pll_enable_seq(uint32_t phy_base, uint32_t pll_base)
{
	uint32_t pll_locked;

	writel(0x10, phy_base + 0x45c);
	writel(0x01, phy_base + 0x48);

	pll_locked = dsi_pll_lock_status(pll_base, 0xcc, 5);
	if (pll_locked)
		pll_locked = dsi_pll_lock_status(pll_base, 0xcc, 0);

	if (!pll_locked)
		dprintf(ERROR, "%s: DSI PLL lock failed\n", __func__);
	else
		dprintf(SPEW, "%s: DSI PLL lock Success\n", __func__);

	return  pll_locked;
}

static int wled_backlight_ctrl(uint8_t enable)
{
	uint8_t slave_id = PMIC_WLED_SLAVE_ID;	/* pmi */

	pm8x41_wled_config_slave_id(slave_id);
	qpnp_wled_enable_backlight(enable);
	qpnp_ibb_enable(enable);
	return NO_ERROR;
}

int target_backlight_ctrl(struct backlight *bl, uint8_t enable)
{
	uint32_t ret = NO_ERROR;
#if defined(SIMCOM_LK_NANO)
	uint8_t slave_id = 2;	/* gpio at pmi */

	if (bl->bl_interface_type == BL_DCS)
		return ret;

	//ret = wled_backlight_ctrl(enable);
	mdelay(100);
	pm8x41_gpio_set_sid(slave_id, bklt_gpio.pin_id, 1);
	pm8x41_pwm_config_enable(true);
#else
	if (bl->bl_interface_type == BL_DCS)
		return ret;

	ret = wled_backlight_ctrl(enable);
#endif

	return ret;
}


int target_panel_clock(uint8_t enable, struct msm_panel_info *pinfo)
{
	int32_t ret = 0, flags, dsi_phy_pll_out;
	struct dfps_pll_codes *pll_codes = &pinfo->mipi.pll_codes;
	struct mdss_dsi_pll_config *pll_data;
	dprintf(SPEW, "target_panel_clock\n");

	pll_data = pinfo->mipi.dsi_pll_config;

	if (pinfo->dest == DISPLAY_2) {
		flags = MMSS_DSI_CLKS_FLAG_DSI1;
		if (pinfo->mipi.dual_dsi)
			flags |= MMSS_DSI_CLKS_FLAG_DSI0;
	} else {
		flags = MMSS_DSI_CLKS_FLAG_DSI0;
		if (pinfo->mipi.dual_dsi)
			flags |= MMSS_DSI_CLKS_FLAG_DSI1;
	}

	if (enable) {
		mdp_gdsc_ctrl(enable);
		mdss_bus_clocks_enable();
		mdp_clock_enable();
		ret = restore_secure_cfg(SECURE_DEVICE_MDSS);
		if (ret) {
			dprintf(CRITICAL,
				"%s: Failed to restore MDP security configs",
				__func__);
			mdp_clock_disable();
			mdss_bus_clocks_disable();
			mdp_gdsc_ctrl(0);
			return ret;
		}

		mdss_dsi_auto_pll_thulium_config(pinfo);

		if (!dsi_pll_enable_seq(pinfo->mipi.phy_base,
			pinfo->mipi.pll_base)) {
			ret = ERROR;
			dprintf(CRITICAL, "PLL failed to lock!\n");
			mmss_dsi_clock_disable(flags);
			mdp_clock_disable();
			mdss_bus_clocks_disable();
			return ret;
		}

		pll_codes->codes[0] = readl_relaxed(pinfo->mipi.pll_base +
						MMSS_DSI_PHY_PLL_CORE_KVCO_CODE);
		pll_codes->codes[1] = readl_relaxed(pinfo->mipi.pll_base +
						MMSS_DSI_PHY_PLL_CORE_VCO_TUNE);
		dprintf(SPEW, "codes %d %d\n", pll_codes->codes[0],
						pll_codes->codes[1]);

		if (pinfo->mipi.use_dsi1_pll)
			dsi_phy_pll_out = DSI1_PHY_PLL_OUT;
		else
			dsi_phy_pll_out = DSI0_PHY_PLL_OUT;
		mmss_dsi_clock_enable(dsi_phy_pll_out, flags,
			pll_data->pclk_m, pll_data->pclk_n, pll_data->pclk_d);

	} else if(!target_cont_splash_screen()) {
		/* stop pll */
		writel(0x0, pinfo->mipi.phy_base + 0x48);

		mmss_dsi_clock_disable(flags);
		mdp_clock_disable();
		mdss_bus_clocks_disable();
		mdp_gdsc_ctrl(enable);
	}

	return 0;

}
#if defined(SIMCOM_LK_NANO)
static void tc358762_dsi_rgb_reset(uint8_t enable)
{
	//uint32_t gpio49 = 0;
	dprintf(CRITICAL, " tc358762_dsi_rgb_reset,enable %d\n", enable);
	gpio_tlmm_config(tc_vdd_pwr_enable_gpio.pin_id, 0,
				tc_vdd_pwr_enable_gpio.pin_direction, tc_vdd_pwr_enable_gpio.pin_pull,
				tc_vdd_pwr_enable_gpio.pin_strength, tc_vdd_pwr_enable_gpio.pin_state);

	gpio_tlmm_config(tc_vdds_gpio.pin_id, 0,
				tc_vdds_gpio.pin_direction, tc_vdds_gpio.pin_pull,
				tc_vdds_gpio.pin_strength, tc_vdds_gpio.pin_state);

/*	gpio_tlmm_config(tc_pwdn_gpio.pin_id, 0,
				tc_pwdn_gpio.pin_direction, tc_pwdn_gpio.pin_pull,
				tc_pwdn_gpio.pin_strength, tc_pwdn_gpio.pin_state);
*/
	gpio_tlmm_config(tc_reset_gpio.pin_id, 0,
				tc_reset_gpio.pin_direction, tc_reset_gpio.pin_pull,
				tc_reset_gpio.pin_strength, tc_reset_gpio.pin_state);

	if(enable) {
		gpio_set_dir(tc_vdd_pwr_enable_gpio.pin_id, 2);
		mdelay(10);
		gpio_set_dir(tc_vdds_gpio.pin_id, 2);
		gpio_set_dir(tc_reset_gpio.pin_id, 2);
		udelay(1);
		gpio_set_dir(tc_reset_gpio.pin_id, 0);
		udelay(1);
		gpio_set_dir(tc_reset_gpio.pin_id, 2);//2:GPIO_STATE_HIGH

	}

		//mdelay(2);
		//gpio49=gpio_status(49);
		//dprintf(CRITICAL, "gpio49  status  %d\n",gpio49);

}

static struct qup_i2c_dev  *i2c_dev;

static int tc358762_i2c_read(uint16_t addr)
//static int tc358762_i2c_read(uint8_t addr)
{
	int ret = 0;
	//uint8_t ret = 0;
	//int err;
	//uint8_t err = 0;
	uint8_t rbuf[4], wbuf[2] = { addr >> 8, addr & 0xff };
	/* Create a i2c_msg buffer, that is used to put the controller into read
	   mode and then to read some data. */
	struct i2c_msg msg_buf[] = {
		{TC358762_I2C_ADDRESS, I2C_M_WR, 2, wbuf},
		{TC358762_I2C_ADDRESS, I2C_M_RD, 4, rbuf}
	};
	//struct i2c_msg msg_buf[] = {
	//	{TC358762_I2C_ADDRESS, I2C_M_WR, 1, &addr},
	//	{TC358762_I2C_ADDRESS, I2C_M_RD, 1, &ret}
	//};

	ret = qup_i2c_xfer(i2c_dev, msg_buf, 2);
	//if(err < 0) {
	//	dprintf(CRITICAL, "qup_i2c_xfer ret %d\n", err);
	//	return err;
	//}
	//ret = i2c_transfer(client->adapter, msg, 2);
	//mdelay(2);

    //if (err == 2) { 
 	ret= (rbuf[3]<<24)|(rbuf[2]<<16) |(rbuf[1]<<8)|rbuf[0];
    //}
	return ret;
}

static int tc358762_i2c_write(uint16_t reg, uint16_t data_l, uint16_t data_h)
//int tc358762_i2c_write(uint8_t reg, uint8_t data_l, uint8_t data_h)
{
	int ret = 0;
	uint8_t cmd[6];
	    cmd[0] = reg >> 8;
	    cmd[1] = reg & 0xFF;
	    cmd[3] = data_l >> 8;
	    cmd[2] = data_l & 0xFF;
	    cmd[5] = data_h >> 8;
	    cmd[4] = data_h & 0xFF;

	/* Create a i2c_msg buffer, that is used to put the controller into write
	   mode and then to write some data. */
	struct i2c_msg msg_buf[] = { {TC358762_I2C_ADDRESS,
				      I2C_M_WR, 6, cmd}  //4
	};

	ret = qup_i2c_xfer(i2c_dev, msg_buf, 1);
	dprintf(CRITICAL, "qup_i2c_xfer lk write ret %d\n", ret);
	if(ret < 0) {
		dprintf(CRITICAL, "qup_i2c_xfer lk write error %d\n", ret);
		return ret;
	}
	return 0;
}

static int tc358762_config_reg(void)
{
	int ret = 0;
	//int index;
	uint16_t index;

	i2c_dev = qup_blsp_i2c_init(BLSP_ID_1, QUP_ID_1, 100000, 19200000);
	if(!i2c_dev) {
			dprintf(CRITICAL, "qup_blsp_i2c_init failed \n");
			ASSERT(0);
	}

	for(index = 0; index < ARRAY_SIZE(tc358762_reg_t); index++){
	//for(index = 0; index < 48; index++){
		ret = tc358762_i2c_write(tc358762_reg_t[index].reg, tc358762_reg_t[index].l_val, tc358762_reg_t[index].h_val);
		if(ret != 0){
			dprintf(CRITICAL, "write reg0x%x err:%d\n", tc358762_reg_t[index].reg, ret);
		}
		ret = tc358762_i2c_read(tc358762_reg_t[index].reg);
		dprintf(CRITICAL, "read reg0x%x ret:0x%x\n", tc358762_reg_t[index].reg, ret);
		mdelay(1);//soft reset need 1000 us
	}

	return ret;
}

struct LCM_setting_table {
    uint8_t para_list[4];
    uint8_t delay;
};


static struct LCM_setting_table lcm_init_setting[] = {
	{{0x05, 0x00, 0xFD, 0x00},0},
    		{{0x05, 0x00, 0x02, 0x00},100},
    	{{0x05, 0x00, 0xFD, 0xFF},0},
		{{0x05, 0x00, 0x81, 0x56},0},
		{{0x05, 0x00, 0x82, 0x01},0},
	{{0x05, 0x00, 0xFD, 0x00},0},
		{{0x05, 0x00, 0x17, 0x00},0},
		{{0x05, 0x00, 0x18, 0x00},0},
		{{0x05, 0x00, 0x45, 0x30},0},
		{{0x05, 0x00, 0x19, 0x96},0},
		{{0x05, 0x00, 0x1A, 0x2C},0},
		{{0x05, 0x00, 0x04, 0x41},0},
        //{{0x05, 0x00, 0x1A, 0x2C},0},
		//{{0x05, 0x00, 0x19, 0x95},0},
		//{{0x05, 0x00, 0x1A, 0x28},0},
		//{{0x05, 0x00, 0x04, 0x41},0},
	{{0x05, 0x00, 0xFD, 0xC5},0},
		{{0x05, 0x00, 0x82, 0x1C},0},
		{{0x05, 0x00, 0xA2, 0x9A},0},
	{{0x05, 0x00, 0xFD, 0xD8},0},
		{{0x05, 0x00, 0x01, 0x1B},0},
		{{0x05, 0x00, 0x02, 0x1B},0},
	{{0x05, 0x00, 0xFD, 0xC4},0},
		{{0x05, 0x00, 0x82, 0x05},0},
	{{0x05, 0x00, 0xFD, 0xC1},0},
		{{0x05, 0x00, 0x91, 0x43},0},
	{{0x05, 0x00, 0xFD, 0xC0},0},
		{{0x05, 0x00, 0xA1, 0x01},0},
		{{0x05, 0x00, 0xA2, 0x1C},0},
		{{0x05, 0x00, 0xA3, 0x0A},0},
		{{0x05, 0x00, 0xA4, 0x34},0},
		{{0x05, 0x00, 0xA5, 0x00},0},
		{{0x05, 0x00, 0xA6, 0x09},0},
		{{0x05, 0x00, 0xA7, 0x34},0},
		{{0x05, 0x00, 0xA8, 0x00},0},
		{{0x05, 0x00, 0xA9, 0x09},0},
		{{0x05, 0x00, 0xAA, 0x34},0},
		{{0x05, 0x00, 0xB1, 0x51},0},
	{{0x05, 0x00, 0xFD, 0xCE},0},
		{{0x05, 0x00, 0x81, 0x14},0},
		{{0x05, 0x00, 0x82, 0x3E},0},
		{{0x05, 0x00, 0x83, 0x3E},0},
		{{0x05, 0x00, 0x91, 0x05},0},
		{{0x05, 0x00, 0x93, 0x33},0},
		{{0x05, 0x00, 0x94, 0x02},0},
		{{0x05, 0x00, 0x95, 0x05},0},
		{{0x05, 0x00, 0x97, 0x33},0},
		{{0x05, 0x00, 0x98, 0x02},0},
		{{0x05, 0x00, 0x99, 0x05},0},
		{{0x05, 0x00, 0x9B, 0x33},0},
		{{0x05, 0x00, 0x9C, 0x02},0},
	//{{0x05, 0x00, 0xFD, 0xC5},0},
	//	{{0x05, 0x00, 0xD1, 0x94},0},
	//	{{0x05, 0x00, 0xD2, 0x42},0},
	//{{0x05, 0x00, 0xFD, 0xD8},0},
	//	{{0x05, 0x00, 0x01, 0x2F},0},
	//	{{0x05, 0x00, 0x02, 0x2F},0},
	{{0x05, 0x00, 0xFD, 0xE1},0},
		{{0x05, 0x00, 0x01, 0x7B},0},
		{{0x05, 0x00, 0x02, 0x06},0},
		{{0x05, 0x00, 0x03, 0x11},0},
		{{0x05, 0x00, 0x04, 0x1C},0},
		{{0x05, 0x00, 0x05, 0x0D},0},
		{{0x05, 0x00, 0x06, 0x0F},0},
		{{0x05, 0x00, 0x07, 0x1D},0},
		{{0x05, 0x00, 0x08, 0x07},0},
		{{0x05, 0x00, 0x09, 0x07},0},
		{{0x05, 0x00, 0x0A, 0x09},0},
		{{0x05, 0x00, 0x0B, 0x0B},0},
		{{0x05, 0x00, 0x0C, 0x18},0},
		{{0x05, 0x00, 0x0D, 0x11},0},
		{{0x05, 0x00, 0x0E, 0x10},0},
		{{0x05, 0x00, 0x0F, 0x14},0},
		{{0x05, 0x00, 0x10, 0x0C},0},
		{{0x05, 0x00, 0x11, 0x04},0},
		{{0x05, 0x00, 0x12, 0x11},0},
	{{0x05, 0x00, 0xFD, 0xE2},0},
		{{0x05, 0x00, 0x01, 0x7B},0},
		{{0x05, 0x00, 0x02, 0x06},0},
		{{0x05, 0x00, 0x03, 0x11},0},
		{{0x05, 0x00, 0x04, 0x1C},0},
		{{0x05, 0x00, 0x05, 0x0D},0},
		{{0x05, 0x00, 0x06, 0x0F},0},
		{{0x05, 0x00, 0x07, 0x1D},0},
		{{0x05, 0x00, 0x08, 0x07},0},
		{{0x05, 0x00, 0x09, 0x07},0},
		{{0x05, 0x00, 0x0A, 0x09},0},
		{{0x05, 0x00, 0x0B, 0x0B},0},
		{{0x05, 0x00, 0x0C, 0x18},0},
		{{0x05, 0x00, 0x0D, 0x11},0},
		{{0x05, 0x00, 0x0E, 0x10},0},
		{{0x05, 0x00, 0x0F, 0x14},0},
		{{0x05, 0x00, 0x10, 0x0C},0},
		{{0x05, 0x00, 0x11, 0x04},0},
		{{0x05, 0x00, 0x12, 0x11},0},
	{{0x05, 0x00, 0xFD, 0xE3},0},
		{{0x05, 0x00, 0x01, 0x7B},0},
		{{0x05, 0x00, 0x02, 0x06},0},
		{{0x05, 0x00, 0x03, 0x11},0},
		{{0x05, 0x00, 0x04, 0x1C},0},
		{{0x05, 0x00, 0x05, 0x0D},0},
		{{0x05, 0x00, 0x06, 0x0F},0},
		{{0x05, 0x00, 0x07, 0x1D},0},
		{{0x05, 0x00, 0x08, 0x07},0},
		{{0x05, 0x00, 0x09, 0x07},0},
		{{0x05, 0x00, 0x0A, 0x09},0},
		{{0x05, 0x00, 0x0B, 0x0B},0},
		{{0x05, 0x00, 0x0C, 0x18},0},
		{{0x05, 0x00, 0x0D, 0x11},0},
		{{0x05, 0x00, 0x0E, 0x10},0},
		{{0x05, 0x00, 0x0F, 0x14},0},
		{{0x05, 0x00, 0x10, 0x0C},0},
		{{0x05, 0x00, 0x11, 0x04},0},
		{{0x05, 0x00, 0x12, 0x11},0},
	{{0x05, 0x00, 0xFD, 0xE4},0},
		{{0x05, 0x00, 0x01, 0x7B},0},
		{{0x05, 0x00, 0x02, 0x06},0},
		{{0x05, 0x00, 0x03, 0x11},0},
		{{0x05, 0x00, 0x04, 0x1C},0},
		{{0x05, 0x00, 0x05, 0x0D},0},
		{{0x05, 0x00, 0x06, 0x0F},0},
		{{0x05, 0x00, 0x07, 0x1D},0},
		{{0x05, 0x00, 0x08, 0x07},0},
		{{0x05, 0x00, 0x09, 0x07},0},
		{{0x05, 0x00, 0x0A, 0x09},0},
		{{0x05, 0x00, 0x0B, 0x0B},0},
		{{0x05, 0x00, 0x0C, 0x18},0},
		{{0x05, 0x00, 0x0D, 0x11},0},
		{{0x05, 0x00, 0x0E, 0x10},0},
		{{0x05, 0x00, 0x0F, 0x14},0},
		{{0x05, 0x00, 0x10, 0x0C},0},
		{{0x05, 0x00, 0x11, 0x04},0},
		{{0x05, 0x00, 0x12, 0x11},0},
	{{0x05, 0x00, 0xFD, 0xE5},0},
		{{0x05, 0x00, 0x01, 0x7B},0},
		{{0x05, 0x00, 0x02, 0x06},0},
		{{0x05, 0x00, 0x03, 0x11},0},
		{{0x05, 0x00, 0x04, 0x1C},0},
		{{0x05, 0x00, 0x05, 0x0D},0},
		{{0x05, 0x00, 0x06, 0x0F},0},
		{{0x05, 0x00, 0x07, 0x1D},0},
		{{0x05, 0x00, 0x08, 0x07},0},
		{{0x05, 0x00, 0x09, 0x07},0},
		{{0x05, 0x00, 0x0A, 0x09},0},
		{{0x05, 0x00, 0x0B, 0x0B},0},
		{{0x05, 0x00, 0x0C, 0x18},0},
		{{0x05, 0x00, 0x0D, 0x11},0},
		{{0x05, 0x00, 0x0E, 0x10},0},
		{{0x05, 0x00, 0x0F, 0x14},0},
		{{0x05, 0x00, 0x10, 0x0C},0},
		{{0x05, 0x00, 0x11, 0x04},0},
		{{0x05, 0x00, 0x12, 0x11},0},
	{{0x05, 0x00, 0xFD, 0xE6},0},
		{{0x05, 0x00, 0x01, 0x7B},0},
		{{0x05, 0x00, 0x02, 0x06},0},
		{{0x05, 0x00, 0x03, 0x11},0},
		{{0x05, 0x00, 0x04, 0x1C},0},
		{{0x05, 0x00, 0x05, 0x0D},0},
		{{0x05, 0x00, 0x06, 0x0F},0},
		{{0x05, 0x00, 0x07, 0x1D},0},
		{{0x05, 0x00, 0x08, 0x07},0},
		{{0x05, 0x00, 0x09, 0x07},0},
		{{0x05, 0x00, 0x0A, 0x09},0},
		{{0x05, 0x00, 0x0B, 0x0B},0},
		{{0x05, 0x00, 0x0C, 0x18},0},
		{{0x05, 0x00, 0x0D, 0x11},0},
		{{0x05, 0x00, 0x0E, 0x10},0},
		{{0x05, 0x00, 0x0F, 0x14},0},
		{{0x05, 0x00, 0x10, 0x0C},0},
		{{0x05, 0x00, 0x11, 0x04},0},
		{{0x05, 0x00, 0x12, 0x11},0},
	{{0x05, 0x00, 0xFD, 0x00},0},
		{{0x05, 0x00, 0x01, 0x01},0},
	//{{0x05, 0x00, 0xFD, 0xC4},0},
		//{{0x05, 0x00, 0x82, 0x45},10},
	{{0x05, 0x00, 0xFD, 0x00},0},
	//	{{0x05, 0x00, 0x01, 0x01},10},/*01 cmd is sleep out*/
		//{{0x05, 0x00, 0x11, 0x00},30},
		//{{0x05, 0x00, 0x29, 0x00},10}, 
};

static int push_table(struct LCM_setting_table *table, uint16_t counts)
{
    int i;
	int ret;

    for (i = 0; i < counts; i++) {
                
		struct i2c_msg msg_buf[] = { {TC358762_I2C_ADDRESS, I2C_M_WR, ARRAY_SIZE(table[i].para_list), table[i].para_list}};
		ret = qup_i2c_xfer(i2c_dev, msg_buf, 1);
	    if(ret < 0){
			dprintf(CRITICAL,"%s: ret=%d\n",__func__, ret);
			return ret;
		}
		
	    if (table[i].delay>0)
	   	 mdelay(table[i].delay);
	 }
	return ret;
}



int tc358762_config_ota5601a(void)
{
	
	int ret = 0;
	ret=push_table(lcm_init_setting, sizeof(lcm_init_setting) / sizeof(struct LCM_setting_table));
	return ret;
}

int tc358762_dsi_rgb_ctl(uint8_t enable)
{
	//int ret = 0;
	//uint32_t ldo_num = ldo?;

	//regulator_enable(ldo_num);//ldo?
	tc358762_dsi_rgb_reset(enable);
	//dprintf(CRITICAL, "clk_gpio begin\n");
	//gpio_tlmm_config(clk_gpio.pin_id, 3,
     //                  clk_gpio.pin_direction, clk_gpio.pin_pull,
      //                 clk_gpio.pin_strength, clk_gpio.pin_state);
	//dprintf(CRITICAL, "clk_gpio end\n");
	//gpio_14_clock_enable();
	tc358762_config_reg();
	tc358762_config_ota5601a(); 
	mdelay(50);
	//return ret;
	return 0;
}
//add end
#endif

int target_panel_reset(uint8_t enable, struct panel_reset_sequence *resetseq,
						struct msm_panel_info *pinfo)
{
	int ret = NO_ERROR;
#if defined(SIMCOM_LK_NANO)
#else
	uint32_t hw_id = board_hardware_id();
#endif

	if (enable) {

		if (pinfo->mipi.use_enable_gpio) {
			gpio_tlmm_config(enable_gpio.pin_id, 0,
				enable_gpio.pin_direction, enable_gpio.pin_pull,
				enable_gpio.pin_strength,
				enable_gpio.pin_state);

			gpio_set_dir(enable_gpio.pin_id, 2);
		}

#if defined(SIMCOM_LK_NANO)
/*			gpio_tlmm_config(lcdpw_gpio.pin_id, 0,
						lcdpw_gpio.pin_direction, lcdpw_gpio.pin_pull,
						lcdpw_gpio.pin_strength, lcdpw_gpio.pin_state);
			//gpio_tlmm_config(0, 0,GPIO_OUTPUT,GPIO_NO_PULL,GPIO_8MA,GPIO_DISABLE);
			dprintf(INFO, "lcd_pw enable mid\n");
			gpio_set_dir(lcdpw_gpio.pin_id, 2);
			//gpio_set_dir(0, 2);
			dprintf(INFO, "lcd_pw enable end\n");*/
#else
		if (hw_id != HW_PLATFORM_QRD) {
			gpio_tlmm_config(bkl_gpio.pin_id, 0,
				bkl_gpio.pin_direction, bkl_gpio.pin_pull,
				bkl_gpio.pin_strength, bkl_gpio.pin_state);

			gpio_set_dir(bkl_gpio.pin_id, 2);
		}
#endif
		gpio_tlmm_config(reset_gpio.pin_id, 0,
				reset_gpio.pin_direction, reset_gpio.pin_pull,
				reset_gpio.pin_strength, reset_gpio.pin_state);

		gpio_set_dir(reset_gpio.pin_id, 2);

		/* reset */
		for (int i = 0; i < RESET_GPIO_SEQ_LEN; i++) {
			if (resetseq->pin_state[i] == GPIO_STATE_LOW)
				gpio_set_dir(reset_gpio.pin_id, GPIO_STATE_LOW);
			else
				gpio_set_dir(reset_gpio.pin_id, GPIO_STATE_HIGH);
			mdelay(resetseq->sleep[i]);
		}

	} else if(!target_cont_splash_screen()) {
		gpio_set_dir(reset_gpio.pin_id, 0);
		gpio_set_dir(enable_gpio.pin_id, 0);
	}

#if defined(SIMCOM_LK_NANO)
	tc358762_dsi_rgb_ctl(1);
#endif
	return ret;
}

static void wled_init(struct msm_panel_info *pinfo)
{
	struct qpnp_wled_config_data config = {0};
	struct labibb_desc *labibb;
	int display_type = 0;
	bool swire_control = 0;
	bool wled_avdd_control = 0;

	labibb = pinfo->labibb;

	if (labibb)
		display_type = labibb->amoled_panel;

	if (display_type) {
		swire_control = labibb->swire_control;
		wled_avdd_control = true;
	} else {
		swire_control = false;
		wled_avdd_control = false;
	}

	config.display_type = display_type;
	config.lab_init_volt = 4600000;	/* fixed, see pmi register */
	config.ibb_init_volt = 1400000;	/* fixed, see pmi register */
	config.lab_ibb_swire_control = swire_control;
	config.wled_avdd_control = wled_avdd_control;

	if (!swire_control) {
		if (labibb && labibb->force_config) {
			config.lab_min_volt = labibb->lab_min_volt;
			config.lab_max_volt = labibb->lab_max_volt;
			config.ibb_min_volt = labibb->ibb_min_volt;
			config.ibb_max_volt = labibb->ibb_max_volt;
			config.pwr_up_delay = labibb->pwr_up_delay;
			config.pwr_down_delay = labibb->pwr_down_delay;
			config.ibb_discharge_en = labibb->ibb_discharge_en;
		} else {
			/* default */
			config.pwr_up_delay = 3;
			config.pwr_down_delay =  3;
			config.ibb_discharge_en = 1;
			if (display_type) {	/* amoled */
				config.lab_min_volt = 4600000;
				config.lab_max_volt = 4600000;
				config.ibb_min_volt = 4000000;
				config.ibb_max_volt = 4000000;
			} else { /* lcd */
				config.lab_min_volt = 5500000;
				config.lab_max_volt = 5500000;
				config.ibb_min_volt = 5500000;
				config.ibb_max_volt = 5500000;
			}
		}
	}

	dprintf(SPEW, "%s: %d %d %d %d %d %d %d %d %d %d\n", __func__,
		config.display_type,
		config.lab_min_volt, config.lab_max_volt,
		config.ibb_min_volt, config.ibb_max_volt,
		config.lab_init_volt, config.ibb_init_volt,
		config.pwr_up_delay, config.pwr_down_delay,
		config.ibb_discharge_en);

	/* QPNP WLED init for display backlight */
	pm8x41_wled_config_slave_id(PMIC_WLED_SLAVE_ID);

	qpnp_wled_init(&config);
}

int target_dsi_phy_config(struct mdss_dsi_phy_ctrl *phy_db)
{
	memcpy(phy_db->strength, panel_strength_ctrl, STRENGTH_SIZE_IN_BYTES *
		sizeof(uint32_t));
	memcpy(phy_db->regulator, panel_regulator_settings,
		REGULATOR_SIZE_IN_BYTES * sizeof(uint32_t));
	memcpy(phy_db->laneCfg, panel_lane_config, LANE_SIZE_IN_BYTES);
	return NO_ERROR;
}


int target_display_get_base_offset(uint32_t base)
{
	return 0;
}

int target_ldo_ctrl(uint8_t enable, struct msm_panel_info *pinfo)
{
	uint32_t ldo_num = REG_LDO6 | REG_LDO3 | REG_SMPS3;
	uint32_t hw_id = board_hardware_id();

	if (enable) {
#if defined(SIMCOM_LK_NANO)
		ldo_num |= REG_LDO17;
		regulator_enable(ldo_num);
		mdelay(10);
#else
		if (hw_id != HW_PLATFORM_QRD)
			ldo_num |= REG_LDO17;

		regulator_enable(ldo_num);
		mdelay(10);
		wled_init(pinfo);
		qpnp_ibb_enable(true); /*5V boost*/
		mdelay(50);
#endif
	} else {
		/*
		 * LDO6, LDO3 and SMPS3 are shared with other subsystems.
		 * Do not disable them.
		 */
		if (hw_id != HW_PLATFORM_QRD)
			regulator_disable(REG_LDO17);
	}

	return NO_ERROR;
}

bool target_display_panel_node(char *pbuf, uint16_t buf_size)
{
	return gcdb_display_cmdline_arg(pbuf, buf_size);
}

void target_display_init(const char *panel_name)
{
	struct oem_panel_data oem;
	int32_t ret = 0;
	uint32_t panel_loop = 0;

	set_panel_cmd_string(panel_name);
	oem = mdss_dsi_get_oem_data();

	if (!strcmp(oem.panel, NO_PANEL_CONFIG)
		|| !strcmp(oem.panel, SIM_VIDEO_PANEL)
		|| !strcmp(oem.panel, SIM_DUALDSI_VIDEO_PANEL)
		|| !strcmp(oem.panel, SIM_CMD_PANEL)
		|| !strcmp(oem.panel, SIM_DUALDSI_CMD_PANEL)
		|| oem.skip) {
		dprintf(INFO, "Selected panel: %s\nSkip panel configuration\n",
			oem.panel);
		oem.cont_splash = false;
	}

	do {
		target_force_cont_splash_disable(false);
		ret = gcdb_display_init(oem.panel, MDP_REV_50, (void *)MIPI_FB_ADDR);
		if (!ret || ret == ERR_NOT_SUPPORTED) {
			break;
		} else {
			target_force_cont_splash_disable(true);
			msm_display_off();
		}
	} while (++panel_loop <= oem_panel_max_auto_detect_panels());

	if (!oem.cont_splash) {
		dprintf(INFO, "Forcing continuous splash disable\n");
		target_force_cont_splash_disable(true);
	}
}

void target_display_shutdown(void)
{
	gcdb_display_shutdown();
}
