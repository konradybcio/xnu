/*
 * Copyright 2013, winocm. <winocm@icloud.com>
 * Copyright 2018, Bingxing Wang. <i@imbushuo.net>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 *   If you are going to use this software in any form that does not involve
 *   releasing the source to this project or improving it, let me know beforehand.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
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
 */

#if defined(BOARD_CONFIG_MSM8974_RM1045)

#include <mach/mach_types.h>

#include <IOKit/IOPlatformExpert.h>

#include <pexpert/pexpert.h>
#include <pexpert/arm/protos.h>
#include <pexpert/arm/boot.h>

#include <machine/machine_routines.h>

#include <vm/pmap.h>
#include <arm/pmap.h>

#include "pe_msm8974_uefi.h"

#define KPRINTF_PREFIX  "PE_MSM8974: "

/* BLSP UART */
vm_offset_t         gMsmBlspUartBase;

/* GIC */
vm_offset_t         gMsmQGICCPUBase;
vm_offset_t         gMsmQGICDistributerBase;

/* Timer */
vm_offset_t         gMsmQTimerBase;
static uint64_t     clock_decrementer = 0;
static boolean_t    clock_initialized = FALSE;
static boolean_t    clock_had_irq = FALSE;
static uint64_t     clock_absolute_time = 0;
static uint32_t     ticks_per_sec = 19200000;

/* Functions */
void msm8974_uart_dm_putc(int c);
int msm8974_uart_dm_getc(void);

void udelay(unsigned usecs);
extern void rtc_configure(uint64_t hz);

/*
 * Stub for printing out to framebuffer.
 */
void vcputc(__unused int l, __unused int u, int c);

/*
 * Qualcomm UART_DM implementation
 * 
 * Assuming we are writing on the first MSM UART_DM port,
 * on Lumia 930, There are three UART test points between 
 * the edge and screen panel. For more information, search
 * "Lumia 930 schematics" on the Internet.
 * 
 * On other devices like Sony phones, UART points are 
 * documented in Sony Developer Website.
 */
void msm8974_uart_dm_putc(int c)
{
    /* Dependency: Do not run UART before clock initialization */
    if (!clock_initialized) return;
    
    assert(gMsmBlspUartBase);

    /* Check if transmit FIFO is empty.
	 * If not we'll wait for TX_READY interrupt. */
	if (!(readl(MSM_BOOT_UART_DM_SR(gMsmBlspUartBase)) & MSM_BOOT_UART_DM_SR_TXEMT)) {
		while (!(readl(MSM_BOOT_UART_DM_ISR(gMsmBlspUartBase)) & MSM_BOOT_UART_DM_TX_READY)) {
			udelay(1);
			/* Kick watchdog? */
		}
	}

    // We need to make sure the DM_NO_CHARS_FOR_TX&DM_TF are are programmed atmoically.
    ml_set_interrupts_enabled(FALSE);

    /* We are here. FIFO is ready to be written. */
	/* Write number of characters to be written */
	writel(1, MSM_BOOT_UART_DM_NO_CHARS_FOR_TX(gMsmBlspUartBase));

    /* Clear TX_READY interrupt */
	writel(MSM_BOOT_UART_DM_GCMD_RES_TX_RDY_INT, MSM_BOOT_UART_DM_CR(gMsmBlspUartBase));

    /* We use one-character word FIFO. So we need to divide data into
	 * one characters and write in UART_DM_TF register */

    /* Wait till TX FIFO has space */
    while (!(readl(MSM_BOOT_UART_DM_SR(gMsmBlspUartBase)) & MSM_BOOT_UART_DM_SR_TXRDY)) 
    {
        udelay(1);
    }

    /* TX FIFO has space. Write the chars */
    writel(c, MSM_BOOT_UART_DM_TF(gMsmBlspUartBase, 0));

    ml_set_interrupts_enabled(TRUE);
    return;
}

int msm8974_uart_dm_getc(void)
{
    // Not implement it for bringup
    return 'A';
}

/*
 * Initialize UART at UART1.
 * uart_dm_init(1, 0, BLSP1_UART1_BASE);
 */
void msm8974_uart_dm_init(void)
{
    /* UART_DM is initialized by LK. Map base address. */
    gMsmBlspUartBase = ml_io_map(BLSP1_UART1_BASE, PAGE_SIZE);

    return;
}

/* Timer */
static void timer_configure(void)
{
    // Map QTimer
    gMsmQTimerBase = ml_io_map(QTMR_BASE, PAGE_SIZE);

    // qtimer_init @ 19.2MHz
    uint64_t hz = 19200000;
    gPEClockFrequencyInfo.timebase_frequency_hz = hz;

    clock_decrementer = 1000;
    kprintf(KPRINTF_PREFIX "decrementer frequency = %llu\n", clock_decrementer);

    rtc_configure(hz);
    return;
}

void msm8974_timer_enabled(int enable)
{
    uint32_t ctrl;

    assert(gMsmQTimerBase);
    ctrl = readl(gMsmQTimerBase + QTMR_V1_CNTP_CTL);

    if (enable)
    {
        /* Program CTRL Register */
        ctrl |= QTMR_TIMER_CTRL_ENABLE;
        ctrl &= ~QTMR_TIMER_CTRL_INT_MASK;
    }
    else
    {
        /* program cntrl register */
        ctrl &= ~QTMR_TIMER_CTRL_ENABLE;
        ctrl |= QTMR_TIMER_CTRL_INT_MASK;
    }

    writel(ctrl, gMsmQTimerBase + QTMR_V1_CNTP_CTL);
    barrier();

    return;
}

uint32_t qtimer_tick_rate()
{
	return ticks_per_sec;
}

void msm8974_timebase_init(void)
{
    uint32_t tick_count;

    // Set RTC Clock, map QTimer.
    timer_configure();

    // Disable QTimer.
    msm8974_timer_enabled(FALSE);

    // Save the timer interval and call back data
    tick_count = clock_decrementer * qtimer_tick_rate() / 1000;
    writel(tick_count, gMsmQTimerBase + QTMR_V1_CNTP_TVAL);
    barrier();

    // Unmask interrupt. (INT_QTMR_FRM_0_PHYSICAL_TIMER_EXP)
    uint32_t reg = GIC_DIST_ENABLE_SET + (INT_QTMR_FRM_0_PHYSICAL_TIMER_EXP / 32) * 4;
	uint32_t bit = 1 << (INT_QTMR_FRM_0_PHYSICAL_TIMER_EXP & 31);
	writel(bit, gMsmQGICDistributerBase + reg);

    // Enable interrupts.
    ml_set_interrupts_enabled(TRUE);

    // Enable QTimer.
    msm8974_timer_enabled(TRUE);

    // Set flag. UART is available now.
    clock_initialized = TRUE;
    
    return;
}

uint64_t msm8974_timer_value(void)
{
    /* Don't bother. (Why?) */
    uint64_t ret = 0;
    return ret;
}

uint64_t msm8974_get_timebase(void)
{
    uint32_t timestamp;

    if (!clock_initialized) return 0;

    timestamp = msm8974_timer_value();
    if (timestamp) 
    {
        uint64_t v = clock_absolute_time;
        v += (uint64_t) (((uint64_t) clock_decrementer) - (uint64_t) (timestamp));
        return v;
    } 
    else 
    {
        clock_absolute_time += clock_decrementer;
        return clock_absolute_time;
    }
}

inline __ALWAYS_INLINE uint64_t qtimer_get_phy_timer_cnt()
{
	uint32_t phy_cnt_lo;
	uint32_t phy_cnt_hi_1;
	uint32_t phy_cnt_hi_2;

	do {
		phy_cnt_hi_1 = readl(gMsmQTimerBase + QTMR_V1_CNTPCT_HI);
		phy_cnt_lo = readl(gMsmQTimerBase + QTMR_V1_CNTPCT_LO);
		phy_cnt_hi_2 = readl(gMsmQTimerBase + QTMR_V1_CNTPCT_HI);
    } while (phy_cnt_hi_1 != phy_cnt_hi_2);

	return ((uint64_t) phy_cnt_hi_1 << 32) | phy_cnt_lo;
}

/* Blocking function to wait until the specified ticks of the timer.
 * Note: ticks to wait for cannot be more than 56 bit.
 *          Should be sufficient for all practical purposes.
 */
static void delay(uint64_t ticks)
{
	volatile uint64_t cnt;
	uint64_t init_cnt;
	uint64_t timeout = 0;

	cnt = qtimer_get_phy_timer_cnt();
	init_cnt = cnt;

	/* Calculate timeout = cnt + ticks (mod 2^56)
	 * to account for timer counter wrapping
	 */
	timeout = (cnt + ticks) & (uint64_t)(QTMR_PHY_CNT_MAX_VALUE);

	/* Wait out till the counter wrapping occurs
	 * in cases where there is a wrapping.
	 */
	while(timeout < cnt && init_cnt <= cnt)
		/* read global counter */
		cnt = qtimer_get_phy_timer_cnt();

	/* Wait till the number of ticks is reached*/
	while(timeout > cnt)
		/* read global counter */
		cnt = qtimer_get_phy_timer_cnt();

}

void udelay(unsigned usecs)
{
	uint64_t ticks;

	ticks = ((uint64_t) usecs * ticks_per_sec) / 1000000;

	delay(ticks);
}

/* Interrupt Routine */
static uint8_t qgic_get_cpumask()
{
	uint32_t mask=0, i;

	/* Fetch the CPU MASK from the SGI/PPI reg */
	for (i=0; i < 32; i += 4) {
		mask = readl(gMsmQGICDistributerBase + GIC_DIST_TARGET + i);
		mask |= mask >> 16;
		mask |= mask >> 8;
		if (mask)
			break;
	}
	return mask;
}

/* Intialize distributor */
static void qgic_dist_config(uint32_t num_irq)
{
	uint32_t i;

	/* Set each interrupt line to use N-N software model
	 * and edge sensitive, active high
	 */
	for (i = 32; i < num_irq; i += 16)
		writel(0xffffffff, gMsmQGICDistributerBase + (GIC_DIST_CONFIG + i * 4 / 16));

	writel(0xffffffff, gMsmQGICDistributerBase + GIC_DIST_CONFIG + 4);

	/* Set priority of all interrupts */

	/*
	 * In bootloader we dont care about priority so
	 * setting up equal priorities for all
	 */
	for (i = 0; i < num_irq; i += 4)
		writel(0xa0a0a0a0, gMsmQGICDistributerBase + (GIC_DIST_PRI + i * 4 / 4));

	/* Disabling interrupts */
	for (i = 0; i < num_irq; i += 32)
		writel(0xffffffff, gMsmQGICDistributerBase + (GIC_DIST_ENABLE_CLEAR + i * 4 / 32));

	writel(0x0000ffff, gMsmQGICDistributerBase + GIC_DIST_ENABLE_SET);
}

static void qgic_dist_init(void)
{
    uint32_t i;
    uint32_t num_irq = 0;
    uint32_t cpumask = 1;

    cpumask = qgic_get_cpumask();

    cpumask |= cpumask << 8;
    cpumask |= cpumask << 16;

    /* Disabling GIC */
	writel(0, gMsmQGICDistributerBase + GIC_DIST_CTRL);

    /*
	 * Find out how many interrupts are supported.
	 */
	num_irq = readl(gMsmQGICDistributerBase + GIC_DIST_CTR) & 0x1f;
	num_irq = (num_irq + 1) * 32;

    /* Set up interrupts for this CPU */
	for (i = 32; i < num_irq; i += 4)
		writel(cpumask, gMsmQGICDistributerBase + GIC_DIST_TARGET + i * 4 / 4);

    qgic_dist_config(num_irq);

	/* Enabling GIC */
	writel(1, gMsmQGICDistributerBase + GIC_DIST_CTRL);
}

static void qgic_cpu_init(void)
{
    writel(0xf0, gMsmQGICCPUBase + GIC_CPU_PRIMASK);
	writel(1, gMsmQGICCPUBase + GIC_CPU_CTRL);

    return;
}

void msm8974_interrupt_init(void)
{
    assert(gMsmQGICCPUBase && gMsmQGICDistributerBase);

    /* Initialize QGIC. Called from platform specific init code */
    qgic_dist_init();
	qgic_cpu_init();

    return;
}

void msm8974_handle_interrupt(void *context)
{
    uint32_t irq_no = readl(gMsmQGICCPUBase + GIC_CPU_INTACK);
    if(irq_no > NR_IRQS) 
    {
        kprintf(KPRINTF_PREFIX "Got a bogus IRQ?");
        return;
    }

    /* Timer interrupt? */
    if(irq_no == INT_QTMR_FRM_0_PHYSICAL_TIMER_EXP) 
    {
        msm8974_timer_enabled(FALSE);
        clock_absolute_time += (clock_decrementer - (int64_t) msm8974_timer_value());
        rtclock_intr((arm_saved_state_t *) context);
        msm8974_timer_enabled(TRUE);
        clock_had_irq = TRUE;
    } 
    else 
    {
        irq_iokit_dispatch(irq_no);
    }

    /* EOI */
    writel(irq_no, gMsmQGICCPUBase + GIC_CPU_EOI);

    return;
}

/* Framebuffer */
static void _fb_putc(int c)
{
    if (c == '\n') {
        vcputc(0, 0, '\r');
    }
    vcputc(0, 0, c);
}

/* Initialize UEFI GOP FB */
void msm8974_uefi_framebuffer_init(void)
{
    char tempbuf[16]; 

    uint32_t lcd_width = RM1045_UEFI_PANEL_WIDTH;
    uint32_t lcd_height = RM1045_UEFI_PANEL_HEIGHT;

    PE_state.video.v_baseAddr = (unsigned long) MSM_UEFI_FRAMEBUFFER_BASE;
    PE_state.video.v_rowBytes = lcd_width * 4;
    PE_state.video.v_width = lcd_width;
    PE_state.video.v_height = lcd_height;
    PE_state.video.v_depth = 4 * (8);   // 32bpp (UEFI BGRA)

    kprintf(KPRINTF_PREFIX "UEFI framebuffer initialized\n");

    /*
     * Enable early framebuffer.
     */
    if (PE_parse_boot_argn("-early-fb-debug", tempbuf, sizeof(tempbuf))) {
        initialize_screen((void *) &PE_state.video, kPEAcquireScreen);
    }

    if (PE_parse_boot_argn("-graphics-mode", tempbuf, sizeof(tempbuf))) {
        initialize_screen((void *) &PE_state.video, kPEGraphicsMode);
    } else {
        initialize_screen((void *) &PE_state.video, kPETextMode);
    }
    return;
}

/* Map to SPMI restart function */
int msm8974_spmi_halt_restart(int type)
{
    return 0;
}

void msm8974_mapping_init(void)
{
    gMsmQGICCPUBase = ml_io_map(MSM_GIC_CPU_BASE, PAGE_SIZE);
    gMsmQGICDistributerBase = ml_io_map(MSM_GIC_DIST_BASE, PAGE_SIZE);

    return;
}

void PE_init_SocSupport_msm8974_rm1045(void)
{
    msm8974_mapping_init();

    gPESocDispatch.uart_getc = msm8974_uart_dm_getc;
    gPESocDispatch.uart_putc = msm8974_uart_dm_putc;
    gPESocDispatch.uart_init = msm8974_uart_dm_init;

    gPESocDispatch.interrupt_init = msm8974_interrupt_init;
    gPESocDispatch.handle_interrupt = msm8974_handle_interrupt;

    gPESocDispatch.timebase_init = msm8974_timebase_init;
    gPESocDispatch.get_timebase = msm8974_get_timebase;

    gPESocDispatch.timer_value = msm8974_timer_value;
    gPESocDispatch.timer_enabled = msm8974_timer_enabled;

    gPESocDispatch.framebuffer_init = msm8974_uefi_framebuffer_init;

    msm8974_uart_dm_init();
    msm8974_uefi_framebuffer_init();
    
    PE_halt_restart = msm8974_spmi_halt_restart;
}

void PE_init_SocSupport_stub(void)
{
    PE_early_puts("PE_init_SocSupport: Initializing for Qualcomm MSM8974\n");
    PE_init_SocSupport_msm8974_rm1045();
}

#endif /* !BOARD_CONFIG_MSM8974_RM1045 */