/*
 * dfu.c -- dfu command
 *
 * Copyright (C) 2015
 * Lukasz Majewski <l.majewski@majess.pl>
 *
 * Copyright (C) 2012 Samsung Electronics
 * authors: Andrzej Pietrasiewicz <andrzej.p@samsung.com>
 *	    Lukasz Majewski <l.majewski@samsung.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <watchdog.h>
#include <dfu.h>
#include <console.h>
#include <g_dnl.h>
#include <usb.h>
#include <net.h>

int run_usb_dnl_gadget(int usbctrl_index, char *usb_dnl_gadget)
{
#ifndef CONFIG_SPL_DFU_SUPPORT
	bool dfu_reset = false;
#endif
	int ret, i = 0;

	board_usb_init(usbctrl_index, USB_INIT_DEVICE);
	g_dnl_clear_detach();
	g_dnl_register(usb_dnl_gadget);
	while (1) {
		if (g_dnl_detach()) {
			/*
			 * Check if USB bus reset is performed after detach,
			 * which indicates that -R switch has been passed to
			 * dfu-util. In this case reboot the device
			 */
			if (dfu_usb_get_reset()) {
#ifndef CONFIG_SPL_DFU_SUPPORT
				dfu_reset = true;
#endif
				goto exit;
			}

			/*
			 * This extra number of usb_gadget_handle_interrupts()
			 * calls is necessary to assure correct transmission
			 * completion with dfu-util
			 */
			if (++i == 10000)
				goto exit;
		}

		if (ctrlc())
			goto exit;

		if (dfu_get_defer_flush()) {
			/*
			 * Call to usb_gadget_handle_interrupts() is necessary
			 * to act on ZLP OUT transaction from HOST PC after
			 * transmitting the whole file.
			 *
			 * If this ZLP OUT packet is NAK'ed, the HOST libusb
			 * function fails after timeout (by default it is set to
			 * 5 seconds). In such situation the dfu-util program
			 * exits with error message.
			 */
			usb_gadget_handle_interrupts(usbctrl_index);
			ret = dfu_flush(dfu_get_defer_flush(), NULL, 0, 0);
			dfu_set_defer_flush(NULL);
			if (ret) {
				error("Deferred dfu_flush() failed!");
				goto exit;
			}
		}

		WATCHDOG_RESET();
		usb_gadget_handle_interrupts(usbctrl_index);
	}
exit:
	g_dnl_unregister();
	board_usb_cleanup(usbctrl_index, USB_INIT_DEVICE);

#ifndef CONFIG_SPL_DFU_SUPPORT
	if (dfu_reset)
		run_command("reset", 0);
#endif

	g_dnl_clear_detach();

	return ret;
}
