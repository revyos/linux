/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_FIRMWARE_THEAD_AON_REBOOT_H
#define _LINUX_FIRMWARE_THEAD_AON_REBOOT_H

/**
 * struct thead_aon_reboot_data - transport for the AON WDG service
 * @call_rpc: Send a 28-byte WDG request in sleepable context. The transport
 *		owns protocol version, acknowledgment policy and error handling.
 * @context: Parent-owned transport context, valid for the child lifetime.
 */
struct thead_aon_reboot_data {
	int (*call_rpc)(void *context, void *msg);
	void *context;
};

#endif
