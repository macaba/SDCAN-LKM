#include <linux/init.h>             // Macros used to mark up functions e.g., __init __exit
#include <linux/module.h>           // Core header for loading LKMs into the kernel
#include <linux/kernel.h>           // Contains types, macros, functions for the kernel
#include <linux/netdevice.h>
#include <linux/can/core.h>
#include <linux/can/dev.h>
#include <linux/can/led.h>

MODULE_LICENSE("GPL");              ///< The license type -- this affects runtime behavior
MODULE_AUTHOR("");                  ///< The author -- visible when you use modinfo
MODULE_DESCRIPTION("SDCAN - Software Defined CAN");  ///< The description -- see modinfo
MODULE_VERSION("0.1");              ///< The version of the module

static char *name = "world";        ///< An example LKM argument -- default value is "world"
module_param(name, charp, S_IRUGO); ///< Param desc. charp = char ptr, S_IRUGO can be read/not changed
MODULE_PARM_DESC(name, "The name to display in /var/log/kern.log");  ///< parameter description

static int sdcan_open(struct net_device *net){
	return 0;
}

static int sdcan_stop(struct net_device *net){
	return 0;
}

static netdev_tx_t sdcan_hard_start_xmit(struct sk_buff *skb, struct net_device *net){
	return NETDEV_TX_OK;	
}

static const struct net_device_ops sdcan_netdev_ops = {
	.ndo_open = sdcan_open,
	.ndo_stop = sdcan_stop,
	.ndo_start_xmit = sdcan_hard_start_xmit,
	.ndo_change_mtu = can_change_mtu
};


