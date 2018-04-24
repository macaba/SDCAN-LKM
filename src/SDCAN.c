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

struct sdcan_priv {
	struct can_priv	   can;
	struct net_device *net;
	struct spi_device *spi;
	//enum sdcan_model model;

	struct mutex sdcan_lock; /* SPI device lock */

	u8 *spi_tx_buf;
	u8 *spi_rx_buf;
	dma_addr_t spi_tx_dma;
	dma_addr_t spi_rx_dma;

	struct sk_buff *tx_skb;
	int tx_len;

	struct workqueue_struct *wq;
	struct work_struct tx_work;
	struct work_struct restart_work;

// 	int force_quit;
// 	int after_suspend;
// #define AFTER_SUSPEND_UP 1
// #define AFTER_SUSPEND_DOWN 2
// #define AFTER_SUSPEND_POWER 4
// #define AFTER_SUSPEND_RESTART 8
// 	int restart_tx;
// 	struct regulator *power;
// 	struct regulator *transceiver;
// 	struct clk *clk;
};

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


