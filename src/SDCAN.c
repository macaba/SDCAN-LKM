#include <linux/init.h>             // Macros used to mark up functions e.g., __init __exit
#include <linux/module.h>           // Core header for loading LKMs into the kernel
#include <linux/kernel.h>           // Contains types, macros, functions for the kernel
#include <linux/netdevice.h>
#include <linux/can/core.h>
#include <linux/can/dev.h>
#include <linux/can/led.h>
#include <linux/spi/spi.h>

MODULE_LICENSE("GPL");              ///< The license type -- this affects runtime behavior
MODULE_AUTHOR("");                  ///< The author -- visible when you use modinfo
MODULE_DESCRIPTION("SDCAN - Software Defined CAN");  ///< The description -- see modinfo
MODULE_VERSION("0.1");              ///< The version of the module

static int sdcan_enable_dma; /* Enable SPI DMA. Default: 0 (Off) */
module_param(sdcan_enable_dma, int, 0444);
MODULE_PARM_DESC(sdcan_enable_dma, "Enable SPI DMA. Default: 0 (Off)");

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

static int sdcan_spi_trans(struct spi_device *spi, int len)
{
	struct sdcan_priv *priv = spi_get_drvdata(spi);
	struct spi_transfer t = {
		.tx_buf = priv->spi_tx_buf,
		.rx_buf = priv->spi_rx_buf,
		.len = len,
		.cs_change = 0,
	};
	struct spi_message m;
	int ret;

	spi_message_init(&m);

	if (sdcan_enable_dma) {
		t.tx_dma = priv->spi_tx_dma;
		t.rx_dma = priv->spi_rx_dma;
		m.is_dma_mapped = 1;
	}

	spi_message_add_tail(&t, &m);

	ret = spi_sync(spi, &m);
	if (ret)
		dev_err(&spi->dev, "spi transfer failed: ret = %d\n", ret);
	return ret;
}

static u8 sdcan_read_reg(struct spi_device *spi, uint8_t reg)
{
	struct sdcan_priv *priv = spi_get_drvdata(spi);
	u8 val = 0;

	//priv->spi_tx_buf[0] = INSTRUCTION_READ;
	priv->spi_tx_buf[1] = reg;

	sdcan_spi_trans(spi, 3);
	val = priv->spi_rx_buf[2];

	return val;
}

static void sdcan_write_reg(struct spi_device *spi, u8 reg, uint8_t val)
{
	struct sdcan_priv *priv = spi_get_drvdata(spi);

	//priv->spi_tx_buf[0] = INSTRUCTION_WRITE;
	priv->spi_tx_buf[1] = reg;
	priv->spi_tx_buf[2] = val;

	sdcan_spi_trans(spi, 3);
}

static void sdcan_hw_tx_frame(struct spi_device *spi, u8 *buf,
				int len, int tx_buf_idx)
{
	struct sdcan_priv *priv = spi_get_drvdata(spi);

	memcpy(priv->spi_tx_buf, buf, len);
	sdcan_spi_trans(spi, len);
}

// static void sdcan_hw_tx(struct spi_device *spi, struct can_frame *frame,
// 			  int tx_buf_idx)
// {
// 	struct sdcan_priv *priv = spi_get_drvdata(spi);
// 	u32 sid, eid, exide, rtr;
// 	u8 buf[SPI_TRANSFER_BUF_LEN];

// 	exide = (frame->can_id & CAN_EFF_FLAG) ? 1 : 0; /* Extended ID Enable */
// 	if (exide)
// 		sid = (frame->can_id & CAN_EFF_MASK) >> 18;
// 	else
// 		sid = frame->can_id & CAN_SFF_MASK; /* Standard ID */
// 	eid = frame->can_id & CAN_EFF_MASK; /* Extended ID */
// 	rtr = (frame->can_id & CAN_RTR_FLAG) ? 1 : 0; /* Remote transmission */

// 	//buf[TXBCTRL_OFF] = INSTRUCTION_LOAD_TXB(tx_buf_idx);
// 	buf[TXBSIDH_OFF] = sid >> SIDH_SHIFT;
// 	buf[TXBSIDL_OFF] = ((sid & SIDL_SID_MASK) << SIDL_SID_SHIFT) |
// 		(exide << SIDL_EXIDE_SHIFT) |
// 		((eid >> SIDL_EID_SHIFT) & SIDL_EID_MASK);
// 	buf[TXBEID8_OFF] = GET_BYTE(eid, 1);
// 	buf[TXBEID0_OFF] = GET_BYTE(eid, 0);
// 	buf[TXBDLC_OFF] = (rtr << DLC_RTR_SHIFT) | frame->can_dlc;
// 	memcpy(buf + TXBDAT_OFF, frame->data, frame->can_dlc);
// 	sdcan_hw_tx_frame(spi, buf, frame->can_dlc, tx_buf_idx);
// }

static int sdcan_open(struct net_device *net){
	printk(KERN_INFO "SDCAN Open\n");
	return 0;
}

static int sdcan_stop(struct net_device *net){
	printk(KERN_INFO "SDCAN Stop\n");
	return 0;
}

static netdev_tx_t sdcan_hard_start_xmit(struct sk_buff *skb, struct net_device *net){
	printk(KERN_INFO "SDCAN Hard Start Transmit\n");
	return NETDEV_TX_OK;	
}

static const struct net_device_ops sdcan_netdev_ops = {
	.ndo_open = sdcan_open,
	.ndo_stop = sdcan_stop,
	.ndo_start_xmit = sdcan_hard_start_xmit,
	.ndo_change_mtu = can_change_mtu
};


