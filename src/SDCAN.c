#include <linux/can/core.h>
#include <linux/can/dev.h>
#include <linux/can/led.h>
//#include <linux/can/platform/mcp251x.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/freezer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>

#define DEVICE_NAME "sdcan"

#define CAN_FRAME_MAX_DATA_LEN	8
#define SPI_TRANSFER_BUF_LEN	(6 + CAN_FRAME_MAX_DATA_LEN)

#define TX_ECHO_SKB_MAX	1

MODULE_LICENSE("GPL");              ///< The license type -- this affects runtime behavior
MODULE_AUTHOR("");                  ///< The author -- visible when you use modinfo
MODULE_DESCRIPTION("SDCAN - Software Defined CAN");  ///< The description -- see modinfo
MODULE_VERSION("0.1");              ///< The version of the module

static int sdcan_enable_dma; /* Enable SPI DMA. Default: 0 (Off) */
module_param(sdcan_enable_dma, int, 0444);
MODULE_PARM_DESC(sdcan_enable_dma, "Enable SPI DMA. Default: 0 (Off)");

static const struct can_bittiming_const sdcan_bittiming_const = {
	.name = DEVICE_NAME,
	.tseg1_min = 3,
	.tseg1_max = 16,
	.tseg2_min = 2,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 64,
	.brp_inc = 1,
};

enum sdcan_model {
	CAN_sdcan_1	= 0x0001
};

struct sdcan_priv {
	struct can_priv	   can;
	struct net_device *net;
	struct spi_device *spi;
	enum sdcan_model model;

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

	int force_quit;
	int after_suspend;
#define AFTER_SUSPEND_UP 1
#define AFTER_SUSPEND_DOWN 2
#define AFTER_SUSPEND_POWER 4
#define AFTER_SUSPEND_RESTART 8
	int restart_tx;
	struct regulator *power;
	struct regulator *transceiver;
	struct clk *clk;
};

static void sdcan_clean(struct net_device *net)
{
	struct sdcan_priv *priv = netdev_priv(net);

	if (priv->tx_skb || priv->tx_len)
		net->stats.tx_errors++;
	if (priv->tx_skb)
		dev_kfree_skb(priv->tx_skb);
	if (priv->tx_len)
		can_free_echo_skb(priv->net, 0);
	priv->tx_skb = NULL;
	priv->tx_len = 0;
}

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

static int sdcan_hw_probe(struct spi_device *spi)
{
	/* Check for power up default value */
	return 0;
}

static int sdcan_power_enable(struct regulator *reg, int enable)
{
	if (IS_ERR_OR_NULL(reg))
		return 0;

	if (enable)
		return regulator_enable(reg);
	else
		return regulator_disable(reg);
}

static int sdcan_stop(struct net_device *net){
	printk(KERN_INFO "SDCAN Stop\n");
	return 0;
}

static void sdcan_hw_sleep(struct spi_device *spi)		//Puts the SDCAN uC to sleep?
{
	//sdcan_write_reg(spi, CANCTRL, CANCTRL_REQOP_SLEEP);
}

static netdev_tx_t sdcan_hard_start_xmit(struct sk_buff *skb, struct net_device *net){
	printk(KERN_INFO "SDCAN Hard Start Transmit\n");
	return NETDEV_TX_OK;	
}

static int sdcan_do_set_mode(struct net_device *net, enum can_mode mode)
{
	struct sdcan_priv *priv = netdev_priv(net);

	switch (mode) {
	case CAN_MODE_START:
		sdcan_clean(net);
		/* We have to delay work since SPI I/O may sleep */
		priv->can.state = CAN_STATE_ERROR_ACTIVE;
		priv->restart_tx = 1;
		if (priv->can.restart_ms == 0)
			priv->after_suspend = AFTER_SUSPEND_RESTART;
		queue_work(priv->wq, &priv->restart_work);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static const struct net_device_ops sdcan_netdev_ops = {
	.ndo_open = sdcan_open,
	.ndo_stop = sdcan_stop,
	.ndo_start_xmit = sdcan_hard_start_xmit,
	.ndo_change_mtu = can_change_mtu
};

static const struct of_device_id sdcan_of_match[] = {
	{
		.compatible	= "sdcan",
		.data		= (void *)CAN_sdcan_1,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, sdcan_of_match);

static const struct spi_device_id sdcan_id_table[] = {
	{
		.name		= "sdcan",
		.driver_data	= (kernel_ulong_t)CAN_sdcan_1,
	},
	{ }
};
MODULE_DEVICE_TABLE(spi, sdcan_id_table);

static int sdcan_can_probe(struct spi_device *spi)
{
	const struct of_device_id *of_id = of_match_device(sdcan_of_match,
							   &spi->dev);
	//struct sdcan_platform_data *pdata = dev_get_platdata(&spi->dev);
	struct net_device *net;
	struct sdcan_priv *priv;
	struct clk *clk;
	int freq, ret;

	clk = devm_clk_get(&spi->dev, NULL);
	if (IS_ERR(clk)) {
		//if (pdata)
		//	freq = pdata->oscillator_frequency;
		//else
			return PTR_ERR(clk);
	} else {
		freq = clk_get_rate(clk);
	}

	/* Sanity check */
	if (freq < 1000000 || freq > 25000000)
		return -ERANGE;

	/* Allocate can/net device */
	net = alloc_candev(sizeof(struct sdcan_priv), TX_ECHO_SKB_MAX);
	if (!net)
		return -ENOMEM;

	if (!IS_ERR(clk)) {
		ret = clk_prepare_enable(clk);
		if (ret)
			goto out_free;
	}

	net->netdev_ops = &sdcan_netdev_ops;
	net->flags |= IFF_ECHO;

	priv = netdev_priv(net);
	priv->can.bittiming_const = &sdcan_bittiming_const;
	priv->can.do_set_mode = sdcan_do_set_mode;
	priv->can.clock.freq = freq / 2;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_3_SAMPLES |
		CAN_CTRLMODE_LOOPBACK | CAN_CTRLMODE_LISTENONLY;
	if (of_id)
		priv->model = (enum sdcan_model)of_id->data;
	else
		priv->model = spi_get_device_id(spi)->driver_data;
	priv->net = net;
	priv->clk = clk;

	spi_set_drvdata(spi, priv);

	/* Configure the SPI bus */
	spi->bits_per_word = 8;
	spi->max_speed_hz = spi->max_speed_hz ? : 10 * 1000 * 1000;
	ret = spi_setup(spi);
	if (ret)
		goto out_clk;

	priv->power = devm_regulator_get_optional(&spi->dev, "vdd");
	priv->transceiver = devm_regulator_get_optional(&spi->dev, "xceiver");
	if ((PTR_ERR(priv->power) == -EPROBE_DEFER) ||
	    (PTR_ERR(priv->transceiver) == -EPROBE_DEFER)) {
		ret = -EPROBE_DEFER;
		goto out_clk;
	}

	ret = sdcan_power_enable(priv->power, 1);
	if (ret)
		goto out_clk;

	priv->spi = spi;
	mutex_init(&priv->sdcan_lock);

	/* If requested, allocate DMA buffers */
	if (sdcan_enable_dma) {
		spi->dev.coherent_dma_mask = ~0;

		/*
		 * Minimum coherent DMA allocation is PAGE_SIZE, so allocate
		 * that much and share it between Tx and Rx DMA buffers.
		 */
		priv->spi_tx_buf = dmam_alloc_coherent(&spi->dev,
						       PAGE_SIZE,
						       &priv->spi_tx_dma,
						       GFP_DMA);

		if (priv->spi_tx_buf) {
			priv->spi_rx_buf = (priv->spi_tx_buf + (PAGE_SIZE / 2));
			priv->spi_rx_dma = (dma_addr_t)(priv->spi_tx_dma +
							(PAGE_SIZE / 2));
		} else {
			/* Fall back to non-DMA */
			sdcan_enable_dma = 0;
		}
	}

	/* Allocate non-DMA buffers */
	if (!sdcan_enable_dma) {
		priv->spi_tx_buf = devm_kzalloc(&spi->dev, SPI_TRANSFER_BUF_LEN,
						GFP_KERNEL);
		if (!priv->spi_tx_buf) {
			ret = -ENOMEM;
			goto error_probe;
		}
		priv->spi_rx_buf = devm_kzalloc(&spi->dev, SPI_TRANSFER_BUF_LEN,
						GFP_KERNEL);
		if (!priv->spi_rx_buf) {
			ret = -ENOMEM;
			goto error_probe;
		}
	}

	SET_NETDEV_DEV(net, &spi->dev);

	/* Here is OK to not lock the MCP, no one knows about it yet */
	ret = sdcan_hw_probe(spi);
	if (ret) {
		if (ret == -ENODEV)
			dev_err(&spi->dev, "Cannot initialize MCP%x. Wrong wiring?\n", priv->model);
		goto error_probe;
	}

	sdcan_hw_sleep(spi);

	ret = register_candev(net);
	if (ret)
		goto error_probe;

	devm_can_led_init(net);

	netdev_info(net, "MCP%x successfully initialized.\n", priv->model);
 	return 0;

error_probe:
	sdcan_power_enable(priv->power, 0);

out_clk:
	if (!IS_ERR(clk))
		clk_disable_unprepare(clk);

out_free:
	free_candev(net);

	dev_err(&spi->dev, "Probe failed, err=%d\n", -ret);
	return ret;
}

static int sdcan_can_remove(struct spi_device *spi)
{
	struct sdcan_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;

	unregister_candev(net);

	sdcan_power_enable(priv->power, 0);

	if (!IS_ERR(priv->clk))
		clk_disable_unprepare(priv->clk);

	free_candev(net);

	return 0;
}

static int __maybe_unused sdcan_can_suspend(struct device *dev)
{
	// struct spi_device *spi = to_spi_device(dev);
	// struct sdcan_priv *priv = spi_get_drvdata(spi);
	// struct net_device *net = priv->net;

	// priv->force_quit = 1;
	// disable_irq(spi->irq);
	// /*
	//  * Note: at this point neither IST nor workqueues are running.
	//  * open/stop cannot be called anyway so locking is not needed
	//  */
	// if (netif_running(net)) {
	// 	netif_device_detach(net);

	// 	sdcan_hw_sleep(spi);
	// 	sdcan_power_enable(priv->transceiver, 0);
	// 	priv->after_suspend = AFTER_SUSPEND_UP;
	// } else {
	// 	priv->after_suspend = AFTER_SUSPEND_DOWN;
	// }

	// if (!IS_ERR_OR_NULL(priv->power)) {
	// 	regulator_disable(priv->power);
	// 	priv->after_suspend |= AFTER_SUSPEND_POWER;
	// }

	return 0;
}

static int __maybe_unused sdcan_can_resume(struct device *dev)
{
	// struct spi_device *spi = to_spi_device(dev);
	// struct sdcan_priv *priv = spi_get_drvdata(spi);

	// if (priv->after_suspend & AFTER_SUSPEND_POWER)
	// 	sdcan_power_enable(priv->power, 1);

	// if (priv->after_suspend & AFTER_SUSPEND_UP) {
	// 	sdcan_power_enable(priv->transceiver, 1);
	// 	queue_work(priv->wq, &priv->restart_work);
	// } else {
	// 	priv->after_suspend = 0;
	// }

	// priv->force_quit = 0;
	// enable_irq(spi->irq);
	return 0;
}

static SIMPLE_DEV_PM_OPS(sdcan_can_pm_ops, sdcan_can_suspend,
	sdcan_can_resume);

static struct spi_driver sdcan_can_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = sdcan_of_match,
		.pm = &sdcan_can_pm_ops,
	},
	.id_table = sdcan_id_table,
	.probe = sdcan_can_probe,
	.remove = sdcan_can_remove,
};
module_spi_driver(sdcan_can_driver);

