
/*
 * socfpga (HPS-ethernet)-to-FPGA software driver (eth2ocm)
 *
 * Copyright (C) 2017-2018 Radis Ltd (Zelenograd, Russia).
 */

#include <asm/irq.h>
#include <asm/page.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/etherdevice.h>
#include <linux/highmem.h>
#include <linux/hrtimer.h>
#include <linux/in.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/random.h>
#include <linux/regmap.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <uapi/linux/ip.h>

//#include <linux/timekeeping.h>

#include "fifo.h"

static int debug_level;
module_param(debug_level, int, 0);
MODULE_PARM_DESC(debug_level, "eth2ocm debug level (NETIF_MSG bits)");

/* Netif debug messages possible */
#define ETH2OCM_DEBUG                                                        \
  (NETIF_MSG_DRV | NETIF_MSG_PROBE | NETIF_MSG_LINK | NETIF_MSG_TIMER |      \
   NETIF_MSG_IFDOWN | NETIF_MSG_IFUP | NETIF_MSG_RX_ERR | NETIF_MSG_TX_ERR | \
   NETIF_MSG_TX_QUEUED | NETIF_MSG_INTR | NETIF_MSG_TX_DONE |                \
   NETIF_MSG_RX_STATUS | NETIF_MSG_PKTDATA | NETIF_MSG_HW | NETIF_MSG_WOL)

#define ETH2OCM_HW_PORTRSTN_ADDR 0xFFC25080
#define ETH2OCM_HW_PORTRSTN_LEN 4

// Disable FPGA to SDRAM port
#define ETH2OCM_HW_ALL_PORT_DIS 0x00

// Reset management registers (for now only brgmodbrst)
#define ETH2OCM_HW_RSTMGR_ADDR 0xFFD05000
#define ETH2OCM_HW_RSTMGR_LEN 0x24
#define ETH2OCM_HW_BRGMODRST_REG_OFF 0x1C

// BRGMODRST fields
#define ETH2OCM_HW_BRGMODRST_MASK 0x7  // 3 bits
#define ETH2OCM_HW_BRGMODRST_F2H_BIT 1 << 2
#define ETH2OCM_HW_BRGMODRST_LWH2F_BIT 1 << 1
#define ETH2OCM_HW_BRGMODRST_H2F_BIT 1 << 0

// L3 GPV registers (for now only remap)
#define ETH2OCM_HW_L3REGS_ADDR 0xFF800000
#define ETH2OCM_HW_L3REGS_LEN 0x100000
#define ETH2OCM_HW_L3REGS_REMAP_OFF 0x0  // Remap register
#define ETH2OCM_HW_L3REGS_SECUR_OFF \
  0x20  // Security settings register (lwhps2fpgaregs)

// remap fields
#define ETH2OCM_HW_L3REGS_REMAP_LWH2F_BIT 1 << 4
#define ETH2OCM_HW_L3REGS_REMAP_H2F_BIT 1 << 3

#define ETH2OCM_HW_MEM_RX_ADDR 0x00000000
#define ETH2OCM_HW_MEM_TX_ADDR 0x00002000

#define ETH2OCM_HW_RX_REGS_BASE_ADDR 0x00004000
#define ETH2OCM_HW_TX_REGS_BASE_ADDR 0x00004080

// Control registers
#define ETH2OCM_TX_HW_FRAME_SIZE_REG 0
#define ETH2OCM_TX_HW_DMA_START_REG 4
#define ETH2OCM_TX_HW_DATA_COPY_REG 16
#define ETH2OCM_TX_HW_IRQ_ENABLE_REG 20
#define ETH2OCM_TX_HW_DMA_FINISHED_REG 24
#define ETH2OCM_TX_HW_IRQ_RESET_REG 28
#define ETH2OCM_TX_HW_HDLCADDR_REG 40

// Status registers
#define ETH2OCM_TX_HW_FIFO_FREE_SPACE_REG 8
#define ETH2OCM_TX_HW_FIFO_READY_REG 8
#define ETH2OCM_TX_HW_FIFO_FRAMES_QUANTITY_REG 12

// Control registers
#define ETH2OCM_RX_HW_FRAME_SIZE_REG 16
#define ETH2OCM_RX_HW_DMA_START_REG 4
#define ETH2OCM_RX_HW_DATA_COPY_REG 24
#define ETH2OCM_RX_HW_IRQ_ENABLE_REG 28
#define ETH2OCM_RX_HW_HDLCADDR_REG 40

// Status registers
#define ETH2OCM_RX_HW_FIFO_FREE_SPACE_REG 8
#define ETH2OCM_RX_HW_DMA_TRANS_DONE_REG 20
#define ETH2OCM_RX_HW_FIFO_FRAMES_QUANTITY_REG 12

#define ETH2OCM_DEF_MIN_ETHPKTSIZE (60) /* Minimum ethernet pkt size */
#define ETH2OCM_DEF_MAX_FRAME_SIZE (1500 + 14 + 4 + 4)
#define ETH2OCM_ONCHIP_MEM_WORD_SIZE 4
#define ETH2OCM_WORD_COPYING_TIME 10  // ns

/* version info */
#define ETH2OCM_MAJOR_VERSION 0
#define ETH2OCM_MINOR_VERSION 3
#define ETH2OCM_MODULE_VERSION "0.3"
MODULE_VERSION(ETH2OCM_MODULE_VERSION);
static const char eth2ocm_version_string[] =
    "Radis Ltd eth2ocm linux driver v0.1";

#define MS_TO_NS(x) (x * 1E6L)
#define PERIOD (2L * 1E6L)  // 2 ms

static char default_mac[ETH_ALEN] = {0x00, 0x23, 0x2E, 0xAA, 0xBB, 0x00};
struct nfifo_t *tx_fifo_tmp1, *tx_fifo_tmp2;
struct hrtimer txch_timer;
int waiting_for_irq = 0;
ktime_t ktime;
static DECLARE_KFIFO(tx_fifo, struct nfifo_t, FIFO_SIZE);

struct eth2ocm_platform_data {
  char mac_addr[ETH_ALEN];
  u8 version;
  void (*interrupt_enable)(void);
  void (*interrupt_disable)(void);
};

/* eth2ocm_priv: eth2ocm private data structure */
struct eth2ocm_priv {
  u32 msg_enable;
  struct net_device* ndev;
  struct platform_device* pdev;
  struct device* dev;
  char mac_addr[6];
  void __iomem* hw_base;
  void __iomem* hw_tx_base;
  void __iomem* hw_rx_base;
  void __iomem* hw_ctrl_tx_base;
  void __iomem* hw_ctrl_rx_base;
  int rx_irq;
  int tx_irq;
  u32 rx_hw_fifo_isr_count;
  u32 rx_hw_dma_isr_count;
  u32 rx_hw_unk_evnt_isr_count;
  u32 tx_isr_count;
  /*platform specific members*/
  void (*int_enable)(void);
  void (*int_disable)(void);
  u32 hdlcaddr_tx;
  u32 hdlcaddr_rx;
};

struct eth2ocm_priv* gpriv;
struct net_device* gndev;

struct word32_t {
  uint32_t data;
} __attribute__((__packed__));

/* Helper macros */
#define hw_data_read(offset) ioread32(priv->hw_rx_base + (offset))
#define hw_data_write(offset, val) iowrite32(val, (priv->hw_tx_base + (offset)))

#define hw_ctrl_rx_read(reg) ioread32((priv->hw_ctrl_rx_base + (reg)))
#define hw_ctrl_rx_write(reg, val) \
  iowrite32(val, (priv->hw_ctrl_rx_base + (reg)))

#define hw_ctrl_tx_read(reg) ioread32((priv->hw_ctrl_tx_base + (reg)))
#define hw_ctrl_tx_write(reg, val) \
  iowrite32(val, (priv->hw_ctrl_tx_base + (reg)))

enum hrtimer_restart tx_check_timer_callback(struct hrtimer* timer) {
  ktime_t currtime, interval;

  if (!kfifo_is_empty(&tx_fifo)) {
    iowrite32(0, (gpriv->hw_ctrl_tx_base + ETH2OCM_TX_HW_IRQ_ENABLE_REG));
    iowrite32(1, (gpriv->hw_ctrl_tx_base + ETH2OCM_TX_HW_IRQ_ENABLE_REG));

    return HRTIMER_NORESTART;
  }

  currtime = ktime_get();
  interval = ktime_set(0, PERIOD);
  hrtimer_forward(timer, currtime, interval);

  return HRTIMER_RESTART;
}

/***********************************************************************
 *  eth2ocm hardware manipulation
 **********************************************************************/

/**
 * eth2ocm_init_hw_interface - init hw interface (FPGA)
 *
 * Map & set hw (FPGA) registers
 *
 */
static int eth2ocm_init_hw_interface(void) {
  void __iomem* RSTMGR = NULL;
  void __iomem* L3REGS = NULL;
  void __iomem* PORTRSTN = NULL;
  uint32_t brgmodrst = 0;
  uint32_t test = 0;
  uint32_t remap = 0;
  int ret = 0;

  /********************************************
   * First, map configuration registers regions
   ********************************************/

  RSTMGR = ioremap(ETH2OCM_HW_RSTMGR_ADDR, ETH2OCM_HW_RSTMGR_LEN);
  if (!RSTMGR) {
    printk("Failed to map RSTMGR address space \n");
    ret = -ENXIO;
    goto err_release_rstmgr;
  }

  if (!request_mem_region(ETH2OCM_HW_L3REGS_ADDR, ETH2OCM_HW_L3REGS_LEN,
                          "eth2ocm")) {
    printk("Failed to request mem region for L3 registers\n");
    ret = -ENXIO;
    goto err_release_rstmgr;
  }

  L3REGS = ioremap(ETH2OCM_HW_L3REGS_ADDR, ETH2OCM_HW_L3REGS_LEN);
  if (!L3REGS) {
    printk("Failed to map L3REGS ddress space \n");
    ret = -ENXIO;
    goto err_release_l3regs;
  }

  if (!request_mem_region(ETH2OCM_HW_PORTRSTN_ADDR, ETH2OCM_HW_PORTRSTN_LEN,
                          "eth2ocm")) {
    printk("Failed to request mem region for FPGA2SDRAM reset registers\n");
    ret = -ENXIO;
    goto err_release_portrstn;
  }

  PORTRSTN = ioremap(ETH2OCM_HW_PORTRSTN_ADDR, ETH2OCM_HW_PORTRSTN_LEN);
  if (!PORTRSTN) {
    printk("Failed to map PORTRSTN address space \n");
    ret = -ENXIO;
    goto err_release_portrstn;
  }

  /********************************************
   * Now do mapping related hw configuration
   ********************************************/

  // 1. Disable reset
  brgmodrst = ~(ETH2OCM_HW_BRGMODRST_H2F_BIT | ETH2OCM_HW_BRGMODRST_LWH2F_BIT |
                ETH2OCM_HW_BRGMODRST_F2H_BIT) &
              ETH2OCM_HW_BRGMODRST_MASK;
  iowrite32(brgmodrst, RSTMGR + ETH2OCM_HW_BRGMODRST_REG_OFF);

  // Check how it was written
  test = ioread32(RSTMGR + ETH2OCM_HW_BRGMODRST_REG_OFF);
  if (test != brgmodrst) {
    printk("Failed to disable H2F, LWH2F and F2H reset\n");
    printk("brgmodrst is 0x%4X, must be 0x%4X\n", test, brgmodrst);
    ret = -ENXIO;
    goto err_release_l3regs;
  }

  // 2. Enable remaping
  remap = ETH2OCM_HW_L3REGS_REMAP_LWH2F_BIT | ETH2OCM_HW_L3REGS_REMAP_H2F_BIT;
  iowrite32(remap, L3REGS + ETH2OCM_HW_L3REGS_REMAP_OFF);
  // remap register is write-only, so we can't check it.

  // Enable reset for FPGA2SDRAM ports
  iowrite32(ETH2OCM_HW_ALL_PORT_DIS, PORTRSTN);

  // We don't need L3 regs, reset management and FPGA2SDRAM registers anymore
err_release_portrstn:
  if (PORTRSTN) {
    iounmap(PORTRSTN);
  }
  release_mem_region(ETH2OCM_HW_PORTRSTN_ADDR, ETH2OCM_HW_PORTRSTN_LEN);

err_release_l3regs:
  if (L3REGS) {
    iounmap(L3REGS);
  }
  release_mem_region(ETH2OCM_HW_L3REGS_ADDR, ETH2OCM_HW_L3REGS_LEN);

err_release_rstmgr:
  if (RSTMGR) {
    iounmap(RSTMGR);
  }

  return ret;
}

/**
 * eth2ocm_int_disable - Disable eth2ocm module interrupt
 * @priv: eth2ocm private structure
 *
 * Disable eth2ocm interrupt
 *
 */
static void eth2ocm_int_disable(struct eth2ocm_priv* priv) {
  hw_ctrl_rx_write(ETH2OCM_RX_HW_IRQ_ENABLE_REG, 0);
  hw_ctrl_tx_write(ETH2OCM_TX_HW_IRQ_ENABLE_REG, 0);
}

/**
 * eth2ocm_int_enable - Enable eth2ocm module interrupt
 * @priv: eth2ocm private structure
 *
 * Enable eth2ocm interrupt
 *
 */
static void eth2ocm_int_enable(struct eth2ocm_priv* priv) {
  hw_ctrl_rx_write(ETH2OCM_RX_HW_IRQ_ENABLE_REG, 1);
  hw_ctrl_tx_write(ETH2OCM_TX_HW_IRQ_ENABLE_REG, 1);
}

/**
 * eth2ocm_tx_fifo_push - push packet in kfifo
 * @irq: interrupt number
 * @skb: SKB pointer
 *
 * Returns success or error (FIFO_IS_FULL)
 */
static int eth2ocm_tx_fifo_push(struct eth2ocm_priv* priv,
                                struct sk_buff* skb) {
  memcpy(tx_fifo_tmp1->data, skb->data, skb->len);
  tx_fifo_tmp1->len = skb->len;

  if (kfifo_put(&tx_fifo, *tx_fifo_tmp1))
    return WRITE_SUCCESS;
  else
    return FIFO_IS_FULL;
}

/**
 * eth2ocm_rx_irq - eth2ocm rx interrupt handler
 * @priv: eth2ocm private structure
 * @dev_id: eth2ocm network adapter data structure ptr
 *
 * Returns interrupt handled condition
 */
static irqreturn_t eth2ocm_rx_irq(int irq, void* dev_id) {
  unsigned int len;
  unsigned char* data;
  struct sk_buff* skb;
  unsigned int i, j, k, frames;
  unsigned int remainder, words;
  unsigned long dma_delay;
  uint32_t word;

  // Прерывание от FPGA о наличии фрейма в fifo буфере FPGA
  frames =
      ioread32(gpriv->hw_ctrl_rx_base + ETH2OCM_RX_HW_FIFO_FRAMES_QUANTITY_REG);
  if (frames) {
    // Disable rx interrupts
    iowrite32(0, (gpriv->hw_ctrl_rx_base + ETH2OCM_RX_HW_IRQ_ENABLE_REG));
    for (k = 0; k < frames; k++) {
      // Можно попробовать прочитать размер принимаемого фрейма
      len = ioread32(gpriv->hw_ctrl_rx_base + ETH2OCM_RX_HW_FRAME_SIZE_REG);
      if (len == 0) {
        gndev->stats.rx_errors++;
        continue;
      }
      skb = netdev_alloc_skb(gndev, len);
      if (unlikely(skb == NULL)) {
        gndev->stats.rx_dropped++;
        continue;
      }
      // Запускаем DMA транзакцию
      iowrite32(1, (gpriv->hw_ctrl_rx_base + ETH2OCM_RX_HW_DMA_START_REG));
      // Ждем ее окончания
      words = len / ETH2OCM_ONCHIP_MEM_WORD_SIZE;
      remainder = len % ETH2OCM_ONCHIP_MEM_WORD_SIZE;
      if (remainder)
        words++;

      dma_delay = ((words * ETH2OCM_WORD_COPYING_TIME) / 1000) + 1;  // +2
      udelay(dma_delay);

      data = skb_put(skb, len);

      j = 0;
      i = 0;
      do {
        word = ioread32(gpriv->hw_rx_base + i * ETH2OCM_ONCHIP_MEM_WORD_SIZE);
        *(uint32_t*)(data + i * ETH2OCM_ONCHIP_MEM_WORD_SIZE) = word;
        j += ETH2OCM_ONCHIP_MEM_WORD_SIZE;
        i++;
      } while (j < len);

      // Cбрасываem DMA_status в 0
      iowrite32(0, (gpriv->hw_ctrl_rx_base + ETH2OCM_RX_HW_DMA_TRANS_DONE_REG));

      skb->protocol = eth_type_trans(skb, gndev);

      netif_rx(skb);
      gndev->stats.rx_packets++;
      gndev->stats.rx_bytes += len;
    }

    // Enable rx interrupts
    iowrite32(1, (gpriv->hw_ctrl_rx_base + ETH2OCM_RX_HW_IRQ_ENABLE_REG));
  }

  return IRQ_HANDLED;
}

/**
 * eth2ocm_tx_irq - eth2ocm tx interrupt handler
 * @irq: interrupt number
 * @dev_id: eth2ocm network adapter data structure ptr
 *
 * Returns interrupt handled condition
 */
static irqreturn_t eth2ocm_tx_irq(int irq, void* dev_id) {
  struct word32_t* word32;
  unsigned char* tx_buff;
  unsigned int words, remainder;
  int i, j;
  u32 ret;

  /* Прерывания означает, что: "приемный FIFO пуст,
   * FPGA готова к приему данных"
   */
  if (waiting_for_irq) {
    waiting_for_irq = 0;
    pr_info("irq_after_timeout\n");
  }

  ret = kfifo_get(&tx_fifo, tx_fifo_tmp2);
  if (ret) {
    ret = ioread32(gpriv->hw_ctrl_tx_base + ETH2OCM_TX_HW_FIFO_FREE_SPACE_REG);
    if (ret * ETH2OCM_ONCHIP_MEM_WORD_SIZE < tx_fifo_tmp2->len)
      goto out_tx_irq;

    iowrite32(tx_fifo_tmp2->len,
              gpriv->hw_ctrl_tx_base + ETH2OCM_TX_HW_FRAME_SIZE_REG);
    words = tx_fifo_tmp2->len / ETH2OCM_ONCHIP_MEM_WORD_SIZE;
    remainder = tx_fifo_tmp2->len % ETH2OCM_ONCHIP_MEM_WORD_SIZE;
    if (remainder)
      words++;

    j = 0;
    i = 0;
    tx_buff = tx_fifo_tmp2->data;
    do {
      word32 = (struct word32_t*)tx_buff;
      iowrite32(word32->data,
                gpriv->hw_tx_base + i * ETH2OCM_ONCHIP_MEM_WORD_SIZE);
      tx_buff += ETH2OCM_ONCHIP_MEM_WORD_SIZE;
      j++;
      i++;
    } while (j < words);

    // Запускаем DMA транзакцию
    iowrite32(1, gpriv->hw_ctrl_tx_base + ETH2OCM_TX_HW_DMA_START_REG);

    if (unlikely(netif_queue_stopped(gndev)))
      netif_wake_queue(gndev);

  } else {
    if (!hrtimer_active(&txch_timer)) {
      hrtimer_start(&txch_timer, ktime, HRTIMER_MODE_REL);
    }
    if (unlikely(netif_queue_stopped(gndev)))
      netif_wake_queue(gndev);
  }

out_tx_irq:
  return IRQ_HANDLED;
}

/**
 * eth2ocm_dev_xmit - eth2ocm Transmit function
 * @skb: SKB pointer
 * @ndev: eth2ocm network dev
 *
 * Called by the system to transmit a packet  - we queue the packet in
 * eth2ocm hardware transmit queue
 *
 * Returns success(NETDEV_TX_OK) or error code (typically out of fifo mem)
 */
static int eth2ocm_dev_xmit(struct sk_buff* skb, struct net_device* ndev) {
  struct device* eth2ocm_dev = &ndev->dev;
  struct eth2ocm_priv* priv = netdev_priv(ndev);
  int ret;

  ret = eth2ocm_tx_fifo_push(priv, skb);
  if (unlikely(ret != 0)) {
    if (netif_msg_tx_err(priv) && net_ratelimit())
      dev_err(eth2ocm_dev, "eth2ocm: fifo pushing failed\n");
    goto fail_tx;
  }

  /* If there is no more tx_fifo left free then we need to
   * tell the kernel to stop sending us tx frames.
   */

  if (unlikely(kfifo_is_full(&tx_fifo))) {
    netif_stop_queue(ndev);

    if (!hrtimer_active(&txch_timer))
      hrtimer_start(&txch_timer, ktime, HRTIMER_MODE_REL);
  }

  ndev->stats.tx_packets++;
  ndev->stats.tx_bytes += skb->len;

  dev_kfree_skb(skb);
  return NETDEV_TX_OK;

fail_tx:
  /* we need to tell the kernel to stop sending us tx frames */
  ndev->stats.tx_dropped++;
  dev_kfree_skb(skb);

  netif_stop_queue(ndev);

  if (!hrtimer_active(&txch_timer))
    hrtimer_start(&txch_timer, ktime, HRTIMER_MODE_REL);

  return NETDEV_TX_BUSY;
}

/**
 * eth2ocm_dev_tx_timeout - eth2ocm Transmit timeout function
 * @ndev: eth2ocm network dev
 *
 * Called when system detects that a skb timeout period has expired
 * potentially due to a fault in the adapter in not being able to send
 * it out on the wire. We teardown the TX channel assuming a hardware
 * error and re-initialize the TX channel for hardware operation
 *
 */
static void eth2ocm_dev_tx_timeout(struct net_device* ndev) {
  struct eth2ocm_priv* priv = netdev_priv(ndev);
  struct device* eth2ocm_dev = &ndev->dev;
  int free, full, frames, dma;

  free = hw_ctrl_tx_read(ETH2OCM_TX_HW_FIFO_FREE_SPACE_REG);
  dma = hw_ctrl_tx_read(ETH2OCM_TX_HW_DMA_FINISHED_REG);
  frames = hw_ctrl_tx_read(ETH2OCM_TX_HW_FIFO_FRAMES_QUANTITY_REG);
  full = kfifo_len(&tx_fifo);
  pr_err("xmit timeout: %i, %i, %i, %i\n", free, full, dma, frames);

  if ((free == 8191) && (full == FIFO_SIZE)) {
    pr_err("tx_irq restart!\n");
    iowrite32(0, (gpriv->hw_ctrl_tx_base + ETH2OCM_TX_HW_IRQ_ENABLE_REG));
    iowrite32(1, (gpriv->hw_ctrl_tx_base + ETH2OCM_TX_HW_IRQ_ENABLE_REG));

    if (!hrtimer_active(&txch_timer)) {
      pr_err("txch_timer restart!\n");
      hrtimer_start(&txch_timer, ktime, HRTIMER_MODE_REL);
    }

    waiting_for_irq = 1;
  }

  if (!netif_queue_stopped(ndev))
    netif_stop_queue(ndev);

  if (netif_msg_tx_err(priv))
    dev_err(eth2ocm_dev, "eth2ocm: xmit timeout, restarting tx\n");

  ndev->stats.tx_errors++;
  if (!hw_ctrl_rx_read(ETH2OCM_RX_HW_IRQ_ENABLE_REG))
    // Enable rx interrupts
    hw_ctrl_rx_write(ETH2OCM_RX_HW_IRQ_ENABLE_REG, 1);
  netif_wake_queue(ndev);
}

/**
 * eth2ocm_setmac - Set mac address in the eth2ocm (internal function)
 * @priv: eth2ocm private adapter structure
 * @mac_addr: MAC address to set in device
 *
 * Called internally to set the mac address of the adapter (Device)
 *
 * Returns success (0) or appropriate error code (none as of now)
 */
static void eth2ocm_setmac(struct eth2ocm_priv* priv, char* mac_addr) {
  // struct device *eth2ocm_dev = &priv->ndev->dev;
  // Сохраняем MAC адрес в регистрах FPGA
}

/**
 * eth2ocm_dev_setmac_addr - Set mac address in the adapter
 * @ndev: eth2ocm network dev
 * @addr: MAC address to set in device
 *
 * Called by the system to set the mac address of eth2ocm (Device)
 *
 * Returns success (0) or appropriate error code (none as of now)
 */
static int eth2ocm_dev_setmac_addr(struct net_device* ndev, void* addr) {
  struct eth2ocm_priv* priv = netdev_priv(ndev);
  struct device* eth2ocm_dev = &priv->ndev->dev;
  struct sockaddr* sa = addr;

  if (!is_valid_ether_addr(sa->sa_data))
    return -EADDRNOTAVAIL;

  /* Store mac addr in priv and set it in eth2ocm hw */
  memcpy(priv->mac_addr, sa->sa_data, ndev->addr_len);
  memcpy(ndev->dev_addr, sa->sa_data, ndev->addr_len);

  /* MAC address is configured only after the interface is enabled */
  if (netif_running(ndev)) {
    eth2ocm_setmac(priv, priv->mac_addr);
  }

  if (netif_msg_drv(priv))
    dev_notice(eth2ocm_dev, "eth2ocm: eth2ocm_dev_setmac_addr %pM\n",
               priv->mac_addr);

  return 0;
}

/*************************************************************************
 *  Linux Driver Model
 *************************************************************************/

/**
 * eth2ocm_dev_open - eth2ocm device open
 * @ndev: eth2ocm network dev
 *
 * Called when system wants to start the interface. We init TX/RX channels
 * and enable the hardware for packet reception/transmission and start the
 * network queue.
 *
 * Returns 0 for a successful open, or appropriate error code
 */
static int eth2ocm_dev_open(struct net_device* ndev) {
  struct eth2ocm_priv* priv = netdev_priv(ndev);
  int cnt;

  /* link ON */
  if (!netif_carrier_ok(ndev))
    netif_carrier_on(ndev);

  for (cnt = 0; cnt < ETH_ALEN; cnt++)
    ndev->dev_addr[cnt] = priv->mac_addr[cnt];

  /* Configuration items */
  // Здесь нам нужно убедиться, что FPGA загружена и готова к работе
  eth2ocm_int_enable(priv);
  hw_ctrl_tx_write(ETH2OCM_TX_HW_IRQ_RESET_REG, 0);
  hw_ctrl_tx_write(ETH2OCM_TX_HW_IRQ_RESET_REG, 1);
  netif_start_queue(ndev);

  return 0;
}

/**
 * eth2ocm_dev_stop - eth2ocm device stop
 * @ndev: eth2ocm network dev
 *
 * Called when system wants to stop or down the interface. We stop the network
 * queue, disable interrupts and cleanup TX/RX channels.
 *
 * We return the statistics in net_device_stats structure pulled from emac
 */
static int eth2ocm_dev_stop(struct net_device* ndev) {
  struct eth2ocm_priv* priv = netdev_priv(ndev);

  /* inform the upper layers. */
  netif_stop_queue(ndev);

  netif_carrier_off(ndev);
  eth2ocm_int_disable(priv);

  /* Free IRQ */

  return 0;
}

/**
 * eth2ocm_dev_getnetstats - eth2ocm get statistics function
 * @ndev: eth2ocm network dev
 *
 * Called when system wants to get statistics from the device.
 *
 * We return the statistics in net_device_stats structure pulled from emac
 */
static struct net_device_stats* eth2ocm_dev_getnetstats(
    struct net_device* ndev) {
  return &ndev->stats;
}

static const struct net_device_ops eth2ocm_netdev_ops = {
    .ndo_open = eth2ocm_dev_open,
    .ndo_stop = eth2ocm_dev_stop,
    .ndo_start_xmit = eth2ocm_dev_xmit,
    .ndo_set_mac_address = eth2ocm_dev_setmac_addr,
    .ndo_tx_timeout = eth2ocm_dev_tx_timeout,
    .ndo_get_stats = eth2ocm_dev_getnetstats,
};

static const struct of_device_id eth2ocm_of_match[];

static struct eth2ocm_platform_data* eth2ocm_of_get_pdata(
    struct platform_device* pdev, struct eth2ocm_priv* priv) {
  struct device_node* np;
  struct eth2ocm_platform_data* pdata = NULL;
  const u8* mac_addr;

  if (!IS_ENABLED(CONFIG_OF) || !pdev->dev.of_node)
    return dev_get_platdata(&pdev->dev);

  pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
  if (!pdata)
    return NULL;

  np = pdev->dev.of_node;

  if (!is_valid_ether_addr(pdata->mac_addr)) {
    mac_addr = of_get_mac_address(np);
    if (mac_addr)
      ether_addr_copy(pdata->mac_addr, mac_addr);
  }
  return pdata;
}

static ssize_t eth2ocm_priv_hdlcaddr_rx_show(struct device* dev,
                                             struct device_attribute* attr,
                                             char* buf) {
  gpriv->hdlcaddr_rx =
      ioread32(gpriv->hw_ctrl_rx_base + ETH2OCM_RX_HW_HDLCADDR_REG);
  return sprintf(buf, "%d\n", gpriv->hdlcaddr_rx);
}

static ssize_t eth2ocm_priv_hdlcaddr_rx_store(struct device* dev,
                                              struct device_attribute* attr,
                                              char* buf, size_t count) {
  sscanf(buf, "%d", &gpriv->hdlcaddr_rx);
  iowrite32(gpriv->hdlcaddr_rx,
            gpriv->hw_ctrl_rx_base + ETH2OCM_RX_HW_HDLCADDR_REG);
  return count;
}

static DEVICE_ATTR(hdlcaddr_rx, S_IRUGO | S_IWUSR,
                   eth2ocm_priv_hdlcaddr_rx_show,
                   eth2ocm_priv_hdlcaddr_rx_store);

static ssize_t eth2ocm_priv_hdlcaddr_tx_show(struct device* dev,
                                             struct device_attribute* attr,
                                             char* buf) {
  gpriv->hdlcaddr_tx =
      ioread32(gpriv->hw_ctrl_tx_base + ETH2OCM_TX_HW_HDLCADDR_REG);
  return sprintf(buf, "%d\n", gpriv->hdlcaddr_tx);
}

static ssize_t eth2ocm_priv_hdlcaddr_tx_store(struct device* dev,
                                              struct device_attribute* attr,
                                              char* buf, size_t count) {
  sscanf(buf, "%d", &gpriv->hdlcaddr_tx);
  iowrite32(gpriv->hdlcaddr_tx,
            gpriv->hw_ctrl_tx_base + ETH2OCM_TX_HW_HDLCADDR_REG);
  return count;
}

static DEVICE_ATTR(hdlcaddr_tx, S_IRUGO | S_IWUSR,
                   eth2ocm_priv_hdlcaddr_tx_show,
                   eth2ocm_priv_hdlcaddr_tx_store);

static struct attribute* eth2ocm_priv_attrs[] = {
    &dev_attr_hdlcaddr_rx.attr, &dev_attr_hdlcaddr_tx.attr, NULL};

static struct attribute_group eth2ocm_priv_attr_group = {
    .name = "params",
    .attrs = eth2ocm_priv_attrs,
};

/**
 * eth2ocm_probe - eth2ocm device probe
 * @pdev: eth2ocm device that we are probing
 *
 * Called when probing for eth2ocm device. We get details of instances and
 * resource information from platform init and register a network device
 * and allocate resources necessary for driver to perform
 */
static int eth2ocm_probe(struct platform_device* pdev) {
  int rc = 0;
  struct resource* res;
  struct net_device* ndev;
  struct eth2ocm_priv* priv;
  struct eth2ocm_platform_data* pdata;

  dev_info(&pdev->dev, "eth2ocm_probe ...\n");

  ndev = alloc_etherdev(sizeof(struct eth2ocm_priv));
  if (!ndev)
    return -ENOMEM;

  tx_fifo_tmp1 = kmalloc(sizeof(struct nfifo_t), GFP_KERNEL);
  if (!tx_fifo_tmp1)
    return -ENOMEM;

  tx_fifo_tmp2 = kmalloc(sizeof(struct nfifo_t), GFP_KERNEL);
  if (!tx_fifo_tmp2)
    return -ENOMEM;

  SET_NETDEV_DEV(ndev, &pdev->dev);

  platform_set_drvdata(pdev, ndev);
  priv = netdev_priv(ndev);
  priv->pdev = pdev;
  priv->ndev = ndev;
  priv->dev = &pdev->dev;

  INIT_KFIFO(tx_fifo);

  ktime = ktime_set(0, PERIOD);
  hrtimer_init(&txch_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
  txch_timer.function = &tx_check_timer_callback;

  priv->msg_enable = netif_msg_init(debug_level, ETH2OCM_DEBUG);

  pdata = eth2ocm_of_get_pdata(pdev, priv);
  if (!pdata) {
    dev_err(&pdev->dev, "no platform data\n");
    rc = -ENODEV;
    goto no_pdata;
  }

  priv->int_enable = pdata->interrupt_enable;
  priv->int_disable = pdata->interrupt_disable;

  priv->tx_irq = platform_get_irq(pdev, 0);
  if (priv->tx_irq <= 0)
    return -ENOENT;

  priv->rx_irq = platform_get_irq(pdev, 1);
  if (priv->rx_irq <= 0)
    return -ENOENT;

  res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  priv->hw_base = devm_ioremap_resource(&pdev->dev, res);
  if (IS_ERR(priv->hw_base))
    goto no_pdata;

  priv->hw_ctrl_tx_base = priv->hw_base + ETH2OCM_HW_TX_REGS_BASE_ADDR;
  priv->hw_ctrl_rx_base = priv->hw_base + ETH2OCM_HW_RX_REGS_BASE_ADDR;
  priv->hw_tx_base = priv->hw_base + ETH2OCM_HW_MEM_TX_ADDR;
  priv->hw_rx_base = priv->hw_base + ETH2OCM_HW_MEM_RX_ADDR;

  // Init hardware interfaces
  rc = eth2ocm_init_hw_interface();
  if (rc) {
    dev_err(&pdev->dev, "hw interface configuration failed\n");
    goto reg_err;
  }

  eth2ocm_int_disable(priv);
  get_random_bytes(&rc, 1);
  default_mac[5] = rc;
  memcpy(priv->mac_addr, default_mac, ETH_ALEN);
  memcpy(ndev->dev_addr, priv->mac_addr, ETH_ALEN);

  if (!is_valid_ether_addr(priv->mac_addr)) {
    /* Use random MAC if none passed */
    eth_hw_addr_random(ndev);
    memcpy(priv->mac_addr, ndev->dev_addr, ndev->addr_len);
    dev_warn(&pdev->dev, "using random MAC addr: %pM\n", priv->mac_addr);
  }

  ndev->netdev_ops = &eth2ocm_netdev_ops;
  ether_setup(ndev);
  // Пока мы не умеем работать с MULTICAST
  ndev->flags &= ~IFF_MULTICAST;
  ndev->watchdog_timeo = msecs_to_jiffies(1500);  // 5000

  /* handle hw rx & tx irqs */
  rc = request_irq(priv->rx_irq, eth2ocm_rx_irq, IRQF_SHARED,
                   dev_name(&priv->ndev->dev), priv);
  if (rc) {
    dev_err(&pdev->dev, "cannot allocate interrupt %d,\n", priv->rx_irq);
    goto reg_err;
  }

  rc = request_irq(priv->tx_irq, eth2ocm_tx_irq, IRQF_SHARED,
                   dev_name(&priv->ndev->dev), priv);
  if (rc) {
    dev_err(&pdev->dev, "cannot allocate interrupt %d,\n", priv->tx_irq);
    goto reg_err;
  }

  /* register the network device */
  rc = register_netdev(ndev);
  if (rc) {
    dev_err(&pdev->dev, "error in register_netdev\n");
    rc = -ENODEV;
    goto reg_err;
  }
  gpriv = priv;
  gndev = ndev;

  rc = sysfs_create_group(&pdev->dev.kobj, &eth2ocm_priv_attr_group);
  if (rc != 0)
    dev_err(&pdev->dev, "failed to create hdlcaddr file in sysfs\n");

  dev_info(&pdev->dev, "eth2ocm_probe ok!\n");

  return 0;

reg_err:
no_pdata:
  free_netdev(ndev);
  return rc;
}

/**
 * eth2ocm_remove - eth2ocm device remove
 * @pdev: eth2ocm device that we are removing
 *
 * Called when removing the device driver. We disable clock usage and release
 * the resources taken up by the driver and unregister network device
 */
static int eth2ocm_remove(struct platform_device* pdev) {
  struct net_device* ndev = platform_get_drvdata(pdev);

  unregister_netdev(ndev);
  free_netdev(ndev);
  /*kfifo_free(&nfifo);*/

  return 0;
}

#if IS_ENABLED(CONFIG_OF)

static const struct of_device_id eth2ocm_of_match[] = {
    {
        .compatible = "radis-zel,eth2ocm-dev",
    },
    {},
};
MODULE_DEVICE_TABLE(of, eth2ocm_of_match);
#endif

/* eth2ocm_driver: eth2ocm platform driver structure */
static struct platform_driver eth2ocm_driver = {
    .driver =
        {
            .name = "eth2ocm",
            .of_match_table = of_match_ptr(eth2ocm_of_match),
        },
    .probe = eth2ocm_probe,
    .remove = eth2ocm_remove,
};

/**
 * eth2ocm_init - eth2ocm driver module init
 *
 * Called when initializing the driver. We register the driver with
 * the platform.
 */
static int __init eth2ocm_init(void) {
  return platform_driver_register(&eth2ocm_driver);
}
late_initcall(eth2ocm_init);

/**
 * eth2ocm_exit - eth2ocm driver module exit
 *
 * Called when exiting the driver completely. We unregister the driver with
 * the platform and exit
 */
static void __exit eth2ocm_exit(void) {
  platform_driver_unregister(&eth2ocm_driver);
}
module_exit(eth2ocm_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michael Sadikov");
MODULE_DESCRIPTION("socfpga (HPS-ethernet)-to-FPGA software driver (eth2ocm)");
