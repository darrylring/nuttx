/****************************************************************************
 * drivers/net/ethcan.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* ethcan presents a CAN FD controller as a NET_LL_ETHERNET network
 * interface.  It binds directly to the controller's "lower-half"
 * struct can_dev_s / struct can_ops_s (the same interface implemented by,
 * e.g., drivers/can/mcp2515.c), rather than going through the
 * character-mode CAN upper half (drivers/can/can.c, CONFIG_CAN) or a
 * SocketCAN AF_CAN socket (net/can, CONFIG_NET_CAN).  A controller bound
 * to ethcan is therefore dedicated to it: it cannot simultaneously be
 * registered as /dev/canX, because can_receive()/can_txdone() are
 * implemented once per image, and this file provides ethcan's versions of
 * them instead of linking drivers/can/can.c's.
 *
 * Wire format
 * -----------
 * An Ethernet frame is fragmented into a run of CAN FD data frames of up
 * to CAN_MAXDATALEN (64) payload bytes each, all carrying the *same* CAN
 * ID.  One bit of metadata rides in that ID rather than in the payload,
 * so the payload is used entirely for frame bytes:
 *
 *   - SOF: set on the first fragment of a new Ethernet frame, clear on
 *     continuations.  Seeing it discards whatever reassembly was in
 *     progress -- this doubles as this driver's loss handling for a
 *     dropped terminator, per its lossy-channel design (see below).
 *
 * A fragment shorter than CAN_MAXDATALEN ends the Ethernet frame ("short
 * frame terminates", the same convention USB bulk transfers use).  If the
 * frame length happens to fall in the range that CAN FD's non-linear DLC
 * encoding rounds up to 64 on the wire (49..64 bytes; see
 * can_bytes2dlc()), that fragment would be indistinguishable from a full,
 * non-terminal one, so an explicit empty terminator fragment is sent
 * instead.
 *
 * Medium access
 * -------------
 * Every ethcan node sends with the same CAN ID.  This is deliberate:
 * CAN's bitwise arbitration only resolves *different* IDs cleanly and
 * silently; two nodes driving identical ID bits and then diverging in the
 * data phase is the one case it cannot resolve, and produces a genuine,
 * detectable CAN bus error instead -- the CAN equivalent of an Ethernet
 * collision. This driver builds a CSMA/CD-style scheme on top of that:
 *
 *   - Before starting a new frame, defer while this node can see another
 *     frame's reassembly already in progress (the best "is the bus busy"
 *     proxy available above struct can_ops_s -- there is no hardware
 *     carrier-sense primitive exposed at this level, unlike an Ethernet
 *     PHY).  Once started, a node's fragments are queued to the
 *     controller back to back.
 *
 *   - Two nodes starting in the same instant, before either has seen the
 *     other's start-of-frame, is a race the above cannot close. If a CAN
 *     bus error is reported while this node's own send is in flight, it
 *     assumes a collision, abandons the burst, and retries the same
 *     staged frame after a randomized backoff, up to ETHCAN_MAXRETRIES
 *     times before giving up on it.
 *
 *   - Because collisions are now routine under contention rather than
 *     exceptional, repeated ones can push the controller toward CAN's
 *     bus-off state, which silently stops it both transmitting and
 *     receiving.  This driver watches for CAN_ERROR_BUSOFF and resets the
 *     controller to recover.
 *
 * Collision detection depends on the bound controller's lower-half driver
 * reporting errors (CONFIG_CAN_ERRORS / ARCH_HAVE_CAN_ERRORS). Where that
 * is not available, a collision is indistinguishable from any other
 * corruption: it just produces a garbled reassembly on both ends, caught
 * (or not) by the reassembled frame's own IP/UDP/TCP checksum like any
 * other loss on this deliberately lossy, no-guarantees, no-flow-control
 * link.
 *
 * Note also that CAN_ERROR_LOSTARB (arbitration loss) is a different,
 * harmless event that this driver does not need to react to: it can only
 * arise if this CAN segment also carries other, differently-ID'd traffic
 * that legitimately outranks ethcan's fixed ID, and the CAN controller
 * hardware already retransmits automatically once that traffic clears.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_NET_ETHCAN

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <time.h>
#include <nuttx/debug.h>

#include <arpa/inet.h>

#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spinlock.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/can/can.h>
#include <nuttx/can/can_common.h>
#include <nuttx/net/net.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ethernet.h>
#include <nuttx/net/ethcan.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#if !defined(CONFIG_CAN_FD) || !defined(CONFIG_CAN_EXTID)
#  error CONFIG_NET_ETHCAN requires CONFIG_CAN_FD and CONFIG_CAN_EXTID
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required in this configuration (CONFIG_SCHED_WORKQUEUE)
#endif

#define ETHCANWORK LPWORK

/* TX safety-net timeout: how long to wait for can_txdone() to resume a
 * fragmented send that stalled because the controller's TX FIFO was full,
 * before giving up on that frame entirely and polling for the next one.
 */

#define ETHCAN_TXTIMEOUT (2 * CLK_TCK)

/* How long a reassembly may sit idle (no new fragment, no terminator)
 * before it is abandoned.  Needed so a lost terminator cannot wedge this
 * node's own "is the bus busy" check forever.
 */

#define ETHCAN_RXIDLE_TIMEOUT (2 * CLK_TCK)

/* CSMA/CD-style retry backoff: a random multiple of a base slot, doubling
 * (capped) with each retry, similar in spirit to Ethernet's truncated
 * binary exponential backoff.  The slot is expressed in OS ticks rather
 * than derived from the configured CAN bit rate (not queryable through
 * struct can_ops_s without an extra ioctl round trip): a couple of
 * milliseconds safely covers one CAN FD fragment time at any bit rate
 * this hardware is likely to run.
 */

#define ETHCAN_BACKOFF_SLOT   MSEC2TICK(2)
#define ETHCAN_BACKOFF_MAXBIT 5             /* Caps the backoff window at
                                              * 2^5 = 32 slots */
#define ETHCAN_MAXRETRIES     8

/* CAN ID: just a start-of-frame marker.  Every node sends the same ID --
 * see the file header comment for why.
 */

#define ETHCAN_SOF_BIT   ((uint32_t)1 << 28)
#define ETHCAN_MKID(sof) ((sof) ? ETHCAN_SOF_BIT | 1 : 1)

/* Any fragment whose intended length is > 48 bytes rounds up to CAN FD
 * DLC 15 (= 64 bytes) on the wire (see can_bytes2dlc()), which is
 * indistinguishable from a full, non-terminal fragment.  A frame that
 * ends with such a fragment needs an explicit empty terminator.
 */

#define ETHCAN_DLC15_THRESHOLD 48

#define PKTBUF_SIZE (CONFIG_NET_ETHCAN_PKTSIZE + CONFIG_NET_GUARDSIZE)

#define BUF ((FAR struct eth_hdr_s *)priv->ec_dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ethcan_reasm_s
{
  bool     active;   /* Reassembly in progress */
  uint16_t len;       /* Bytes accumulated so far */
};

struct ethcan_driver_s
{
  bool                  ec_bifup;
  spinlock_t            ec_lock;        /* Protects fields shared with
                                          * can_receive()/can_txdone(),
                                          * which may run in any context */

  FAR struct can_dev_s *ec_can;         /* Bound lower-half CAN FD dev */

  struct wdog_s         ec_txtimeout;   /* TX safety net (see above) */
  struct wdog_s         ec_rxidle;      /* Abandons a stalled reassembly */

  struct work_s         ec_rxwork;      /* Reassembled-frame delivery */
  struct work_s         ec_rxidlework;  /* ec_rxidle bottom half */
  struct work_s         ec_txdonework;  /* Resume/repoll after TX space */
  struct work_s         ec_pollwork;    /* Out-of-cycle txavail poll */

#ifdef CONFIG_CAN_ERRORS
  /* Collision detection and recovery: only meaningful when the bound
   * lower-half driver can report CAN bus errors. Without this, ethcan
   * cannot tell a collision apart from any other corruption -- see the
   * file header comment.
   */

  struct wdog_s         ec_backoff;     /* CSMA/CD retry backoff */
  struct work_s         ec_backoffwork; /* ec_backoff bottom half */
  struct work_s         ec_errwork;     /* Bus error bottom half */
  uint32_t              ec_errflags;    /* Pending CAN_ERROR_* bits */
  uint8_t               ec_retries;
#endif

  /* Fragmentation cursor for the frame currently staged for transmission.
   * ec_txstage is a private copy of the frame -- it must not alias
   * ec_dev.d_buf, which the network stack may reuse for the next poll
   * (or ethcan_deliver() may reuse for an incoming frame) before a
   * multi-fragment send finishes draining. ec_txlen != 0 means a frame
   * is staged, whether or not it has actually started going out yet
   * (ec_txstarted): a staged frame may still be waiting for the bus to
   * look idle, or (with CONFIG_CAN_ERRORS) waiting out a backoff after
   * a collision.
   */

  FAR uint8_t          *ec_txstage;
  uint16_t              ec_txlen;
  uint16_t              ec_txoff;
  bool                  ec_txstarted;

  struct ethcan_reasm_s ec_reasm;
  FAR uint8_t          *ec_rxbuf;
  bool                  ec_rxready;

  struct net_driver_s   ec_dev;         /* Interface understood by the
                                          * network */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct ethcan_driver_s *g_ethcan[CONFIG_NET_ETHCAN_NINTERFACES];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static FAR struct ethcan_driver_s *
  ethcan_find(FAR struct can_dev_s *candev);

static bool ethcan_bus_busy(FAR struct ethcan_driver_s *priv);
static bool ethcan_send_fragment(FAR struct ethcan_driver_s *priv);
static void ethcan_send_pending(FAR struct ethcan_driver_s *priv);
static void ethcan_kick_tx(FAR struct ethcan_driver_s *priv);
static void ethcan_start_frame(FAR struct ethcan_driver_s *priv,
                                FAR const uint8_t *buf, uint16_t len);

static int  ethcan_txpoll(FAR struct net_driver_s *dev);
static void ethcan_reply(FAR struct ethcan_driver_s *priv);
static void ethcan_deliver(FAR struct ethcan_driver_s *priv);

static void ethcan_rx_work(FAR void *arg);

static void ethcan_rxidle_work(FAR void *arg);
static void ethcan_rxidle_expiry(wdparm_t arg);

static void ethcan_txdone_work(FAR void *arg);

#ifdef CONFIG_CAN_ERRORS
static void ethcan_err_work(FAR void *arg);

static void ethcan_backoff_work(FAR void *arg);
static void ethcan_backoff_expiry(wdparm_t arg);
#endif

static void ethcan_txtimeout_work(FAR void *arg);
static void ethcan_txtimeout_expiry(wdparm_t arg);

static int  ethcan_ifup(FAR struct net_driver_s *dev);
static int  ethcan_ifdown(FAR struct net_driver_s *dev);

static void ethcan_txavail_work(FAR void *arg);
static int  ethcan_txavail(FAR struct net_driver_s *dev);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ethcan_find
 *
 * Description:
 *   Recover the ethcan_driver_s instance that owns a given lower-half
 *   struct can_dev_s.  can_receive() and can_txdone() are called by the
 *   lower-half driver with only the can_dev_s pointer, and cd_priv is
 *   already owned by that lower half, so ethcan keeps its own small
 *   lookup table instead.
 *
 ****************************************************************************/

static FAR struct ethcan_driver_s *
  ethcan_find(FAR struct can_dev_s *candev)
{
  int i;

  for (i = 0; i < CONFIG_NET_ETHCAN_NINTERFACES; i++)
    {
      if (g_ethcan[i] != NULL && g_ethcan[i]->ec_can == candev)
        {
          return g_ethcan[i];
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: ethcan_bus_busy
 *
 * Description:
 *   Best-effort "is someone else already sending" check: is a reassembly
 *   currently in progress. This cannot see a sender that has not yet put
 *   its first fragment on the wire, which is exactly the race the
 *   collision-and-backoff path exists to handle.
 *
 ****************************************************************************/

static bool ethcan_bus_busy(FAR struct ethcan_driver_s *priv)
{
  irqstate_t flags;
  bool busy;

  flags = spin_lock_irqsave(&priv->ec_lock);
  busy  = priv->ec_reasm.active;
  spin_unlock_irqrestore(&priv->ec_lock, flags);

  return busy;
}

/****************************************************************************
 * Name: ethcan_send_fragment
 *
 * Description:
 *   Send exactly one CAN FD fragment from the in-progress TX frame.
 *   Caller must have already verified dev_txready().
 *
 * Returned Value:
 *   true  - The frame is now fully sent (or was dropped on error);
 *           ec_txlen has been reset to 0.
 *   false - More fragments remain.
 *
 ****************************************************************************/

static bool ethcan_send_fragment(FAR struct ethcan_driver_s *priv)
{
  struct can_msg_s msg;
  uint16_t remaining;
  uint16_t fraglen;

  remaining = priv->ec_txlen - priv->ec_txoff;
  fraglen   = remaining > CAN_MAXDATALEN ? CAN_MAXDATALEN : remaining;

  memset(&msg, 0, sizeof(msg));
  msg.cm_hdr.ch_id    = ETHCAN_MKID(priv->ec_txoff == 0);
  msg.cm_hdr.ch_extid = 1;
  msg.cm_hdr.ch_dlc   = can_bytes2dlc(fraglen);
  msg.cm_hdr.ch_edl   = 1;
  memcpy(msg.cm_data, &priv->ec_txstage[priv->ec_txoff], fraglen);

  if (dev_send(priv->ec_can, &msg) < 0)
    {
      NETDEV_TXERRORS(&priv->ec_dev);
      priv->ec_txlen     = 0;
      priv->ec_txstarted = false;
      return true;
    }

  priv->ec_txoff += fraglen;

  if (priv->ec_txoff < priv->ec_txlen)
    {
      return false;
    }

  /* The frame is fully sent.  If the last fragment's length rounds up
   * to CAN FD's DLC 15 (64 bytes) on the wire, it looks identical to a
   * full, non-terminal fragment to the receiver, so push an explicit
   * empty terminator.
   */

  if (fraglen > ETHCAN_DLC15_THRESHOLD && dev_txready(priv->ec_can))
    {
      struct can_msg_s term;

      memset(&term, 0, sizeof(term));
      term.cm_hdr.ch_id    = ETHCAN_MKID(false);
      term.cm_hdr.ch_extid = 1;
      term.cm_hdr.ch_edl   = 1;
      dev_send(priv->ec_can, &term);
    }

  priv->ec_txlen     = 0;
  priv->ec_txstarted = false;
#ifdef CONFIG_CAN_ERRORS
  priv->ec_retries   = 0;
#endif
  return true;
}

/****************************************************************************
 * Name: ethcan_send_pending
 *
 * Description:
 *   Send as many fragments of the in-progress TX frame as the controller
 *   currently has room for.  If the controller runs out of room first,
 *   arm a safety-net timeout and rely on can_txdone() to resume.
 *
 ****************************************************************************/

static void ethcan_send_pending(FAR struct ethcan_driver_s *priv)
{
  while (priv->ec_txlen != 0 && dev_txready(priv->ec_can))
    {
      if (ethcan_send_fragment(priv))
        {
          break;
        }
    }

  if (priv->ec_txlen != 0 && priv->ec_txstarted)
    {
      wd_start(&priv->ec_txtimeout, ETHCAN_TXTIMEOUT,
               ethcan_txtimeout_expiry, (wdparm_t)priv);
    }
}

/****************************************************************************
 * Name: ethcan_kick_tx
 *
 * Description:
 *   Try to make progress on whatever frame is currently staged: start it
 *   if the bus looks idle and it has not started yet, or keep feeding an
 *   already-started burst. A no-op if nothing is staged, or if a start
 *   is still being deferred because the bus looks busy -- in which case
 *   ethcan_rx_work()/ethcan_rxidle_work() will call this again once that
 *   changes.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void ethcan_kick_tx(FAR struct ethcan_driver_s *priv)
{
  if (priv->ec_txlen == 0)
    {
      return;
    }

  if (!priv->ec_txstarted)
    {
      if (ethcan_bus_busy(priv))
        {
          return;
        }

      priv->ec_txstarted = true;
    }

  ethcan_send_pending(priv);
}

/****************************************************************************
 * Name: ethcan_start_frame
 *
 * Description:
 *   Stage a new outgoing Ethernet frame and try to send it.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void ethcan_start_frame(FAR struct ethcan_driver_s *priv,
                                FAR const uint8_t *buf, uint16_t len)
{
  if (priv->ec_txlen != 0)
    {
      /* A previous frame is still staged/draining.  Lossy channel: drop
       * the new one rather than corrupt the in-flight fragmentation
       * cursor.
       */

      NETDEV_TXERRORS(&priv->ec_dev);
      return;
    }

  memcpy(priv->ec_txstage, buf, len);
  priv->ec_txoff     = 0;
  priv->ec_txlen     = len;
  priv->ec_txstarted = false;
#ifdef CONFIG_CAN_ERRORS
  priv->ec_retries   = 0;
#endif

  NETDEV_TXPACKETS(&priv->ec_dev);
  ethcan_kick_tx(priv);
}

/****************************************************************************
 * Name: ethcan_txpoll
 *
 * Description:
 *   Callback from devif_poll(): the network has a packet ready to send.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int ethcan_txpoll(FAR struct net_driver_s *dev)
{
  FAR struct ethcan_driver_s *priv =
    (FAR struct ethcan_driver_s *)dev->d_private;

  if (dev->d_len > 0)
    {
      ethcan_start_frame(priv, dev->d_buf, dev->d_len);
    }

  return 0;
}

/****************************************************************************
 * Name: ethcan_reply
 *
 * Description:
 *   After a received frame has been dispatched to the network, send any
 *   reply (e.g. an ARP or ping response) that it produced in d_buf.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void ethcan_reply(FAR struct ethcan_driver_s *priv)
{
  if (priv->ec_dev.d_len > 0)
    {
      ethcan_start_frame(priv, priv->ec_dev.d_buf, priv->ec_dev.d_len);
    }
}

/****************************************************************************
 * Name: ethcan_deliver
 *
 * Description:
 *   Hand one reassembled frame up to the network stack.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void ethcan_deliver(FAR struct ethcan_driver_s *priv)
{
  irqstate_t flags;
  uint16_t len;

  flags = spin_lock_irqsave(&priv->ec_lock);
  len   = priv->ec_reasm.len;
  spin_unlock_irqrestore(&priv->ec_lock, flags);

  if (len < sizeof(struct eth_hdr_s) || len > CONFIG_NET_ETHCAN_PKTSIZE)
    {
      NETDEV_RXDROPPED(&priv->ec_dev);
      return;
    }

  memcpy(priv->ec_dev.d_buf, priv->ec_rxbuf, len);
  priv->ec_dev.d_len = len;

#ifdef CONFIG_NET_PKT
  pkt_input(&priv->ec_dev);
#endif

#ifdef CONFIG_NET_IPv4
  if (BUF->type == HTONS(ETHTYPE_IP))
    {
      NETDEV_RXIPV4(&priv->ec_dev);
      ipv4_input(&priv->ec_dev);
      ethcan_reply(priv);
    }
  else
#endif
#ifdef CONFIG_NET_IPv6
  if (BUF->type == HTONS(ETHTYPE_IP6))
    {
      NETDEV_RXIPV6(&priv->ec_dev);
      ipv6_input(&priv->ec_dev);
      ethcan_reply(priv);
    }
  else
#endif
#ifdef CONFIG_NET_ARP
  if (BUF->type == HTONS(ETHTYPE_ARP))
    {
      arp_input(&priv->ec_dev);
      NETDEV_RXARP(&priv->ec_dev);
      ethcan_reply(priv);
    }
  else
#endif
    {
      NETDEV_RXDROPPED(&priv->ec_dev);
    }
}

/****************************************************************************
 * Name: ethcan_rx_work
 *
 * Description:
 *   Work queue bottom half for can_receive(): deliver a completed
 *   reassembly, then see if a staged TX frame can now start -- the bus
 *   just looked busy from this node's point of view and may not any
 *   more.  Deferred here (rather than done directly in can_receive())
 *   because can_receive() may run in a context -- interrupt or
 *   otherwise -- unsuitable for taking the network lock or calling into
 *   the IP stack.
 *
 ****************************************************************************/

static void ethcan_rx_work(FAR void *arg)
{
  FAR struct ethcan_driver_s *priv = (FAR struct ethcan_driver_s *)arg;
  irqstate_t flags;
  bool ready;

  netdev_lock(&priv->ec_dev);

  flags = spin_lock_irqsave(&priv->ec_lock);
  ready = priv->ec_rxready;
  priv->ec_rxready = false;
  spin_unlock_irqrestore(&priv->ec_lock, flags);

  if (ready)
    {
      ethcan_deliver(priv);
    }

  ethcan_kick_tx(priv);

  netdev_unlock(&priv->ec_dev);
}

/****************************************************************************
 * Name: ethcan_rxidle_work / ethcan_rxidle_expiry
 *
 * Description:
 *   A reassembly that has gone quiet for too long (its terminator was
 *   presumably lost) is abandoned, so it cannot wedge ethcan_bus_busy()
 *   into permanently deferring this node's own sends.
 *
 ****************************************************************************/

static void ethcan_rxidle_work(FAR void *arg)
{
  FAR struct ethcan_driver_s *priv = (FAR struct ethcan_driver_s *)arg;
  irqstate_t flags;

  netdev_lock(&priv->ec_dev);

  flags = spin_lock_irqsave(&priv->ec_lock);
  priv->ec_reasm.active = false;
  spin_unlock_irqrestore(&priv->ec_lock, flags);

  NETDEV_RXDROPPED(&priv->ec_dev);
  ethcan_kick_tx(priv);

  netdev_unlock(&priv->ec_dev);
}

static void ethcan_rxidle_expiry(wdparm_t arg)
{
  FAR struct ethcan_driver_s *priv = (FAR struct ethcan_driver_s *)arg;

  work_queue(ETHCANWORK, &priv->ec_rxidlework, ethcan_rxidle_work,
             priv, 0);
}

/****************************************************************************
 * Name: can_receive
 *
 * Description:
 *   ethcan's implementation of the lower-half CAN receive callback (see
 *   include/nuttx/can/can.h).  Called by the bound controller's lower-half
 *   driver for every CAN frame it receives -- including, if CAN_ERRORS is
 *   enabled, error reports, which arrive here with ch_error set and ch_id
 *   reinterpreted as a CAN_ERROR_* bitmask -- and may run in any context
 *   that driver chooses, including hard interrupt context, so this
 *   function only ever touches ec_lock-protected state and defers
 *   everything else to a work queue.
 *
 ****************************************************************************/

int can_receive(FAR struct can_dev_s *dev, FAR struct can_hdr_s *hdr,
                 FAR uint8_t *data)
{
  FAR struct ethcan_driver_s *priv = ethcan_find(dev);
  irqstate_t flags;
  bool sof;
  uint8_t nbytes;
  bool rxready = false;

  if (priv == NULL)
    {
      return OK;
    }

#ifdef CONFIG_CAN_ERRORS
  if (hdr->ch_error)
    {
      flags = spin_lock_irqsave(&priv->ec_lock);
      priv->ec_errflags |= hdr->ch_id;
      spin_unlock_irqrestore(&priv->ec_lock, flags);

      work_queue(ETHCANWORK, &priv->ec_errwork, ethcan_err_work, priv, 0);
      return OK;
    }
#endif

  if (!hdr->ch_extid || hdr->ch_rtr)
    {
      return OK;
    }

  sof    = (hdr->ch_id & ETHCAN_SOF_BIT) != 0;
  nbytes = can_dlc2bytes(hdr->ch_dlc);

  flags = spin_lock_irqsave(&priv->ec_lock);

  if (sof)
    {
      priv->ec_reasm.active = true;
      priv->ec_reasm.len    = 0;
    }

  if (priv->ec_reasm.active)
    {
      if ((unsigned int)priv->ec_reasm.len + nbytes >
          CONFIG_NET_ETHCAN_PKTSIZE)
        {
          priv->ec_reasm.active = false;
        }
      else
        {
          memcpy(&priv->ec_rxbuf[priv->ec_reasm.len], data, nbytes);
          priv->ec_reasm.len += nbytes;

          if (nbytes < CAN_MAXDATALEN)
            {
              priv->ec_reasm.active = false;
              priv->ec_rxready      = true;
              rxready                = true;
            }
        }
    }

  spin_unlock_irqrestore(&priv->ec_lock, flags);

  if (priv->ec_reasm.active || rxready)
    {
      wd_start(&priv->ec_rxidle, ETHCAN_RXIDLE_TIMEOUT,
               ethcan_rxidle_expiry, (wdparm_t)priv);
    }
  else
    {
      wd_cancel(&priv->ec_rxidle);
    }

  if (rxready)
    {
      work_queue(ETHCANWORK, &priv->ec_rxwork, ethcan_rx_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: ethcan_txdone_work
 ****************************************************************************/

static void ethcan_txdone_work(FAR void *arg)
{
  FAR struct ethcan_driver_s *priv = (FAR struct ethcan_driver_s *)arg;

  netdev_lock(&priv->ec_dev);

  wd_cancel(&priv->ec_txtimeout);

  if (priv->ec_txlen != 0 && priv->ec_txstarted)
    {
      /* Resume a fragmented send that stalled for lack of TX space */

      ethcan_send_pending(priv);
    }

  if (priv->ec_txlen == 0)
    {
      /* Fully drained: ask the network for the next frame */

      devif_poll(&priv->ec_dev, ethcan_txpoll);
    }

  netdev_unlock(&priv->ec_dev);
}

/****************************************************************************
 * Name: can_txdone
 *
 * Description:
 *   ethcan's implementation of the lower-half CAN transmit-done callback
 *   (see include/nuttx/can/can.h).  Called by the bound controller's
 *   lower-half driver when it has room for another TX frame; may run in
 *   any context, so the actual work is deferred to ethcan_txdone_work().
 *
 ****************************************************************************/

int can_txdone(FAR struct can_dev_s *dev)
{
  FAR struct ethcan_driver_s *priv = ethcan_find(dev);

  if (priv == NULL)
    {
      return OK;
    }

  NETDEV_TXDONE(&priv->ec_dev);
  work_queue(ETHCANWORK, &priv->ec_txdonework, ethcan_txdone_work, priv, 0);
  return OK;
}

#ifdef CONFIG_CAN_ERRORS
/****************************************************************************
 * Name: ethcan_backoff_work / ethcan_backoff_expiry
 *
 * Description:
 *   A collision retry's backoff has elapsed; try the staged frame again.
 *
 ****************************************************************************/

static void ethcan_backoff_work(FAR void *arg)
{
  FAR struct ethcan_driver_s *priv = (FAR struct ethcan_driver_s *)arg;

  netdev_lock(&priv->ec_dev);
  ethcan_kick_tx(priv);
  netdev_unlock(&priv->ec_dev);
}

static void ethcan_backoff_expiry(wdparm_t arg)
{
  FAR struct ethcan_driver_s *priv = (FAR struct ethcan_driver_s *)arg;

  work_queue(ETHCANWORK, &priv->ec_backoffwork, ethcan_backoff_work,
             priv, 0);
}

/****************************************************************************
 * Name: ethcan_err_work
 *
 * Description:
 *   Work queue bottom half for a CAN error report.  A bus-off report is
 *   recovered from unconditionally, regardless of what this node was
 *   doing.  Any other error arriving while this node's own send is in
 *   flight is conservatively assumed to be a collision -- struct
 *   can_hdr_s's error report has no way to say otherwise -- so the burst
 *   is abandoned and retried after a random backoff, up to
 *   ETHCAN_MAXRETRIES times before the frame is dropped.
 *
 ****************************************************************************/

static void ethcan_err_work(FAR void *arg)
{
  FAR struct ethcan_driver_s *priv = (FAR struct ethcan_driver_s *)arg;
  irqstate_t flags;
  uint32_t errflags;

  netdev_lock(&priv->ec_dev);

  flags    = spin_lock_irqsave(&priv->ec_lock);
  errflags = priv->ec_errflags;
  priv->ec_errflags = 0;
  spin_unlock_irqrestore(&priv->ec_lock, flags);

  if ((errflags & CAN_ERROR_BUSOFF) != 0)
    {
      dev_rxint(priv->ec_can, false);
      dev_shutdown(priv->ec_can);
      dev_reset(priv->ec_can);
      dev_setup(priv->ec_can);
      dev_rxint(priv->ec_can, true);

      flags = spin_lock_irqsave(&priv->ec_lock);
      priv->ec_reasm.active = false;
      spin_unlock_irqrestore(&priv->ec_lock, flags);
      wd_cancel(&priv->ec_rxidle);
    }

  if (priv->ec_txlen != 0 && priv->ec_txstarted)
    {
      wd_cancel(&priv->ec_txtimeout);
      priv->ec_txoff     = 0;
      priv->ec_txstarted = false;

      if (priv->ec_retries < ETHCAN_MAXRETRIES)
        {
          int backoffbit = priv->ec_retries > ETHCAN_BACKOFF_MAXBIT ?
                            ETHCAN_BACKOFF_MAXBIT : priv->ec_retries;
          int window     = 1 << backoffbit;
          clock_t delay  = ((clock_t)(rand() % window) + 1) *
                            ETHCAN_BACKOFF_SLOT;

          priv->ec_retries++;
          NETDEV_TXERRORS(&priv->ec_dev);
          wd_start(&priv->ec_backoff, delay, ethcan_backoff_expiry,
                   (wdparm_t)priv);
        }
      else
        {
          NETDEV_TXERRORS(&priv->ec_dev);
          priv->ec_txlen   = 0;
          priv->ec_retries = 0;
          devif_poll(&priv->ec_dev, ethcan_txpoll);
        }
    }

  netdev_unlock(&priv->ec_dev);
}
#endif /* CONFIG_CAN_ERRORS */

/****************************************************************************
 * Name: ethcan_txtimeout_work / ethcan_txtimeout_expiry
 *
 * Description:
 *   Safety net for the case where a fragmented send stalls waiting for
 *   TX space and can_txdone() never arrives (e.g. a lower-half driver
 *   quirk, or the far end of a wedged bus).  Give up on that frame and
 *   go back to polling -- consistent with this driver's lossy-channel,
 *   no-guarantees design.
 *
 ****************************************************************************/

static void ethcan_txtimeout_work(FAR void *arg)
{
  FAR struct ethcan_driver_s *priv = (FAR struct ethcan_driver_s *)arg;

  netdev_lock(&priv->ec_dev);

  NETDEV_TXTIMEOUTS(&priv->ec_dev);
  priv->ec_txlen     = 0;
  priv->ec_txstarted = false;
#ifdef CONFIG_CAN_ERRORS
  priv->ec_retries   = 0;
#endif

  devif_poll(&priv->ec_dev, ethcan_txpoll);

  netdev_unlock(&priv->ec_dev);
}

static void ethcan_txtimeout_expiry(wdparm_t arg)
{
  FAR struct ethcan_driver_s *priv = (FAR struct ethcan_driver_s *)arg;

  work_queue(ETHCANWORK, &priv->ec_txdonework, ethcan_txtimeout_work,
             priv, 0);
}

/****************************************************************************
 * Name: ethcan_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the interface when an IP address is
 *   assigned (or unconditionally, for interfaces without one).
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int ethcan_ifup(FAR struct net_driver_s *dev)
{
  FAR struct ethcan_driver_s *priv =
    (FAR struct ethcan_driver_s *)dev->d_private;

  dev_reset(priv->ec_can);
  dev_setup(priv->ec_can);
  dev_rxint(priv->ec_can, true);

  priv->ec_bifup = true;
  netdev_carrier_on(dev);

  return OK;
}

/****************************************************************************
 * Name: ethcan_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int ethcan_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct ethcan_driver_s *priv =
    (FAR struct ethcan_driver_s *)dev->d_private;
  irqstate_t flags;

  dev_rxint(priv->ec_can, false);
  dev_shutdown(priv->ec_can);

  wd_cancel(&priv->ec_txtimeout);
  wd_cancel(&priv->ec_rxidle);
#ifdef CONFIG_CAN_ERRORS
  wd_cancel(&priv->ec_backoff);
#endif

  flags = spin_lock_irqsave(&priv->ec_lock);
  priv->ec_txlen         = 0;
  priv->ec_txstarted     = false;
  priv->ec_rxready       = false;
  priv->ec_reasm.active  = false;
#ifdef CONFIG_CAN_ERRORS
  priv->ec_retries       = 0;
  priv->ec_errflags      = 0;
#endif
  spin_unlock_irqrestore(&priv->ec_lock, flags);

  priv->ec_bifup = false;
  netdev_carrier_off(dev);

  return OK;
}

/****************************************************************************
 * Name: ethcan_txavail_work / ethcan_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available, to perform an
 *   out-of-cycle poll and thereby reduce TX latency.
 *
 ****************************************************************************/

static void ethcan_txavail_work(FAR void *arg)
{
  FAR struct ethcan_driver_s *priv = (FAR struct ethcan_driver_s *)arg;

  netdev_lock(&priv->ec_dev);

  if (priv->ec_bifup && priv->ec_txlen == 0)
    {
      devif_poll(&priv->ec_dev, ethcan_txpoll);
    }

  netdev_unlock(&priv->ec_dev);
}

static int ethcan_txavail(FAR struct net_driver_s *dev)
{
  FAR struct ethcan_driver_s *priv =
    (FAR struct ethcan_driver_s *)dev->d_private;

  if (work_available(&priv->ec_pollwork))
    {
      work_queue(ETHCANWORK, &priv->ec_pollwork, ethcan_txavail_work,
                 priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ethcan_initialize
 *
 * See include/nuttx/net/ethcan.h for a full description.
 *
 ****************************************************************************/

int ethcan_initialize(int intf, FAR struct can_dev_s *candev,
                       FAR const uint8_t *macaddr)
{
  FAR struct ethcan_driver_s *priv;
  int ret;

  if (intf < 0 || intf >= CONFIG_NET_ETHCAN_NINTERFACES || candev == NULL)
    {
      return -EINVAL;
    }

  if (g_ethcan[intf] != NULL || ethcan_find(candev) != NULL)
    {
      return -EBUSY;
    }

  priv = kmm_zalloc(sizeof(struct ethcan_driver_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->ec_dev.d_buf = kmm_malloc(PKTBUF_SIZE);
  priv->ec_txstage   = kmm_malloc(CONFIG_NET_ETHCAN_PKTSIZE);
  priv->ec_rxbuf     = kmm_malloc(CONFIG_NET_ETHCAN_PKTSIZE);

  if (priv->ec_dev.d_buf == NULL || priv->ec_txstage == NULL ||
      priv->ec_rxbuf == NULL)
    {
      ret = -ENOMEM;
      goto errout;
    }

  priv->ec_can = candev;

  spin_lock_init(&priv->ec_lock);

  priv->ec_dev.d_ifup    = ethcan_ifup;
  priv->ec_dev.d_ifdown  = ethcan_ifdown;
  priv->ec_dev.d_txavail = ethcan_txavail;
  priv->ec_dev.d_private = priv;

  memcpy(priv->ec_dev.d_mac.ether.ether_addr_octet, macaddr, 6);

  g_ethcan[intf] = priv;

  ret = netdev_register(&priv->ec_dev, NET_LL_ETHERNET);
  if (ret >= 0)
    {
      return OK;
    }

  g_ethcan[intf] = NULL;

errout:
  kmm_free(priv->ec_dev.d_buf);
  kmm_free(priv->ec_txstage);
  kmm_free(priv->ec_rxbuf);
  kmm_free(priv);
  return ret;
}

#endif /* CONFIG_NET_ETHCAN */
