#define DEBUG 1

#include "rio.h"

#include "object.h"
#include "regops.h"

enum sp_tx_fifo_state {
    SP_TX_FIFO_STATE_IDLE   = 0x0,
    SP_TX_FIFO_STATE_ARMED  = 0x1,
    SP_TX_FIFO_STATE_ACTIVE = 0x2,
};

enum sp_rx_fifo_state {
    SP_RX_FIFO_STATE_IDLE   = 0x0,
    SP_RX_FIFO_STATE_ACTIVE = 0x1,
};

REG32(IR_SP_TX_CTRL, 0x107000)
    FIELD(IR_SP_TX_CTRL, OCTETS_TO_SEND,         0, 16)
REG32(IR_SP_TX_STAT, 0x107004)
    FIELD(IR_SP_TX_STAT, OCTETS_REMAINING,       0, 16)
    FIELD(IR_SP_TX_STAT, BUFFERS_FILLED,        16,  4)
    FIELD(IR_SP_TX_STAT, FULL,                  27,  1)
    FIELD(IR_SP_TX_STAT, TX_FIFO_STATE,         28,  4)
REG32(IR_SP_TX_DATA, 0x107008)
    FIELD(IR_SP_TX_DATA, DATA,                   0, 32)
REG32(IR_SP_RX_CTRL, 0x10700c)
    FIELD(IR_SP_RX_CTRL, OVERLFOW_MODE,         31,  1)
REG32(IR_SP_RX_STAT, 0x107010)
    FIELD(IR_SP_RX_STAT, OCTETS_REMAINING,       0, 16)
    FIELD(IR_SP_RX_STAT, BUFFERS_FILLED,        16,  4)
    FIELD(IR_SP_RX_STAT, FULL,                  27,  1)
    FIELD(IR_SP_RX_STAT, RX_FIFO_STATE,         28,  4)
REG32(IR_SP_RX_DATA, 0x107014)
    FIELD(IR_SP_RX_DATA, DATA,                   0, 32)

/* See Chapter 4 in RIO Spec P1.
   Also see Chapter 14 (assuming little-endian byte order in words) */
FIELD(RIO_PKT_WORD0, PRIO,        0,  2)
FIELD(RIO_PKT_WORD0, TT,          2,  2)
FIELD(RIO_PKT_WORD0, FTYPE,       4,  4)
FIELD(RIO_PKT_WORD0, DEST_ID,     8,  8)
FIELD(RIO_PKT_WORD0, SRC_ID,      16, 8)
FIELD(RIO_PKT_WORD0, TRANSACTION, 24, 4)
FIELD(RIO_PKT_WORD0, STATUS,      28, 4)
FIELD(RIO_PKT_WORD1, TARGET_TID,  0, 8)
FIELD(RIO_PKT_WORD1, DATA_3LSB,   8, 24)
FIELD(RIO_PKT_WORD2, DATA_4MID,   0, 32)
FIELD(RIO_PKT_WORD3, DATA_1MSB,   0, 8)

#define MAX_RIO_ENDPOINTS 2

static struct rio_ep rio_eps[MAX_RIO_ENDPOINTS] = {0};

#define RIO_MAX_PKT_SIZE 64 /* TODO */
#define PKT_BUF_WORDS (RIO_MAX_PKT_SIZE / sizeof(uint32_t))
uint32_t pkt_buf[PKT_BUF_WORDS];

static unsigned pack_pkt(uint32_t *buf, unsigned buf_size, struct rio_pkt *pkt)
{
    unsigned w = 0;

    switch (pkt->ftype) {
      case RIO_FTYPE_MAINT:

        ASSERT(w < buf_size);
        pkt_buf[w++] = ((pkt->prio << RIO_PKT_WORD0__PRIO__SHIFT) & RIO_PKT_WORD0__PRIO__MASK) |
            ((pkt->ttype << RIO_PKT_WORD0__TT__SHIFT) & RIO_PKT_WORD0__TT__MASK) |
            ((pkt->ftype << RIO_PKT_WORD0__FTYPE__SHIFT) & RIO_PKT_WORD0__FTYPE__MASK) |
            ((pkt->dest_id << RIO_PKT_WORD0__DEST_ID__SHIFT) &
                            RIO_PKT_WORD0__DEST_ID__MASK) |
            ((pkt->src_id << RIO_PKT_WORD0__SRC_ID__SHIFT) &
                                    RIO_PKT_WORD0__SRC_ID__MASK) |
            ((pkt->transaction << RIO_PKT_WORD0__TRANSACTION__SHIFT) &
                                    RIO_PKT_WORD0__TRANSACTION__MASK) |
            ((pkt->status << RIO_PKT_WORD0__STATUS__SHIFT) &
                                    RIO_PKT_WORD0__STATUS__MASK);
        ASSERT(w < buf_size);
        pkt_buf[w] = ((pkt->target_tid << RIO_PKT_WORD1__TARGET_TID__SHIFT) &
                                    RIO_PKT_WORD1__TARGET_TID__MASK);

        for (unsigned d = 0; d < pkt->payload_len; ++d) {
            uint64_t data = pkt->payload[d];
            for (int i = 0; i < 3; ++i) {
                pkt_buf[w] |= (data & 0xff) << (RIO_PKT_WORD1__DATA_3LSB__SHIFT + 8 * i);
                data >>= 8;
            }
            ++w;
            ASSERT(w < buf_size);
            pkt_buf[w] = 0;
            for (int i = 0; i < 4; ++i) {
                pkt_buf[w] |= (data & 0xff) << (RIO_PKT_WORD2__DATA_4MID__SHIFT + 8 * i);
                data >>= 8;
            }
            ++w;
            ASSERT(w < buf_size);
            pkt_buf[w] |= (data & 0xff) << RIO_PKT_WORD3__DATA_1MSB__SHIFT;
        }
        break;
      default:
        ASSERT(!"pkt type not implemented");
    }

    return w + 1; /* index of last filled word to length */
}

static int unpack_pkt(struct rio_pkt *pkt, uint32_t *buf, unsigned buf_len)
{
    unsigned w = 0;

    ASSERT(buf_len > 1); /* Fixed-length fields make up at least one word */

    /* Transport layer */
    pkt->prio = FIELD_EX32(pkt_buf[w], RIO_PKT_WORD0, PRIO);
    pkt->ttype = FIELD_EX32(pkt_buf[w], RIO_PKT_WORD0, TT);

    /* Logical layer */
    pkt->ftype = FIELD_EX32(pkt_buf[w], RIO_PKT_WORD0, FTYPE);

    switch (pkt->ftype) {
        case RIO_FTYPE_MAINT:
            pkt->dest_id = FIELD_EX32(pkt_buf[w], RIO_PKT_WORD0, DEST_ID);
            pkt->src_id = FIELD_EX32(pkt_buf[w], RIO_PKT_WORD0, SRC_ID);
            pkt->transaction = FIELD_EX32(pkt_buf[w], RIO_PKT_WORD0, TRANSACTION);
            pkt->status = FIELD_EX32(pkt_buf[w], RIO_PKT_WORD0, STATUS);
            ++w;
            pkt->target_tid = FIELD_EX32(pkt_buf[w], RIO_PKT_WORD1, TARGET_TID);

            pkt->payload_len = 0;
            while (w + 1 < buf_len) {
                DPRINTF("RIO: decoding word %u out of %u\r\n", w, buf_len);

				uint64_t data = 0;
				data |= (pkt_buf[w + 2] >> RIO_PKT_WORD3__DATA_1MSB__SHIFT) & 0xff;

                ASSERT(w < buf_len);
                for (int i = 4 - 1; i >= 0; --i) {
                    data <<= 8;
                    data |= (pkt_buf[w + 1] >> (RIO_PKT_WORD2__DATA_4MID__SHIFT + 8 * i)) & 0xff;
                }

                for (int i = 3 - 1; i >= 0; --i) {
                    data <<= 8;
                    data |= (pkt_buf[w] >> (RIO_PKT_WORD1__DATA_3LSB__SHIFT + 8 * i)) & 0xff;
                }
                w += 2;

                pkt->payload[pkt->payload_len++] = data;
            }
            break;
        default:
            ASSERT(!"pkt type not implemented");
    }

    return 0;
}

void rio_print_pkt(struct rio_pkt *pkt)
{
    switch (pkt->ftype) {
        case RIO_FTYPE_MAINT:
            printf("RIO PKT: payload len %u\r\n"
                   "\tftype %x\r\n"
                   "\ttransaction %x\r\n"
                   "\tsrc_id %x\r\n"
                   "\ttarget_id %x\r\n"
                   "\tsrc_tid %x\r\n"
                   "\ttarget_tid %x\r\n"
                   "\tsize %x\r\n"
                   "\tconfig_offset %x\r\n"
				   "\tpayload %08x%08x ...\r\n",
                   pkt->payload_len,
                   pkt->ftype, pkt->transaction,
                   pkt->src_id, pkt->dest_id,
                   pkt->src_tid, pkt->target_tid,
                   pkt->size, pkt->config_offset,
				   (uint32_t)(pkt->payload[0] >> 32),
				   (uint32_t)(pkt->payload[0] & 0xffffffff));
            break;
        default:
            ASSERT(!"pkt type not implemented");
    }
}

struct rio_ep *rio_ep_create(const char *name, volatile uint32_t *base)
{
    struct rio_ep *ep = OBJECT_ALLOC(rio_eps);
    if (!ep)
        return NULL;
    ep->name = name;
    ep->base = base;
    printf("RIO EP %s: created\r\n", ep->name);
    return ep;
}

int rio_ep_destroy(struct rio_ep *ep)
{
    ASSERT(ep);
    printf("RIO EP %s: destroy\r\n", ep->name);
    OBJECT_FREE(ep);
    return 0;
}

int rio_ep_sp_send(struct rio_ep *ep, struct rio_pkt *pkt)
{
    unsigned pkt_len = pack_pkt(pkt_buf, PKT_BUF_WORDS, pkt);
    DPRINTF("RIO EP %s: packed packet into %u words...\r\n", ep->name, pkt_len);

    DPRINTF("RIO EP %s: waiting for space in FIFO...\r\n", ep->name);
    while (REGB_READ32(ep->base, IR_SP_TX_STAT) & IR_SP_TX_STAT__FULL__MASK);

    if ((REGB_READ32(ep->base, IR_SP_TX_STAT) & IR_SP_TX_STAT__TX_FIFO_STATE__MASK)
            != SP_TX_FIFO_STATE_IDLE) {
        printf("RIO EP %s: ERROR: SP send: TX FIFO not idle before enqueue\r\n",
               ep->name);
        return 1;
    }

    REGB_WRITE32(ep->base, IR_SP_TX_CTRL,
        ((pkt_len * 4) << IR_SP_TX_CTRL__OCTETS_TO_SEND__SHIFT)
            & IR_SP_TX_CTRL__OCTETS_TO_SEND__MASK);
    for (int i = 0; i < pkt_len; ++i) {
        REGB_WRITE32(ep->base, IR_SP_TX_DATA, pkt_buf[i]);
    };

    if ((REGB_READ32(ep->base, IR_SP_TX_STAT) & IR_SP_TX_STAT__TX_FIFO_STATE__MASK)
            != SP_TX_FIFO_STATE_IDLE) {
        printf("RIO: ERROR: SP send: TX FIFO not idle after enqueue\r\n");
        return 2;
    }

    return 0;
}

int rio_ep_sp_recv(struct rio_ep *ep, struct rio_pkt *pkt)
{
    DPRINTF("RIO EP %s: waiting for packet in RX FIFO...\r\n", ep->name);
    while ((REGB_READ32(ep->base, IR_SP_RX_STAT) & IR_SP_TX_STAT__BUFFERS_FILLED__MASK) == 0);

    if ((REGB_READ32(ep->base, IR_SP_RX_STAT) & IR_SP_RX_STAT__RX_FIFO_STATE__MASK)
            != SP_RX_FIFO_STATE_IDLE) {
        printf("RIO: ERROR: SP recv: RX FIFO not idle before dequeue\r\n");
        return 1;
    }
    
    int pkt_len = 0;
    pkt_buf[pkt_len++] = REGB_READ32(ep->base, IR_SP_RX_DATA);

    while ((REGB_READ32(ep->base, IR_SP_RX_STAT) & IR_SP_RX_STAT__OCTETS_REMAINING__MASK) > 0) {
        pkt_buf[pkt_len++] = REGB_READ32(ep->base, IR_SP_RX_DATA);
    }

    if ((REGB_READ32(ep->base, IR_SP_RX_STAT) & IR_SP_RX_STAT__RX_FIFO_STATE__MASK)
            != SP_RX_FIFO_STATE_IDLE) {
        printf("RIO: ERROR: SP recv: RX FIFO not idle after dequeue\r\n");
        return 2;
    }

    return unpack_pkt(pkt, pkt_buf, pkt_len);
}
