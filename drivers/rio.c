#define DEBUG 1

#include <stdbool.h>
#include <stdint.h>

#include "rio.h"

#include "object.h"
#include "mem.h"
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
   Also see Chapter 14 (assuming little-endian byte order in words)
   Note that some fields overlap, being in different packet types. */
FIELD(RIO_PKT_WORD0, PRIO,        	0,  2)
FIELD(RIO_PKT_WORD0, TTYPE,        	2,  2)
FIELD(RIO_PKT_WORD0, FTYPE,       	4,  4)
FIELD(RIO_PKT_WORD0, DEST_ID,     	8,  8)
FIELD(RIO_PKT_WORD0, SRC_ID,      	16, 8)
FIELD(RIO_PKT_WORD0, TRANSACTION, 	24, 4)
FIELD(RIO_PKT_WORD0, STATUS,      	28, 4)
FIELD(RIO_PKT_WORD0, RDWRSIZE,     	28, 4)
FIELD(RIO_PKT_WORD1, SRC_TID,  		0,  8)
FIELD(RIO_PKT_WORD1, TARGET_TID,  	0,  8)
FIELD(RIO_PKT_WORD1, DATA_3LSB,     8, 24)
FIELD(RIO_PKT_WORD1, CONFIG_OFFSET, 8, 21)
FIELD(RIO_PKT_WORD1, WDPTR,         29, 1)
FIELD(RIO_PKT_WORD2, DATA_4MID,     0, 32)
FIELD(RIO_PKT_WORD3, DATA_1MSB,     0, 8)

enum access_type {
    ACCESS_READ,
    ACCESS_WRITE,
};

#define MAX_RIO_ENDPOINTS 2

static struct rio_ep rio_eps[MAX_RIO_ENDPOINTS] = {0};

#define RIO_MAX_PKT_SIZE 64 /* TODO */
#define PKT_BUF_WORDS (RIO_MAX_PKT_SIZE / sizeof(uint32_t))
uint32_t pkt_buf[PKT_BUF_WORDS];

static struct rio_pkt in_pkt, out_pkt;

#define DIV_CEIL(x, y) ((x) / (y) + ((x) % (y) ? 1 : 0))

#define RDWR_SIZE_INDEX(wdptr, rdwrsize) ((wdptr << 4) | (rdwrsize))
#define RDWR_SIZE_INDEX_WDPTR(index) (((index) & (1 << 4)) >> 4)
#define RDWR_SIZE_INDEX_RDWRSIZE(index) ((index) & 0b1111)

#define RDWR_SIZE_ENTRY(num_bytes, shift) \
    { .bytes = num_bytes, .mask = (((1ULL << (num_bytes * 8)) - 1) << shift) }
#define RDWR_SIZE_ENTRY_DW(num_bytes) \
    { .bytes = num_bytes, .mask = ~0x0 }

/* Tables 4-3, 4-4 in Spec */
typedef struct RdWrSizeEntry {
    uint16_t bytes;
    uint64_t mask;
} RdWrSizeEntry;

const RdWrSizeEntry rdwr_size_table[] = {
    [RDWR_SIZE_INDEX(0b0, 0b0000)] = RDWR_SIZE_ENTRY(     1, 8 * 7 ),
    [RDWR_SIZE_INDEX(0b0, 0b0001)] = RDWR_SIZE_ENTRY(     1, 8 * 6 ),
    [RDWR_SIZE_INDEX(0b0, 0b0010)] = RDWR_SIZE_ENTRY(     1, 8 * 5 ),
    [RDWR_SIZE_INDEX(0b0, 0b0011)] = RDWR_SIZE_ENTRY(     1, 8 * 4 ),
    [RDWR_SIZE_INDEX(0b1, 0b0000)] = RDWR_SIZE_ENTRY(     1, 8 * 3 ),
    [RDWR_SIZE_INDEX(0b1, 0b0001)] = RDWR_SIZE_ENTRY(     1, 8 * 2 ),
    [RDWR_SIZE_INDEX(0b1, 0b0010)] = RDWR_SIZE_ENTRY(     1, 8 * 1 ),
    [RDWR_SIZE_INDEX(0b1, 0b0011)] = RDWR_SIZE_ENTRY(     1, 8 * 0 ),
    [RDWR_SIZE_INDEX(0b0, 0b0100)] = RDWR_SIZE_ENTRY(     2, 8 * 6 ),
    [RDWR_SIZE_INDEX(0b0, 0b0101)] = RDWR_SIZE_ENTRY(     3, 8 * 5 ),
    [RDWR_SIZE_INDEX(0b0, 0b0110)] = RDWR_SIZE_ENTRY(     2, 8 * 4 ),
    [RDWR_SIZE_INDEX(0b0, 0b0111)] = RDWR_SIZE_ENTRY(     5, 8 * 3 ),
    [RDWR_SIZE_INDEX(0b1, 0b0100)] = RDWR_SIZE_ENTRY(     2, 8 * 2 ),
    [RDWR_SIZE_INDEX(0b1, 0b0101)] = RDWR_SIZE_ENTRY(     3, 8 * 0 ),
    [RDWR_SIZE_INDEX(0b1, 0b0110)] = RDWR_SIZE_ENTRY(     2, 8 * 0 ),
    [RDWR_SIZE_INDEX(0b1, 0b0111)] = RDWR_SIZE_ENTRY(     5, 8 * 0 ),
    [RDWR_SIZE_INDEX(0b0, 0b1000)] = RDWR_SIZE_ENTRY(     4, 8 * 4 ),
    [RDWR_SIZE_INDEX(0b1, 0b1000)] = RDWR_SIZE_ENTRY(     4, 8 * 0 ),
    [RDWR_SIZE_INDEX(0b0, 0b1001)] = RDWR_SIZE_ENTRY(     6, 8 * 2 ),
    [RDWR_SIZE_INDEX(0b1, 0b1001)] = RDWR_SIZE_ENTRY(     6, 8 * 0 ),
    [RDWR_SIZE_INDEX(0b0, 0b1010)] = RDWR_SIZE_ENTRY(     7, 8 * 1 ),
    [RDWR_SIZE_INDEX(0b1, 0b1010)] = RDWR_SIZE_ENTRY(     7, 8 * 0 ),
    [RDWR_SIZE_INDEX(0b0, 0b1011)] = RDWR_SIZE_ENTRY_DW(  8),
    [RDWR_SIZE_INDEX(0b1, 0b1011)] = RDWR_SIZE_ENTRY_DW( 16),
    [RDWR_SIZE_INDEX(0b0, 0b1100)] = RDWR_SIZE_ENTRY_DW( 32),
    [RDWR_SIZE_INDEX(0b1, 0b1100)] = RDWR_SIZE_ENTRY_DW( 64),
    [RDWR_SIZE_INDEX(0b0, 0b1101)] = RDWR_SIZE_ENTRY_DW( 96),
    [RDWR_SIZE_INDEX(0b1, 0b1101)] = RDWR_SIZE_ENTRY_DW(128),
    [RDWR_SIZE_INDEX(0b0, 0b1110)] = RDWR_SIZE_ENTRY_DW(160),
    [RDWR_SIZE_INDEX(0b1, 0b1110)] = RDWR_SIZE_ENTRY_DW(192),
    [RDWR_SIZE_INDEX(0b0, 0b1111)] = RDWR_SIZE_ENTRY_DW(224),
    [RDWR_SIZE_INDEX(0b1, 0b1111)] = RDWR_SIZE_ENTRY_DW(256),
};

#define RDWR_SIZE_TABLE_ENTRIES (sizeof(rdwr_size_table) / sizeof(rdwr_size_table[0]))

static uint16_t find_rdwr_size_entry(uint16_t bytes, uint64_t mask)
{
    int i;
    for (i = 0; i < RDWR_SIZE_TABLE_ENTRIES; ++i) {
        if (rdwr_size_table[i].bytes == bytes &&
            rdwr_size_table[i].mask == mask)
            break;
    }
    return i;
}

static bool check_rdwr_size(uint16_t bytes, uint64_t mask)
{
    uint16_t entry_index = find_rdwr_size_entry(bytes, mask);
    if (entry_index >= RDWR_SIZE_TABLE_ENTRIES) {
        printf("RIO EP: rd/wr size bytes/mask not supported: %u/%08x%08x\r\n",
               bytes, (uint32_t)(mask >> 32), (uint32_t)(mask & 0xffffffff));
        return false;
    }
    return true;
}

static unsigned pack_pkt(uint32_t *buf, unsigned buf_size, struct rio_pkt *pkt)
{
    unsigned w = 0;
    uint16_t entry_index;

    switch (pkt->ftype) {
      case RIO_FTYPE_MAINT:

        ASSERT(w < buf_size);
        buf[w] = FIELD_DP32(buf[w], RIO_PKT_WORD0, PRIO, pkt->prio);
        buf[w] = FIELD_DP32(buf[w], RIO_PKT_WORD0, TTYPE, pkt->ttype);
        buf[w] = FIELD_DP32(buf[w], RIO_PKT_WORD0, FTYPE, pkt->ftype);
        buf[w] = FIELD_DP32(buf[w], RIO_PKT_WORD0, DEST_ID, pkt->dest_id);
        buf[w] = FIELD_DP32(buf[w], RIO_PKT_WORD0, SRC_ID, pkt->src_id);
        buf[w] = FIELD_DP32(buf[w], RIO_PKT_WORD0, TRANSACTION, pkt->transaction);

        switch (pkt->transaction) {
            case RIO_TRANS_MAINT_REQ_READ:
            case RIO_TRANS_MAINT_REQ_WRITE:
                entry_index = find_rdwr_size_entry(pkt->rdwr_bytes, pkt->rdwr_mask);

                buf[w] = FIELD_DP32(buf[w], RIO_PKT_WORD0, RDWRSIZE,
                                        RDWR_SIZE_INDEX_RDWRSIZE(entry_index));
                ++w;
                ASSERT(w < buf_size);
                buf[w] = FIELD_DP32(buf[w], RIO_PKT_WORD1, SRC_TID, pkt->src_tid);
                buf[w] = FIELD_DP32(buf[w], RIO_PKT_WORD1, CONFIG_OFFSET, pkt->config_offset);
                buf[w] = FIELD_DP32(buf[w], RIO_PKT_WORD1, WDPTR, RDWR_SIZE_INDEX_WDPTR(entry_index));
                ++w;
                ASSERT(w < buf_size);

                /* TODO: For wrsize = 4, do we return payload of one word or one double-word? */
                if (pkt->transaction == RIO_TRANS_MAINT_REQ_WRITE) {
                    ASSERT(DIV_CEIL(pkt->rdwr_bytes, 8) == pkt->payload_len);
                    for (int i = 0; i < pkt->payload_len; ++i) {
                        buf[w] = pkt->payload[i] >> 32;
                        ++w;
                        buf[w] = pkt->payload[i] & 0xffffffff;
                        ++w;
                    }
                }
                break;
            case RIO_TRANS_MAINT_RESP_READ:
            case RIO_TRANS_MAINT_RESP_WRITE:
                buf[w] = FIELD_DP32(buf[w], RIO_PKT_WORD0, STATUS, pkt->status);
                ++w;
                ASSERT(w < buf_size);
                buf[w] = FIELD_DP32(buf[w], RIO_PKT_WORD1, TARGET_TID, pkt->target_tid);
                ++w;

                if (pkt->transaction == RIO_TRANS_MAINT_RESP_READ) {
                    for (int i = 0; i < pkt->payload_len; ++i) {
                        buf[w] = pkt->payload[i] >> 32;
                        ++w;
                        buf[w] = pkt->payload[i] & 0xffffffff;
                        ++w;
                    }
                }
                break;
            default:
                ASSERT(!"packing for this transaction type not implemented");
        }
        break;
      default:
        ASSERT(!"pkt type not implemented");
    }

    return w;
}

static int unpack_pkt(struct rio_pkt *pkt, uint32_t *buf, unsigned buf_len)
{
    unsigned w = 0;
    uint8_t rdwrsize, wdptr;

    ASSERT(buf_len > 1); /* Fixed-length fields make up at least one word */
    ASSERT(w < buf_len);

    /* Transport layer */
    pkt->prio = FIELD_EX32(buf[w], RIO_PKT_WORD0, PRIO);
    pkt->ttype = FIELD_EX32(buf[w], RIO_PKT_WORD0, TTYPE);

    /* Logical layer */
    pkt->ftype = FIELD_EX32(buf[w], RIO_PKT_WORD0, FTYPE);

    switch (pkt->ftype) {
        case RIO_FTYPE_MAINT:
            pkt->dest_id = FIELD_EX32(buf[w], RIO_PKT_WORD0, DEST_ID);
            pkt->src_id = FIELD_EX32(buf[w], RIO_PKT_WORD0, SRC_ID);
            pkt->transaction = FIELD_EX32(buf[w], RIO_PKT_WORD0, TRANSACTION);

            switch (pkt->transaction) {
                case RIO_TRANS_MAINT_REQ_READ:
                case RIO_TRANS_MAINT_REQ_WRITE:
                    ASSERT(w < buf_len);
                    rdwrsize = FIELD_EX32(buf[w], RIO_PKT_WORD0, RDWRSIZE);
                    ++w;
                    ASSERT(w < buf_len);
                    pkt->src_tid = FIELD_EX32(buf[w], RIO_PKT_WORD1, SRC_TID);
                    pkt->config_offset = FIELD_EX32(buf[w], RIO_PKT_WORD1, CONFIG_OFFSET);
                    wdptr = FIELD_EX32(buf[w], RIO_PKT_WORD1, WDPTR);
                    ++w;

                    const RdWrSizeEntry *rdwr_entry = &rdwr_size_table[RDWR_SIZE_INDEX(wdptr, rdwrsize)];
                    pkt->rdwr_bytes = rdwr_entry->bytes;
                    pkt->rdwr_mask = rdwr_entry->mask;

                    pkt->payload_len = 0;

                    /* TODO: For wrsize = 4, do we return payload of one word or one double-word? */
                    if (pkt->transaction == RIO_TRANS_MAINT_REQ_WRITE) {
                        for (int i = 0; i < buf_len - w; i += 2) {
                            ASSERT(w < buf_len);
                            pkt->payload[i] = buf[w];
                            ++w;
                            ASSERT(w < buf_len);
                            pkt->payload[i] <<= 32;
                            pkt->payload[i] |= buf[w];
                            ++w;
                            ++pkt->payload_len;
                        }
                    }
                    break;
                case RIO_TRANS_MAINT_RESP_READ:
                case RIO_TRANS_MAINT_RESP_WRITE:
                    ASSERT(w < buf_len);
                    pkt->status = FIELD_EX32(buf[w], RIO_PKT_WORD0, STATUS);
                    ++w;
                    ASSERT(w < buf_len);
                    pkt->target_tid = FIELD_EX32(buf[w], RIO_PKT_WORD1, TARGET_TID);
                    ++w;

                    pkt->payload_len = 0;

                    if (pkt->transaction == RIO_TRANS_MAINT_RESP_READ) {
                        ASSERT(w < buf_len);
                        for (int i = 0; i < buf_len - w; i += 2) {
                            ASSERT(w < buf_len);
                            pkt->payload[i] = buf[w];
                            ++w;
                            ASSERT(w < buf_len);
                            pkt->payload[i] <<= 32;
                            pkt->payload[i] |= buf[w];
                            ++w;
                            ++pkt->payload_len;
                        }
                    }
                    break;
                default:
                    ASSERT(!"pkt transaction not implemented");
                    break;
            }
            break;
        default:
            ASSERT(!"pkt type not implemented");
            break;
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
                   "\trdwr_bytes %u mask %08x%08x\r\n"
                   "\tstatus %x\r\n"
                   "\tconfig_offset %x\r\n"
				   "\tpayload %08x%08x ...\r\n",
                   pkt->payload_len,
                   pkt->ftype, pkt->transaction,
                   pkt->src_id, pkt->dest_id,
                   pkt->src_tid, pkt->target_tid,
                   pkt->rdwr_bytes,
                   (uint32_t)(pkt->rdwr_mask >> 32),
                   (uint32_t)(pkt->rdwr_mask & 0xffffffff),
                   pkt->status, pkt->config_offset,
				   (uint32_t)(pkt->payload[0] >> 32),
				   (uint32_t)(pkt->payload[0] & 0xffffffff));
            break;
        default:
            ASSERT(!"pkt type not implemented");
    }
}

struct rio_ep *rio_ep_create(const char *name, volatile uint32_t *base, rio_devid_t devid)
{
    struct rio_ep *ep = OBJECT_ALLOC(rio_eps);
    if (!ep)
        return NULL;
    ep->name = name;
    ep->base = base;
	ep->devid = devid;
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
    bzero(pkt_buf, sizeof(pkt_buf));
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
    bzero(pkt_buf, sizeof(pkt_buf));

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

static int rio_ep_access_csr(struct rio_ep *ep, uint64_t *data, rio_devid_t dest,
                      uint32_t offset, uint16_t len, uint64_t mask,
                      enum access_type access)
{
    int rc;
    enum rio_transaction resp_transaction;
    unsigned resp_payload_len;

    out_pkt.ttype = RIO_TRANSPORT_DEV8;
    out_pkt.src_id = ep->devid;
    out_pkt.dest_id = dest;

    out_pkt.ftype = RIO_FTYPE_MAINT;
    out_pkt.src_tid = 0x3; /* TODO: to be delivered to maint unit, must be 0x0,0x1,0x2 */

    if (!check_rdwr_size(len, mask))
        return 1;
    out_pkt.rdwr_bytes = len;
    out_pkt.rdwr_mask = mask;

    out_pkt.config_offset = offset;

    switch (access) {
        case ACCESS_READ:
            out_pkt.transaction = RIO_TRANS_MAINT_REQ_READ;
            resp_transaction = RIO_TRANS_MAINT_RESP_READ;
            resp_payload_len = DIV_CEIL(len, 8);
            break;
        case ACCESS_WRITE:
            out_pkt.transaction = RIO_TRANS_MAINT_REQ_WRITE;
            resp_transaction = RIO_TRANS_MAINT_RESP_WRITE;
            resp_payload_len = 0;
            out_pkt.payload_len = DIV_CEIL(len, 8);
            for (int i = 0; i < out_pkt.payload_len; ++i) {
                out_pkt.payload[i] = data[i];
            }
            break;
    }

    printf("RIO EP %s: sending maint req:\r\n", ep->name);
    rio_print_pkt(&out_pkt);

	rc = rio_ep_sp_send(ep, &out_pkt);
	if (rc)
		return rc;

    rc = rio_ep_sp_recv(ep, &in_pkt);
	if (rc)
		return rc;

    printf("RIO EP %s: received maint resp:\r\n", ep->name);
    rio_print_pkt(&in_pkt);

    if (!(in_pkt.ftype == RIO_FTYPE_MAINT &&
          in_pkt.transaction == resp_transaction &&
          in_pkt.target_tid == out_pkt.src_tid &&
          in_pkt.payload_len == resp_payload_len)) {
        printf("RIO EP %s: ERROR: bad response to MAINTENANCE request\r\n", ep->name);
		return 1;
    }

    if (in_pkt.status != RIO_STATUS_DONE) {
        printf("RIO EP %s: ERROR: MAINTENANCE request failed with status %u\r\n",
               ep->name, in_pkt.status);
        return 1;
    }

    if (access == ACCESS_READ) {
        for (int i = 0; i < resp_payload_len; ++i) {
            data[i] = in_pkt.payload[i];
        }
    }
	return 0;
}

int rio_ep_read_csr(struct rio_ep *ep, uint64_t *data, rio_devid_t dest,
                    uint32_t offset, uint16_t len, uint64_t mask)
{
    return rio_ep_access_csr(ep, data, dest, offset, len, mask, ACCESS_READ);
}

int rio_ep_write_csr(struct rio_ep *ep, const uint64_t *data, rio_devid_t dest,
                     uint32_t offset, uint16_t len, uint64_t mask)
{
    return rio_ep_access_csr(ep, (uint64_t *)data, dest,
                             offset, len, mask, ACCESS_WRITE);
}

int rio_ep_read_csr_32(struct rio_ep *ep, uint32_t *data, rio_devid_t dest,
                       uint32_t offset)
{
    uint64_t data_dw;
    int rc = rio_ep_read_csr(ep, &data_dw, dest, offset, 4, 0xffffffff);
    if (rc)
        return rc;
    *data = (uint32_t)(data_dw & 0xffffffff);
    return 0;
}

int rio_ep_write_csr_32(struct rio_ep *ep, const uint32_t *data,
                        rio_devid_t dest, uint32_t offset)
{
    uint64_t data_dw = *data;
    int rc = rio_ep_write_csr(ep, &data_dw, dest, offset, 4, 0xffffffff);
    if (rc)
        return rc;
    return 0;
}
