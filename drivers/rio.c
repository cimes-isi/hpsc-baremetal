#define DEBUG 1

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


#if 0
static unsigned pack_pkt(uint32_t *buf, unsigned buf_size, struct rio_pkt *pkt)
{
    unsigned w = 0;

    switch (pkt->ftype) {
      case RIO_FTYPE_MAINT:

        ASSERT(w < buf_size);
        buf[w] = FIELD_DP32(buf[w], RIO_PKT_WORD0, PRIO, pkt->prio);
        buf[w] = FIELD_DP32(buf[w], RIO_PKT_WORD0, TTYPE, pkt->ttype);
        buf[w] = FIELD_DP32(buf[w], RIO_PKT_WORD0, FTYPE, pkt->ftype);
        buf[w] = FIELD_DP32(buf[w], RIO_PKT_WORD0, DEST_ID, pkt->dest_id);
        buf[w] = FIELD_DP32(buf[w], RIO_PKT_WORD0, SRC_ID, pkt->src_id);
        buf[w] = FIELD_DP32(buf[w], RIO_PKT_WORD0, TRANSACTION, pkt->transaction);
        buf[w] = FIELD_DP32(buf[w], RIO_PKT_WORD0, STATUS, pkt->status);
        ++w;
        ASSERT(w < buf_size);


        buf[w] = FIELD_DP32(buf[w], RIO_PKT_WORD1, TARGET_TID, pkt->target_tid);

        for (unsigned d = 0; d < pkt->payload_len; ++d) {
            uint64_t data = pkt->payload[d];
            for (int i = 0; i < 3; ++i) {
                buf[w] |= (data & 0xff) << (RIO_PKT_WORD1__DATA_3LSB__SHIFT + 8 * i);
                data >>= 8;
            }
            ++w;
            ASSERT(w < buf_size);
            buf[w] = 0;
            for (int i = 0; i < 4; ++i) {
                buf[w] |= (data & 0xff) << (RIO_PKT_WORD2__DATA_4MID__SHIFT + 8 * i);
                data >>= 8;
            }
            ++w;
            ASSERT(w < buf_size);
            buf[w] |= (data & 0xff) << RIO_PKT_WORD3__DATA_1MSB__SHIFT;
        }
        break;
      default:
        ASSERT(!"pkt type not implemented");
    }

    return w + 1; /* index of last filled word to length */
}
#else
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
#endif

#if 0
static int unpack_pkt(struct rio_pkt *pkt, uint32_t *buf, unsigned buf_len)
{
    unsigned w = 0;

    ASSERT(buf_len > 1); /* Fixed-length fields make up at least one word */

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
            		pkt->rdwrsize = FIELD_EX32(buf[w], RIO_PKT_WORD0, RDWRSIZE);
					break;
				case RIO_TRANS_MAINT_RESP_READ:
				case RIO_TRANS_MAINT_RESP_WRITE:
            		pkt->status = FIELD_EX32(buf[w], RIO_PKT_WORD0, STATUS);
					break;
				default:
            		ASSERT(!"pkt transaction not implemented");
			}

            ++w;
            pkt->target_tid = FIELD_EX32(buf[w], RIO_PKT_WORD1, TARGET_TID);

            pkt->payload_len = 0;
            while (w + 1 < buf_len) {
                DPRINTF("RIO: decoding word %u out of %u\r\n", w, buf_len);

				uint64_t data = 0;
				data |= (buf[w + 2] >> RIO_PKT_WORD3__DATA_1MSB__SHIFT) & 0xff;

                ASSERT(w < buf_len);
                for (int i = 4 - 1; i >= 0; --i) {
                    data <<= 8;
                    data |= (buf[w + 1] >> (RIO_PKT_WORD2__DATA_4MID__SHIFT + 8 * i)) & 0xff;
                }

                for (int i = 3 - 1; i >= 0; --i) {
                    data <<= 8;
                    data |= (buf[w] >> (RIO_PKT_WORD1__DATA_3LSB__SHIFT + 8 * i)) & 0xff;
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
#else
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
                        for (int i = 0; i < buf_len; i += 2) {
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
#endif

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

int rio_ep_read_csr(struct rio_ep *ep, uint64_t *data, rio_devid_t dest,
                    uint32_t offset, uint16_t len, uint64_t mask)
{
    int rc;

    out_pkt.ttype = RIO_TRANSPORT_DEV8;
    out_pkt.src_id = ep->devid;
    out_pkt.dest_id = dest;

    out_pkt.ftype = RIO_FTYPE_MAINT;
    out_pkt.src_tid = 0x3; /* TODO: to be delivered to maint unit, must be 0x0,0x1,0x2 */
    out_pkt.transaction = RIO_TRANS_MAINT_REQ_READ;

    uint16_t entry_index = find_rdwr_size_entry(len, mask);
    if (entry_index >= RDWR_SIZE_TABLE_ENTRIES) {
        printf("RIO EP %s: rd/wr size bytes/mask not supported: %u/%08x%08x\r\n",
               ep->name, len,
               (uint32_t)(mask >> 32), (uint32_t)(mask & 0xffffffff));
        return 1;
    }
    out_pkt.rdwr_bytes = len;
    out_pkt.rdwr_mask = mask;

    out_pkt.config_offset = offset;
    out_pkt.payload_len = 0x0;

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
          in_pkt.transaction == RIO_TRANS_MAINT_RESP_READ &&
          in_pkt.target_tid == out_pkt.src_tid &&
          in_pkt.payload_len == DIV_CEIL(len, 8))) {
        printf("RIO EP %s: ERROR: bad response to MAINTENANCE request\r\n", ep->name);
		return 1;
    }

    if (in_pkt.status != RIO_STATUS_DONE) {
        printf("RIO EP %s: ERROR: MAINTENANCE request failed with status %u\r\n",
               ep->name, in_pkt.status);
        return 1;
    }

    for (int i = 0; i < DIV_CEIL(len, 8); ++i) {
        data[i] = in_pkt.payload[i];
    }
	return 0;
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
