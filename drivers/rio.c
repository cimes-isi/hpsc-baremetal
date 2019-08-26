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

REG32(IR_MSG_CTRL, 0x107100)
REG32(IR_MSG_INIT_PS_STAT, 0x107110)
    FIELD(IR_MSG_INIT_PS_STAT, IDLE,                   0, 1)
    FIELD(IR_MSG_INIT_PS_STAT, MAX_DESCRIPTORS,        1, 3)
    FIELD(IR_MSG_INIT_PS_STAT, DESCRIPTORS,            4, 3)
REG32(IR_MSG_INIT_PS_DESCR_FIFO_L, 0x107114)
REG32(IR_MSG_INIT_PS_DESCR_FIFO_H, 0x107118)
REG32(IR_MSG_INIT_DS_STAT, 0x107130)
    FIELD(IR_MSG_INIT_DS_STAT, IDLE,                   0, 1)
    FIELD(IR_MSG_INIT_DS_STAT, MAX_DESCRIPTORS,        1, 3)
    FIELD(IR_MSG_INIT_DS_STAT, DESCRIPTORS,            4, 3)
REG32(IR_MSG_INIT_DS_DESCR_FIFO_L, 0x107134)
REG32(IR_MSG_INIT_DS_DESCR_FIFO_H, 0x107138)
REG32(IR_MSG_TRGT_STAT, 0x107150)
    FIELD(IR_MSG_TRGT_STAT, IDLE,                   0, 1)
    FIELD(IR_MSG_TRGT_STAT, MAX_DESCRIPTORS,        1, 3)
    FIELD(IR_MSG_TRGT_STAT, DESCRIPTORS,            4, 3)
REG32(IR_MSG_TRGT_DESCR_FIFO_L, 0x107154)
REG32(IR_MSG_TRGT_DESCR_FIFO_H, 0x107158)

/* See Chapter 4 in RIO Spec P1.
   Also see Chapter 14 (assuming little-endian byte order in words)
   Note that some fields overlap, being in different packet types. */

/* TODO: switch to big endian? */

/* Common */
FIELD(RIO_PKT_WORD0, PRIO,        	0,  2)
FIELD(RIO_PKT_WORD0, TTYPE,        	2,  2)
FIELD(RIO_PKT_WORD0, FTYPE,       	4,  4)
FIELD(RIO_PKT_WORD0, DEST_ID,     	8,  8)
FIELD(RIO_PKT_WORD0, SRC_ID,      	16, 8)

/* Maintanance class packets */
FIELD(RIO_PKT_WORD0, TRANSACTION, 	24, 4)
FIELD(RIO_PKT_WORD0, STATUS,      	28, 4)
FIELD(RIO_PKT_WORD0, RDWRSIZE,     	28, 4)
FIELD(RIO_PKT_WORD1, SRC_TID,  		0,  8)
FIELD(RIO_PKT_WORD1, TARGET_TID,  	0,  8)
FIELD(RIO_PKT_WORD1, CONFIG_OFFSET, 8, 21)
FIELD(RIO_PKT_WORD1, WDPTR,         29, 1)

/* Message class packets */
FIELD(RIO_PKT_WORD0, MSG_LEN, 	24, 4)
FIELD(RIO_PKT_WORD0, SEG_SIZE,  28, 4)
FIELD(RIO_PKT_WORD1, LETTER,  	0,  2)
FIELD(RIO_PKT_WORD1, MBOX,  	2,  2)
FIELD(RIO_PKT_WORD1, MSG_SEG,   4,  4)
FIELD(RIO_PKT_WORD1, XMBOX,   	4,  4)
FIELD(RIO_PKT_WORD1, DATA_3LSB, 8, 24)
FIELD(RIO_PKT_WORD2, DATA_4MID, 0, 32)
FIELD(RIO_PKT_WORD3, DATA_1MSB, 0,  8)

/* See Chapter 7.2 in User Guide, but flip the byte order,
   since we will read each 8 bytes as a 64-bit double-word and access the
   fields in that double-word. First byte in memory (numbered 0 in
   the user guide) becomes MSB in the double-word. */
#if 0
FIELD(MSG_DESC, PRIO,           60,  4)
FIELD(MSG_DESC, INTERRUPT,      59,  1)
FIELD(MSG_DESC, FREE,           58,  1)
FIELD(MSG_DESC, TTYPE,          56,  2)
FIELD(MSG_DESC, DEST_ID,        24, 32)
FIELD(MSG_DESC, MSG_LEN,        20,  4)
FIELD(MSG_DESC, SEG_SIZE,       16,  4)
FIELD(MSG_DESC, LETTER,         14,  2)
FIELD(MSG_DESC, MBOX,           12,  2)
FIELD(MSG_DESC, MSG_SEG,         8,  4)
FIELD(MSG_DESC, XMBOX,           8,  4) /* overlaps across desc types */
#else
FIELD(MSG_DESC, PRIO,            0,  4)
FIELD(MSG_DESC, INTERRUPT,       4,  1)
FIELD(MSG_DESC, FREE,            5,  1)
FIELD(MSG_DESC, TTYPE,           6,  2)
FIELD(MSG_DESC, DEST_ID,         8, 32)
FIELD(MSG_DESC, SRC_ID,         8, 32) /* overlaps */
FIELD(MSG_DESC, MSG_LEN,        40,  4)
FIELD(MSG_DESC, SEG_SIZE,       44,  4)
FIELD(MSG_DESC, LETTER,         48,  2)
FIELD(MSG_DESC, MBOX,           50,  2)
FIELD(MSG_DESC, MSG_SEG,        52,  4)
FIELD(MSG_DESC, XMBOX,          52,  4) /* overlaps across desc types */
#endif

typedef enum MsgDescType {
    MSG_DESC_INITIATOR_PS,
    MSG_DESC_INITIATOR_DS,
    MSG_DESC_TARGET,
} MsgDescType;

typedef struct MsgDesc {
    MsgDescType type;
    uint8_t prio;
    bool interrupt;
    bool free;
    enum rio_transport_type ttype;
    union {
        uint32_t dest_id;
        uint32_t src_id;
    } dev_id;
    uint8_t msg_len;
    uint8_t seg_size;
    uint8_t mbox;
    uint8_t letter;
    uint8_t msg_seg;
    uint8_t *payload_ptr;
    uint8_t *next_desc_addr; /* packed, so not a MsgDesc* */
    union {
        uint64_t launch_time;
        uint64_t rcv_time;
    } timestamp;
} MsgDesc;

#if 1
/* Figure 10 in 7.2.1 in User Guide (in bytes) */
static const uint8_t msg_desc_sizes[] = {
    [MSG_DESC_INITIATOR_PS] = 3 * 8,
    [MSG_DESC_INITIATOR_DS] = 4 * 8,
    [MSG_DESC_TARGET]       = 4 * 8,
};
#define MSG_DESC_SIZES_ENTRIES (sizeof(msg_desc_sizes) / sizeof(msg_desc_sizes[0]))
#define MAX_MSG_DESC_SIZE 32 /* keep in sync with the list above */

static uint8_t msg_desc_size(MsgDescType type)
{
    ASSERT(type < MSG_DESC_SIZES_ENTRIES);
    return msg_desc_sizes[type];
}
#else
#define MAX_MSG_DESC_SIZE 32 /* keep in sync with the list above */
#endif

/* Table 4-4 (ssize field) in Spec (in bytes) */
/* TODO: move to device independent code? */
#define MSG_SEG_SIZE(ssize_field) (ssize_field - 0b1000) /* to save space */
#define MSG_SEG_SIZE_FIELD(seg_size) (seg_size | 0b1000)
static const uint16_t msg_seg_sizes[] = {
    [MSG_SEG_SIZE(0b1001)]  =   8,
    [MSG_SEG_SIZE(0b1010)]  =  16,
    [MSG_SEG_SIZE(0b1011)]  =  32,
    [MSG_SEG_SIZE(0b1100)]  =  64,
    [MSG_SEG_SIZE(0b1101)]  = 128,
    [MSG_SEG_SIZE(0b1110)]  = 256,
};
#define MSG_SEG_SIZES_ENTRIES (sizeof(msg_seg_sizes) / sizeof(msg_seg_sizes[0]))
#define MAX_MSG_SEG_SIZE 256 /* keep in sync with the list above */

#define MSG_SEG_SIZE_INVALID 0
#define MSG_SEG_SIZE_FIELD_INVALID 0xff
static inline uint16_t msg_seg_size(uint8_t seg_size_field)
{
    if (MSG_SEG_SIZE(seg_size_field) >= MSG_SEG_SIZES_ENTRIES)
        return MSG_SEG_SIZE_INVALID;
    return msg_seg_sizes[MSG_SEG_SIZE(seg_size_field)];
}

static inline uint8_t msg_seg_size_field(uint8_t seg_size)
{
    for (int i = 0; i < MSG_SEG_SIZES_ENTRIES; ++i) {
        if (seg_size == msg_seg_sizes[i])
            return MSG_SEG_SIZE_FIELD(i);
    }
    return MSG_SEG_SIZE_FIELD_INVALID;
}

enum access_type {
    ACCESS_READ,
    ACCESS_WRITE,
};

#define MAX_RIO_ENDPOINTS 2

static struct rio_ep rio_eps[MAX_RIO_ENDPOINTS] = {0};

#define MAX_MSG_SEGMENTS 16 /* Spec 2.3.1 */

#define MAX_MSG_SIZE 256 /* TODO */

#define NUM_MBOXES 4
#define NUM_LETTERS 4

#define NUM_RX_CHAINS 2

#define RIO_MAX_PKT_SIZE 64 /* TODO */
#define PKT_BUF_WORDS (RIO_MAX_PKT_SIZE / sizeof(uint32_t))
static uint32_t pkt_buf[PKT_BUF_WORDS];

static uint8_t msg_tx_desc_buf[MAX_MSG_DESC_SIZE * MAX_MSG_SEGMENTS];
static uint8_t msg_rx_desc_buf[NUM_RX_CHAINS][MAX_MSG_DESC_SIZE * MAX_MSG_SEGMENTS];

struct rx_msg {
    uint8_t payload[MAX_MSG_SIZE];
    unsigned len; /* bytes */
    uint32_t segments; /* assert(width >= MAX_MSG_SEGMENTS) */
    rio_devid_t src_id;
    uint64_t rcv_time;
};

#define RX_MSG_FIFO_SIZE 4

struct rx_msg_fifo {
    unsigned head;
    unsigned tail;
    struct rx_msg q[RX_MSG_FIFO_SIZE];
};

/* A FIFO for received messages, one FIFO per mbox, letter endpoint.
   Only one in-flight (i.e. incomplete) message per mbox/letter is supported.
   But, we support a FIFO between driver and consumer, where *completed*
   messages are queued. The in-flight message is assembled in the "current"
   slot in this FIFO. */
static struct rx_msg_fifo rx_msgs[NUM_MBOXES][NUM_LETTERS];

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
    pkt->dest_id = FIELD_EX32(buf[w], RIO_PKT_WORD0, DEST_ID);
    pkt->src_id = FIELD_EX32(buf[w], RIO_PKT_WORD0, SRC_ID);

    switch (pkt->ftype) {
        case RIO_FTYPE_MAINT:
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

        case RIO_FTYPE_MSG:
            pkt->msg_len = FIELD_EX32(buf[w], RIO_PKT_WORD0, MSG_LEN) + 1;
            uint8_t seg_size_field = FIELD_EX32(buf[w], RIO_PKT_WORD0, SEG_SIZE);
            pkt->seg_size = msg_seg_size(seg_size_field);
            if (pkt->seg_size == MSG_SEG_SIZE_INVALID) {
                printf("RIO: NOTICE: invalid seg size in message packet: %x\r\n",
                       seg_size_field);
                return 1;
            }
            ++w;
            ASSERT(w < buf_len);
            pkt->letter = FIELD_EX32(buf[w], RIO_PKT_WORD1, LETTER);
            pkt->mbox = FIELD_EX32(buf[w], RIO_PKT_WORD1, MBOX);
            if (pkt->msg_len > 1) {
                pkt->msg_seg = FIELD_EX32(buf[w], RIO_PKT_WORD1, MSG_SEG);
                ASSERT(pkt->mbox < 4); /* enforced by field widths */
            } else {
                pkt->mbox |= FIELD_EX32(buf[w], RIO_PKT_WORD1, XMBOX) << 2;
                ASSERT(pkt->mbox < 64); /* enforced by field widths */
            }
            uint64_t data = 0;
            data = FIELD_EX32(buf[w], RIO_PKT_WORD1, DATA_3LSB);
            ++w;
            ASSERT(w < buf_len);
            data |= ((uint64_t)buf[w] << (3 * 8));
            ++w;
            ASSERT(w < buf_len);
            data |= (uint64_t)FIELD_EX32(buf[w], RIO_PKT_WORD3, DATA_1MSB) << (7 * 8);
            pkt->payload[0] = data;
            pkt->payload_len = 1; /* TODO: make the above a loop */
            break;
        default:
            ASSERT(!"pkt type not implemented");
            break;
    }

    return 0;
}

void rio_print_pkt(struct rio_pkt *pkt)
{
    printf("RIO PKT: payload len %u\r\n"
           "\tftype %u\r\n"
           "\tsrc_id %x\r\n"
           "\tdest_id %x\r\n",
           pkt->payload_len,
           pkt->ftype,
           pkt->src_id, pkt->dest_id);

    switch (pkt->ftype) {
        case RIO_FTYPE_MAINT:
            printf("\ttransaction %x\r\n"
                    "\tsrc_tid %x\r\n"
                    "\ttarget_tid %x\r\n"
                    "\trdwr_bytes %u mask %08x%08x\r\n"
                    "\tstatus %x\r\n"
                    "\tconfig_offset %x\r\n"
                    "\tpayload %08x%08x ...\r\n",
                    pkt->transaction,
                    pkt->src_tid, pkt->target_tid,
                    pkt->rdwr_bytes,
                    (uint32_t)(pkt->rdwr_mask >> 32),
                    (uint32_t)(pkt->rdwr_mask & 0xffffffff),
                    pkt->status, pkt->config_offset,
                    (uint32_t)(pkt->payload[0] >> 32),
                    (uint32_t)(pkt->payload[0] & 0xffffffff));
            break;
        case RIO_FTYPE_MSG:
            printf("\tmsg_len %u msg_seg %u seg size %u\r\n"
                    "\tmbox %u letter %u\r\n"
                    "\tpayload %08x%08x ...\r\n",
                    pkt->msg_len, pkt->msg_seg, pkt->seg_size,
                    pkt->mbox, pkt->letter,
                    (uint32_t)(pkt->payload[0] >> 32),
                    (uint32_t)(pkt->payload[0] & 0xffffffff));
            break;
        default:
            ASSERT(!"pkt type not implemented");
    }
}

static int pack_msg_desc(uint8_t *buf_b, unsigned size, MsgDesc *desc)
{
    uint64_t *buf = (uint64_t *)buf_b;
    unsigned dw = 0;
    bzero(buf_b, size);
    ASSERT(dw < size);
    buf[dw] = FIELD_DP64(buf[dw], MSG_DESC, PRIO, desc->prio);
    buf[dw] = FIELD_DP64(buf[dw], MSG_DESC, INTERRUPT, desc->interrupt);
    buf[dw] = FIELD_DP64(buf[dw], MSG_DESC, FREE, desc->free);
    buf[dw] = FIELD_DP64(buf[dw], MSG_DESC, TTYPE, desc->ttype);
    buf[dw] = FIELD_DP64(buf[dw], MSG_DESC, DEST_ID, desc->dev_id.dest_id);
    buf[dw] = FIELD_DP64(buf[dw], MSG_DESC, MSG_LEN, desc->msg_len - 1); /* 0->1 */

    uint8_t seg_size_field = msg_seg_size_field(desc->seg_size);
    ASSERT(seg_size_field != MSG_SEG_SIZE_FIELD_INVALID);
    buf[dw] = FIELD_DP64(buf[dw], MSG_DESC, SEG_SIZE, seg_size_field);

    buf[dw] = FIELD_DP64(buf[dw], MSG_DESC, MBOX, desc->mbox & 0b11);
    buf[dw] = FIELD_DP64(buf[dw], MSG_DESC, LETTER, desc->letter);
    if (desc->msg_len > 0) {
        buf[dw] = FIELD_DP64(buf[dw], MSG_DESC, MSG_SEG, desc->msg_seg);
        ASSERT(desc->mbox >> 2 == 0);
    } else {
        buf[dw] = FIELD_DP64(buf[dw], MSG_DESC, XMBOX, (desc->mbox >> 2) & 0b1111);
        ASSERT(desc->mbox >> (2 + 4) == 0);
    }
    ++dw;
    ASSERT(dw < size);
    buf[dw++] = (uint32_t)desc->payload_ptr;
    ASSERT(dw < size);
    buf[dw++] = (uint32_t)desc->next_desc_addr;
    if (desc->type == MSG_DESC_INITIATOR_DS) {
        ASSERT(dw < size);
        buf[dw++] = desc->timestamp.launch_time;
    }
    return dw * 8;
}

static int unpack_msg_desc(MsgDesc *desc, uint8_t *buf_b, MsgDescType type)
{
    int dw = 0;
    unsigned len = msg_desc_size(type);
    uint64_t *buf = (uint64_t *)buf_b;

    desc->type = type;

    ASSERT(dw < len);
    desc->prio = FIELD_EX64(buf[dw], MSG_DESC, PRIO);
    desc->interrupt = FIELD_EX64(buf[dw], MSG_DESC, INTERRUPT);
    desc->free = FIELD_EX64(buf[dw], MSG_DESC, FREE);
    desc->ttype = FIELD_EX64(buf[dw], MSG_DESC, TTYPE);
    if (type == MSG_DESC_TARGET) {
        desc->dev_id.src_id = FIELD_EX64(buf[dw], MSG_DESC, SRC_ID);
    } else {
        desc->dev_id.dest_id = FIELD_EX64(buf[dw], MSG_DESC, DEST_ID);
    }
    desc->msg_len = FIELD_EX64(buf[dw], MSG_DESC, MSG_LEN) + 1; /* 0->1 */

    uint8_t seg_size = FIELD_EX64(buf[dw], MSG_DESC, SEG_SIZE);
    if (msg_seg_size(seg_size) == MSG_SEG_SIZE_INVALID) {
        printf("RIO: NOTICE: invalid segment size in msg descriptor: %x\n",
                 seg_size);
        return 1;
    }
    desc->seg_size = msg_seg_size(seg_size);

    desc->mbox = FIELD_EX64(buf[dw], MSG_DESC, MBOX);
    desc->letter = FIELD_EX64(buf[dw], MSG_DESC, LETTER);
    if (desc->msg_len > 1) {
        desc->msg_seg = FIELD_EX64(buf[dw], MSG_DESC, MSG_SEG);
        if (desc->msg_seg >= desc->msg_len) {
            printf("RIO: NOTICE: invalid msg segment index in msg descriptor: "
                     "%u (>= msg_len = %u)\n",
                     desc->msg_seg, desc->msg_len);
            return 2;
        }
    } else {
        desc->mbox |= (FIELD_EX64(buf[dw], MSG_DESC, XMBOX) << 2);
    }
    ++dw;
    ASSERT(dw < len);
    desc->payload_ptr = (uint8_t *)(uint32_t)buf[dw];
    ++dw;
    ASSERT(dw < len);
    desc->next_desc_addr = (uint8_t *)(uint32_t)buf[dw];
    ++dw;
    if (type == MSG_DESC_INITIATOR_DS) {
        ASSERT(dw < len);
        desc->timestamp.launch_time = buf[dw];
    }
    return 0;
}

static void print_msg_desc(MsgDesc *desc)
{
    uint64_t timestamp;
    switch (desc->type) {
        case MSG_DESC_INITIATOR_PS: timestamp = 0; break;
        case MSG_DESC_INITIATOR_DS: timestamp = desc->timestamp.launch_time; break;
        case MSG_DESC_TARGET: timestamp = desc->timestamp.rcv_time; break;
        default: ASSERT(!"invalid msg descriptor type");
    }
    uint64_t *payload = (uint64_t *)&desc->payload_ptr[0];
    printf("msg desc: next %p type %u\r\n"
           "\tinterrupt %u free %u\r\n"
           "\tprio %u ttype %u\r\n"
           "\tsrc/dest id 0x%x\r\n"
           "\tmsg_len %u seg_size %u seg %u\r\n"
           "\tmbox %u letter %u"
           "\ttimestamp %08x%08x\r\n"
           "\tpayload %p: %08x%08x ...\r\n",
           desc->next_desc_addr, desc->type,
           desc->interrupt, desc->free,
           desc->prio, desc->ttype,
           desc->dev_id.dest_id,
           desc->msg_len, desc->seg_size, desc->msg_seg,
           desc->mbox, desc->letter,
           (uint32_t)(timestamp >> 32),
           (uint32_t)(timestamp & 0xffffffff),
           desc->payload_ptr,
           (uint32_t)(payload[0] >> 32),
           (uint32_t)(payload[0] & 0xffffffff));
}

static inline void init_rx_msg(struct rx_msg *msg)
{
    msg->segments = ~0;
    msg->len = 0;
    bzero(msg->payload, sizeof(msg->payload));
}

static unsigned init_rx_chain(uint8_t *buf_b, int size_b)
{
    int size = size_b / sizeof(uint64_t);
    uint8_t *desc = buf_b;
    unsigned num_desc = 0;

    bzero(buf_b, size);

    while (desc + MAX_MSG_DESC_SIZE + MAX_MSG_SEG_SIZE <= buf_b + size) {
        uint64_t *buf = (uint64_t *)desc;
        unsigned dw = 0;

        ASSERT(dw < size);
        buf[dw] = FIELD_DP64(buf[dw], MSG_DESC, INTERRUPT, 0);
        buf[dw] = FIELD_DP64(buf[dw], MSG_DESC, FREE, 0);
        ++dw;
        ASSERT(dw < size);
        buf[dw++] = (uint32_t)(desc + MAX_MSG_DESC_SIZE);
        ASSERT(dw < size);
        desc += MAX_MSG_DESC_SIZE + MAX_MSG_SEG_SIZE;
        buf[dw++] = (uint32_t)desc;
        ASSERT(dw < size);
        buf[dw++] = 0;
        ++num_desc;
    }
    return num_desc;
}

static unsigned reserve_rx_chain(uint8_t *desc, int buf_size)
{
    uint8_t *desc_addr = desc;

    while (desc_addr) {
        unsigned dw = 0;
        uint64_t *buf = (uint64_t *)desc_addr;

        buf[dw] = FIELD_DP64(buf[dw], MSG_DESC, FREE, 0);
        ++dw;

        /* Do some consistency checks, while walking the chain */

        uint8_t interrupt = FIELD_EX64(buf[dw], MSG_DESC, INTERRUPT);
        ASSERT(interrupt == 0);

        desc_addr = (uint8_t *)(uint32_t)buf[dw++];
        ASSERT(desc_addr <= desc + buf_size);

        uint8_t *payload_addr = (uint8_t *)(uint32_t)buf[dw++];
        ASSERT(payload_addr <= desc + buf_size);
    }
    return 0;
}

static void enqueue_rx_chain(struct rio_ep *ep, uint8_t *head_desc)
{
    REGB_WRITE32(ep->base, IR_MSG_TRGT_DESCR_FIFO_H, 0);
    REGB_WRITE32(ep->base, IR_MSG_TRGT_DESCR_FIFO_L, (uint32_t)head_desc);
}

struct rio_ep *rio_ep_create(const char *name, volatile uint32_t *base, rio_devid_t devid)
{
    int num_msg_rx_desc;
    struct rio_ep *ep = OBJECT_ALLOC(rio_eps);
    if (!ep)
        return NULL;
    ep->name = name;
    ep->base = base;
    ep->devid = devid;

    for (int i = 0; i < NUM_RX_CHAINS; ++i) {
        num_msg_rx_desc =
            init_rx_chain(msg_rx_desc_buf[i], sizeof(msg_rx_desc_buf[i]));
        enqueue_rx_chain(ep, &msg_rx_desc_buf[i][0]);
    }
    ep->rx_chain = 0;
    ep->msg_rx_desc_addr = &msg_rx_desc_buf[ep->rx_chain][0];

    printf("RIO EP %s: created; msg rx descs %u\r\n", ep->name, num_msg_rx_desc);
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

int rio_ep_msg_send(struct rio_ep *ep, rio_devid_t dest, uint64_t launch_time,
                    uint8_t mbox, uint8_t letter, uint8_t seg_size,
                    uint8_t *payload, unsigned len)
{
    if (msg_seg_size_field(seg_size) == MSG_SEG_SIZE_FIELD_INVALID) {
        printf("RIO EP %s: ERROR: invalid msg segment size: %u"
               " (must be power of 2 <= 256)\r\n", ep->name, seg_size);
        return 1;
    }

    uint8_t msg_len = DIV_CEIL(len, seg_size);
    if (msg_len > MAX_MSG_SEGMENTS) {
        printf("RIO EP %s: ERROR: msg of length %u with segment size %u"
               " has too many segments: %u (>= %u)\r\n",
               ep->name, len, seg_size, MAX_MSG_SEGMENTS);
        return 1;
    }

    if (msg_len > 0 && mbox >= 4) {
        printf("RIO EP %s: ERROR: invalid mailbox for multi-segment message: %u"
               " (>= 4)\r\n", ep->name);
        return 2;
    } else if (msg_len == 0 && mbox >= 64) {
        printf("RIO EP %s: ERROR: invalid mailbox for single-segment message: %u"
               " (>= 64)\r\n", ep->name, mbox);
        return 3;
    }

    if (letter > 4) {
        printf("RIO EP %s: ERROR: invalid letter index for mailbox: %u"
               " (>= 4)\r\n", ep->name, letter);
        return 4;
    }

    /* TODO: is zero timestamp a reasonable invalid value? */
    MsgDescType desc_type = launch_time > 0 ?
        MSG_DESC_INITIATOR_DS : MSG_DESC_INITIATOR_PS;

    uint32_t desc_chain_addr = (uint32_t)&msg_tx_desc_buf[0];
    uint8_t *desc_addr;

    for (int seg = 0; seg < msg_len; ++seg) {

        MsgDesc desc = { /* fields validated later in pack() */
            .type = desc_type,
            .free = 0,
            .interrupt = false, /* TODO */

            .ttype = RIO_TRANSPORT_DEV8,
            .dev_id.dest_id = dest,

            .mbox = mbox,
            .letter = letter,

            .seg_size = seg_size,

            .msg_len = msg_len,
            .msg_seg = seg,

            /* TODO: save across segments? */
            .timestamp.launch_time = launch_time,

            /* this func is synchronous */
            .payload_ptr = payload + seg * seg_size,
            .next_desc_addr = (seg == msg_len - 1) ? 0 : &msg_tx_desc_buf[seg + 1],
        };

        desc_addr = &msg_tx_desc_buf[seg * MAX_MSG_DESC_SIZE];
        printf("RIO EP %s: created msg descriptor %u/%u at %p:\r\n",
               ep->name, seg + 1, msg_len, desc_addr);
        print_msg_desc(&desc);

        pack_msg_desc(desc_addr, MAX_MSG_DESC_SIZE, &desc);
    }

    switch (desc_type) {
        case MSG_DESC_INITIATOR_PS:
            REGB_WRITE32(ep->base, IR_MSG_INIT_PS_DESCR_FIFO_H, 0);
            REGB_WRITE32(ep->base, IR_MSG_INIT_PS_DESCR_FIFO_L, desc_chain_addr);
            break;
        case MSG_DESC_INITIATOR_DS:
            REGB_WRITE32(ep->base, IR_MSG_INIT_DS_DESCR_FIFO_H, 0);
            REGB_WRITE32(ep->base, IR_MSG_INIT_DS_DESCR_FIFO_L, desc_chain_addr);
            break;
        default:
            ASSERT(!"unexpected msg descriptor type");
    }

    /* TODO: wait for interrupt */
    printf("RIO EP %s: waiting for last msg desc to be released...\r\n", ep->name);
    bool free = false;
    do {
        uint64_t header = *((uint64_t *)desc_addr); /* one byte from last desc into word */
        free = FIELD_EX64(header, MSG_DESC, FREE);
    } while (!free);
    printf("RIO EP %s: msg sent\r\n", ep->name);

    return 0;
}

static struct rx_msg *rx_msg_peek(struct rx_msg_fifo *f)
{
    if (f->head == f->tail)
        return NULL; /* empty */
    return &f->q[f->head];
}

static struct rx_msg *rx_msg_alloc(struct rx_msg_fifo *f)
{
    struct rx_msg *m;
    unsigned next_tail = (f->tail + 1) % RX_MSG_FIFO_SIZE;
    if (next_tail == f->head)
        return NULL; /* full */
    m = &f->q[f->tail];
    f->tail = next_tail;
    init_rx_msg(m);
    return m;
}

static int rx_msg_free(struct rx_msg_fifo *f)
{
    if (f->head == f->tail)
        return -1; /* empty */
    bzero(&f->q[f->head], sizeof(f->q[0]));
    f->head = (f->head + 1) % RX_MSG_FIFO_SIZE;
    return 0;
}

static void receive_msg_segment(struct rio_ep *ep)
{
    int rc;
    uint8_t free;
    MsgDesc desc;

    ASSERT(ep->msg_rx_desc_addr);

    /* TODO: wait for interrupt */
    printf("RIO EP %s: waiting for segment...\r\n", ep->name);
    do { /* wait for the next segment */
        uint64_t header = *((uint64_t *)ep->msg_rx_desc_addr);
        free = FIELD_EX64(header, MSG_DESC, FREE);
    } while (!free);
    rc = unpack_msg_desc(&desc, ep->msg_rx_desc_addr, MSG_DESC_TARGET);
    ASSERT(!rc && "invalid RIO msg descriptor"); /* not easily recoverable */

    printf("RIO EP %s: got segment:", ep->name);
    print_msg_desc(&desc);

    ASSERT(desc.mbox < NUM_MBOXES);
    ASSERT(desc.letter < NUM_LETTERS);
    struct rx_msg *msg = rx_msg_peek(&rx_msgs[desc.mbox][desc.letter]);
    ASSERT(msg); /* FIFO is can't be empty, is replenished internally */

    printf("RIO EP %s: msg segments bitmask (pre): 0x%x", ep->name, msg->segments);
    if (msg->segments == ~0) { /* starting fresh message (first segment) */
        msg->len = desc.msg_len * desc.seg_size;
        msg->segments = (1 << desc.msg_len) - 1;
        msg->rcv_time = 0;
    } else if (msg->segments & (1 << desc.msg_seg)) {
        /* This segment already marked received, so assuming that this new
           segment belongs to the next message. This is fatal because we
           don't support more than one message in flight (i.e. previous
           message is incomplete, when the next one is already arriving). */
        printf("RIO EP %s: ERROR: more than 1 in-flight message\r\n",
                ep->name);
        ASSERT(!"more than one RIO msg in flight"); /* not easily recoverable */
    }
    vmem_cpy(msg->payload + desc.msg_seg * desc.seg_size, desc.payload_ptr, desc.seg_size);
    msg->rcv_time = desc.timestamp.rcv_time; /* keep time of latest segment */
    msg->src_id = desc.dev_id.src_id;
    msg->segments |= (1 << desc.msg_seg);
    printf("RIO EP %s: msg segments bitmask (post): 0x%x", ep->name, msg->segments);

    if (!desc.next_desc_addr) { /* chain still has unused buffers */
        ep->msg_rx_desc_addr = (uint8_t *)desc.next_desc_addr;
    } else { /* all buffers in the chain have been used */

        printf("RIO EP %s: chain %u used up, reenqeueuing%x",
               ep->name, ep->rx_chain);

        uint8_t *head_desc = &msg_rx_desc_buf[ep->rx_chain][0];
        reserve_rx_chain(head_desc, sizeof(msg_rx_desc_buf[0]));
        enqueue_rx_chain(ep, head_desc);

        ep->rx_chain = (ep->rx_chain + 1) % NUM_RX_CHAINS;
        ep->msg_rx_desc_addr = &msg_rx_desc_buf[ep->rx_chain][0];
    }

    if (msg->segments == 0) { /* all segments received */
        printf("RIO EP %s: msg for mbox:letter %u:%u completed", ep->name,
               desc.mbox, desc.letter);
        msg = rx_msg_alloc(&rx_msgs[desc.mbox][desc.letter]);
        /* not easily recoverable: consumers may be peeking at the oldest
           message, so we can't drop it. What might be implementable is to
           drop the new (just completed) message instead of oldest. */
        ASSERT(msg && "RIO RX MSG FIFO overflow");
    }
}

int rio_ep_msg_recv(struct rio_ep *ep, uint8_t mbox, uint8_t letter,
                    rio_devid_t *src, uint64_t *rcv_time,
                    uint8_t *payload, unsigned len)
{

    if (mbox >= NUM_MBOXES || letter >= NUM_LETTERS) {
        printf("RIO EP %s: ERROR: invalid mailbox/letter index: %u/%u "
               "(total available %u/%u)\r\n",
                ep->name, mbox, letter, NUM_MBOXES, NUM_LETTERS);
        return 1;
    }
    struct rx_msg *msg = rx_msg_peek(&rx_msgs[mbox][letter]);
    ASSERT(msg); /* FIFO can't be empty, replenished internally */
    while (msg->segments) { /* while msg not complete */
        receive_msg_segment(ep);
    }
    ASSERT(len >= msg->len);
    vmem_cpy(payload, msg->payload, msg->len);
    *src = msg->src_id;
    *rcv_time = msg->rcv_time;
    rx_msg_free(&rx_msgs[mbox][letter]);
    return 0;
}
