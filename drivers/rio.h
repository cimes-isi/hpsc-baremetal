#ifndef RIO_H
#define RIO_H

#include <stdint.h>

#include "regops.h"

#define RIO_MAX_PAYLOAD_SIZE 256 /* TODO: check */
#define MAX_MSG_SEGMENTS 16 /* Spec 2.3.1 */
#define MAX_MSG_SEG_SIZE 256 /* keep in sync with msg_seg_sizes list in rio.c */
#define MAX_MSG_DESC_SIZE 32 /* keep in sync with msg_desc_sizes in rio.c */

/* Common CSRs */
REG32(DEV_ID,      0x000000)
    FIELD(DEV_ID, DEVICE_IDENTITY,               0, 16)
    FIELD(DEV_ID, DEVICE_VENDOR_IDENTITY,       16, 16)

REG32(B_DEV_ID,      0x000060)
    FIELD(B_DEV_ID, BASE_DEVICE_ID,              8, 16)
    FIELD(B_DEV_ID, LARGE_BASE_DEVICE_ID,       16, 16)

/* Max supported by Praesum BRC1 EP are 16-bit IDs */
typedef uint16_t rio_devid_t;

/* Address in memory as accessed by the Rapid IO endpoint */
typedef uint64_t rio_ep_addr_t;

#define MSG_CHAIN_BUF_SIZE ((MAX_MSG_DESC_SIZE + MAX_MSG_SEG_SIZE) * MAX_MSG_SEGMENTS)

struct rio_ep {
    const char *name;
    volatile uint32_t *base;

    rio_ep_addr_t buf_mem_ep;
    uint8_t *buf_mem_cpu;
    unsigned buf_mem_size;

    uint8_t (*msg_tx_desc_buf);
    unsigned msg_tx_desc_buf_size;

    uint8_t (*msg_rx_desc_buf)[MSG_CHAIN_BUF_SIZE];
    unsigned msg_rx_desc_buf_size;

    rio_devid_t devid;
    uint8_t *msg_rx_desc_addr;
    unsigned rx_chain;
};

enum rio_transport_type {
    RIO_TRANSPORT_DEV8,
    RIO_TRANSPORT_DEV16,
    RIO_TRANSPORT_DEV32, /* not supported by Praesum BRC1 EP */
};

/* Tabel 4-1 in RIO Spec P1 */
enum rio_ftype {
    RIO_FTYPE_REQ           =  2,
    RIO_FTYPE_WRITE         =  5,
    RIO_FTYPE_STREAM_WRITE  =  6,
    RIO_FTYPE_MAINT         =  8,
    RIO_FTYPE_DOORBELL      = 10,
    RIO_FTYPE_MSG           = 11,
    RIO_FTYPE_RESP          = 13,
};

enum rio_transaction {
    RIO_TRANS_MAINT_REQ_READ        = 0b0000,
    RIO_TRANS_MAINT_REQ_WRITE       = 0b0001,
    RIO_TRANS_MAINT_RESP_READ       = 0b0010,
    RIO_TRANS_MAINT_RESP_WRITE      = 0b0011,
    RIO_TRANS_MAINT_REQ_PORT_WRITE  = 0b0100,
};

/* Table 4-7 in Spec */
enum rio_status {
    RIO_STATUS_DONE     = 0x0000,
    RIO_STATUS_ERROR    = 0b0111,
};

/* Struct sufficiently general to hold a packet of any type */
/* TODO: consider splitting into physical, logical, and transport? */
struct rio_pkt {
    /* Physical Layer */
    uint8_t prio;

    /* Transport Layer */
    enum rio_transport_type ttype;
    rio_devid_t dest_id;
    rio_devid_t src_id;

    /* Logical Layer */
    enum rio_ftype ftype;
    uint8_t src_tid;
    uint8_t target_tid;

    /* Maintainance Class */
    enum rio_transaction transaction;
    uint16_t rdwr_bytes;
    uint64_t rdwr_mask;
    uint8_t wdptr;
    uint8_t status;
    uint32_t config_offset;
    unsigned payload_len;

    /* Messaging Layer */
    uint8_t msg_len;
    uint8_t seg_size;
    uint8_t mbox;
    uint8_t letter;
    uint8_t msg_seg;

    /* could do variable len tail array ([0]) and use obj allocator */
    uint64_t payload[RIO_MAX_PAYLOAD_SIZE];
};

void rio_print_pkt(struct rio_pkt *pkt);

struct rio_ep *rio_ep_create(const char *name, volatile uint32_t *base,
                             rio_devid_t devid,
                             rio_ep_addr_t buf_mem_ep, uint8_t *buf_mem_cpu,
                             unsigned buf_mem_size);
int rio_ep_destroy(struct rio_ep *ep);
int rio_ep_sp_send(struct rio_ep *ep, struct rio_pkt *pkt);
int rio_ep_sp_recv(struct rio_ep *ep, struct rio_pkt *pkt);
int rio_ep_read_csr(struct rio_ep *ep, uint64_t *data, rio_devid_t dest,
                    uint32_t offset, uint16_t len, uint64_t mask);
int rio_ep_write_csr(struct rio_ep *ep, const uint64_t *data,rio_devid_t dest,
                     uint32_t offset, uint16_t len, uint64_t mask);
int rio_ep_read_csr_32(struct rio_ep *ep, uint32_t *data,
                       rio_devid_t dest, uint32_t offset);
int rio_ep_write_csr_32(struct rio_ep *ep, const uint32_t *data,
                        rio_devid_t dest, uint32_t offset);

int rio_ep_msg_send(struct rio_ep *ep, rio_devid_t dest, uint64_t launch_time,
                    uint8_t mbox, uint8_t letter, uint8_t seg_size,
                    uint8_t *payload, unsigned len);
int rio_ep_msg_recv(struct rio_ep *ep, uint8_t mbox, uint8_t letter,
                    rio_devid_t *src, uint64_t *rcv_time,
                    uint8_t *payload, unsigned *size);

#endif // RIO_H
