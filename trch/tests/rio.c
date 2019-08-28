#include <stdbool.h>

#include "printf.h"
#include "panic.h"
#include "mem.h"
#include "hwinfo.h"
#include "mem-map.h"
#include "nvic.h"
#include "dma.h"
#include "bit.h"
#include "rio.h"
#include "test.h"

#define MSG_SEG_SIZE 16

static struct rio_pkt in_pkt, out_pkt;

static int test_send_receive(struct rio_ep *ep0, struct rio_ep *ep1)
{
	int rc;

	printf("RIO TEST: running send receive test...\r\n");

    out_pkt.ttype = RIO_TRANSPORT_DEV8;
    out_pkt.src_id = RIO_DEVID_EP0;
    out_pkt.dest_id = RIO_DEVID_EP1;

    out_pkt.ftype = RIO_FTYPE_MAINT;
    out_pkt.target_tid = 0x3; /* TODO: to deliver to SW via SP RX, must not be 0x0,0x1,0x2 */
    out_pkt.transaction = RIO_TRANS_MAINT_RESP_READ; /* TODO: see above */

	out_pkt.status = 0x1;

    /* TODO: compose an actual CAP/CSR read, for now assuming echo */
    out_pkt.config_offset = 0x0;
    out_pkt.payload_len = 0x1;
    out_pkt.payload[0] = 0xdeadbeef;

    printf("RIO TEST: sending pkt on EP 0:\r\n");
    rio_print_pkt(&out_pkt);

	rc = rio_ep_sp_send(ep0, &out_pkt);
	if (rc)
		return rc;

    rc = rio_ep_sp_recv(ep1, &in_pkt);
	if (rc)
		return rc;

    printf("RIO TEST: received pkt on EP 1:\r\n");
    rio_print_pkt(&in_pkt);

    /* TODO: receive packets until expected response or timeout,
     * instead of blocking on receive and checking the first pkt */
    /* TODO: more checks */
    if (!(in_pkt.ftype == RIO_FTYPE_MAINT &&
    	  in_pkt.transaction == out_pkt.transaction && /* TODO: see above */
          in_pkt.target_tid == out_pkt.target_tid &&
          in_pkt.payload_len == 1 && in_pkt.payload[0] == 0xdeadbeef)) {
        printf("RIO TEST: ERROR: receive packet does not match sent packet\r\n");
        return rc;
    }
	
	return 0;
}

static int test_read_csr(struct rio_ep *ep0, struct rio_ep *ep1)
{
	uint32_t dev_id;
	uint32_t expected_dev_id = 0xdeadbeef;
	int rc;

	printf("RIO TEST: running read CSR test...\r\n");

	rc = rio_ep_read_csr_32(ep0, &dev_id, RIO_DEVID_EP1, DEV_ID);
	if (rc)
		return rc;
	if (dev_id != expected_dev_id) {
		printf("RIO TEST EP %s: unexpected value of DEV_ID CSR in EP1: %x (expected %x)\r\n",
			   ep0->name, dev_id, expected_dev_id);
		return 1;	
	}

	rc = rio_ep_read_csr_32(ep1, &dev_id, RIO_DEVID_EP0, DEV_ID);
	if (rc)
		return rc;
	if (dev_id != expected_dev_id) {
		printf("RIO TEST EP %s: unexpected value of DEV_ID CSR in EP0: %x (expected %x)\r\n",
			   ep1->name, dev_id, expected_dev_id);
		return 2;	
	}

	return 0;
}

static int test_write_csr(struct rio_ep *ep0, struct rio_ep *ep1)
{
	uint32_t device_id = 0x0000aa00, read_device_id = 0x0;
	int rc;

	printf("RIO TEST: running write CSR test...\r\n");

	rc = rio_ep_write_csr_32(ep0, &device_id, RIO_DEVID_EP1, B_DEV_ID);
	if (rc)
		return rc;

	rc = rio_ep_read_csr_32(ep0, &read_device_id, RIO_DEVID_EP1, B_DEV_ID);
	if (rc)
		return rc;
	if (read_device_id != device_id) {
		printf("RIO TEST EP %s: read value differs from written: "
               "B_DEV_ID = %x (written %x)\r\n",
			   ep1->name, read_device_id, device_id);
		return 2;
	}

	return 0;
}

static int test_write_csr_byte(struct rio_ep *ep0, struct rio_ep *ep1)
{
    /* Only 0xccbb should be written, but all should be read. */
    /* NOTE: This test depends on test_write_csr() */
	uint32_t large_device_id = 0xccbb0000, read_device_id = 0x0;
    uint32_t device_id = 0x0000aa00 | large_device_id;
	int rc;

	printf("RIO TEST: running write CSR by bytes test...\r\n");

    uint64_t value = large_device_id;
	rc = rio_ep_write_csr(ep0, &value, RIO_DEVID_EP1, B_DEV_ID, 2, 0xffff0000);
	if (rc)
		return rc;

	rc = rio_ep_read_csr_32(ep0, &read_device_id, RIO_DEVID_EP1, B_DEV_ID);
	if (rc)
		return rc;
	if (read_device_id != device_id) {
		printf("RIO TEST EP %s: read value differs from written: "
               "B_DEV_ID = %x (written %x)\r\n",
			   ep1->name, read_device_id, device_id);
		return 2;
	}

	return 0;
}

static int test_msg(struct rio_ep *ep0, struct rio_ep *ep1)
{
    int rc;

	printf("RIO TEST: running message test...\r\n");

    uint64_t payload = 0xdeadbeef;

    rc = rio_ep_msg_send(ep0, RIO_DEVID_EP1, /* launch_time */ 0, 
                         /* mbox */ 0, /* letter */ 0, /* seg_size */ 8,
                        (uint8_t *)&payload, sizeof(payload));
    if (rc)
        return rc;

#if 0
    struct rio_pkt in_pkt;

    rc = rio_ep_sp_recv(ep1, &in_pkt);
	if (rc)
		return rc;

    printf("RIO TEST: received pkt on EP 1:\r\n");
    rio_print_pkt(&in_pkt);

    if (!(in_pkt.ftype == RIO_FTYPE_MSG)) {
        printf("RIO TEST: ERROR: received packet is not a message packet\r\n");
        return rc;
    }
#else
    rio_devid_t src_id = 0;
    uint64_t rcv_time = 0;
    uint64_t rx_payload = 0;
    unsigned payload_len = sizeof(rx_payload);
    rc = rio_ep_msg_recv(ep1, /* mbox */ 0, /* letter */ 0,
                         &src_id, &rcv_time,
                         (uint8_t *)&rx_payload, &payload_len);
    if (rc)
        return rc;

    printf("RIO TEST: recved msg from %u at %08x%08x payload len %u %08x%08x\r\n",
           src_id, (uint32_t)(rcv_time >> 32), (uint32_t)(rcv_time & 0xffffffff),
           payload_len,
           (uint32_t)(rx_payload >> 32), (uint32_t)(rx_payload & 0xffffffff));

    if (rx_payload != payload) {
        printf("RIO TEST: ERROR: received msg payload mismatches sent\r\n");
        return 1;
    }
#endif
    return 0;
}

#if 0
static uint8_t test_buf1[1024] __aligned__((256));
static uint8_t test_buf2[1024] __aligned__((256));
#else
static uint8_t *test_buf1 = (uint8_t *)(RIO_MEM_TEST_WIN_ADDR +    0x0);
static uint8_t *test_buf2 = (uint8_t *)(RIO_MEM_TEST_WIN_ADDR + 0x1000);
static unsigned test_buf_size = RIO_MEM_TEST_SIZE / 2;
#endif

static int test_map(struct rio_ep *ep0, struct rio_ep *ep1, struct dma *dmac)
{
    int rc;

	printf("RIO TEST: running map test...\r\n");

    /* TODO: setup map */
#if 0
    if (rc)
        return rc;
#else
    (void)rc;
#endif

#define MAP_REGION_OFFSET    0x00440000 /*  in Implementation Registers section (? TODO) */
    memset(test_buf1, 0x42, test_buf_size);

    uint8_t *map_reg = (uint8_t *)((uint8_t *)RIO_EP0_BASE + MAP_REGION_OFFSET);

#if 0
    vmem_cpy(map_reg, test_buf1, sizeof(test_buf1));
    vmem_cpy(test_buf2, map_reg, sizeof(test_buf2));
#else
    /* TODO: use HPPS SRIO DMA */
    unsigned tx_size = ALIGN(test_buf_size, DMA_MAX_BURST_BITS);
    struct dma_tx *dtx = dma_transfer(dmac, /* chan */ 0, // TODO: allocate channel per user
        (uint32_t *)test_buf1, (uint32_t *)map_reg, tx_size,
        NULL, NULL /* no callback */);
    rc = dma_wait(dtx);
    if (rc) {
        printf("TEST RIO map: DMA transfer to map reg failed: rc %u\r\n", rc);
        return rc;
    }
    dtx = dma_transfer(dmac, /* chan */ 0, // TODO: allocate channel per user
        (uint32_t *)map_reg, (uint32_t *)test_buf2, tx_size,
        NULL, NULL /* no callback */);
    rc = dma_wait(dtx);
    if (rc) {
        printf("TEST RIO map: DMA transfer from map reg failed: rc %u\r\n", rc);
        return rc;
    }
#endif

    for (int i = 0; i < sizeof(test_buf1); ++i) {
        if (test_buf1[i] != test_buf2[i]) {
            printf("RIO TEST map: FAILED: mismatch in data written to map region\r\n");
            return 1;
        }
    }

    printf("RIO TEST map: PASS\r\n");

    return 0;
}

int test_rio(struct dma *dmac)
{
    int rc = 1;

    nvic_int_enable(TRCH_IRQ__RIO_1);

    /* Partition buffer memory evenly among the endpoints */
    const unsigned buf_mem_size = RIO_MEM_SIZE / 2;
    uint8_t *buf_mem_cpu = (uint8_t *)RIO_MEM_WIN_ADDR;
    rio_ep_addr_t buf_mem_ep = RIO_MEM_ADDR;

    struct rio_ep *ep0 = rio_ep_create("RIO_EP0", RIO_EP0_BASE, RIO_DEVID_EP0,
                                       buf_mem_ep, buf_mem_cpu, buf_mem_size);
	if (!ep0)
		goto fail_ep0;
    buf_mem_ep += buf_mem_size;
    buf_mem_cpu += buf_mem_size;

    struct rio_ep *ep1 = rio_ep_create("RIO_EP1", RIO_EP1_BASE, RIO_DEVID_EP1,
                                       buf_mem_ep, buf_mem_cpu, buf_mem_size);
	if (!ep1)
		goto fail_ep1;
    buf_mem_ep += buf_mem_size;
    buf_mem_cpu += buf_mem_size;

#if 0
	rc = test_send_receive(ep0, ep1);
	if (rc) {
		printf("RIO TEST: FAILED: send_receive test failed\r\n");
		goto fail;
	}

	rc = test_read_csr(ep0, ep1);
	if (rc) {
		printf("RIO TEST: FAILED: read_csr test failed\r\n");
		goto fail;
	}

	rc = test_write_csr(ep0, ep1);
	if (rc) {
		printf("RIO TEST: FAILED: write_csr test failed\r\n");
		goto fail;
	}

	rc = test_write_csr_byte(ep0, ep1);
	if (rc) {
		printf("RIO TEST: FAILED: write_csr test failed\r\n");
		goto fail;
	}

    rc = test_msg(ep0, ep1);
    if (rc) {
	    printf("RIO TEST: FAILED: messsage test failed\r\n");
	    goto fail;
    }
#else
    (void)test_send_receive;
    (void)test_read_csr;
    (void)test_write_csr;
    (void)test_write_csr_byte;
    (void)test_msg;
#endif

    rc = test_map(ep0, ep1, dmac);
    if (rc) {
	    printf("RIO TEST: FAILED: map test failed\r\n");
	    goto fail;
    }

fail:
    rio_ep_destroy(ep1);
fail_ep1:
    rio_ep_destroy(ep0);
fail_ep0:
    nvic_int_disable(TRCH_IRQ__RIO_1);
    return rc;
}

void rio_1_isr()
{
    nvic_int_disable(TRCH_IRQ__RIO_1);
}
