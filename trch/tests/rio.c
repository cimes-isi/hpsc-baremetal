#include <stdbool.h>

#include "printf.h"
#include "panic.h"
#include "mem.h"
#include "hwinfo.h"
#include "nvic.h"
#include "rio.h"
#include "test.h"

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

int test_rio()
{
    int rc = 1;

    nvic_int_enable(TRCH_IRQ__RIO_1);

    struct rio_ep *ep0 = rio_ep_create("RIO_EP0", RIO_EP0_BASE, RIO_DEVID_EP0);
	if (!ep0)
		goto fail_ep0;
    struct rio_ep *ep1 = rio_ep_create("RIO_EP1", RIO_EP1_BASE, RIO_DEVID_EP1);
	if (!ep1)
		goto fail_ep1;

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
