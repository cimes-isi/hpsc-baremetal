#include <stdbool.h>

#include "printf.h"
#include "panic.h"
#include "hwinfo.h"
#include "nvic.h"
#include "rio.h"
#include "test.h"

static struct rio_pkt in_pkt, out_pkt;

int test_rio()
{
    int rc = 1;

    nvic_int_enable(TRCH_IRQ__RIO_1);

    struct rio_ep *ep0 = rio_ep_create("RIO_EP0", RIO_EP0_BASE);
    struct rio_ep *ep1 = rio_ep_create("RIO_EP1", RIO_EP1_BASE);

    out_pkt.ttype = RIO_TRANSPORT_DEV8,
    out_pkt.src_id = RIO_DEVID_EP0,
    out_pkt.dest_id = RIO_DEVID_EP1,

    out_pkt.ftype = RIO_FTYPE_MAINT,
    out_pkt.target_tid = 0x3; /* TODO: to deliver to SW via SP RX, must not be 0x0,0x1,0x2 */
    out_pkt.transaction = RIO_TRANS_MAINT_REQ_READ,
    out_pkt.size = 4,
    /* TODO: compose an actual CAP/CSR read, for now assuming echo */
    out_pkt.config_offset = 0x0,
    out_pkt.payload_len = 0x1;
    out_pkt.payload[0] = 0xdeadbeef;

    printf("RIO TEST: sending pkt on EP 0:\r\n");
    rio_print_pkt(&out_pkt);

	rc = rio_ep_sp_send(ep0, &out_pkt);
	if (rc)
		goto fail_send;

    rc = rio_ep_sp_recv(ep1, &in_pkt);
	if (rc)
		goto fail_recv;

    printf("RIO TEST: received pkt on EP 1:\r\n");
    rio_print_pkt(&in_pkt);

    /* TODO: receive packets until expected response or timeout,
     * instead of blocking on receive and checking the first pkt */
    /* TODO: more checks */
    if (!(in_pkt.ftype == RIO_FTYPE_MAINT &&
          in_pkt.target_tid == out_pkt.target_tid &&
          in_pkt.payload_len == 1 && in_pkt.payload[0] == 0xdeadbeef)) {
        printf("RIO TEST: ERROR: bad response to MAINTENANCE request\r\n");
        goto fail_resp;
    }

	rc = 0;

fail_resp:
    rio_ep_destroy(ep1);
fail_recv:
    rio_ep_destroy(ep0);
fail_send:
    nvic_int_disable(TRCH_IRQ__RIO_1);
    return rc;
}

void rio_1_isr()
{
    nvic_int_disable(TRCH_IRQ__RIO_1);
}
