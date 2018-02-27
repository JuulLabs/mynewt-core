#include "syscfg/syscfg.h"
#include "nimble/ble_hci_trans.h"
#include "ble_hs_priv.h"

#if MYNEWT_VAL(BLE_HS_FLOW_CTRL)

#define BLE_HS_FLOW_ITVL_TICKS  \
    (MYNEWT_VAL(BLE_HS_FLOW_CTRL_ITVL) * OS_TICKS_PER_SEC / 1000)

/**
 * The number of freed buffers since the most-recent
 * number-of-completed-packets event was sent.  This is used to determine if an
 * immediate event transmission is required.
 */
static uint16_t ble_hs_flow_num_completed_pkts;

/** Periodically sends number-of-completed-packets events.  */
static struct os_callout ble_hs_flow_timer;

static os_event_fn ble_hs_flow_event_cb;

static struct os_event ble_hs_flow_ev = {
    .ev_cb = ble_hs_flow_event_cb,
};

static int
ble_hs_flow_tx_num_comp_pkts(void)
{
    uint8_t buf[
        BLE_HCI_HOST_NUM_COMP_PKTS_HDR_LEN + 
        BLE_HCI_HOST_NUM_COMP_PKTS_ENT_LEN
    ];
    struct hci_host_num_comp_pkts_entry entry;
    struct ble_hs_conn *conn;
    int rc;

    BLE_HS_DBG_ASSERT(ble_hs_locked_by_cur_task());

    /* For each connection with completed packets, send a separate
     * host-number-of-completed-packets command.
     */
    for (conn = ble_hs_conn_first();
         conn != NULL;
         conn = SLIST_NEXT(conn, bhc_next)) {

        if (conn->bhc_completed_pkts > 0) {
            /* Only specify one connection per command. */
            buf[0] = 1;

            /* Append entry for this connection. */
            entry.conn_handle = conn->bhc_handle;
            entry.num_pkts = conn->bhc_completed_pkts;
            rc = ble_hs_hci_cmd_build_host_num_comp_pkts_entry(
                &entry,
                buf + BLE_HCI_HOST_NUM_COMP_PKTS_HDR_LEN,
                sizeof buf - BLE_HCI_HOST_NUM_COMP_PKTS_HDR_LEN);
            BLE_HS_DBG_ASSERT(rc == 0);

            conn->bhc_completed_pkts = 0;

            /* The host-number-of-completed-packets command does not elicit a
             * response from the controller, so don't use the normal blocking
             * HCI API when sending it.
             */
            rc = ble_hs_hci_cmd_send_buf(
                BLE_HCI_OP(BLE_HCI_OGF_CTLR_BASEBAND,
                           BLE_HCI_OCF_CB_HOST_NUM_COMP_PKTS),
                buf, off);
            if (rc != 0) {
                return rc;
            }
        }
    }

    return 0;
}

static void
ble_hs_flow_event_cb(struct os_event *ev)
{
    int rc;

    ble_hs_lock();

    if (ble_hs_flow_num_completed_pkts > 0) {
        rc = ble_hs_flow_tx_num_comp_pkts();
        if (rc != 0) {
            ble_hs_sched_reset(rc);
        }

        ble_hs_flow_num_completed_pkts = 0;
    }

    ble_hs_unlock();
}

static void
ble_hs_flow_inc_completed_pkts(struct ble_hs_conn *conn)
{
    uint16_t num_free;

    int rc;

    BLE_HS_DBG_ASSERT(ble_hs_locked_by_cur_task());

    conn->bhc_completed_pkts++;
    ble_hs_flow_num_completed_pkts++;

    if (ble_hs_flow_num_completed_pkts > MYNEWT_VAL(BLE_ACL_BUF_COUNT)) {
        ble_hs_sched_reset(BLE_HS_ECONTROLLER);
        return;
    }

    /* If the number of free buffers is at or below the configured threshold,
     * send an immediate number-of-copmleted-packets event.
     */
    num_free = MYNEWT_VAL(BLE_ACL_BUF_COUNT) - ble_hs_flow_num_completed_pkts;
    if (num_free <= MYNEWT_VAL(BLE_HS_FLOW_CTRL_THRESH)) {
        os_eventq_put(ble_hs_evq_get(), &ble_hs_flow_ev);
        os_callout_stop(&ble_hs_flow_timer);
    } else if (ble_hs_flow_num_completed_pkts == 1) {
        rc = os_callout_reset(&ble_hs_flow_timer, BLE_HS_FLOW_ITVL_TICKS);
        BLE_HS_DBG_ASSERT_EVAL(rc == 0);
    }
}

static os_error_t
ble_hs_flow_acl_free(struct os_mempool_ext *mpe, void *data, void *arg)
{
    struct ble_hs_conn *conn;
    const struct os_mbuf *om;
    uint16_t conn_handle;
    int rc;

    om = data;

    /* An ACL data packet must be a single mbuf, and it must contain the
     * corresponding connection handle in its user header.
     */
    assert(OS_MBUF_IS_PKTHDR(om));
    assert(OS_MBUF_USRHDR_LEN(om) >= sizeof conn_handle);

    /* Copy the connection handle out of the mbuf. */
    memcpy(&conn_handle, OS_MBUF_USRHDR(om), sizeof conn_handle);

    /* Free the mbuf back to its pool. */
    rc = os_memblock_put_from_cb(&mpe->mpe_mp, data);
    if (rc != 0) {
        return rc;
    }

    /* Allow nested locks - there are too many places where acl buffers can get
     * freed.
     */
    ble_hs_lock_nested();
    
    conn = ble_hs_conn_find(conn_handle);
    if (conn != NULL) {
        ble_hs_flow_inc_completed_pkts(conn);
    }

    ble_hs_unlock_nested();

    return 0;
}
#endif /* MYNEWT_VAL(BLE_HS_FLOW_CTRL) */

void
ble_hs_flow_connection_broken(uint16_t conn_handle)
{
#if MYNEWT_VAL(BLE_HS_FLOW_CTRL) &&                 \
    MYNEWT_VAL(BLE_HS_FLOW_CTRL_TX_ON_DISCONNECT)
    ble_hs_lock();
    ble_hs_flow_tx_num_comp_pkts(); 
    ble_hs_unlock();
#endif
}

/**
 * Fills the user header of an incoming data packet.  On function return, the
 * header contains the connection handle associated with the sender.
 *
 * If flow control is disabled, this function is a no-op.
 */
void
ble_hs_flow_fill_acl_usrhdr(struct os_mbuf *om)
{
#if MYNEWT_VAL(BLE_HS_FLOW_CTRL)
    const struct hci_data_hdr *hdr;
    uint16_t *conn_handle;

    BLE_HS_DBG_ASSERT(OS_MBUF_USRHDR_LEN(om) >= sizeof *conn_handle);
    conn_handle = OS_MBUF_USRHDR(om);

    hdr = (void *)om->om_data;
    *conn_handle = BLE_HCI_DATA_HANDLE(hdr->hdh_handle_pb_bc);
#endif
}

/**
 * Sends the HCI commands to the controller required for enabling host flow
 * control.
 *
 * If flow control is disabled, this function is a no-op.
 */
int
ble_hs_flow_startup(void)
{
#if MYNEWT_VAL(BLE_HS_FLOW_CTRL)
    struct hci_host_buf_size buf_size_cmd;
    int rc;

    /* Assume failure. */
    ble_hci_trans_set_acl_free_cb(NULL, NULL);
    os_callout_stop(&ble_hs_flow_timer);

    rc = ble_hs_hci_cmd_tx_set_ctlr_to_host_fc(BLE_HCI_CTLR_TO_HOST_FC_ACL);
    if (rc != 0) {
        return rc;
    }

    buf_size_cmd = (struct hci_host_buf_size) {
        .acl_pkt_len = MYNEWT_VAL(BLE_ACL_BUF_SIZE),
        .num_acl_pkts = MYNEWT_VAL(BLE_ACL_BUF_COUNT),
    };
    rc = ble_hs_hci_cmd_tx_host_buf_size(&buf_size_cmd);
    if (rc != 0) {
        ble_hs_hci_cmd_tx_set_ctlr_to_host_fc(BLE_HCI_CTLR_TO_HOST_FC_OFF);
        return rc;
    }

    /* Flow control successfully enabled. */
    ble_hs_flow_num_completed_pkts = 0;
    ble_hci_trans_set_acl_free_cb(ble_hs_flow_acl_free, NULL);
    os_callout_init(&ble_hs_flow_timer, ble_hs_evq_get(),
                    ble_hs_flow_event_cb, NULL);
#endif

    return 0;
}
