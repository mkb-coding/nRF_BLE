/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Peripheral Heart Rate over LE Coded PHY sample
 */
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/hrs.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/addr.h>

#include <zephyr/drivers/gpio.h>
#include "hal/nrf_gpio.h"

#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)

#define ADVERTISING_FRAME			5000
#define BTN_PRESS_SEND_DELAY	 	100
#define NOTIFY_INTERVAL         	1000

typedef enum{
	BT_DISCONNECTED,
	BT_CONNECTED,
	BT_ADVERTISING,
	BT_ADVERTISING_STOPPED,
}bt_state_t;

uint8_t alarm_value = 0;
bt_state_t bt_state = BT_DISCONNECTED;


static const struct gpio_dt_spec button0_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(sw0), gpios);
static struct gpio_callback button0_cb_data;

static void start_advertising(struct k_work *work);
static void stop_advertising(struct k_work *work);
static void notify_work_handler(struct k_work *work);


static K_WORK_DEFINE(start_advertising_worker, start_advertising);
static K_WORK_DELAYABLE_DEFINE(stop_advertising_worker, stop_advertising);
static K_WORK_DELAYABLE_DEFINE(notify_work, notify_work_handler);


static struct bt_le_ext_adv *adv;
static struct bt_le_adv_param adv_param;
static bt_addr_le_t bond_addr;
uint8_t paired_count = 0;

uint8_t	debug_test_DirectAdvertising = true;

// static const struct bt_data ad[] = {
// 	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
// 	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HRS_VAL)),
// 	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
// 	BT_DATA(0x55, 0x1234, 2),
// };

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HRS_VAL),
					  BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),
					  BT_UUID_16_ENCODE(BT_UUID_DIS_VAL)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN)
};

void button_pressed_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	printk("Button 0 pressed\n");


	switch(bt_state){
		case BT_CONNECTED:
			alarm_value++;
			k_work_schedule(&notify_work, K_NO_WAIT);
			break;

		case BT_DISCONNECTED:
		case BT_ADVERTISING_STOPPED:
			k_work_submit(&start_advertising_worker);
			k_work_schedule(&stop_advertising_worker, K_MSEC(ADVERTISING_FRAME));
			break;

		case BT_ADVERTISING:
			k_work_schedule(&stop_advertising_worker, K_NO_WAIT);
			break;
	}
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	int err;
	struct bt_conn_info info;
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err) {
		printk("Connection failed, err 0x%02x %s\n", conn_err, bt_hci_err_to_str(conn_err));
		return;
	}

	err = bt_conn_get_info(conn, &info);
	if (err) {
		printk("Failed to get connection info (err %d)\n", err);
	} else {
#ifdef CONFIG_BT_CTLR_PHY_CODED
		const struct bt_conn_le_phy_info *phy_info;
		phy_info = info.le.phy;

		printk("Connected: %s, tx_phy %u, rx_phy %u\n",
		       addr, phy_info->tx_phy, phy_info->rx_phy);
#endif
	}	

	k_work_schedule(&stop_advertising_worker, K_NO_WAIT);
	bt_state = BT_CONNECTED;

	err = bt_conn_set_security(conn, BT_SECURITY_L3);
	if (err) {
		printk("Failed to set security (err %d)\n", err);
	}

	// bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN); //to disconnect from

}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	volatile uint8_t debug_reason = reason;
	printk("Disconnected, reason 0x%02x %s\n", debug_reason, bt_hci_err_to_str(debug_reason));
	bt_state = BT_DISCONNECTED;

}

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
    if (err) {
        printk("Connection security failed (err %d)\n", err);
    } else {
        printk("Connection security changed: level %d\n", level);
    }
}

static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	printk("LE param req");
	return true;
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
	printk("LE param updated");
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.identity_resolved = NULL,
	.security_changed = security_changed,
	.le_param_req = le_param_req,
	.le_param_updated = le_param_updated,
	.security_changed = security_changed,
};

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Pairing cancelled: %s\n", addr);
} 

static void auth_pairing_confirm(struct bt_conn *conn)
{
    bt_conn_auth_pairing_confirm(conn);
    printk("Pairing Confirmed\n");
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
    .passkey_display = auth_passkey_display,
    .pairing_confirm = auth_pairing_confirm,
	.cancel = auth_cancel,
};

static void start_advertising(struct k_work *work)
{
	int err;

	if(debug_test_DirectAdvertising && paired_count){
		printk("Starting Direct Advertising\n");
		err = bt_le_adv_start(&adv_param, NULL, 0, NULL, 0); //Direct ADvertising
	}else{
		printk("Starting Extended Advertising\n");
		err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
	}

	if (err) {
		printk("Failed to start advertising set (err %d)\n", err);
		return;
	}

	bt_state = BT_ADVERTISING;

	printk("Advertiser %p set started\n", adv);
}

static void stop_advertising(struct k_work *work)
{
	int err;

	if(debug_test_DirectAdvertising && paired_count){
		printk("Stoping Direct Advertising\n");
		err = bt_le_adv_stop();
	}else{
		err = bt_le_ext_adv_stop(adv);
	}

	if (err) {
		printk("Failed to stop advertising set (err %d)\n", err);
		return;
	}
	if(bt_state == BT_ADVERTISING){
		bt_state = BT_ADVERTISING_STOPPED;
	} 
	printk("Advertiser %p set stopped\n", adv);
}

static void notify_work_handler(struct k_work *work)
{
	//Services data simulation.
	bt_hrs_notify(alarm_value);
}

static void setup_accept_list_cb(const struct bt_bond_info *info, void *user_data)
{
	int *bond_cnt = user_data;
	if ((*bond_cnt) < 0) {
		return;
	}
	int err = bt_le_filter_accept_list_add(&info->addr);
	printk("Added following peer to whitelist: %x %x \n",info->addr.a.val[0],info->addr.a.val[1]);
	
	bt_addr_le_copy(&bond_addr, &info->addr);

	if (err) {
		printk("Cannot add peer to Filter Accept List (err: %d)\n", err);
		(*bond_cnt) = -EIO;
	} else {
		(*bond_cnt)++;
	}
}

static int setup_accept_list(uint8_t local_id)
{
	int err;
	int bond_cnt = 0;

	err = bt_le_filter_accept_list_clear();
	if (err) {
		printk("Cannot clear Filter Accept List (err: %d)\n", err);
		return err;
	}

	bt_foreach_bond(local_id, setup_accept_list_cb, &bond_cnt);
	return bond_cnt;
}

static int create_advertising_coded(void)
{
	int err;

	static struct bt_le_adv_param param = BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_EXT_ADV,
												  				BT_GAP_ADV_FAST_INT_MIN_2,	//advertising timing
												  				BT_GAP_ADV_FAST_INT_MAX_2,
												  				NULL);

	if (IS_ENABLED(CONFIG_BT_CTLR_PHY_CODED)) {
		param.options |= BT_LE_ADV_OPT_CODED;
	}

	/* If we have got at least one bond, activate the filter */
	if (paired_count > 0) {
		printk("Bonded devices found, enabling accept list\n");

		adv_param = *BT_LE_ADV_CONN_DIR_LOW_DUTY(&bond_addr);
		adv_param.options |= BT_LE_ADV_OPT_DIR_ADDR_RPA;

		param.options |= BT_LE_ADV_OPT_FILTER_CONN | BT_LE_ADV_OPT_FILTER_SCAN_REQ;
		// | BT_LE_ADV_OPT_ONE_TIME | BT_LE_ADV_OPT_DIR_MODE_LOW_DUTY;
	}
	
	
	err = bt_le_ext_adv_create(&param, NULL, &adv);
	if (err) {
		printk("Failed to create advertiser set (err %d)\n", err);
		return err;
	}

	printk("Created adv: %p\n", adv);

	err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Failed to set advertising data (err %d)\n", err);
		return err;
	}


	return 0;
}

int main(void){
	int err;

	gpio_pin_configure_dt(&button0_spec, GPIO_INPUT);
	gpio_init_callback(&button0_cb_data, button_pressed_callback, BIT(button0_spec.pin));
	gpio_add_callback(button0_spec.port, &button0_cb_data);
	gpio_pin_interrupt_configure_dt(&button0_spec, GPIO_INT_EDGE_FALLING);
	// gpio_pin_interrupt_configure_dt(&button0_spec, GPIO_INT_LEVEL_LOW);
	

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	if (!gpio_pin_get(button0_spec.port, button0_spec.pin)){
		printk("Button 0 is pressed at start\n");
		err = bt_unpair(BT_ID_DEFAULT, BT_ADDR_LE_ANY);
		if (err) {
			printk("Failed to unpair devices: %d\n", err);
		}
	}
	
	paired_count = setup_accept_list(BT_ID_DEFAULT);

	bt_passkey_set(123123);
	
	err = bt_conn_auth_cb_register(&conn_auth_callbacks);
	if (err) {
		printk("Failed to register authorization callbacks.\n");
	 	return 0;
	}

	err = create_advertising_coded();
	if (err) {
 		printk("Advertising failed to create (err %d)\n", err);
		return 0;
	}

	for (;;) {
		k_sleep(K_FOREVER);
	}
}
