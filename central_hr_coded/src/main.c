/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Central over LE Coded PHY sample
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>
#include <bluetooth/services/hrs_client.h>
#include <zephyr/settings/settings.h>
#include "devicetree_generated.h"

#include <zephyr/drivers/gpio.h>
#include "hal/nrf_gpio.h"
#include <zephyr/drivers/uart.h>

#define BT_TEST_TYPE BT_SCAN_TYPE_SCAN_ACTIVE

#define LED_RED 	(1 << 0)
#define LED_GREEN 	(1 << 1)
#define LED_BLUE 	(1 << 2)

typedef enum {
	BT_STOPPED,
	BT_SCANNING,
} bt_scan_status_t;

struct hrs_entry {
	struct bt_hrs_client client;
	struct bt_conn *conn;
	uint8_t id;
};

static struct bt_hrs_client hrs_c;
static struct hrs_entry hrs_clients[CONFIG_BT_MAX_CONN];
static uint8_t next_id = 1;

static struct bt_conn *default_conn;
static uint8_t volatile conn_count;
bt_scan_status_t bt_scan_status = BT_STOPPED;
bool bt_scan_enabled = false;

static bt_addr_le_t bond_addr;

static const struct gpio_dt_spec ledred_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(ledr), gpios);
static const struct gpio_dt_spec ledgreen_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(ledg), gpios);
static const struct gpio_dt_spec ledblue_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(ledb), gpios);
static const struct gpio_dt_spec button0_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(sw0), gpios);

//**for dev kit
// static const struct gpio_dt_spec ledred_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(led2), gpios);
// static const struct gpio_dt_spec ledgreen_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(led1), gpios);
// static const struct gpio_dt_spec ledblue_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(led0), gpios);
// static const struct gpio_dt_spec button0_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(button0), gpios);

static struct gpio_callback button0_cb_data;

static void scan_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(scan_work, scan_work_handler);

void alarm_beep(int bilau){
	// gpio_pin_set_dt(&rele0_spec, 0);
	gpio_pin_set_dt(&ledgreen_spec, 1);
	gpio_pin_set_dt(&ledred_spec, 1);
	k_sleep(K_MSEC(bilau));
	gpio_pin_set_dt(&ledgreen_spec, 0);
	gpio_pin_set_dt(&ledred_spec, 0);
	// gpio_pin_set_dt(&rele0_spec, 1);
}

void blinky_led(uint8_t led_rgb, uint16_t mdelay){

	if(led_rgb & LED_RED){
		gpio_pin_set_dt(&ledred_spec, 1);
	}
	if(led_rgb & LED_GREEN){
		gpio_pin_set_dt(&ledgreen_spec, 1);
	}
	if(led_rgb & LED_BLUE){
		gpio_pin_set_dt(&ledblue_spec, 1);
	}
		
	k_sleep(K_MSEC(mdelay));

	gpio_pin_set_dt(&ledred_spec, 0);
	gpio_pin_set_dt(&ledgreen_spec, 0);
	gpio_pin_set_dt(&ledblue_spec, 0);
}
void button_pressed_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	printk("Button 0 pressed\n");

	bt_scan_enabled = !bt_scan_enabled;
	k_work_schedule(&scan_work, K_NO_WAIT);
}

static void notify_func(struct bt_hrs_client *hrs_c, const struct bt_hrs_client_measurement *meas, int err){

	if (err) {
		printk("Error during receiving notification, err: %d\n", err);
		return;
	}

	// Obtém o endereço do dispositivo remoto
    char addr_str[BT_ADDR_LE_STR_LEN] = {0};
    const bt_addr_le_t *peer_addr = bt_conn_get_dst(hrs_c->conn);
    bt_addr_le_to_str(peer_addr, addr_str, sizeof(addr_str));

	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		if (&hrs_clients[i].client == hrs_c) {
			printk("\n\t ID[%d]Addr[%s]-> Received Value: %d \n", hrs_clients[i].id, addr_str, meas->hr_value);
			printk("\n");
			break;
		}
	}

	if(meas->hr_value){
		alarm_beep(500);
	}
	// gpio_pin_set_dt(&rele0_spec, meas->hr_value);
}

static void discover_hrs_completed(struct bt_gatt_dm *dm, void *ctx)
{
	int err;

	printk("The discovery procedure succeeded\n");
	bt_gatt_dm_data_print(dm);

	struct bt_conn *conn = bt_gatt_dm_conn_get(dm);

	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		if (hrs_clients[i].conn == NULL) {
			hrs_clients[i].conn = conn;
			hrs_clients[i].id = next_id++;
			bt_hrs_client_init(&hrs_clients[i].client);
			bt_hrs_client_handles_assign(dm, &hrs_clients[i].client);
			err = bt_hrs_client_measurement_subscribe(&hrs_clients[i].client, notify_func);
			if (err && err != -EALREADY) {
				printk("Subscribe failed (err %d)\n", err);
			} else {
				printk("[SUBSCRIBED]\n");
			}
			hrs_clients[i].client.conn = conn;
			break;
		}
	}
	
	err = bt_gatt_dm_data_release(dm);
	if (err) {
		printk("Could not release the discovery data (err %d)\n", err);
	}
}

static void discover_hrs_service_not_found(struct bt_conn *conn, void *ctx)
{
	printk("No more services\n");
}

static void discover_hrs_error_found(struct bt_conn *conn, int err, void *ctx)
{
	printk("The discovery procedure failed, err %d\n", err);
}


static struct bt_gatt_dm_cb discover_hrs_cb = {
	.completed = discover_hrs_completed,
	.service_not_found = discover_hrs_service_not_found,
	.error_found = discover_hrs_error_found,
};

static void scan_filter_match(struct bt_scan_device_info *device_info, struct bt_scan_filter_match *filter_match, bool connectable){

	if(default_conn)	{
		printk("Already processing a connection\n");
		return;
	}

	int err;
	char addr[BT_ADDR_LE_STR_LEN];
	struct bt_conn_le_create_param *create_params;
	struct bt_le_conn_param *conn_param;

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

	printk("Filters matched. Address: %s connectable: %s\n", addr, connectable ? "yes" : "no");

	err = bt_scan_stop();
	if (err) {
		printk("Stop LE scan failed (err %d)\n", err);
	}
	bt_scan_status = BT_STOPPED;
	bt_scan_enabled = false;

	create_params 	= BT_CONN_LE_CREATE_PARAM(BT_CONN_LE_OPT_NONE, BT_GAP_SCAN_FAST_INTERVAL, BT_GAP_SCAN_FAST_INTERVAL);
	conn_param 		= BT_LE_CONN_PARAM(BT_GAP_INIT_CONN_INT_MIN, BT_GAP_INIT_CONN_INT_MAX, 0, 800);
	
	if (IS_ENABLED(CONFIG_BT_CTLR_PHY_CODED)) {
		create_params->options |=	BT_CONN_LE_OPT_CODED | BT_CONN_LE_OPT_NO_1M;
	}

	// err = bt_conn_le_create(device_info->recv_info->addr, create_params, conn_param, &default_conn);
	err = bt_conn_le_create(device_info->recv_info->addr, create_params, device_info->conn_param, &default_conn);
	if (err) {
		printk("Create conn failed (err %d)\n", err);
	}

	blinky_led(LED_GREEN, 100);
	printk("Connection pending\n");
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	printk("Connecting failed.");
}

static void scan_connecting(struct bt_scan_device_info *device_info, struct bt_conn *conn)
{
	printk("Scan connecting.");
}

static void scan_filter_no_match(struct bt_scan_device_info *device_info, bool connectable)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));
	// printk("Filters no match. Address: %s connectable: %s", addr, connectable ? "yes" : "no");

	if (device_info->recv_info->adv_type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));
		printk("Direct advertising received from %s", addr);
		bt_scan_stop();

		err = bt_conn_le_create(device_info->recv_info->addr, BT_CONN_LE_CREATE_CONN, device_info->conn_param, &default_conn);
		if (err) {
			printk("An error has occur!(%d)", err);
		}
	}
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, scan_filter_no_match, scan_connecting_error, scan_connecting);

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
	int err = bt_le_filter_accept_list_clear();
	if (err) {
		printk("Cannot clear Filter Accept List (err: %d)\n", err);
		return err;
	}
	int bond_cnt = 0;
	bt_foreach_bond(local_id, setup_accept_list_cb, &bond_cnt);

	printk("Previous Bonded devices (%d)\n", bond_cnt);
	return bond_cnt;
}

static void scan_init(void)
{
	int err;
	uint16_t bond_dev_cnt;
	/* Use active scanning and disable duplicate filtering to handle any
	 * devices that might update their advertising data at runtime. */
	struct bt_le_scan_param scan_param = {
		.type     = BT_TEST_TYPE,
		.interval = BT_GAP_SCAN_FAST_INTERVAL,
		.window   = BT_GAP_SCAN_FAST_WINDOW,
		.options  = BT_LE_SCAN_OPT_NONE,
	};


	if (IS_ENABLED(CONFIG_BT_CTLR_PHY_CODED)) {
		scan_param.options |= BT_LE_SCAN_OPT_CODED | BT_LE_SCAN_OPT_NO_1M;
	}

	bond_dev_cnt = setup_accept_list(BT_ID_DEFAULT) ;

	/* If we have got at least one bond, activate the filter */
	if (bond_dev_cnt > 0) {
		printk("Bonded devices found, enabling accept list\n");
		scan_param.options |= BT_LE_SCAN_OPT_FILTER_ACCEPT_LIST;
	}

	struct bt_scan_init_param scan_init = {
		.connect_if_match = true, //auto connect
		.scan_param = &scan_param,
		.conn_param = BT_LE_CONN_PARAM_DEFAULT
	};

	bt_scan_init(&scan_init);
	bt_scan_cb_register(&scan_cb);

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_HRS);
	if (err) {
		printk("Scanning filters cannot be set (err %d)\n", err);
		return;
	}

	// if (bond_dev_cnt > 0) {
	// 	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_ADDR, &bond_addr);
	// 	if (err) {
	// 		printk("Scanning filters cannot be set (err %d)\n", err);
	// 		return;
	// 	}
	// }

	err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
	if (err) {
		printk("Filters cannot be turned on (err %d)\n", err);
	}
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	int err;
	struct bt_conn_info info;
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	uint8_t debug = conn_err;

	if (conn_err) {
		printk("Failed to connect to %s, 0x%02x %s\n", addr, debug, bt_hci_err_to_str(conn_err));

		bt_conn_unref(default_conn);
		default_conn = NULL;

		return;
	}

	err = bt_conn_get_info(conn, &info);
	if (err) {
		printk("Failed to get connection info (err %d)\n", err);
	}
#ifdef CONFIG_BT_CTLR_PHY_CODED
	else {
		const struct bt_conn_le_phy_info *phy_info;

		phy_info = info.le.phy;
		printk("Connected: %s, tx_phy %u, rx_phy %u\n", addr, phy_info->tx_phy, phy_info->rx_phy);
	}
#endif 

	if (conn == default_conn) {
		err = bt_gatt_dm_start(conn, BT_UUID_HRS, &discover_hrs_cb, NULL);
		if (err) {
			printk("Failed to start discovery (err %d)\n", err);
		}
	}
	
	printk("Connected (%u): %s\n", conn_count, addr);

	err = bt_conn_set_security(default_conn, BT_SECURITY_L2);
	if (err) {
		printk("Failed to set security level (err %d)\n", err);
	}
	conn_count++;
	blinky_led(LED_GREEN, 1000);

	default_conn = NULL;
	/* Definir intervalo de comunicação (mín: 500ms, máx: 1s) */
	// // struct bt_le_conn_param *param = BT_LE_CONN_PARAM(0x190, 0x320, 0, 400);
	// param.interval_min = 1000;
	// param.interval_max = 1000;
	// param.latency = 10;
	// param.timeout = 3200; //10s
	// int ret = bt_conn_le_param_update(conn, &param);
	// int ret = bt_conn_le_param_update(conn, &info);
    // if (ret) {
    //     printk("Erro ao atualizar intervalo de conexao: %d\n", ret);
    // } else {
    //     printk("Solicitacao de atualizacao de conexao enviada!\n");
    // }
}

void on_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
    double connection_interval = interval*1.25;         // in ms
    uint16_t supervision_timeout = timeout*10;          // in ms
    printk("Connection parameters updated: interval %.2f ms, latency %d intervals, timeout %d ms\n", connection_interval, latency, supervision_timeout);
	blinky_led(LED_RED | LED_GREEN | LED_BLUE, 100);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s, reason 0x%02x %s\n", addr, reason, bt_hci_err_to_str(reason));

	bt_conn_unref(conn);

	if(conn_count > 0)	conn_count--;

	blinky_led(LED_RED, 1000);
}

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (err == BT_SECURITY_ERR_SUCCESS) {
        printk("Connection with %s: security changed to level %u\n", addr, level);
    } else {
        printk("Connection security failed with %s: error %d\n", addr, err);
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_updated = on_le_param_updated,
	.security_changed = security_changed,
};

/* Callback invoked when the peer device requires the user to enter a passkey */
static void auth_passkey_entry(struct bt_conn *conn)
{
    printk("Passkey entry requested by peer device.\n");

    /* In a real application, prompt the user to input the passkey.
     * For demonstration purposes, we'll use a fixed passkey: 123456
     */
    int err = bt_conn_auth_passkey_entry(conn, 123123);

    if (err) {
        printk("Failed to enter passkey (err %d)\n", err);
    }
}

/* Callback invoked when the peer device requests passkey confirmation */
static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
    printk("Passkey confirmation requested. Passkey: %06u\n", passkey);

    /* In a real application, display the passkey to the user and ask for confirmation.
     * If the user confirms the passkey matches, proceed with confirmation.
     */
    int err = bt_conn_auth_passkey_confirm(conn);
    if (err) {
        printk("Failed to confirm passkey (err %d)\n", err);
    }
}

/* Callback invoked when the pairing process is canceled */
static void auth_cancel(struct bt_conn *conn)
{
    printk("Pairing process canceled.\n");
}

/* Define the authentication callbacks structure */
static struct bt_conn_auth_cb auth_cb = {
    .passkey_entry = auth_passkey_entry,
    .passkey_confirm = auth_passkey_confirm,
    .cancel = auth_cancel,
};

static void scan_work_handler(struct k_work *work)
{
	uint8_t err;

	switch(bt_scan_status){
		case BT_STOPPED:{
				if (!bt_scan_enabled) {
					return;
				}else if (conn_count < CONFIG_BT_MAX_CONN) {
					err = bt_scan_start(BT_TEST_TYPE);
					if (err) {
						printk("Scanning failed to start (err %d)\n", err);
					}else{
						printk("Scanning successfully started\n");
						bt_scan_status = BT_SCANNING;
					}
				}else{
					blinky_led(LED_GREEN, 500);
				}
			}break;
		case BT_SCANNING:{
				if(conn_count >= CONFIG_BT_MAX_CONN || !bt_scan_enabled){
					err = bt_scan_stop();
					if (err) {
						printk("Scanning failed to stop (err %d)\n", err);
					}else{
						printk("Scanning successfully stopped\n");
						bt_scan_status = BT_STOPPED;
					}
				}

				blinky_led(LED_BLUE, 500);
			}break;
		default:
			printk("Unknown state\n");
			break;
	}

	k_work_reschedule(k_work_delayable_from_work(work), K_MSEC(1000));
}

int main(void){
	int err;

	gpio_pin_configure_dt(&ledred_spec, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
	gpio_pin_configure_dt(&ledgreen_spec, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
	gpio_pin_configure_dt(&ledblue_spec, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
	gpio_pin_configure_dt(&button0_spec, GPIO_INPUT);
	gpio_init_callback(&button0_cb_data, button_pressed_callback, BIT(button0_spec.pin));
	gpio_add_callback(button0_spec.port, &button0_cb_data);
	gpio_pin_interrupt_configure_dt(&button0_spec, GPIO_INT_EDGE_FALLING);

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	if(!gpio_pin_get(button0_spec.port, button0_spec.pin)){
		printk("Button 0 is pressed at start\n");
		err = bt_unpair(BT_ID_DEFAULT, NULL);
		if (err) {
			printk("Failed to unpair device: %d\n", err);
		}
	}
 
	err = bt_hrs_client_init(&hrs_c);
	if (err) {
		printk("Heart Rate Service client failed to init (err %d)\n", err);
		return 0;
	}

	scan_init();

	k_work_schedule(&scan_work, K_NO_WAIT);

	for (;;) {
		k_sleep(K_FOREVER);
	}
}
