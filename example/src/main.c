#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ble_to_mqtt, LOG_LEVEL_DBG);
#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/random/rand32.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

//Config for the example
#define ZEPHYR_ADDR				"192.168.10.100"
#define SERVER_ADDR				"192.168.10.12"
#define SERVER_PORT				1883
#define APP_CONNECT_TIMEOUT_MS	5000
#define APP_SLEEP_MSECS			500
#define APP_CONNECT_TRIES		10
#define APP_MQTT_BUFFER_SIZE	128
#define MQTT_CLIENTID			"repeat"


#define APP_BMEM
#define APP_DMEM

//Topic prefix string
const char * topic_prefix = "repeat/ble/";

//Defines for the FEM GPIO
#define PDN_NODE DT_NODELABEL(pdn_gpio)
static const struct gpio_dt_spec pdn_pin = GPIO_DT_SPEC_GET(PDN_NODE, gpios);

#define RXEN_NODE DT_NODELABEL(rxen_gpio)
static const struct gpio_dt_spec rxen_pin = GPIO_DT_SPEC_GET(RXEN_NODE, gpios);

#define TXEN_NODE DT_NODELABEL(txen_gpio)
static const struct gpio_dt_spec txen_pin = GPIO_DT_SPEC_GET(TXEN_NODE, gpios);

#define ANTSEL_NODE DT_NODELABEL(antsel_gpio)
static const struct gpio_dt_spec antsel_pin = GPIO_DT_SPEC_GET(ANTSEL_NODE, gpios);

#define MODE_NODE DT_NODELABEL(mode_gpio)
static const struct gpio_dt_spec mode_pin = GPIO_DT_SPEC_GET(MODE_NODE, gpios);

//BLE packet structure 
typedef struct {
	char mac[18];
	int8_t rssi;
	char buffer[66];
} ble_packet_t;

//Queue for BLE packets
char __aligned(1) bleq_buffer[50 * sizeof(ble_packet_t)];
struct k_msgq bleq;

//Buffers for MQTT client
static APP_BMEM uint8_t rx_buffer[APP_MQTT_BUFFER_SIZE];
static APP_BMEM uint8_t tx_buffer[APP_MQTT_BUFFER_SIZE];

//MQTT client struct 
static APP_BMEM struct mqtt_client client_ctx;

//MQTT Broker details. 
static APP_BMEM struct sockaddr_in broker;
static APP_BMEM struct zsock_pollfd fds[1];
static APP_BMEM int nfds;
static APP_BMEM bool connected;

//Basic wait function
static int wait(int timeout)
{
	int ret = 0;
	if (nfds > 0) {
		ret = zsock_poll(fds, nfds, timeout);
		if (ret < 0) {
			LOG_ERR("poll error: %d", errno);
		}
	}

	return ret;
}

//Event handler for MQTT events
void mqtt_evt_handler(struct mqtt_client *const client,
		      const struct mqtt_evt *evt)
{
	int err;

	switch (evt->type) {
	case MQTT_EVT_CONNACK:
		if (evt->result != 0) {
			LOG_ERR("MQTT connect failed %d", evt->result);
			break;
		}

		connected = true;
		LOG_INF("MQTT client connected!");

		break;

	case MQTT_EVT_DISCONNECT:
		LOG_INF("MQTT client disconnected %d", evt->result);

		connected = false;
		nfds = 0;

		break;

	case MQTT_EVT_PUBACK:
		if (evt->result != 0) {
			LOG_ERR("MQTT PUBACK error %d", evt->result);
			break;
		}

		LOG_INF("PUBACK packet id: %u", evt->param.puback.message_id);

		break;

	case MQTT_EVT_PUBREC:
		if (evt->result != 0) {
			LOG_ERR("MQTT PUBREC error %d", evt->result);
			break;
		}

		LOG_INF("PUBREC packet id: %u", evt->param.pubrec.message_id);

		const struct mqtt_pubrel_param rel_param = {
			.message_id = evt->param.pubrec.message_id
		};

		err = mqtt_publish_qos2_release(client, &rel_param);
		if (err != 0) {
			LOG_ERR("Failed to send MQTT PUBREL: %d", err);
		}

		break;

	case MQTT_EVT_PUBCOMP:
		if (evt->result != 0) {
			LOG_ERR("MQTT PUBCOMP error %d", evt->result);
			break;
		}

		LOG_INF("PUBCOMP packet id: %u",
			evt->param.pubcomp.message_id);

		break;

	case MQTT_EVT_PINGRESP:
		LOG_INF("PINGRESP packet");
		break;

	default:
		break;
	}
}


static int publish(struct mqtt_client *client, enum mqtt_qos qos, char * top, char * payl)
{
	struct mqtt_publish_param param;
	//Assemble the MQTT message from provided parameters
	param.message.topic.qos = qos;
	param.message.topic.topic.utf8 = (uint8_t *)top;
	param.message.topic.topic.size =
			strlen(param.message.topic.topic.utf8);
	param.message.payload.data = payl;
	param.message.payload.len =
			strlen(param.message.payload.data);
	param.message_id = sys_rand32_get();
	param.dup_flag = 0U;
	param.retain_flag = 0U;

	//Publish it
	return mqtt_publish(client, &param);
}

#define RC_STR(ret) ((ret) == 0 ? "OK" : "ERROR")

#define PRINT_RESULT(func, ret) \
	LOG_INF("%s: %d <%s>", (func), ret, RC_STR(ret))

/* In this routine we block until the connected variable is 1 */
static int mqtt_try_to_connect(struct mqtt_client *client)
{
	int ret, i = 0;

	while (i++ < APP_CONNECT_TRIES && !connected) {
		//Initialize client
		mqtt_client_init(client);

		//Set up the broker IP and port
		broker.sin_family = AF_INET;
		broker.sin_port = htons(SERVER_PORT);
		zsock_inet_pton(AF_INET, SERVER_ADDR, &broker.sin_addr);

		//MQTT client configuration
		client->broker = &broker;
		client->evt_cb = mqtt_evt_handler;
		client->client_id.utf8 = (uint8_t *)MQTT_CLIENTID;
		client->client_id.size = strlen(MQTT_CLIENTID);

		//Setup password and username credentials needed to connect to the broker
		static struct mqtt_utf8 password;
		static struct mqtt_utf8 user_name;
		password.utf8 = (uint8_t*)CONFIG_MQTT_BROKER_PASSWORD;
		password.size = strlen(CONFIG_MQTT_BROKER_PASSWORD);
		user_name.utf8 = (uint8_t*)CONFIG_MQTT_BROKER_USERNAME;
		user_name.size = strlen(CONFIG_MQTT_BROKER_USERNAME);

		//Assign the credentials to the client
		client->password = &password;
		client->user_name = &user_name;
		//Setup MQTT version
		client->protocol_version = MQTT_VERSION_3_1_1;

		//Configure buffers
		client->rx_buf = rx_buffer;
		client->rx_buf_size = sizeof(rx_buffer);
		client->tx_buf = tx_buffer;
		client->tx_buf_size = sizeof(tx_buffer);

		//Configure transport type
		client->transport.type = MQTT_TRANSPORT_NON_SECURE;


		//Try to connect to the broker
		ret = mqtt_connect(client);
		if (ret != 0) {
			PRINT_RESULT("mqtt_connect", ret);
			k_sleep(K_MSEC(APP_SLEEP_MSECS));
			continue;
		}

		//Set the transport type for the client
		if (client->transport.type == MQTT_TRANSPORT_NON_SECURE) {
			fds[0].fd = client->transport.tcp.sock;
		}
		fds[0].events = ZSOCK_POLLIN;
		nfds = 1;

		//Wait for the feedback from the broker
		if (wait(APP_CONNECT_TIMEOUT_MS)) {
			mqtt_input(client);
		}

		//If the connection was unsuccesful, abort
		if (!connected) {
			mqtt_abort(client);
		}
	}

	//If the connection was succesful, return 0
	if (connected) {
		return 0;
	}

	return -EINVAL;
}


static int fem_init(bool rx, bool tx, bool powerdown, bool antenna, bool mode)
{
	//Setup all the GPIO needed to control the radio FEM onboard and set it
	int ret = -1;

	if (!device_is_ready(pdn_pin.port)) {
		return ret;
	}

	ret = gpio_pin_configure_dt(&pdn_pin, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return ret;
	}

	ret = gpio_pin_set_dt(&pdn_pin, !powerdown);

	if (!device_is_ready(antsel_pin.port)) {
		return ret;
	}

	ret = gpio_pin_configure_dt(&antsel_pin, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return ret;
	}

	ret = gpio_pin_set_dt(&antsel_pin, antenna);

	if (!device_is_ready(rxen_pin.port)) {
		return ret;
	}

	ret = gpio_pin_configure_dt(&rxen_pin, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return ret;
	}

	ret = gpio_pin_set_dt(&rxen_pin, tx);

	if (!device_is_ready(txen_pin.port)) {
		return ret;
	}

	ret = gpio_pin_configure_dt(&txen_pin, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return ret;
	}

	ret = gpio_pin_set_dt(&txen_pin, tx);

	if (!device_is_ready(mode_pin.port)) {
		return ret;
	}

	ret = gpio_pin_configure_dt(&mode_pin, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return ret;
	}

	ret = gpio_pin_set_dt(&mode_pin, rx);

	return ret;

}


static void ble_scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type, struct net_buf_simple *buf)
{

	//Assemble the packet object from the incomming data
	ble_packet_t received;
	//Create stringified MAC
	sprintf(received.mac, "%02X:%02X:%02X:%02X:%02X:%02X", addr->a.val[5], addr->a.val[4], addr->a.val[3], addr->a.val[2], addr->a.val[1], addr->a.val[0]);
	//Get RSSI of the incoming packet
	received.rssi = rssi;

	char byteBuff[3];

	//Fill the data buffer in human readable format
	uint8_t i;
	for(i = 0; i < buf->len; i++){
		if(i == 32) break;
		sprintf(byteBuff, "%02X", buf->data[i]);
		received.buffer[2*i] = byteBuff[0];
		received.buffer[2*i+1] = byteBuff[1];
	}

	//Terminate with null
	received.buffer[2*i] = 0;

	
	
	//Push the object into queue
	while (k_msgq_put(&bleq, &received, K_NO_WAIT) != 0) {
        //Message queue is full: purge old data & try again
        k_msgq_purge(&bleq);
    }
}



static int ble_init(){
	//Define scan parameters
	struct bt_le_scan_param scan_param = {
		.type       = BT_HCI_LE_SCAN_PASSIVE,
		.options    = BT_LE_SCAN_OPT_NONE,
		.interval   = 0x00A0,
		.window     = 0x00A0,
	};

	int err;

	//Enable bluetooth
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return -1;
	}

	printk("Bluetooth initialized\n");

	//Start the scan with predefined parameters
	err = bt_le_scan_start(&scan_param, ble_scan_cb);
	if (err) {
		printk("Starting scanning failed (err %d)\n", err);
		return -1;
	}

	return err;
}


void main(void)
{
	int ret;

	//Init the message queue between ble scan thread and mqtt publish thread
	k_msgq_init(&bleq, bleq_buffer, sizeof(ble_packet_t), 50);

	//Init FEM for RX mode
	ret = fem_init(1,0,0,1,0);
	if(ret < 0){
		printk("Unable to setup FEM (err %d)\n", ret);
		exit(-1);
	}


	//Connect to the broker
	ret = mqtt_try_to_connect(&client_ctx);
	if(ret < 0){
		printk("Unable to connect to the broker (err %d)\n", ret);
		exit(-1);
	}

	//Init BLE
	ret = ble_init();
	if(ret < 0){
		printk("Failed to initialize BLE (err %d)\n", ret);
		exit(-1);
	}

	//Create topic and payload buffers
	static APP_BMEM char topic[100];
	static APP_BMEM char payload[160];

	//Create data object to which will the data from the queue be retreived
	ble_packet_t data;

	//Check for new data in the queue and if available, send them to the broker
	while (1) {
		//Get new data from the queue if available, if not, wait for them
		k_msgq_get(&bleq, &data, K_FOREVER);

		//Create a JSON payload
		sprintf(payload, "{ \"mac\": \"%s\", \"rssi\": \"%2d\", \"raw\": \"%s\"}", data.mac, data.rssi, data.buffer);
		//Create the topic from device mac adress and prefix
		sprintf(topic, "%s%s", topic_prefix, data.mac);

		int rc;
		//Try to publish to the broker
		rc = publish(&client_ctx, MQTT_QOS_0_AT_MOST_ONCE, topic, payload);
		//Log what you published and the error code
		printk("Publish message %s to topic %s finished with code %d\n", payload, topic, rc);

		//Sleep a little
		k_sleep(K_MSEC(100));
	}

}

