
#ifndef UNIT_TEST

#include "config.h"
#include "debug.h"

#ifdef MQTT_ENABLED

#include "uext/uext_mqtt.h"

#include "load.h"
#include "thingset.h"
#include <inttypes.h>

#include "pcb.h"

UARTSerial uext_serial(PIN_UEXT_TX, PIN_UEXT_RX, 115200);

// ESP32 wifi(uext_serial);
//DigitalOut wifi_enable(PIN_UEXT_SCL);
//DigitalOut wifi_enable(PIN_UEXT_SSEL);

enum WifiState {
    STATE_WIFI_RESET,       // state before reset
    STATE_WIFI_INIT,        // initial state
    STATE_WIFI_CONN,        // successfully connected to WiFi AP
    STATE_LAN_CONN,         // local IP address obtained
    STATE_INTERNET_CONN,    // internet connection active
    STATE_WIFI_IDLE,        // switch off WiFi to save energy
    STATE_WIFI_ERROR
};

extern LoadOutput load;

extern ThingSet ts;
extern const int PUB_CHANNEL_CAN;

static    char json[400];

int wifi_send_json_data()
{
    // Write the ThingSet publication message to URL end - 2 bytes and afterwards
    // overwrite two publication message start bytes "# " with correct URL
    // content again to avoid strcpy
    int ts_len = ts.pub_msg_json(json, sizeof(json) - 1, PUB_CHANNEL_CAN);
    json[ts_len] = '\n';  // ThingSet does not null-terminate
    int res = 0;
    if (ts_len > 2)
    {
        res = uext_serial.write(json+2,ts_len+1-2);
    }

    print_info("JSON SEND DATA: %s\n", (res == ts_len) ? "OK" : "ERROR");
    return 0;
}


static UExtMqtt uext_mqtt; // local instance, will self register itself

UExtMqtt::UExtMqtt() {}

void UExtMqtt::enable() {
#ifdef PIN_UEXT_DIS
    DigitalOut uext_dis(PIN_UEXT_DIS);
    uext_dis = 0;
#endif
}

int wifi_reset()
{
    return 1;
}

int wifi_connect_ap()
{
    return 1;
}

int wifi_check_lan_connected() {
    return 1;
}

int wifi_setup_internet_conn() {
    return 1;
}

int wifi_check_internet_connected() {
    return 1;
}

void UExtMqtt::process_asap(void) {}

void UExtMqtt::process_1s() {
    static WifiState state = STATE_WIFI_RESET;

    if (load.usb_state != LOAD_STATE_ON) {
        state = STATE_WIFI_RESET;
    } else
    {
        state = STATE_WIFI_RESET;
    }
    if (state != STATE_WIFI_IDLE) {
        if (uext_serial.readable()) {
    
            // add code to handle input...
        }
        if (time(NULL) % 10 == 0) {
            wifi_send_json_data();
        }
    }
}



#endif /* MQTT_ENABLED */

#endif /* UNIT_TEST */
