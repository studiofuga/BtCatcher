#include <Arduino.h>
#include <bluefruit.h>
#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier

#include "MacSet.h"

uint32_t currentTime = 0;
uint32_t currentTimeslot = 0;
bool ready = false;
bool started = false;
MacSet *macHead = nullptr;

void scan_callback(ble_gap_evt_adv_report_t* report);
void bt_connect_callback(uint16_t handle);
void bt_disconnect_callback(uint16_t conn_handle, uint8_t reason);

void startSearch()
{
    Bluefruit.Central.setDisconnectCallback(bt_disconnect_callback);
    Bluefruit.Central.setConnectCallback(bt_connect_callback);


    Bluefruit.Scanner.setRxCallback(scan_callback);
    Bluefruit.Scanner.restartOnDisconnect(false);
    Bluefruit.Scanner.filterRssi(-95);  // Only invoke callback when RSSI >= -80 dBm

    Bluefruit.Scanner.setInterval(160, 80);       // in units of 0.625 ms
    Bluefruit.Scanner.useActiveScan(false);        // Request scan response data
    Bluefruit.Scanner.start(0);                   // 0 = Don't stop scanning after n seconds
}

void print_report (ble_gap_evt_adv_report_t* report)
{
    /* Display the timestamp and device address */
    if (report->type.scan_response)
    {
        /* This is a Scan Response packet */
        Serial.printf("[SR%10d] Packet received from ", millis());
    }
    else
    {
        /* This is a normal advertising packet */
        Serial.printf("[ADV%9d] Packet received from ", millis());
    }
    Serial.printBuffer(report->peer_addr.addr, 6, ':');
    Serial.print("\n");

    /* Raw buffer contents */
    Serial.printf("%14s %d bytes\n", "PAYLOAD", report->data.len);
    if (report->data.len)
    {
        Serial.printf("%15s", " ");
        Serial.printBuffer(report->data.p_data, report->data.len, '-');
        Serial.println();
    }

    /* RSSI value */
    Serial.printf("%14s %d dBm\n", "RSSI", report->rssi);

    /* Adv Type */
    Serial.printf("%14s ", "ADV TYPE");

    if (report->type.connectable) {
        Serial.printf("Connectable ");
    } else {
        Serial.printf("Non-connectable ");
    }
    if (report->type.directed) {
        Serial.printf("Directed ");
    } else {
        Serial.printf("Undirected ");
    }
    if (report->type.scannable) {
        Serial.printf("Scannable ");
    }

    /* Check for BLE UART UUID */
    if ( Bluefruit.Scanner.checkReportForUuid(report, BLEUART_UUID_SERVICE) )
    {
        Serial.printf("%14s %s\n", "BLE UART", "UUID Found!");
    }

    /* Check for DIS UUID */
    if ( Bluefruit.Scanner.checkReportForUuid(report, UUID16_SVC_DEVICE_INFORMATION) )
    {
        Serial.printf("%14s %s\n", "DIS", "UUID Found!");
    }

    Serial.println();
}

void bt_connect_callback(uint16_t handle)
{
    Serial.printf("Connected Handler 0x%04x\n", handle);

    // Get the reference to current connection
    BLEConnection* connection = Bluefruit.Connection(handle);

    char peer_name[32] = { 0 };
    connection->getPeerName(peer_name, sizeof(peer_name));

    Serial.print("[Cent] Connected to ");
    Serial.println(peer_name);

    /*
    if (timesync_check(handle)){
        Bluefruit.Scanner.stop();
        ready = true;
    } else {
        Serial.println("Scanning...");
        Bluefruit.Scanner.resume();
    }*/

    Bluefruit.Scanner.resume();
}

void bt_disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
    (void) conn_handle;
    (void) reason;

    Serial.println("Disconnected");
}

void scan_callback(ble_gap_evt_adv_report_t* report)
{
    MacSet *mac = macHead;
    MacSet *prevmac = nullptr;

    while (mac != nullptr) {
        auto eq = memcmp (mac->mac,report->peer_addr.addr,6);
        if (eq == 0) {
            Bluefruit.Scanner.resume();
            return;
        }
        prevmac = mac;
        mac = mac->next;
    }

    static uint8_t bf[6];
    for (int i = 0; i < 6; ++i) {
        bf[5-i] = report->peer_addr.addr[i];
    }
    Serial.printBuffer(&bf[0], 6, ':');

    mac = new MacSet;
    memcpy(mac->mac, report->peer_addr.addr,6);
    if (prevmac != nullptr)
        prevmac->next = mac;
    else
        macHead = mac;

    Serial.printf(" - (%d) ", report->data.len);
    for (int i = 0; i < report->data.len; ++i) {
        Serial.printf("%02x ", report->data.p_data[i]); // 9+i
    }

    /*
    for (int i = 0; i < report->data.len; ++i) {
        //Serial.printf("\n%d %02x, %d\n", i, report->data.p_data[i+1], report->data.p_data[i]);
        if (report->data.p_data[i+1] == time_signature[1]) {
            //Serial.printf("* %d\n", i);
            if (memcmp(report->data.p_data + i, time_signature, SIGNATURE_SIZE )== 0) {
                Serial.println(" TimeBeacon found");
                Bluefruit.Scanner.stop();
                timesync_inquiry(report);
                return;
            }
        } 
        i += report->data.p_data[i];
    }
    */

    Serial.println();
    Bluefruit.Scanner.resume();
}

void setup()
{
    Wire.begin();

    Serial.begin(115200);
    while ( !Serial ) delay(10);

    Bluefruit.begin(1, 1);

    // off Blue LED for lowest power consumption
    Bluefruit.autoConnLed(false);

    pinMode(LED_RED, OUTPUT);
    digitalWrite(LED_RED, LOW);
    pinMode(LED_BLUE, OUTPUT);
    digitalWrite(LED_BLUE, LOW);

    // See bluefruit.h for the accepted values. Tx Values are taken from support.kontact.io
#if defined(NRF52832_XXAA)
    int8_t const TxValues[] = { 115, 84, 81, 77, 72, 69, 68, 65, 59 };
    int8_t const Accepted[] = { -40, -20, -16, -12, -8, -4, 0, 3, 4 };
#elif defined( NRF52840_XXAA)
    int8_t const TxValues[] = { 115, 84, 81, 77, 72, 69, 65, 62, 59, 56, 53, 50, 47, 44 };
    int8_t const Accepted[] = { -40, -20, -16, -12, -8, -4, 0, 2, 3, 4, 5, 6, 7, 8 };
#endif

    Bluefruit.setTxPower(-8);    // Check bluefruit.h for supported values
    Bluefruit.setName("ZZZ");

    uint8_t mac[6];
    uint8_t  tp = Bluefruit.getAddr(mac);

    delay(2000);
    Serial.printf("Setting up... (%02x): ", tp);
    Serial.printBuffer(mac, 6, ':');
    Serial.println();
    delay(2000);

    startSearch();
}

void loop()
{
    digitalWrite(LED_BLUE, HIGH);
    delay (250);
    digitalWrite(LED_BLUE, LOW);
    delay (750);
    delay(4 * 1000);
}
