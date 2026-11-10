//! T-Beam v1.1 OTAA node, AS923-1, for the sx1302-hal-rs gateway.
//! RadioLib treats equal NwkKey and AppKey as LoRaWAN 1.0.x, and different keys as 1.1
//! (NwkKey + AppKey, four session keys). DevNonce persisted in NVS.
//! shortcut: RadioLib 7.1 has no LoRaWAN 1.2 session. Keep forwarding on the gateway.

#include <Arduino.h>
#include <Preferences.h>
#include <RadioLib.h>
#include <SPI.h>
#include <Wire.h>
#include <XPowersLib.h>

#include "keys.h"

static const int PIN_LORA_SCK = 5;
static const int PIN_LORA_MISO = 19;
static const int PIN_LORA_MOSI = 27;
static const int PIN_LORA_CS = 18;
static const int PIN_LORA_RST = 23;
static const int PIN_LORA_DIO0 = 26;
static const int PIN_LORA_DIO1 = 33;
static const int PIN_SDA = 21;
static const int PIN_SCL = 22;
static const uint32_t UPLINK_INTERVAL_MS = 30000;

XPowersAXP192 pmu;
SX1276 radio = new Module(PIN_LORA_CS, PIN_LORA_DIO0, PIN_LORA_RST, PIN_LORA_DIO1);
LoRaWANNode node(&radio, &AS923);
Preferences prefs;

static uint8_t app_key[16] = OTAA_APP_KEY;
// Define OTAA_NWK_KEY in keys.h for LoRaWAN 1.1. Omit it to stay on 1.0.x.
#ifdef OTAA_NWK_KEY
static uint8_t nwk_key[16] = OTAA_NWK_KEY;
#else
static uint8_t nwk_key[16] = OTAA_APP_KEY;
#endif
static uint32_t counter = 0;

static void pmu_power_lora() {
    Wire.begin(PIN_SDA, PIN_SCL);
    if (!pmu.begin(Wire, AXP192_SLAVE_ADDRESS, PIN_SDA, PIN_SCL)) {
        Serial.println("AXP192 not found; trying radio anyway");
        return;
    }
    pmu.setProtectedChannel(XPOWERS_DCDC1);
    pmu.setLDO2Voltage(3300);
    pmu.enableLDO2();
    pmu.enableLDO3();
}

static void save_nonces() {
    prefs.putBytes("nonces", node.getBufferNonces(), RADIOLIB_LORAWAN_NONCES_BUF_SIZE);
}

static void join() {
    while (true) {
        int16_t st = node.activateOTAA();
        save_nonces();
        if (st == RADIOLIB_LORAWAN_NEW_SESSION) {
            Serial.printf("join OK devaddr=%08lx\n", (unsigned long)node.getDevAddr());
            return;
        }
        Serial.printf("join failed %d, retry in 15 s\n", (int)st);
        delay(15000);
    }
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.printf("tbeam-otaa-node AS923-1 deveui=%016llx\n", (unsigned long long)OTAA_DEV_EUI);
    pmu_power_lora();
    SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_CS);
    int16_t st = radio.begin();
    if (st != RADIOLIB_ERR_NONE) {
        Serial.printf("radio.begin failed %d\n", (int)st);
        while (true) delay(1000);
    }
    st = node.beginOTAA(OTAA_JOIN_EUI, OTAA_DEV_EUI, nwk_key, app_key);
    if (st != RADIOLIB_ERR_NONE) {
        Serial.printf("beginOTAA failed %d\n", (int)st);
        while (true) delay(1000);
    }
    prefs.begin("otaa", false);
    uint8_t buf[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];
    if (prefs.getBytes("nonces", buf, sizeof buf) == sizeof buf) {
        st = node.setBufferNonces(buf);
        Serial.printf("restored nonces (%d)\n", (int)st);
    }
    join();
}

void loop() {
    uint8_t up[6] = {'R', 'S', (uint8_t)(counter >> 24), (uint8_t)(counter >> 16),
                     (uint8_t)(counter >> 8), (uint8_t)counter};
    uint8_t down[64];
    size_t down_len = 0;
    LoRaWANEvent_t ev_up, ev_down;
    int16_t st = node.sendReceive(up, sizeof up, 1, down, &down_len, true, &ev_up, &ev_down);
    if (st < RADIOLIB_ERR_NONE) {
        Serial.printf("uplink %lu err %d\n", (unsigned long)counter, (int)st);
    } else {
        Serial.printf("uplink fcnt=%lu %.1f MHz -> ", (unsigned long)ev_up.fCnt, ev_up.freq);
        if (st == 0) {
            Serial.println("no downlink");
        } else {
            Serial.printf("RX%d ack=%d fport=%u len=%u rssi=%.1f snr=%.1f\n", (int)st,
                          (int)ev_down.confirming, (unsigned)ev_down.fPort, (unsigned)down_len,
                          radio.getRSSI(), radio.getSNR());
        }
    }
    counter++;
    delay(UPLINK_INTERVAL_MS);
}
