//! T-Beam v1.1 LoRa node: AS923-1 923.2 MHz SF7 BW125, public sync 0x34.
//! Uplink IQ invert matches SX1302 SWAP_IQ default (LoRaWAN).

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <XPowersLib.h>

static const float FREQ_MHZ = 923.2f;
static const float BW_KHZ = 125.0f;
static const uint8_t SF = 7;
static const uint8_t CR = 5; // 4/5
static const uint8_t SYNC = 0x34;
static const int8_t PWR_DBM = 14;

// T-Beam v1.0/v1.1 SX1276
static const int PIN_LORA_SCK = 5;
static const int PIN_LORA_MISO = 19;
static const int PIN_LORA_MOSI = 27;
static const int PIN_LORA_CS = 18;
static const int PIN_LORA_RST = 23;
static const int PIN_LORA_DIO0 = 26;
static const int PIN_LORA_DIO1 = 33;
static const int PIN_SDA = 21;
static const int PIN_SCL = 22;

XPowersAXP192 pmu;
SX1276 radio = new Module(PIN_LORA_CS, PIN_LORA_DIO0, PIN_LORA_RST, PIN_LORA_DIO1);
uint32_t seq = 0;

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
    Serial.println("AXP192 LDO2 (LoRa) on");
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("tbeam-as923-node 923.2 SF7 BW125 sync=0x34");
    pmu_power_lora();
    SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_CS);
    int16_t st = radio.begin(FREQ_MHZ, BW_KHZ, SF, CR, SYNC, PWR_DBM, 8, 0);
    if (st != RADIOLIB_ERR_NONE) {
        Serial.printf("radio.begin failed %d\n", (int)st);
        while (true) delay(1000);
    }
    radio.setCRC(true);
    radio.invertIQ(false);
    Serial.println("radio ready, RX 8s then TX IQ=normal");
}

void loop() {
    uint8_t rxbuf[64];
    int16_t rst = radio.receive(rxbuf, sizeof rxbuf, 8000);
    if (rst == RADIOLIB_ERR_NONE) {
        size_t n = radio.getPacketLength();
        Serial.printf("rx n=%u rssi=%.1f snr=%.1f data=", (unsigned)n, radio.getRSSI(), radio.getSNR());
        for (size_t i = 0; i < n && i < sizeof rxbuf; i++) {
            Serial.printf("%02x", rxbuf[i]);
        }
        Serial.println();
    } else if (rst != RADIOLIB_ERR_RX_TIMEOUT) {
        Serial.printf("rx err %d\n", (int)rst);
    }

    uint8_t pkt[12];
    pkt[0] = 'T';
    pkt[1] = 'B';
    pkt[2] = 'E';
    pkt[3] = 'A';
    pkt[4] = 'M';
    pkt[5] = 0x01;
    pkt[6] = (uint8_t)(seq >> 24);
    pkt[7] = (uint8_t)(seq >> 16);
    pkt[8] = (uint8_t)(seq >> 8);
    pkt[9] = (uint8_t)seq;
    pkt[10] = 0xA5;
    pkt[11] = 0x23;
    int16_t st = radio.transmit(pkt, sizeof pkt);
    uint32_t t_tx_done = micros();
    Serial.printf("tx seq=%lu st=%d\n", (unsigned long)seq, (int)st);
    seq++;

    // Class A style RX1 check: gateway downlinks use inverted IQ (LoRaWAN ipol=true).
    // Report time from uplink end to downlink end so the gateway scheduler can be measured.
    radio.invertIQ(true);
    uint8_t dl[64];
    int16_t dst = radio.receive(dl, sizeof dl, 2500);
    uint32_t t_rx_done = micros();
    if (dst == RADIOLIB_ERR_NONE) {
        size_t n = radio.getPacketLength();
        float toa_ms = radio.getTimeOnAir(n) / 1000.0f;
        float dt_ms = (t_rx_done - t_tx_done) / 1000.0f;
        Serial.printf("dl n=%u rssi=%.1f snr=%.1f dt=%.2fms toa=%.2fms start=%.2fms data=",
                      (unsigned)n, radio.getRSSI(), radio.getSNR(), dt_ms, toa_ms, dt_ms - toa_ms);
        for (size_t i = 0; i < n && i < sizeof dl; i++) {
            Serial.printf("%02x", dl[i]);
        }
        Serial.println();
    } else if (dst == RADIOLIB_ERR_RX_TIMEOUT) {
        Serial.println("dl none (inverted IQ, 2.5s)");
    } else {
        Serial.printf("dl err %d\n", (int)dst);
    }
    radio.invertIQ(false);
}
