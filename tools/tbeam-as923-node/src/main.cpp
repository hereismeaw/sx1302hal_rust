//! T-Beam v1.1 gateway test node (SX1276), AS923-1, public sync 0x34.
//!
//! Cycles through a list of uplink configurations covering both SX1250 radios,
//! SF7–SF12, the LoRa service chain (BW250) and the FSK chain. After each uplink
//! it listens for up to two downlinks (tools/gwmp_echo_ns.py answers at
//! tmst+1 s and tmst+2 s) and prints `start=` = uplink end -> downlink start.

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <XPowersLib.h>

static const int PIN_LORA_SCK = 5;
static const int PIN_LORA_MISO = 19;
static const int PIN_LORA_MOSI = 27;
static const int PIN_LORA_CS = 18;
static const int PIN_LORA_RST = 23;
static const int PIN_LORA_DIO0 = 26;
static const int PIN_LORA_DIO1 = 33;
static const int PIN_SDA = 21;
static const int PIN_SCL = 22;
static const int8_t PWR_DBM = 14;

struct TestCase {
    bool fsk;
    float freq_mhz;
    uint8_t sf;
    float bw_khz;
};

static const TestCase CASES[] = {
    {false, 922.6f, 7, 125.0f},   // radio A
    {false, 923.4f, 7, 125.0f},   // radio B
    {false, 924.0f, 9, 125.0f},   // radio B edge
    {false, 923.2f, 10, 125.0f},  // RX2 data rate
    {false, 922.8f, 11, 125.0f},  // LDRO
    {false, 923.6f, 12, 125.0f},  // LDRO, JIT collision of 2nd downlink
    {false, 924.3f, 7, 250.0f},   // LoRa service chain
    {true, 922.3f, 0, 0.0f},      // FSK 50 kbps
};
static const size_t N_CASES = sizeof(CASES) / sizeof(CASES[0]);

XPowersAXP192 pmu;
SX1276 radio = new Module(PIN_LORA_CS, PIN_LORA_DIO0, PIN_LORA_RST, PIN_LORA_DIO1);
uint16_t seq = 0;
size_t case_idx = 0;

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

static int16_t configure(const TestCase &c) {
    if (c.fsk) {
        // Semtech SX1302 FSK: 50 kbps, fdev 25 kHz, sync C1 94 C1, CCITT CRC,
        // whitening, variable length, Gaussian BT 0.5.
        int16_t st = radio.beginFSK(c.freq_mhz, 50.0f, 25.0f, 125.0f, PWR_DBM, 40, false);
        if (st != RADIOLIB_ERR_NONE) return st;
        uint8_t sync[] = {0xC1, 0x94, 0xC1};
        radio.setSyncWord(sync, sizeof sync);
        radio.setCRC(true, false);
        radio.setEncoding(RADIOLIB_ENCODING_WHITENING);
        radio.setDataShaping(RADIOLIB_SHAPING_0_5);
        radio.variablePacketLengthMode(255);
        return RADIOLIB_ERR_NONE;
    }
    int16_t st = radio.begin(c.freq_mhz, c.bw_khz, c.sf, 5, 0x34, PWR_DBM, 8, 0);
    if (st != RADIOLIB_ERR_NONE) return st;
    radio.setCRC(true);
    return RADIOLIB_ERR_NONE;
}

static void describe(const TestCase &c) {
    if (c.fsk) {
        Serial.printf("FSK %.1f MHz 50k", c.freq_mhz);
    } else {
        Serial.printf("%.1f MHz SF%u BW%.0f", c.freq_mhz, c.sf, c.bw_khz);
    }
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("tbeam-gw-test AS923-1: radio A/B, SF7-12, BW250 service, FSK");
    pmu_power_lora();
    SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_CS);
}

void loop() {
    const TestCase &c = CASES[case_idx];
    int16_t st = configure(c);
    if (st != RADIOLIB_ERR_NONE) {
        Serial.printf("case %u configure failed %d\n", (unsigned)case_idx, (int)st);
        delay(2000);
        case_idx = (case_idx + 1) % N_CASES;
        return;
    }
    if (!c.fsk) radio.invertIQ(false);

    uint8_t pkt[6] = {'T', 'B', (uint8_t)case_idx, (uint8_t)(seq >> 8), (uint8_t)seq, 0xA5};
    st = radio.transmit(pkt, sizeof pkt);
    uint32_t t_tx_done = micros();
    Serial.printf("case %u ", (unsigned)case_idx);
    describe(c);
    Serial.printf(" up seq=%u st=%d\n", (unsigned)seq, (int)st);

    // Downlinks: LoRaWAN-style inverted IQ for LoRa, plain for FSK.
    if (!c.fsk) radio.invertIQ(true);
    for (int n = 0; n < 2; n++) {
        uint8_t dl[64];
        uint32_t remain_us = 3500000 - (micros() - t_tx_done);
        if ((int32_t)remain_us <= 0) break;
        int16_t dst = radio.receive(dl, sizeof dl, remain_us / 1000);
        uint32_t t_rx_done = micros();
        if (dst == RADIOLIB_ERR_NONE) {
            size_t len = radio.getPacketLength();
            float toa_ms = radio.getTimeOnAir(len) / 1000.0f;
            float dt_ms = (t_rx_done - t_tx_done) / 1000.0f;
            Serial.printf("  dl%d n=%u rssi=%.1f dt=%.2fms toa=%.2fms start=%.2fms data=", n + 1,
                          (unsigned)len, radio.getRSSI(), dt_ms, toa_ms, dt_ms - toa_ms);
            for (size_t i = 0; i < len && i < sizeof dl; i++) Serial.printf("%02x", dl[i]);
            Serial.println();
        } else if (dst == RADIOLIB_ERR_RX_TIMEOUT) {
            Serial.printf("  dl%d none\n", n + 1);
            break;
        } else {
            Serial.printf("  dl%d err %d\n", n + 1, (int)dst);
        }
    }
    seq++;
    case_idx = (case_idx + 1) % N_CASES;
    delay(300);
}
