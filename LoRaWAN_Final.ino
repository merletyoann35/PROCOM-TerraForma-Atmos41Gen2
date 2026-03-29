#include <Arduino.h>
#include <LoRaWan-Arduino.h>
#include <SPI.h>
#include "RAK13010_SDI12.h"

// PIN DEFINITIONS & CONSTANTS
// SDI-12 Pins (Slot A default)
#define TX_PIN   WB_IO6
#define RX_PIN   WB_IO5
#define OE       WB_IO4
#define POWER_PIN WB_IO2
#define SENSOR_ADDRESS '0'

// LoRaWAN Config
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE
#define SCHED_QUEUE_SIZE 60
#define LORAWAN_APP_DATA_BUFF_SIZE 64
#define LORAWAN_APP_TX_DUTYCYCLE   300000   // 30000=30 sec - 300000=5min
#define JOINREQ_NBTRIALS 3

// GLOBAL OBJECTS & VARIABLES
RAK_SDI12 mySDI12(RX_PIN, TX_PIN, OE);

// LoRaWAN Objects
hw_config hwConfig;
TimerEvent_t appTimer;
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0};

// State Machine Flags
volatile bool startMeasurementCycle = false; // Set by Timer, read by Loop
bool sdiMsgReady = false;
String sdiMsgStr = "";

// Data Container to bridge Sensor -> LoRa
struct WeatherData {
    float rain_mm = 0.0;
    float wind_dir_deg = 0.0;
    float wind_speed_ms = 0.0;
    float temp_c = 0.0;
    float humidity_pct = 0.0;
    float pressure_kpa = 0.0;
} currentMeasure;

// LORAWAN KEYS (OTAA)
uint8_t nodeDeviceEUI[8] = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x07, 0x45, 0xE6};
uint8_t nodeAppEUI[8]    = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t nodeAppKey[16]   = {0x14, 0x31, 0x4F, 0xD7, 0x96, 0x11, 0xBB, 0x5F, 0xB5, 0xD5, 0x01, 0xAE, 0xF1, 0xEF, 0x62, 0xEF};

uint32_t nodeDevAddr = 0x260116F8;
uint8_t nodeNwsKey[16] = {0};
uint8_t nodeAppsKey[16] = {0};

// FORWARD DECLARATIONS
static void lorawan_has_joined_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void lorawan_join_failed_handler(void);
static void tx_lora_periodic_handler(void);
static void send_lora_frame(void);
void decodeATMOS41(String msg);

static lmh_callback_t lora_callbacks = {
  BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
  lorawan_rx_handler, lorawan_has_joined_handler,
  lorawan_confirm_class_handler, lorawan_join_failed_handler
};

static lmh_param_t lora_param_init = {
  LORAWAN_ADR_ON, LORAWAN_DEFAULT_DATARATE, LORAWAN_PUBLIC_NETWORK,
  JOINREQ_NBTRIALS, LORAWAN_DEFAULT_TX_POWER, LORAWAN_DUTYCYCLE_OFF
};

// HELPER FUNCTIONS
static void put_u16_be(uint8_t *buf, uint8_t &i, uint16_t v) {
  buf[i++] = (uint8_t)(v >> 8);
  buf[i++] = (uint8_t)(v & 0xFF);
}
static void put_i16_be(uint8_t *buf, uint8_t &i, int16_t v) {
  uint16_t uv = (uint16_t)v;
  buf[i++] = (uint8_t)(uv >> 8);
  buf[i++] = (uint8_t)(uv & 0xFF);
}

// SETUP
void setup() {
    // Init Serial
    Serial.begin(115200);
    time_t timeout = millis();
    while (!Serial && (millis() - timeout) < 5000) delay(100);
    Serial.println("=====================================");
    Serial.println("   METEO STATION: ATMOS 41 + LORA    ");
    Serial.println("=====================================");

    // Init SDI-12 Hardware
    pinMode(POWER_PIN, OUTPUT);
    digitalWrite(POWER_PIN, HIGH); // Turn on SDI-12 power
    
    Serial.println("[SDI12] Opening bus...");
    mySDI12.begin();
    delay(500);
    mySDI12.forceListen();
    sdiMsgStr.reserve(200);

    // Init LoRa Hardware
#ifdef RAK4630
    lora_rak4630_init();
#else
    lora_hardware_init(hwConfig);
#endif

    // Init LoRaWAN Stack
    // Timer setup
    appTimer.timerNum = 3;
    TimerInit(&appTimer, tx_lora_periodic_handler);

    lmh_setDevEui(nodeDeviceEUI);
    lmh_setAppEui(nodeAppEUI);
    lmh_setAppKey(nodeAppKey);
    lmh_setNwkSKey(nodeNwsKey);
    lmh_setAppSKey(nodeAppsKey);
    lmh_setDevAddr(nodeDevAddr);

    lmh_init(&lora_callbacks, lora_param_init, true, CLASS_A, LORAMAC_REGION_EU868);
    
    Serial.println("[LORA] Sending Join Request...");
    lmh_join();
}

// MAIN LOOP (The Coordinator)
void loop() {
    // Trigger Measurement (From Timer Flag)
    if (startMeasurementCycle) {
        startMeasurementCycle = false; // Reset flag

        Serial.println("\n--- [CYCLE START] ---");
        
        // Clear old buffers
        sdiMsgStr = "";
        sdiMsgReady = false;
        while(mySDI12.available()) mySDI12.read();

        // Send Command
        String cmd = String(SENSOR_ADDRESS) + "R0!";
        mySDI12.sendCommand(cmd);
        Serial.println("[SDI12] Transmitted: " + cmd);
        
        // Note: We do NOT delay here. We let the loop continue to picking up characters in the next block.
    }

    // Read Sensor Response (Non-blocking)
    int avail = mySDI12.available();
    if (avail > 0) {
        for (int i = 0; i < avail; i++) {
            char c = mySDI12.read();
            if (c == '\n') {
                sdiMsgReady = true;
            } else if (c != '\r') {
                sdiMsgStr += String(c);
            }
        }
    }

    // Decode & Send LoRa
    if (sdiMsgReady) {
        Serial.println("[SDI12] Raw: " + sdiMsgStr);
        
        // Parse string into 'currentMeasure' struct
        decodeATMOS41(sdiMsgStr);

        // Send the packet via LoRa
        send_lora_frame();

        // Cleanup
        sdiMsgStr = "";
        sdiMsgReady = false;
        Serial.println("--- [CYCLE END] ---\n");
    }
}

// LOGIC FUNCTIONS

// PARSER
void decodeATMOS41(String msg) {
    if (msg.length() < 2) return;
    msg.remove(0, 1); // Remove Address '0'

    float values[25]; 
    int vCount = 0;
    String currentVal = "";
    
    // Standard Parsing Logic
    for (int i = 0; i < msg.length(); i++) {
        char c = msg[i];
        bool isDelimiter = (c == '+' || c == ' ');
        bool isSign = (c == '-');

        if ((isDelimiter || isSign) && currentVal.length() > 0) {
            if (vCount < 25) values[vCount++] = currentVal.toFloat();
            currentVal = "";
            if (isSign) currentVal += c;
        } else if (c != '+' && c != ' ') {
            currentVal += c;
        }
    }
    if (currentVal.length() > 0 && vCount < 25) values[vCount++] = currentVal.toFloat();

    // Mapping to Global Struct (Indices based on ATMOS 41 Gen 2 Manual)
    // [1] Precip, [4] WindSpd, [5] WindDir, [7] Temp, [9] Baro, [10] RH
    if(vCount > 10) {
        currentMeasure.rain_mm       = values[1];
        currentMeasure.wind_speed_ms = values[4];
        currentMeasure.wind_dir_deg  = values[5];
        currentMeasure.temp_c        = values[7];
        currentMeasure.pressure_kpa  = values[9];
        currentMeasure.humidity_pct  = values[10];

        Serial.printf("[DATA] Temp: %.2f C, Hum: %.2f %%, Rain: %.2f mm\n", 
                      currentMeasure.temp_c, currentMeasure.humidity_pct, currentMeasure.rain_mm);
    } else {
        Serial.println("[ERR] Parsing failed or incomplete frame.");
    }
}

// Fonction pour afficher le buffer en HEX
void debugBufferState(uint8_t* buf, int len) {
    Serial.print("   -> Buffer actuel (" + String(len) + " octets) : [ ");
    for (int j = 0; j < len; j++) {
        if (buf[j] < 0x10) Serial.print("0"); // Ajoute un 0 pour l'alignement (ex: 0F)
        Serial.print(buf[j], HEX);
        Serial.print(" ");
    }
    Serial.println("]");
}

// LORA SENDER
static void send_lora_frame(void) {
    if (lmh_join_status_get() != LMH_SET) {
        Serial.println("[LORA] Not joined yet. Skip send.");
        return;
    }

    // Scale floats to integers (x100 for 2 decimal precision)
    int16_t  rain_cnt      = (int16_t)(currentMeasure.rain_mm * 100);
    uint16_t wind_dir      = (uint16_t)(currentMeasure.wind_dir_deg);
    int16_t  wind_spd_cnt  = (int16_t)(currentMeasure.wind_speed_ms * 100);
    int16_t  temp_cnt      = (int16_t)(currentMeasure.temp_c * 100);
    int16_t  hum_cnt       = (int16_t)(currentMeasure.humidity_pct * 100);
    int16_t  press_cnt     = (int16_t)(currentMeasure.pressure_kpa * 100);

    uint8_t i = 0;
    m_lora_app_data.port = LORAWAN_APP_PORT;
    
    Serial.println("--- Début de la construction du payload ---");

    // Rain
    Serial.print("Ajout Rain (Int16): "); Serial.println(rain_cnt);
    put_i16_be(m_lora_app_data.buffer, i, rain_cnt);
    debugBufferState(m_lora_app_data.buffer, i);

    // Wind Direction
    Serial.print("Ajout Wind Dir (Uint16): "); Serial.println(wind_dir);
    put_u16_be(m_lora_app_data.buffer, i, wind_dir);
    debugBufferState(m_lora_app_data.buffer, i);

    // Wind Speed
    Serial.print("Ajout Wind Spd (Int16): "); Serial.println(wind_spd_cnt);
    put_i16_be(m_lora_app_data.buffer, i, wind_spd_cnt);
    debugBufferState(m_lora_app_data.buffer, i);

    // Temperature
    Serial.print("Ajout Temp (Int16): "); Serial.println(temp_cnt);
    put_i16_be(m_lora_app_data.buffer, i, temp_cnt);
    debugBufferState(m_lora_app_data.buffer, i);

    // Humidity
    Serial.print("Ajout Hum (Int16): "); Serial.println(hum_cnt);
    put_i16_be(m_lora_app_data.buffer, i, hum_cnt);
    debugBufferState(m_lora_app_data.buffer, i);

    // Pressure
    Serial.print("Ajout Press (Int16): "); Serial.println(press_cnt);
    put_i16_be(m_lora_app_data.buffer, i, press_cnt);
    debugBufferState(m_lora_app_data.buffer, i);

    Serial.println("--- Fin du payload ---");


    m_lora_app_data.buffsize = i;

    lmh_error_status error = lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG);
    if (error == LMH_SUCCESS) {
        Serial.println("[LORA] Packet sent successfully.");
    } else {
        Serial.printf("[LORA] Send failed. Error: %d\n", error);
    }
}

// EVENT HANDLERS
// The TIMER only sets the FLAG. It does NO heavy work.
static void tx_lora_periodic_handler(void) {
    TimerSetValue(&appTimer, LORAWAN_APP_TX_DUTYCYCLE);
    TimerStart(&appTimer);
    
    // Trigger the main loop
    startMeasurementCycle = true;
}

static void lorawan_has_joined_handler(void) {
    Serial.println("[LORA] Joined!");
    lmh_class_request(CLASS_A);
    // Start the timer immediately upon join
    TimerSetValue(&appTimer, LORAWAN_APP_TX_DUTYCYCLE);
    TimerStart(&appTimer);
    // Trigger immediate first measure
    startMeasurementCycle = true; 
}

static void lorawan_join_failed_handler(void) {
    Serial.println("[LORA] Join Failed. Retrying...");
    lmh_join(); // Retry
}

static void lorawan_rx_handler(lmh_app_data_t *app_data) {
    // Optional: Handle downlink
}
static void lorawan_confirm_class_handler(DeviceClass_t Class) {}