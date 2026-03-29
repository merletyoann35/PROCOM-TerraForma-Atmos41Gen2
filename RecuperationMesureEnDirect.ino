/**
    @file SecondAtmosCode.ino
*/

#include "RAK13010_SDI12.h"

#define TX_PIN   WB_IO6
#define RX_PIN   WB_IO5
#define OE       WB_IO4

#define SENSOR_ADDRESS '0'

RAK_SDI12 mySDI12(RX_PIN, TX_PIN, OE);

String sdiResponse = "";
String sdiMsgStr   = "";
bool   sdiMsgReady = false;

void setup()
{
    pinMode(WB_IO2, OUTPUT);
    digitalWrite(WB_IO2, HIGH);

    Serial.begin(115200);
    time_t timeout = millis();
    while (!Serial && (millis() - timeout) < 5000) delay(100);

    Serial.println("Opening SDI-12 bus.");
    mySDI12.begin();
    delay(500);

    mySDI12.forceListen();
    Serial.println("Press ENTER to send 0R0!");
}

void loop()
{
    static uint8_t serialMsgRflag = 0;

    // Detect ENTER key
    if (Serial.available())
    {
        char c = Serial.read();
        if (c == '\n' || c == '\r')
            serialMsgRflag = 1;
    }

    // Read SDI-12 incoming
    int avail = mySDI12.available();
    if (avail < 0)
    {
        mySDI12.clearBuffer();
    }
    else if (avail > 0)
    {
        for (int i = 0; i < avail; i++)
        {
            char c = mySDI12.read();
            if (c == '\n')
                sdiMsgReady = true;
            else
                sdiMsgStr += String(c);
        }
    }

    // When full SDI-12 message received
    if (sdiMsgReady)
    {
        Serial.println("\nRaw SDI-12:");
        Serial.println(sdiMsgStr);

        decodeR0(sdiMsgStr);

        sdiMsgStr = "";
        sdiMsgReady = false;
        serialMsgRflag = 0;
    }

    // User requested a measurement
    if (serialMsgRflag == 1)
    {
        serialMsgRflag = 2;
        String cmd = String(SENSOR_ADDRESS) + "R0!";
        mySDI12.sendCommand(cmd);
        Serial.println("T: " + cmd);
    }
}

/**
 * Decode and print the values from the R0 response
 */
void decodeR0(String msg)
{
    // Remove address character (e.g., "0")
    if (msg.length() < 2) return;
    msg.remove(0, 1);

    // Split using '+'
    float values[20];
    int vCount = 0;

    int start = 0;
    for (int i = 0; i <= msg.length(); i++)
    {
        if (msg[i] == '+' || msg[i] == '-' || i == msg.length())
        {
            if (i != start)
            {
                String token = msg.substring(start, i);
                values[vCount++] = token.toFloat();
            }
            start = i;
        }
    }

    Serial.println("\n------ Decoded ATMOS-41 (R0) ------");
    if (vCount < 17)
    {
        Serial.println("Error: Too few values!");
        return;
    }

    Serial.print("Rain (raw) ................. "); Serial.println(values[0]);
    Serial.print("Rain (corr) ................ "); Serial.println(values[1]);
    Serial.print("Rain flag 1 ................ "); Serial.println(values[2]);
    Serial.print("Rain flag 2 ................ "); Serial.println(values[3]);
    Serial.print("Rain last min (mm) ......... "); Serial.println(values[4]);
    Serial.print("Wind direction (°) ......... "); Serial.println(values[5]);
    Serial.print("Wind speed avg (m/s) ....... "); Serial.println(values[6]);
    Serial.print("Wind gust (m/s) ............ "); Serial.println(values[7]);
    Serial.print("Vapor pressure (kPa) ....... "); Serial.println(values[8]);
    Serial.print("Humidity (%) ............... "); Serial.println(values[9]);
    Serial.print("Rain rate (mm/h) ........... "); Serial.println(values[10]);
    Serial.print("Air temp (°C) .............. "); Serial.println(values[11]);
    Serial.print("Solar radiation (W/m²) ..... "); Serial.println(values[12]);
    Serial.print("Barometric pressure (kPa) .. "); Serial.println(values[13]);
    Serial.print("Wind avg 5 min (m/s) ....... "); Serial.println(values[14]);
    Serial.print("Wind U component (m/s) ..... "); Serial.println(values[15]);
    Serial.print("Wind V component (m/s) ..... "); Serial.println(values[16]);

    Serial.println("-----------------------------------\n");
}
