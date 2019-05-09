
#include <SPI.h>
#include <RH_RF95.h>

const uint8_t RFM95_CS = 4;
const uint8_t RFM95_RST = 2;
const uint8_t RFM95_INT = 3;
const double RF95_FREQ = 434.0;

RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup()
{
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);

    Serial.begin(38400);
    delay(100);

    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    while (!rf95.init()) {
        Serial.println("LoRa radio init failed");
        while (1);
    }
    Serial.println("LoRa radio init OK!");

    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
    if (!rf95.setFrequency(RF95_FREQ)) {
        Serial.println("setFrequency failed");
        while (1);
    }
    Serial.print("Set Freq to: ");
    Serial.println(RF95_FREQ);
    rf95.setTxPower(23, false);
}

void loop()
{
    size_t available = Serial.available();
    uint8_t toSend[available];
    Serial.readBytes(toSend, available);

    if (available > 0)
    {
        rf95.send(toSend, available);
    }

    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.waitAvailableTimeout(100) && rf95.recv(buf, &len))
    {
        Serial.write(buf, len);
    }
}
