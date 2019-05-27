uint32_t counter = 0;
uint64_t bps = 10;
uint64_t last_packet = 0;
uint64_t wait_ms = 50;
uint8_t packet[] = {0xAA, 0x14, 4, 2, 0, 0, 0, 0, 0x00, 0x00};

void setup()
{
    Serial.begin(115200);
    Serial.println("Begin echo program using baudrate 115200.");
    delay(3000);
}

void loop()
{
    uint8_t buffer[Serial.available()];
    Serial.readBytes(buffer, sizeof(buffer));
    Serial.write(buffer, sizeof(buffer));
   
    if (millis() - last_packet >= wait_ms)
    {
        Serial.write((uint8_t) counter++);
        
        double wave1 = sin(PI*millis()/1000.0)*80;
        double wave2 = cos(PI*millis()/700.0)*110;
        double wave3 = sin(PI*millis()/2200.0)*3;
        wave3 = wave3*wave3*wave3*wave3;
        
        int8_t int1 = round(wave1);
        int8_t int2 = round(wave2);
        int8_t int3 = round(wave3);
        
        packet[7] = (uint8_t) (int3 & 0xFF);
        packet[6] = (uint8_t) (int3 >> 8) & 0xFF;
        packet[5] = int2;
        packet[4] = int1;
        
        packet[8] = 0 ^ packet[0] ^ packet[2] ^ packet[4] ^ packet[6];
        packet[9] = 0 ^ packet[1] ^ packet[3] ^ packet[5] ^ packet[7];
        
        Serial.write(packet, sizeof(packet));
        last_packet += wait_ms;
    }
}
