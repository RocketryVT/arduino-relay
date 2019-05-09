void setup()
{
    Serial.begin(115200);
}

void loop()
{
    uint8_t buffer[Serial.available()];
    Serial.readBytes(buffer, sizeof(buffer));
    Serial.write(buffer, sizeof(buffer));
}
