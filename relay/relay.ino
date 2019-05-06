#include <CircularBuffer.h>
#include <SPI.h>
#include <RH_RF95.h>

const uint8_t RFM95_CS = 4;
const uint8_t RFM95_RST = 2;
const uint8_t RFM95_INT = 3;
const double RF95_FREQ = 434.0;
const size_t BUFFER_SIZE = RH_RF95_MAX_MESSAGE_LEN * 2;
const uint8_t REPLY_PACKET[6] = {0xAA, 0x14, 0x69, 0x00, 0x13, 0x14};
const uint32_t serial_baudrate = 38400;

RH_RF95 rf95(RFM95_CS, RFM95_INT);

CircularBuffer<uint8_t, BUFFER_SIZE> buf1, buf2, buf3, buf4;

/**
 * Parses the provided buffer for packets, and removes all bytes which were processed.
 */
CircularBuffer<uint8_t, BUFFER_SIZE> parse_packet(CircularBuffer<uint8_t, BUFFER_SIZE>& buf);

/**
 * Gets all data from a serial port, up to the maximum size of the circular buffer.
 */
void serial_receive(CircularBuffer<uint8_t, BUFFER_SIZE> &buffer, HardwareSerial &dstream);

/**
 * Listens for a short time and stores all data from the radio,
 * up to the maximum size of the buffer.
 */
void radio_recieve(CircularBuffer<uint8_t, BUFFER_SIZE> &buffer);

/**
 * Writes a data buffer to 2 serial ports and the radio, and sends a confirmation
 * packet to the source serial port.
 */
void echo_serial(CircularBuffer<uint8_t, BUFFER_SIZE> &buffer,
    HardwareSerial &source, HardwareSerial &dest1, HardwareSerial &dest2);

/**
 * Writes a data buffer to 3 serial ports, and sends a confirmation packet
 * over the radio.
 */
void echo_radio(CircularBuffer<uint8_t, BUFFER_SIZE> &buffer,
    HardwareSerial &dest1, HardwareSerial &dest2, HardwareSerial &dest3);

void setup()
{
    // put your setup code here, to run once:
    Serial1.begin(serial_baudrate);
    Serial2.begin(serial_baudrate);
    Serial3.begin(serial_baudrate);
    Serial.begin(serial_baudrate);

    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);
    delay(100);
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    while (!rf95.init())
    {
        Serial.println("LoRa radio init failed");
        while (1);
    }
    Serial.println("LoRa radio init OK!");

    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
    if (!rf95.setFrequency(RF95_FREQ))
    {
        Serial.println("setFrequency failed");
        while (1);
    }

    Serial.print("Set Freq to: ");
    Serial.println(RF95_FREQ);
    rf95.setTxPower(23, false);

}

void loop() {

    serial_receive(buf1, Serial1);
    serial_receive(buf2, Serial2);
    serial_receive(buf3, Serial3);
    radio_recieve(buf4);

    CircularBuffer<uint8_t, BUFFER_SIZE> from1 = parse_packet(buf1);
    CircularBuffer<uint8_t, BUFFER_SIZE> from2 = parse_packet(buf2);
    CircularBuffer<uint8_t, BUFFER_SIZE> from3 = parse_packet(buf3);
    CircularBuffer<uint8_t, BUFFER_SIZE> fromRadio = parse_packet(buf4);

    // echoes, and sends confirmation to, the first serial port
    echo_serial(from1, Serial1, Serial2, Serial3);
    echo_serial(from2, Serial2, Serial1, Serial3);
    echo_serial(from3, Serial3, Serial1, Serial2);
    // echoes, and sends confirmation to, the radio
    echo_radio(fromRadio, Serial1, Serial2, Serial3);
}

void serial_receive(CircularBuffer<uint8_t, BUFFER_SIZE> &buffer, HardwareSerial &dstream)
{
    while (buffer.available() > 0 && dstream.available() > 0)
    {
        buffer.push(dstream.read());
    }
}

void radio_recieve(CircularBuffer<uint8_t, BUFFER_SIZE> &buffer)
{
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.waitAvailableTimeout(100))
    {
        if (rf95.recv(buf, &len))
        {
            Serial.println("Got Reply: ");
            for (int i = 0; i < len && buffer.available() > 0; i++)
            {
                buffer.push(buf[i]);
            }
        }
    }
}

CircularBuffer<uint8_t, BUFFER_SIZE> parse_packet(CircularBuffer<uint8_t, BUFFER_SIZE> &buf)
{
    bool parsing = true;
    CircularBuffer<uint8_t, BUFFER_SIZE> packet;
    while (parsing && buf.size() > 0)
    {
        Serial.println("in firstwhile");
        while (buf.size() > 0 && buf.first() != 0xAA)
        {
            buf.shift();
        }

        if (buf.size() < 2)
        {
            continue;
        }

        if (buf[1] != 0x14)
        {
            buf.shift();
            continue;
        }

        if (buf.size() < 4)
        {
            parsing = false;
            continue;
        }

        uint8_t length = buf[2];
        if (buf.size() < 6 + length)
        {
            parsing = false;
            continue;
        }

        for (int i = 0; i < 6 + length; ++i)
        {
            packet.push(buf.shift());
        }
        return packet;
    }
    return packet;
}

void echo_serial(CircularBuffer<uint8_t, BUFFER_SIZE> &buffer,
                 HardwareSerial &source, HardwareSerial &dest1, HardwareSerial &dest2)
{
    uint8_t toSend[buffer.size()];
    const size_t bufferSize = buffer.size();
    for (size_t i = 0; buffer.size() > 0; i++)
    {
        toSend[i] = buffer.shift();
    }

    source.write(REPLY_PACKET, sizeof(REPLY_PACKET));
    dest1.write(toSend, bufferSize);
    dest2.write(toSend, bufferSize);
    rf95.send(toSend, bufferSize);
}

void echo_radio(CircularBuffer<uint8_t, BUFFER_SIZE> &buffer,
                HardwareSerial &dest1, HardwareSerial &dest2, HardwareSerial &dest3)
{
    uint8_t toSend[buffer.size()];
    const size_t bufferSize = buffer.size();
    for (size_t i = 0; buffer.size() > 0; i++)
    {
        toSend[i] = buffer.shift();
    }

    rf95.send(REPLY_PACKET, sizeof(REPLY_PACKET));
    dest1.write(toSend, bufferSize);
    dest2.write(toSend, bufferSize);
    dest3.write(toSend, bufferSize);
}
