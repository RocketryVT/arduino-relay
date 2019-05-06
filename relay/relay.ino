#include <CircularBuffer.h>
#include <SPI.h>
#include <RH_RF95.h>
#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3
#define RF95_FREQ 434.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

const size_t BUFFER_SIZE = 256;
CircularBuffer<uint8_t, BUFFER_SIZE> buf1, buf2, buf3, buf4;

CircularBuffer<uint8_t, BUFFER_SIZE> parse_r(CircularBuffer<uint8_t, BUFFER_SIZE>* buf);
void send_r(CircularBuffer<uint8_t, BUFFER_SIZE>* buffer, HardwareSerial stream1, HardwareSerial stream2, HardwareSerial stream3, uint8_t len, bool);
void fill_r(CircularBuffer<uint8_t, BUFFER_SIZE>* buffer, HardwareSerial *dstream);
void fill_spi(CircularBuffer<uint8_t, BUFFER_SIZE>* buffer);


void setup() {
  // put your setup code here, to run once:
  Serial1.begin(38400);
  Serial2.begin(38400);
  Serial3.begin(38400);
  //Serial.begin(38400);
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

void loop() {
  //  rf95.send((uint8_t*)(0x69), sizeof(0x69));
  //      delay(10);
  //      rf95.waitPacketSent();
  Serial.println("hello");

  fill_r(&buf1, &Serial1);
  fill_r(&buf2, &Serial2);
  fill_r(&buf3, &Serial3);
  fill_spi(&buf4);
  //fill_r(&buf4, &Serial);

  Serial.print("buf1 size: ");
  Serial.println(buf1.size());
  Serial.print("buf2 size: ");
  Serial.println(buf2.size());
  Serial.print("buf3 size: ");
  Serial.println(buf3.size());
  Serial.print("buf4 size: ");
  Serial.println(buf4.size());

  Serial.println("before first parse");
  CircularBuffer<uint8_t, BUFFER_SIZE> packet1 = parse_r(&buf1);
  Serial.print("packet1 size: ");
  Serial.println(packet1.size());
  Serial.println("after first parse");
  CircularBuffer<uint8_t, BUFFER_SIZE> packet2 = parse_r(&buf2);
  Serial.println("after second parse");
  CircularBuffer<uint8_t, BUFFER_SIZE> packet3 = parse_r(&buf3);
  Serial.println("after third parse");
  CircularBuffer<uint8_t, BUFFER_SIZE> packet4 = parse_r(&buf4);
  Serial.println("after fourth parse");
  Serial.print("packet4 size: ");
  Serial.println(packet4.size());

  send_r(&packet1, Serial2, Serial3, Serial1, packet1.size(), true);
  Serial.println("after first send");
  send_r(&packet2, Serial1, Serial3, Serial2, packet2.size(), true);
  Serial.println("after second send");
  send_r(&packet3, Serial1, Serial2, Serial3, packet3.size(), true);
  Serial.println("after third send");
  send_r(&packet4, Serial1, Serial2, Serial3, packet4.size(), false);//should be false
  Serial.println("after fourth send");

  delay(1000);
}

void fill_r(CircularBuffer<uint8_t, BUFFER_SIZE>* buffer, HardwareSerial *dstream)
{
  while (buffer->available() > 0 && dstream->available() > 0)
  {
    buffer->push(dstream->read());
  }
}

void fill_spi(CircularBuffer<uint8_t, BUFFER_SIZE>* buffer)
{
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  Serial.println("Waiting for reply...");
  if (rf95.waitAvailableTimeout(100))
  {
    if (rf95.recv(buf, &len))
    {
      Serial.println("Got Reply: ");
      for (int i = 0; i < len; i++)
      {
        buffer->push(*(buf + i));
        Serial.print("Data: ");
        Serial.println(*(buf + i));
        Serial.print("buffer size in fill: ");
        Serial.println(buffer->size());
      }
    }
    else
    {
      Serial.println("Receive failed you goddamn scrub");
    }
  }
}

CircularBuffer<uint8_t, BUFFER_SIZE> parse_r(CircularBuffer<uint8_t, BUFFER_SIZE>* buf)
{
  Serial.println("in parse");
  bool parsing = true;
  CircularBuffer<uint8_t, BUFFER_SIZE> packet;
  while (parsing && buf->size() > 0)
  {
    Serial.println("in firstwhile");
    while (buf->size() > 0 && buf->first() != 0xAA)
    {
      Serial.println("looking for first 0xAA");
      buf->shift();
    }

    if (buf->size() < 2)
    {
      Serial.println("doesnt have at least 2 bytes yet");
      continue;
    }

    if ((*buf)[1] != 0x14)
    {
      Serial.println("2nd byte not right");
      buf->shift();
      continue;
    }

    if (buf->size() < 4)
    {
      Serial.println("havent recieved full header yet");
      parsing = false;
      continue;
    }

    uint8_t length = (*buf)[2];
    Serial.print("length: ");
    Serial.println(length);

    if (buf->size() < 6 + length)
    {
      Serial.println("packet header recieved, waiting for data");
      parsing = false;
      continue;
    }

    for (int i = 0; i < 6 + length; ++i)
    {
      packet.push(buf->shift());
    }
    Serial.println("Packet fully received");
    return packet;
  }
  Serial.println("no packet");
  return packet;
}

void send_r(CircularBuffer<uint8_t, BUFFER_SIZE> *buffer, HardwareSerial stream1, HardwareSerial stream2, HardwareSerial stream3, uint8_t len, bool spi)
{
  Serial.println("before while in send");
  uint8_t toSend[len];
  uint8_t received[1] = {0x69};
  size_t index = 0;
  while (buffer->size() > 0)
  {
    Serial.println("in while loop");
    if (spi) //should just return received key to spi. This nonsense should be in else
    {
      toSend[index] = buffer->first();
      index++;
      if (stream1)
      {
        Serial.println("Trying to send to stream 1");
        stream1.write(buffer->first());
        Serial.println("Sent to stream 1");
      }
      if (stream2)
      {
        Serial.println("Trying to send to stream 2");
        stream2.write(buffer->shift());
        Serial.println("Sent to stream 2");
      }
      if (stream3)
      {
        stream3.write(0x69);//packet received message
      }

    }
    else
    {
      if (stream1) {
        Serial.println("Trying to send to stream 1");
        stream1.write(buffer->first());
        Serial.println("sent to stream 1");
      }
      if (stream2) {
        Serial.println("Trying to send to stream 2");
        stream2.write(buffer->first());
        Serial.println("sent to stream 2");
      }
      if (stream3) {
        Serial.println("Trying to send to stream 3");
        stream3.write(buffer->shift());
        Serial.println("sent to stream 3");
      }
    }
  }
  if (spi)
  {
    if (index > 0)
    {
      rf95.send(toSend, sizeof(toSend));
    }
  }
  else
  {
    if (len > 0)
    {
      rf95.send(received, sizeof(received));
    }
  }

}
