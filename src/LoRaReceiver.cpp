#include <LoRa.h>
#include <ArduinoJson.h>
#include "BluetoothSerial.h"
#include "boards.h"

byte localAddress = 0x20; // address of this device
byte destination = 0x10;  // destination to send to
byte broadcast = 0xFF;

BluetoothSerial SerialBT;

void sendStringMessage(String outgoing)
{
    LoRa.beginPacket();            // start packet
    LoRa.write(destination);       // add destination address
    LoRa.write(localAddress);      // add sender address
    LoRa.write(outgoing.length()); // add payload length
    LoRa.print(outgoing);          // add payload
    LoRa.endPacket();              // finish packet and send it

    delay(100);

    LoRa.receive();

    // Serial.println("Transmitted message");
    // Serial.println("Sent to: 0x" + String(destination, HEX));
    // Serial.println("Message length: " + String(outgoing.length()));
    // Serial.println("Message: " + outgoing);
    // Serial.println();

#ifdef HAS_DISPLAY
        if (u8g2)
        {
            u8g2->clearBuffer();
            char buf[256];
            u8g2->drawStr(0, 12, "Send OK!");
            u8g2->drawStr(0, 26, outgoing.c_str());
            snprintf(buf, sizeof(buf), "Length:%i", outgoing.length());
            u8g2->drawStr(0, 40, buf);
            snprintf(buf, sizeof(buf), "Dest:%.1f", destination);
            u8g2->drawStr(0, 56, buf);
            u8g2->sendBuffer();
        }
#endif    
}

void sendMessage(DynamicJsonDocument doc)
{
    String output;
    serializeJson(doc, output);
    sendStringMessage(output);
}

void setup()
{
    initBoard();
    // When the power is turned on, a delay is required.
    delay(1500);

    Serial.begin(115200);

    SerialBT.begin("Kart - Base Staion"); // Bluetooth device name
    Serial.println("The device started, now you can pair it with bluetooth!");

    Serial.println("LoRa Receiver");

    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DI0_PIN);
    if (!LoRa.begin(LoRa_frequency))
    {
        Serial.println("Starting LoRa failed!");
        while (1)
            ;
    }

    Serial.println("LoRa init succeeded.");
}

void loop()
{
    if (SerialBT.available())
    {
        DynamicJsonDocument doc(1024);
        doc["type"] = "message";
        doc["message"] = SerialBT.readString();
        sendMessage(doc);
    }

    if (Serial.available())
    {
        DynamicJsonDocument doc(1024);
        doc["type"] = "message";
        doc["message"] = Serial.readString();
        sendMessage(doc);
    }

    // try to parse packet
    int packetSize = LoRa.parsePacket();
    if (packetSize)
    {
        // read packet header bytes:
        int recipient = LoRa.read();       // recipient address
        byte sender = LoRa.read();         // sender address
        byte incomingLength = LoRa.read(); // incoming msg length

        String incoming = ""; // payload of packet

        while (LoRa.available())
        {                                  // can't use readString() in callback, so
            incoming += (char)LoRa.read(); // add bytes one by one
        }

        if (incomingLength != incoming.length())
        { // check length for error
            Serial.println("error: message length does not match length");
            return; // skip rest of function
        }

        // if the recipient isn't this device or broadcast,
        if (recipient != localAddress && recipient != 0xFF)
        {
            Serial.println("This message is not for me.");
            return; // skip rest of function
        }

        // if message is for this device, or broadcast, print details:
        // Serial.println("Received from: 0x" + String(sender, HEX));
        // Serial.println("Sent to: 0x" + String(recipient, HEX));
        // Serial.println("Message length: " + String(incomingLength));
        // Serial.println("Message: " + incoming);
        // Serial.println("RSSI: " + String(LoRa.packetRssi()));
        // Serial.println("Snr: " + String(LoRa.packetSnr()));
        // Serial.println();

        DynamicJsonDocument idoc(1024);
        deserializeJson(idoc, incoming);

        DynamicJsonDocument doc(1024);
        doc["data"] = idoc;
        doc["rssi"] = LoRa.packetRssi();
        doc["snr"] = LoRa.packetSnr();
        doc["sender"] = String(sender, HEX);
        doc["recipient"] = String(recipient, HEX);

        String output;
        serializeJson(doc, output);
        SerialBT.println(output);
        Serial.println(output);

#ifdef HAS_DISPLAY
        if (u8g2)
        {
            u8g2->clearBuffer();
            char buf[256];
            u8g2->drawStr(0, 12, "Received OK!");
            u8g2->drawStr(0, 26, incoming.c_str());
            snprintf(buf, sizeof(buf), "RSSI:%i", LoRa.packetRssi());
            u8g2->drawStr(0, 40, buf);
            snprintf(buf, sizeof(buf), "SNR:%.1f", LoRa.packetSnr());
            u8g2->drawStr(0, 56, buf);
            u8g2->sendBuffer();
        }
#endif
    }
}
