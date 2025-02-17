// Modified version of example: https://github.com/espressif/arduino-esp32/blob/master/libraries/BLE/examples/UART/UART.ino
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool serialStarted = false;
bool mtuChecked = false;
uint16_t connId;
uint16_t actualMTU;


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      connId = pServer->getConnId();
      Serial.println("C");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      serialStarted = false;
      mtuChecked = false;
      Serial.println("D");
    }
};


class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      if (!serialStarted) {
        serialStarted = true;

        Serial0.setRxBufferSize(16384);  // really just for initial connection, actual use much less
        Serial0.setTxBufferSize(512);    // allow for MAVFTP messages
        //Serial0.begin(230400, SERIAL_8N1, 16, 17);  // for S3 board
        Serial0.begin(115200, SERIAL_8N1, 20, 21); // for C3 bridge
        Serial.println("S0");
      }

      std::string rxValue = pCharacteristic->getValue();
      size_t payloadLength = rxValue.length();
      if (payloadLength > 0) {
        Serial0.write((uint8_t*)rxValue.c_str(), payloadLength);
      }
    }
};


void setup() {
  Serial.begin(115200);
  Serial.println("Setup Started");

  // Create BLE Device, set MTU
  BLEDevice::init("mLRS BLE Bridge");
  BLEDevice::setMTU(515);  // plenty of headroom

  // Set BLE Power, P12 = 12 dBm
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P12);

  // Create BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  

  // Create BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create BLE Characteristics
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  pTxCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start Service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Setup Done, Advertising");
}

void loop() {

    if (deviceConnected) {

      if (!mtuChecked) { // have to do here because if you try and get the MTU in the callback it always reports 23...
        mtuChecked = true;
        delay(500);
        Serial.print("Conn ID from the onConnect Callback: ");
        Serial.print(connId);
        Serial.print(", MTU: ");
        actualMTU = pServer->getPeerMTU(connId);      
        Serial.println(actualMTU);
      }
       
      if (Serial0.available() >= 32) {

        // Calculate the number of bytes to read
        uint16_t availableBytes = Serial0.available();
        uint16_t bytesToRead = min(availableBytes, (uint16_t)(actualMTU - 3));  // Limit to negotiated MTU - 3 bytes for header

        // Create a buffer to hold the data
        uint8_t buffer[bytesToRead];
        
        // Read data into the buffer
        Serial0.readBytes(buffer, bytesToRead);

        // Send the data over BLE as a single chunk
        pTxCharacteristic->setValue(buffer, bytesToRead);
        pTxCharacteristic->notify();

        //Serial.println(bytesToRead);
      }


    }

    // disconnecting
    if (!deviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial0.end();
        Serial.println("Advertising");
        delay(4500); // less spam
    }
}
