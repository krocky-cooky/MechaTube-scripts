#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>


int decodeCommand(const char* command, bool* pPower, bool* pControl, bool* pMode, float* pTorque, float* pSpeed);

class characteristicCallbacks : public BLECharacteristicCallbacks {
    public:
        bool* pPower;
        bool* pControl;
        bool* pMode;
        float* pTorque;
        float* pSpeed;

    characteristicCallbacks( bool* _pPower, bool* _pControl, bool* _pMode, float* _pTorque, float* _pSpeed):BLECharacteristicCallbacks() {
        pPower = _pPower;
        pControl = _pControl;
        pMode = _pMode;
        pTorque = _pTorque;
        pSpeed = _pSpeed;
    }

    void onWrite(BLECharacteristic* pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        Serial.println(value.c_str());
        decodeCommand(value.c_str(),pPower,pControl,pMode,pTorque,pSpeed);
    }
};

class funcServerCallbacks: public BLEServerCallbacks {
    public:
        bool* deviceConnected;

    funcServerCallbacks(bool* _deviceConnected) : BLEServerCallbacks(){
        this->deviceConnected = _deviceConnected;
    }
    void onConnect(BLEServer* pServer) {
        *deviceConnected = true;
        Serial.println("connected");
    }

    void onDisconnect (BLEServer* pServer) {
        *deviceConnected = false;
        Serial.println("disconnected");
    }
};