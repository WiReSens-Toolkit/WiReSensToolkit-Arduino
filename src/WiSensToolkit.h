/*
  WiSensToolkit.h - Library for wireless resistive tactile sensing
  Created by Devin Murphy, May 21, 2024
*/
#ifndef WiSensToolkit_h
#define WiSensToolkit_h

#ifdef ESP32
#include <esp_now.h>
#include <driver/adc.h>
#include <NimBLEDevice.h>
#endif

#include <WiFi.h>
#include <DS_MCP4018.h>
#include <set>
#include <EEPROM.h>
#include <ArduinoJson.h>

#include "Arduino.h"

using namespace std;

// Define Bluetooth UUID
#define SERVICE_UUID "dc643f2d-a2f8-4a00-a964-e138b4d82fdb"
#define CHARACTERISTIC_UUID "1766324e-8b30-4d23-bff2-e5209c3d986f"

boolean calibrationCallback();

struct struct_message
{
    int8_t sendId;
    uint16_t startIdx;
    uint16_t *TruePressure;
    uint16_t *PredPressure;
    size_t PressureArraySize;

    // Constructor
    struct_message(size_t size)
        : PressureArraySize(size), TruePressure(new uint16_t[size]), PredPressure(new uint16_t[size]) {}

    // Destructor
    ~struct_message()
    {
        delete[] TruePressure;
        delete[] PredPressure;
    }

    // Disable copy constructor and copy assignment operator
    struct_message(const struct_message &) = delete;
    struct_message &operator=(const struct_message &) = delete;

    // Move constructor
    struct_message(struct_message &&other) noexcept
        : sendId(other.sendId), startIdx(other.startIdx),
          TruePressure(other.TruePressure), PredPressure(other.PredPressure), PressureArraySize(other.PressureArraySize)
    {
        other.TruePressure = nullptr;
        other.PredPressure = nullptr;
    }

    // Move assignment operator
    struct_message &operator=(struct_message &&other) noexcept
    {
        if (this != &other)
        {
            delete[] TruePressure;
            delete[] PredPressure;
            sendId = other.sendId;
            startIdx = other.startIdx;
            TruePressure = other.TruePressure;
            PredPressure = other.PredPressure;
            PressureArraySize = other.PressureArraySize;
            other.TruePressure = nullptr;
            other.PredPressure = nullptr;
        }
        return *this;
    }
};

// Enum to specify communication protocol
typedef enum
{
    WIFI,
    BLUETOOTH,
    ESPNOW,
    SERIALCOMM
} CommProtocol;

// Struct for WiFi configuration
struct WiFiConfig
{
    char *ssid;
    char *password;
    char *host;
    uint16_t port;

    // Default constructor
    WiFiConfig() : ssid(nullptr), password(nullptr), host(nullptr), port(0) {}

    // Parameterized constructor
    WiFiConfig(const char *ssid, const char *password, const char *host, uint16_t port)
    {
        this->ssid = strdup(ssid);
        this->password = strdup(password);
        this->host = strdup(host);
        this->port = port;
    }

    // Copy constructor
    WiFiConfig(const WiFiConfig &other)
    {
        this->ssid = strdup(other.ssid);
        this->password = strdup(other.password);
        this->host = strdup(other.host);
        this->port = other.port;
    }

    // Destructor
    ~WiFiConfig()
    {
        free(ssid);
        free(password);
        free(host);
    }

    // Assignment operator
    WiFiConfig &operator=(const WiFiConfig &other)
    {
        if (this != &other)
        {
            // Free existing resources
            free(ssid);
            free(password);
            free(host);

            // Duplicate resources from the other object
            ssid = strdup(other.ssid);
            password = strdup(other.password);
            host = strdup(other.host);
            port = other.port;
        }
        return *this;
    }
};
// Struct for Bluetooth configuration
struct BluetoothConfig
{
    char *deviceName;

    // Default constructor
    BluetoothConfig() : deviceName(nullptr) {}

    // Parameterized constructor
    BluetoothConfig(const char *deviceName) : deviceName(strdup(deviceName)) {}

    // Copy constructor
    BluetoothConfig(const BluetoothConfig &other) : deviceName(strdup(other.deviceName)) {}

    // Destructor
    ~BluetoothConfig()
    {
        free(deviceName);
    }

    // Assignment operator
    BluetoothConfig &operator=(const BluetoothConfig &other)
    {
        if (this != &other)
        {
            free(deviceName);
            deviceName = strdup(other.deviceName);
        }
        return *this;
    }
};

// Struct for ESP-NOW configuration
struct EspNowConfig
{
    uint8_t peerAddress[6];

    // Default constructor
    EspNowConfig()
    {
        memset(peerAddress, 0, sizeof(peerAddress));
    }

    // Parameterized constructor
    EspNowConfig(const uint8_t *peerAddress)
    {
        memcpy(this->peerAddress, peerAddress, sizeof(this->peerAddress));
    }

    // Copy constructor
    EspNowConfig(const EspNowConfig &other)
    {
        memcpy(peerAddress, other.peerAddress, sizeof(peerAddress));
    }
};

// Union to hold configuration data
struct CommConfigUnion
{
    WiFiConfig wifiConfig;
    BluetoothConfig bluetoothConfig;
    EspNowConfig espNowConfig;
};

// Struct to handle communication configuration
struct CommConfig
{
    int numNodes;           // Number of nodes to read before sending
    int delay = 0;          // Time in ms to delay after sending
    CommProtocol protocol;  // Protocol type
    CommConfigUnion config; // Union for configurations

    // Constructor for WiFi
    CommConfig(CommProtocol prot, int nodes, int delay, const WiFiConfig &wifi) : protocol(prot), numNodes(nodes), delay(delay)
    {
        if (protocol == WIFI)
        {
            config.wifiConfig = WiFiConfig(wifi);
        }
    }

    // Constructor for Bluetooth
    CommConfig(CommProtocol prot, int nodes, int delay, const BluetoothConfig &bluetooth) : protocol(prot), numNodes(nodes), delay(delay)
    {
        if (protocol == BLUETOOTH)
        {
            new (&config.bluetoothConfig) BluetoothConfig(bluetooth);
        }
    }

    // Constructor for ESP-NOW
    CommConfig(CommProtocol prot, int nodes, int delay, const EspNowConfig &espNow) : protocol(prot), numNodes(nodes), delay(delay)
    {
        if (protocol == ESPNOW)
        {
            new (&config.espNowConfig) EspNowConfig(espNow);
        }
    }

    // Copy constructor
    CommConfig(const CommConfig &other) : protocol(other.protocol)
    {
        switch (protocol)
        {
        case WIFI:
            new (&config.wifiConfig) WiFiConfig(other.config.wifiConfig);
            break;
        case BLUETOOTH:
            new (&config.bluetoothConfig) BluetoothConfig(other.config.bluetoothConfig);
            break;
        case ESPNOW:
            new (&config.espNowConfig) EspNowConfig(other.config.espNowConfig);
            break;
        default:
            // Handle invalid protocol
            break;
        }
    }

    // Destructor
    ~CommConfig()
    {
        switch (protocol)
        {
        case WIFI:
            config.wifiConfig.~WiFiConfig();
            break;
        case BLUETOOTH:
            config.bluetoothConfig.~BluetoothConfig();
            break;
        case ESPNOW:
            config.espNowConfig.~EspNowConfig();
            break;
        default:
            // Handle invalid protocol
            break;
        }
    }

    // Assignment operator
    CommConfig &operator=(const CommConfig &other)
    {
        if (this != &other)
        {
            // Cleanup current configuration
            this->~CommConfig();
            // Copy new configuration
            new (this) CommConfig(other);
        }
        return *this;
    }
};

struct ReadoutConfig
{
    int numGroundPins; // Number of Digital Pins used to Control Grounding Wires
    int numReadPins;   // Number of Digital Pins used to control Reading wires
    int *groundPins;   // Pointer to array of Digital Pins controlling Grounding Wires
    int *readPins;     // Pointer to array of Digital Pins controlling Reading Wires
    int *startCoord;   // Pointer to start coordinates
    int *endCoord;     // Pointer to end coordinates
    int adcPin;        // Digital Pin for the Analog-to-Digital Converter
    double resistance; // Resistance to set for the Digital Potentiometer (in Ohms)

    // Default Constructor
    ReadoutConfig() : numGroundPins(0), numReadPins(0), groundPins(nullptr), readPins(nullptr), startCoord(nullptr), endCoord(nullptr), adcPin(0), resistance(0.0) {}

    // Parameterized Constructor
    ReadoutConfig(int numGroundPins, int numReadPins, int *groundPins, int *readPins, int *startCoord, int *endCoord, int adcPin, double resistance) : numGroundPins(numGroundPins), numReadPins(numReadPins), adcPin(adcPin), resistance(resistance)
    {
        this->groundPins = new int[numGroundPins];
        this->readPins = new int[numReadPins];
        for (int i = 0; i < numGroundPins; ++i)
        {
            this->groundPins[i] = groundPins[i];
        }
        for (int i = 0; i < numReadPins; ++i)
        {
            this->readPins[i] = readPins[i];
        }
        this->startCoord = new int[2];
        this->startCoord[0] = startCoord[0];
        this->startCoord[1] = startCoord[1];
        this->endCoord = new int[2];
        this->endCoord[0] = endCoord[0];
        this->endCoord[1] = endCoord[1];
    }
    // Copy Constructor
    ReadoutConfig(const ReadoutConfig &other)
        : numGroundPins(other.numGroundPins), numReadPins(other.numReadPins), adcPin(other.adcPin), resistance(other.resistance)
    {
        groundPins = new int[numGroundPins];
        readPins = new int[numReadPins];
        for (int i = 0; i < numGroundPins; ++i)
        {
            groundPins[i] = other.groundPins[i];
        }
        for (int i = 0; i < numReadPins; ++i)
        {
            readPins[i] = other.readPins[i];
        }
        startCoord = new int[2];
        startCoord[0] = other.startCoord[0];
        startCoord[1] = other.startCoord[1];
        endCoord = new int[2];
        endCoord[0] = other.endCoord[0];
        endCoord[1] = other.endCoord[1];
    }
    // Assignment Operator
    ReadoutConfig &operator=(const ReadoutConfig &other)
    {
        if (this == &other)
            return *this; // Self-assignment check

        // Clean up existing resources
        delete[] groundPins;
        delete[] readPins;
        delete[] startCoord;
        delete[] endCoord;

        // Copy data
        numGroundPins = other.numGroundPins;
        numReadPins = other.numReadPins;
        adcPin = other.adcPin;
        resistance = other.resistance;

        groundPins = new int[numGroundPins];
        readPins = new int[numReadPins];
        for (int i = 0; i < numGroundPins; ++i)
        {
            groundPins[i] = other.groundPins[i];
        }
        for (int i = 0; i < numReadPins; ++i)
        {
            readPins[i] = other.readPins[i];
        }

        startCoord = new int[2];
        startCoord[0] = other.startCoord[0];
        startCoord[1] = other.startCoord[1];
        endCoord = new int[2];
        endCoord[0] = other.endCoord[0];
        endCoord[1] = other.endCoord[1];

        return *this;
    }
    // Destructor
    ~ReadoutConfig()
    {
        delete[] groundPins;
        delete[] readPins;
        delete[] startCoord;
        delete[] endCoord;
    }
};

typedef boolean (*CalibrationCallback)();

struct PotConfig
{
    uint8_t potAddress;      // I2C address of the Pot
    uint8_t writeCommand;    // Pot write command
    uint8_t readCommand;     // Pot read command
    uint8_t numSteps;        // Number of steps in the pot
    int maxResistance;       // Maximum resistance of the pot
    uint8_t wiperResistance; // Estimated resistance of the wiper

    // Constructor with default values
    PotConfig(uint8_t potAddress = 0x2F, uint8_t writeCommand = 0x00, uint8_t readCommand = 0x01,
              uint8_t numSteps = 128, int maxResistance = 50000, uint8_t wiperResistance = 100)
        : potAddress(potAddress), writeCommand(writeCommand), readCommand(readCommand),
          numSteps(numSteps), maxResistance(maxResistance), wiperResistance(wiperResistance)
    {
    }
};

struct Node
{
    int id;
    int value;

    // Comparison operator to sort the set by value first, then by id
    bool operator<(const Node &other) const
    {
        if (value != other.value)
            return value < other.value;
        return id < other.id;
    }
};

class MinValuesTracker
{
public:
    MinValuesTracker(int k) : k(k) {}
    void addNode(int id, int value)
    {
        Node newNode{id, value};

        // If the set already has k nodes, check if the new node's value is less than the max value in the set
        if (nodes.size() == k)
        {
            auto maxNodeIt = std::prev(nodes.end());
            if (newNode.value >= maxNodeIt->value)
            {
                // If the new node's value is not less than the max value, do not insert it
                return;
            }
        }

        // Check if the node is already in the set
        auto it = std::find_if(nodes.begin(), nodes.end(),
                               [&id](const Node &n)
                               { return n.id == id; });

        if (it != nodes.end())
        {
            // If node is already in the set, remove it
            nodes.erase(it);
        }

        // Insert the new node
        nodes.insert(newNode);

        // If there are more than k nodes, remove the node with the maximum value
        if (nodes.size() > k)
        {
            nodes.erase(std::prev(nodes.end()));
        }
    }

    double avgMinValues()
    {
        double sum = 0;
        for (const auto &node : nodes)
        {
            sum += node.value;
        }
        return sum / k;
    }

private:
    std::set<Node> nodes;
    int k;
};

class TwoDArray
{
public:
    TwoDArray(int rows, int cols) : rows(rows), cols(cols)
    {
        data = new uint16_t[rows * cols];
    }

    ~TwoDArray()
    {
        delete[] data;
    }

    u16_t &operator()(int row, int col)
    {
        return data[row * cols + col];
    }

    void copyInto(u16_t startIdx, u16_t *src, int length)
    {
        for (int i = 0; i < length; ++i)
        {
            int index = (startIdx + i) % (rows * cols);
            data[index] = src[i];
        }
    }

private:
    uint16_t *data;
    int rows, cols;
};

class WiSensToolkit
{
public:
    ReadoutConfig *thisReadoutConfig;
    WiSensToolkit(ReadoutConfig *readoutConfig, CommProtocol commType, CommConfig *commConfig, int sendId);
    void scanArray();
    void calibrate(long, double saturatedPercentage);
    bool serial_communication = true;
    void readNode(int readWire, int groundWire);
    float p = 15;
    float d = 81;
    bool intermittent = false;

private:
    class MyServerCallbacks : public BLEServerCallbacks
    {
        bool *connection_ptr = nullptr;

    public:
        MyServerCallbacks(bool *_bptr);
        void onConnect(BLEServer *pServer);
        void onDisconnect(BLEServer *pServer);
    };
    struct_message *tactile_data = nullptr;
    MCP4018 *MCP;
    BLEService *pService = nullptr;
    BLECharacteristic *pCharacteristic = nullptr;
    BLEServer *pServer = nullptr;
    BLEAdvertising *pAdvertising = nullptr;
    WiFiClient *tactileClient;
    IPAddress *thisIp;
    void sendBluetooth();
    void BluetoothSetup();
    bool deviceConnected = false;
    bool oldDeviceConnected = false;
    bool serial_print = false; // Flag for printing debug messages to serial monitor
    uint32_t packetNumber = 0;
    uint8_t packetCount = 0;
    int8_t thisSendId;
    bool intermittentInit;
    bool firstPass;
    int totalError;
    int currPacket;
    uint8_t *buffer;
    size_t bufferSize;
    CommProtocol currentCommType;
    CommConfig *thisCommConfig;
    TwoDArray *currNodes;
    TwoDArray *pastNodes;
    void WiFiSetup();
    void sendWiFi();
    void initCommProtocol(CommProtocol commType, CommConfig *commConfig);
    void EspNowSetup();
    void sendEspNow();
    void ADCSetup();
    void selectMuxPin(byte pin, bool row);
    void sendResult();
    void createBuffer(struct_message *tactile_data);

    void sendSerial();
    void initDigiPot(uint8_t potStep);
    double calculateResistance(uint8_t potValue, int maxResistance);
    uint8_t calculatePotValue(double resistance, int maxResistance);
};

WiSensToolkit *createKit(bool useSaved);

void clearEEPROM(int startAddress, int length);

void saveDataToEEPROM(const String &data);

#endif