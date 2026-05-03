/**
 * @file esp32_gamepad_bridge.cpp
 * @brief ESP32 Bluetooth Gamepad to UART Bridge
 *
 * This firmware utilizes the Bluepad32 library to receive input from a
 * Bluetooth gamepad and translates these inputs into single-byte UART
 * commands for the main STM32 motor controller. It employs an event-driven
 * architecture to minimize UART traffic and includes connection safety fallbacks.
 */

#include <Bluepad32.h>

#define LED_PIN 2

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

/**
 * State tracking variables for event-driven transmission.
 * lastSentChar stores the most recently transmitted command to prevent UART spamming.
 * 'O' represents the default neutral state (all buttons released).
 */
char lastSentChar = '\0'; 
bool wasConnected = false;

void setup() {
    // Initialize standard USB UART for system debugging and monitoring
    Serial.begin(115200);  
    delay(1000);

    // Initialize Hardware Serial 2 for communication with STM32
    // RX2 = GPIO 16, TX2 = GPIO 17
    Serial2.begin(115200, SERIAL_8N1, 16, 17);
    pinMode(LED_PIN, OUTPUT);

    Serial.println("System Initialization: Starting Bluepad32...");
    
    // Register controller connection callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);
}

void loop() {
    // Process incoming Bluetooth data and update internal controller states
    BP32.update();
    
    bool isAnyConnected = false;
    char currentChar = 'O'; // Default state: No action

    // Iterate through connected controllers to process input
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        ControllerPtr ctl = myControllers[i];
        
        if (ctl && ctl->isConnected()) {
            isAnyConnected = true;

            // Directly read the current state of the controller
            uint16_t btns = ctl->buttons();
            int ry = ctl->axisRY();
            int lx = ctl->axisX();
            int ly = ctl->axisY();

            /*
             * Priority 1: Emergency Stop evaluation
             * This must override all other inputs to ensure system safety.
             */
            if (btns & 0x0020) {
                currentChar = 'P';
            }
            /*
             * Priority 2: Standard action buttons mapping
             */
            else if (btns & 0x0001) currentChar = 'A';
            else if (btns & 0x0002) currentChar = 'B';
            else if (btns & 0x0004) currentChar = 'Y';
            else if (btns & 0x0010) currentChar = 'M';
            /*
             * Priority 3: Analog sticks and D-Pad evaluation
             * Incorporates a +/- 200 unit deadzone to prevent drift inputs.
             */
            else if (ly < -200) currentChar = 'U';
            else if (ly > 200)  currentChar = 'D';
            else if (lx < -200) currentChar = 'L';
            else if (lx > 200)  currentChar = 'R';
            else if (ry > 0)    currentChar = 'F';

            break; // Process input from the first active controller only
        }
    }

    /*
     * Event-Driven Transmission Logic:
     * Only transmit data over UART when a state change is detected.
     */
    if (currentChar != lastSentChar) {
        Serial2.print(currentChar); 
        
        Serial.print("Event Triggered - Transmitted to STM32: ");
        Serial.println(currentChar);
        
        lastSentChar = currentChar;
    }

    /*
     * Connection State Management and Visual Feedback
     */
    if (isAnyConnected) {
        digitalWrite(LED_PIN, HIGH); // Solid LED indicates active connection
        
        if (!wasConnected) {
            Serial.println("System Status: Controller Paired and Active.");
            wasConnected = true;
        }
    } else {
        // Blinking LED indicates searching/pairing mode (1Hz frequency)
        digitalWrite(LED_PIN, (millis() / 500) % 2); 
        
        /*
         * Safety Fallback Mechanism:
         * If the controller disconnects unexpectedly, instantly transmit the
         * neutral command ('O') to halt all STM32 motor operations.
         */
        if (wasConnected) {
            Serial2.print('O');
            lastSentChar = 'O';
            wasConnected = false;
            Serial.println("CRITICAL WARNING: Controller connection lost. Emergency neutral command 'O' transmitted.");
        }
    }

    // Maintain an approximate 50Hz polling rate
    delay(20); 
}

/**
 * @brief Callback function triggered when a new controller connects.
 * @param ctl Pointer to the connected controller object.
 */
void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            myControllers[i] = ctl;
            break;
        }
    }
}

/**
 * @brief Callback function triggered when a controller disconnects.
 * @param ctl Pointer to the disconnected controller object.
 */
void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            myControllers[i] = nullptr;
            break;
        }
    }
}