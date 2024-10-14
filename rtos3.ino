#include <STM32FreeRTOS.h>

#define LED_TEST_PIN PA5       // LD2 on Nucleo board for reaction test indication
#define LED_BAD_PIN PA5        // Red LED for bad reaction time (connected to SCK/D13)
#define LED_GOOD_PIN PA6       // Green LED for good reaction time (connected to PWM/MOSI/D11)
#define LED_NORMAL_PIN PA7     // Blue LED for intermediate reaction time (connected to MISO/D12)
#define BUTTON_PIN PC13        // Button connected to pin PC13

#define DEBOUNCE_DELAY 50      
#define GOOD_REACTION_THRESHOLD 250  // Reaction time < 250 ms is good
#define BAD_REACTION_THRESHOLD 350   // Reaction time > 350 ms is bad

volatile int reactionTime = 0;  
volatile bool ledOn = false;    // State of the test LED (on/off)
volatile bool gameActive = false; 
SemaphoreHandle_t reactionSemaphore; // Semaphore for signaling button press


void TaskLEDControl(void *pvParameters);
void TaskButtonPress(void *pvParameters);
void TaskGameMonitor(void *pvParameters);

void setup() {
    Serial.begin(9600);
    while (!Serial) {
        ; // Wait for serial port to connect.
    }

    reactionSemaphore = xSemaphoreCreateBinary();

    randomSeed(analogRead(A0));

    // Create tasks
    xTaskCreate(TaskLEDControl, "LEDControl", 128, NULL, 2, NULL);
    xTaskCreate(TaskButtonPress, "ButtonPress", 128, NULL, 2, NULL);
    xTaskCreate(TaskGameMonitor, "GameMonitor", 128, NULL, 1, NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    // If the scheduler returns, print error and halt
    Serial.println("Insufficient RAM");
    while (1);
}

void loop() {
}

void TaskLEDControl(void *pvParameters) {
    (void) pvParameters;
    pinMode(LED_TEST_PIN, OUTPUT);

    // Initially, turn off the test LED
    digitalWrite(LED_TEST_PIN, LOW);

    for (;;) {
        // Wait for a random time between 1 and 3 seconds
        int waitTime = random(1000, 3000);
        vTaskDelay(waitTime / portTICK_PERIOD_MS);

        // Turn on the test LED and signal the game is active
        Serial.println("Turning on test LED (LD2)");
        digitalWrite(LED_TEST_PIN, HIGH);
        ledOn = true;
        gameActive = true;

        // Capture the time when the test LED is turned on
        unsigned long ledOnTime = millis();

        // Wait for the button press signal or timeout
        if (xSemaphoreTake(reactionSemaphore, portMAX_DELAY) == pdTRUE) {
            reactionTime = millis() - ledOnTime;

            // Turn off the test LED
            Serial.println("Turning off test LED (LD2)");
            digitalWrite(LED_TEST_PIN, LOW);
            ledOn = false;

            // Signal the game monitor task to handle the reaction result
            gameActive = false;
        }

        vTaskDelay(2000 / portTICK_PERIOD_MS); 
    }
}

// Task to handle button presses and calculate the reaction time
void TaskButtonPress(void *pvParameters) {
    (void) pvParameters;
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    for (;;) {
        if (digitalRead(BUTTON_PIN) == LOW) { 
            vTaskDelay(DEBOUNCE_DELAY / portTICK_PERIOD_MS); 

            if (digitalRead(BUTTON_PIN) == LOW && ledOn) { // Confirm button press and LED is on
                xSemaphoreGive(reactionSemaphore); // Signal the LED control task

                // Wait for button release
                while (digitalRead(BUTTON_PIN) == LOW);
            }
        }

        vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay to prevent task hogging
    }
}

// Task to monitor the game and provide feedback based on the reaction time
void TaskGameMonitor(void *pvParameters) {
    (void) pvParameters;
    pinMode(LED_BAD_PIN, OUTPUT);
    pinMode(LED_GOOD_PIN, OUTPUT);
    pinMode(LED_NORMAL_PIN, OUTPUT);

    for (;;) {
        if (!gameActive && reactionTime > 0) {
            digitalWrite(LED_TEST_PIN, LOW);

            if (reactionTime < GOOD_REACTION_THRESHOLD) {
                Serial.println("Good reaction time: Green LED");
                digitalWrite(LED_GOOD_PIN, HIGH);
            } else if (reactionTime < BAD_REACTION_THRESHOLD) {
                Serial.println("Intermediate reaction time: Yellow LED");
                digitalWrite(LED_NORMAL_PIN, HIGH);
            } else {
                Serial.println("Bad reaction time: Red LED");
                digitalWrite(LED_BAD_PIN, HIGH);
            }

            vTaskDelay(1000 / portTICK_PERIOD_MS);

            digitalWrite(LED_BAD_PIN, LOW);
            digitalWrite(LED_GOOD_PIN, LOW);
            digitalWrite(LED_NORMAL_PIN, LOW);

            Serial.print("Reaction Time: ");
            Serial.print(reactionTime);
            Serial.println(" ms");

            // Reset reaction time for the next round
            reactionTime = 0;

            vTaskDelay(3000 / portTICK_PERIOD_MS); // 3-second delay before the next round
        }

        vTaskDelay(100 / portTICK_PERIOD_MS); 
    }
}
