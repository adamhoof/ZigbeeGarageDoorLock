#include <Arduino.h>
#include <TMCStepper.h>
/*#include <HardwareSerial.h>*/

// Pin definitions and constants
#define DIR_PIN 1
#define STEP_PIN 0
#define STALL_PIN 27 //DIAG
#define DRIVER_ADDRESS 2
#define R_SENSE 0.11f

constexpr uint16_t fullPathStepCount = 12500;
constexpr uint16_t offsetSteps = 500;

enum Direction
{
    BACKWARD = 1,
    FORWARD = 0,
};

volatile bool stalled = false;

void stallInterrupt()
{
    stalled = true;
}

enum ResultCode
{
    MOVE_OK,
    MOVE_STALL,
    MOVE_TOO_LONG
};

struct Result
{
    size_t steps;
    ResultCode resultCode;
};

TMC2209Stepper driver(&Serial1, R_SENSE, DRIVER_ADDRESS);

// Configure the TMC2209 driver
void configureDriver()
{
    driver.begin();
    driver.toff(5);
    driver.rms_current(600);
    driver.microsteps(16);
    driver.en_spreadCycle(false);
    driver.pwm_autoscale(true);
    driver.TCOOLTHRS(0xFFFFF);
    driver.SGTHRS(20);
}

// Set up the pins
void configurePins()
{
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(STALL_PIN, INPUT);
}

// Move stepper until a stall is detected or maximum steps reached
Result moveUntilStall(const size_t maxSteps, const Direction dir)
{
    size_t doneSteps = 0;
    stalled = false;
    driver.shaft(dir);

    unsigned long lastStepTime = micros();  // Store the last step time

    while (true) {
        if (stalled) {
            return Result{doneSteps, MOVE_STALL};
        }
        if (doneSteps > maxSteps) {
            return Result{doneSteps, MOVE_TOO_LONG};
        }

        unsigned long now = micros();
        if (now - lastStepTime >= 320) {  // 160us HIGH + 160us LOW = 320us per step
            lastStepTime = now;
            digitalWrite(STEP_PIN, HIGH);
            delayMicroseconds(1);  // Short pulse for HIGH state
            digitalWrite(STEP_PIN, LOW);
            ++doneSteps;
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
        // Yield control to other tasks to avoid watchdog timeout
    }
}


// Move a set number of steps
Result moveSteps(size_t stepsLeft, const Direction dir)
{
    size_t doneSteps = 0;
    stalled = false;
    driver.shaft(dir);

    for (; stepsLeft > 0; --stepsLeft) {
        if (stalled) {
            return Result{doneSteps, MOVE_STALL};
        }
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(160);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(160);
        ++doneSteps;
    }
    return Result{doneSteps, MOVE_OK};
}

// Open the lock by moving the motor in the BACKWARD direction
void openLock()
{
    uint8_t counter = 0;
    Result res = moveSteps(fullPathStepCount, BACKWARD);
    while (res.resultCode != MOVE_OK) {
        if (counter == 3) {
            Serial.println("Lock bricked");
            break;
        }
        res = moveSteps(fullPathStepCount - res.steps, BACKWARD);
        ++counter;
    }
}

// Close the lock by moving until a stall is detected (FORWARD direction)
void closeLock()
{
    uint8_t counter = 0;
    Result res = moveUntilStall(fullPathStepCount + offsetSteps, FORWARD);
    while (res.resultCode != MOVE_STALL) {
        if (counter == 3) {
            Serial.println("Lock bricked");
            break;
        }
        res = moveUntilStall(fullPathStepCount + offsetSteps - res.steps, FORWARD);
        ++counter;
    }
}

// Home the lock by closing it
void homeLock()
{
    closeLock();
}


void motorTask(void *pvParameters) {
    homeLock();
    while (true) {
        Serial.println(driver.microsteps());
        vTaskDelay(pdMS_TO_TICKS(1000));  // FreeRTOS-friendly delay
    }
}

extern "C" void app_main()
{
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, 22, 12);

    // Configure hardware pins and attach interrupt for stall detection
    configurePins();
    attachInterrupt(digitalPinToInterrupt(STALL_PIN), stallInterrupt, RISING);

    // Initialize and configure the driver
    configureDriver();
    delay(100);
    xTaskCreate(motorTask, "MotorTask", 4096, nullptr, 1, nullptr);
}
