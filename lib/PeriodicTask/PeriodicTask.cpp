#include "PeriodicTask.h"
#include <Arduino.h>

PeriodicTask::PeriodicTask(unsigned int interval, void (*task)()) 
    : previousMillis(0), interval(interval), task(task) {}

void PeriodicTask::update() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        task();
    }
}