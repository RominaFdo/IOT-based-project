#include <Wire.h>
#include <RTClib.h>
#include <Preferences.h>

RTC_DS3231 rtc;
Preferences preferences;

void setup() {
    Serial.begin(115200);

    // Initialize I2C with custom SDA and SCL pins
    Wire.begin(15, 0);

    // Initialize preferences
    preferences.begin("time", false);

    if (!rtc.begin()) {
        Serial.println("Couldn't find RTC");
        while (1);
    }

    // Clear previous time settings
    preferences.clear();

    // Set the RTC with the correct time
    DateTime correctTime = DateTime(2024, 7, 9, 17, 45, 0); // Replace with the correct time
    rtc.adjust(correctTime);

    // Store the new correct time in preferences
    preferences.putUInt("year", correctTime.year());
    preferences.putUInt("month", correctTime.month());
    preferences.putUInt("day", correctTime.day());
    preferences.putUInt("hour", correctTime.hour());
    preferences.putUInt("minute", correctTime.minute());
    preferences.putUInt("second", correctTime.second());
}

void loop() {
    DateTime now = rtc.now();

    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();

    delay(1000);
}
