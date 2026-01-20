#include "DHT.h"

#define DHTPIN 3
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println(F("DHT11 test!"));

    dht.begin();
}

void loop() {
    // put your main code here, to run repeatedly:
    delay(2000);

    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (isnan(h) || isnan(t))
    {
        Serial.println(F("Failed to read from DHT sensor!"));
        return;
    }

    Serial.print(F("Humidity: "));
    Serial.print(h);
    Serial.print(F("%  Temperature: "));
    Serial.print(t);
    Serial.print(F("¡ÆC "));
    Serial.println("");
}