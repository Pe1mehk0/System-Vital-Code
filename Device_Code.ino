#include <WiFi.h>
#include <Wire.h>
#include "Adafruit_MPRLS.h"
#include "ecg3_click.h"
#include <SparkFun_Bio_Sensor_Hub_Library.h>
#include "Protocentral_MAX30205.h"

// WiFi Configuration
const char* ssid = "TransceiverNet";
const char* password = "12345678";
const char* serverIP = "192.168.4.1";
const int port = 5000;
WiFiClient client;

// Pin Definitions
#define ECG_CS_PIN 15
#define RESET_PIN  -1  
#define EOC_PIN    -1 
#define ECG_BUFFER_SIZE 5000
#define ECG_BATCH_SIZE 15000
#define ECG_INTERVAL 2
#define TCAADDR 0x70

#define pumpPin  12      // Pin to control the air pump
#define solenoidPin 27

int resPin = -4;
int mfioPin = 14;

// Sensor Measurement Intervals
#define SENSOR_INTERVAL 60000  // 60 seconds

// Sensor Instances
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
SparkFun_Bio_Sensor_Hub bioHub(-1, 14);
MAX30205 tempSensor;
ECG3Click ecg(ECG_CS_PIN);
bioData body;  

// Task Handles
TaskHandle_t TaskECG;
TaskHandle_t TaskECGSend;
TaskHandle_t TaskWiFiSend;
TaskHandle_t TaskSensors;

// ECG & Sensor Queues
QueueHandle_t ecgQueue;
QueueHandle_t sensorQueue;
QueueHandle_t ECGQueue;

// Mutex for WiFi
SemaphoreHandle_t wifiMutex;

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
  delay(10); 
}

unsigned long start_pressure = millis();
unsigned long start_temp = millis();
unsigned long start_sat = millis();
unsigned long current_pressure = 0;
unsigned long current_temp = 0;
unsigned long current_sat = 0;

float baselinePressure;


void setup() {
    Serial.begin(115200);
    delay(100);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi!");

    ecg.begin();

    pinMode(pumpPin, OUTPUT);
    pinMode(solenoidPin, OUTPUT);
    digitalWrite(pumpPin, LOW);
    digitalWrite(solenoidPin, LOW);

    // Create Queues
    ecgQueue = xQueueCreate(ECG_BATCH_SIZE, sizeof(int32_t));
    if (!ecgQueue) Serial.println("ECG Queue Creation Failed!");

    sensorQueue = xQueueCreate(5, 256);
    if (!sensorQueue) Serial.println("Sensor Queue Creation Failed!");

   ECGQueue = xQueueCreate(1, sizeof(int32_t*)); 
    if (!ECGQueue) Serial.println("ECG Queue Creation Failed!");

    // Create Mutex
    wifiMutex = xSemaphoreCreateMutex();

    // Start Tasks
    xTaskCreatePinnedToCore(TaskECGFunction, "ECG_Task", 12000, NULL, 2, &TaskECG, 0);
    xTaskCreatePinnedToCore(TaskECGSendFunction, "ECG_Send_Task", 15000, NULL, 2, &TaskECGSend, 1);
    xTaskCreatePinnedToCore(TaskWiFiSendFunction, "WiFi_Send_Task", 10000, NULL, 1, &TaskWiFiSend, 1);
    xTaskCreatePinnedToCore(TaskSensorsFunction, "Sensor_Task", 10000, NULL, 1, &TaskSensors, 1);

    Wire.begin(22, 20);
    Wire.setClock(100000);
    delay(500);

    tcaselect(0);
    Serial.println("MPRLS Simple Test");
    if (!mpr.begin(0x18)) {
      Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
      while (1) {
        delay(10);
      }
    }
    Serial.println("Found MPRLS sensor");

    tcaselect(1);
    while(!tempSensor.scanAvailableSensors()){
      Serial.println("Couldn't find the temperature sensor, please connect the sensor." );
      delay(10000);
    }
    tempSensor.begin();

    tcaselect(2);
    delay(300);
    int result = bioHub.begin();
    if (result == 0) // Zero errors!
      Serial.println("Sensor started!");
    else
      Serial.println("Could not communicate with the sensor!");
 
    Serial.println("Configuring Sensor...."); 
    int error = bioHub.configBpm(MODE_ONE); // Configuring just the BPM settings. 
    if(error == 0){ // Zero errors!
      Serial.println("Sensor configured.");
    }
    else {
      Serial.println("Error configuring sensor.");
      Serial.print("Error: "); 
      Serial.println(error); 
    }

  bioHub.setPulseWidth(411);
  delay(50); // Wider pulse for deep tissue
  // Step 5: Set Sample Rate for Stability
  bioHub.setSampleRate(25);
  delay(50);  // Reduce noise, better for upper arm
  // Step 6: Increase ADC Range for Weaker Signals
  bioHub.setAdcRange(16384); 
  delay(50);
  // Step 7: Enable Automatic Gain Control
  bioHub.agcAlgoControl(ENABLE);
  delay(50); 
  // Step 8: Optimize Algorithm Sensitivity
  bioHub.setAlgoSensitivity(40); 
  delay(50); 
  bioHub.setAlgoSamples(32); 
  delay(50);     
  bioHub.setAlgoRange(40);  
  delay(50);      


  // Data lags a bit behind the sensor, if you're finger is on the sensor when
  // it's being configured this delay will give some time for the data to catch
  // up. 
  Serial.println("Loading up the buffer with data....");
  delay(4000);  
}

// ECG Measurement Task
int counter_ECG = 0;

float base = 0.00;

void inflateCuff(float targetPressure) {
  Serial.println("Inflating cuff...");
  base = mpr.readPressure() / 68.947572932;
  Serial.println(base);
  digitalWrite(pumpPin, HIGH);
  digitalWrite(solenoidPin, HIGH);

  while (true) {
    float pressure = mpr.readPressure() / 68.947572932 - base; // Read pressure in PSI
    if (!isnan(pressure)) {
      Serial.print("Current Pressure: ");
      Serial.print(pressure);
      Serial.println(" PSI");
      if (pressure >= targetPressure) {
        break;
      }
    }
    delay(100);
  }
  digitalWrite(pumpPin, LOW); // Stop the pump
  Serial.println("Target pressure reached.");
}

void deflateCuffGradually() {
  Serial.println("Deflating cuff gradually...");
  digitalWrite(solenoidPin, HIGH); // Open solenoid for gradual deflation
}

void stopDeflation() {
  digitalWrite(solenoidPin, LOW);  // Close solenoid
  Serial.println("Deflation stopped.");
}




void TaskECGFunction(void *pvParameters) {
    while (1) {
        int32_t ecgValue = ecg.getECG();  

        if (xQueueSend(ecgQueue, &ecgValue, 0) != pdTRUE) {
            Serial.println("ECG Queue FULL! Dropping Data...");
        } else {
            counter_ECG++;  
        }

        if (counter_ECG >= ECG_BUFFER_SIZE) {
            Serial.println("5000 ECG Samples Collected! Ready to Send.");
            counter_ECG = 0;
            xTaskNotifyGive(TaskECGSend);
        }
      delay(8); // My sampling rate is 8 <-- cjange later
    }
}

// ECG Data Collection & Sending Task
void TaskECGSendFunction(void *pvParameters) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Каждый раз выделяем новую память
        int32_t* sendBuffer = (int32_t*)malloc(ECG_BUFFER_SIZE * sizeof(int32_t));
        if (!sendBuffer) {
            Serial.println("Memory Allocation Failed!");
            continue;  // не продолжаем без памяти
        }

        Serial.println("📤 Preparing ECG Data for Transmission...");

        for (int i = 0; i < ECG_BUFFER_SIZE; i++) {
            if (xQueueReceive(ecgQueue, &sendBuffer[i], portMAX_DELAY) != pdTRUE) {
                Serial.println("⚠️ ECG Queue Empty! Waiting...");
                break;
            }
        }

        if (xQueueSend(ECGQueue, &sendBuffer, 0) != pdTRUE) {
            Serial.println("⚠️ WiFi Queue Full! Dropping ECG Data.");
            free(sendBuffer);
        } else {
            Serial.println("ECG Data Added to WiFi Queue");
        }
    }
}

// Sensor Data Collection Task (Runs Every 60 Seconds)
void TaskSensorsFunction(void *pvParameters) {
    float temp = 36.5; // Default parameter
    int spo2 = 0;     // Default parameter
    int hr = 0;       // Default parameter
    float sysBP = 0; 
    float diaBP = 0;
    bool flag = false;
    
    while (1) {  
      char sensorMessage[256];
      flag = false;

      current_pressure = millis();
      current_temp = millis();
      current_sat = millis();  
        
      if(current_temp - start_temp >= 30000)
      {
        tcaselect(1);
        temp = getTemperature();
        start_temp = millis();
        flag = true;
        Serial.println("Meassured Tempreture");
        Serial.println(temp);
      }
      if(current_sat - start_sat >= 60000)
      {
        tcaselect(2);
        delay(300);
        start_sat = millis();
        getHeartRateAndSpO2(&hr, &spo2);
        flag = true;
        Serial.println("Meassured SpO2");
        Serial.println(hr, spo2);
      }
      if(current_pressure - start_pressure >= 900000)
      {
        tcaselect(0);
        measureBloodPressure(&sysBP, &diaBP);
        start_pressure = millis();
        flag = true;
      }
      if(flag == false){
        delay(30000);
      }

        snprintf(sensorMessage, sizeof(sensorMessage), "SENSOR_DATA TEMP=%.2f SPO2=%d HR=%d SYS=%.2f DIA=%.2f",
                 temp, spo2, hr, sysBP, diaBP);

        Serial.println("Adding Sensor Data to WiFi Queue...");
        
        if (xQueueSend(sensorQueue, sensorMessage, 0) != pdTRUE) {
            Serial.println("WiFi Queue Full! Dropping Sensor Data.");
        } else {
            Serial.println("Sensor Data Added to WiFi Queue");
        }
    }
}

// WiFi Transmission Task (Handles ECG & Sensor Data)
void TaskWiFiSendFunction(void *pvParameters) {
    int32_t* ecgSendBuffer;
    char sensorData[256];

    while (1) {
        if (xQueueReceive(ECGQueue, &ecgSendBuffer, 0) == pdTRUE) {
            Serial.println("Sending ECG Data...");
            if (xSemaphoreTake(wifiMutex, portMAX_DELAY)) {
                if (client.connect(serverIP, port)) {
                    client.print("ECGDATA\n");
                    client.flush();
                    delay(100);  // give server a chance to process header

                    const int chunkSize = 1024;
                    int bytesToSend = ECG_BUFFER_SIZE * sizeof(int32_t);
                    uint8_t* dataPtr = (uint8_t*)ecgSendBuffer;

                  while (bytesToSend > 0) {
                    size_t sendNow = min((size_t)chunkSize, (size_t)bytesToSend);
                    size_t sent = client.write(dataPtr, sendNow);

                    if (sent == 0) {
                      Serial.println("Failed to send chunk!");
                      break;
                    }

                    dataPtr += sent;
                    bytesToSend -= sent;

                    delay(10); // give some breathing room for the network
                  }

                // Check if all data was sent before printing success message
                if (bytesToSend == 0) {
                    Serial.println("ECG Data sent completely in chunks");
                } else {
                    Serial.print("Incomplete ECG Data sent. Remaining bytes: ");
                    Serial.println(bytesToSend);
                }

                client.stop();
                } else {
                    Serial.println("WiFi Connection Failed!");
                }
                xSemaphoreGive(wifiMutex);
            }
            free(ecgSendBuffer);  // Only now free the memory
        }

        if (xQueueReceive(sensorQueue, &sensorData, 0) == pdTRUE) {
            Serial.println("Sending Sensor Data...");
            if (xSemaphoreTake(wifiMutex, portMAX_DELAY)) {
                if (client.connect(serverIP, port)) {
                    client.println("SENSORD");
                    client.print(sensorData);
                    client.flush();
                    delay(50);
                    client.stop();
                    Serial.println("Sensor Data Sent Successfully");
                } else {
                    Serial.println("WiFi Connection Failed! Retrying...");
                }
                xSemaphoreGive(wifiMutex);
            }
        }

        delay(100);
    }
}


void getHeartRateAndSpO2(int* hrOut, int* spo2Out) {
  const int MAX_READINGS = 50;
  int spo2Values[MAX_READINGS];
  int hrValues[MAX_READINGS];
  int count = 0;
  int count2 = 0;

  unsigned long start = millis();
  while (millis() - start < 15000 && count < MAX_READINGS) {
    bioData current = bioHub.readBpm();
    if (current.heartRate != -999 && current.confidence >= 30) {
      spo2Values[count] = current.oxygen;
      hrValues[count] = current.heartRate;
      count2++;
    }
    count++;
    delay(250);  // 10Hz
  }

  if (count2 == 0) {
    *hrOut = -1;
    *spo2Out = -1;
    Serial.println("No valid HR/SpO₂ readings");
    return;
  }

  // Mode calculation
  int countsHR[141] = {0};    // for HR 40-180
  int countsSpO2[11] = {0};   // for SpO2 90-100

  for (int i = 0; i < count; i++) {
    int hri = hrValues[i];
    int s = spo2Values[i];
    if (hri >= 40 && hri <= 180) countsHR[hri - 40]++;
    if (s >= 90 && s <= 100) countsSpO2[s - 90]++;
  }

  int bestHR = 40;
  int bestSpO2 = 90;
  int maxHR = 0, maxSpO2 = 0;

  for (int i = 0; i < 141; i++) {
    if (countsHR[i] > maxHR) {
      maxHR = countsHR[i];
      bestHR = i + 40;
    }
  }

  for (int i = 0; i < 11; i++) {
    if (countsSpO2[i] > maxSpO2) {
      maxSpO2 = countsSpO2[i];
      bestSpO2 = i + 90;
    }
  }

  *hrOut = bestHR;
  *spo2Out = bestSpO2;

  Serial.print("HR: ");
  Serial.print(bestHR);
  Serial.print(" | SpO₂: ");
  Serial.println(bestSpO2);
}

float getTemperature() {  // Select the appropriate I2C channel
  return tempSensor.getTemperature();
}

void measureBloodPressure(float* systolicOut, float* diastolicOut) {
  float systolic = 0.0;
  float diastolic = 0.0;

  inflateCuff(3.5); // Inflate to 3.5 PSI (~180 mmHg)

  Serial.println("Measuring blood pressure...");
  
  bool systolicDetected = false;
  bool diastolicDetected = false;

  // Arrays to store pressure oscillations
  const int bufferSize = 100;
  float pressureReadings[bufferSize];
  int index = 0;

  // Deflate cuff gradually and read pressure
  deflateCuffGradually();
  while (!diastolicDetected) {
    float currentPressure = mpr.readPressure() / 68.947572932 - base;
    if (!isnan(currentPressure)) {
      pressureReadings[index % bufferSize] = currentPressure;

      // Check for systolic and diastolic pressures
      if (index > 1) {
        float prevPressure = pressureReadings[(index - 1) % bufferSize];
        float oscillation = currentPressure - prevPressure;

        // Detect systolic pressure (first large oscillation)
        if (!systolicDetected && oscillation > 0.04 && currentPressure > 1.5) {
          systolic = currentPressure * 51.7149; // Convert PSI to mmHg
          systolicDetected = true;
          Serial.print("Systolic detected at: ");
          Serial.print(systolic);
          Serial.println(" mmHg");
        }

        // Detect diastolic pressure (smaller oscillations near baseline)
       if (systolicDetected && !diastolicDetected && oscillation < 0.03 && currentPressure < 1.5) {
        diastolic = currentPressure * 51.7149; // Convert PSI to mmHg
        diastolicDetected = true;
        Serial.print("Diastolic detected at: ");
        Serial.print(diastolic);
        Serial.println(" mmHg");
        }
      }

      index++;
      Serial.print("Pressure: ");
      Serial.print(currentPressure);
      Serial.println(" PSI");
    }

    delay(100); // Delay between readings
  }
  stopDeflation();

  Serial.println("Measurement complete.");
  Serial.print("Systolic: ");
  Serial.print(systolic);
  Serial.println(" mmHg");
  Serial.print("Diastolic: ");
  Serial.print(diastolic);
  Serial.println(" mmHg");

  *systolicOut = systolic;
  *diastolicOut = diastolic;
}

void loop() {
    vTaskDelay(portMAX_DELAY);  // Prevents loop() from running
}
