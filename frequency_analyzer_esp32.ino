// Author: Kinza Rashid
#include "FFT.h" //Yash Sanghvi’s ESP32 FFT library (available on Github)

#define SAMPLES 128
#define SAMPLING_FREQUENCY 4096

//ADC GPIO pins on ESP32
const int MIC_ONE = 32;  //yellow wire
const int MIC_TWO = 33;  //orange wire

//for storing Microphone history
unsigned int MIC_ONE_freq_history = 0;
unsigned int MIC_ONE_mag_history = 0;
unsigned int MIC_TWO_freq_history = 0;
unsigned int MIC_TWO_mag_history = 0;

int count_history_mic_one = 0;
int count_history_mic_two = 0;

//FFT configurations and buffers
fft_config_t *fft_config1 = NULL;
fft_config_t *fft_config2 = NULL;
float *fft_input1 = NULL;
float *fft_input2 = NULL;
float *fft_output1 = NULL;
float *fft_output2 = NULL;

//Detection parameters
const float MIN_WHISTLE_FREQ = 1100.0;
const float MAX_WHISTLE_FREQ = 2000.0;
const float MAGNITUDE_THRESHOLD = 10000.0;
const int MAGNITUDE_THRESHOLD_COUNT = 50;
const int HISTORY_THRESHOLD_COUNT = 5;
int threshold_count = 0;

//Motor A - Right wheel
int motorAPin1 = 4;
int motorAPin2 = 2;
int enableAPin = 15;

//Motor B - Left wheel
int motorBPin1 = 17;
int motorBPin2 = 16;
int enableBPin = 5;

void setup() {
  Serial.begin(9600);

  //initializing motor pins
  pinMode(motorAPin1, OUTPUT);
  pinMode(motorAPin2, OUTPUT);
  pinMode(enableAPin, OUTPUT);
  pinMode(motorBPin1, OUTPUT);
  pinMode(motorBPin2, OUTPUT);
  pinMode(enableBPin, OUTPUT);

  //allocating memory for FFT buffers
  fft_input1 = (float *)malloc(SAMPLES * sizeof(float));
  fft_output1 = (float *)malloc(SAMPLES * 2 * sizeof(float));
  fft_input2 = (float *)malloc(SAMPLES * sizeof(float));
  fft_output2 = (float *)malloc(SAMPLES * 2 * sizeof(float));

  if (!fft_input1 || !fft_output1 || !fft_input2 || !fft_output2) {
    Serial.println("Failed to allocate FFT buffers!");
    while (1)
      ;
  }

  //initializing FFT
  fft_config1 = fft_init(SAMPLES, FFT_REAL, FFT_FORWARD, fft_input1, fft_output1);
  fft_config2 = fft_init(SAMPLES, FFT_REAL, FFT_FORWARD, fft_input2, fft_output2);

  if (!fft_config1 || !fft_config2) {
    Serial.println("Failed to initialize FFT!");
    while (1)
      ;
  }

  //configureing ADC parameters
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
}

void loop() {

  Serial.println("Recording... ");
  unsigned long sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));

  //sampling collection with proper timing
  for (int i = 0; i < SAMPLES; i++) {
    unsigned long start_time = micros();
    fft_input1[i] = (float)analogRead(MIC_ONE);
    fft_input2[i] = (float)analogRead(MIC_TWO);
    while (micros() - start_time < sampling_period_us) {}
  }

  //executing FFT on sample
  fft_execute(fft_config1);
  fft_execute(fft_config2);

  //analysis
  float max_magnitude1 = 0;
  int peak_index1 = 0;
  float max_magnitude2 = 0;
  int peak_index2 = 0;

  for (int i = 1; i < SAMPLES / 2; i++) {
    float frequency1 = i * SAMPLING_FREQUENCY / SAMPLES;
    float frequency2 = i * SAMPLING_FREQUENCY / SAMPLES;

    if (frequency1 >= MIN_WHISTLE_FREQ && frequency1 <= MAX_WHISTLE_FREQ) {
      float magnitude1 = sqrt(sq(fft_output1[2 * i]) + sq(fft_output1[2 * i + 1]));
      if (magnitude1 > max_magnitude1) {
        max_magnitude1 = magnitude1;
        peak_index1 = i;
      }
    }

    if (frequency2 >= MIN_WHISTLE_FREQ && frequency2 <= MAX_WHISTLE_FREQ) {
      float magnitude2 = sqrt(sq(fft_output2[2 * i]) + sq(fft_output2[2 * i + 1]));
      if (magnitude2 > max_magnitude2) {
        max_magnitude2 = magnitude2;
        peak_index2 = i;
      }
    }
  }

  if (max_magnitude1 > MAGNITUDE_THRESHOLD) {
    float peak_frequency1 = peak_index1 * SAMPLING_FREQUENCY / SAMPLES;
    Serial.printf("MIC ONE - Freq: %.2f Hz, Mag: %.2f\n", peak_frequency1, max_magnitude1);

    MIC_ONE_freq_history += peak_frequency1;
    MIC_ONE_mag_history += max_magnitude1;
    count_history_mic_one++;
  } else threshold_count++;

  if (max_magnitude2 > MAGNITUDE_THRESHOLD) {
    float peak_frequency2 = peak_index2 * SAMPLING_FREQUENCY / SAMPLES;
    Serial.printf("MIC TWO - Freq: %.2f Hz, Mag: %.2f\n", peak_frequency2, max_magnitude2);

    MIC_TWO_freq_history += peak_frequency2;
    MIC_TWO_mag_history += max_magnitude2;
    count_history_mic_two++;
  } else threshold_count++;
  Serial.print("Threshold count: ");
  Serial.println(threshold_count);
  if (count_history_mic_one >= HISTORY_THRESHOLD_COUNT || count_history_mic_two >= HISTORY_THRESHOLD_COUNT) task_audio_processing();
  if (threshold_count >= MAGNITUDE_THRESHOLD_COUNT) resetHistoryVariables();
  delay(100);
}

void task_audio_processing() {
  Serial.println("\n*****Audio Processing*****\n");

  Serial.print("count_history_mic_one:");
  Serial.println(count_history_mic_one);
  Serial.print("count_history_mic_two:");
  Serial.println(count_history_mic_two);

  //initializing variables
  float mic_one_average_frequency = 0;
  float mic_two_average_frequency = 0;
  int mic_one_average_mag = 0;
  int mic_two_average_mag = 0;

  if (count_history_mic_one != 0) { //this will prevent any potential IntegerDivideByZero errors
    mic_one_average_frequency = MIC_ONE_freq_history / (float)count_history_mic_one;
    mic_one_average_mag = MIC_ONE_mag_history / count_history_mic_one;
  }

  if (count_history_mic_two != 0) {  //this will prevent any potential IntegerDivideByZero errors
    mic_two_average_frequency = MIC_TWO_freq_history / (float)count_history_mic_two;
    mic_two_average_mag = MIC_TWO_mag_history / count_history_mic_two;
  }


  Serial.println("--------------------------------------------------------");
  Serial.print(mic_one_average_frequency);
  Serial.println(" Hz: MIC ONE average frequency");
  Serial.print(mic_two_average_frequency);
  Serial.println(" Hz: MIC TWO average frequency");
  Serial.print(mic_one_average_mag);
  Serial.println(" : MIC ONE average magnitude");
  Serial.print(mic_two_average_mag);
  Serial.println(" : MIC TWO average magnitude");
  Serial.println("--------------------------------------------------------\n");


  if (mic_one_average_frequency > 1200 || mic_two_average_frequency > 1200)  //whistle frequency response
  {
    //if magitudes are within 85% of each other and there are equal number of history values collected
    if (((max(mic_one_average_mag, mic_two_average_mag) * .85) < min(mic_one_average_mag, mic_two_average_mag))
        && (count_history_mic_one == count_history_mic_two)) {
      moveForward(127);  //moving forward at 1/2-full speed
      Serial.print("Both are close in magnitude! MIC ONE: ");
      Serial.print(mic_one_average_mag);
      Serial.print(" and MIC TWO: ");
      Serial.println(mic_two_average_mag);
      delay(25);
    } else if ((mic_one_average_mag > mic_two_average_mag) && (count_history_mic_one >= count_history_mic_two))  //TURN LEFT
    {                                                                                                            // only if magnitude of sound is larger at mic one and there are more than or the same number of history counts for mic one
      Serial.println("CLOSER to MIC ONE:");
      turnRight(127);  //turn right at 1/2-full speed
      delay(25);
    } else if ((mic_one_average_mag < mic_two_average_mag) && (count_history_mic_one <= count_history_mic_two))  //(Turn RIGHT)
    {                                                                                                            // only if magnitude of sound is larger at mic two and there are more than or the same number of history counts for mic two
      Serial.println("CLOSER to MIC TWO:");
      turnLeft(127);  //turn left at 1/2-full speed
      delay(25);
    } else {  //if there is discrepancy between magnitudes and history counts, then this indicates error/heavy echo. So no motor actuation.
      Serial.println("CONFLICTING SOUND DATA...\nAdjust distance/angle to microphone(s) and try whistling again.\n");
    }
    // Some more statistics printed out
    //float ratio = min(mic_one_average_mag, mic_two_average_mag) / (float)max(mic_one_average_mag, mic_two_average_mag);
    //Serial.print("Low mic magnitude is within **");
    //Serial.print(ratio * 100);
    //Serial.println("\%** of larger mic mag.");
  }
  else{ //moving backwards upon low-frequency whistle (below 1200 Hz)
    Serial.println("Low-frequency whistle detected.");
    moveBackwards(127);  //turn right at 1/2-full speed
    delay(25);
  }

  delay(25);
  resetHistoryVariables();
}

void resetHistoryVariables() {
  MIC_ONE_freq_history = 0;
  MIC_ONE_mag_history = 0;
  count_history_mic_one = 0;
  MIC_TWO_freq_history = 0;
  MIC_TWO_mag_history = 0;
  count_history_mic_two = 0;
  threshold_count = 0;

  Serial.println("\n*************************************");
  Serial.println("*****RESETTING HISTORY VARIABLES*****");
  Serial.println("*************************************\n");

  stopMovement();
}

//Motor functions
void moveForward(int dutycycle) {
  Serial.print("Moving Forward at ");
  Serial.print(dutycycle);
  Serial.println(" speed.");
  analogWrite(enableAPin, dutycycle);
  analogWrite(enableBPin, dutycycle);
  digitalWrite(motorAPin1, LOW);
  digitalWrite(motorAPin2, HIGH);
  digitalWrite(motorBPin1, LOW);
  digitalWrite(motorBPin2, HIGH);
}

void moveBackwards(int dutycycle) {
  Serial.print("Moving Backwards at ");
  Serial.print(dutycycle);
  Serial.println(" speed.");
  analogWrite(enableAPin, dutycycle);
  analogWrite(enableBPin, dutycycle);
  digitalWrite(motorAPin1, HIGH);
  digitalWrite(motorAPin2, LOW);
  digitalWrite(motorBPin1, HIGH);
  digitalWrite(motorBPin2, LOW);
}

void turnLeft(int dutycycle) {
  Serial.print("Turning Left at ");
  Serial.print(dutycycle);
  Serial.println(" speed.");
  analogWrite(enableAPin, dutycycle);
  analogWrite(enableBPin, dutycycle);
  digitalWrite(motorAPin1, LOW);
  digitalWrite(motorAPin2, LOW);
  digitalWrite(motorBPin1, LOW);
  digitalWrite(motorBPin2, HIGH);
}

void turnRight(int dutycycle) {
  Serial.print("Turning Right at ");
  Serial.print(dutycycle);
  Serial.println(" speed.");
  analogWrite(enableAPin, dutycycle);
  analogWrite(enableBPin, dutycycle);
  digitalWrite(motorAPin1, LOW);
  digitalWrite(motorAPin2, HIGH);
  digitalWrite(motorBPin1, LOW);
  digitalWrite(motorBPin2, LOW);
}

void stopMovement() {
  Serial.println("\n||||||||||||Stopping All Motors...||||||||||||\n");
  digitalWrite(motorAPin1, LOW);
  digitalWrite(motorAPin2, LOW);
  digitalWrite(motorBPin1, LOW);
  digitalWrite(motorBPin2, LOW);
}

//simple min and max functions
int max(int num1, int num2) {
  int result = 0;
  if (num1 > num2) return num1;
  if (num2 > num1) return num2;
  if (num1 == num2) return result;
}

int min(int num1, int num2) {
  int result = 0;
  if (num1 < num2) return num1;
  if (num2 < num1) return num2;
  if (num1 == num2) return result;
}