#include <Arduino.h>

#define SIGNAL 22

#define sensorVoltagePin 33

int freq = 50000;
int ledChannel1 = 0;
int resolution = 10;


unsigned long previousMillis = 0; // previous time
const unsigned long interval = 0.1; // interval in milliseconds

class PIController {
public:
    PIController(double kp, double ki, double setpoint)
        : kp_(kp), ki_(ki), setpoint_(setpoint), integral_(0.0), last_error_(0.0)
    {
    }

    double calculate(double input, double dt) {
        double error = setpoint_ - input;
        integral_ += error * dt;
        double derivative = (error - last_error_) / dt;
        last_error_ = error;
        return kp_ * error + ki_ * integral_;
    }

private:
    double kp_;
    double ki_;
    double setpoint_;
    double integral_;
    double last_error_;
};


void setup() {
  ledcSetup(ledChannel1, freq, resolution);//For example, if you set the resolution to 10 bits, the PWM signal can be divided into 1024 steps (2^10), which provides a high level of granularity and smoothness in the PWM signal.
  ledcAttachPin(SIGNAL, ledChannel1);
  pinMode(sensorVoltagePin, INPUT);
  Serial.begin(115200);
}

void loop() {


   PIController controller(220, 15, 9);

  unsigned long currentMillis = millis(); // current time
  
 // check if the interval has elapsed
  if (currentMillis - previousMillis >= interval) {

      int sensorValue = analogRead(sensorVoltagePin);
     // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
     float input = sensorValue * (3.3 / 4095.0)*11.6;
     
    // do something here
   /*  float input = analog_input * (5 / 1023.0)*10; */
    double output = controller.calculate(input, interval);
    previousMillis = currentMillis; // save the current time

     if(output<=0){

      output=0;


    };

     if(output>=1023){

      output=1023;


    };

    if(output>0&&output<1023){

       output = controller.calculate(input, interval);
       Serial.println(output);
    }; 

    ledcWrite(ledChannel1, output);
   
}

}
