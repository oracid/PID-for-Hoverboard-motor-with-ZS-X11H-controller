// MinMax - 10/02/2024 - reference version
int HALL_PIN = 2;                     // interrupt feedback pin 
int PWM_PIN_OUT = 3;                  // 490Hz PWM Output
int POT_PIN_IN = A7;                  // Analog potentiomer for speed

volatile float feedback=0.00, tic, tac;                                 // interrupt function variables
float  pot=0, target, trgt_min=0, trgt_max=255, fb_min=0, fb_max=0;     // target_PWM value - min and max values between target_PWM and feedback

void setup() {
  Serial.begin(115200);                                                 // serial monitor
  pinMode(PWM_PIN_OUT, OUTPUT);                                         // 490Hz PWM Output
  pinMode(POT_PIN_IN, INPUT);                                           // potentiometer
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), intrupt, RISING);    // Attach Interrupt for motor Hall sensors
}

void loop() {
  pot = analogRead(POT_PIN_IN);                     // throttle value, determined by potentiometer
  target = map(pot,0,1023,trgt_min,trgt_max);       // convert throttle in PWM value
  analogWrite(PWM_PIN_OUT, target);                 // output PWM target
  Serial.print("\n  target: ");Serial.print(target,0); Serial.print("  feedback: ");Serial.print(feedback,0); delay(200);
}

void intrupt(){
  tic = millis();                                   // time
  feedback = tic - tac;                             // time between 2 tic
  tac = tic;                                        // save previous time
}
