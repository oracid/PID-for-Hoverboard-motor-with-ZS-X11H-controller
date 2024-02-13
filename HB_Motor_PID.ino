// HB_Motor_PID - with PWM and ZS-X11H controller - 10/02/2024
int pb = A0;                                // Start/Stop pgm button
int DIR_PIN_BT = A2;                        // Direction button 
int DIR_PIN_OUT = 10;                       // Direction Output
int HALL_PIN = 2;                           // interrupt feedback pin 
int PWM_PIN_OUT = 3;                        // 490Hz PWM Output

volatile float tic, tac, feedback=0.00;     // interrupt variables
bool dir=0;                                 // direction, 0=clockwise, 1=counterclockwise
float now, prvTime, dt;                     // time variables
float P=0.00, I=0.00, D=0.00, error=0.00, errDiff=0.00, prevErr=0.00, maxSum=50, errSum=0.00, pid=0.00;  // PID variables

float kp=0.15, ki=0.7, kd=0.001, target=15, trgt_min=12, trgt_max=25, fb_min=104, fb_max=46; // variables to be modified

void setup() {
  Serial.begin(115200);
  pinMode(pb,INPUT_PULLUP);                                               // Start pgm button
  pinMode(DIR_PIN_BT,INPUT_PULLUP);                                       // Direction button
  pinMode(DIR_PIN_OUT, OUTPUT);                                           // Direction Output
  pinMode(PWM_PIN_OUT, OUTPUT);                                           // 490Hz PWM Output
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), intrupt, RISING);      // Attach Interrupt for motor Hall sensors
  Serial.print("\n\t To start pgm, click on the Start button"); while(digitalRead(pb)); delay(400); Serial.print("\n\t Started"); // start button
}

void loop() {
  now = millis(); dt = (now - prvTime) /1000.00; prvTime = now;             // time between two loops
  if( ! digitalRead(DIR_PIN_BT)){ dir = dir ^ 1; delay(300); analogWrite(PWM_PIN_OUT,0); digitalWrite(DIR_PIN_OUT, dir); } // change direction when button pushed
  pid = PID();                                                              // PID calculation
  analogWrite(PWM_PIN_OUT, round(pid = constrain(pid,trgt_min,trgt_max)) ); // output PWM PID - constrain speed for security
  Plotter();                                                                // values for the plotter
  //Trace();                                                                  // print variables
}

void intrupt(){
  tic = millis();
  feedback = tic - tac;  tac = tic;                                 // time between 2 Hall sensor detections
  feedback = map( feedback, fb_min, fb_max, trgt_min, trgt_max );   // convert feedback milliseconds to PWM value
}

float PID(){
  noInterrupts();  error = target - feedback;  interrupts();
  P = kp * error;
  I = ki * (errSum = errSum + (error * dt)); errSum = constrain( errSum, -maxSum, maxSum );
  D = kd * (error - prevErr) / dt;  prevErr = error;
  return P + I + D;
}

void Plotter(){
  Serial.print(0);            Serial.print("  ");                         // to limit plotter scale
  Serial.print(feedback,3);   Serial.print("  ");
  Serial.print(pid,3);        Serial.print("  ");
  Serial.println(trgt_max,0);                                             // to limit plotter scale
}

void Trace(){
  Serial.print(String() + "\n" 
                        + "  target: "    + target
                        + "  feedback: "  + String(feedback,3)
                        + "  pid: "       + String(pid,3)
                        + "  error: "     + String(error,3)
                        + "  prevErr: "   + String(prevErr,3)
                        + "  errSum: "    + String(errSum,3)
                        + "  P: "         + String(P,3)
                        + "  I: "         + String(I,3)
                        + "  D: "         + String(D,3)
                        + "  dir: "       + dir
                        + "  dt: "        + String(dt,3)
                        );
}
