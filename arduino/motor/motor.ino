int input_analogico = A1;
int analog_value = 0;
int duty_cycle;
int output_pwm = 11;
int top = 50; 
// pin: 11, 12 -> fPWM: 20 khz
// 50  -> fPWM: 20k
// 100 -> fPWM: 10k

void setup() { 
  pinMode(output_pwm, OUTPUT);
  TCCR1A = 0x00;
  TCCR1B = 0x12;
  ICR1 = top;
  Serial.begin(9600); 
} 

void loop() {   
  analog_value  =  analogRead(input_analogico);
  duty_cycle = map(analog_value, 0, 1023, 0, top);
  analogWrite(output_pwm, duty_cycle);
  // Print in serial port.
  Serial.print(analog_value);
  Serial.print(" - "); 
  Serial.println(duty_cycle);
}
