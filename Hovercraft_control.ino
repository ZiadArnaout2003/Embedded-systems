#include <Wire.h> // really hate using this but the C library is not working. 


#define F_CPU 16000000UL
#define BAUD 9600UL
#define UBRR ((F_CPU)/((BAUD)*(16UL))-1)
#define IMU 0b1101000 // PMU-6050 TWI Address
// frequency 16000000/(8*(40000-1)) = 50Hz
#define PWM_TOP 40000
// Gyro Config 
// 250,   500  ,  1000  , 2000 deg/s
// 000    001      010     011 config
// 131    65.5    32.8    16.4 comp
// Accel Config
// 2g     4g      8g      16g  g-force
// 000    001     010     011  config
//16,384  8192    4096    2048 comp
//
#define GYRO_CONFIG 0b00000000
#define GYRO_COMPEN 121.1 // down from 131.1, seems to work better, might be a timing issue. 
#define ACEL_CONFIG 0b00000000
#define ACEL_COMPEN 16384.0
#define PROCESS_TIME 8 //ms

static volatile struct{
  long X;
  long Y;
  long Z;
} accel;

static volatile struct{
  long X;
  long Y;
  long Z;
} gyro;

static volatile struct{
  float X;
  float Y;
  float Z;
} gForce;

static volatile struct{
  float X;
  float Y;
  float Z;
} rot;

uint8_t led_d3 = PB3;
uint8_t led_L  = PB5;

float yawAngle = 0;  // Yaw angle
int brightness = 0;
uint16_t servoPos = PWM_TOP/13.5;

float distance;
float prev_gforce_x = 0; // For X-axis
float velocity_x = 0, distance_x = 0;
uint8_t overslows = 0;
int microseconds = 0;
int last_micros  = 0;

void setup() {
  
  init_UART(UBRR);
  init_PORTS();
  setup_IMU();
  _delay_ms(1000);
  init_timers();
  //sei();
} 

void loop() {
  poll_accel();
  poll_gyro();
  _delay_ms(PROCESS_TIME);
  int microsec = micros();
  microseconds = microsec-last_micros; // time since last poll
  last_micros = microsec; // storage 
  yawAngle += rot.Z*microseconds/1000000; // yaw integrates cumulative rotational moment
  yawAngle = constrain(yawAngle, -180.0, 180.0);
  // Map the yaw angle to servo position
  servoPos = (uint16_t) map(constrain(yawAngle,-80.0, 80.0), -90, 90, PWM_TOP/8.8, PWM_TOP/30.0); // Adjust the mapping as needed
  servoPos = (uint16_t) constrain(servoPos, PWM_TOP/30.0, PWM_TOP/8.8); // Constrain to valid servo range
  //servoPos = (uint16_t) PWM_TOP/13.5; /// midpoint
  if(abs(yawAngle)>85) PORTB|=(1<<led_L);
  else PORTB&=~(1<<led_L);
  //OCR0A = servoPos;
//===============================================//
// IF YOU'RE READING THIS, THANKS FOR DOING THAT. 
// THIS IS THE PROBLEMATIC CODE: WE ARE TRYING TO DO NUMERICAL INTEGRATION BASED ON ACCELEROMETER DATA ONLY.
// THIS IS NEVER DONE IN PRACTICE IN INDUSTRY. IT IS BAD AND PRONE TO HELL OF ERRORS. THIS IS SIMILAR TO "DEAD RECKONING": NAVIGATION WITHOUT REFERENCE.
// TRY TO COME UP WITH A FUNCTION THAT NORMALIZES X G-FORCE AND ELIMINATES THE STRAY COMPONENTS OF GRAVITY DUE TO UNIT TILTING. 
// THEN NUMERICALLY INTEGRATE ACCELERATION INTO VELOCITY, AND VELOCITY INTO DISPLACEMENT. POSITION IS THE SUM OF DISPLACEMENTS. 
  velocity_x = (float) 9.8* microseconds*(prev_gforce_x + gForce.X)/2000000; // velocity out
  prev_gforce_x = gForce.X; // store last value 

  distance = velocity_x*microseconds/10000; // displacement = velocity * time. 10,000 = 1,000,000 microseconds / 100 cm/metre
  distance_x += distance;  // distance = position + displacement.

// THIS PART WORKS FINE IT'S WHATEVER. JUST FOCUS ON THE ABOVE SNIPET. 
  float gForceX_new= abs(gForce.X) * 100.0;
  brightness = map(gForceX_new, 8, 108, 0, 255); // 255 is off and 0 is on
  brightness = constrain(brightness-1, 0, 255); // sanity
  if(brightness <1 ) DDRB&=~(1<<led_d3);
  else DDRB|=(1<<led_d3);
  //OCR2A = brightness; this happens in OVF2 now 

  if(overslows >= 200){ // adjust value for timing 
    print_data();
    overslows = 0;
  }
 // print_data();
}

void poll_accel(){
  Wire.beginTransmission(0b1101000);
  Wire.write(0x3B); // to start register for Accel Readings
  Wire.endTransmission();

  Wire.requestFrom(0b1101000,6); //Request Accel Registers (which are 6) for each axis
  while (Wire.available()<6);
  accel.X = Wire.read()<<8|Wire.read(); //Store first two bytes into X 
  //the first Wire.read() reads byte 1, second Wire.read() reads byte 2
  accel.Y = Wire.read()<<8|Wire.read(); //Store middle two bytes into Y
  //the first Wire.read() reads byte 3, second Wire.read() reads byte 4
  accel.Z = Wire.read()<<8|Wire.read(); //Store last two bytes into Z
  //the first Wire.read() reads byte 5, second Wire.read() reads byte 67
  process_accel();

  }

  void process_accel(){
    gForce.X = accel.X/ACEL_COMPEN; 
    // this division is to help convert the reading of accel.X into something
    // more meaningful (in g)
    gForce.Y = accel.Y/ACEL_COMPEN;
    gForce.Z = accel.Z/ACEL_COMPEN;

  }

  void poll_gyro(){
    Wire.beginTransmission(0b1101000);
    Wire.write(0x43);   // starting register for Gyro Readings
    Wire.endTransmission();
    Wire.requestFrom(0b1101000, 6); //request gyro registers, same as for accel registers( we have 6 registers)

    while(Wire.available()<6);
    gyro.X = Wire.read()<<8|Wire.read(); // store first 2 bytes
    gyro.Y = Wire.read()<<8|Wire.read(); // store middle 2 bytes
    gyro.Z = Wire.read()<<8|Wire.read(); // store last 2 bytes
    process_gyro();

  }

void process_gyro(){
  rot.X = gyro.X / GYRO_COMPEN; // 131.0 is the respective gyro sensitivity respective to +- 250 deg/sec
  rot.Y = gyro.Y / GYRO_COMPEN; // gyro is divided by that to get a reading in deg/sec
  rot.Z = gyro.Z / GYRO_COMPEN;
}

void print_data(){
  uart_println("\n----------------------------");
  uart_print(" YAW=");
  uart_print_float(yawAngle);
  uart_println(" deg");
  uart_print("Filtered acceleration: ");
  uart_print_float(velocity_x);

  uart_print("\n X=");
  uart_print_float(rot.X);
  uart_print(" deg");
  uart_print(" X=");
  uart_print_float(gForce.X);
  uart_println(" g");
  uart_print(" Y=");
  uart_print_float(rot.Y);
  uart_print(" deg");
  uart_print(" Y=");
  uart_print_float(gForce.Y);
  uart_println(" g");
  uart_print(" Z=");
  uart_print_float(rot.Z);
  uart_print(" deg");
  uart_print(" Z=");
  uart_print_float(gForce.Z);
  uart_println(" g");
  uart_print(" Servo PWM Value= ");
  uart_print_float(OCR1A);
  uart_print("\nDistance: ");
  uart_print_float(distance_x);
  uart_println(" cm");

}


void Read_Reg_N(uint8_t TWI_addr, uint8_t reg_addr, uint8_t bytes, int16_t* data){ 
	
	uint8_t *p_data=(uint8_t*)data;
  Wire.beginTransmission(TWI_addr);
  Wire.write(reg_addr); // set register to request from
  Wire.endTransmission();

  Wire.requestFrom(TWI_addr, bytes); // request data
  while(Wire.available() < bytes); // wait for all bytes to be queued.
  for (uint8_t i=0; i<bytes-1; i++) {
		*p_data=Wire.read();
		p_data++;	
	}
}

void Write_Reg(uint8_t TWI_addr, uint8_t reg_addr, uint8_t value){
  //this is ugly but im trying to get a working product here. 
  Wire.beginTransmission(TWI_addr);
  Wire.write(reg_addr);
  Wire.write(value);
  Wire.endTransmission();
 }

void uart_transmit(unsigned char data) {
  // Wait for empty transmit buffer
  while (!(UCSR0A & (1 << UDRE0)));
  // Put data into buffer, sends the data
  UDR0 = data;
}
 
void uart_print(const char *str) {
  while (*str) {
    uart_transmit(*str++);
  }
}
 
void uart_println(const char *str) {
  uart_print(str);
  uart_transmit('\r');
  uart_transmit('\n');
}
 
void uart_print_float(float value) {
  char buffer[20];
  dtostrf(value, 6, 2, buffer);  // Convert float to string
  uart_print(buffer);
}

ISR (TIMER2_OVF_vect){//timer 2 overflow
  overslows++; //serial out time
  OCR2A = brightness;
}

ISR (TIMER1_OVF_vect){//timer 2 overflow
  OCR1A = (servoPos);
}


/* _____ _   _ _____ _______ 
|_   _| \ | |_   _|__   __|
  | | |  \| | | |    | |   
  | | | . ` | | |    | |   
 _| |_| |\  |_| |_   | |   
|_____|_| \_|_____|  |_|   */
void setup_IMU(){

  uart_println("IMU SETUP: ");
  Wire.begin();
  uart_println("WIRE BEGIN");
  Write_Reg(IMU, 0x6B, 0b00000000);//0x6b: Power management, Value 0 for sleep.
  Write_Reg(IMU, 0x1B, GYRO_CONFIG);//0x1B: gyro config, Value 0 for +/- 250 deg.
  Write_Reg(IMU, 0x1C, ACEL_CONFIG);//0x1C: accel config, Value 0 for +/- 2g
  uart_println("REGISTERS SET");

}

void init_UART(uint8_t ubrr){
UBRR0H = (uint8_t)((ubrr)>>8);
UBRR0L = (uint8_t)ubrr;
UCSR0B|=(1<<RXEN0); // enable receiver // (1<<RXCIE0) and its interrupt
UCSR0B|=(1<<TXEN0); // enable transmitter // (1<<TXCIE0) and its interrupt
UCSR0C= (1 << UCSZ01)|(1 << UCSZ00); // 8-bit, 1 stop, no parity
}

void init_PORTS(){
  DDRB|=(1<<led_d3); // set D3 output
  PORTB|=(1<<led_d3); // turn it off
  DDRB|=(1<<led_L);  // set L output
  DDRB|=(1<<PB1);  // servo output (will be PWM)
  DDRD|=(1<<PD6)|(1<<PD5); //servo OC0A
}

void init_timers(){
  // timer 2: controls LED PWM and Data Output
  TCCR2B=(1<<CS22)|(1<<CS21); // timer control register2, prescaler 
  TIMSK2|=(1<<TOIE2); //Timer/Counter2 Interrupt Flag Register - set to interrupt on matching OCR2A and Overflow. 
  TCCR2A=(1<<COM2A1)|(1<<COM2A0)|(1<<WGM21)|(1<<WGM20);// Fast PWM inverting mode
  //OCR2A=20;///test


  TIMSK1|=(1<<TOIE1); // overflow interupt enable 
  // Clear OC1A on match, set on bottom
  TCCR1A |= (1<<COM1A1);
  TCCR1A &=~(1<<COM1A0);
  // Fast PWM Mode 14 ICR1=TOP
  TCCR1B |= (1<<WGM13)|(1<<WGM12);
  TCCR1A |= (1<<WGM11);
  TCCR1A &=~(1<<WGM10);
  
  ICR1  = PWM_TOP;
  // Prescaler to CLKio/8
  TCCR1B &=~(1<<CS12);
  TCCR1B |= (1<<CS11);
  TCCR1B &=~(1<<CS10);

 uart_print("TIMERS SET");
}




