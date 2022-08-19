
#include <Wire.h>

//const byte PICO_I2C_ADDRESS = 0x55;
#define PICO_I2C_SDA  26
#define PICO_I2C_SCL  27
#define PICO_LED  25

byte volatile rx_flag = 0;
byte volatile tx_flag = 0;
uint8_t volatile ram_addr = 0;
uint8_t volatile ram[256];


void setup() {
  pinMode (PICO_LED, OUTPUT);
  //Wire.begin();
  Wire1.setSDA(PICO_I2C_SDA);
  Wire1.setSCL(PICO_I2C_SCL);
  
  Wire.begin(0x47);    // join i2c bus as slave
 // Wire1.onReceive(i2c_receive);     // i2c interrupt receive
 // Wire1.onRequest(i2c_transmit);    // i2c interrupt send
}

void loop() {
  Serial.println("sending");
//  if (rx_flag) {
//    rx_flag = 0;
//    digitalWrite (PICO_LED, HIGH);
//    delay(1);
//    digitalWrite (PICO_LED, LOW);
//  }
//
//  if (tx_flag) {
//    tx_flag = 0;
//    digitalWrite (PICO_LED, HIGH);
//    delay(1);
//    digitalWrite (PICO_LED, LOW);
//  }
//
//  delay(1);
}
//
//void i2c_receive(int bytes_count) {     // bytes_count gives number of bytes in rx buffer   
//  if (bytes_count == 0) {
//    return;
//  }
//  ram_addr = (uint8_t)Wire1.read();      // first byte is ram offset address (0-255)
//  for (byte i=1; i<bytes_count; i++) {
//    ram[ram_addr] = (uint8_t)Wire1.read();
//    ram_addr++;
//  } 
//  rx_flag = bytes_count;
//}
//
//void i2c_transmit() {
//  tx_flag = 1;
//  Wire1.write((uint32_t)ram[ram_addr]);
//  ram_addr++;
//}
