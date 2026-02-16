#include <Arduino.h>
#include <Wire.h>
#include <INA226_WE.h>

/*
*==========Arduino Nano pinout====== 
 *                            _______
 *                       TXD-|       |-Vin 
 *                       RXD-|       |-Gnd  
 *                       RST-|       |-RST
 *                       GND-|       |-+5V  
 *             BUTTON_IO  D2-|       |-A7  
 *                        D3-|       |-A6  
 *         LED_BUTTON_IO  D4-|       |-A5   
 *                        D5-|       |-A4   
 *                        D6-|       |-A3   
 *                        D7-|       |-A2   
 *                        D8-|       |-A1   
 *                        D9-|       |-A0   
 *                       D10-|       |-Ref
 *                  SSR  D11-|       |-3.3V   
 *                       D12-|       |-D13
 *                            --USB--        
 */
#define BAUDRATE (115200)
#define I2C_ADDRESS 0x40
INA226_WE ina226 = INA226_WE(I2C_ADDRESS);
////////////  I/O   ////////////
#define BUTTON_IO       2  
#define LED_BUTTON_IO   4
#define SSR             11  // 
#define MODE_IO         7  // Manual/AUTO 

////////Button ///////
bool buttonPressed = LOW;
bool check = LOW;
bool first_press = true;
bool first_led_on = true;
const int BOUNCE_TIME = 50;
const bool MANUAL = true;
bool first_session = true;
//////// SSR ///////
long timer_ssr_on = 0;
long timer_ssr_off = 0;
const uint32_t SSR_ON = 60000;
const long SSR_OFF = 10000;


///////// INA226 ///////////
const int ERROR_CURRENT_DELAY = 2000;
const float R_SHUNT_OHMS = 0.00215f;    // ex: 2 mΩ
const float I_RANGE_A    = 20.0f;      // ex: tu veux mesurer jusqu’à ~10 A
const float CURRENT_SYSTEM = 2.5;
int current_flag;
float current;

bool PRESS_BUTTON() {
  // Check if the button is pressed
  if (digitalRead(BUTTON_IO) == LOW && check == LOW) {
     //Serial.println("press :");
     check = HIGH;         // Mark that the button is being pressed
    delay(BOUNCE_TIME); // Apply debounce delay
  }

  // Check if the button is released
  if (digitalRead(BUTTON_IO) == HIGH && check == HIGH) {
    //Serial.println("unpress");
    check = LOW;  // Reset the state for the next button press
    return HIGH;  // Indicate that the button was successfully pressed and released
  }
  return LOW; // Return false if the button is not in the desired state
}





bool init_current_sensor(){
  Wire.begin();
  if(!ina226.init()){
     Serial.println("No communication with the current sensor");
     return false;
  }
  ina226.setResistorRange(R_SHUNT_OHMS,I_RANGE_A); // choose resistor 5 mOhm and gain range up to 10 A
  return true;
}

int current_valid(){
  ina226.readAndClearFlags();
  current = ina226.getCurrent_A();

  if (ina226.getI2cErrorCode() != 0) {
      return 0;
      delay(200);
    }
  
  if (current > CURRENT_SYSTEM){
    return 1;
  }
  return 2;
}

void reset_session(){
  Serial.println("\nCurrent : " + String(current) + "A");
  Serial.println("Voltage : " + String(ina226.getBusVoltage_V()) + "V\n");

  Serial.println("SSR OFF");
  first_press = true;
  first_led_on = true;
  timer_ssr_off = millis();
  digitalWrite(SSR,LOW);

}

void BLINK_LED_STATE(uint32_t DELAY){
  digitalWrite(LED_BUTTON_IO, HIGH);   // Turn on
  delay(DELAY);
  digitalWrite(LED_BUTTON_IO, LOW);   // Turn off activation LED
  delay(DELAY);
}

 

void setup() {
  pinMode(LED_BUTTON_IO, OUTPUT);       // Activation indicator LED
  pinMode(SSR, OUTPUT);                 // Activation SSR
  pinMode(BUTTON_IO, INPUT_PULLUP);     // Ignition button (active LOW)
  pinMode(MODE_IO, INPUT_PULLUP);     // Ignition button (active LOW)
  Serial.begin(BAUDRATE);               // Start serial communication
  delay(100); 

  if(!init_current_sensor()){
    Serial.println("reset the arduino");
    while(1);
  }
  
  Serial.println("init");  
}
  

void loop() {

  if(digitalRead(MODE_IO) == MANUAL){
    buttonPressed = PRESS_BUTTON();
  }
  else{
    buttonPressed = true;
  }

  

  

  if(millis() - timer_ssr_off > SSR_OFF || first_session){

    if(first_led_on){
      digitalWrite(LED_BUTTON_IO,HIGH);
      first_led_on = false;
      Serial.println("Led ON");
    }

    current_flag = current_valid();//check and print current
    if(current_flag != 2){
      reset_session();
      if(current_flag == 0){
        Serial.println("reset session because No communication with the current sensor");
        Serial.println("reset the arduino");
        while(1){
          BLINK_LED_STATE(ERROR_CURRENT_DELAY);
        }
      }
      if(current_flag == 1){
        Serial.println("reset session because over current");
      }     
    }

    if(buttonPressed && first_press){
      first_session = false;
      Serial.println("first press");
      Serial.println("SSR ON");

      digitalWrite(LED_BUTTON_IO,LOW);
      delay(10);
      digitalWrite(SSR,HIGH);
      delay(1000);
      Serial.println("\nCurrent : " + String(ina226.getCurrent_A()) + "A");
      Serial.println("Voltage : " + String(ina226.getBusVoltage_V()) + "V\n");
      first_press = LOW;
      timer_ssr_on = millis();
    }

    

    if(millis() - timer_ssr_on > SSR_ON && first_press == false){
      reset_session();
      
    }

  }

  
}
















