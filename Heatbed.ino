#include <LiquidCrystal.h>
#define D8 8 //ohearen output 
#define TEMP_SENSOR_BED 14 //termistorea input

#define  Rc  100000 //valor de la resistencia 
const float A = 1.11492089e-3;    const float B = 2.372075385e-4;   const float C = 6.954079529e-8;
const float K = 2.5; //factor de disipacion en mW/C


LiquidCrystal lcd( 16, 17, 23, 25, 27, 29);  // Para MEGA + RAMPS 1.4

void setup() {
  Serial.begin (9600);
  Serial.println("Arduino ready");
  lcd.begin(20, 4); //(column, row)
  setPins();
  //EncKonfig_1();//Enc(); 

  int value = map(12,0,12,0,255);
  analogWrite(D8,value);
}

void loop() {

  c_kalkulatu();
  //c_kalk1();

}




void c_kalkulatu(){
  float Vcc = readVcc() / 1000.0; // leer el voltaje que llega al arduino
  float raw =  analogRead(TEMP_SENSOR_BED);
  float V = raw / 1023 * Vcc;
  //Vout = Vin * (R / R+Rt)
  float R = (Rc * V ) / (Vcc - V); //bertsio honetarako ez du balio, Vcc -/\/\/R---V----/\/\/-Rt--
  //float R = (Rc * Vcc / V) - Rc;
  float logR  = log(R);
  float R_th = 1.0 / (A + B * logR + C * pow(logR,3));
  float kelvin = R_th - V * V / (K * R) * 1000;
  float celsius = kelvin - 273.15;

 // Serial.print(raw/1023*5);
  Serial.print("R: ");
  Serial.print(R);
  Serial.print(' ');

Serial.print("V: ");
  Serial.print(V);
  Serial.print(' ');

  Serial.print("Vcc: ");
  Serial.print(Vcc);
  Serial.print(' ');

  Serial.print("Celsius: ");
  Serial.print(celsius);
  Serial.println(' ');
  //  Serial.println(V);




}


long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  //ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  ADMUX = bit (REFS0) | bit (REFS1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#elif defined (__AVR_ATmega328P__) //Arduino genuino UNO-rentzat
  ADMUX = bit (REFS0) | bit (REFS1);  // Internal 1.1V reference - kode berria
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1); kode zaharra
#endif
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
    //result = 1126400L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1024*1000
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000   
  //Serial.println(result);
  return result;
}


void setPins(){
  pinMode(D8, OUTPUT);
  pinMode(TEMP_SENSOR_BED, INPUT);
}
