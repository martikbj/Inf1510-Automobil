Kode: 
//koden for aa faa selve GPSen til aa fungere ble tatt fra eksemplene til Adafruit
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(8, 7);
int poeng;
int fartsgrense;
int knappTrykket;

Adafruit_GPS GPS(&mySerial);

#define GPSECHO  true

boolean usingInterrupt = false;
void useInterrupt(boolean); 

void setup()  
{
  fartsgrense = 0;
 
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");
  
  GPS.begin(9600);
  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   
  
  GPS.sendCommand(PGCMD_ANTENNA);

  useInterrupt(true);
  int poeng = 0;
  
  //Roede lyset
  pinMode(9, OUTPUT);
  //Groenne lyset
  pinMode(4, OUTPUT);
  //Piezo
  pinMode(3, OUTPUT);
  //Knapp
  pinMode(12, INPUT);
    
  mySerial.println(PMTK_Q_RELEASE);
}

//Fra Adafruits kodeeksempel
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    
#endif
}

//Fra Adafruit kodeeksempel
void useInterrupt(boolean v) {
  if (v) {
    
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
uint32_t timer4 = millis();
uint32_t timer3 = millis();
uint32_t timer2 = millis();
uint32_t timer = millis();

void loop() {
  
  //Lysene blinker for aa indikere at produktet et slått på/av med knappen
  if(digitalRead(12) == HIGH && knappTrykket < 2){
    knappTrykket++;
    blink();
  }

  //Fra Adafruit
  if (! usingInterrupt) {
    
    char c = GPS.read();
    
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  //Fra Adafruit
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   
      return;  
  }
   
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // Skriver info til serialmonitor hvert sekund, men begynner ikke aa kjore for knappen har blitt trykket minst en gang
  if (millis() - timer > 1000 && knappTrykket >= 1) { 
    timer = millis(); // reset the timer
    
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    digitalWrite(9, HIGH);
    digitalWrite(4, HIGH);
    
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees, 4);
      innenRadius(GPS.longitudeDegrees, GPS.latitudeDegrees);
      Serial.print("Speed (kmph): "); Serial.println(GPS.speed * 1.865);
      Serial.print("Poeng: "); Serial.println(poeng);
      
        if(GPS.speed * 1.865 > hentFartsgrense()){ //sjekker om man kjorer over fartsgrensen og skrur paa det rode lyset saa lenge man gjor det
         timer2 = millis();
         digitalWrite(9, HIGH);
         digitalWrite(4, LOW);
          
           if(millis() - timer3 > 60000){ //dersom man kjorer over fartsgrensen i ett minutt sammenhengende vil det bli spilt av en lyd som advarer at man kjorer over fartsgrensen
             tone(3, 200, 400);
             timer3 = millis();
             digitalWrite(9, HIGH);
             digitalWrite(4, LOW);  
           }
        }
        else if(GPS.speed * 1.865 <= 30){ //dersom man kjorer opptil 30 km i timen skal ikke arduinoen telle opp noen av timerene
           digitalWrite(9, HIGH);
           digitalWrite(4, HIGH);
           timer2 = millis();
           timer3 = millis();      
        } 
            
        else{ //dersom man kjorer under fartsgrensen, men over 30 km i timen vil det gronne lyset vaere paa saa lenge man gjor det
          digitalWrite(4, HIGH);
          digitalWrite(9, LOW);
          timer3 = millis();
         
          if(millis() - timer2 > 30000){ //dersom man kjorer over 30 km i timen og under fartsgrensen i 30 sek vil man bli gitt ett poeng og en lyd vil bli spilt av for aa si ifra om dette
           poeng++;
           tone(3, 200, 100);
           tone(3, 600, 150);
           digitalWrite(9, LOW);
           digitalWrite(4, HIGH);
           if(poeng == 20){ //dersom man faar 20 poeng vil en melodi bli spilt av
             spillMelodi();
           }
           timer2 = timer2 + 14000;    
          }   
        }       
    }
  } 
} //slutt paa loop
  
void innenRadius(float lengde, float bredde){
 //sjekker om bilen befinner seg innenfor gitte koordinater. Dersom den gjor det blir fartsgrensen satt til det som gjelder i omraadet, eller hvis den er utenfor vil den beholde den forrige fartsgrensen og bruke den 
      //hvis ikke bredden og lengden ganges opp med 10000 saa fjernes noen av tallene etter komma, og radiusen blir for stor
      Serial.print("BREDDE: ");
      Serial.println(bredde, 4);
      float nyBredde = bredde*10000;
      Serial.print("NY BREDDE VERDI: ");
      Serial.println(nyBredde);
      Serial.print("LENGDE: ");
      Serial.println(lengde, 4);
      float nyLengde = lengde*10000;
      Serial.print("NY LENGDE VERDI: ");
      Serial.println(nyLengde);
      
    /* Testkoordinater  
   skjære i gjerde
   if ((nyBredde >= 599350 - 4 && nyBredde <= 599350 + 4) && (nyLengde >= 106868 - 4 && nyLengde <= 106868 + 4)) {
    Serial.println("Skjære i gjerdet - Fartsgrensen er naa 20 km i timen");
     fartsgrense = 20;
  }
  der per bor
  if ((nyBredde >= 599358 - 4 && nyBredde <= 599358 + 4) && (nyLengde >= 106887 - 4 && nyLengde <= 106887 + 4)) {
    Serial.println("Her bor Per - Fartsgrensen er naa 30 km i timen");
     fartsgrense = 30;
  }
   der elisabeth bor
  if ((nyBredde >= 599355 - 4 && nyBredde <= 599355 + 4) && (nyLengde >= 106906 - 4 && nyLengde <= 106906 + 4)) {
    Serial.println("Her bor Elisabeth - Fartsgrensen er naa 20 km i timen");
     fartsgrense = 20;
  }
  Annikens park
  if ((nyBredde >= 599341 - 4 && nyBredde <= 599341 + 4) && (nyLengde >= 106892 - 4 && nyLengde <= 106892 + 4)) {
    Serial.println("Annikens park - Fartsgrensen er naa 30 km i timen");
     fartsgrense = 30;
  }
  Bianca
  if ((nyBredde >= 599337 - 4 && nyBredde <= 599337 + 4) && (nyLengde >= 106861 - 4 && nyLengde <= 106861 + 4)) {
    Serial.println("Bianca - Fartsgrensen er naa 20 km i timen");
     fartsgrense = 20;
  } 
  Svalbardveien
  if((nyBredde >= 599351 - 4 && nyBredde <= 599351 + 4) && (nyLengde >= 106940 - 4 && nyLengde <= 106940 + 4)) {
    Serial.println("Svalbardveien - Fartsgrensen er naa 20 km i timen");
    fartsgrense = 20;
  }
  Tuengen alle
  if((nyBredde >= 599380 - 4 && nyBredde <= 599380 + 4) && (nyLengde >= 106937 - 4 && nyLengde <= 106937 + 4)) {
    Serial.println("Tuengen alle - Fartsgrensen er naa 35 km i timen");
    fartsgrense = 35;
  }
  Kinesisk ambassade
  if((nyBredde >= 599415 - 4 && nyBredde <= 599415 + 4) && (nyLengde >= 107012 - 4 && nyLengde <= 107012 + 4)) {
    Serial.println("Kinas ambassade - Fartsgrensen er naa 20 km i timen");
    fartsgrense = 20;
  }
  Kryss
  if((nyBredde >= 599392 - 4 && nyBredde <= 599392 + 4) && (nyLengde >= 106948 - 4 && nyLengde <= 106948 + 4)) {
    Serial.println("Kryss - Fartsgrensen er naa 30 km i timen");
    fartsgrense = 30;
  }
  */
  
  //Koordinater brukt paa sluttevalueringen med bruker i bil
  //Kjorer inn paa Sorkedalsveien
  if((nyBredde >= 599350 - 4 && nyBredde <= 599350 + 4) && (nyLengde >= 106938 - 4 && nyLengde <= 106938 + 4)) {
    Serial.println("Kjorer inn paa Sorkedalsveien - Fartsgrensen er naa 60 km i timen");
    fartsgrense = 60;
  }
  
  //Smestad
  if((nyBredde >= 599378 - 4 && nyBredde <= 599378 + 4) && (nyLengde >= 106824 - 4 && nyLengde <= 106824 + 4)) {
    Serial.println("Kjorer forbi Smestad - Fartsgrensen er naa 50 km i timen");
    fartsgrense = 50;
  }
  
  if((nyBredde >= 599455 - 4 && nyBredde <= 599455 + 4) && (nyLengde >= 105929 - 4 && nyLengde <= 105929 + 4)) {
    Serial.println("Fartsgrensen er naa 70 km i timen");
    fartsgrense = 70;
  }
  
  if((nyBredde >= 599401 - 4 && nyBredde <= 599401 + 4) && (nyLengde >= 105787 - 4 && nyLengde <= 105787 + 4)) {
    Serial.println("Fartsgrensen er naa 60 km i timen");
    fartsgrense = 60;
  }
  
  if((nyBredde >= 599276 - 4 && nyBredde <= 599276 + 4) && (nyLengde >= 105684 - 4 && nyLengde <= 105684 + 4)) {
    Serial.println("Fartsgrense er naa 50 km i timen");
    fartsgrense = 50;
  }
  
  else{
   Serial.println("du er utenfor.");
  }
} //slutt paa innenRadius

int hentFartsgrense(){ //henter den gjeldende fartsgrensen
  return fartsgrense;
} //slutt paa hentFartsgrense

void spillMelodi(){ //spiller av melodien som signaliserer at man har oppnaadd 20 poeng
  tone(3, 600, 150);
   delay(150);
   tone(3, 800, 150);
   delay(150);
   tone(3, 900, 150);
} //slutt paa spillMelodi

void blink(){ //faar lysene til aa blinke en gang
  digitalWrite(4, HIGH);
  digitalWrite(9, LOW);
  delay(1000);
  
  digitalWrite(9, HIGH);
  digitalWrite(4, LOW);
  delay(1000);
  
  digitalWrite(9, LOW);
} //slutt paa blink
 //kode slutt
