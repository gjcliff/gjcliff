 /*
  * See documentation at https://nRF24.github.io/RF24
  * See License information at root directory of this library
  * Author: Brendan Doherty (2bndy5)
  */
  
 #include <SPI.h>
 #include "printf.h"
 #include "RF24.h"
  
 // instantiate an object for the nRF24L01 transceiver
 RF24 radio(9, 8); // using pin 7 for the CE pin, and pin 8 for the CSN pin
  
 // Let these addresses be used for the pair
 uint8_t address[][6] = {"1Node", "2Node"};
 // It is very helpful to think of an address as a path instead of as
 // an identifying device destination
  
 // to use different addresses on a pair of radios, we need a variable to
 // uniquely identify which address this radio will use to transmit
 bool radioNumber = 0; // 0 uses address[0] to transmit, 1 uses address[1] to transmit
  
 // Used to control whether this node is sending or receiving
 bool role = true;  // true = TX role, false = RX role
  
 // For this example, we'll be using a payload containing
 // a single float number that will be incremented
 // on every successful transmission
 float payload = 0.0;
  
 void setup() {
  
   Serial.begin(9600);
   // initialize the transceiver on the SPI bus
   while(!radio.begin()) {
     Serial.println(F("radio hardware is not responding!!"));
   }
  
   // print example's introductory prompt
   Serial.println(F("RF24/examples/GettingStarted"));
   Serial.print(F("radioNumber = "));
   Serial.println((int)radioNumber);
  
   // Set the PA Level low to try preventing power supply related problems
   // because these examples are likely run with nodes in close proximity to
   // each other.
   radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
  
   // save on transmission time by setting the radio to only transmit the
   // number of bytes we need to transmit a float
   radio.setPayloadSize(sizeof(payload)); // float datatype occupies 4 bytes
  
   // set the TX address of the RX node into the TX pipe
   radio.openWritingPipe(address[radioNumber]);     // always uses pipe 0
  
   // set the RX address of the TX node into a RX pipe
   radio.openReadingPipe(1, address[!radioNumber]); // using pipe 1
  
   // additional setup specific to the node's role
  
     radio.stopListening();  // put radio in TX mode
   
  
   // For debugging info
   printf_begin();             // needed only once for printing details
   // radio.printDetails();       // (smaller) function that prints raw register values
   //radio.printPrettyDetails(); // (larger) function that prints human readable data
  
 } // setup
  
 void loop() {
   // This device is a TX node

   if(Serial.available()>1){
    payload = Serial.parseFloat();
   }

   unsigned long start_timer = micros();                    // start the timer
   bool report = radio.write(&payload, sizeof(float));      // transmit & save the report
   unsigned long end_timer = micros();                      // end the timer

   if (report) {
     Serial.print(F("Transmission successful! "));          // payload was delivered
     Serial.print(F("Time to transmit = "));
     Serial.print(end_timer - start_timer);                 // print the timer result
     Serial.print(F(" us. Sent: "));
     Serial.println(payload);                               // print payload sent
     //payload += 0.01;                                       // increment float payload
   } else {
     Serial.println(F("Transmission failed or timed out")); // payload was not delivered
   }

   // to make this example readable in the serial monitor
   delay(1000);  // slow transmissions down by 1 second

 } // loop
