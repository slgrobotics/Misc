/*
 * See documentation at https://nRF24.github.io/RF24
 * See License information at root directory of this library
 * Author: Brendan Doherty (2bndy5)
 */

/**
 * A simple example of sending data from 1 nRF24L01 transceiver to another.
 *
 * This example was written to be used on 2 devices acting as "nodes".
 * Use the Serial Monitor to change each node's behavior.
 */
#include <SPI.h>
#include "printf.h"
#include "RF24.h"

#define CE_PIN 7
#define CSN_PIN 8

// instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);

// Let these addresses be used for the pair
uint8_t address[][6] = { "1Node", "2Node" };
// It is very helpful to think of an address as a path instead of as
// an identifying device destination

// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit
//bool radioNumber = 1;  // false uses address[0] (pipe 0) to transmit, true uses address[1] (pipe 1) to transmit

// Used to control whether this node is sending or receiving
bool role = false;  // true = TX role, false = RX role

void initRf24(bool isTx, bool radioNumber) {

  role = isTx;

  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
#ifdef TRACE
    Serial.println(F("radio hardware is not responding!!"));
#endif // TRACE
    //digitalWrite(GREEN_LED_PIN, LOW);
    //digitalWrite(RED_LED_PIN, HIGH);
    while (1) {}  // hold in infinite loop
  }

#ifdef TRACE
  Serial.print(F("radioNumber = "));
  Serial.println((int)radioNumber);
#endif // TRACE

  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
  //radio.setPALevel(RF24_PA_MAX);  // RF24_PA_MAX is default.

  // save on transmission time by setting the radio to only transmit the
  // number of bytes we need to transmit a float
  //radio.setPayloadSize(sizeof(payload));  // float datatype occupies 4 bytes
  radio.setPayloadSize(sizeof(controls));  // float datatype occupies 4 bytes

  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address[radioNumber]);  // always uses pipe 0

  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[!radioNumber]);  // using pipe 1

  // additional setup specific to the node's role
  if (role) {
    radio.stopListening();  // put radio in TX mode
  } else {
    radio.startListening();  // put radio in RX mode
  }

  // For debugging info
  // printf_begin();             // needed only once for printing details
  // radio.printDetails();       // (smaller) function that prints raw register values
  // radio.printPrettyDetails(); // (larger) function that prints human readable data

}  // setup

bool workRf24() {

  bool ret = false;

  if (role) {
    // This device is a TX node

    unsigned long start_timer = micros();                // start the timer
    //bool report = radio.write(&payload, sizeof(float));  // transmit & save the report
    bool report = radio.write(controls, sizeof(controls));  // transmit & save the report
    unsigned long end_timer = micros();                  // end the timer

    if (report) {
#ifdef TRACE
      Serial.print(F("Transmission successful! "));  // payload was delivered
      Serial.print(F("Time to transmit = "));
      Serial.print(end_timer - start_timer);  // print the timer result
      Serial.println(F(" us"));
      //Serial.print(F(" us. Sent: "));
      //Serial.println(payload);  // print payload sent
#endif // TRACE
      //payload += 0.01;          // increment float payload
      ret = true;
#ifdef TRACE
    } else {
      Serial.println(F("Transmission failed or timed out"));  // payload was not delivered
#endif // TRACE
    }

  } else {
    // This device is a RX node

    uint8_t pipe;
    if (radio.available(&pipe)) {              // is there a payload? get the pipe number that recieved it
      uint8_t bytes = radio.getPayloadSize();  // get the size of the payload
      //radio.read(&payload, bytes);             // fetch payload from FIFO
      radio.read(controls, bytes);             // fetch payload from FIFO
#ifdef TTRACE
      Serial.print(F("Received "));
      Serial.print(bytes);  // print the size of the payload
      Serial.print(F(" bytes on pipe "));
      Serial.print(pipe);  // print the pipe number
      Serial.print(F(": "));
      Serial.println(payload);  // print the payload's value
#endif // TRACE
      ret = true;
    }

  }  // role

  return ret;
}  // loop
