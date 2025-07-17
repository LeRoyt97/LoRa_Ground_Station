#include <LoRa.h>
#include <SPI.h>



// // LoRa pins For Feather M0 Adalogger. Remeber to connect MISO, MOSI, SCK also.
#define LORA_CS     A5
#define LORA_RST    A4
#define LORA_DIO0   A3

// ID BITS to check for Tx window
#define ID_BIT1 0                     // ID_BIT1 index
#define Tx_ID '1'                     // ID value to signal Tx mode
#define Tx_time 10000                 // Tx window length

// Command Definitions
#define IDLE "00000000"
#define CUT "11111111"
#define OPEN "22222222"
#define CLOSE "33333333"

// Declare Variables
unsigned long Tx_mark = 0;              // Marks time zero
bool Tx_ok = false;                     // Are we in the Tx window: true or false
int packetSendCount = 0;
String packet = "";
String cmd = "";
bool Tx_window_changed = true;          // Window Change notification
bool wait_for_next_Tx_window = true;    // If sent in the middle of Tx wait till next window
int number_of_packets = 7;              // Number of packets to send. Indexed from 1!!

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(1000);
  Serial.println("***************LoRa Ground Station****************");

// **************************Initialize LoRa **************************
  long LoraFreq = 915E6; // LoRa Frequency
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LoraFreq)) {
    Serial.println("LoRa init Failed!");
    while(1);
  }

// **************************LoRa Settings **************************
  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(62.5E3);
  LoRa.setCodingRate4(8);
  LoRa.setSyncWord(0xF3);
  // LoRa.enableCrc();
  

// Command Instructions:
  Serial.println("LoRa Init Successful");
  Serial.println("Ground Station started in Rx mode");
}



void loop() {
  // **************************Set radio to Receive packet **************************
  int packetSize = LoRa.parsePacket();
  if (packetSize > 0) {
    while (LoRa.available()) {
      packet = LoRa.readString();
      Serial.print(packet);
    }
    Serial.print(":");
    Serial.print(LoRa.packetRssi());
    Serial.print(":");
    Serial.println(LoRa.packetSnr());
    


  }
  // **************************Check ID_BITs for Tx window **************************
  if (packet[ID_BIT1] ==  Tx_ID && packet[packet.length()-1] ==  Tx_ID && wait_for_next_Tx_window==true) {
    if (Tx_window_changed == true) {
      Serial.println("Tx window now open");
      Tx_window_changed = false;
    }

    if (cmd == "IDLE") {          // If there is a command stored and Tx window is open, send it
      idleCmd();
    } else if (cmd == "CUT") {
      cutCmd();
    } else if (cmd == "OPEN") {
      openCmd();
    } else if (cmd == "CLOSE") {
      closeCmd();
    }
  }

  if (Tx_window_changed == false && packet[ID_BIT1] ==  '0') {    // Reset Indicator
    Serial.println("Tx window closed");
    Tx_window_changed = true;
    wait_for_next_Tx_window = true;
  }


  // **************************Check for keyboard input**************************
  if (Serial.available()) {
    cmd = Serial.readStringUntil('\n');
    cmd.trim(); // Remove whitespace and newlines
    cmd.toUpperCase(); // Make it case-insensitive

    if (packet[ID_BIT1] ==  Tx_ID && packet[packet.length()-1] ==  Tx_ID) {   // If in Tx window, wait until next window
      if (cmd == "CUT" or cmd == "OPEN" or cmd == "CLOSE" or cmd == "IDLE") {
        Serial.print("\nWaiting till next Tx Window will send: ");
        Serial.println(cmd);
        Serial.println("");
        wait_for_next_Tx_window = false;
      } else {
        Serial.print(cmd);
        Serial.println(" is unknown. Retry");
      }
    } else {          // If not in Tx window store command prompt and indicate when command will be sent
      Serial.println("");
      Serial.print("Not in Tx window. Will Send ");  
      Serial.print(cmd);
      Serial.println(" when Tx_ID is received");
      Serial.println("");
      }    

  }
}



//-----------------------------IDLE COMMAND-----------------------------
void idleCmd() {
  Serial.println("Sending idle...");

  LoRa.idle();                    // Give radio time to get ready

  while (packetSendCount <= number_of_packets) {   // Send specified # of packets
    LoRa.beginPacket();             // Prepare and Send Packet
    LoRa.print(IDLE);
    LoRa.endPacket();

    packetSendCount++;
    Serial.print("                ");
    Serial.print(packetSendCount);
    Serial.println(" Packets sent");
    delay(50);
  }

  packetSendCount = 1;
  cmd = "";
  Serial.println("Done Sending.");
  LoRa.receive();
  Serial.println("Back to listenting");
  
}

//-----------------------------CUT COMMAND-----------------------------
void cutCmd() {
  Serial.println("Sending cut...");

  LoRa.idle();        // Give radio time to get ready

  while (packetSendCount <= number_of_packets) {   // Send specified # of packets
    LoRa.beginPacket(); // Prepare and Send Packet
    LoRa.print(CUT);
    LoRa.endPacket();


    packetSendCount++;
    Serial.print("                ");
    Serial.print(packetSendCount);
    Serial.println(" Packets sent");
    delay(50);
  }

  packetSendCount = 1;
  cmd = "";
  Serial.println("Done Sending.");
  LoRa.receive();
  Serial.println("Back to listenting");
}

//-----------------------------OPEN COMMAND-----------------------------
void openCmd() {
  Serial.println("Sending open...");

  LoRa.idle();                      // Give radio time to get ready

  while (packetSendCount <= number_of_packets) {   // Send specified # of packets
    LoRa.beginPacket();             // Prepare and Send Packet
    LoRa.print(OPEN);
    LoRa.endPacket();

    packetSendCount++;
    Serial.print("                ");
    Serial.print(packetSendCount);
    Serial.println(" Packets sent");
    delay(50);
  }

  packetSendCount = 1;
  cmd = "";
  Serial.println("Done Sending.");
  LoRa.receive();
  Serial.println("Back to listenting");
}

//-----------------------------CLOSE COMMAND-----------------------------
void closeCmd() {
  Serial.println("Sending close...");

  LoRa.idle();        // Give radio time to get ready

  while (packetSendCount <= number_of_packets) {   // Send specified # of packets
    LoRa.beginPacket(); // Prepare and Send Packet
    LoRa.print(CLOSE);
    LoRa.endPacket();

    packetSendCount++;
    Serial.print("                ");
    Serial.print(packetSendCount);
    Serial.println(" Packets sent");
    delay(50);
  }

  packetSendCount = 1;
  cmd = "";
  Serial.println("Done Sending.");
  LoRa.receive();
  Serial.println("Back to listenting");
}

