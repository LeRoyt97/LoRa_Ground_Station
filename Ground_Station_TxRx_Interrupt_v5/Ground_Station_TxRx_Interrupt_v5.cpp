#include "LoRa.h"
#include "SPI.h"

/*
  todo:tariq

  might want to make the LoRa settings change depending on where the
  balloon is relative to us also something worth looking into is why we use the
  LoRa settings we do
*/
// Hardware configuration constants
#define LORA_CS A5   // LoRa chip select pin
#define LORA_RST A4  // LoRa reset pin
#define LORA_DIO0 A3 // LoRa DIO0 interrupt pin

// Communication protocol constants
#define SERIAL_BAUD_RATE 115200  // Serial communication with Python host (bps)
#define LORA_FREQUENCY 915E6     // LoRa frequency (Hz)
#define LORA_TX_POWER 20         // LoRa transmit power (dBm)
#define LORA_SPREADING_FACTOR 12 // LoRa spreading factor (7-12)
#define LORA_BANDWIDTH 62500     // LoRa signal bandwidth (Hz)
#define LORA_CODING_RATE 8       // LoRa coding rate denominator
#define LORA_SYNC_WORD 0xF3      // LoRa network sync word
#define LORA_INIT_RETRIES 5      // Number of LoRa initialization attempts
#define LORA_RETRY_DELAY_MS 1000 // Delay between initialization retries (ms)
#define PACKET_REPEAT_COUNT 7    // Number of command packets to send
#define PACKET_DELAY_MS 50       // Delay between packet transmissions (ms)

// Transmission window protocol
#define ID_BIT1 0     // Index of first ID bit in received packet
#define Tx_ID '1'     // Character indicating TX window is open
#define Tx_time 10000 // TX window duration (ms)

// Balloon command definitions
#define IDLE "00000000"  // Keep balloon in idle state
#define CUT "11111111"   // Cut parachute/balloon separation
#define OPEN "22222222"  // Open vent valve for gas release
#define CLOSE "33333333" // Close vent valve to retain gas

// Declare Variables
unsigned long Tx_mark = 0; // Marks time zero
bool Tx_ok = false;        // Are we in the Tx window: true or false
int packet_send_count = 0;
String packet = "";
String cmd = "";
bool tx_window_changed = true; // Window Change notification
bool wait_for_next_tx_window =
    true;                // If sent in the middle of Tx wait till next window
int number_of_packets =  // todo:tariq is this ever going to be < 0?
    PACKET_REPEAT_COUNT; // Number of command packets to send

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial) {
  }
  delay(1000);
  Serial.println("***************LoRa Ground Station****************");

  // Initialize LoRa with retry strategy:
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  bool lora_initialized = false;
  for (int attempt = 1; attempt <= LORA_INIT_RETRIES; attempt++) {
    Serial.print("LoRa initialization attempt ");
    Serial.print(attempt);
    Serial.print("/");
    Serial.println(LORA_INIT_RETRIES);

    if (LoRa.begin(LORA_FREQUENCY)) {
      lora_initialized = true;
      Serial.println("LoRa initialization successful!");
      break;
    }

    if (attempt < LORA_INIT_RETRIES) {
      Serial.println("LoRa init failed, retrying in 1 second...");
      delay(LORA_RETRY_DELAY_MS);
    }
  }

  if (!lora_initialized) {
    Serial.println("CRITICAL: LoRa initialization failed after 5 attempts");
    Serial.println(
        "Check: Hardware connections, power supply, module compatibility");
    Serial.println("SYSTEM HALTED - Reset required after fixing hardware");
    while (1)
      ;
  }

  // LoRa Settings:
  LoRa.setTxPower(LORA_TX_POWER);
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  // LoRa.enableCrc();

  // Command Instructions:
  Serial.println("LoRa Init Successful");
  Serial.println("Ground Station started in Rx mode");
}

// IDLE COMMAND
void idleCmd() {
  Serial.println("Sending idle...");

  LoRa.idle(); // Give radio time to get ready

    // Send specified # of packets
  while (packet_send_count <= number_of_packets) {
    LoRa.beginPacket(); // Prepare and Send Packet
    LoRa.print(IDLE);
    LoRa.endPacket();

    packet_send_count++;
    Serial.print("                ");
    Serial.print(packet_send_count);
    Serial.println(" Packets sent");
    delay(PACKET_DELAY_MS);
  }

  packet_send_count = 1;
  cmd = "";
  Serial.println("Done Sending.");
  LoRa.receive();
  Serial.println("Back to listenting");
}

//-----------------------------CUT COMMAND-----------------------------
void cutCmd() {
  Serial.println("Sending cut...");

  LoRa.idle(); // Give radio time to get ready

  while (packet_send_count <=
         number_of_packets) { // Send specified # of packets
    LoRa.beginPacket();       // Prepare and Send Packet
    LoRa.print(CUT);
    LoRa.endPacket();

    packet_send_count++;
    Serial.print("                ");
    Serial.print(packet_send_count);
    Serial.println(" Packets sent");
    delay(PACKET_DELAY_MS);
  }

  packet_send_count = 1;
  cmd = "";
  Serial.println("Done Sending.");
  LoRa.receive();
  Serial.println("Back to listenting");
}

//-----------------------------OPEN COMMAND-----------------------------
void openCmd() {
  Serial.println("Sending open...");

  LoRa.idle(); // Give radio time to get ready

  while (packet_send_count <=
         number_of_packets) { // Send specified # of packets
    LoRa.beginPacket();       // Prepare and Send Packet
    LoRa.print(OPEN);
    LoRa.endPacket();

    packet_send_count++;
    Serial.print("                ");
    Serial.print(packet_send_count);
    Serial.println(" Packets sent");
    delay(PACKET_DELAY_MS);
  }

  packet_send_count = 1;
  cmd = "";
  Serial.println("Done Sending.");
  LoRa.receive();
  Serial.println("Back to listenting");
}

//-----------------------------CLOSE COMMAND-----------------------------
void closeCmd() {
  Serial.println("Sending close...");

  LoRa.idle(); // Give radio time to get ready

  while (packet_send_count <=
         number_of_packets) { // Send specified # of packets
    LoRa.beginPacket();       // Prepare and Send Packet
    LoRa.print(CLOSE);
    LoRa.endPacket();

    packet_send_count++;
    Serial.print("                ");
    Serial.print(packet_send_count);
    Serial.println(" Packets sent");
    delay(PACKET_DELAY_MS);
  }

  packet_send_count = 1;
  cmd = "";
  Serial.println("Done Sending.");
  LoRa.receive();
  Serial.println("Back to listenting");
}

void loop() {
  // **************************Set radio to Receive packet
  // **************************
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
  // **************************Check ID_BITs for Tx window
  // **************************
  if (packet[ID_BIT1] == Tx_ID && packet[packet.length() - 1] == Tx_ID &&
      wait_for_next_tx_window == true) {
    if (tx_window_changed == true) {
      Serial.println("Tx window now open");
      tx_window_changed = false;
    }

    if (cmd ==
        "IDLE") { // If there is a command stored and Tx window is open, send it
      idleCmd();
    } else if (cmd == "CUT") {
      cutCmd();
    } else if (cmd == "OPEN") {
      openCmd();
    } else if (cmd == "CLOSE") {
      closeCmd();
    }
  }

  if (tx_window_changed == false && packet[ID_BIT1] == '0') { // Reset Indicator
    Serial.println("Tx window closed");
    tx_window_changed = true;
    wait_for_next_tx_window = true;
  }

  // **************************Check for command input from Python
  // host**************************
  if (Serial.available()) {
    cmd = Serial.readStringUntil('\n');
    cmd.trim();        // Remove whitespace and newlines
    cmd.toUpperCase(); // Make it case-insensitive

    if (packet[ID_BIT1] == Tx_ID &&
        packet[packet.length() - 1] ==
            Tx_ID) { // If in Tx window, wait until next window
      if (cmd == "CUT" or cmd == "OPEN" or cmd == "CLOSE" or cmd == "IDLE") {
        Serial.print("\nWaiting till next Tx Window will send: ");
        Serial.println(cmd);
        Serial.println("");
        wait_for_next_tx_window = false;
      } else {
        Serial.print(cmd);
        Serial.println(" is unknown. Retry");
      }
    } else { // If not in Tx window store command prompt and indicate when
             // command will be sent
      Serial.println("");
      Serial.print("Not in Tx window. Will Send ");
      Serial.print(cmd);
      Serial.println(" when Tx_ID is received");
      Serial.println("");
    }
  }
}
