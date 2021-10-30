#include <wiring_private.h>
#define TDI         12
#define TDO         15
#define TCK         13
#define TMS         14
#define MB_INT      28
#define MB_INT_PIN  31
#define SIGNAL_OUT  41
#define SIGNAL_IN   33

void setup_fpga()
{
  // Configure onboard LED Pin as output
  pinMode(LED_BUILTIN, OUTPUT);

  // Disable all JTAG Pins (useful for USB BLASTER connection)
  pinMode(TDO, INPUT);
  pinMode(TMS, INPUT);
  pinMode(TDI, INPUT);
  pinMode(TCK, INPUT);

  // JTAG pins? Double check in Vidor's variant.cpp
  pinMode(26, INPUT);
  pinMode(27, INPUT);
  pinMode(28, INPUT);
  pinMode(29, INPUT);
  
  // Configure other share pins as input too
  pinMode(SIGNAL_IN, INPUT);  // oSAM_INTstat
  pinMode(MB_INT_PIN, INPUT);
  pinMode(MB_INT, INPUT);

  pinMode(6, INPUT); // Init / Done
  pinMode(7, INPUT); // CRC Error
  pinMode(8, INPUT); // NCEO
  pinMode(9, INPUT); // CLKUSER
}

void setup() {
  // put your setup code here, to run once:
  setup_fpga();
  Serial.begin(9600);
  Serial.println("FPGA Clock configured");  
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(32, 1);
  delay(500);
  digitalWrite(LED_BUILTIN, 0);
  delay(500);
  Serial.println("Heartbeat");  
}