/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
  
  This example code is in the public domain.
 */
 
// Pin 13 has an LED connected on most Arduino boards.
// Pin 11 has the LED on Teensy 2.0
// Pin 6  has the LED on Teensy++ 2.0
// Pin 13 has the LED on Teensy 3.0
// give it a name:
const int PIN_LED = 13;

// ================================================================
// Helper function for parsing Serial input
// ================================================================

// Read buffer for commands from PC
#define PC_SERIAL_BUF_LENGTH    512
#define PC_SERIAL_MAX_ARGS      16
char    g_PCSerialBuffer[PC_SERIAL_BUF_LENGTH];
uint8_t g_PCSerialBufferIdx;
char*   g_PCSerialArgs[PC_SERIAL_MAX_ARGS];

// -- command parsing function
uint8_t parse(char *line, char **argv, uint8_t maxArgs, char sep = ' ') {

  uint8_t argCount = 0;

  while (*line != '\0') {
    /* if not the end of line ....... */
    while (*line == sep || *line == '\n')
      *line++ = '\0';     /* replace commas and white spaces with 0    */
    *argv++ = line;       /* save the argument position     */
    argCount++;
    if (argCount == maxArgs - 1)
      break;
    while (*line != '\0' && *line != sep && *line != '\n')
      line++;             /* skip the argument until ...    */
  }

  *argv = '\0';           /* mark the end of argument list  */
  return argCount;
}

// ================================================================
// the setup routine runs once when you press reset:
// ================================================================

void setup() {                
  // initialize the digital pin as an output.
  pinMode(PIN_LED, OUTPUT);  

  Serial.begin(9600);
}


// ================================================================
// ====== COMMAND: Identify
// ================================================================

void cmdIdentify(char** pArgs, uint8_t numArgs) {
  Serial.println("Device ID string --- change this...");
}

// ================================================================
// ====== COMMAND: Blink
// ================================================================

void cmdBlink(char** pArgs, uint8_t numArgs) {

  if (numArgs != 1) {
    Serial.println("Invalid arguments.");
    return;
  }

  int duration = String(pArgs[0]).toInt();

  digitalWrite(PIN_LED, HIGH);
  delay(duration);
  digitalWrite(PIN_LED, LOW);
}

// ================================================================
// Route serial commands to the right function
// ================================================================

void processSerialCommand(char** pArgs, uint8_t numArgs) {

  String cmd = String( pArgs[0] );
  void (*fun)(char**, uint8_t);

  // Determine function to execute
  if (cmd == "h"             ) { fun = &cmdIdentify; } else 
  if (cmd == "blink"         ) { fun = &cmdBlink; } else 
  if (cmd == "?") {

    Serial.println("Available commands: ");
    Serial.println("h");
    Serial.println("blink");
    return;
  } else {
    
    // Command not recognized....
    Serial.print("Command not recognized: ");
    Serial.println(cmd);
    return;
  }
  
  // Execute selected function
  fun( &(pArgs[1]), numArgs - 1 );
}

// ================================================================
// the loop routine runs over and over again forever:
// ================================================================

void loop() {

  while ( Serial.available() > 0 )
  {
    char c = Serial.read();

    if ( c != '\n' ) {
      if (g_PCSerialBufferIdx > PC_SERIAL_BUF_LENGTH - 1) {
        // Error: Out of range
        Serial.println("Error: serial input buffer overflow. Resetting buffer.");
        // Clear buffer
        memset(g_PCSerialBuffer, 0, PC_SERIAL_BUF_LENGTH);
        g_PCSerialBufferIdx = 0;
      } else {
        // Save the new character and continue
        g_PCSerialBuffer[g_PCSerialBufferIdx] = c;
        g_PCSerialBufferIdx += 1;
      }
    } else {
      // Parse arguments
      uint8_t numArgs = parse( (char*) g_PCSerialBuffer,
                               g_PCSerialArgs, PC_SERIAL_MAX_ARGS);
      // Execute command
      processSerialCommand( (char**) g_PCSerialArgs, numArgs);
      // Clear buffer
      memset(g_PCSerialBuffer, 0, PC_SERIAL_BUF_LENGTH);
      g_PCSerialBufferIdx = 0;
    }
  }
}

