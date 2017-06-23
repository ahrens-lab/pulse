/*
  Pulse
  Deliver TTL pulse with timing based on serial commands.
  
  Original code by Abel Corver, modified by Davis Bennett.
  
  This example code is in the public domain.
 */
// Pin 17 has the LED for ProMicro 5V/16MHz (use for debugging) 
// Pin 2 is the output for this example
const int PIN_OUTPUT = 2;

// Pin for sampling camera triggers
const int PIN_TTL_INPUT = 0;
volatile int ANALOG_VALUE = 0;

// Interrupt mapping for promicro: 
// pin 3 : interrupt 0
// pin 2 : interrupt 1
// pin 0 : interrupt 2
// pin 1 : interrupt 3
// pin 7 : interrupt 4
// Pin for interrupts:
const int PIN_INTERRUPT = 4;

// Update this when the camera starts a new frame
volatile unsigned long FRAME_COUNT = 0;
// keep track of time
volatile unsigned long CURRENT_TIME = 0;
volatile unsigned long LAST_FRAME_TIME = 0;

// The first frame of a SPIM stack generates a ~3.9 V ttl signal, which is converted to 800 by analogRead() 
// The next frames generate a ~3.4 V ttl signal, which is converted to 700 by analogRead() 
const int INITIAL_SPIM_FRAME = 800;
const int NONINITIAL_SPIM_FRAME = 700;

// Define various ADC prescaler values. We use this to speed up analogRead()
const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

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
   // set up the ADC
  ADCSRA &= ~PS_128;  // remove bits set by Arduino library

  // you can choose a prescaler from above.
  // PS_16, PS_32, PS_64 or PS_128
  // Smaller prescaler means fster analog reading, with reduced accuracy.  
  ADCSRA |= PS_16;    // set our own prescaler to 16

  pinMode(PIN_OUTPUT, OUTPUT);
  pinMode(PIN_TTL_INPUT, INPUT);
  attachInterrupt(PIN_INTERRUPT, cmdFrameClockRising, RISING);
     
  Serial.begin(38400);
  while(!Serial) { delay(1); }
  cmdIdentify(NULL, NULL);
}
  

// ================================================================
// ====== COMMAND: Identify
// ================================================================

void cmdIdentify(char** pArgs, uint8_t numArgs) {
  Serial.println("ProMicro 5v/16 MHz");
}

// ================================================================
// ====== COMMAND: Pulse
// ================================================================

void cmdPulse(char** pArgs, uint8_t numArgs) {

  if (numArgs != 3) {
    Serial.println("Invalid arguments.");
    return;
  }
  
  unsigned long duration = String(pArgs[0]).toInt();
  unsigned long interpulse = String(pArgs[1]).toInt();  
  unsigned long pulseno = String(pArgs[2]).toInt();
  
  // Handle non-positive values
  if (duration < 1){duration = 1;}
  if (interpulse < 1){interpulse = 1;}
  if (pulseno < 1){pulseno = 1;}
  
  int counter = 0;
    
  while (counter < pulseno)  
    {
      // check for halt signal
      if (Serial.available())
        {
          char ch = Serial.read();
          if (ch == 'q')
          {
            Serial.println("Halt command received. Quitting pulse function");
            digitalWrite(PIN_OUTPUT, LOW);
            return;
           }
        }
      digitalWrite(PIN_OUTPUT, LOW);     
      digitalWrite(PIN_OUTPUT, HIGH);
      // todo:replace delay with millis() implementation
      delay(duration);
      digitalWrite(PIN_OUTPUT, LOW);
      delay(interpulse);
      counter++;
      Serial.println(counter);
    }
}

// ================================================================
// ====== COMMAND: PulseTriggered
// ================================================================

void cmdPulseTriggered(char** pArgs, uint8_t numArgs) {

  // Pulse width, in microseconds
  unsigned long pulse_duration = 1000 * String(pArgs[0]).toInt();
  
  // Time between pulses, in microseconds
  unsigned long interpulse = 1000 * String(pArgs[1]).toInt();   

  // Total length of each pulse train
  unsigned long pulse_train_duration = 1000 * String(pArgs[2]).toInt();
  
  // Interval, in light sheet planes, of pulse delivery 
  unsigned int plane_stim_interval = 1;
     
  // Time in microseconds when pulse train was last triggered
  unsigned long pulse_train_onset = 0;

  // Time in microseconds when digital output was last set HIGH
  unsigned long pulse_onset = 0;

  // Time in microseconds when digital output was last set LOW
  unsigned long pulse_offset = 0;
  
  // Time in microseconds when the last camera frame was recorded 
  unsigned long time_since_frame = 0;
  
  // Plane within a stack of the last camera frame
  int spim_plane = 0;

  // whether a camera trigger has been checked or not
  
  // State of ditial pulse emission. 
  // 0 : untriggered
  // 1 : triggered, no output desired
  // 2 : triggered, digital output LOW
  // 3 : triggered, digital output HIGH  
  int pulse_state = 0;

  // Handle non-positive values
  if (pulse_duration < 1){pulse_duration = 1;}
  if (interpulse < 1){interpulse = 1;}
       
  while (true)  
    {           
      // check for halt signal
      if (Serial.available())
        {
          char ch = Serial.read();
          if (ch == 'q')
          {
            Serial.println("Halt command received. Quitting function");
            digitalWrite(PIN_OUTPUT, LOW);
            return;
           }
        }
                 
      time_since_frame = micros() - LAST_FRAME_TIME;
        
      // React to trigger signal within 1 ms, transition from 0 to 1 or 0 to 2
      if ((pulse_state == 0) && (time_since_frame < 1000) && (time_since_frame > 0))
        { 
          // Increment spim plane counter
         if (ANALOG_VALUE > INITIAL_SPIM_FRAME) {spim_plane = 0;}
         else {spim_plane += 1;}
                 
         if (spim_plane % plane_stim_interval == 0)         
         {         
          pulse_state = 2;
          pulse_train_onset  = micros();
         }
         else {pulse_state = 1;}
      }
      
      // Transition from state 1 to 0 if its been over 1 ms since the last camera frame
      if ((pulse_state == 1) && (time_since_frame > 1000))
      {
        pulse_state = 0;
      }

      // Pulse train is activated
      while (pulse_state > 1)
      {        
          // Transition from state 2 to state 3
          if ((pulse_state == 2) && ((micros() - pulse_offset) > interpulse)) 
          {                        
            digitalWrite(PIN_OUTPUT, HIGH);
            pulse_onset = micros();            
            pulse_state = 3;
          }

          // Transition from state 3 to state 2
          if ((pulse_state == 3) && ((micros() - pulse_onset) > pulse_duration))
          {
            digitalWrite(PIN_OUTPUT, LOW);
            pulse_offset = micros(); 
            pulse_state = 2;
          }

          // Transition from state 2 or 3 to 0
          if ((micros() - pulse_train_onset) > pulse_train_duration)                        
          {
              digitalWrite(PIN_OUTPUT, LOW);
              pulse_offset = micros();
              pulse_state = 0;
          }
      }
  }
}

// ================================================================
// ====== COMMAND: Listen
// ================================================================

void cmdListen() {
  
 // 0 - 5v range will be mapped to 0 - 1023 
  
  while (true)
  {
  if (Serial.available())
    {
      char ch = Serial.read();
        if (ch == 'q')
          {
            Serial.println("Halt command received. Quitting function");
            return;
          }
    }
    //analog_value = analogRead(PIN_TTL_INPUT);
    Serial.println(ANALOG_VALUE);
    delay(20);
}
}
// ================================================================
// ====== INTERRUPT: FrameClockRising
// ================================================================

void cmdFrameClockRising(){
  FRAME_COUNT += 1;
  LAST_FRAME_TIME = micros();
  ANALOG_VALUE = analogRead(PIN_TTL_INPUT);  
}

// ================================================================
// Route serial commands to the right function
// ================================================================

void processSerialCommand(char** pArgs, uint8_t numArgs) {

  String cmd = String( pArgs[0] );
  void (*fun)(char**, uint8_t);

  // Determine function to execute
  if (cmd == "h"              ) { fun = &cmdIdentify; } else 
  if (cmd == "pulse"          ) { fun = &cmdPulse; } else 
  if (cmd == "listen"         ) { fun = &cmdListen;} else 
  if (cmd == "pulse_triggered") { fun = &cmdPulseTriggered;} else  
  if (cmd == "?") {

    Serial.println("Available commands: ");
    Serial.println("h");
    Serial.println("pulse");
    Serial.println("listen");
    Serial.println("pulse_triggered");
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

