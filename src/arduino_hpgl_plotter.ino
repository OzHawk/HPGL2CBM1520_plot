/*
 Arduino HPGL Plotter

 This code implements a small HPGL pen plotter.

 Based on Mini CNC Plotter firmware, based in TinyCNC https://github.com/MakerBlock/TinyCNC-Sketches

 */

#include <Servo.h>
#include <Stepper.h>

#define LINE_BUFFER_LENGTH 512

#define X_SPEED     (250)
#define Y_SPEED     (250)

#define MAX_X       (4000)    // Maximum number of steps in X-direction
#define MAX_Y       (4000)    // Maximum number of steps in Y-direction

#define HOME_X      (MAX_X)   // Steps from 0
#define HOME_Y      (MAX_Y)   // Steps from 0

#define X_STEP_DIR  (1)       // Direction for positive X (+1 or -1)
#define Y_STEP_DIR  (1)       // Direction for positive Y (+1 or -1)

// Set to true to get debug output.
boolean verbose = false;

// Hardware setup for X/Y stepper motors
//
// Should be right for DVD steppers, but is not too important here
const int stepsPerRevolution = 20;
// Initialize steppers for X- and Y-axis using this Arduino pins for the L293D H-bridge
Stepper myStepperY(stepsPerRevolution, 2,3,4,5);
Stepper myStepperX(stepsPerRevolution, 8,9,10,11);

// Pen servo hardware setup
//
const int penServoPin = 6;    // Servo on PWM pin 6
const int penZUp = 80;        // Servo position for Up
const int penZDown = 40;      // Servo position for Down
Servo penServo;               // Create servo object to control a servo

// Motor steps to go 1 millimeter.
// Use test sketch to go 100 steps. Measure the length of line.
// Calculate steps per mm. Enter here.
float StepsPerMillimeterX = 6.0;
float StepsPerMillimeterY = 6.0;

//  Drawing settings, should be OK
float StepInc = 1;
int StepDelay = 0;
int LineDelay = 50;
int penDelay = 50;

// Pen limits in steps
//
float Xmin   = 0;
float Xmax   = MAX_X;
float Ymin   = 0;
float Ymax   = MAX_Y;

float Xpos   = Xmin;
float Ypos   = Ymin;
float Zpos   = penZUp;
bool  isPA  = false;

/* Structures, global variables    */
struct point {
  float x;
  float y;
};

struct point penPos;          // Current absolute position of plothead. The current
                              // position my be beyond the physical limits
                              // of the plot area.

/*
    This plotter understands a limited subset of the original HPGL
    command set.
    It understands the following commands.
    Initialisation Commands:
      DF - Resets certain features to defaults.  Mostly ignored.
      IN - Initialise
    Pen Control Commands:
      PU [x,y(,x,y...)] - Pen Up
      PD [x,y(,x,y...)] - Pen Down
      PA [x,y(,x,y...)] - Plot Absolute
      PR [x,y(,x,y...)] - Plot Relative
      LT - Line Type <Not implemented>
      SP [<n>] - Pen Select <TBD> (May pause, prompting for pen change)
      VS [<s>] - Velocity Select
*/

/**********************************
 * void setup() - Initialisations
 **********************************/
void setup() {
  //  Setup
  Serial.begin( 9600 );

  penServo.attach(penServoPin);
  penServo.write(penZUp);
  delay(200);

  // Decrease if necessary
  myStepperX.setSpeed(X_SPEED);
  myStepperY.setSpeed(Y_SPEED);

  //  Set & move to initial default position
  //  Without endstop detection, this may be too hard on the
  //  worm screw mechanism.  To avoid too much damage, the plotter
  //  should be turned on with the pen in the HOME position.
  //
  // Move to diagonal opposite home
  moveRel(MAX_X*(-1),MAX_Y*(-1));
  // Move to Home point
  moveRel(MAX_X,MAX_Y);
  penPos.x = MAX_X;
  penPos.y = MAX_Y;

  Serial.println("-------------------");
  Serial.println(" Mini HPGL Plotter");
  Serial.println("-------------------");
  Serial.print("X range is from ");
  Serial.print(Xmin);
  Serial.print(" to ");
  Serial.print(Xmax);
  Serial.println(" steps.");
  Serial.print("Y range is from ");
  Serial.print(Ymin);
  Serial.print(" to ");
  Serial.print(Ymax);
  Serial.println(" steps.");
}

/***************************
 * void loop() - Main loop
 ***************************/
void loop()
{
  delay(200);
  char line[ LINE_BUFFER_LENGTH ];
  char c;
  int lineIndex;
  bool lineSemiColon;

  lineIndex = 0;
  lineSemiColon = false;

  while (1) {

    // Serial reception
    //
    while ( Serial.available()>0 ) {
      c = Serial.read();
      if (( c == '\n') || (c == '\r') ) {             // End of line reached
        if ( lineIndex > 0 ) {                        // Line is complete. Then execute!
          line[ lineIndex ] = '\0';                   // Terminate string
          if (verbose) {
            Serial.print( "Received : ");
            Serial.println( line );
          }
          processIncoming();
          lineIndex = 0;
        }
        else {
          // Empty or comment line. Skip block.
        }
        lineSemiColon = false;
        Serial.println("OK");
      }
      else {
        if ( c <= ' ' ) {                             // Throw away whitepace and control characters
        }
        else if ( c == ';' ) {
          lineSemiColon = true;
        }
        else if ( lineIndex >= LINE_BUFFER_LENGTH-1 ) {
          Serial.println( "[ERR] Buffer overflow" );
          lineSemiColon = false;
        }
        else if ( c >= 'a' && c <= 'z' ) {
          line[ lineIndex++ ] = c-'a'+'A';        // Convert to upper case
        }
        else {
          line[ lineIndex++ ] = c;
        }
      }
    }
  }
}

void processIncoming(void)
{
  bool  finished = false;
  char  cmd[3];
  int   num;

  discardSeparators();

  // Read the next command
  while(Serial.available()>1) {
    num = Serial.readBytes(cmd, 2);
    cmd[2] = '/0';
    if (num<2) {
      // We should never get here.
      //
      Serial.print("[WRN] Timeout reading command: ");
      Serial.println(cmd);
    }
    else {
      //
      // Process commands
      //
      switch (cmd[0]) {
        // Pen movement commands
        case 'P':
          switch (cmd[1]) {
            // PU - Pen Up
            case 'U':
              penUp();
              break;
            // PD - Pen Down
            case 'D':
              penDown();
              break;
            // PA - Plot Absolute
            case 'A':
              isPA = true;
              break;
            // PR - Plot Relative
            case 'R':
              isPA = false;
              break;
          }
          // Process any optional pen movements associated with this command.
          //
          while (Serial.peek()!=';' && Serial.peek()!='\n') {
            processMove();
          }
          break;
        case 'S':
          switch (cmd[1]) {
            // SP - Select Pen <Not implemented>
            case 'P':
              Serial.print("[WRN] SP Command not yet implemented.");
              break;
            // Unrecognised command
            default:
              Serial.print("[WRN] Unrecognised command: S");
              Serial.println(cmd);
              break;
          }
          // Process any remaining characters associated with this command.
          //
          break;
        case 'V':
          switch (cmd[1]) {
            // VS - Velocity Select <Not implemented>
            case 'S':
              Serial.print("[WRN] VS Command not yet implemented.");
              break;
          }
          // Process any remaining characters associated with this command.
          //
          break;
      default:
        Serial.print("[WRN] Unrecognised command: ");
        Serial.println(cmd);
      }
    }
  }
}

/*
  This method removes unwanted separators from the serial input stream.
*/
void discardSeparators(void)
{
  bool  finished = false;
  char  c;

  // Discard unwanted separators
  while ((Serial.available()>0) && (!finished)) {
    c = Serial.peek();
    switch (c) {
      // The following characters get removed from the input stream.
      case ';':
      case '/n':
      case ' ':
      case '/r':
      case '/t':
        // Read the byte and discard it.
        Serial.read();
        break;
      default:
        finished = true;
    }
  }
}

/*
  This method processes the input queue expecting pen movements.
*/
void processMove(void)
{
  int   x, y;
  bool  finished = false;

  x = Serial.parseInt();
  y = Serial.parseInt();
  moveRel(x-penPos.x, y-penPos.y);
}

/*
  Move the pen from the current position to the new position in as straight a line as possible.
  DX,DY is specified in steps relative to the current position.

  Bresenham algo from https://www.marginallyclever.com/blog/2013/08/how-to-build-an-2-axis-arduino-cnc-gcode-interpreter/

*/
void moveRel(int dx, int dy)
{
  if (verbose) {
    Serial.print("[INF] moveRel (dx,dy): ");
    Serial.print(dx);
    Serial.print(",");
    Serial.print(dy);
  }

  int sx = dx>0 ? StepInc : -StepInc;
  int sy = dy>0 ? StepInc : -StepInc;

  long i;
  long over = 0;

  if (dx > dy) {
    for (i=0; i<dx; ++i) {
      penPos.x+=sx;
      // Only step the motor if not beyond the limits
      if (penPos.x>0) && (penPos.x<MAX_X) {
       myStepperX.step(sx);
      }
      over+=dy;
      if (over>=dx) {
        over-=dx;
        penPos.y+=sy
        // Only step the motor if not beyond the limits
        if (penPos.y>0) && (penPos.y<MAX_Y) {
          myStepperY.step(sy);
        }
        delay(StepDelay);
      }
    }
  }
  else {
    for (i=0; i<dy; ++i) {
      penPos.y+=sy;
      // Only step the motor if not beyond the limits
      if (penPos.y>0) && (penPos.y<MAX_Y) {
        myStepperY.step(sy);
      }
      over+=dx;
      if (over>=dy) {
        over-=dy;
        penPos.x+=sx;
        // Only step the motor if not beyond the limits
        if (penPos.x>0) && (penPos.x<MAX_X) {
          myStepperX.step(sx);
        }
      }
      delay(StepDelay);
    }
  }

  //  Delay before any next lines are submitted
  delay(LineDelay);

  if (verbose) {
    Serial.print(" --> New position (x0,y0): ");
    Serial.print(penPos.x);
    Serial.print(",");
    Serial.println(penPos.y);
  }
}

//  Raises pen
void penUp() {
  penServo.write(penZUp);
  delay(LineDelay);
  Zpos=Zmax;
  if (verbose) {
    Serial.println("[INF] Pen up");
  }
}
//  Lowers pen
void penDown() {
  penServo.write(penZDown);
  delay(LineDelay);
  Zpos=Zmin;
  if (verbose) {
    Serial.println("[INF] Pen down.");
  }
}