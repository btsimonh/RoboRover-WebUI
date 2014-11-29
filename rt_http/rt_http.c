//
// Raspberry Tank HTTP Remote Control script
// Released under the BSD licence
// Ian Renton, February 2013
// http://ianrenton.com
// 
// Based on the Raspberry Pi GPIO example by Dom and Gert
// (http://elinux.org/Rpi_Low-level_peripherals#GPIO_Driving_Example_.28C.29)
// ...and I2C Compass/Rangefinder code by James Henderson, from robot-electronics.co.uk
// Using Heng Long op codes discovered by ancvi_pIII
// (http://www.rctanksaustralia.com/forum/viewtopic.php?p=1314#p1314)
//

#define BCM2708_PERI_BASE        0x20000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <assert.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include "mongoose.h"


#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

int  h_serial;


// serial commmand sto be used with modified UBW
char startup[]              = "\n\n\n\n\nR\n\nV\n";

char setup_IR_in_direction[]= "PD,B,6,1\nPD,B,7,1\nPD,A,0,1\n";
char setup_IR_out[]         = "PD,A,1,0\nPD,A,2,0\nPD,A,3,0\nPO,A,1,1\nPO,A,2,1\nPO,A,3,1\n";
char setup_IR_out_power[]   = "PD,B,1,0\nPO,B,1,1\n";

char setup_hbridge[]        = "HS,D,2,D,3,D,0,D,1\n";
char stop_hbridge[]         = "HB,0,0\n";
char setup_analogue[]       = "C,255,255,255,1\n";


char setup_IR_modulation[]  = "W,160,63400\n";

char request_data[]         = "T,500,0\nT,500,1\n";
char stop_data[]            = "T,0,0\nT,0,1\n";

char sense_lowpower[]       = "IX,3,1,5555\n";
char sense_backwards[]      = "IX,2,1,1111\n";
char sense_highpower[]      = "IX,1,1,3333\n";


char LastIR[3][100];


// 0 = high power left ear 
// 1 = high power right ear
// 2 = low power left ear 
// 3 = low power right ear
// 4 = rear

int IRDetect[5] = {0};

typedef struct tag_Commands
{
    char name[20];
    char Serial[20];
} COMMANDS;

COMMANDS Commands[] =
{
    { "stop", "HB,0,0\n" },

    { "forward", "HB,5,5\n" },

    { "fwdleft", "HB,0,5\n" },
    { "left", "HB,-5,5\n" },
    { "bwdleft", "HB,-5,0\n" },

    { "fwdright", "HB,5,0\n" },
    { "right", "HB,5,-5\n" },
    { "bwdright", "HB,0,-5\n" },

    { "reverse", "HB,-5,-5\n" },
};


// Mutexes
pthread_mutex_t userCommandMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t autonomyCommandMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t sensorDataMutex = PTHREAD_MUTEX_INITIALIZER;

pthread_mutex_t SerialSendMutex = PTHREAD_MUTEX_INITIALIZER;


// Mutex-controlled Variables
char* userCommand;
char* autonomyCommand;
int range;
int bearing;
int pitch;
int roll;

unsigned int AnalogueValues[32] = {0};
int NumAnalogueValues = 0;
unsigned int DigitalValues[3] = {0};
int NumDigitalValues = 0;


// Function declarations
void setup_io();
void sendCode(char *code);
void* launch_server();
static int http_callback(struct mg_connection *conn);
void* launch_sensors();
void* launch_autonomy();
void* autonomySendCommand(char* cmd);

char ubwinputstore[256];
int ubwcount = 0;
void parse_ubw( char c );

// Main
int main(int argc, char **argv) 
{ 
  printf("\nMK802 Tank HTTP Remote Control script\nIan Renton, February 2013 - modified for RoboRover SH\nhttp://ianrenton.com\n\n");

  int g,rep,i;
  char inchar;
  userCommand = malloc(sizeof(char)*100);
  autonomyCommand = malloc(sizeof(char)*100);
  char* copiedCommand = malloc(sizeof(char)*100);

  // Set up gpio pointer for direct register access
  setup_io();

  // Initialise our output
  printf("Initialise\n");

  sendCode(startup);
  sendCode(setup_analogue);
  sendCode(setup_IR_in_direction);
  sendCode(setup_IR_out);
  sendCode(setup_IR_out_power);
  sendCode(setup_IR_modulation);
  sendCode(setup_hbridge);
  sendCode(stop_hbridge);
  sendCode(stop_data);  

  sendCode(request_data);



  // Launch HTTP server
  pthread_t httpThread; 
  int httpThreadExitCode = pthread_create( &httpThread, NULL, &launch_server, (void*) NULL);
  
  // Launch sensor polling thread
  pthread_t sensorThread; 
  int sensorThreadExitCode = pthread_create( &sensorThread, NULL, &launch_sensors, (void*) NULL);
  
  // Launch autonomy thread
  pthread_t autonomyThread; 
  int autonomyThreadExitCode = pthread_create( &autonomyThread, NULL, &launch_autonomy, (void*) NULL);
  
  // Loop, sending movement commands indefinitely
  while(1) 
  {
    int i;

    pthread_mutex_lock( &userCommandMutex );
    strcpy(&copiedCommand[0], &userCommand[0]);
    userCommand[0] = 0;
    pthread_mutex_unlock( &userCommandMutex );
    
    for ( i = 0; i < sizeof(Commands)/sizeof(Commands[0]); i++)
    {
        if (!strcmp( copiedCommand, Commands[i].name ))
        {
            sendCode(Commands[i].Serial);
            break;
        }
    }

    copiedCommand[0] = 0;

#ifdef NOTHERE
    if (copiedCommand[8] == '1') 
    {
      // Autonomy requested, so obey autonomy's commands not the user commands.
      pthread_mutex_lock( &autonomyCommandMutex );
      strcpy(&copiedCommand[0], &autonomyCommand[0]);
      pthread_mutex_unlock( &autonomyCommandMutex );
    }
#endif

    usleep(40000);
  }
  
  printf("Leaving\n");
  
  close( h_serial );

  return 0;
} // main



char lastsent[100];
// Sends one individual code to the main tank controller
void sendCode(char *cmd) 
{
  int len = strlen(cmd);
  pthread_mutex_lock( &SerialSendMutex );
        
//  if (0 == strcmp(lastsent, cmd))
//    return;

  strcpy( lastsent, cmd );
  write( h_serial, cmd, len );

  printf("Sending <%s>\r\n", cmd );
  pthread_mutex_unlock( &SerialSendMutex );

} // sendCode


// Set up a memory region to access GPIO
void setup_io() 
{

   /* open /dev/mem */
   if ((h_serial = open("/dev/ttyACM0", O_RDWR | O_NONBLOCK) ) < 0) {
      printf("can't open /dev/ttyACM0 \n");
      exit (-1);
   }

} // setup_io


// Launch HTTP server
void* launch_server() 
{
  struct mg_context *ctx;
  struct mg_callbacks callbacks;

  const char *options[] = {"listening_ports", "3000", NULL};
  memset(&callbacks, 0, sizeof(callbacks));
  callbacks.begin_request = http_callback;
  
  printf("Starting HTTP Server on port 3000\n");

  ctx = mg_start(&callbacks, NULL, options);
  getchar();  // Wait until user hits "enter".  This will never happen when this
              // code runs on the tank at startup
  mg_stop(ctx);
}

// HTTP server callback
static int http_callback(struct mg_connection *conn) 
{

    const struct mg_request_info *request_info = mg_get_request_info(conn);

    char* tempCommand = malloc(sizeof(char)*100);
    memset(tempCommand, 0, sizeof(char)*100);
    if (request_info->query_string)
	{
    	strncpy(&tempCommand[0], &request_info->query_string[0], 99);
	}
    tempCommand[99] = 0;
    printf("Received command from HTTP: %s\n", tempCommand);

    // Set received, so send it over to the control thread
    if ((tempCommand[0] == 's') && (tempCommand[1] == 'e') && (tempCommand[2] == 't')) {
      pthread_mutex_lock( &userCommandMutex );
      strncpy(&userCommand[0], &tempCommand[3], 9);
      pthread_mutex_unlock( &userCommandMutex );
      printf("Set motion command: %.*s\n", 9, userCommand);

      // Send an HTTP header back to the client
      mg_printf(conn, "HTTP/1.1 200 OK\r\n"
              "Content-Type: text/plain\r\n\r\n");
    }

    // Get received, so return sensor data
    /*else if ((tempCommand[0] == 'g') && (tempCommand[1] == 'e') && (tempCommand[2] == 't')) {
      // Get data from the variables while the mutex is locked
      pthread_mutex_lock( &sensorDataMutex );
      int tmpRange = range;
      int tmpBearing = bearing;
      int tmpPitch = pitch;
      int tmpRoll = roll;
      pthread_mutex_unlock( &sensorDataMutex );
      printf("Sensor data acquired.\n");

      // Prepare the response
      char response[100];
      int contentLength = snprintf(response, sizeof(response),
            "Range: %d   Bearing: %d   Pitch: %d   Roll: %d",
            tmpRange, tmpBearing, tmpPitch, tmpRoll);

      printf("Sending HTTP response: %s\n", response);

      // Send an HTTP response back to the client
      mg_printf(conn, "HTTP/1.1 200 OK\r\n"
              "Content-Type: text/plain\r\n"
              "Content-Length: %d\r\n"
              "\r\n"
              "%s",
              contentLength, response);
      
    }*/
    printf("Finished responding to HTTP request.\n");

    return 1;  // Mark as processed
}


// Launch sensor polling thread
void* launch_sensors() 
{
  char buffer[256];
  int i;
  int Index = 0;

  float LastVolts = 0.0;
  printf("Starting sensor polling\n");
  while(1) 
  {
     int count = read(h_serial, buffer, 255);

     if (count > 0)
     {
        for (i = 0; i < count; i ++)
        {
           parse_ubw( buffer[i] );
        }
     }

     if (NumAnalogueValues)
     {
        float Volts = AnalogueValues[0];
        Volts = Volts / 36000.0;
        Volts = Volts * 7.53;
	    LastVolts = Volts;
	    printf("    Battery = %02.2fV\n", Volts );
        NumAnalogueValues = 0;
     }

    // Initial pause for safety
    usleep(10000);

    Index++;

    if (!(Index % 150))
    {
        // send IR for fwd sensing
        sendCode(sense_highpower);
        if (IRDetect[0] > 0) IRDetect[0]--;
        if (IRDetect[1] > 0) IRDetect[1]--;
    }

    if (!((Index+50) % 150))
    {
        sendCode(sense_lowpower);
        if (IRDetect[2] > 0) IRDetect[2]--;
        if (IRDetect[3] > 0) IRDetect[3]--;
    }

    if (!((Index+100) % 150))
    {
        sendCode(sense_backwards);
        if (IRDetect[4] > 0) IRDetect[4]--;
    }

    for (i = 0; i < 5; i++)
    {
        if (IRDetect[i] > 5)
            IRDetect[i] = 5;
    }

    if (!(Index % 10))
    {
        // Output to file
        FILE* f = fopen("/var/www/localhost/htdocs/tmp/sensordata.txt", "w");
        if (f)
    	{
    	    fprintf(f, "Battery = %02.2fV&nbsp;&nbsp;&nbsp;&nbsp;IR LH%d RH%d LL%d RL%d B%d\n", 
                LastVolts,
                IRDetect[0],
                IRDetect[1],
                IRDetect[2],
                IRDetect[3],
                IRDetect[4] );

    	    fclose(f);	
    	}
    }

  }
}


// Launch autonomy thread
void* launch_autonomy() {
    //char* tempCommand = malloc(sizeof(char)*12);
    //strncpy(&tempCommand[0], &request_info->query_string[0], 11);
    //tempCommand[11] = 0;

  printf("Starting autonomy\n");
  printf("Autonomy: Driving forward.\n");
  while(1) {
    // Get data from the variables while the mutex is locked
    pthread_mutex_lock( &sensorDataMutex );
    int tmpRange = range;
    int tmpBearing = bearing;
    int tmpPitch = pitch;
    int tmpRoll = roll;
    pthread_mutex_unlock( &sensorDataMutex );

    // Check for forward obstacles.  Ranges <10 are errors, so ignore them.
    if ((tmpRange < 120) && (tmpRange > 10)) {
      printf("Autonomy: Forward obstacle detected.\n");
      autonomySendCommand("00000000");
      usleep(500000);
      printf("Autonomy: Reversing...\n");
      autonomySendCommand("01000000");
      usleep(500000);
      autonomySendCommand("00000000");
      usleep(500000);
      printf("Autonomy: Shooting...\n");
      autonomySendCommand("00000001");
      usleep(2000000);
      autonomySendCommand("00000000");
      usleep(500000);
      printf("Autonomy: Turning...\n");
      autonomySendCommand("00010000");
      usleep(2000000);
      autonomySendCommand("00000000");
      printf("Autonomy: Recheck Pause...\n");
      usleep(2000000);
      printf("Autonomy: Recheck Pause complete.\n");
      printf("Autonomy: Driving forward.\n");
    }
    else
    {
      //printf("Autonomy: Driving forward.\n");
      autonomySendCommand("10000000");
    }

    // Don't need to run that fast, sensor polling is pretty slow anyway.
    usleep(100000);
  }
}

void* autonomySendCommand(char* cmd) {
  pthread_mutex_lock( &autonomyCommandMutex );
  strncpy(&autonomyCommand[0], &cmd[0], 8);
  autonomyCommand[9] = 0;
  pthread_mutex_unlock( &autonomyCommandMutex );
  //printf("Autonomy set motion command: %.*s\n", 8, autonomyCommand);
}


char IRInputStr[][20] =
{
    "Left",
    "Back",
    "Right",
    "Invalid"
};

char IRTypeStr[][20] = 
{
    "",
    "LG/Humax",
    "RoboRover",
    "Panasonic",
    "I-Cybie",
    "Invalid"
};

void parse_ubw( char c )
{
  unsigned int IRValue;

  if (c < 0x20)
  {
     ubwinputstore[ubwcount] = 0;
     if (ubwcount)
     {
     switch( ubwinputstore[0] )
     {
     case 'A':
        NumAnalogueValues = sscanf( ubwinputstore, "A,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
		&AnalogueValues[0],
		&AnalogueValues[1],
		&AnalogueValues[2],
		&AnalogueValues[3],
		&AnalogueValues[4],
		&AnalogueValues[5],
		&AnalogueValues[6],
		&AnalogueValues[7],
		&AnalogueValues[8],
		&AnalogueValues[9],
		&AnalogueValues[10],
		&AnalogueValues[11],
		&AnalogueValues[12]);
        printf( "%s\n", ubwinputstore );
        break;

     case 'I':
        {
        if (ubwinputstore[1] == 'R')
            {
            int IRInput = 0;
            int IRPeriod = 0;

            int IRBits1 = 0;
            int IRBits2 = 0;

            int IRType = 0;
            NumDigitalValues = sscanf( ubwinputstore, "IR%d,T=%d,Type=%d,bits=%d:%d %X",
                &IRInput,
                &IRPeriod,
                &IRType,
                &IRBits1,
                &IRBits2,
                &IRValue);
            printf( "%s\n", ubwinputstore );
            printf( "-> IRIn %s; IRType %s; IRVal %X\n", IRInputStr[IRInput], IRTypeStr[IRType], IRValue );
            sprintf(LastIR[IRInput], "%s:%08.8X", IRTypeStr[IRType], IRValue );

            //if (IRValue == 0xd555)
            if (IRValue == 0x9111)
                {
                IRDetect[4]+=5;
                }

            //if (IRValue == 0x9111)
            if (IRValue == 0xB333)
                {
                if (IRInput == 0)
                    IRDetect[0]+=5;
                if (IRInput == 2)
                    IRDetect[1]+=5;
                }
            
            //if (IRValue == 0xB333)
            if (IRValue == 0xD555)
                {
                if (IRInput == 0)
                    IRDetect[2] += 5;
                if (IRInput == 2)
                    IRDetect[3] += 5;
                }
            }
        else
            {
            NumDigitalValues = sscanf( ubwinputstore, "I,%d,%d,%d",
                &DigitalValues[0],
                &DigitalValues[1],
                &DigitalValues[2]);
            printf( "%s\n", ubwinputstore );
            }
        }
        break;

     case 'P':
	if (ubwinputstore[1] == 'I')
        {
           // read value
        }
        printf( "%s\n", ubwinputstore );
        break;

     case '!':
        printf( "%s\n", ubwinputstore );
        break;

     default:
        printf( "[%s]???\n", ubwinputstore );
        break;
     }
     }
     ubwcount = 0;
  }
  else
  {
     if (ubwcount < 254)
        ubwinputstore[ubwcount++] = c;
  }
}