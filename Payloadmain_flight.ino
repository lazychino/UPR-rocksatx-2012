#include <Arduino.h>
#include <SD.h>
#include "nyan_x.h"


// timed events
#define openValves		60000
#define deployBooms 		73000
#define retractBooms		300000

File myFile;

void setup() 
{
//-------------- pre launch---------------------------
	
	initPins();
        while(digitalRead(7) == 0) { delay(100); }
	
        sdInit();       
	parallelLine( (char)1 );	// computer ready
        
        delay(10000);
        parallelLine( (char)2 );	// RGA1 booting up
	RGA_boot( Serial1 );

	parallelLine( (char)3 );	// RGA1 booted and being configure
	RGA1_config();

	parallelLine( (char)4 );	// RGA1 ready! RGA2 booting up
	RGA_boot( Serial2 );
		1
	parallelLine( (char)5 );	// RGA2 booted and being configure	
	RGA2_config();

	parallelLine( (char)6 );	// RGA2 ready! all systems ready for launch!

	while( analogRead(A1) < 1000 ){} 	// wait for launch signal

//--------------- After Launch ---------------------
	
	unsigned long launch = millis();	// set time events in reference to lauch
	parallelLine( (char)7 );
	unsigned long nextEvent = openValves + launch;
	FLUSH( Serial1 );
	FLUSH( Serial2 );

	while( millis() < nextEvent ){}
        parallelLine( (char)8 );		// opening valves
	nextEvent = deployBooms + launch;
	open_valve();
	//delay(3000);
        
	Serial1.println("set:Filament:1");
	Serial2.println("set:Filament:1");
	parallelLine( (char)9 );		// filaments on
	Serial2.println("sweep");
	
	
	while( millis() < nextEvent ){}
	nextEvent = retractBooms + launch;
	
	parallelLine( (char)10 );
	while( analogRead(A0) > 100 ){}		// wait for proximity sensor to sense no rocket skin
	openArms();
        //delay(25000);
	parallelLine( (char)11 );		// booms open
	
	Serial1.println("sweep");
	String inputString = "";
	while( millis() <  nextEvent )
	{
		if( Serial1.available() )
		{
	      		char inChar = (char)Serial1.read(); 
	      		inputString += inChar;

	      		if (inChar == '\n') 
	      		{
	      			myFile = SD.open("1.txt", O_CREAT | O_WRITE);
	      			myFile.print(inputString);
	      			myFile.flush();
					inputString = "";
	      		}
	      	}
	}
        myFile.close();
	Serial1.println("set:Filament:0");
	parallelLine( (char)12 );		// filament 1 off, retracting booms
	
	closeArms();
        //delay(25000);
	parallelLine( (char)13 );		// booms retracted, closing valves	

	close_valve();
        //delay(3000);
	parallelLine( (char)255 );

	// mission acomplish 
}

void loop()
{
}

