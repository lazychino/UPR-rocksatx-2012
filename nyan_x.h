// Author: Pedro A. Melendez
// nyan.h: function library for the upr rocksatX 2012 payload

extern File myFile;

void parallelLine( const char state )  // receives a byte and send it via parallel line
{
	digitalWrite(35, state & 0x1 ); // parallel status BOOT
	digitalWrite(34, (state & 0x2)>>1 );
	digitalWrite(33, (state & 0x4)>>2 );
	digitalWrite(32, (state & 0x8)>>3 );
	digitalWrite(31, (state & 0x10)>>4 );
	digitalWrite(30, (state & 0x20)>>5 );
	digitalWrite(29, (state & 0x40)>>6 );
	digitalWrite(28, (state & 0x80)>>7 );
}

void sdInit()
{
    // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
    // Note that even if it's not used as the CS pin, the hardware SS pin
    // (10 on most Arduino boards, 53 on the Mega) must be left as an output
    // or the SD library functions will not work.
    pinMode(53, OUTPUT);

    if (!SD.begin(53)) {
    Serial.println("initialization failed!");
    return;
    }
    Serial.println("initialization done.");
}

bool RGA_boot( HardwareSerial rga ) // with baudrate of 57600 boot time is 34 seconds
{
	rga.begin(9600);				//open serial port
	myFile = SD.open("qpbox.l2");  	// open RGA firmware file
	if( myFile )
	{
		String *packet = new String("");
		String *rx = new String("");
		char *chr = new char;

		for(unsigned int i=0; i < 1000; i++)  // send 1000 zero bytes to reset CCU
		{
			int zero = 0;
			rga.write(zero);
		}

		while( !rga.available() ) {} // wait for 0xAC
		rga.flush();

		//Serial.println("starting init");
		int packetCount = 0;
		while( myFile.available() )
		{
			*chr = myFile.read(); // read packet from file
			*packet += *chr;
			if( *chr == '}'  ) // after is packet read from file is sent to the CCU
			{
				//Serial.println(*packet);
				rga.print(*packet);
				*packet = "";
				packetCount++;

				while( !rga.available() ){} // wait for package receive confirmation
				delay(100);		   			 // delay to ensure the complete message read

				while(rga.available())
				{
					*rx += (char)rga.read();
				}

                                if( packetCount == 2 )
                                {
                                        //Serial.println(*rx);
					 if( *rx != "{Init=1}") // need to check if packet receive is the correct one
						return false;
                                }

				else if( packetCount == 3)
				{
                                        rga.begin(57600); // after sending packet to change baudrate change arduino baudrate
				}

				else if(packetCount == 40) // read firmware checksum
				{
					while( myFile.available() )
					{
						rga.print( (char)myFile.read() );
					}
					break;
				}
				else{} //nothing
				*rx = "";
			}
		}
		myFile.close();
		delete packet;
		delete rx;
		delete chr;
		rga.print("{Go}");   	// start firmware
		delay(5000);			// wait for it to boot
		rga.println("set:Filament:0");	// turn off filament
		//Serial.println("RGA initialized");
		return true;
	}
	else
	{
		Serial.println("error opening opening qpbox.l2");
		return false;
	}
}

void RGA1_config()
{

	HardwareSerial rga = Serial1;
	
	rga.println("set:ScanSpeed:48");
	rga.println("set:LowMass:1");
	rga.println("set:HighMass:200");
	rga.println("set:SamplesPerAmu:6");
	rga.println("set:Focus1Volts:-20");
	rga.println("set:LowCalMass:1");
	rga.println("set:LowCalResolution:630");
	rga.println("set:LowCalPosition:0.25");
	rga.println("set:LowCalIonEnergy:4.5");
	rga.println("set:HighCalMass:200");
	rga.println("set:HighCalResolution:1475");
	rga.println("set:HighCalPosition:0.65");
	rga.println("set:HighCalIonEnergy:5.000");
	rga.println("set:TotalOffset:2000");
	rga.println("set:PartialOffset:2200");
	rga.println("set:TotalCapPf:10.00");
	rga.println("set:PartialCapPf:3.000");
	rga.println("set:RfSettleTicks:50");
	rga.println("set:SwSettleTicks:10");
	rga.println("set:Pirani1ATM:2.27900");
	rga.println("set:PiraniZero:0.34800");
	rga.println("set:PartialSensitivity:6.00e-004");
	rga.println("set:TotalSensitivity:1.00e+001");
/*

3272 

ScanParameters
      ScanSpeed="144"
      LowMass="1"
      HighMass="210"
      SamplesPerAMU="10"
OperatingParameters
      Mode="Mass sweep"
      Focus1="-20"
      Focus2="-72"
      ElectronEnergy="70"
      FilamentEmission="2.0"
      AutoZero="Off"
      ScanMode="Sweep"
      Filament="1"
      PressureUnits="Torr"
      EnableElectronMultiplier="0"
      MultiplierVoltage="0"
CalibrationParameters
      LowCalMass="1"
      LowCalResolution="630"
      LowCalPosition="0.25"
      LowCalIonEnergy="4.5"
      HighCalMass="200"
      HighCalResolution="1475"
      HighCalPosition="0.65"
      HighCalIonEnergy="5.0"
      TotalAmpOffset="2000"
      PartialAmpOffset="2200"
      TotalIntegratingCap="10.00"
      PartialIntegratingCap="3.00"
      RFSettleTime="50"
      SWSettleTime="10"
      Pirani1ATM="2.27900"
      PiraniZero="0.34800"
      PiraniAutoRecalibrate="0"
      PartialSensitivity="6.00e-004"
      TotalSensitivity="1.00e+001"
      debug="0"	
*/
}

void RGA2_config()
{
	HardwareSerial rga2 = Serial2;
	
	rga2.println("set:ScanSpeed:48");
	rga2.println("set:LowMass:1");
	rga2.println("set:HighMass:200");
	rga2.println("set:SamplesPerAmu:6");
	rga2.println("set:Focus1Volts:-15");
	rga2.println("set:LowCalMass:1");
	rga2.println("set:LowCalResolution:655");
	rga2.println("set:LowCalPosition:0.28");
	rga2.println("set:LowCalIonEnergy:4.5");
	rga2.println("set:HighCalMass:200");
	rga2.println("set:HighCalResolution:1000");
	rga2.println("set:HighCalPosition:0.56");
	rga2.println("set:HighCalIonEnergy:5.000");
	rga2.println("set:TotalOffset:2000");
	rga2.println("set:PartialOffset:2200");
	rga2.println("set:TotalCapPf:10.00");
	rga2.println("set:PartialCapPf:3.000");
	rga2.println("set:RfSettleTicks:50");
	rga2.println("set:SwSettleTicks:10");
	rga2.println("set:Pirani1ATM:2.20200");
	rga2.println("set:PiraniZero:0.35900");
	rga2.println("set:PartialSensitivity:6.00e-004");
	rga2.println("set:TotalSensitivity:1.00e+001");
	rga2.println("set:TotalSensitivity:1.00e+001");
        rga2.println("set:BaudRate:19200");
        delay(1000);
        rga2.begin(19200);
        rga2.println("symbols");
        
	
/*   3273 calibration parameters
 *
ScanParameters
      ScanSpeed="144"			set:ScanSpeed:144
      LowMass="1"			set:LowMass:1
      HighMass="210"			set:HighMass:210
      SamplesPerAMU="10"		set:SamplesPerAmu:10
OperatingParameters
      Mode="Mass sweep"
      Focus1="-15"			set:Focus1Volts:-15
      Focus2="-72"
      ElectronEnergy="70"		set:ElectronVolts:70
      FilamentEmission="2.0"		set:FilamentEmissionMa:2.000
      AutoZero="Off"			:AutoZero:0
      ScanMode="Sweep"
      Filament="1"			:Filament:1
      PressureUnits="Torr"
      EnableElectronMultiplier="0"
      MultiplierVoltage="0"		:MultiplierVolts:0
CalibrationParameters
      LowCalMass="1"			set:LowCalMass:1
      LowCalResolution="655"		set:LowCalResolution:655
      LowCalPosition="0.28"		set:LowCalPosition:0.28
      LowCalIonEnergy="4.5"		set:LowCalIonEnergy:4.5
      HighCalMass="200"			set:HighCalMass:200
      HighCalResolution="1000"		set:HighCalResolution:1000
      HighCalPosition="0.56"		set:HighCalPosition:0.56
      HighCalIonEnergy="5.0"		set:HighCalIonEnergy:5.000
      TotalAmpOffset="2000"		set:TotalOffset:2000
      PartialAmpOffset="2200"		set:PartialOffset:2200
      TotalIntegratingCap="10.00"	set:TotalCapPf:10.00
      PartialIntegratingCap="3.00"	set:PartialCapPf:3.000
      RFSettleTime="50"			set:RfSettleTicks:50
      SWSettleTime="10"			set:SwSettleTicks:10
      Pirani1ATM="2.20200"		set:Pirani1ATM:2.20200
      PiraniZero="0.35900"		set:PiraniZero:0.35900
      PiraniAutoRecalibrate="0"		
      PartialSensitivity="6.00e-004"	set:PartialSensitivity:6.00e-004
      TotalSensitivity="1.00e+001"	set:TotalSensitivity:1.00e+001
      debug="0"
 */
}

void openArms()
{
	int revolutions = 75;
	digitalWrite(38, HIGH);							// set direction outward
	int period = 175;							// set initial speed
	digitalWrite(39, LOW);							// enable board
	for(int h = 0 ; h < revolutions ; h++)
	{
		for(long i=0; i < 3200; i++)					// this for performs one revolution or 3200 microsteps
		{
			delayMicroseconds(10);					//one square pulse is send to the driver
			digitalWrite(37, HIGH);
			delayMicroseconds(10);
			digitalWrite(37, LOW);
			delayMicroseconds(period);

			if( period > 60  && i%200 == 0 && h < revolutions/4 )	// on the first revolutions
			 	period--;					// the period of the pulse is decrease to acelerate

			else if( h == revolutions-1 && i%20 == 0)		// on the last revolution period
				period++;					// increase to deacelerate
			else{}
		}

	}
	digitalWrite(39, HIGH);							// disable board
}

void closeArms()
{
	int revolutions = 75;
	digitalWrite(38, LOW);							// set direction inward
	int period = 175;							// set initial speed
	digitalWrite(39, LOW);							// enable board
	for(int h=0; h < revolutions; h++)					// for each revolution the linear actuartor move 0.03139"
	{
		for(long i=0; i < 3200; i++)					// this for performs one revolution or 3200 microsteps
		{
			delayMicroseconds(10);					//one square pulse is send to the driver
			digitalWrite(37, HIGH);
			delayMicroseconds(10);
			digitalWrite(37, LOW);
			delayMicroseconds(period);

			if( period > 60  && i%200 == 0 && h < revolutions/4 )	// on the first revolutions
			 	period--;					// the period of the pulse is decrease to acelerate

			else if( h == revolutions-1 && i%20 == 0)		// on the last revolution period
				period++;					// increase to deacelerate
			else{}
		}
	}
	digitalWrite(39, HIGH);							// disable board
}

void closeArmsSlow()
{
	int revolutions = 400;
	digitalWrite(38, LOW);							// set direction inward
	int period = 400;							// set initial speed
	digitalWrite(39, LOW);							// enable board
	for(int h=0; h < revolutions; h++)					// for each revolution the linear actuartor move 0.03139"
	{
		for(long i=0; i < 3200; i++)					// this for performs one revolution or 3200 microsteps
		{
			delayMicroseconds(10);					//one square pulse is send to the driver
			digitalWrite(37, HIGH);
			delayMicroseconds(10);
			digitalWrite(37, LOW);
			delayMicroseconds(period);
		}
                
                
	}
	digitalWrite(39, HIGH);							// disable board
}

void initPins()
{
/*
 * INITIALIZE ARDUINO PINS SIGNALS FOR CONTROLLERS
 */
	pinMode( 7, INPUT ); //Remove before flight connector signal
        
	pinMode( 39, OUTPUT );	// BigEasy driver - enabled default HIGH
	digitalWrite( 39, HIGH );

	pinMode( 37, OUTPUT );	// BigEasy driver - step
	pinMode( 38, OUTPUT );	// BigEasy driver - dir

	pinMode( 25, OUTPUT );	// Valve Controller - IN 1 & IN 3
	pinMode( 26, OUTPUT );	// Valve Controller - Enable A & Enable B
	pinMode( 27, OUTPUT );	// Valve Controller - IN 2 & IN 4

	/* parallel telemetry */
  	pinMode( 28, OUTPUT );
	pinMode( 29, OUTPUT );
	pinMode( 30, OUTPUT );
	pinMode( 31, OUTPUT );
	pinMode( 32, OUTPUT );
	pinMode( 33, OUTPUT );
	pinMode( 34, OUTPUT );
	pinMode( 35, OUTPUT );
	pinMode( 36, INPUT  );
/*
 * END OF INITIALIZATION
 */
}

void FLUSH( HardwareSerial port)
{
        char dummy;
	while( port.available() )
	{
		dummy = port.read();
	}
}



void open_valve() {
  	boolean valve_1_open = false;
  	boolean valve_2_open = false;
  	boolean failsafe = true; 		// variable used if the encoders dont work 
  	int maxnumber_1 = 900;			// variable for the max number of the encoder 1
 	int maxnumber_2 = 900;			// variable for the max number of the encoder 2 
  	unsigned long timer = millis()+3000; 	// varible used to stop valve if the encoders malfunction 

    	digitalWrite(26,HIGH);	// turns valve 1 on 
    	digitalWrite(44,HIGH);	// turns valve 2 on 
    	digitalWrite(27,HIGH);	// sends the direction of the valves
        delay(1000);
         
  	while( millis()<timer ) {
    
	    	if(analogRead(A3)> maxnumber_1) {
	       		valve_1_open = true ; 
	       		digitalWrite(26,LOW);// turns off valve 1
	    	}
	    	if(analogRead(A4)> maxnumber_2){
	     		valve_2_open = true;
	      		digitalWrite(44,LOW);// turns off valve 2
	    	}
	    	if(valve_1_open && valve_2_open){
	      		failsafe = false; 
	       		digitalWrite(27,LOW);
	      		break;
	    	}
  	}
  	if(failsafe){
    		digitalWrite(26,LOW);	// turns valve 1 off
    		digitalWrite(44,LOW);	// turns valve 2 off
    		digitalWrite(27,LOW);	// off direction of the valves 
  	}
}

void close_valve() {
  	boolean valve_1_close = false;
  	boolean valve_2_close = false;
  	boolean failsafe = true; 
  	int maxnumber_1 = 900;	//variable for the max number of the encoder 1
  	int maxnumber_2 = 900;		//variable for the max number of the encoder 2 

  	unsigned long timer = millis()+5000; // varible used to stop valve if the encoders malfunction 

    	digitalWrite(26,HIGH);// turns valve 1 on 
    	digitalWrite(44,HIGH);// turns valve 2 on 
    	digitalWrite(25,HIGH);// sends the direction of the valves 
        delay(1000);
  	while( millis()<timer ) {

    		if(analogRead(A3)> maxnumber_1) {
       			valve_1_close = true ;
                        delay(330);
       			digitalWrite(26,LOW); // turns off valve 1
   		}
    		if(analogRead(A4)> maxnumber_2) {
      			valve_2_close = true;
                        delay(330);
     			digitalWrite(44,LOW);// turns off valve 2
    		}
	    	if(valve_1_close && valve_2_close){
	      		failsafe = false; 
	       		digitalWrite(25,LOW);
	     		break;
	    	}
  	}
  	if(failsafe) {
    		digitalWrite(26,LOW);// turns valve 1 on 
    		digitalWrite(44,LOW);// turns valve 2 on 
    		digitalWrite(25,LOW);// sends the direction of the valves 
  	}
}

