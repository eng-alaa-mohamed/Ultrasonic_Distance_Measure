
/* Include the HI-TECH Universal Tool suite Header File
Name: Alaa Mohamed  ID: W0411797
Name: Andrew Kim    ID: W0414568
Date: Dec 16, 2019   
ULTRASONIC DISTANCE MEASUREMENT PROJECT
EETD 5000 - Microcontroller Applications
The objective of this program is to build a code for portable Ultrasonic Distance Measurement device. 
The core functionality of the device will be perimeter detection of objects
in four directions (Front, Back, Left and Right). Four ultrasonic sensors will be utilized to continuously
detect objects and measure the distance from the device. Of the object detected by the device, the closest
object will take precedence and will be displayed on an LCD module indicating the direction and distance
of the object. The distance displayed shall be measured in both metric and imperial units which can be
selected using a switch on the device. To enhance user usability distance displayed will also have automatic
unit conversion for distances greater than 100 cm or 12 inches to meters and feet respectively depending
on the selected distance unit. In addition to the LCD display, LED will also light up in the direction of the
closest object for immediate visual verification. An audible verification (buzzer sound) will also be
implemented to notify the user when any object gets closer than 15 cm or 6 inches.. In accordance to 
the main functionality of the device stated above, user will also be able to enhance device accuracy by
selecting the calibration mode to reduce and adjust the margin of error that may be present for each sensor.
*/

// **********************************************************
// Completed for the 40 pin DIL PIC18F45K20 Microcontroller.
// **********************************************************

//*********************************************************************
// Configures the PIC for Low Voltage Programming Disabled, Brown Out Enable to Off, 
// Oscillator to internal with I/O function on RA6 and RA7, Watchdog Timer disabled and 
// Power Up Timer Enabled
//*********************************************************************
#include <htc.h>
#include <string.h>
#include <stdio.h>
//*********************************************************************
// Configures the PIC for Low Voltage Programming Disabled, Brown Out Enable to Off, 
// Oscillator to internal with I/O function on RA6 and RA7, Watchdog Timer disabled and 
// Power Up Timer Enabled
//*********************************************************************
#include <xc.h>

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1H
#pragma config FOSC = INTIO67   // Oscillator Selection bits (Internal oscillator block, port function on RA6 and RA7)
#pragma config WDTEN = OFF       // Watchdog Timer Enable bit (WDT is always Disabled.)
#pragma config LVP = OFF         // Single-Supply ICSP Enable bit (Single-Supply ICSP Disabled)
#define _XTAL_FREQ 16000000
//*********************************************************************
// Function Prototypes:
// Defines the functioned used by main()
// Arguments: 
// Returns:
//*********************************************************************
void Initialize();                          // Function to set the PIC for the desired operation i.e. AtoD Conversion
int GetAnalogVoltage(char);			        // Function that gets the converted representation of the 4 analog inputs
static int GetClosest(int direction[4]);    // Function that gets the lowest converted digital out of the 4 converted digital representation
void GetCalibrated(int[]);                 // Function that gets delta values for calibration values 
void SetOutput(int);                       // Function that converts digital value to distance
void LCD_direction();                      // Function that shows direction of the closest object on LCD  
void LCD_distance(float dist, char un);    // Function that shows distance of the closest objet and the measurement unit on LCD  
void Lcd_Cmd(char);
	 
//*********************************************************************
// Main Function - Calls all the functions required to perform the 
// AtoD Conversion for 4 inputs, calibrate 4 digital converted, determine the lowest digital signal
// Convert the lowest to distance and show it on LCD in addition to the direction of the closest object   
// Arguments: None
// Returns: None
//*********************************************************************
void main()
{
    int mdir[]={0,0,0,0};   		// define array of 4 elements with zero initial values that holds converted 4 digital values 
    int delta[]={0,0,0,0}; 			// define array of 4 elements with zero initial values that holds 4 delta values for calibration  
    int closest;                    // Integer variable that holds the closest (lowest digital value)
    char i, j;					    // define 2 char varibales used for iteration 
    Initialize();                   // Call the Initialize function for all your one time setups
	while(1) 						// Infinite loop 
	{
        for(i=0;i<4;i++)   
            mdir[i] = GetAnalogVoltage(i);   // Calling function: GetAnalogVoltage 4 times, one for each direction then store in array of 4 elements
        while(RB1)                          // Calibration switch is ON      
            GetCalibrated(delta);			// Calling function: GetCalibrated
        for(j=0;j<4;j++)
            mdir[j] = mdir[j] + delta[j];  // Adding the 4 delta values from calibration process to original converted values  
        closest = GetClosest(mdir);	      // Calling function: GetAnalogVoltage
        SetOutput(mdir[closest]);	      // Calling function: SetOutput
	}
}	

//*********************************************************************
// Function name: GetAnalogVoltage
// Function to sample the input voltage based on input argument which ranges between 1 to 4 
// and convert it to a 10-Bit representation stored in ADRESL & ADRESH
// a variable ip is defined to have the ADCON0 value corresponding to the required analog input
// The value is added and stored in a single variable dist
// Arguments: ip
// Returns: dist
//*********************************************************************
int GetAnalogVoltage(char ip)
{
    int dist;                       // Define an integer variable to hold converted digital distance  
    if(ip == 0)
        ADCON0 = 0x00;   			// ADCON0 - Analog input (RA0), ADC off, and GO_DONE = 0 (default value while switch is open))
    else if(ip == 1)
        ADCON0 = 0x05;				// ADCON0 - Analog input (RA1), ADC off, and GO_DONE = 0 (default value while switch is open))
    else if(ip == 2)
        ADCON0 = 0x08;				// ADCON0 - Analog input (RA2), ADC off, and GO_DONE = 0 (default value while switch is open))
    else if(ip == 3)
        ADCON0 = 0x0C;				// ADCON0 - Analog input (RA3), ADC off, and GO_DONE = 0 (default value while switch is open))

    ADON = 1;                             // Turn the AtoD converter on
    GO_DONE = 1;                         // Set GO_DONE bit to start AtoD conversion
    while (GO_DONE==1);                  // Wait for GO_DONE bit to go low signaling conversion is complete
    dist = (ADRESH * 256) + ADRESL;      // Store the ADRESH and ADRESL values in ADRegVal (10-bit)
     ADON = 0;                          // Turn AtoD Converter off
   return dist; 						// return converted digital distance  
}
//*********************************************************************
// Function name: GetCalibrated
// Function to increment or decrement delta values to be added to original converted digital values 
//for the purpose of calibration measurements based on switches increment or decrement and direction calibration switch  
// Arguments: cal[]
// Returns: None
//*********************************************************************
void GetCalibrated(int cal[])
{
    int val;  			  // define integer variable to hold non-calibrated conversion values  
    int calval=0;        // define integer variable to hold calibrated conversion values with zero initial value
    if (RB5)             //Front calibration switch is ON   
    {
       val = GetAnalogVoltage(0);  // move stored digital value corresponding to 1st input analogue input to val 
        if (RB2==1)   		// Increment switch is ON 
            cal[0]++;       // increment cal[0] (delta)
        if (RB3==1)  		// decrement switch is ON 
            cal[0]--;  			// decrement cal[0] (delta)
       while(RB2 || RB3);       // Momentarily switch to increment/decrement only one bit at a time 
       RD0=1;                  // Turn on Front LED 
       RD1=0; RD2=0; RD3=0;     // Turn off other LEDs
       calval = val+cal[0];   // Add non-calibrated to cal (delta) and move it to a new variable calval
       SetOutput(calval);   // call function SetOutput with calval argument to show distance on screen during calibration 
    }
    else if (RC3)         // Back calibration switch is ON  
    {
       val = GetAnalogVoltage(1);// move stored digital value corresponding to 2nd input analogue input to val 
        if (RB2==1)
            cal[1]++;
        if (RB3==1)   
            cal[1]--;
       while(RB2 || RB3);
       RD1=1;					 // Turn on back LED 
       RD0=0; RD2=0; RD3=0;		// Turn off other LEDs
       calval = val+cal[1];
       SetOutput(calval);
    }	
    else if (RB4)         // Right calibration switch is ON  
    {
       val = GetAnalogVoltage(2);// move stored digital value corresponding to 3rd input analogue input to val 
        if (RB2==1)  
            cal[2]++;
        if (RB3==1)
            cal[2]--;				
       while(RB2 || RB3);
       RD2=1; 					// Turn on right LED 
       RD0=0; RD1=0; RD3=0;     // Turn off other LEDs
       calval = val+cal[2];
       SetOutput(calval);
    }
    else if (RC2)         // Left calibration switch is ON  
    {
       val = GetAnalogVoltage(3); // move stored digital value corresponding to 4th input analogue input to val 
        if (RB2==1) 
            cal[3]++;
        if (RB3==1)
            cal[3]--;				
       while(RB2 || RB3);
       RD3=1; 					// Turn on Left LED 
       RD0=0; RD1=0; RD2=0;    // Turn off other LEDs
       calval = val+cal[3];
       SetOutput(calval);
    }
    
}	
//*********************************************************************
// Function name: GetClosest
// Function to determine the smallest digital values out of the 4 converted digital values to 
// determine the direction of closest object then return array index of the smallest value and light up LED corresponding to this direction  
// Arguments: direction[]
// Returns: array index []
//*********************************************************************
static int GetClosest(int direction[])
{
    if (direction[0]< direction[1] && direction[0]< direction[2] && direction[0]< direction[3])
	{
		RD0=1;
	    RD1=0; RD2=0; RD3=0; 
		return 0;                          // Return array index 0 to the calling function
	}
    else if (direction[1]< direction[0] && direction[1]< direction[2] && direction[1]< direction[3])
    {
		RD1=1;
        RD0=0; RD2=0; RD3=0; 
        return 1;					// Return array index 0 to the calling function
	}  
    else if (direction[2]< direction[0] && direction[2]< direction[1] && direction[2]< direction[3])
    {
		RD2=1;
        RD0=0; RD1=0; RD3=0; 
        return 2;				 // Return array index 0 to the calling function
	}	
    else if (direction[3]< direction[0] && direction[3]< direction[1] && direction[3]< direction[2])
	{
		RD3=1;
        RD0=0; RD1=0; RD2=0; 
        return 3;				// Return array index 0 to the calling function
	}	
}

//*********************************************************************
// Function name: SetOutput - to Set the output port based on the AtoD Conversion displaying 
// the distance of closest object and its direction by calling distance and direction function 
// The displayed distance depends on unit selection switch 
// in addition to enable buzzer if object distance is less 6 inches or 15 cm   
// Arguments: ADPortVal
// Returns: None
//*********************************************************************
void SetOutput(int ADPortVal)
{
    float value, distance;		// Define a variable to hold the index for the Light[] array 
    //float LSB= 0.0048828125;   		// Calculate the resolution of the LSB for an 8 bit word and a 5 volt reference
	float LSB= (3.3/1024); 				// Calculate the resolution of the LSB for an 8 bit word and a 3.3 volt reference
    value = ADPortVal * LSB;    	// calculate value of the analog input (i.e. LSB* Digital word = analog input)
	distance = (value /0.0098);    	// calculate distance in inches based on resolution of sensor is 9.8mv/inch 
	char unit [6];                  // define array thats holds unit name (strings)  
    if (distance <=6)              // Buzzer alarms if distance is less than 6 inches
        RD4=1;               // Buzzer alarms   
    else
        RD4=0;
		// Unit conversion and distance display
	if (RB0)  // metric switch ON
    {
        distance = distance * 2.55;    	 // calculate distance in centimeters
        if (distance <100)              // if distanc < 100cm    
        {   
            strcpy(unit, "cm");          // copy string cm to array "unit"
        }
        else if (distance >= 100)           // if distanc > 100cm 
        {    
            distance = (distance/100);    // calculate distance in meters
            strcpy(unit, "m");            // copy string m to array "unit"
        }
    } 
    
	else 								// metric switch OFF
    {
        if (distance <12)             
        {   
            strcpy(unit, "in");				// copy string m to array "unit"
        }
        else if (distance >= 12)           // if distanc > 12ft 
        {    
            distance = (distance/12);     // calculate distance in feet
            strcpy(unit, "ft");			// copy string m to array "unit"
        }     
    }
    // Direction display 
	Lcd_Cmd(0x0c);                    // put cursor left 
    LCD_direction();                 // call direction function
	Lcd_Cmd(0x94);                  // new line 
    LCD_distance(distance, unit);    // call distance function
}
//*********************************************************************
// Function name: LCD_distance 
// to show distance and unit on LCD screen by enabling USART Transmitter  
// Arguments: dist and un 
// Returns: None
//*********************************************************************
void LCD_distance(float dist, char un)
{
 	float k;
    char u;
    k=dist;
    u = un;
    TXEN = 1;			// Set to 1 to enable the USART Transmitter
    printf("Distance:%0.2f%s", k, u);
    __delay_ms(300);
	TXEN = 0;			// Disable the Transmitter
}
//*********************************************************************
// Function name: LCD_direction 
// to show direction on LCD screen by enabling USART Transmitter based on 
// LED in case of measurement mode or based on direction switches in calibration mode   
// Arguments: None
// Returns: None
//*********************************************************************
void LCD_direction()
{
 	char dir[6];
	if (RD0 || RB5)
        strcpy(dir, "Front");
	else if (RD1 || RC3)
        strcpy(dir, "Back");
	else if (RD2 || RB4)
        strcpy(dir, "Right");
	else if (RD3 || RC2)
        strcpy(dir, "Left");
	TXEN = 1;			// Set to 1 to enable the USART Transmitter
	printf("Direction: %s", dir);  // Display direction 
    __delay_ms(20);
	TXEN = 0;			// Disable the Transmitter
}
//*********************************************************************
// Function name: Lcd_Cmd  
// Arguments: cmd
// Returns: None
//*********************************************************************
void Lcd_Cmd(char cmd)
{
    char j;
    j=cmd;
    TXEN = 1;
	while(!TXIF);	  // Continue to loop while the TXREG still has data
	TXREG = j; 	      // Once TXREG has been cleared, load it with new line
	__delay_ms(10);
    TXEN = 0;
 }
//*********************************************************************
// Function to initialize the PIC Controller for AtoD conversion, USART 
// Function Initialize() definition: set conversion clock speed to 16 MHz 
// Select 4 analog inputs RA0, RA1, RA2, RA3 
// Set the voltage reference to Vdd/Vss 
// Set the justification for the ADRESH & ADRESL registers
// to set the registers for the outputs to PORTD
// Arguments: None
// Returns: None
//*********************************************************************
void Initialize()
{
	
    OSCCON = 0x7F;	// Sets up internal oscillator for 16 MHz.
    INTCON = 0x00;  // Disable interrupts
	// LCD Initiation function
    OSCTUNE = 0;	// Used to tune the frequency of the internal oscillator.
	BRGH = 1; 		// TXSTA Reg, Chooses high speed Baud rate of development
	SYNC = 0;		// TXSTA, Set the TX Control to asynchronous mode.
	BRG16= 0;		// BAUDCTL reg. 8-Bit Baud generator is used
	SPBRG = 103;	// Baud selection for 9600. Obtained from the BAUD Rates for asynchronous modes tables
	INTCON = 0x00;	// INTCON, disables interrupts General an peripheral.
	RX9 = 0;		// RCSTA, Sets 8 BIT Reception
	TX9 = 0;		// TXSTA, Sets 8 Bit Transmission
	SPEN = 1;		// RCSTA, Serial Port enabled
 	CREN = 0;		// RCSTA, continuous receive disabled
    // Set up registers for the analog inputs (4 ultrasonic sensors)
    TRISA0 = 1;                // assign analog input pin RA0 (Front)
    ANS0 = 1;                // Set input port to analog - RA0
    TRISA1 = 1;                // assign analog input pin. RA1 (Rear)
    ANS1 = 1;                // Set input port to analog - RA1 
    TRISA2 = 1;                // assign analog input pin RA2 (Right)
    ANS2 = 1;                // Set input port to analog - RA2
    TRISA3 = 1;                // assign analog input pin RA3 (left)
    ANS3 = 1;                // Set input port to analog - RA3
    // Set up registers for the A2D
    ADCON0 = 0x00;           // ADCON0 - Analog input 0 (RB3), ADC off, and GO_DONE = 0 (default value while switch is open))
    ADCON1 = 0x00;                // ADCON1 - Reference Voltage set to Vdd/Vss
    ADCON2 = 0xA5;                // ADCON2 - Right Justified, Acquisition time of 8 TAD (8uS) and F(osc)/16)
	// Set up register for the digital outputs (4 LEDs & 1 Buzzer) 
	TRISD0=0;
	TRISD1=0;
	TRISD2=0;
	TRISD3=0;
	TRISD4=0;
    RD0=0;         // Initialize output to zero value  
    RD1=0;
    RD2=0;
    RD3=0;
    RD4=0;
	// Set up register for 4 digital inputs (4 Switches)for unit selection, calibration, increment & decrement 
	TRISB0=1;        // assign digital input pin RB0 - unit selection 
	ANS12=0;
	TRISB1=1;		// assign digital input pin RB1 - calibration  
	ANS10=0;
	TRISB2=1;		// assign digital input pin RB2 - increment  
	ANS8=0;
	TRISB3=1;		// assign digital input pin RB3 - decrement 
	ANS9=0;
    // Set up register for 4 digital inputs (Switches)- direction selection for calibration 
    TRISB4=1;      // assign digital input pin RB4
    ANS11=0;
    TRISB5=1;      // assign digital input pin RB4
    TRISC2=1;		// assign digital input pin RB4
    TRISC3=1;		// assign digital input pin RC3
}