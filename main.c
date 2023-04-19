/* 
 *  (c) 2021 Microchip Technology Inc. and its subsidiaries.
 *
 *  Subject to your compliance with these terms, you may use Microchip software
 *  and any derivatives exclusively with Microchip products. You’re responsible
 *  for complying with 3rd party license terms applicable to your use of 3rd
 *  party software (including open source software) that may accompany
 *  Microchip software.
 * 
 *  SOFTWARE IS “AS IS.” NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
 *  APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF
 *  NON-INFRINGEMENT, MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
 *  
 *  IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
 *  INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
 *  WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
 *  BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
 *  FULLEST EXTENT ALLOWED BY LAW, MICROCHIP’S TOTAL LIABILITY ON ALL CLAIMS
 *  RELATED TO THE SOFTWARE WILL NOT EXCEED AMOUNT OF FEES, IF ANY, YOU PAID
 *  DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "main.h"
#include "tcb0.h"
#include "tcb1.h"

// the .rww_data section is at address 0x8000 (word address 0x4000) in this example
// this is done to uncomplicate the use of the unified data space
// it can be put anywhere from PROGMEM_RWW_START and up
#define RWW_DATA_SECTION __attribute__((used, section(".rww_data")))
// the .nrww_program section is at address 0x0400 (word address 0x0200)
// it can be put anywhere in PROGMEM_NRWW, without overlapping .text
#define NRWW_PROG_SECTION __attribute__((section(".nrww_program")))

// RWW variables
const RWW_DATA_SECTION uint8_t rww_array[RWW_DATA_SIZE] = {0};
uint8_t *flashWritePointer = (uint8_t *) &rww_array;
uint8_t *flashReadPointer = (uint8_t *) &rww_array;

volatile uint16_t flashReadPointer_addr = 0x00;
volatile uint16_t RWW_DATA_last_addr = 0x00;

// Buffer variables
uint8_t buffer[BUFFER_SIZE] = {0};
volatile uint8_t writeIndex = 0;
volatile uint8_t readIndex = 0;

typedef enum DATA_FLASH_enum
{
    IDLE = 0,
    INIT_NRWW,
    ERASE_NRWW,
    WRITE_NRWW,
    INIT_RWW,
    ERASE_RWW,
    WRITE_RWW,
} DATA_FLASH_t;

typedef enum BUTTON_enum
{
    UNKNOWN = 0,
    PRESS,
    DEPRESS
} BUTTON_t;

BUTTON_t buttonState = UNKNOWN; // Cleaned-up version of the SW0 button input signal

// Details of the algorithm can be found here:
// https://www.kennethkuhn.com/electronics/debounce.c

// TCB0 is set to interrupt with the SAMPLE_FREQUENCY rate.
// Within the interrupt SW0 logic level is sampled.
// A local variable is incremented or decremented according based on the SW0 state.
// The SW0 signal must be in a logical state (0 or 1) for the specified
// DEBOUNCE_TIME in order for the output to change to that state.
// DEBOUNCE_TIME is in seconds and SAMPLE_FREQUENCY is in Hertz
#define DEBOUNCE_TIME		0.1
#define SAMPLE_FREQUENCY	333
#define MAXIMUM			(DEBOUNCE_TIME * SAMPLE_FREQUENCY)

void SystemInitialize(void);
void ProgramingFlash(void);
void ReadWhileWriting(void);

int main(void)
{
    DATA_FLASH_t eraseWriteState = IDLE;
    
    // Pointer to rww flash program memory mapped in data space
    uint8_t *rwwFlashPointer;
    
    static BUTTON_t prev_buttonState = UNKNOWN;
    
    SystemInitialize();
	
	FillBuffer();

    

    while (1) 
    {
        switch (eraseWriteState)
        {
        case INIT_RWW:
            // Start TCB0  that will be filling up the data buffer
            TCB0.CTRLA |= TCB_ENABLE_bm; 
            
             // Set rwwFlashPointer address to RWW data space
             rwwFlashPointer = (uint8_t *)((((uint16_t) &rww_array) & 0x7FFF) + MAPPED_PROGMEM_START);
            
            eraseWriteState = ERASE_RWW;
            break;
        case ERASE_RWW:
            RWW_DATA_last_addr = ((((uint16_t) &rww_array) & 0x7FFF) + MAPPED_PROGMEM_START + RWW_DATA_SIZE);
            
            while ((uint16_t)rwwFlashPointer < RWW_DATA_last_addr)
            {
                SCOPE_PORT.OUTSET = SCOPE_FLPER_bm;
                //FLASH_PageErase((flash_address_t)rwwFlashPointer);
                
                _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_NOCMD_gc);
                
                // Perform a dummy write to this address to update the address register in NVMCTL
                *rwwFlashPointer = 0;
                
                _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_FLPER_gc);
                
                while (NVMCTRL.STATUS & NVMCTRL_FLBUSY_bm)
                {
                    ; // wait for flash erase to complete
                }
                SCOPE_PORT.OUTCLR = SCOPE_FLPER_bm;
                rwwFlashPointer = rwwFlashPointer + PROGMEM_PAGE_SIZE;
            }
            
            // Set rwwFlashPointer address to RWW data space
             rwwFlashPointer = (uint8_t *)((((uint16_t) &rww_array) & 0x7FFF) + MAPPED_PROGMEM_START);
            
            // set state machine to Write to the RWW data space
            eraseWriteState = WRITE_RWW;
            
            break;
        case WRITE_RWW:
             if ((uint16_t)rwwFlashPointer < RWW_DATA_last_addr)
             {
                 // Check if we have a full page to fill
                 if ((writeIndex - readIndex) >= PROGMEM_PAGE_SIZE)
                 {
                     // Fill "page" buffer
                     SCOPE_PORT.OUTSET = SCOPE_BUFFER_bm;
                     for (uint8_t i = 0; i < PROGMEM_PAGE_SIZE; i++)
                     {
                         *rwwFlashPointer++ = buffer[readIndex++];
                     }
                     SCOPE_PORT.OUTCLR = SCOPE_BUFFER_bm;

                     SCOPE_PORT.OUTSET = SCOPE_FLPER_bm;
                     // Write the flash page
                     _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_FLPW_gc);
                     while (NVMCTRL.STATUS & NVMCTRL_FLBUSY_bm)
                     {
                        ; // wait flash write page operation to complete
                     }                        
                     SCOPE_PORT.OUTCLR = SCOPE_FLPER_bm;
                 }
             }
             else
             {
                 eraseWriteState = IDLE;
                 TCB0.CTRLA &= ~TCB_ENABLE_bm; /* Stop Timer */

                 // reset buffer
                 writeIndex = readIndex;
             }
            
            break;
        case IDLE:
        {
            if (PRESS == buttonState)
            {
                PORTB_OUTCLR = 0x8;
            }
            else if (DEPRESS == buttonState)
            {
                PORTB_OUTSET = 0x8;
            }
            else
            {
                ; // Do nothing
            }

            if (prev_buttonState != buttonState)
            {
                if (PRESS == buttonState)
                {
                    // start a new ERASE/Write cycle
                    eraseWriteState = INIT_RWW;
                }

                prev_buttonState = buttonState;
            }
            break;
        }            
        default:
            break;
        }
    }
}

void SystemInitialize(void)
{
    // Enable prescaler
	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, (CLKCTRL_PDIV_DIV6_gc | CLKCTRL_PEN_bm));

    // Set all the status pins as outputs
    SCOPE_PORT.DIRSET = SCOPE_gm;

    PORTB.DIRSET = 0x8;

    // PB2 input
    PORTB.DIRSET &= (~PIN2_bm);
    PORTB.PIN2CTRL = 0x8;

	// Initialize TCB with periodic interrupt
	Tcb0Init();
    Tcb1Init();

	// Move the interrupt vector to the boot section (NRWW section)
	_PROTECTED_WRITE(CPUINT.CTRLA, CPUINT_IVSEL_bm);
	
	// Enable global interrupts
	sei();

	// Set the mapped program space to the 1st section (0k - 32k) 
	_PROTECTED_WRITE(NVMCTRL.CTRLB, NVMCTRL_FLMAP_SECTION0_gc);

	// Adding the mapped progmem offset to correct for the unified data space
	flashWritePointer = (uint8_t *) (((uint16_t) flashWritePointer & 0x7FFF) + MAPPED_PROGMEM_START);
	flashReadPointer = (uint8_t *) ((uint16_t) flashReadPointer | MAPPED_PROGMEM_START);
}

// This function reads from the mapped flash while writing
void ReadWhileWriting(void)
{
    // Read from the mapped flash (to show the CPU halts)
	SCOPE_PORT.OUTSET = SCOPE_READ_bm;
	GPR.GPR0 = flashReadPointer[0];
	SCOPE_PORT.OUTCLR = SCOPE_READ_bm;
}

void FillBuffer(void)
{
	static volatile uint32_t data = 0;

	// Fill the buffer with some data
	SCOPE_PORT.OUTTGL = SCOPE_ISR_bm;
	buffer[writeIndex++] = (data >> 2);
	data++;
    	
	// Check if we have a buffer overflow
	if (writeIndex == readIndex)
	{
	    // Overflow
		SCOPE_PORT.OUTTGL = SCOPE_OVERFLOW_bm;
	}
}

void DebounceSW0(void)
{
    static volatile uint16_t debounceCounter = MAXIMUM / 2; // Will range from 0 to the specified MAXIMUM

    if (VPORTB.IN & PIN2_bm)
    {
        if (MAXIMUM > debounceCounter)
        {
            debounceCounter++;
        }
    }
    else
    {
        if (0 < debounceCounter)
        {
            debounceCounter--;
        }
    }

    if (0 == debounceCounter)
    {
        buttonState = PRESS;
    }
    else if (MAXIMUM <= debounceCounter)
    {
        buttonState = DEPRESS;
        debounceCounter = MAXIMUM;
    }
    else
    {
        buttonState = UNKNOWN; // Do nothing
    }
}
