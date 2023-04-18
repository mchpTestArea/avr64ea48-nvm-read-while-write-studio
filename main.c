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

void SystemInitialize(void);
void ProgramingFlash(void);
void ReadWhileWriting(void);

int main(void)
{
    SystemInitialize();
	
	FillBuffer();

    RWW_DATA_last_addr = ((((uint16_t) &rww_array) & 0x7FFF) + MAPPED_PROGMEM_START + RWW_DATA_SIZE);

    while (1) 
    {
		if (!(NVMCTRL.STATUS & NVMCTRL_FLBUSY_bm))
		{
			ProgramingFlash();
			// We cannot keep programming the flash forever, so stop after some pages
			flashReadPointer_addr = (uint16_t) flashWritePointer ;
			
			if (flashReadPointer_addr >= RWW_DATA_last_addr )
			//if ((uint16_t) flashWritePointer > ((uint16_t) &rww_array + RWW_DATA_SIZE))
			{
				RWW_DATA_last_addr++;
				return 0;
			}
		}

		// This code if defined in, demonstrated the CPU halting
		#ifdef READ_MAPPED_DATA_WHILE_WRITING
		else
		{
		    ReadWhileWriting();
		}
		#endif // READ_MAPPED_DATA_WHILE_WRITING
	    	
    }
}

void SystemInitialize(void)
{
    // Enable prescaler
	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, (CLKCTRL_PDIV_DIV6_gc | CLKCTRL_PEN_bm));

	// Set all the status pins as outputs
	SCOPE_PORT.DIRSET = SCOPE_gm;

	// Initialize TCB with periodic interrupt
	Tcb0Init();

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

// This function fills a page buffer, and then writes the page buffer to Flash
void NRWW_PROG_SECTION ProgramingFlash(void)
{
	// Erase write done (or we do not have a full page to fill)
	SCOPE_PORT.OUTCLR = SCOPE_FLPERW_bm;

	// Check if we have a full page to fill
	if ((writeIndex - readIndex) >= PROGMEM_PAGE_SIZE)
	{
	    // Clear NVM Command
		_PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_NOCMD_gc);

		// Fill page buffer
		SCOPE_PORT.OUTSET = SCOPE_BUFFER_bm;
		for (uint8_t i = 0; i < PROGMEM_PAGE_SIZE; i++)
		{
		    *flashWritePointer++ = buffer[readIndex++];
		}
		SCOPE_PORT.OUTCLR = SCOPE_BUFFER_bm;

		// Send the page erase write NVM command
		SCOPE_PORT.OUTSET = SCOPE_FLPERW_bm;
		_PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_FLPERW_gc);
	}
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
		while (1)
		{
		}
	}
}
