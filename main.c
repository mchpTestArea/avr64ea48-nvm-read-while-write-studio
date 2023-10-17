/**
 * Non-Volatile Memory Controller - Read While Write Bare Metal Implementation
 *
 * @file main.c
 * Author: Microchip Technology Inc.
 *
 * @ingroup main 
 *
 * @brief This is an application implementing the Read While Write functionality of the 
 * AVR EA using bare metal code in Microchip Studio
 *
 * @version MAIN file Version 1.0.0
 */
/*
*  © 2023 Microchip Technology Inc. and its subsidiaries.
*
*  Subject to your compliance with these terms, you may use this Microchip software and any
*  derivatives exclusively with Microchip products. You are responsible for complying with
*  third party license terms applicable to your use of third party software (including open
*  source software) that may accompany this Microchip software.
* 
*  SOFTWARE IS “AS IS.” NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO
*  THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
*  OR FITNESS FOR A PARTICULAR PURPOSE.
*  
*  IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL
*  OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
*  SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE
*  DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP’S TOTAL
*  LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT EXCEED AMOUNT OF FEES, IF ANY,
*  YOU PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*
*  MICROCHIP OFFERS NO SUPPORT FOR THE SOFTWARE. YOU MAY CONTACT MICROCHIP AT
*  https://www.microchip.com/en-us/support-and-training/design-help/client-support-services
*  TO INQUIRE ABOUT SUPPORT SERVICES AND APPLICABLE FEES, IF AVAILABLE.
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include "include/main.h"
#include "include/tcb0.h"
#include "include/tcb1.h"

/* 
 * The following three defines uses addresses defined in the linker section.
 * .rww_data, .nrww_data and .nrww_program
 * See in the Readme under Project Properties for more information.
 */
// The .rww_data section is configured to be at address 0x2000 (word address 0x1000)
#define RWW_DATA_SECTION __attribute__((used, section(".rww_data")))

// The .nrww_data section is configured to be at address 0x1F00 (word address 0x0F80)
#define NRWW_DATA_SECTION __attribute__((used, section(".nrww_data")))

// The .nrww_program section is configured to be at address 0x0400 (word address 0x0200)
#define NRWW_PROG_SECTION __attribute__((section(".nrww_program")))

#define BUFFER_SIZE 256 // Because uint8 index will wrap around perfectly
#define DATA_SIZE 256

#define DEBOUNCE_TIME 20 // Button debounce time in ms

// RWW variables
const RWW_DATA_SECTION uint8_t rwwArray[DATA_SIZE] = {0};
uint8_t *flashWritePointer = (uint8_t *) &rwwArray;

// NRWW variables
const NRWW_DATA_SECTION uint8_t nrwwArray[DATA_SIZE] = {0};

// Buffer variables
uint8_t buffer[BUFFER_SIZE] = {0};
volatile uint8_t writeIndex = 0;
volatile uint8_t readIndex = 0;
static volatile uint32_t data = 0;

volatile uint16_t rwwLastDataAddress = 0;
volatile uint16_t nrwwLastDataAddress = 0;

typedef enum BUTTON_enum
{
    PRESSED,
    RELEASED
} BUTTON_t;

static uint8_t buttonDebounceCounter = 0; // Counter variable to track debounce for on-board button (SW0)
BUTTON_t buttonState = RELEASED; // Cleaned-up version of the SW0 button input signal

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

// Function prototypes
void SystemInitialize(void);
void ProgramingFlash(void);
void ReadWhileWriting(void);
NRWW_PROG_SECTION void FillBuffer(void);


int main(void)
{
    // Pointer to rww flash program memory mapped in data space
    uint8_t *rwwFlashPointer;
    // Pointer to nrww flash program memory mapped in data space
    uint8_t *nrwwFlashPointer;
    
    static BUTTON_t previousButtonState = RELEASED;
    
    // Sets the system state to IDLE
    DATA_FLASH_t eraseWriteState = IDLE;
    
    SystemInitialize();

    while (1) 
    {
        switch (eraseWriteState)
        {
            case IDLE:
            {
                if (PRESSED == buttonState)
                {
                    PORTB.OUTCLR = PIN3_bm;
                }
                else if (RELEASED == buttonState)
                {
                    PORTB.OUTSET = PIN3_bm;
                }
                
                if (previousButtonState != buttonState)
                {
                    if (PRESSED == buttonState)
                    {
                        // Sets the system state to INIT_NRWW to start a new Erase/Write cycle
                        eraseWriteState = INIT_NRWW;
                    }
                    
                    previousButtonState = buttonState;
                }
                break;
            }
            case INIT_NRWW:
                // Sets rwwFlashPointer address to NRWW data space
                nrwwFlashPointer = (uint8_t *)((((uint16_t) &nrwwArray) & 0x7FFF) + MAPPED_PROGMEM_START);
                
                // Starts TCB0, will fill the data buffer
                TCB0.CTRLA |= TCB_ENABLE_bm;
                
                // Sets the system state to ERASE_NRWW to start the Erase cycle
                eraseWriteState = ERASE_NRWW;
                break;
            case ERASE_NRWW:
                nrwwLastDataAddress = ((((uint16_t) &nrwwArray) & 0x7FFF) + MAPPED_PROGMEM_START + DATA_SIZE);
                
                while ((uint16_t)nrwwFlashPointer < nrwwLastDataAddress)
                {
                    SCOPE_PORT.OUTSET = SCOPE_FLPER_bm;
                    
                    _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_NOCMD_gc);
                    
                    // Performs a dummy write to this address to update the address register in NVMCTRL
                    *nrwwFlashPointer = 0;
                    
                    _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_FLPER_gc);
                    
                    while (NVMCTRL.STATUS & NVMCTRL_FLBUSY_bm)
                    {
                        ; // Waits for flash erase to complete
                    }
                    SCOPE_PORT.OUTCLR = SCOPE_FLPER_bm;
                    
                    nrwwFlashPointer = nrwwFlashPointer + PROGMEM_PAGE_SIZE;
                }
                
                // Sets nrwwFlashPointer address to NRWW data space
                nrwwFlashPointer = (uint8_t *)((((uint16_t) &nrwwArray) & 0x7FFF) + MAPPED_PROGMEM_START);
                
                // Sets the system state to WRITE_NRWW to start the write to the RWW data space
                eraseWriteState = WRITE_NRWW;
                break;
            case WRITE_NRWW:
                if ((uint16_t)nrwwFlashPointer < nrwwLastDataAddress)
                {
                    // Check if we have a full page to fill
                    if ((writeIndex - readIndex) >= PROGMEM_PAGE_SIZE)
                    {
                        _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_NOCMD_gc);
                        
                        // Fills "page" buffer
                        SCOPE_PORT.OUTSET = SCOPE_BUFFER_bm;
                        for (uint8_t i = 0; i < PROGMEM_PAGE_SIZE; i++)
                        {
                            *nrwwFlashPointer++ = buffer[readIndex++];
                        }
                        SCOPE_PORT.OUTCLR = SCOPE_BUFFER_bm;
                        
                        SCOPE_PORT.OUTSET = SCOPE_FLPW_bm;
                        // Writes the flash page
                        _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_FLPW_gc);
                        while (NVMCTRL.STATUS & NVMCTRL_FLBUSY_bm)
                        {
                            ; // Waits for flash page write operation to complete
                        }
                        SCOPE_PORT.OUTCLR = SCOPE_FLPW_bm;
                    }
                }
                else
                {
                    // Sets the system state to INIT_RWW to start a new Erase/Write cycle
                    eraseWriteState = INIT_RWW;
                    
                    TCB0.CTRLA &= ~TCB_ENABLE_bm; // Stop Timer
                    
                    // Resets buffer
                    writeIndex = readIndex;
                }
                break;
            case INIT_RWW:
                // Sets rwwFlashPointer address to RWW data space
                rwwFlashPointer = (uint8_t *)((((uint16_t) &rwwArray) & 0x7FFF) + MAPPED_PROGMEM_START);
                
                // Starts TCB0,  will fill the data buffer
                TCB0.CTRLA |= TCB_ENABLE_bm; 
                
                // Sets the system state to ERASE_RWW to start Erase cycle
                eraseWriteState = ERASE_RWW;
                break;
            case ERASE_RWW:
                rwwLastDataAddress = ((((uint16_t) &rwwArray) & 0x7FFF) + MAPPED_PROGMEM_START + DATA_SIZE);
                
                while ((uint16_t)rwwFlashPointer < rwwLastDataAddress)
                {
                    SCOPE_PORT.OUTSET = SCOPE_FLPER_bm;
                    
                    _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_NOCMD_gc);
                    
                    // Performs a dummy write to this address to update the address register in NVMCTL
                    *rwwFlashPointer = 0;
                    
                    _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_FLPER_gc);
                    
                    while (NVMCTRL.STATUS & NVMCTRL_FLBUSY_bm)
                    {
                        ; // Waits for flash erase to complete
                    }
                    SCOPE_PORT.OUTCLR = SCOPE_FLPER_bm;
                    rwwFlashPointer = rwwFlashPointer + PROGMEM_PAGE_SIZE;
                }
                
                // Sets rwwFlashPointer address to RWW data space
                rwwFlashPointer = (uint8_t *)((((uint16_t) &rwwArray) & 0x7FFF) + MAPPED_PROGMEM_START);
                
                // Sets the system state to WRITE_RWW to start Flash Write to the RWW data space
                eraseWriteState = WRITE_RWW;
                
                break;
            case WRITE_RWW:
                if ((uint16_t)rwwFlashPointer < rwwLastDataAddress)
                {
                    // Checks if we have a full page to fill
                    if ((writeIndex - readIndex) >= PROGMEM_PAGE_SIZE)
                    {
                        _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_NOCMD_gc);
                        
                        // Fills "page" buffer
                        SCOPE_PORT.OUTSET = SCOPE_BUFFER_bm;
                        for (uint8_t i = 0; i < PROGMEM_PAGE_SIZE; i++)
                        {
                            *rwwFlashPointer++ = buffer[readIndex++];
                        }
                        SCOPE_PORT.OUTCLR = SCOPE_BUFFER_bm;
                        
                        // Writes the flash page
                        _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_FLPW_gc);
                        SCOPE_PORT.OUTSET = SCOPE_FLPW_bm;
                        while (NVMCTRL.STATUS & NVMCTRL_FLBUSY_bm)
                        {
                            ; // Waits for flash page write operation to complete
                        }                        
                        SCOPE_PORT.OUTCLR = SCOPE_FLPW_bm;
                    }
                }
                else
                {
                    // Sets the system state to IDLE
                    eraseWriteState = IDLE;
                    TCB0.CTRLA &= ~TCB_ENABLE_bm; // Stop Timer
                
                    // Resets buffer
                    writeIndex = readIndex;
                }
                
                break;
            default:
                break;
        }
    }
}


void SystemInitialize(void)
{
    // Enables prescaler
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, (CLKCTRL_PDIV_DIV6_gc | CLKCTRL_PEN_bm));
    
    // Sets all the status pins as outputs
    SCOPE_PORT.DIRSET = SCOPE_gm;
    
    // Sets PB3 as output
    PORTB.DIRSET = PIN3_bm;
    
    // Sets PB2 as input
    PORTB.DIRSET &= (~PIN2_bm);
    PORTB.PIN2CTRL = PORT_PULLUPEN_bm;
    
    // Initializes TCB with periodic interrupt
    TCB0_Init();
    TCB1_Init();
    
    // Moves the interrupt vector to the boot section (NRWW section)
    _PROTECTED_WRITE(CPUINT.CTRLA, CPUINT_IVSEL_bm);
    
    // Enables global interrupts
    sei();
    
    // Sets the mapped program space to the 1st section (0k - 32k) 
    _PROTECTED_WRITE(NVMCTRL.CTRLB, NVMCTRL_FLMAP_SECTION0_gc);
    
    // Adds the mapped progmem offset to correct for the unified data space
    flashWritePointer = (uint8_t *) (((uint16_t) flashWritePointer & 0x7FFF) + MAPPED_PROGMEM_START);
    
    // Initializes data value with the last value being read from Flash.
    data = (*(uint16_t *)(MAPPED_PROGMEM_START + (((uint16_t)(& rwwArray[254])) & 0x7FFF)));
    data = data << 2;
}


NRWW_PROG_SECTION void FillBuffer(void)
{
    // Fills the buffer with some data
    SCOPE_PORT.OUTTGL = SCOPE_ISR_bm;
    buffer[writeIndex++] = (data >> 2);
    data++;
    
    // Checks if we have a buffer overflow
    if (writeIndex == readIndex)
    {
        // Overflow
        SCOPE_PORT.OUTTGL = SCOPE_OVERFLOW_bm;
    }
}


void DebounceSW0(void)
{
    // Gets current button state
    bool buttonIsPressed = !(VPORTB.IN & PIN2_bm);
    
    if(0 == buttonDebounceCounter) // Debounce is done
    {
        if(buttonIsPressed) // New SW0 pressed
        {
            // Sets button state to pressed
            buttonState = PRESSED;
            // Sets counter to debounce time
            buttonDebounceCounter = DEBOUNCE_TIME;
        }
        else{
            // Sets button state to released
                buttonState = RELEASED;
        }
    }
    else // Debounce counter still counting down
    {
        if (!buttonIsPressed) // Button is released
        {
            // Decrements debounce counter and checks if it has reached 0
            --buttonDebounceCounter;
            if (0 == buttonDebounceCounter)
            {
                // Sets button state to released
                buttonState = RELEASED;
            }
        }
        else // Button held or repressed
        {
            // Sets counter back to debounce time
            buttonDebounceCounter = DEBOUNCE_TIME;
        }
    }
}