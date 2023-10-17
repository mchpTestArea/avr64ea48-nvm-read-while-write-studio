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
#include <avr/fuse.h>

//Configures Fuse bits
FUSES =
{
    .SYSCFG0 = FUSE_SYSCFG0_DEFAULT,
    .SYSCFG1 = FUSE_SYSCFG1_DEFAULT,
    
    .WDTCFG = FUSE_WDTCFG_DEFAULT,
    .BODCFG = FUSE_BODCFG_DEFAULT,
    .OSCCFG = FUSE_OSCCFG_DEFAULT,
    
    .BOOTSIZE = 31,    // BOOT section from FLASHSTART to 0x1F00 ((BOOTSIZE*256)-1)
    .CODESIZE = 1,     // APPCODE Section - Not available
                       // APP DATA from 0x1F00 to 0xFFFF
};