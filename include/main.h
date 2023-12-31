/* 
 *  (c) 2021 Microchip Technology Inc. and its subsidiaries.
 *
 *  Subject to your compliance with these terms, you may use Microchip software
 *  and any derivatives exclusively with Microchip products. You�re responsible
 *  for complying with 3rd party license terms applicable to your use of 3rd
 *  party software (including open source software) that may accompany
 *  Microchip software.
 * 
 *  SOFTWARE IS �AS IS.� NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
 *  APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF
 *  NON-INFRINGEMENT, MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
 *  
 *  IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
 *  INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
 *  WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
 *  BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
 *  FULLEST EXTENT ALLOWED BY LAW, MICROCHIP�S TOTAL LIABILITY ON ALL CLAIMS
 *  RELATED TO THE SOFTWARE WILL NOT EXCEED AMOUNT OF FEES, IF ANY, YOU PAID
 *  DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 */

// This file includes all the definitions used in main.c

#ifndef MAIN_H_
#define MAIN_H_

// Status pins used for measurements
#define SCOPE_PORT PORTA
#define SCOPE_ISR_bm PIN2_bm
#define SCOPE_BUFFER_bm PIN3_bm
#define SCOPE_FLPER_bm PIN4_bm
#define SCOPE_FLPW_bm PIN5_bm
#define SCOPE_OVERFLOW_bm PIN6_bm

#define SCOPE_gm (SCOPE_BUFFER_bm | SCOPE_ISR_bm | SCOPE_FLPER_bm | SCOPE_FLPW_bm | SCOPE_OVERFLOW_bm)

// Function prototypes
void FillBuffer(void);
void DebounceSW0(void);

#endif /* MAIN_H_ */