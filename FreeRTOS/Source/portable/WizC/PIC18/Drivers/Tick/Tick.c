/*
	FreeRTOS V3.0.0 - Copyright (C) 2003 - 2005 Richard Barry.

	This file is part of the FreeRTOS distribution.

	FreeRTOS is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	FreeRTOS is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with FreeRTOS; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

	A special exception to the GPL can be applied should you wish to distribute
	a combined work that includes FreeRTOS, without being obliged to provide
	the source code for any proprietary components.  See the licensing section 
	of http://www.FreeRTOS.org for full details of how and when the exception
	can be applied.

	***************************************************************************
	See http://www.FreeRTOS.org for documentation, latest information, license 
	and contact details.  Please ensure to read the configuration and relevant 
	port sections of the online documentation.
	***************************************************************************
*/

/* 
Changes from V3.0.0

*/

#include <FreeRTOS.h>
#include <task.h>
/*
 * ISR for the tick.
 * This increments the tick count and, if using the preemptive scheduler, 
 * performs a context switch.  This must be identical to the manual 
 * context switch in how it stores the context of a task. 
 */

/* IO port constants. */
#define portBIT_SET		(1)
#define portBIT_CLEAR	(0)

/* Hardware setup for tick. */
#define	portTIMER_COMPARE_VALUE			((APROCFREQ/4)/configTICK_RATE_HZ)

/*-----------------------------------------------------------*/

void portTICKisr( void )
{
	/*
	 * Reset the interrupt flag
	 */
	bCCP1IF = 0;
	
	/*
	 * Maintain the tick count.
	 */
	vTaskIncrementTick();

	#if configUSE_PREEMPTION == 1
	{
		/*
		 * Switch to the highest priority task that is ready to run.
		 */
		vTaskSwitchContext();
	}
	#endif
}

/*-----------------------------------------------------------*/

/*
 * Setup a timer for a regular tick.
 */
void portSetupTick( void )
{
	/*
	 * Interrupts are disabled when this function is called.
	 */

	/*
	 * Setup CCP1: provide the tick interrupt using a compare match
	 * on timer1.
	 */

	/*
	 * Clear the time count then setup timer.
	 */
	TMR1H = ( unsigned portCHAR ) 0x00;
	TMR1L = ( unsigned portCHAR ) 0x00;

	/*
	 * Set the compare match value.
	 */
	CCPR1L = ( unsigned portCHAR )   ( portTIMER_COMPARE_VALUE & 0xff );
	CCPR1H = ( unsigned portCHAR ) ( ( portTIMER_COMPARE_VALUE >> 8 ) & 0xff );

	bCCP1M0		= portBIT_SET;		// Compare match mode.
	bCCP1M1 	= portBIT_SET;		// Compare match mode.
	bCCP1M2 	= portBIT_CLEAR;	// Compare match mode.
	bCCP1M3 	= portBIT_SET;		// Compare match mode.
	bCCP1IE 	= portBIT_SET;		// Interrupt enable.

	/*
	 * We are only going to use the global interrupt bit, so disable
	 * interruptpriorities and enable peripheral interrupts.
	 */
	bIPEN		= portBIT_CLEAR;
	bPEIE		= portBIT_SET;

	/*
	 * Set up timer1 that will produce the tick.
	 */
	bRD16		= portBIT_SET;		// 16-bit
	bT1CKPS0	= portBIT_CLEAR;	// 1:1 prescaler
	bT1CKPS1	= portBIT_CLEAR;	// 1:1 prescaler
	bT1OSCEN	= portBIT_SET;		// Oscillator enable
	bT1SYNC		= portBIT_SET;		// No external clock sync
	bTMR1CS		= portBIT_CLEAR;	// Internal clock
	
	bTMR1ON		= portBIT_SET;		// Start timer1
}
