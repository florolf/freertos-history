/*
	FreeRTOS V2.4.2 - Copyright (C) 2003, 2004 Richard Barry.

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


/*-----------------------------------------------------------
 * Components that can be compiled to either ARM or THUMB mode are
 * contained in port.c  The ISR routines, which can only be compiled
 * to ARM mode, are contained in this file.
 *----------------------------------------------------------*/


/* Scheduler includes. */
#include "projdefs.h"
#include "portable.h"
#include "task.h"

/* Constants required to handle interrupts. */
#define portTIMER_MATCH_ISR_BIT		( ( unsigned portCHAR ) 0x01 )
#define portCLEAR_VIC_INTERRUPT		( ( unsigned portLONG ) 0 )
/*-----------------------------------------------------------*/

/* ISR to handle manual context switches (from a call to taskYIELD()). */
void vPortYieldProcessor( void ) __attribute__((interrupt("SWI"), naked));

/* 
 * The scheduler can only be started from ARM mode, hence the inclusion of this
 * function here.
 */
void vPortISRStartFirstTask( void );
/*-----------------------------------------------------------*/

void vPortISRStartFirstTask( void )
{
	/* Simply start the scheduler.  This is included here as it can only be
	called from ARM mode. */
	portRESTORE_CONTEXT();
}
/*-----------------------------------------------------------*/

void vPortYieldProcessor( void )
{
	/* Within an IRQ ISR the link register has an offset from the true return 
	address, but an SWI ISR does not.  Add the offset manually so the same 
	ISR return code can be used in both cases. */
	asm volatile ( "ADD		LR, LR, #4" );

	/* Perform the context switch. */
	portSAVE_CONTEXT();
	vTaskSwitchContext();
	portRESTORE_CONTEXT();	
}
/*-----------------------------------------------------------*/

/* 
 * The ISR used for the scheduler tick depends on whether the cooperative or
 * the preemptive scheduler is being used.
 */

#if portUSE_PREEMPTION == 0

	/* The cooperative scheduler requires a normal IRQ service routine to 
	simply increment the system tick. */
	static void vNonPreemptiveTick( void ) __attribute__ ((interrupt ("IRQ")));
	static void vNonPreemptiveTick( void )
	{		
		vTaskIncrementTick();
		T0_IR = portTIMER_MATCH_ISR_BIT;
		VICVectAddr = portCLEAR_VIC_INTERRUPT;
	}

#else

	/* The preemptive scheduler is defined as "naked" as the full context is
	saved on entry as part of the context switch. */
	void vPreemptiveTick( void ) __attribute__((naked));
	void vPreemptiveTick( void )
	{
		portSAVE_CONTEXT();	

		vTaskIncrementTick();
		vTaskSwitchContext();
		T0_IR = portTIMER_MATCH_ISR_BIT;
		VICVectAddr = portCLEAR_VIC_INTERRUPT;
		
		portRESTORE_CONTEXT();
	}

#endif
/*-----------------------------------------------------------*/

/*
 * The interrupt management utilities can only be called from ARM mode.  When
 * THUMB_INTERWORK is defined the utilities are defined as functions here to
 * ensure a switch to ARM mode.  When THUMB_INTERWORK is not defined then
 * the utilities are defined as macros in portmacro.h - as per other ports.
 */
#ifdef THUMB_INTERWORK

	void vPortEnterCriticalFromThumb( void ) __attribute__ ((naked));
	void vPortExitCriticalFromThumb( void ) __attribute__ ((naked));
	void vPortDisableInterruptsFromThumb( void ) __attribute__ ((naked));
	void vPortEnableInterruptsFromThumb( void ) __attribute__ ((naked));

	void vPortEnterCriticalFromThumb( void )
	{
		asm volatile ( "STMDB	SP!, {R0, R1}" );	/* Push R0 to leave space for CPSR, then R1	*/
		asm volatile ( "MRS		R1, CPSR" );		/* Get CPSR.								*/
		asm volatile ( "STR		R1, [SP, #4]" );	/* Store CPSR in created space.				*/
		asm volatile ( "ORR		R1, R1, #0xC0" );	/* Disable IRQ and FIQ.						*/
		asm volatile ( "MSR		CPSR, R1" );		/* Write back modified value.				*/
		asm volatile ( "LDMIA	SP!, {R1}" );		/* Pop R0. */
		asm volatile ( "BX		R14" );				/* Return back to thumb.					*/
	}

	void vPortExitCriticalFromThumb( void )
	{
		asm volatile ( "STMDB	SP!, {R0, R1}" );	/* Push R0 and R1.							*/
		asm volatile ( "LDR		R0, [SP, #+8 ]" );	/* Retrieve CPSR from stack.				*/
		asm volatile ( "AND		R0, R0, #0xC0" );	/* Just look and interrupt bits.			*/
		asm volatile ( "MRS		R1, CPSR" );		/* Get current CPSR.						*/
		asm volatile ( "BIC		R1, R1, #0xC0" );	/* Mast off the IRQ and FIQ bits.			*/
		asm volatile ( "ORR		R0, R0, R1" );		/* Combine with stored IRQ bits.			*/
		asm volatile ( "MSR		CPSR, R0" );		/* Write back new CPSR value.				*/
		asm volatile ( "LDMIA	SP!, {r0, R1}" );	/* Pop R0 and R1.							*/
		asm volatile ( "ADD		SP, SP, #4" );		/* Correct stack.							*/
		asm volatile ( "BX		R14" );				/* Return back to thumb.					*/
	}

	void vPortDisableInterruptsFromThumb( void )
	{
		asm volatile ( "STMDB	SP!, {R0}" );		/* Push R0.									*/
		asm volatile ( "MRS		R0, CPSR" );		/* Get CPSR.								*/
		asm volatile ( "ORR		R0, R0, #0xC0" );	/* Disable IRQ, FIQ.						*/
		asm volatile ( "MSR		CPSR, R0" );		/* Write back modified value.				*/
		asm volatile ( "LDMIA	SP!, {R0}" );		/* Pop R0.									*/
		asm volatile ( "BX		R14" );				/* Return back to thumb.					*/
	}
			
	void vPortEnableInterruptsFromThumb( void )
	{
		asm volatile ( "STMDB	SP!, {R0}" );		/* Push R0.									*/	
		asm volatile ( "MRS		R0, CPSR" );		/* Get CPSR.								*/	
		asm volatile ( "BIC		R0, R0, #0xC0" );	/* Enable IRQ, FIQ.							*/	
		asm volatile ( "MSR		CPSR, R0" );		/* Write back modified value.				*/	
		asm volatile ( "LDMIA	SP!, {R0}" );		/* Pop R0.									*/
		asm volatile ( "BX		R14" );				/* Return back to thumb.					*/
	}

#endif /* THUMB_INTERWORK */
