/*
	FreeRTOS V2.6.1 - Copyright (C) 2003 - 2005 Richard Barry.

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


#ifndef PORTMACRO_H
#define PORTMACRO_H

/* IO definitions for the chosen device. */
#include <2329S.h>

/*-----------------------------------------------------------
 * Port specific definitions for the H8S port.
 *----------------------------------------------------------*/

/* These are the only definitions that can be modified!. */

#define portUSE_PREEMPTION		1
#define portCPU_CLOCK_HZ		( ( unsigned portLONG ) 22118400 )
#define portTICK_RATE_HZ		( ( portTickType ) 1000 )
#define portMAX_PRIORITIES		( ( unsigned portCHAR ) 4 )
#define portMINIMAL_STACK_SIZE	( ( unsigned portSHORT ) 200 )
#define portTOTAL_HEAP_SIZE		( ( unsigned portSHORT ) ( 15 * 1024 ) )

/* The maximum number of characters a task name can take, 
including the null terminator. */
#define portMAX_TASK_NAME_LEN		 ( 8 )

/* Set the following definitions to 1 to include the component, or zero
to exclude the component. */

/* Include/exclude the stated API function. */
#define INCLUDE_vTaskPrioritySet		1
#define INCLUDE_ucTaskPriorityGet		1
#define INCLUDE_vTaskDelete				1
#define INCLUDE_vTaskCleanUpResources	0
#define INCLUDE_vTaskSuspend			1
#define INCLUDE_vTaskDelayUntil			1
#define INCLUDE_vTaskDelay				1

/* Use/don't use the trace visualisation. */
#define USE_TRACE_FACILITY				0

/* 
 * The tick count (and times defined in tick count units) can be either a 16bit
 * or a 32 bit value.  See documentation on http://www.FreeRTOS.org to decide
 * which to use.
 */
#define USE_16_BIT_TICKS	1


/*-----------------------------------------------------------
 * Do not modify anything below here. 
 *----------------------------------------------------------*/

/* General port definitions. */

#define portCHAR		char
#define portFLOAT		float
#define portDOUBLE		double
#define portLONG		long
#define portSHORT		short
#define portSTACK_TYPE	unsigned portCHAR

#define portBYTE_ALIGNMENT			2

#if( USE_16_BIT_TICKS == 1 )
	typedef unsigned portSHORT portTickType;
	#define portMAX_DELAY ( portTickType ) 0xffff
#else
	typedef unsigned portLONG portTickType;
	#define portMAX_DELAY ( portTickType ) 0xffffffff
#endif

#define portSTACK_GROWTH			( -1 )
#define portTICK_RATE_MS			( ( portTickType ) 1000 / portTICK_RATE_HZ )		

/* The trap instruction used to force a context switch. */
#define portYIELD()					asm volatile( "TRAPA #0" );

/*-----------------------------------------------------------*/

/* Interrupt and critical section macros. */

#define portENABLE_INTERRUPTS()		asm volatile( "ANDC	#0x7F, CCR" );
#define portDISABLE_INTERRUPTS()	asm volatile( "ORC  #0x80, CCR" );

/* Push the CCR then disable interrupts. */
#define portENTER_CRITICAL()  		asm volatile( "STC	CCR, @-ER7" ); \
                               		portDISABLE_INTERRUPTS();

/* Pop the CCR to set the interrupt masking back to its previous state. */
#define  portEXIT_CRITICAL()    	asm volatile( "LDC  @ER7+, CCR" );

/*-----------------------------------------------------------*/

/* Context switch macros.  These macros are very simple as the context 
is saved simply by selecting the saveall attribute of the context switch 
interrupt service routines.  These macros save and restore the stack
pointer to the TCB. */

#define portSAVE_STACK_POINTER()								\
extern void* pxCurrentTCB;										\
																\
	asm volatile(												\
					"MOV.L	@_pxCurrentTCB, ER5			\n\t" 	\
					"MOV.L	ER7, @ER5					\n\t"	\
				);


#define	portRESTORE_STACK_POINTER()								\
extern void* pxCurrentTCB;										\
																\
	asm volatile(												\
					"MOV.L	@_pxCurrentTCB, ER5			\n\t"	\
					"MOV.L	@ER5, ER7					\n\t"	\
				);

/*-----------------------------------------------------------*/

/* Macros to allow a context switch from within an application ISR. */

#define portENTER_SWITCHING_ISR() portSAVE_STACK_POINTER(); {

#define portEXIT_SWITCHING_ISR( x )							\
	if( x )													\
	{														\
		extern void vTaskSwitchContext( void );				\
		vTaskSwitchContext();								\
	}														\
	} portRESTORE_STACK_POINTER();


#endif /* PORTMACRO_H */

