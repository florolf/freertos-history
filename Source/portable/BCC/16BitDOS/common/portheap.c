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
 * >Very< basic block allocation scheme.  This uses a block size that will
 * prevent malloc being called once the scheduler has been started in
 * the demo application.  Blocks greater than heapBLOCK_SIZE still use 
 * malloc.
 *
 * Only the suicide tasks use memory allocation after the the scheduler has
 * been started.
 *
 * NOTES:
 *  - This relies on static memory being initialised to 0!
 *  - This also relies on the compiler not leaving any space between ucFull
 *    and ucBlock.
 *----------------------------------------------------------*/

/* Compiler include files. */
#include <stdlib.h>

/* Scheduler include files. */
#include "projdefs.h"
#include "portable.h"

#define heapBLOCK_SIZE		128

/* The number of heapBLOCK_SIZE blocks that will be available. */
#define heapNUM_LARGE_BLOCKS		( 250 )

/* Each block contains the data area and a variable that marks the block as
empty of full. */
typedef struct
{
	unsigned portCHAR ucFull;
	unsigned portCHAR ucBlock[ heapBLOCK_SIZE ];
} xSingleBlock;

/* Allocate the blocks. */
static xSingleBlock xBlocks[ heapNUM_LARGE_BLOCKS ];

/*-----------------------------------------------------------*/
void *pvPortMalloc( unsigned portSHORT usSize )
{
void *pv = NULL;
unsigned portCHAR ucBlock;

	portENTER_CRITICAL();
	{
		/* Try to find a free block in the list of static blocks. */
		if( usSize <= heapBLOCK_SIZE ) 
		{		
			for( ucBlock = 0; ucBlock < heapNUM_LARGE_BLOCKS; ucBlock++ )
			{	
				if( xBlocks[ ucBlock ].ucFull == pdFALSE )
				{
					xBlocks[ ucBlock ].ucFull = pdTRUE;
					pv = ( void * ) ( xBlocks[ ucBlock ].ucBlock );
					break;
				}
			}
		}
	}
	portEXIT_CRITICAL();

	if( pv == NULL )
	{
		/* If we are out of blocks, or usSize is greater than heapBLOCK_SIZE,
		just allocate with malloc. */
		portENTER_CRITICAL();
			pv = malloc( usSize );
		portEXIT_CRITICAL();
	}

	return pv;
}
/*-----------------------------------------------------------*/

void vPortFree( void *pv )
{
portCHAR * pChar;

	if( ( pv < &( xBlocks[ 0 ] ) ) || ( pv > &( xBlocks[ heapNUM_LARGE_BLOCKS - 1 ] ) ) )
	{
		/* The memory is outside the static blocks, so must have been allocated
		using malloc. */
		portENTER_CRITICAL();
			free( pv );
		portEXIT_CRITICAL();
	}
	else
	{
		/* The block is within the static blocks.  Set the flag to say the
		block is free again. */
		pChar = ( portCHAR * ) pv;
		pChar -= sizeof( portCHAR );
		*pChar = pdFALSE;
	}
	pv = NULL;
}
/*-----------------------------------------------------------*/

