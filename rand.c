#include "FreeRTOS.h"
#include "task.h"
#include "STM32F4xx.h"

/* Use by the pseudo random number generator. */
static UBaseType_t ulNextRand;
static RNG_HandleTypeDef hrng;

static void prvSRand( UBaseType_t ulSeed )
{
	/* Utility function to seed the pseudo random number generator. */
	ulNextRand = ulSeed;
}
/*-----------------------------------------------------------*/

UBaseType_t uxRand( void )
{
const uint32_t ulMultiplier = 0x015a4e35UL, ulIncrement = 1UL;
static BaseType_t xInitialised = pdFALSE;

	/* Don't initialise until the scheduler is running, as the timeout in the
	random number generator uses the tick count. */
	if( xInitialised == pdFALSE )
	{
		if( xTaskGetSchedulerState() !=  taskSCHEDULER_NOT_STARTED )
		{
//		RNG_HandleTypeDef xRND;
		uint32_t ulSeed;

			/* Generate a random number with which to seed the local pseudo random
			number generating function. */
//			HAL_RNG_Init( &xRND );
			HAL_RNG_GenerateRandomNumber( &hrng, &ulSeed );
			prvSRand( ulSeed );
			xInitialised = pdTRUE;
		}
	}

	/* Utility function to generate a pseudo random number. */

	ulNextRand = ( ulMultiplier * ulNextRand ) + ulIncrement;
	return( ( int ) ( ulNextRand >> 16UL ) & 0x7fffUL );
}
/*-----------------------------------------------------------*/
