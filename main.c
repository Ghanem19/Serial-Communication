#include "Measurement_ECU.h"
#include "interrupt.h"

int main(void)
{

	/*Initialize the measurement ECU*/
	MEASURE_ECU_Init();
	/* Enable the global interrupt*/
	sei();
    while(1)
    {
    	/* update the ECU state by status of fetch and print push buttons */
    	MEASURE_ECU_Update();

    }
}
