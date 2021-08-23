#ifndef __FSR_H
#define	__FSR_H


#include "BSP.h"

#include "func_adc.h"

void FSR_Init (void);
uint16_t GetFSRForce (void);


TEST FSR_test(void);

TEST FSRCollectExperiment(void);

#endif // __BSP_H

