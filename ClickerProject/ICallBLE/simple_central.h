/*
 * simple_central.h
 *
 *  Created on: 10 áôáø× 2017
 *      Author: dor
 */

#ifndef SIMPLE_CENTRAL_H
#define SIMPLE_CENTRAL_H
#ifdef __cplusplus
extern "C"
{
#endif


#include "bcomdef.h"
#include "osal.h"
#include "gap.h"


extern void SimpleBLECentral_init(void);
/*
extern void SimpleBLECentral_taskFxn(UArg a0, UArg a1);

extern void SimpleBLECentral_handleKeys(uint8_t shift, uint8_t keys);
extern void SimpleBLECentral_startDiscovery(void);
extern bool SimpleBLECentral_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                         uint8_t dataLen);
extern void SimpleBLECentral_discoverDevices(void);
extern void SimpleBLECentral_addDeviceInfo(uint8_t *pAddr, uint8_t addrType);
extern bool SimpleBLECentral_findLocalName(uint8_t *pEvtData, uint8_t dataLen);
extern void SimpleBLECentral_addDeviceName(uint8_t i, uint8_t *pEvtData,
										   uint8_t dataLen);
extern void SimpleBLECentral_processPairState(uint8_t pairState, uint8_t status);
extern void SimpleBLECentral_processPasscode(uint16_t connectionHandle,
                                             uint8_t uiOutputs);
*/


#ifdef __cplusplus
}
#endif

#endif /* CENTRAL_H */
