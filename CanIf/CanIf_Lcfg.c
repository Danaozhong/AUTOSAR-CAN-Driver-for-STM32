#include "CanIf.h"


void CanIf_RxIndication(Can_HwHandleType hrh, Can_IdType canId, uint8 canDlc, const uint8* canSduPtr) {
	// function callbed by CAN controller. Different names for different driver units
  ///todo instead of accessing canIfHrhCfg directly it should be accessed via the root pbcfg object. Solve this by passing driverUnit instead of hrh
  CanIf_Arc_RxIndication(hrh, canId, canDlc, canSduPtr, 0);
}


