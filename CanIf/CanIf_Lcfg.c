#include "CanIf.h"


//void CanIf_RxIndication(Can_HwHandleType hrh, Can_IdType canId, uint8 canDlc, const uint8* canSduPtr)
void CanIf_RxIndication(const Can_HwType* MailBox, const PduInfoType* PduInfoPtr)
{
  CanIf_Internal_RxIndication(MailBox, PduInfoPtr);
}


