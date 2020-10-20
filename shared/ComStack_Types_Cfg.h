
#ifndef COMSTACK_TYPES_CFG_H
#define COMSTACK_TYPES_CFG_H

/*******************************************************************************
*   Includes
*******************************************************************************/
#include "Std_Types.h"
/*******************************************************************************
*   Typedef
*******************************************************************************/

#define COMSTACK_INVALID_PDUID    ((PduIdType)0xFFU)
/*The size of this global type depends on the maximum length of PDUs to be
sent by an ECU.*/
typedef uint16       PduLengthType;

typedef uint8    NetworkHandleType;
typedef uint8    PNCHandleType;
#define COMSTACK_INVALID_NETWORK_HANDLE    (0xFFU)
#endif  /* COMSTACK_TYPES_CFG_H */
