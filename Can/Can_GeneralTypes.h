#ifndef CAN_GENERAL_TYPES
#define CAN_GENERAL_TYPES

// uint16: if only Standard IDs are used
// uint32: if also Extended IDs are used
typedef uint32 Can_IdType;



typedef struct Can_PduType_s {
	// the CAN ID, 29 or 11-bit
	Can_IdType 	id;
	// Length, max 8 bytes
	uint8		length;
	// data ptr
	uint8 		*sdu;
	// private data for CanIf,just save and use for callback
	PduIdType   swPduHandle;
} Can_PduType;


/** SWS_Can_91013 */
typedef enum {
  /** UNINIT mode. Default mode of the CAN driver and all
   *  CAN controllers connected to one CAN network after
   *  power on. */
  CAN_CS_UNINIT = 0x0,

  /** STARTED mode. All CAN controllers connected to
   *  one CAN network are started by the CAN driver and
   *  in full-operational mode. */
  CAN_CS_STARTED = 0x01,
  
  /**  STOPPED mode. At least one of all CAN controllers
   *   connected to one CAN network are halted and does
   *   not operate on the bus. */
  CAN_CS_STOPPED = 0x02,

  /** SLEEP mode. At least one of all CAN controllers
   *  connected to one CAN network are set into the
   *  SLEEP mode and can be woken up by request of the
   *  CAN driver or by a network event (must be supported
   *  by CAN hardware) */
  CAN_CS_SLEEP = 0x03
} Can_ControllerStateType;

/** SWS_Can_91003 */
typedef enum {
	CAN_ERRORSTATE_ACTIVE,
	CAN_ERRORSTATE_PASSIVE,
	CAN_ERRORSTATE_BUSOFF
} Can_ErrorStateType;



/** SWS_Can_00429 */
// uint16: if more than 255 hw handles
// uint8: else
#ifdef CAN_USE_EXTENDED_HW_HANDLE
typedef uint16 Can_HwHandleType;
#else
typedef uint8 Can_HwHandleType;
#endif

/** SWS_CAN_00496 */
typedef struct
{
    Can_IdType CanId; /* CAN ID of the CAN L-PDU */
    Can_HwHandleType Hoh; /* ID of the corresponding hardware Object Range */
    uint8 ControllerId; /* Cntrollre provided by Canif clearly identify the corresponding controller */

} Can_HwType;


typedef enum {
	/** Transceiver mode NORMAL */
  CANTRCV_TRCVMODE_NORMAL = 0,
  /** Transceiver mode STANDBY */
  CANTRCV_TRCVMODE_STANDBY,
  /** Transceiver mode SLEEP */
  CANTRCV_TRCVMODE_SLEEP
} CanTrcv_TrcvModeType;


#if 0
// TODO Standard 4-3 classic This should contain all types and constants that are shared among the AUTOSAR moduels Can, CanIf, and CanTrcv.

///todo should this file be included here?




// uint16: if only Standard IDs are used
// uint32: if also Extended IDs are used
typedef uint32 Can_IdType;



typedef enum {
	CAN_T_STOP = 1, // cannot request mode CAN_UNINIT
	CAN_T_START,
	CAN_T_SLEEP,
	CAN_T_WAKEUP
} Can_StateTransitionType;



typedef enum {
	CAN_OK,
	CAN_NOT_OK,
	CAN_BUSY
// 	CAN_WAKEUP,		// Removed in 3.0
} Can_ReturnType;
 
typedef struct {
	// data ptr
  const uint8 		*sdu;
	// the CAN ID, 29 or 11-bit
	Can_IdType 	id;
	// private data for CanIf,just save and use for callback
	PduIdType   swPduHandle;
	// Length, max 8 bytes
	uint8		length;
} Can_PduType;


typedef enum {
  /** Due to an error wake up reason was not detected.
   *  This value may only be reported when error was
   *  reported to DEM before. */
	CANTRCV_WU_ERROR = 0,
	/** The transceiver does not support any information
	 *  for the wakeup reason. */
	CANTRCV_WU_NOT_SUPPORTED,
	/** The transceiver has detected, that the network has
	 *  caused the wake up of the ECU */
	CANTRCV_WU_BY_BUS,
	/** The transceiver has detected a wake-up event at
	 *  one of the transceiver's pins (not at the CAN bus). */
	CANTRCV_WU_BY_PIN,
	/** The transceiver detected, that the network has woken
	 *  the ECU via a request to NORMAL mode */
	CANTRCV_WU_INTERNALLY,
	/** The transceiver has detected, that the "wake up"
	 *  is due to an ECU reset */
	CANTRCV_WU_RESET,
	/** The transceiver has detected, that the "wake up"
	 *  is due to an ECU reset after power on. */
	CANTRCV_WU_POWER_ON
} CanTrcv_TrcvWakeupReasonType;

typedef enum {
  /** The notification for wakeup events is enabled
   *  on the addressed network. */
	CANTRCV_WUMODE_ENABLE = 0,
	/** The notification for wakeup events is disabled
	 *  on the addressed network. */
	CANTRCV_WUMODE_DISABLE,
	/** A stored wakeup event is cleared on the addressed network */
	CANTRCV_WUMODE_CLEAR
} CanIf_TrcvWakeupModeType;

#endif
#endif // CAN_GENERAL_TYPES
