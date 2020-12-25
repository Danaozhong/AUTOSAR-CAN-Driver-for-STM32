/** @addtogroup CanIf CAN Interface
 *  @{ */

/** @file CanIf_Types.h
 *  Definitions of configuration parameters for CAN Interface.
 */

#ifndef CANIF_TYPES_H_
#define CANIF_TYPES_H_

#include "ComStack_Types.h"
#include "Can_GeneralTypes.h"

/** SWS_CANIF_00137 */
typedef enum {
	/** Channel shall be set to the offline mode
	 *  => no transmission and reception */
	CANIF_OFFLINE = 0x00,


	/** Transmit path of the corresponding channel
	 *  shall be disabled */
	CANIF_TX_OFFLINE = 0x01,
    
	/** Transmit path of the corresponding channel
	 *  shall be set to the offline active mode
	 *  => notifications are processed but transmit
	 *  requests are blocked. */
	CANIF_TX_OFFLINE_ACTIVE = 0x02,

	/** Channel shall be set to online mode
	 *  => full operation mode */
	CANIF_ONLINE = 0x03,
} CanIf_PduModeType;

#if 0
// value is bitcoded: bit 0 = rx online, bit 1 = tx online, bit 2 = tx notification online
typedef enum {
	/** Channel is in the offline mode ==> no transmission or reception */
  CANIF_GET_OFFLINE = 0,
  /** Receive path of the corresponding channel is enabled and
   *  transmit path is disabled */
  CANIF_GET_RX_ONLINE = 0x1,
  /** Transmit path of the corresponding channel is enabled and
   *  receive path is disabled */
  CANIF_GET_TX_ONLINE = 0x6,
  /** Channel is in the online mode ==> full operation mode */
  CANIF_GET_ONLINE = 0x7,
  /** Transmit path of the corresponding channel is in
   *  the offline mode ==> transmit notifications are processed but
   *  transmit requests are blocked. The receiver path is disabled. */
  CANIF_GET_OFFLINE_ACTIVE = 0x4,
  /** Transmit path of the corresponding channel is in the offline
   *  active mode ==> transmit notifications are processed but transmit
   *  requests are blocked. The receive path is enabled. */
  CANIF_GET_OFFLINE_ACTIVE_RX_ONLINE = 0x5

} CanIf_PduGetModeType;
#endif
typedef enum {
	/** No transmit or receive event occurred for
	 *  the requested L-PDU. */
	CANIF_NO_NOTIFICATION = 0,
	/** The requested Rx/Tx CAN L-PDU was
	 *  successfully transmitted or received. */
	CANIF_TX_RX_NOTIFICATION

} CanIf_NotifStatusType;

#if CANIF_PUBLIC_MULTIPLE_DRV_SUPPORT
typedef struct {
  void (*CanIf_TxConfirmation)(PduIdType CanTxPduId); // L-PDU id
  void (*CanIf_RxIndication)(const Can_HwType* MailBox, const PduInfoType* PduInfoPtr);
#if CANIF_CTRLDRV_TX_CANCELLATION
  void (*CanIf_CancelTxConfirmation)(const Can_PduType* PduInfoPtr);
#endif
  void (*CanIf_ControllerBusOff)(uint8 Controller);
  void (*CanIf_ControllerModeIndication)(uint8 Controller, CanIf_ControllerModeType ControllerMode);
  uint8 numControllers;
//  Can_HwHandleType numHth;
//  Can_HwHandleType numHrh;
  //uint8 controllerId; // canif controller id
}CanIf_DriverUnitConfigType;
#endif

typedef struct {
    /// can id used for transmission, msb indicates extended id
    Can_IdType id;

    /// data length (DLC)
    uint8 dlc;

    /// can driver controller id to be used for transmission
    uint8 controller;

    /// can driver hth id to be used for transmission
    Can_HwHandleType hth;

    /// upper layer confirmation function, set to null if no confirmation
    void(*user_TxConfirmation)(PduIdType txPduId);

    /// upper layer pdu id passed to callout function
    PduIdType ulPduId;
} CanIf_TxPduConfigType;


typedef struct {
    /// can id used for reception filtering
    ///todo add support for range reception
    Can_IdType id;

    /// min dlc and dlc reported to upper layers. Set to -1 to disable dlc check
    uint8 dlc;

    /// can driver controller id from where to receive lpdu
    uint8 controller;

    /** SWS_CANIF_00012
     upper layer indication function, set to null if no rx indication */
    void(*user_RxIndication)(PduIdType RxPduId, const PduInfoType* PduInfoPtr);

    /// upper layer pdu id passed to callout function
    PduIdType ulPduId;
} CanIf_RxLPduConfigType;




/* rewrite Clemens end */
#if 0
typedef struct {
  union {
    PduIdType lpduId;
    PduIdType *array;
  }pduInfo;
  PduIdType arrayLen; // 0 means no ptr no filtering = fullCan reception
} CanIf_HrHConfigType;
#endif
#if 0
typedef struct {
} CanIf_ControllerConfigType;

typedef struct {
	void(*user_ControllerModeIndication)(uint8 controllerId, CanIf_ControllerModeType controllerMode);
	void(*user_ControllerBusOff)(uint8 controllerId);
//  void(*user_SetWakeupEvent)(asdf)
//  void(*user_TrcvModeIndication)(asdf)
}CanIf_DispatchCfgType;
#endif

#include "CanIf_Cfg.h"
#include "Can_Cfg.h"

typedef struct
{
	uint8 WakeupSupport;
	uint8 CanIfControllerIdRef;
	uint8 CanIfDriverNameRef[30];
	const Can_ControllerConfigType* CanIfInitControllerRef;
} CanIf_ControllerConfigType;


typedef struct
{
	void (*CanIfBusOffNotification)(uint8);

	void (*CanIfWakeUpNotification)(uint8);
	void (*CanIfWakeupValidNotification)(uint8);
	void (*CanIfErrorNotificaton)(uint8);
} CanIf_DispatchConfigType;

typedef struct
{
    union {
        PduIdType lpduId;
        PduIdType *array;
    }pduInfo;
    PduIdType arrayLen; // 0 means no ptr no filtering = fullCan reception
}   CanIf_HrHConfigType;

typedef struct {
	/* Everything in this structure is implementation specific */
    const CanIf_TxPduConfigType* TxPduCfg;
	const CanIf_RxLPduConfigType* RxLpduCfg;

    const CanIf_ControllerConfigType* ControllerConfig;
    const CanIf_DispatchConfigType* DispatchConfig;
    const CanIf_HrHConfigType** canIfHrhCfg;  // This is an array of Hrh objects, for each controller ID
} CanIf_ConfigType;


extern const CanIf_ConfigType CanIf_Config;

#endif /*CANIF_TYPES_H_*/
/** @} */
