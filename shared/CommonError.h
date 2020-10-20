

#ifndef COMMON_ERROR_H
#define COMMON_ERROR_H


typedef	unsigned long int       ER_RESULT;			        /* Error result type */

#define	ER_OK					(ER_RESULT)(0x00000000U)	/* OK, no error */
#define	ER_PARA_CONST			(ER_RESULT)(0x00000001U)	/* Parameter is constant */
#define	ER_PARA_RANGE			(ER_RESULT)(0x00000002U)	/* Parameter out of range */
#define	ER_PARA_VALUE			(ER_RESULT)(0x00000004U)	/* Parameter has incorrect value */
#define	ER_OVERTIME				(ER_RESULT)(0x00000008U)	/* Wait overtime */
#define	ER_BUSY					(ER_RESULT)(0x00000010U)	/* Device is busy */
#define	ER_NOT_INIT				(ER_RESULT)(0x00000020U)	/* Device has not been initialized */
#define	ER_NOT_SUPPORT			(ER_RESULT)(0x00000040U)	/* Request not supported */
#define	ER_BUFF_EMPTY			(ER_RESULT)(0x00000080U)	/* Buffer is empty */
#define	ER_BUFF_FULL			(ER_RESULT)(0x00000100U)	/* Buffer is full */
#define	ER_HW_PER				(ER_RESULT)(0x00000200U)	/* HW Peripherals error */
#define	ER_HW_IC				(ER_RESULT)(0x00000400U)	/* External IC error */
#define	ER_ACCESS				(ER_RESULT)(0x00000800U)	/* Can not access the desired area */
#define	ER_CHECK				(ER_RESULT)(0x00001000U)	/* Value Check Error */
#define	ER_BUS_OFF				(ER_RESULT)(0x00002000U)	/* CAN bus OFF */
#define	ER_ABORT				(ER_RESULT)(0x00004000U)	/* Step has been aborted */
#define	ER_OVERFLOW				(ER_RESULT)(0x00008000U)	/* Overflow */
#define	ER_UNKNOW				(ER_RESULT)(0x80000000U)	/* Unknown */

#endif	/* COMMON_ERROR_H */


