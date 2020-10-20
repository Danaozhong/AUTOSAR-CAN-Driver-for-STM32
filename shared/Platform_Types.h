
#ifndef _PLATFORM_TYPES_H_
#define _PLATFORM_TYPES_H_

/*******************************************************************************
*   Includes 
*******************************************************************************/

/*******************************************************************************
*   Macro 
*******************************************************************************/
#define CPU_TYPE_8       (8)
#define CPU_TYPE_16      (16)
#define CPU_TYPE_32      (32)

#define MSB_FIRST        (0)    /* big endian bit ordering */
#define LSB_FIRST        (1)    /* little endian bit ordering */

#define HIGH_BYTE_FIRST  (0)    /* big endian byte ordering */
#define LOW_BYTE_FIRST   (1)    /* little endian byte ordering */

#ifndef TRUE
   #define TRUE   (1)
#endif
    
#ifndef FALSE
   #define FALSE  (0)
#endif


#define CPU_TYPE            CPU_TYPE_16 
#define CPU_BIT_ORDER       LSB_FIRST 
#define CPU_BYTE_ORDER      HIGH_BYTE_FIRST                      

#ifndef TRUE
#define TRUE             1U
#endif

#ifndef FALSE
#define FALSE            0U
#endif

#ifndef NULL
#define NULL         	 ((void*)0)
#endif

/*******************************************************************************
*   Typedef 
*******************************************************************************/
typedef unsigned char boolean;

typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;
typedef unsigned long long uint64;

typedef signed char sint8;
typedef signed short sint16;
typedef signed int sint32;
typedef signed long long sint64;

typedef signed char sint8_least;
typedef unsigned char uint8_least;
typedef signed short sint16_least;
typedef unsigned short uint16_least;
typedef signed long sint32_least;
typedef unsigned long uint32_least;

typedef float  float32;
typedef double float64;

#ifndef BOOL
typedef unsigned char	 BOOL;
#endif

#endif  /* _PLATFORM_TYPES_H_ */

