
#ifndef _COMPILER_H_
#define _COMPILER_H_

/*******************************************************************************
*   Includes 
*******************************************************************************/
#include "Compiler_Cfg.h"

/*******************************************************************************
*   Macro 
*******************************************************************************/
#define COMPILER_VENDOR_ID    (0xFFFFU)

#define _CODEWARRIOR_C_MPC5XXX_  
 
/* AUTOMATIC used for the declaration of local pointers */
#define AUTOMATIC

#define TYPEDEF

/*_STATIC_ define for abstraction of compiler keyword static*/
#define STATIC       static

/*NULL_PTR define with a void pointer to zero definition*/
#ifndef NULL_PTR
   #define NULL_PTR  ((void*)0)
#endif

/*_INLINE_ define for abstraction of the keyword inline */
/* These macro are empty because we dont suggest to use inline */
#define INLINE       
#define LOCAL_INLINE

/*******************************************************************************
* PRQA S 3409 
* PRQA S 3410 MISRA-C:2004 Rule 19.10
* PRQA S 3453 MISRA-C:2004 Rule 19.7
* These function-like macros are AUTOSAR standard API, they will be used as 
* prefixion when define functions, variable and so on. So the parameters or the  
* body of the macros cannot be enclosed in ().
* This part of code is verified manually and has no impact.
*******************************************************************************/
/* FUNC macro for the declaration and definition of functions, that ensures 
   correct syntax of function declarations
   rettype     return type of the function
   memclass    classification of the function itself*/
#define FUNC(rettype, memclass) rettype  

/* P2VAR macro for the declaration and definition of pointers in RAM, 
   pointing to variables
   ptrtype     type of the referenced variable memclass
   memclass    classification of the pointer's variable itself
   ptrclass    defines the classification of the pointer's distance
 */
 /* in CW, can not use any key words to locate the memclass 
    we can only use #pragma to locate the memclass which is in MemMap.h
 */
#define P2VAR(ptrtype, memclass, ptrclass) ptrtype * 
 /* in CW, can not use any key words to locate the memclass 
    we can only use #pragma to locate the memclass which is in MemMap.h
 */
#define P2P2VAR(ptrtype,memclass,ptrclass) ptrtype **

/* P2CONST macro for the declaration and definition of pointers in RAM 
   pointing to constants
   ptrtype     type of the referenced data
   memclass    classification of the pointer's variable itself
   ptrclass    defines the classification of the pointer's distance
 */
#define P2CONST(ptrtype, memclass, ptrclass) const ptrtype * 

/* CONSTP2VAR macro for the declaration and definition of constant 
   pointers accessing variables
   ptrtype     type of the referenced data
   memclass    classification of the pointer's variable itself
   ptrclass    defines the classification of the pointer's distance
 */
#define CONSTP2VAR(ptrtype, memclass, ptrclass) ptrtype* const 

/* CONSTP2CONST macro for the declaration and definition of constant pointers 
   accessing constants
   ptrtype     type of the referenced data
   memclass    classification of the pointer's variable itself
   ptrclass    defines the classification of the pointer's distance
 */
#define CONSTP2CONST(ptrtype, memclass, ptrclass) const ptrtype* const 

/* P2FUNC macro for the type definition of pointers to functions
   rettype     return type of the function
   ptrclass    defines the classification of the pointer's distance
   fctname     function name respectivly name of the defined type
 */
#define P2FUNC(rettype, ptrclass, fctname) rettype (* fctname)

/* CONST macro for the declaration and definition of constants
   type        type of the constant
   memclass    classification of the constant itself
 */
#define CONST(type, memclass) const type

/* VAR macro for the declaration and definition of variables
   vartype        type of the variable
   memclass    classification of the variable itself
 */
#define VAR(vartype, memclass) vartype

#endif  /* _COMPILER_H_ */

