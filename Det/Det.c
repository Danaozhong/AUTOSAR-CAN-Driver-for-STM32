#include "Det.h"
// TODO make a hook out of this so that the CAN Stack can be build as a library #include "excp_handler_if.h"

void Det_Init( void )
{

}

void Det_DeInit( void )
{

}

void Det_ReportError(uint16 ModuleId, uint8 InstanceId, uint8 ApiId, uint8 ErrorId)
{
#if 0
    ExceptionHandler_handle_exception(
            EXCP_MODULE_AUTOSAR_BASE + ModuleId,
            EXCP_TYPE_AUTOSAR_BASE + ErrorId,
            false, __FILE__, __LINE__, ((uint32_t)InstanceId << 4) | (uint32_t)ApiId);
#endif
}
