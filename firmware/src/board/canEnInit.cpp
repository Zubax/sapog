//
// Created by ramy on 04.03.21.
//

#include "canEnInit.hpp"
#include <cassert>
#include <cstring>
#include <board/internal.hpp>

#if UAVCAN_STM32_CHIBIOS
# include <hal.h>
#elif UAVCAN_STM32_NUTTX
# include <nuttx/arch.h>
# include <nuttx/irq.h>
# include <arch/board/board.h>
#elif UAVCAN_STM32_BAREMETAL
#include <chip.h>
#elif UAVCAN_STM32_FREERTOS
#else
# error "Unknown OS"
#endif

#if (UAVCAN_STM32_CHIBIOS && CH_KERNEL_MAJOR == 2) || UAVCAN_STM32_BAREMETAL
# if !(defined(STM32F10X_CL) || defined(STM32F2XX) || defined(STM32F4XX))
// IRQ numbers
#  define CAN1_RX0_IRQn USB_LP_CAN1_RX0_IRQn
#  define CAN1_TX_IRQn USB_HP_CAN1_TX_IRQn
// IRQ vectors
#  if !defined(CAN1_RX0_IRQHandler) || !defined(CAN1_TX_IRQHandler)
#   define CAN1_TX_IRQHandler   USB_HP_CAN1_TX_IRQHandler
#   define CAN1_RX0_IRQHandler  USB_LP_CAN1_RX0_IRQHandler
#  endif
# endif
#endif

#if (UAVCAN_STM32_CHIBIOS && CH_KERNEL_MAJOR == 3)
#define CAN1_TX_IRQHandler      STM32_CAN1_TX_HANDLER
#define CAN1_RX0_IRQHandler     STM32_CAN1_RX0_HANDLER
#define CAN1_RX1_IRQHandler     STM32_CAN1_RX1_HANDLER
#define CAN1_SCE_IRQHandler     STM32_CAN1_SCE_HANDLER
#define CAN2_TX_IRQHandler      STM32_CAN2_TX_HANDLER
#define CAN2_RX0_IRQHandler     STM32_CAN2_RX0_HANDLER
#define CAN2_RX1_IRQHandler     STM32_CAN2_RX1_HANDLER
#define CAN2_SCE_IRQHandler     STM32_CAN2_SCE_HANDLER
#endif

#if UAVCAN_STM32_NUTTX
# if !defined(STM32_IRQ_CAN1TX) && !defined(STM32_IRQ_CAN1RX0)
#  define STM32_IRQ_CAN1TX      STM32_IRQ_USBHPCANTX
#  define STM32_IRQ_CAN1RX0     STM32_IRQ_USBLPCANRX0
# endif
extern "C"
{
static int can1_irq(const int irq, void*);
#if UAVCAN_STM32_NUM_IFACES > 1
static int can2_irq(const int irq, void*);
#endif
}
#endif

/* STM32F3's only CAN inteface does not have a number. */
#if defined(STM32F3XX)
#define RCC_APB1ENR_CAN1EN     RCC_APB1ENR_CANEN
#define RCC_APB1RSTR_CAN1RST   RCC_APB1RSTR_CANRST
#define CAN1_TX_IRQn           CAN_TX_IRQn
#define CAN1_RX0_IRQn          CAN_RX0_IRQn
#define CAN1_RX1_IRQn          CAN_RX1_IRQn
#define CAN1_SCE_IRQn          CAN_SCE_IRQn
#endif

namespace uavcan_stm32
{
void CanEn() {
    /*
     * CAN1, CAN2
     */
    {
        CriticalSectionLocker lock;
        #if UAVCAN_STM32_NUTTX
                    modifyreg32(STM32_RCC_APB1ENR,  0, RCC_APB1ENR_CAN1EN);
                    modifyreg32(STM32_RCC_APB1RSTR, 0, RCC_APB1RSTR_CAN1RST);
                    modifyreg32(STM32_RCC_APB1RSTR, RCC_APB1RSTR_CAN1RST, 0);
        # if UAVCAN_STM32_NUM_IFACES > 1
                    modifyreg32(STM32_RCC_APB1ENR,  0, RCC_APB1ENR_CAN2EN);
                    modifyreg32(STM32_RCC_APB1RSTR, 0, RCC_APB1RSTR_CAN2RST);
                    modifyreg32(STM32_RCC_APB1RSTR, RCC_APB1RSTR_CAN2RST, 0);
        # endif
        #else
                    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
                    RCC->APB1RSTR |= RCC_APB1RSTR_CAN1RST;
                    RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN1RST;
        # if UAVCAN_STM32_NUM_IFACES > 1
                    RCC->APB1ENR  |=  RCC_APB1ENR_CAN2EN;
                    RCC->APB1RSTR |=  RCC_APB1RSTR_CAN2RST;
                    RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN2RST;
        # endif
        #endif
    }
}
}