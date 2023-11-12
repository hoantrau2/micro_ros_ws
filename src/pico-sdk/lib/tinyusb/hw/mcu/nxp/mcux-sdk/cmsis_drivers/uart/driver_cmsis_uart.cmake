#Description: UART CMSIS Driver; user_visible: True
include_guard(GLOBAL)
message("driver_cmsis_uart component is included.")

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/fsl_uart_cmsis.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/.
)

#OR Logic component
if(${MCUX_DEVICE} STREQUAL "K32L2B31A")
    include(driver_uart_dma)
endif()
if(${MCUX_DEVICE} STREQUAL "MK64F12")
    include(driver_uart_edma)
endif()
if(${MCUX_DEVICE} STREQUAL "MK66F18")
    include(driver_uart_edma)
endif()
if(${MCUX_DEVICE} STREQUAL "MKV11Z7")
    include(driver_uart_edma)
endif()
if(${MCUX_DEVICE} STREQUAL "MKV31F51212")
    include(driver_uart_edma)
endif()
if(${MCUX_DEVICE} STREQUAL "MK22F51212")
    include(driver_uart_edma)
endif()
if(${MCUX_DEVICE} STREQUAL "MKE06Z4")
    include(driver_uart)
endif()
if(${MCUX_DEVICE} STREQUAL "MKM34ZA5")
    include(driver_uart_dma)
endif()
if(${MCUX_DEVICE} STREQUAL "MKE02Z4")
    include(driver_uart)
endif()
if(${MCUX_DEVICE} STREQUAL "MKE04Z4")
    include(driver_uart)
endif()
if(${MCUX_DEVICE} STREQUAL "MKM35Z7")
    include(driver_uart_dma)
endif()
if(${MCUX_DEVICE} STREQUAL "MKM34Z7")
    include(driver_uart_dma)
endif()

include(CMSIS_Driver_Include_USART)
