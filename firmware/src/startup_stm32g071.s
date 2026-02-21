.syntax unified
.cpu cortex-m0plus
.thumb

.global g_pfnVectors
.global Reset_Handler
.global Default_Handler
.weak NMI_Handler
.weak HardFault_Handler
.weak SVC_Handler
.weak PendSV_Handler
.weak SysTick_Handler

.thumb_set NMI_Handler, Default_Handler
.thumb_set HardFault_Handler, Default_Handler
.thumb_set SVC_Handler, Default_Handler
.thumb_set PendSV_Handler, Default_Handler
.thumb_set SysTick_Handler, Default_Handler

.extern SystemInit
.extern main

.section .isr_vector, "a", %progbits
.type pfnVectors, %object
g_pfnVectors:
    .word _estack
    .word Reset_Handler
    .word NMI_Handler
    .word HardFault_Handler
    .word 0,0,0,0,0,0,0
    .word SVC_Handler
    .word 0,0
    .word PendSV_Handler
    .word SysTick_Handler

    .rept 32
        .word Default_Handler
    .endr

.section .text.Reset_Handler, "ax", %progbits
Reset_Handler:
    ldr r0, =_sidata
    ldr r1, =_sdata
    ldr r2, =_edata
1:
    cmp r1, r2
    bcc 2f
    b 3f
2:
    ldr r3, [r0], #4
    str r3, [r1], #4
    b 1b
3:
    ldr r0, =_sbss
    ldr r1, =_ebss
4:
    cmp r0, r1
    bcc 5f
    b 6f
5:
    movs r2, #0
    str r2, [r0], #4
    b 4b
6:
    bl SystemInit
    bl main
    b .

.section .text.Default_Handler, "ax", %progbits
Default_Handler:
    b .
