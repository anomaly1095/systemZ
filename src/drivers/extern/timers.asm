@----------------------------------------------------------------------
@---------------------------------------------------------------------- Drivers accible by apps

@-----------------------------------
@ High speed Advanced control timer situated in the APB2
@-----------------------------------
.section .text.drivers.TIM1, "ax", %progbits

@ make functions callable by apps and kernel


@-----------------------------------
@ Lower speed General purpose Timers situated in the APB1
@-----------------------------------
.section .text.drivers.TIM2_5, "ax", %progbits

@ make functions callable by apps and kernel


@-----------------------------------
@ Higher speed General purpose Timers situated in the APB2
@-----------------------------------
.section .text.drivers.TIM9_11, "ax", %progbits

@ make functions callable by apps and kernel
