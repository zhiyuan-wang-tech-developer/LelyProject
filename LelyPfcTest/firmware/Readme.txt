Zhiyuan Wang
2018-07-10

Debugging program process:

01. Debug the project and program starts at main.
02. Continue (F5) the program and now program get stuck when checking Vref ready and Vref fault.
03. Open Window -> Stimulus -> Pin/Register Actions.
04. Click green button on upper left side to apply synchronous stimulus to ADCCON2.BGVRRDY and ADCANCON.WKRDYx.
05. It prompts "synchronous stimulus applied successfully" at buttom.
06. Now program is running the SYS_Tasks() in while loop in main.
07. And UART2 Output window displays "System is initialized!".
08. Click green button again on upper left side to remove synchronous stimulus.
09. It prompts "synchronous stimulus has been removed successfully".

10. Open Window -> Stimulus -> Advanced Pin/Register.
11. Click green button on upper left side to apply synchronous stimulus to ADCDSTAT1.ARDYx or ADCDSTAT2.ARDYx and IFS3.AD1DxIF to trigger ADC data interrupt handlers.
12. And UART2 Output window prints ADC scan result list.
14. Go to 'adc_test_data.txt' to change the ADC raw value for voltage and current load channels and observe printing results.
15. Go to 'adc_temp_data.txt' to change the ADC raw value for temperature channel and observe printing results.

16. led_controller.c file implements 200ms timing. Uncomment SYS_DEBUG_BreakPoint() in case LED_CONTROLLER_STATE_BLINK_LED in led_controller_tasks()
    to observe 200ms timing as well as changes on green LED pin . 

17. Open Window -> Stimulus -> Asynchronous.
18. Click Fire RC13 and then UART2 Output window prints IL12 overcurrent.
19. Click Fire RD6 and then UART2 Output window prints IL34 overcurrent. 