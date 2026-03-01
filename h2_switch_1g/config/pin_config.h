#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// $[CMU]
// [CMU]$

// $[LFXO]
// [LFXO]$

// $[PRS.ASYNCH0]
// [PRS.ASYNCH0]$

// $[PRS.ASYNCH1]
// [PRS.ASYNCH1]$

// $[PRS.ASYNCH2]
// [PRS.ASYNCH2]$

// $[PRS.ASYNCH3]
// [PRS.ASYNCH3]$

// $[PRS.ASYNCH4]
// [PRS.ASYNCH4]$

// $[PRS.ASYNCH5]
// [PRS.ASYNCH5]$

// $[PRS.ASYNCH6]
// [PRS.ASYNCH6]$

// $[PRS.ASYNCH7]
// [PRS.ASYNCH7]$

// $[PRS.ASYNCH8]
// [PRS.ASYNCH8]$

// $[PRS.ASYNCH9]
// [PRS.ASYNCH9]$

// $[PRS.ASYNCH10]
// [PRS.ASYNCH10]$

// $[PRS.ASYNCH11]
// [PRS.ASYNCH11]$

// $[PRS.ASYNCH12]
// [PRS.ASYNCH12]$

// $[PRS.ASYNCH13]
// [PRS.ASYNCH13]$

// $[PRS.ASYNCH14]
// [PRS.ASYNCH14]$

// $[PRS.ASYNCH15]
// [PRS.ASYNCH15]$

// $[PRS.SYNCH0]
// [PRS.SYNCH0]$

// $[PRS.SYNCH1]
// [PRS.SYNCH1]$

// $[PRS.SYNCH2]
// [PRS.SYNCH2]$

// $[PRS.SYNCH3]
// [PRS.SYNCH3]$

// $[GPIO]
// GPIO SWCLK on PA01
#ifndef GPIO_SWCLK_PORT                         
#define GPIO_SWCLK_PORT                          SL_GPIO_PORT_A
#endif
#ifndef GPIO_SWCLK_PIN                          
#define GPIO_SWCLK_PIN                           1
#endif

// GPIO SWDIO on PA02
#ifndef GPIO_SWDIO_PORT                         
#define GPIO_SWDIO_PORT                          SL_GPIO_PORT_A
#endif
#ifndef GPIO_SWDIO_PIN                          
#define GPIO_SWDIO_PIN                           2
#endif

// [GPIO]$

// $[TIMER0]
// TIMER0 CC0 on PD05
#ifndef TIMER0_CC0_PORT                         
#define TIMER0_CC0_PORT                          SL_GPIO_PORT_D
#endif
#ifndef TIMER0_CC0_PIN                          
#define TIMER0_CC0_PIN                           5
#endif

// TIMER0 CC1 on PD04
#ifndef TIMER0_CC1_PORT                         
#define TIMER0_CC1_PORT                          SL_GPIO_PORT_D
#endif
#ifndef TIMER0_CC1_PIN                          
#define TIMER0_CC1_PIN                           4
#endif

// [TIMER0]$

// $[TIMER1]
// TIMER1 CC0 on PD02
#ifndef TIMER1_CC0_PORT                         
#define TIMER1_CC0_PORT                          SL_GPIO_PORT_D
#endif
#ifndef TIMER1_CC0_PIN                          
#define TIMER1_CC0_PIN                           2
#endif

// TIMER1 CC1 on PD03
#ifndef TIMER1_CC1_PORT                         
#define TIMER1_CC1_PORT                          SL_GPIO_PORT_D
#endif
#ifndef TIMER1_CC1_PIN                          
#define TIMER1_CC1_PIN                           3
#endif

// [TIMER1]$

// $[TIMER2]
// [TIMER2]$

// $[TIMER3]
// [TIMER3]$

// $[TIMER4]
// [TIMER4]$

// $[USART0]
// [USART0]$

// $[I2C1]
// [I2C1]$

// $[EUSART1]
// [EUSART1]$

// $[KEYSCAN]
// [KEYSCAN]$

// $[LETIMER0]
// [LETIMER0]$

// $[IADC0]
// [IADC0]$

// $[ACMP0]
// [ACMP0]$

// $[ACMP1]
// [ACMP1]$

// $[VDAC0]
// [VDAC0]$

// $[VDAC1]
// [VDAC1]$

// $[PCNT0]
// [PCNT0]$

// $[HFXO0]
// [HFXO0]$

// $[I2C0]
// [I2C0]$

// $[EUSART0]
// [EUSART0]$

// $[PTI]
// [PTI]$

// $[MODEM]
// [MODEM]$

// $[CUSTOM_PIN_NAME]
#ifndef _PORT                                   
#define _PORT                                    SL_GPIO_PORT_A
#endif
#ifndef _PIN                                    
#define _PIN                                     0
#endif

#ifndef SWCLK_PORT                              
#define SWCLK_PORT                               SL_GPIO_PORT_A
#endif
#ifndef SWCLK_PIN                               
#define SWCLK_PIN                                1
#endif

#ifndef SWDIO_PORT                              
#define SWDIO_PORT                               SL_GPIO_PORT_A
#endif
#ifndef SWDIO_PIN                               
#define SWDIO_PIN                                2
#endif











#ifndef BTN1_PORT                               
#define BTN1_PORT                                SL_GPIO_PORT_B
#endif
#ifndef BTN1_PIN                                
#define BTN1_PIN                                 3
#endif


#ifndef BTN2_PORT                               
#define BTN2_PORT                                SL_GPIO_PORT_B
#endif
#ifndef BTN2_PIN                                
#define BTN2_PIN                                 5
#endif













#ifndef LED2_BLUE_PORT                          
#define LED2_BLUE_PORT                           SL_GPIO_PORT_D
#endif
#ifndef LED2_BLUE_PIN                           
#define LED2_BLUE_PIN                            2
#endif

#ifndef LED2_RED_PORT                           
#define LED2_RED_PORT                            SL_GPIO_PORT_D
#endif
#ifndef LED2_RED_PIN                            
#define LED2_RED_PIN                             3
#endif

#ifndef LED1_RED_PORT                           
#define LED1_RED_PORT                            SL_GPIO_PORT_D
#endif
#ifndef LED1_RED_PIN                            
#define LED1_RED_PIN                             4
#endif

#ifndef LED1_BLUE_PORT                          
#define LED1_BLUE_PORT                           SL_GPIO_PORT_D
#endif
#ifndef LED1_BLUE_PIN                           
#define LED1_BLUE_PIN                            5
#endif

// [CUSTOM_PIN_NAME]$


#endif // PIN_CONFIG_H


