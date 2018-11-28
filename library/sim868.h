//*****************************************************************************
//
//  Prototypes for the SIM868.
//  File:     sim868.h
//  Version:  1.0v
//  Author:   Ronald Rodriguez Ruiz.
//  Date:     August 28, 2017.
//  ---------------------------------------------------------------------------
//  Specifications:
//  This library is meant to run on all kinds of MCUs (8-bit, 16-bit, 32-bit)
//  from different vendors. However, the .h file definitions should be first
//  modified to match the hadware-software interface provided for the host
//  MCU.
//
//*****************************************************************************

#ifndef __SIM868_H__
#define __SIM868_H__

//*****************************************************************************
//
//  The following are defines for the hardware API. These definitions should be
//  modified based on the MCU hadware-software interface for UART and GPIO.
//
//*****************************************************************************

//
//  SIM868 UART Interface
//
#define _sim_data_available()                       (SIM868_GetRxBufferSize() > 0)
#define _sim_read_buffer()                          SIM8868_GetChar()
#define _sim_clear_buffer()                         SIM868_ClearRxBuffer()
#define _sim_send_data(...)                         SIM868_PutString(__VA_ARGS__)

//
//  SIM868 Power Control GPIO
//
#define _sim_state()                                SIM_STATE_Read()
#define _sim_pwm(...)                               SIM_POWER_Write(__VA_ARGS__)

//
//  GNSS UART Interface
//
#define _gnss_data_available()                      (GNSS_GetRxBufferSize() > 0)
#define _gnss_read_buffer()                         GNSS_GetChar()
#define _gnss_clear_buffer()                        GNSS_ClearRxBuffer()

//
//  GNSS Power Control GPIO
//
#define _gnss_enable(...)                           GNSS_ENABLE_Write(__VA_ARGS__)

//
//  DEBUGGING UART Interface
//
#define AT_DEBUG
#define _debug_printf(...)                          DEBUGGER_PutString(__VA_ARGS__)

//
//  Delay function from the hosting MCU,
//  the function should provide a delay in ms.
//
#define _debug_delay(...)                           CyDelay(__VA_ARGS__)

//*****************************************************************************
//
//  The following is an enumeration if the bearer service provider available
//  for this application.
//
//*****************************************************************************

enum Bearer_Service_Provider
{
    M2M,
    ATT,
    IUSACELL,
    TELCEL,
    MOVISTAR
};

//*****************************************************************************
//
//  Prototypes for the API
//
//*****************************************************************************

//
//  SIM868
//
extern uint8_t SIM868_init(void);
extern uint8_t SIM868_get_state(void);
extern uint8_t SIM868_set_power_level(uint8_t state);

//
//  SIM Card
//
extern uint8_t SIM868_sim_card_init(void);

//
//  GPRS/GSM (Mobile Network)
//
extern void SIM868_gprs_set_apn(uint8_t serivce);
extern uint8_t SIM868_gprs_enable(uint8_t state);
extern uint8_t SIM868_gprs_gsm_init(void);

//
//  GNSS (Global Navigation Satellite System )
//
extern uint8_t SIM868_gnss_get_fix_status(void);
extern void SIM868_gnss_set_power_level(uint8_t state);
extern uint8_t SIM868_gnss_get_data(float *lat, float *lon, uint8_t *speed_kph);
extern uint8_t SIM868_gnss_get_seconds(void);
extern uint8_t SIM868_gnss_get_minutes(void);
extern uint8_t SIM868_gnss_get_hour(void);
extern uint8_t SIM868_gnss_get_day(void);
extern uint8_t SIM868_gnss_get_month(void);
extern uint16_t SIM868_gnss_get_year(void);

// Http Protocol
extern void SIM868_http_set_root(char* root);
extern void SIM868_http_set_user_data(char* user_data);
extern void SIM868_http_set_user_agent(char* user_agent);
extern void SIM868_http_set_web_serivce(char* web_service);
extern void SIM868_http_set_content_type(char* content_type);
extern void SIM868_http_set_json_structure(char* json_structure);
extern uint8_t SIM868_http_send_request(uint8_t method, uint8_t max_attempts);

#endif
