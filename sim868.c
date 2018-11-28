//*****************************************************************************
//
//  Prototypes for the SIM868.
//  API and private functions for the SIM868.
//  File:     sim868.c
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

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "sim868.h"

//*****************************************************************************
//
//  The following are defines for the different errors types.
//
//*****************************************************************************

#define NO_ERROR                                    0
#define ERROR_AUTOBAUD_MODE                         1
#define ERROR_SIMCARD_STATUS                        2
#define ERROR_SIMCARD_PIN                           3
#define ERROR_NETWORK_RSSI                          4
#define ERROR_NETWORK_REGISTRATION                  5
#define ERROR_GPRS_SERVICE                          6
#define ERROR_GPRS_CONTEXT                          7
#define ERROR_REPLY                                 8
#define ERROR_HTTP_SERVICE                          9
#define ERROR_HTTP_REQUEST                          10
#define ERROR_HTTP_STATUS_CODE                      11
#define ERROR_JSON_STRUCTURE                        12
#define ERROR_GNSS_FIX                              13
#define ERROR_POWER_STATE                           14

//*****************************************************************************
//
//  The following are defines for the power On/Off and reset operation.
//
//*****************************************************************************

#define OFF                                         0
#define ON                                          1
#define RESET                                       2

//*****************************************************************************
//
//  The following are defines for the system delays, values are given in ms.
//
//*****************************************************************************

#define DEFAULT_TIMEOUT                             500
#define DEBUG_SHORT_DELAY                           1000
#define DEBUG_MEDIUM_DELAY                          2000
#define DEBUG_LONG_DELAY                            3000

//*****************************************************************************
//
//  The following are defines for the different buffers sizes.
//
//*****************************************************************************

#define REPLY_BUFFER_LENGTH                         255
#define APN_BUFFER_LENGTH                           50
#define URL_BUFFER_LENGTH                           55
#define UA_BUFFER_LENGTH                            30
#define CONTENT_BUFFER_LENGTH                       42
#define UD_BUFFER_LENGTH                            52

//*****************************************************************************
//
//  The following are defines for the number of lines to be read on the UART.
//
//*****************************************************************************

#define MULTILINE                                   1
#define SINGLE_LINE                                 0

//*****************************************************************************
//
//  The following are defines for the connection state of the network.
//
//*****************************************************************************

#define CONNECTING                                  0
#define CONNECTED                                   1
#define CLOSING                                     2
#define CLOSED                                      3

//*****************************************************************************
//
//  The following are defines for the HTTP methods.
//
//*****************************************************************************

#define GET                                         0
#define POST                                        1

//*****************************************************************************
//
//  The following are defines for the most common HTTP responds from the
//  the server.
//
//*****************************************************************************

#define OK                                          200
#define CREATED                                     201
#define NOT_FOUND                                   404
#define SERVICE_UNAVAILABLE                         503
#define NETWORK_ERROR                               601

//*****************************************************************************
//
//  The following are defines for the GSM registration status.
//
//*****************************************************************************

#define NOT_REGISTERED                              0
#define REGISTERED_HOME_NET                         1
#define SEARCHING                                   2
#define REGISTRATION_DENIED                         3
#define REGISTRED_NO_HOME_NET                       5

//*****************************************************************************
//
//  The following are structus to manage the SIM868 configurations and data.
//
//*****************************************************************************

//
//  Bearer Serivce.
//  APN: Acces Point Name.
//  User: Name of the service provider.
//  Password: Password of the user.
//
struct Bearer_Service
{
  char *apn, *user, pwd;
};
struct Bearer_Service gs_bearer_config;

//
//  Http Header Parameters.
//  User agent: Identifier for the mobile.
//  User data: Authorization keys.
//  Content type: Media type of the resource.
//  Root: Server application.
//  Web service: Interface to a database server.
//  Json structure: Data packet in json format.
//
struct Http_Header
{
  char *user_agent, *user_data, *content_type;
  char *root, *web_service, *json_structure;
};
struct Http_Header gs_http_header;

//
//  GNSS Date-Time.
//  The date and time accessed from the satellites.
//
struct GNSS_Data_Time
{
  uint8_t seconds, minutes, hour, day, month, year;
};
struct GNSS_Data_Time gs_gnss_data_time;

//*****************************************************************************
//
//  The following are global arrays to store data and variables
//  to save reponse codes and flags.
//
//*****************************************************************************

static uint8_t g_gnss_is_data_fixed;
static uint16_t g_http_status_code;
static char g_sim_buffer[REPLY_BUFFER_LENGTH];
static char g_http_buffer[REPLY_BUFFER_LENGTH];
static char g_gnss_buffer[REPLY_BUFFER_LENGTH];

const static tuint8_t g_utc_hours[] =
{
  0, 1, 2,
  3, 4, 5
};

const static uint8_t g_fix_hours[] =
{
  18, 19, 20,
  21, 22, 23
};

const static uint8_t g_last_day_month[] =
{
  0, 31, 28,
  31, 30, 31,
  30, 31, 31,
  30, 31, 30,
  31
};

//*****************************************************************************
//
//  Prototypes for private functions.
//
//*****************************************************************************

//
//  SIM868.
//
static uint8_t sim_power_up(void);
static uint8_t sim_power_dowm(void);
static void sim_change_state(void);

//
//  SIM Card.
//
static uint8_t sim_card_pin(void);
static uint8_t sim_card_status(void);
static uint8_t sim_card_enable(void);

//
//  GPRS/GSM.
//
static uint16_t gprs_query(void);
static uint8_t gprs_network_rssi(void);
static uint8_t gprs_network_mode(void);
static uint8_t gsm_network_registration(void);

//
//  GNSS (Global Navigation Satellite System )
//
static void gnss_read_data(void);

//
//  Http Protocol.
//
static uint8_t http_init(void);
static uint8_t http_read_all(void);
static uint8_t http_start(uint8_t method);
static uint8_t http_action(uint16_t time_out, uint8_t method);

//
//  Helper functions to verify responses.
//
static void get_reply(char *at, uint16_t time_out);
static void read_line(uint16_t time_out, uint8_t lines);
static uint8_t send_check_reply(char *at, char *reply, uint16_t time_out);
static uint8_t parse_reply(char *reply, uint16_t *v, char divider, uint8_t index);
static uint8_t send_parse_reply(char *at, char *reply, uint16_t *v, char divider, uint8_t index, uint16_t time_out);

//
//  Delay function for long periods.
//
static void delay_seconds(uint8_t seconds);

//*****************************************************************************
//
//  Functions for the API.
//
//*****************************************************************************
//*****************************************************************************
//
//! @brief Initialize the SIM868.
//!
//! This function tries for two seconds to enable the autobaud mode of the
//! SIM868 to establish a synchronize communication be the module and the MCU.
//!
//! @return error_status Result of autobauding, if error_status is equal to
//!                      false, then the operation was successful, if not,
//!                      an error_status occurred.
//
//*****************************************************************************
uint8_t
SIM868_init(void)
{
  uint8_t error_status = false;

  _debug_printf( "Initializing....\r\n\r\n");

  //
  //  Wait two sec for autobauding between the SIM868 and the MCU.
  //
  uint16_t time_out = 2000;
  while (time_out > 0)
  {
    //
    // Flush data.
    //
    while (_sim_data_available())
    {
      _sim_read_buffer();
    }

    //
    //  Waiting for the SIM868 to echo an "OK".
    //
    if (!send_check_reply("AT", "OK", DEFAULT_TIMEOUT))
    {
      break;
    }

    //
    // Flush data.
    //
    while (_sim_data_available())
    {
      _sim_read_buffer();
    }

    //
    //  Waiting for the SIM868 to echo an "AT".
    //
    if (!send_check_reply("AT", "AT", DEFAULT_TIMEOUT))
    {
      break;
    }

    _debug_delay(500);
    time_out -= 500;
  }

  //
  //  Turn off the echo mode.
  //
  send_check_reply("ATE0", "OK", DEFAULT_TIMEOUT);
  _debug_delay(100);

  //
  //  Check if the autobauding was successful.
  //
  if (send_check_reply("ATE0", "OK", DEFAULT_TIMEOUT))
  {
    error_status = ERROR_AUTOBAUD_MODE;
  }
  else
  {
    _debug_printf("SIM868 auto baud, OK!\r\n\r\n");
    _debug_delay(DEBUG_SHORT_DELAY);
  }

  return error_status;
}

//*****************************************************************************
//
//! @brief Powers up/down or resets the SIM868.
//!
//! This function sets the power level of the SIM868 to a desired state.
//!
//! @param[in] state Indicates the operation to be performed.
//!                  Turn Off -> 0.
//!                  Turn On -> 1.
//!                  Reset -> 2.
//!
//! @return error_status Result of powering up/down or reseting, if
//!                      error_status is eqaul to false, then the operation was
//!                      successful, if not, an error_status occurred.
//
//*****************************************************************************
uint8_t
SIM868_set_power_level(uint8_t state)
{
  uint8_t error_status;
  uint8_t curr_state = _sim_state();

  switch (state)
  {
    //
    //  Turn it on only if it's already off.
    //
    case ON:
      if(curr_state == OFF)
      {
        error_status = sim_power_up();
      }
    break;

    //
    //  Turn it off only if it's already on.
    //
    case OFF:
      if(curr_state == ON)
      {
        error_status = sim_power_dowm();
      }
    break;

    case RESET:
      //
      //  Reset it only if it's already on.
      //
      if(curr_state == ON)
      {
        error_status = sim_power_dowm();
        //
        //  Double check it's off, before turn it on.
        //
        if(_sim_state() == OFF)
        {
            _debug_delay(2000);
            error_status = sim_power_up();
        }
      }
      else if(curr_state == OFF)
      {
        //
        //  If it was off, turn it on.
        //
        error_status = sim_power_up();
      }
    break;

    default:
      error_status = ERROR_POWER_STATE;
  }

  return error_status;
}

//*****************************************************************************
//
//! @brief Returns the current state of the SIM868.
//!
//! @return _sim_state() Macro function to read the state of the SIM868 from
//!                      the MCU.
//
//*****************************************************************************
uint8_t
SIM868_get_state(void)
{
    return _sim_state();
}

//*****************************************************************************
//
//! @brief Initialize the SIM868 SIM Card.
//!
//! This function performs several steps required to initilize successfully
//! the SIM Card of the SIM868.
//!
//! @return error_status Result of SIM Card initialize, if error_status is
//!                      equal to false, then the operation was successful, if
//!                      not, an error_status occurred.
//
//*****************************************************************************
uint8_t
SIM868_sim_card_init(void)
{
  uint8_t error_status;

  error_status = sim_card_enable();
  if (error_status)
  {
    return error_status
  }

  error_status = sim_card_status();
  if (error_status)
  {
    return error_status
  }

  error_status = sim_card_pin();
  if (error_status)
  {
    return error_status
  }

  _debug_printf("SIM Card ready!\r\n\r\n");
  _debug_delay(DEBUG_SHORT_DELAY);

  return error_status;
}

//*****************************************************************************
//
//! @brief Set the bearer service provider.
//!
//! This function sets the bearer sevice provider for later usage with
//! connecting to the network.
//!
//! @param[in] serivce Bearer service provider defined in the header file by
//!                    the @enum Bearer_Service_Provider.
//!
//! @return None.
//
//*****************************************************************************
void
SIM868_gprs_set_apn(uint8_t serivce)
{
    switch(serivce)
    {
        case M2M:
            gs_bearer_config.apn = "m2m.amx";
            gs_bearer_config.user = "jasper";
            gs_bearer_config.pwd = "jasper";
        break;

        case ATT:
            gs_bearer_config.apn = "modem.nexteldata.com.mx";
            gs_bearer_config.user = " ";
            gs_bearer_config.pwd = " ";
        break;

        case IUSACELL:
            gs_bearer_config.apn = "modem.nexteldata.com.mx";
            gs_bearer_config.user = " ";
            gs_bearer_config.pwd = " ";
        break;

        case MOVISTAR:
            gs_bearer_config.apn = "internet.movistar.mx";
            gs_bearer_config.user = "movistar";
            gs_bearer_config.pwd = "movistar";
        break;

        case TELCEL:
            gs_bearer_config.apn = "internet.itelcel.com";
            gs_bearer_config.user = "webgprs";
            gs_bearer_config.pwd = "webgprs2003";
        break;

        default:
          _debug_printf("The bearer service provider entered is no available!\n\r");
    }
}

//*****************************************************************************
//
//! @brief Enable the GPRS serivce.
//!
//! This function executes a serie of steps to connect the SIM868 to a 2G
//! network, starting from the configuration of the bearer service provider
//! up to establishing the physical connection.
//!
//! @param[in] state Action to be perform (connect or disconnect).
//!
//! @return error_status Result of enabling the GPRS service, if error_status
//!                      is equal to false, then the operation was successful,
//!                      if not, an error_status occurred.
//
//*****************************************************************************
uint8_t SIM868_gprs_enable(bool state)
{
  uint8_t error_status;
  uint16_t module_state;

  //
  //  Verify that the GPRS modem is attached to the network.
  //
  if(send_parse_reply("AT+CGATT?", "+CGATT: ", &module_state, ',', 0, 20000))
  {
    return ERROR_REPLY;
  }

  //
  //  If the device is not yet attached to GPRS serivce, then attach it.
  //
  if (state && !module_state)
  {
    if (send_check_reply("AT+CGATT=1", "OK", 20000))
    {
      return ERROR_GPRS_SERVICE;
    }
  }

  //
  //  Get the current bearer connection status.
  //
  uint8_t bearer_state = gprs_query();

  //
  //  If the connection is closed, and should be opend (state == ON),
  //  then open the connection.
  //
  if(state && (bearer_state == CLOSED))
  {
    //
    //  Set the bearer profile -> connection type.
    //
    if (send_check_reply("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"", "OK", 10000))
    {
      return ERROR_REPLY;
    }

    char bearer_apn[APN_BUFFER_LENGTH];
    sprintf(bearer_apn, "AT+SAPBR=3,1,\"APN\",\"%s\"", gs_bearer_config.apn);

    //
    //  Set the bearer profile -> access point name.
    //
    if (send_check_reply(bearer_apn, "OK", 10000))
    {
      return ERROR_REPLY;
    }

    char bearer_user[APN_BUFFER_LENGTH];
    sprintf(bearer_user, "AT+SAPBR=3,1,\"USER\",\"%s\"", gs_bearer_config.user);

    //
    //  Set APN -> username.
    //
    if (send_check_reply(bearer_user, "OK", 10000))
    {
      return ERROR_REPLY;
    }

    char bearer_pwd[APN_BUFFER_LENGTH];
    sprintf(bearer_pwd, "AT+SAPBR=3,1,\"PWD\",\"%s\"", gs_bearer_config.pwd);

    //
    //  Set APN -> password.
    //
    if (send_check_reply(bearer_pwd , "OK", 10000))
    {
      return ERROR_REPLY;
    }

    //
    //  Open the GPRS context.
    //
    if (send_check_reply("AT+SAPBR=1,1", "OK", 30000))
    {
      return ERROR_GPRS_CONTEXT;
    }

    if(gprs_query() != CONNECTED)
    {
      return ERROR_GPRS_CONTEXT;
    }

    _debug_printf("Bearer is connected!\r\n\r\n");
  }
  else if(!state && (bearer_state == CONNECTED))
  {
    //
    //  Close the GPRS context.
    //
    if (send_check_reply("AT+SAPBR=0,1", "OK", 30000))
    {
      return  ERROR_GPRS_CONTEXT;
    }

    if(gprs_query() != CLOSED)
    {
      error_status = ERROR_GPRS_CONTEXT;
    }

    _debug_printf("Bearer is closed!\r\n\r\n");
  }

  //
  //  If the device is attached to the GPRS service, an shouldn't be (state == OFF),
  //  then detached it.
  //
  if(!state && module_state)
  {
      if (send_check_reply("AT+CGATT=0", "OK", 20000))
      {
        return ERROR_GPRS_SERVICE;
      }
  }

  _debug_delay(DEBUG_SHORT_DELAY);

  return NO_ERROR;
}

//*****************************************************************************
//
//! @brief Initialization of the GPRS/GSM.
//!
//! This function performs several steps required to initilize successfully
//! the GPRS/GSM network services of the SIM868.
//!
//! @return error_status Result of gprs/gsm initialization, if error_status
//!                      is equal to false, then the operation was successful,
//!                      if not, an error_status occurred.
//
//*****************************************************************************
uint8_t
SIM868_gprs_gsm_init(void)
{
  uint8_t error_status;

  error_status = gprs_network_mode();
  if(error_status)
  {
    return error_status;
  }

  error_status = gprs_network_rssi();
  if(error_status)
  {
    return error_status;
  }

  error_status = gsm_network_registration();
  if(error_status)
  {
    return error_status;
  }

  _debug_printf("GSM network registration, OK!\r\n\r\n");
  _debug_delay(DEBUG_SHORT_DELAY);

  error_status = SIM868_gprs_enable(true);

  return error_status;
}

//*****************************************************************************
//
//! @brief Sends an Http request.
//!
//! This function initilize and executes an new Http request. The number of
//! attempts to perform this operation is passed as a parameter.
//!
//! @param[in] method       Http method to be request (POTS/GET).
//! @param[in] max_attempts Number of attempts to perfrom the full request.
//!
//! @return error_status Result of the Http request, if error_status is equal
//!                      to false, then the operation was successful, if not,
//!                      an error_status occurred.
//
//*****************************************************************************
uint8_t
SIM868_http_send_request(uint8_t method, uint8_t max_attempts)
{
  uint8_t error_status;
  uint8_t attempts;

  //
  //  Initialize Http Service.
  //
  attempts = max_attempts;
  while(attempts--)
  {
    //
    //  Set all http header parameters.
    //
    error_status = http_init();
    if (error_status == NO_ERROR)
    {
      break;
    }
    if (attempts == 0)
    {
      return ERROR_HTTP_SERVICE;
    }
  }

  //
  //  Start Http Session.
  //
  attempts = max_attempts;
  while(attempts--)
  {
    //
    //  Get response from server.
    //
    error_status = http_start(method);
    if(error_status == NO_ERROR)
    {
      break;
    }
    if(attempts-- == 0)
    {
      return ERROR_HTTP_REQUEST;
    }
  }

  return error_status;
}

//*****************************************************************************
//
//! @brief Sets user agent.
//!
//! This function copies the user agent passed into the Http header parameters.
//!
//! @param[in] user_agent Http user agent parameter.
//!
//! @return None.
//
//*****************************************************************************
void
SIM868_http_set_user_agent(char *user_agent)
{
  strcpy(gs_http_header.userAgent, user_agent);
}

//*****************************************************************************
//
//! @brief Sets content-type.
//!
//! This function copies the content-type passed into the Http header
//! parameters.
//!
//! @param[in] content_type Http content-type parameter.
//!
//! @return None.
//
//*****************************************************************************
void
SIM868_http_set_content_type(char *content_type)
{
  strcpy(gs_http_header.content_type, content_type);
}

//*****************************************************************************
//
//! @brief Sets user data.
//!
//! This function copies the user data passed into the Http header
//! parameters.
//!
//! @param[in] user_data Http user data parameter.
//!
//! @return None.
//
//*****************************************************************************
void
SIM868_http_set_user_data(char *user_data)
{
  strcpy(gs_http_header.userData, user_data);
}

//*****************************************************************************
//
//! @brief Sets root.
//!
//! This function copies the root passed into the Http header parameters.
//!
//! @param[in] root Http root parameter.
//!
//! @return None.
//
//*****************************************************************************
void
SIM868_http_set_root(char *root)
{
  strcpy(gs_http_header.root, root);
}

//*****************************************************************************
//
//! @brief Sets web service.
//!
//! This function copies the web service passed into the Http header parameters.
//!
//! @param[in] web_serivce Http web service parameter.
//!
//! @return None.
//
//*****************************************************************************
void
SIM868_http_set_web_serivce(char *web_serivce)
{
  strcpy(gs_http_header.web_service, web_serivce);
}

//*****************************************************************************
//
//! @brief Sets json structure.
//!
//! This function copies the json structure passed into the Http header
//! parameters.
//!
//! @param[in] json_structure Http json structure parameter.
//!
//! @return None.
//
//*****************************************************************************
void
SIM868_http_set_json_structure(char *json_structure)
{
  strcpy(gs_http_header.json_structure, json_structure);
}

//*****************************************************************************
//
//! @brief Sets the GNSS module power state (ON/OFF).
//!
//! @return None.
//
//*****************************************************************************
void
SIM868_gnss_set_power_level(uint8_t state)
{
    _gnss_enable(state);
}

//*****************************************************************************
//
//! @brief Returns the fix data status.
//!
//! This function reads the GNSS serial port to determine if the data is valid.
//!
//! @return g_gnss_is_data_fixed Fix data status.
//
//*****************************************************************************
uint8_t
SIM868_gnss_get_fix_status(void)
{
  //
  //  Read the output data from the GNSS serial port,
  //  this will update the data fix status.
  //
  gnss_read_data();

  return g_gnss_is_data_fixed;
}

//*****************************************************************************
//
//! @brief Get the data from the GNSS output.
//!
//! This function parses and store the GNSS output. The latitud, longitude and
//! speed will be assigened to the address of the variables passed. The data
//! and time can be access through getter fucntions.
//!
//! @return None.
//
//*****************************************************************************
uint8_t
SIM868_gnss_get_data(float *lat, float *lon, uint8_t *speed_kph)
{
  //
  //  IMPORTANT: This functios considers time zone
  //  UTC-6 for the data and time calculations.
  //
  char sec[2];
  char min[2];
  char hour[2];
  char time[10];

  char day[2];
  char month[2];
  char year[2];
  char date[6];

  double latitude;
  double longitude;
  double minutes;
  float degrees;

  uint8_t i;
  uint8_t one_more_day;

  //
  //  Skip output mode.
  //
  char *tok = strtok(g_gnss_buffer, delimiter);
  if (!tok)
  {
    return false;
  }
  //
  //  Grab output time.
  //
  char *timep = strtok(NULL, delimiter);
  if (!timep)
  {
    return false;
  }

  //
  //  Get current time.
  //
  strcpy(time, timep);
  //
  //  Get UTC hour.
  //
  hour[0] = time[0];
  hour[1] = time[1];
  //
  //  Get minutes.
  //
  min[0] = time[2];
  min[1] = time[3];
  //
  //  Get seconds.
  //
  sec[0] = time[4];
  sec[1] = time[5];

  //
  //  Convert seconds, minutes and hours into integers.
  //
  gs_gnss_data_time.seconds = (uint8_t)atoi(sec);
  gs_gnss_data_time.minutes = (uint8_t)atoi(min);
  gs_gnss_data_time.hour = (uint8_t)atoi(hour);

  //
  //  If hour is about to change, then modify it.
  //
  if(gs_gnss_data_time.minutes == 59)
  {
    if(gs_gnss_data_time.hour < 23)
    {
      gs_gnss_data_time.hour += 1;
    }
    else
    {
      gs_gnss_data_time.hour = 0;
    }
  }

  //
  //  Adjust hour to UTC-6
  //
  if(gs_gnss_data_time.hour >= 6)
  {
    gs_gnss_data_time.hour -= 6;
    one_more_day = false;
  }
  else
  {
    //
    // One day must be subtracted from current date.
    //
    one_more_day = true;
    for(i = 0; i < 6; i++)
    {
      if(gs_gnss_data_time.hour == g_utc_hours[i])
      {
        gs_gnss_data_time.hour = g_fix_hours[i];
      }
    }
  }

  //
  //  Skip fix identifier.
  //
  tok = strtok(NULL, delimiter);
  if (!tok)
  {
    return false;
  }
  //
  //  Grab the latitude.
  //
  char *latp = strtok(NULL, delimiter);
  if (!latp)
  {
    return false;
  }
  //
  //  Grab latitude direction.
  //
  char *latdir = strtok(NULL, delimiter);
  if (!latdir)
  {
    return false;
  }
  //
  //  Grab longitude.
  //
  char *longp = strtok(NULL, delimiter);
  if (!longp)
  {
    return false;
  }
  //
  //  Grab longitude direction.
  //
  char *longdir = strtok(NULL, delimiter);
  if (!longdir)
  {
    return false;
  }

  latitude = atof(latp);
  longitude = atof(longp);

  //
  // Convert latitude from minutes to decimal.
  //
  degrees = floor(latitude / 100);
  minutes = latitude - (100 * degrees);
  minutes /= 60;
  degrees += minutes;

  //
  //  Turn direction into + or -
  //
  if (latdir[0] == 'S')
  {
    degrees *= -1;
  }

  *lat = degrees;

  //
  //  Convert longitude from minutes to decimal.
  //
  degrees = floor(longitude / 100);
  minutes = longitude - (100 * degrees);
  minutes /= 60;
  degrees += minutes;

  //
  //  Turn direction into + or -
  //
  if (longdir[0] == 'W')
  {
    degrees *= -1;
  }

  *lon = degrees;

  //
  //  Grab the speed in knots.
  //
  char *speedp = strtok(NULL, delimiter);

  //
  //  Convert to kph.
  //
  *speed_kph = atof(speedp) * 1.852;

  //
  //  Skip track angle.
  //
  tok = strtok(NULL, delimiter);
  if (!tok)
  {
    return false;
  }
  //
  //  Grab date.
  //
  char *datep = strtok(NULL, delimiter);
  if (!datep)
  {
    return false;
  }

  //
  //  Get current date.
  //
  strcpy(date, datep);
  //
  //  Get day.
  //
  day[0] = date[0];
  day[1] = date[1];
  //
  //  Get month.
  //
  month[0] = date[2];
  month[1] = date[3];
  //
  //  Get seconds.
  //
  year[0] = date[4];
  year[1] = date[5];

  //
  //  Convert days, months and years into integers.
  //
  gs_gnss_data_time.day = (uint8_t)atoi(day);
  gs_gnss_data_time.month = (uint8_t)atoi(month);
  gs_gnss_data_time.year = (uint8_t)atoi(year);

  //
  //  Check if there is one day less compare to UTC-6.
  //
  if(one_more_day)
  {
    //
    //  If it is not the first day of the month, then only substract one day.
    //  Otherwise, the current month has to be adjusted and maybe the year too.
    //
    if(gs_gnss_data_time.day == 1)
    {
      //
      //  If it is not the first month of the year, then
      //  only subtract one month, otherwise adjust both.
      //
      if(gs_gnss_data_time.month > 1)
      {
        gs_gnss_data_time.month -= 1;
      }
      else
      {
        gs_gnss_data_time.month = 12;
        gs_gnss_data_time.year -= 1;
      }

      //
      //  Use a table to get the last day of the month.
      //
      for(i = 0; i < 13; i++)
      {
        if(gs_gnss_data_time.month == i)
        {
          gs_gnss_data_time.day = g_last_day_month[i];
        }
      }
    }
    else
    {
      gs_gnss_data_time.day -= 1;
    }
  }
}

//*****************************************************************************
//
//! @brief Returns the seconds of the current time.
//!
//! @return seconds.
//
//*****************************************************************************
uint8_t
SIM868_gnss_get_seconds(void)
{
    return gs_gnss_data_time.seconds;
}

//*****************************************************************************
//
//! @brief Returns the minutes of the current time.
//!
//! @return minutes.
//
//*****************************************************************************
uint8_t
SIM868_gnss_get_minutes(void)
{
    return gs_gnss_data_time.minutes;
}

//*****************************************************************************
//
//! @brief Returns the hour of the current time.
//!
//! @return hour.
//
//*****************************************************************************
uint8_t
SIM868_gnss_get_hour(void)
{
    return gs_gnss_data_time.hour;
}

//*****************************************************************************
//
//! @brief Returns the day of the current date.
//!
//! @return day.
//
//*****************************************************************************
uint8_t
SIM868_gnss_get_day(void)
{
    return gs_gnss_data_time.day;
}

//*****************************************************************************
//
//! @brief Returns the month of the current date.
//!
//! @return month.
//
//*****************************************************************************
uint8_t
SIM868_gnss_get_month(void)
{
    return gs_gnss_data_time.month;
}

//*****************************************************************************
//
//! @brief Returns the year of the current date.
//!
//! @return year.
//
//*****************************************************************************
uint16_t
SIM868_gnss_get_year(void)
{
    return (2000 + gs_gnss_data_time.year);
}

//*****************************************************************************
//
//! @brief Powers up the SIM868.
//!
//! This function change the current state of the SIM868 to power it up.
//!
//! @return error_status Result of powering up, if error_status is equal to
//!                      false, then the operation was successful, if not, an
//!                      error occurred.
//
//*****************************************************************************
static uint8_t
sim_power_up(void)
{
  uint8_t error_status;
  uint16_t time_out = 1000;

  //
  //  Change the state from off to on
  //
  sim_change_state();

  //
  //  Wait for 1 sec to see if the module goes on
  //
  while(time_out--)
  {
    //
    //  Read the current state of the SIM868 to see if it changed
    //
    if(_sim_state())
    {
      error_status = NO_ERROR;
      break;
    }
    else
    {
      error_status = ERROR_POWER_STATE;
    }

    _debug_delay(1);
  }

  return error_status;
}

//*****************************************************************************
//
//  Private Functions.
//
//*****************************************************************************
//*****************************************************************************
//
//! @brief Powers down the SIM868.
//!
//! This function change the current state of the SIM868 to power it down.
//!
//! @return error_status Result of powering down, if error_status is equal to
//!                      false, then the operation was successful, if not, an
//!                      error occurred.
//
//*****************************************************************************
static uint8_t
sim_power_dowm(void)
{
  uint8_t error_status;
  uint16_t time_out = 1000;

  //
  //  Change the state from off to on
  //
  sim_change_state();

  //
  //  Wait for 1 sec to see if the module goes off
  //
  while(time_out--)
  {
    //
    //  Read the current state of the SIM868 to see if it changed
    //
    if(!_sim_state())
    {
      error_status = NO_ERROR;
      break;
    }
    else
    {
      error_status = ERROR_POWER_STATE;
    }

    _debug_delay(1);
  }

  return error_status;
}

//*****************************************************************************
//
//! @brief Changes the SIM868 current state.
//!
//! This function switches the SIM868 state either from on to off or off to on.
//!
//! @return None.
//
//*****************************************************************************
static void
sim_change_state(void)
{
    //
    //  Change the SIM868 current state
    //
    _sim_power(ON);
    _debug_delay(2000);
    _sim_power(OFF);
    _debug_delay(100);
}

//*****************************************************************************
//
//! @brief Enables the SIM Card detecting mode.
//!
//! This function checks if the detecting mode of the SIM Card is already
//! activated, if not, it is activated.
//!
//! @return error_status Result of SIM Card operation, if error_status is
//!                      equal to false, then the operation was successful, if
//!                      not, an error occurred.
//
//*****************************************************************************
static uint8_t
sim_card_enable(void)
{
  uint16_t state;

  //
  //  First check the SIM Card detecting mode, this mode will
  //  allow the SIM868 to detect if the SIM Card is inserted.
  //
  if(send_parse_reply("AT+CSDT?", "+CSDT: ", &state, ',', 0, DEFAULT_TIMEOUT))
  {
    return ERROR_REPLY;
  }

  //
  //  If the detecting mode is not set, then set it.
  //
  if(state != 1)
  {
    if (send_check_reply("AT+CSDT=1", "OK", DEFAULT_TIMEOUT))
    {
      return ERROR_REPLY;
    }
  }

  return NO_ERROR;
}

//*****************************************************************************
//
//! @brief Check the SIM Card status.
//!
//! This function checks if the SIM Card is inserted, if not then an error_status
//! is returned.
//!
//! @return error_status Result of SIM Card operation, if error_status is
//!                      equal to false, then the operation was successful, if
//!                      not, an error occurred.
//
//*****************************************************************************
static uint8_t
sim_card_status(void)
{
  uint16_t state;

  //
  //  Check if the SIM Card is inserted.
  //
  if(send_parse_reply("AT+CSMINS?", "+CSMINS: ", &state, ',', 1, DEFAULT_TIMEOUT))
  {
    return ERROR_REPLY;
  }

  //
  //  If the SIM Card is not inserted, then send an error_status.
  //
  if(state != 1)
  {
    return ERROR_SIMCARD_STATUS;
  }

  return NO_ERROR;
}

//*****************************************************************************
//
//! @brief Check the SIM Card pin.
//!
//! This function checks if the SIM Card requires any password, if it does,
//! then an error_status is returned.
//!
//! @return error_status Result of SIM Card operation, if error_status is
//!                      equal to false, then the operation was successful, if
//!                      not, an error occurred.
//
//*****************************************************************************
static uint8_t
sim_card_pin(void)
{
  //
  //  Request the pin state of the SIM Card.
  //
  get_reply("AT+CPIN?", 5000);

  //
  //  Parse the reply.
  //
  char *p = strstr(g_sim_buffer, "+CPIN: ");
  if (p == 0)
  {
    return ERROR_REPLY;
  }
  p += strlen("+CPIN: ");

  //
  //  Check if the SIM Card require any password.
  //
  if(strcmp(p, "READY") != 0)
  {
    return ERROR_SIMCARD_PIN
  }

  return NO_ERROR;
}

//*****************************************************************************
//
//! @brief Get the status of the network.
//!
//! This function request the status of the bearer connection.
//!
//! @return bearer_status Status of the network.
//
//*****************************************************************************
static uint16_t
gprs_query(void)
{
  uint16_t bearer_status;

  //
  //  Check the status of the bearer service provider.
  //
  if (send_parse_reply("AT+SAPBR=2,1", "+SAPBR: ", &bearer_status, ',', 1, 10000)) return ERROR_REPLY;

  return bearer_status;
}

//*****************************************************************************
//
//! @brief Set network selection.
//!
//! This function sets the automatic network selection mode of the SIM868.
//!
//! @return error_status Result of network selection mode, if error_status is
//!                      equal to false, then the operation was successful, if
//!                      not, an error occurred.
//
//*****************************************************************************
static uint8_t
gprs_network_mode(void)
{
  uint16_t state;

  //
  //  Verify that the currect selection of is in automatic mode.
  //
  if(send_parse_reply("AT+COPS?", "+COPS: ", &state, ',' , 0, DEFAULT_TIMEOUT))
  {
    return ERROR_REPLY;
  }

  //
  //  If the selection mode is no in "automatic network selection", then set it.
  //
  if(state != 0 && error_status == false)
  {
    if (send_check_reply("AT+COPS=0", "OK", DEFAULT_TIMEOUT))
    {
      return ERROR_REPLY;
    }
  }

  return NO_ERROR;
}

//*****************************************************************************
//
//! @brief Evalutes the signal strength (RSSI).
//!
//! This function evalutes if the signal strength reciben by the service
//! provider is strong enough to continue performing.
//!
//! @return error_status Result of network rssi operation, if error_status is
//!                      equal to false, then the operation was successful, if
//!                      not, an error occurred.
//
//*****************************************************************************
static uint8_t
gprs_network_rssi(void)
{
  uint16_t state;

  //
  //  Request the signal strength indicator (RSSI).
  //
  if (send_parse_reply("AT+CSQ", "+CSQ: ", &state, ',', 0, DEFAULT_TIMEOUT))
  {
    return ERROR_REPLY;
  }

  //
  //  Signal strength:
  //  state < 9  - Unusable intensity.
  //  state < 15  - Low intensity.
  //  state < 20  - Medium intensity.
  //  state < 32  - High intensity.
  //
  if(state < 9 && state > 32)
  {
    return ERROR_NETWORK_RSSI;
  }

  return NO_ERROR;
}

//*****************************************************************************
//
//! @brief Evalutes the signal strength (RSSI).
//!
//! This function evalutes if the signal strength reciben by the service
//! provider is strong enough to continue performing.
//!
//! @return error_status Result of network registration, if error_status is
//!                      equal to false, then the operation was successful, if
//!                      not, an error occurred.
//
//*****************************************************************************
static uint8_t
gsm_network_registration(void)
{
  uint16_t status;
  uint8_t error_status;

  //
  //  Perform GSM network registration in periods of 15-30 sec for 2 min.
  //
  uint8_t seconds;
  uint8_t minutes;

  for(minutes = 0; minutes < 2; minutes++)
  {
    seconds = 30;

    while (seconds > 0)
    {
      //
      //  Request the GSM network registration status.
      //
      send_parse_reply("AT+CREG?", "+CREG: ", &status, ',', 1, DEFAULT_TIMEOUT);

      //
      //  Both REGISTERED_HOME_NET and REGISTRED_NO_HOME_NET are successful
      //  network registration results.
      //
      switch(status)
      {
        case NOT_REGISTERED:
          error_status = ERROR_NETWORK_REGISTRATION;
        break;

        case REGISTERED_HOME_NET:
        case REGISTRED_NO_HOME_NET:
          error_status = NO_ERROR;
        break;

        case SEARCHING:
          _debug_printf("Searching network...\r\n\r\n");
        break;

        case REGISTRATION_DENIED:
          _debug_printf("Network registration denied\r\n\r\n");
        break;

        default:
          error_status = ERROR_REPLY;
        break;
      }

      delay_seconds(15);
      seconds -= 15;
    }
  }

  return error_status;
}

//*****************************************************************************
//
//! @brief Initilize an Http request.
//!
//! This function configues the handles any pending request, configues the
//! header parameters for the new Http request.
//!
//! @return error_status Result of the Http request initialization, if
//!                      error_status is equal to false, then the operation was
//!                      successful, if not, an error occurred.
//
//*****************************************************************************
static uint8_t
http_init(void)
{
  //
  //  Handle any pending.
  //
  send_check_reply("AT+HTTPTERM", "OK", DEFAULT_TIMEOUT);

  //
  //  Init HTTP service.
  //
  if(send_check_reply("AT+HTTPINIT", "OK", DEFAULT_TIMEOUT))
  {
    return ERROR_HTTP_SERVICE;
  }

  //
  //  Set bearer profile identifier.
  //
  if (send_check_reply("AT+HTTPPARA=\"CID\",1", "OK", DEFAULT_TIMEOUT))
  {
    return ERROR_REPLY;
  }

  //
  //  Set user agent.
  //
  char userAgent[35];
  sprintf(userAgent, "AT+HTTPPARA=\"UA\",\"%s\"", gs_http_header.user_agent);
  if (send_check_reply(userAgent, "OK", DEFAULT_TIMEOUT))
  {
    return ERROR_REPLY;
  }

  //
  //  Set Content-type.
  //
  char contentType[CONTENT_BUFFER_LENGTH];
  sprintf(contentType, "AT+HTTPPARA=\"CONTENT\",\"%s\"", gs_http_header.content_type);
  if (send_check_reply(contentType, "OK", DEFAULT_TIMEOUT))
  {
    return ERROR_REPLY;
  }

  //
  //  Set user data (Authorization).
  //
  char userData[UD_BUFFER_LENGTH];
  sprintf(userData, "AT+HTTPPARA=\"USERDATA\",\"%s\"", gs_http_header.user_data);
  if (send_check_reply(userData, "OK", DEFAULT_TIMEOUT))
  {
    return ERROR_REPLY;
  }

  //
  //  Set Http URL.
  //
  char URL[URL_BUFFER_LENGTH];
  sprintf(URL, "AT+HTTPPARA=\"URL\",\"%s%s\"", gs_http_header.root, gs_http_header.web_service);
  if (send_check_reply(URL, "OK", DEFAULT_TIMEOUT))
  {
    return ERROR_REPLY;
  }

  return NO_ERROR;
}

//*****************************************************************************
//
//! @brief Perform Http action.
//!
//! This function send an specifc Http request method to the server.
//!
//! @param[in] method   Http request method (POST/GET).
//! @param[in] time_out Time for exectuing the reading operation.
//!
//! @return error_status Result of the Http action, if error_status is equal to
//!                      false, then the operation was successful, if not, an
//!                      error occurred.
//
//*****************************************************************************
static uint8_t
http_action(uint16_t time_out, uint8_t method)
{
  uint16_t status;

  //
  //  Only for POST method.
  //
  if(method)
  {
    //
    //  Prepare the POST : JSON structure of 255 characters
    //  to send and the timing is fixed to 8 seconds.
    //
    if (send_check_reply("AT+HTTPDATA=200,8000", "DOWNLOAD", DEFAULT_TIMEOUT))
    {
      return ERROR_REPLY;
    }

    if (send_check_reply(gs_http_header.json_structure, "OK", 16000))
    {
      return ERROR_JSON_STRUCTURE;
    }

    //
    //  IMPORTANT: The 2 above lines must be completed within 12 seconds
    //  to avoid error_status message such as "JSON structure empty".
    //
  }

  //
  //  Build the Http action query (method).
  //
  char http_method[18];
  sprintf(http_method, "AT+HTTPACTION=%u", method);

  //
  //  Send request.
  //
  if (send_check_reply(http_method, "OK", time_out))
  {
    return ERROR_HTTP_REQUEST;
  }

  //
  //  Parse status response.
  //
  read_line(time_out, SINGLE_LINE);
  if(parse_reply("+HTTPACTION: ", &status, ',', 1))
  {
    return ERROR_REPLY;
  }

  //
  //  Print status from last request.
  //
  char http_status[18];
  sprintf(http_status, "Status code: %lu\r\n\r\n", (unsigned long)status);
  _debug_printf(http_status);

  //
  //  Check if everything went OK.
  //
  if(status == OK || status == CREATED)
  {
    return NO_ERROR;
  }

  //
  //  At this point something went wrong,
  //  hold the status code to check it later.
  //
  g_http_status_code = status;

  return ERROR_HTTP_STATUS_CODE;
}

//*****************************************************************************
//
//! @brief Read server response.
//!
//! This function reads the reponse form the server and copys it into the http
//! buffer.
//!
//! @return error_status Result of the reding operation, if error_status is
//!                      equal to false, then the operation was successful, if
//!                      not, an error occurred.
//
//*****************************************************************************
static uint8_t
http_read_all(void)
{
    get_reply("AT+HTTPREAD", DEFAULT_TIMEOUT);

    //
    //  Check if the server returned an OK.
    //
    if(strstr(g_sim_buffer,"+HTTPREAD: ") == 0)
    {
      return ERROR_REPLY;
    }

    //
    //  Copy response from sim buffer to http buffer.
    //
    read_line(5000, SINGLE_LINE);
    strcpy(g_http_buffer, g_sim_buffer);

    return NO_ERROR;
}

//*****************************************************************************
//
//! @brief Start an Http session.
//!
//! This function start an Http session for the requested method and downloads.
//! the data returned from the server.
//!
//! @param[in] method   Http request method (POST/GET).
//!
//! @return error_status Result of the Http operation, if error_status is equal
//!                      to false, then the operation was successful, if not,
//!                      an error occurred.
//
//*****************************************************************************
static uint8_t
http_start(uint8_t method)
{
    //
    //  Http Request session start.
    //
    error_status = http_action(30000, method);
    if (error_status)
    {
      return error_status;
    }

    //
    //  Http response data.
    //
    error_status = http_read_all();
    if (error_status)
    {
      return error_status;
    }

    //
    //  Terminate Http service.
    //
    if(send_check_reply("AT+HTTPTERM", "OK", DEFAULT_TIMEOUT))
    {
      return ERROR_REPLY;
    }

    _debug_printf("HTTP request, done!\r\n\r\n");
    _debug_delay(DEBUG_SHORT_DELAY);

    return NO_ERROR;
}

//*****************************************************************************
//
//! @brief Read a new line.
//!
//! This function reads a new line coming from the SIM868 serial port.
//!
//! @param[in] time_out Time for exectuing the reading operation.
//! @param[in] lines    Number of lines to be read.
//!
//! @return None.
//
//*****************************************************************************
static void
read_line(uint16_t time_out, uint8_t lines)
{
  uint8_t reply_idx = 0;

  //
  //  Read the SIM868 serial port until time_out
  //  is zero or the sim buffer is full.
  //
  while (time_out--)
  {
    if (reply_idx > REPLY_BUFFER_LENGTH)
    {
      break;
    }

    while(_sim_data_available())
    {
      char incoming_char =  _sim_read_buffer();

      //
      //  Ignore the carriage return.
      //
      if (incoming_char == '\r')
      {
        continue;
      }

      if (incoming_char == '\n')
      {
        //
        //  Ignore the first new line.
        //
        if (reply_idx == 0)
        {
          continue;
        }
        //
        // The second new line is the end of the line.
        //
        if (!lines)
        {
            time_out = 0;
            break;
        }
      }

      //
      //  Store the incoming characters in the buffer.
      //
      g_sim_buffer[reply_idx] = incoming_char;
      reply_idx++;
    }

    if (time_out == 0)
    {
      break;
    }

    _debug_delay(1);
  }

  //
  //  Close the string with a null character.
  //
  g_sim_buffer[reply_idx] = 0;
}

//*****************************************************************************
//
//! @brief Check SIM868 reply.
//!
//! This function compares the expected reply with the reply recieved from
//! the SIM868. The operation is an string comparison.
//!
//! @param[in] at       AT command used in mobile applications.
//! @param[in] reply    Expected reply from the SIM868.
//! @param[in] time_out Time for exectuing the reading operation.
//!
//! @return true/false   Result of an string comparison.
//
//*****************************************************************************
static uint8_t
send_check_reply(char *at, char *reply, uint16_t time_out)
{
    get_reply(at, time_out);
    return (!(strcmp(g_sim_buffer, reply) == 0));
}

//*****************************************************************************
//
//! @brief Get reply from SIM868.
//!
//! This function sends an specifc AT command to the SIM868 and gets its reply.
//!
//! @param[in] at       AT command used in mobile applications.
//! @param[in] time_out Time for exectuing the reading operation.
//!
//! @return None.
//
//*****************************************************************************
static void
get_reply(char *at, uint16_t time_out)
{
    _sim_clear_buffer();

    //
    //  Print AT command.
    //
    #ifdef  AT_DEBUG
        _debug_printf("\t---> ");
        _debug_printf(at);
        _debug_printf("\r\n");
    #endif

    //
    //  Send AT command.
    //
    _sim_send_data(at);
    _sim_send_data("\r\n");

    //
    //  Get reply -> SINGLE_LINE.
    //
    read_line(time_out, SINGLE_LINE);

    //
    //  Print reply received.
    //
    #ifdef  AT_DEBUG
        _debug_printf("\t<--- ");
        _debug_printf(g_sim_buffer);
        _debug_printf("\r\n\r\n");
    #endif
}

//*****************************************************************************
//
//! @brief Send a parsing request.
//!
//! This function gets the reply of the SIM868 and sends a parsing request to
//! retrieve an identifier within the string response returned.
//!
//! @param[in] at       AT command used in mobile applications.
//! @param[in] reply    Expected reply from the SIM868.
//! @param[in] v        Address were the value is stored.
//! @param[in] divider  Character delimiter used by the SIM868.
//! @param[in] index    Index of the character.
//! @param[in] time_out Time for exectuing the reading operation.
//!
//! @return error_status Result of the requested operation, if error_status is
//!                      equal to false, then the operation was successful, if
//!                      not, an error occurred.
//
//*****************************************************************************
static uint8_t
send_parse_reply(char *at, char *reply, uint16_t *v, char divider, uint8_t index, uint16_t time_out)
{
    get_reply(at, time_out);

    if (parse_reply(reply, v, divider, index)) return ERROR_REPLY;

    return NO_ERROR;
}

//*****************************************************************************
//
//! @brief Parse the SIM868 reply.
//!
//! This function parse the SIM868 reply to store the value of identifier within
//! the string response return.
//!
//! @param[in] reply    Expected reply from the SIM868.
//! @param[in] v        Address were the value is stored.
//! @param[in] divider  Character delimiter used by the SIM868.
//! @param[in] index    Index of the character.
//!
//! @return error_status Result of the parsing operation, if error_status is
//!                      equal to false, then the operation was successful, if
//!                      not, an error occurred.
//
//*****************************************************************************
static uint8_t
parse_reply(char *reply, uint16_t *v, char divider, uint8_t index)
{
    uint8_t i;

    //
    //  get the pointer
    //
    char *p = strstr(g_sim_buffer, reply);
    if (p == 0)
    {
      return ERROR_REPLY;
    }

    //
    //  point to the result
    //
    p += strlen(reply);
    for (i=0; i<index; i++)
    {
        //
        //  increment dividers
        //
        p = strchr(p, divider);
        if (!p)
        {
          return ERROR_REPLY;
        }
        p++;
    }

    //
    //  get always the first value
    //
    if(strlen(p) > 1)
    {
      p = strtok(p, ",");
    }

    *v = atoi(p);

    return NO_ERROR;
}

//*****************************************************************************
//
//! @brief Read the output of the GNSS serial port.
//!
//! This function stores in a buffer the $GNRMC output, one of the eight NMEA
//! data outputs that the the serial port returns when is read. This funtion
//! also determines whether  or not the data is valid (fixed).
//!
//! @return None.
//
//*****************************************************************************
static void
gnss_read_data(void)
{
  uint8_t cnt = 0;
  uint8_t gnss_idx = 0;
  char incoming_char;
  char *delimiter = ",";

  //
  //  $GNRMC flags.
  //
  uint8_t gnrmcR = false;
  uint8_t gnrmcM = false;
  uint8_t gnrmcC = false;

  //
  //  Clear all old data from gnss buffer.
  //
  _gnss_clear_buffer();

  //
  //  Wait until one of the eight NMEA data
  //  outputs ($GNRMC) has been recieved.
  //
  while(1)
  {
    if(_gnss_data_available())
    {
      //
      //  Read imcoming data.
      //
      incoming_char =  _gnss_read_buffer();

      if(!gnrmcC)
      {
        //
        //  Check if the 'R' char of $GNRMC was recieved.
        //
        if(incoming_char == 0x52)
        {
          gnrmcR = true;
        }

        //
        //  Check if the 'M' char of $GNRMC was recieved.
        //
        if(incoming_char == 0x4D && gnrmcR)
        {
          gnrmcM = true;
        }

        //
        //  Check if the 'C' char of $GNRMC was recieved.
        //
        if(incoming_char == 0x43 && gnrmcM)
        {
          gnrmcC = true;
        }
      }

      //
      //  If the $GNRMC output has been founded, then get
      //   the data fix status, and the output into gnss buffer.
      //
      if(gnrmcC)
      {
        //
        //  End of $GNRMC output.
        //
        if(incoming_char == 0x0D)
        {
          break;
        }

        //
        //  Check if data is valid (fixed),
        //  'A' -> OK.
        //  'V' -> Invalid.
        //
        if(incoming_char == 0x56)
        {
          g_gnss_is_data_fixed  = false;
          break;
        }
        else if(incoming_char == 0x41)
        {
          if(cnt++ == 30)
          {
            g_gnss_is_data_fixed = true;
          }
        }

        //
        //  Store each character read in the buffer.
        //
        g_gnss_buffer[gnss_idx++] = incoming_char;
      }
    }
  }
}

//*****************************************************************************
//
//! @brief Delay in seconds.
//!
//! This function delays the program exectuing for an specific number of
//! seconds.
//!
//! @return None.
//
//*****************************************************************************
static void
delay_seconds(uint8_t seconds)
{
    uint8_t cnt;
    for(cnt = 0; cnt < seconds; cnt++)
    {
        _debug_delay(DEBUG_SHORT_DELAY);
    }
}
