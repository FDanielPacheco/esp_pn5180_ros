/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Introduction
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @file      firmware.cpp
 *
 * @version   2.0
 *
 * @date      14-02-2025
 *
 * @brief     Interface between the ESP-PN5180 and the robot.
 *
 * @author    Fábio D. Pacheco,
 * @email     fabio.d.pacheco@inesctec.pt
 *
 * @copyright Copyright (c) [2024] [Fábio D. Pacheco]
 *
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * VERSION ESP-PN5180-ROS-1
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

#define FIRMWARE_VERSION "1.1.0"

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Imported libraries
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <libserialposix.h>
#include "simpleJson.h"

#include "esp_pn5180_ros/TagCargo.h"
#include "esp_pn5180_ros/TagMrths.h"
#include "esp_pn5180_ros/SetTimeout.h"
#include "esp_pn5180_ros/RequestVersion.h"
#include "esp_pn5180_ros/RequestSetTimeout.h"
#include "esp_pn5180_ros/RequestMrths.h"
#include "esp_pn5180_ros/RequestCargo.h"
#include "esp_pn5180_ros/RequestSerial.h"
#include "esp_pn5180_ros/RequestStop.h"
#include "esp_pn5180_ros/FirmwareVersion.h"

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Definitions for the ESP-PN5180
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

#define BAUD_RATE         B115200                                              //!< The ESP-PN5180 baud rate obtained in it's datasheet                                     
#define TIMEOUT           100                                                  //!< Time until any serial port read operation timeout (in deciseconds)                                
#define SLEEP_TIME        50                                                   //!< ROS pool time (in Hertz)                               
#define BUF_SIZE          256                                                  //!< ROS buffer size                         
#define SERIAL_SIZE       512                                                  //!< Size of the serial input buffer                                 

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Definitions for the ROS system
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

#define NODE_NAME         "rfid_supervisor_node"

#define PUB_LOGS          "/rfid/logs"
#define PUB_VERSION       "/rfid/version"                                      //!< This topic will publish the firware version of the ESP-PN5180
#define PUB_SERIAL        "/rfid/serial"                                       //!< This topic will publish the configuration of the serial port
#define PUB_SERIAL_OUT    "/rfid/serial/out"                                   //!< This topic will publish all the data received from the serial port
#define PUB_SERIAL_IN_MON "/rfid/serial/in/mon"                                //!< This topic will publish all the data sent to the serial port
#define PUB_TAG           "/rfid/tag"                                          //!< This topic will publish the information of a tag in JSON, as receives from the serial port                                     
#define PUB_TAG_CARGO     "/rfid/tag/cargo"                                    //!< This topic will publish the infoamation of a tag according to produtech format, (protocol message) tagCargo      
#define PUB_TAG_MRTHS     "/rfid/tag/mrths"                                    //!< This topic will publish the information of a tag according to mrths format, (protocol message) tagMRTHS

#define SUB_MODE_CARGO     "/rfid/request/cargo"                               //!< This topic will receive the commands to start any of the possible operations including possible strucutre of data related to operation on Cargo data format
#define SUB_MODE_MRTHS     "/rfid/request/mrths"                               //!< This topic will receive the commands to start any of the possible operations including possible strucutre of data related to operations on simple text data format
#define SUB_TIMEOUT        "/rfid/request/setTimeout"                          //!< This topic will receive the time to set up the timeout for read/write functions
#define SUB_SERIAL_IN      "/rfid/serial/in"                                   //!< This topic allows for direct contact with the serial port
#define SUB_STOP           "/rfid/stop"                                        //!< Stop any RFID operation topic
#define SUB_GET_VERSION    "/rfid/version/get"                                 //!< Get firmware version of the device ESP-PN5180

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Definitions for the application commands
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

#define  NONE         0                                                        //!< Unkown mode 
#define  READ_STRUCT  1                                                        //!< Reading the tag's stored struct of data
#define  READ_RFID    2                                                        //!< Performing an RFID read operation
#define  WRITE_STRUCT 3                                                        //!< Writting the new tag's structure to the storage
#define  WRITE_RFID   4                                                        //!< Performing an RFID write operation
#define  STOP_RFID    5                                                        //!< Stop any on going RFID operations
#define  VERSION      6                                                        //!< Get the firmware version of the ESP-PN5180
#define  SET_TIMEOUT  7                                                        //!< Set the desired timeout
#define  CHANGE_MODE  8                                                        //!< Change the tag mode
#define  LAST         255                                                      //!< Last mode selected before in the next iteration

#define  STAGE_FREE   0                                                        //!< If this stage is used it means there is no dependencies on other previous commands
#define  STAGE_SELECT 1                                                        //!< On going state, a process was selected 
#define  STAGE_JSON   2                                                        //!< On going state, a json string was received
#define  STAGE_DONE   3                                                        //!< On going state, a process was done
#define  STAGE_ERROR  4                                                        //!< A process gave an error

#define  MODE_CARGO   0                                                      
#define  MODE_MRTHS   1                                                       

#define  READ  0
#define  WRITE 1

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Prototypes
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief It starts and configures the serial port with the required parameters passed as argument.
 *
 * @param[in] argc The number of arguments passed to the main function
 * @param[in] argv An array with the strings passed to the main function
 *
 * @return Upon success it will return 0. \n 
 *         Otherwise, it will return 1.
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
uint8_t initSerial( int argc, char ** argv );

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief It receives requests to perform communication between the robot and the ESP-PN5180, based on Cargo data format specification.
 *
 * @param msg[in] The request sent by the robot (user).
 *
 * @return none
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
void cargoCallback( const esp_pn5180_ros::RequestCargo::ConstPtr& msg  );

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief It receives requests to perform communication between the robot and the ESP-PN5180, based on MRTHS data format specification.
 *
 * @param msg[in] The request sent by the robot (user).
 *
 * @return none
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
void mrthsCallback( const esp_pn5180_ros::RequestMrths::ConstPtr& msg  );

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief It receives requests to change the timeout clock on the ESP-PN5180.
 *
 * @param msg[in] The request sent by the robot (user).
 *
 * @return none
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
void timeoutCallback( const esp_pn5180_ros::RequestSetTimeout::ConstPtr& msg );

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief It receives the information that the robot wants to send directly to the serial port.
 *
 * @param msg[in] The request sent by the robot (user).
 *
 * @return none
 * 
 * @warning There is no user safety, the communication will flow from the ROS system to the serial port without filter or post-processing.
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
void serialInCallback( const esp_pn5180_ros::RequestSerial::ConstPtr& msg );

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief It receives requests to stop any ongoing RFID transactions, if none are happening it will be ignored.
 *
 * @param msg[in] The request sent by the robot (user).
 *
 * @return none
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
void stopCallback( const esp_pn5180_ros::RequestStop::ConstPtr& msg );

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief It receives requests to reply the device's firmware version, it can be used as a close loop function to check if the ESP-PN5180 is awake.
 *
 * @param msg[in] The request sent by the robot (user).
 *
 * @return none
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
void versionCallback( const esp_pn5180_ros::RequestVersion::ConstPtr& msg );

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Performs an action based on the command passed and current stage.
 *
 * @param[in] cmd The command number from the application custom definition specified by the ESP-PN5180 program.
 * @param[in] stg The stage number represents the stage of the state machine. 
 *
 * @return none
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
void action( const uint8_t cmd, const uint8_t stg );

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Parses the information recieved from the serial port and indicates the command select and it's stage.
 *
 * @param[in] output The information that was received from the serial port, to find for a code and return it.
 * @param[out] stage It will be filled with the current ongoing stage. 
 *
 * @return Upon success it returns the code of the command number from the application custom definition specified by the ESP-PN5180 program. \n 
 *         Otherwise `NONE` is returned.
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
uint8_t parser( const char * output, uint8_t * stage );

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief It receives a string containing the tag information formated in json, and returns a fill structure of the tag MRTHS.
 *
 * @param[in] json The string containing the json data.
 *
 * @return Uppon success returns a filled tag structured in MRTHS data format. \n 
 *         Otherwise a empty tag is returned.
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
esp_pn5180_ros::TagMrths jsonParseMRTHS( const char * json );

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief It receives a string containing the tag information formated in json, and returns a fill structure of the tag Cargo.
 *
 * @param[in] json The string containing the json data.
 *
 * @return Uppon success returns a filled tag structured in Cargo data format. \n 
 *         Otherwise a empty tag is returned.
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
esp_pn5180_ros::TagCargo jsonParseCargo( const char * json );

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief It receives a string containing the json information about the device firmware version.
 *
 * @param[in] json The string containing the json data.
 *
 * @return Uppon success returns a filled FirmwareVersion struct with the device's information. \n 
 *         Otherwise a empty struct is returned.
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
esp_pn5180_ros::FirmwareVersion jsonParseVersion( const char * json );

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Structs
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Flag data structure
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
 typedef struct{
  uint8_t request:1;                                                           //!< Flag that gets trigger by any request                    
  uint8_t read:1;                                                              //!< Flag that gets trigger by an read operation request             
  uint8_t write:1;                                                             //!< Flag that gets trigger by an write operation request
  uint8_t setTimeout:1;                                                        //!< Flag that gets trigger by an desired to change the timeout clock on the device request
} flag_t;

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Global variables
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

serial_t           *serial;                                                     //!< The serial port data structure                            
char               buf[ SERIAL_SIZE ];                                          //!< A buffer to the serial port output                    
std_msgs::String   std_pub_buffer;                                              //!< A buffer used to publish information to topics that use std_msgs::String data format                
std::string        UID;                                                         //!< Variable used to store UID from the recent tag     

uint8_t            mode;                                                        //!< Stores the current selected mode
flag_t             flag = {0};                                                  //!< Flags used to define the program state

ros::Publisher     pub_logs;                                                    //!< This topic is responsible for publishing logs about the ongoing of the system          
ros::Publisher     pub_version;                                                 //!< This topic publishes the version of the ESP-PN5180 requested    
ros::Publisher     pub_serial;                                                  //!< This topic presents the current configuration of the serial port where the ESP-PN5180 is connected
ros::Publisher     pub_serial_out;                                              //!< This topic publishes all the information coming from the serial port
ros::Publisher     pub_serial_in_mon;                                           //!< This topic published all the information going to the serial port          
ros::Publisher     pub_tag;                                                     //!< This topic publishes the information about any tag in JSON format
ros::Publisher     pub_tag_cargo;                                               //!< This topic publishes tag's data in the Cargo data format
ros::Publisher     pub_tag_mrths;                                               //!< This topic publishes tag's data in the MRTHS data format       

ros::Subscriber    sub_cargo;                                                   //!< This topic receives requests for mode Cargo data format  
ros::Subscriber    sub_mrths;                                                   //!< This topic receives requests for mode MRTHS data format  
ros::Subscriber    sub_timeout;                                                 //!< This topic receives requests for changing the timeout structure   
ros::Subscriber    sub_serial_in;                                               //!< This topic is a direct on contact between the robot and the serial port     
ros::Subscriber    sub_stop;                                                    //!< This topic will receive stop requests
ros::Subscriber    sub_version;                                                 //!< This topic will receive get version requests

esp_pn5180_ros::TagCargo   struct_tag_cargo;                                        
esp_pn5180_ros::TagMrths   struct_tag_mrths; 
esp_pn5180_ros::SetTimeout struct_timeout;

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
uint8_t
initSerial( int argc, char ** argv ){
  char pathname[ 64 ];
  
  for( uint8_t i = 0 ; i < argc ; ++i ){                                       //!< Going around all parameters
    if( !strcmp("-p", argv[i] ) )                                              //!< If it's found the -p
      strcpy( pathname, argv[i+1] );                                           //!< Save the path to the file 
  }

  serial = serial_open( pathname, 0, NULL );
  if( !serial ) return 1;
  
  if( -1 == serial_set_baudrate( BAUD_RATE, serial ) ) 
    return 1;
  
  if( -1 == serial_set_rule( TIMEOUT, 0, serial ) ) 
    return 1;

  if( -1 == serial_set_line_state( SERIAL_DTR, false, serial ) ) 
    return 1;
  
  usleep( 2e3 );
  
  if( -1 == serial_set_line_state( SERIAL_DTR, true, serial ) )  
    return 1;  
  
  usleep( 3e6 );
  
  serial_flush( serial, TCIOFLUSH );

  return EXIT_SUCCESS;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
esp_pn5180_ros::FirmwareVersion 
jsonParseVersion( const char * json ){
  esp_pn5180_ros::FirmwareVersion data;
  if( !json ) return data;

  char * s = NULL;
  if( NULL != (s = jsonParse( json, "\"Product\"") ) )     data.icProductVersion = std::string( s );
  if( NULL != (s = jsonParse( json, "\"Firmware\"") ) )    data.icFirmwareVersion = std::string( s );
  if( NULL != (s = jsonParse( json, "\"EEPROM\"") ) )      data.eepromVersion = std::string( s );
  if( NULL != (s = jsonParse( json, "\"ESP-PN5180\"") ) )  data.firmwareVersion = std::string( s );
  if( NULL != s ) free( s );

  return data; 
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
esp_pn5180_ros::TagMrths 
jsonParseMRTHS( const char * json ){
  esp_pn5180_ros::TagMrths tag;
  if( !json ) return tag;

  char * s = NULL;
  if( NULL != (s = jsonParse( json, "\"TEXT\"") ) ) tag.text = std::string( s );
  if( NULL != (s = jsonParse( json, "\"UID\"") ) ) UID = std::string( s );
  if( NULL != s ) free( s );
    
  return tag;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
esp_pn5180_ros::TagCargo 
jsonParseCargo( const char * json ){
  esp_pn5180_ros::TagCargo tag;
  if( !json ) return tag;

  char * s = NULL;
  if( NULL != (s = jsonParse( json, "\"V-ID\"") ) )        tag.vehicle.id = atol( s );
  if( NULL != (s = jsonParse( json, "\"V-DIM-X\"") ) )     tag.vehicle.dimension.x = atol( s );
  if( NULL != (s = jsonParse( json, "\"V-DIM-Y\"") ) )     tag.vehicle.dimension.y = atol( s );
  if( NULL != (s = jsonParse( json, "\"V-DIM-Z\"") ) )     tag.vehicle.dimension.z = atol( s );
  if( NULL != (s = jsonParse( json, "\"C-ID\"") ) )        tag.content.id = atol( s );
  if( NULL != (s = jsonParse( json, "\"C-DIM-X\"") ) )     tag.content.dimension.x = atol( s );
  if( NULL != (s = jsonParse( json, "\"C-DIM-Y\"") ) )     tag.content.dimension.y = atol( s );
  if( NULL != (s = jsonParse( json, "\"C-DIM-Z\"") ) )     tag.content.dimension.z = atol( s );
  if( NULL != (s = jsonParse( json, "\"C-DES-F1\"") ) )    tag.content.destination.f1 = atol( s );
  if( NULL != (s = jsonParse( json, "\"C-DES-F2\"") ) )    tag.content.destination.f2 = atol( s );
  if( NULL != (s = jsonParse( json, "\"C-DES-F3\"") ) )    tag.content.destination.f3 = atol( s );
  if( NULL != (s = jsonParse( json, "\"C-DES-F4\"") ) )    tag.content.destination.f4 = atol( s );
  if( NULL != (s = jsonParse( json, "\"UID\"") ) )         UID = std::string( s );
  if( NULL != s ) free( s );

  return tag;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
void 
cargoCallback( const esp_pn5180_ros::RequestCargo::ConstPtr& msg  ){
  if( READ == msg->operation ){
    ROS_INFO_STREAM( "Request to read Cargo format with id: " << msg->request_id );
    flag.read = 1;
  }
    
  if( WRITE == msg->operation ){
    ROS_INFO_STREAM( "Request to write Cargo format with id: " << msg->request_id );
    flag.write = 1;
    struct_tag_cargo = msg->arg;
  }

  mode = MODE_CARGO;
  action( CHANGE_MODE, STAGE_FREE );
  flag.request = 1;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
void 
mrthsCallback( const esp_pn5180_ros::RequestMrths::ConstPtr& msg  ){
  if( READ == msg->operation ){
    ROS_INFO_STREAM( "Request to read MRTHS format with id: " << msg->request_id );
    flag.read = 1;
  }
    
  if( WRITE == msg->operation ){
    ROS_INFO_STREAM( "Request to write MRTHS format with id: " << msg->request_id );
    flag.write = 1;
    struct_tag_mrths = msg->arg;
  }

  mode = MODE_MRTHS;
  action( CHANGE_MODE, STAGE_FREE );
  flag.request = 1;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
void 
timeoutCallback( const esp_pn5180_ros::RequestSetTimeout::ConstPtr& msg ){
  ROS_INFO_STREAM( "Request to set timeout with id: " << msg->request_id );
  struct_timeout = msg->arg;
  flag.setTimeout = 1;
  action( SET_TIMEOUT, STAGE_FREE );
  flag.request = 1;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
void 
serialInCallback( const esp_pn5180_ros::RequestSerial::ConstPtr& msg ){
  ROS_INFO_STREAM( "Request to send " << msg->arg << " in direct link with id: " << msg->request_id );
  serial_write( serial, "%s\n", msg->arg.c_str( ) );
  flag.request = 1;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
void 
stopCallback( const esp_pn5180_ros::RequestStop::ConstPtr& msg ){
  ROS_INFO_STREAM( "Request to stop with id: " << msg->request_id );
  action( STOP_RFID, STAGE_FREE );
  flag.request = 1;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
void 
versionCallback( const esp_pn5180_ros::RequestVersion::ConstPtr& msg ){
  ROS_INFO_STREAM( "Request to get firmware version with id: " << msg->request_id );
  action( VERSION, STAGE_FREE );
  flag.request = 1;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
uint8_t
parser( const char * output, uint8_t * stg ){
  if( !output || !stg )
    return NONE;

  char code[ 16 ];
  
  if( 0 < sscanf( output, "[WARN] Select %s\n", code ) ){
    *stg = STAGE_SELECT;
    return atoi( code );
  }
  
  if( 0 < sscanf( output, "[WARN] Done %s\n", code ) ){
    *stg = STAGE_DONE;
    return atoi( code );
  }

  if( 0 < sscanf( output, "[ERROR] Error %s\n", code ) ){
    *stg = STAGE_ERROR;
    return atoi( code );
  }

  if( jsonCheck( output ) ){
    *stg = STAGE_JSON;
    return LAST;
  }
  
  return NONE;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
void 
action( const uint8_t cmd, const uint8_t stg ){
  ROS_INFO_STREAM( "Performing an action with command " << (int)cmd << " during stage " << (int)stg );

  switch( cmd ){
    default: 
      break;
    
    case READ_RFID:
      switch( stg ){
        default:
          break;
        
        case STAGE_FREE:
          serial_write( serial, "READ_RFID\n" );
          std_pub_buffer.data = std::string( "READ_RFID" );
          pub_serial_in_mon.publish( std_pub_buffer );
          break;

        case STAGE_DONE:
          action( READ_STRUCT, STAGE_FREE );
          break;
        
        case STAGE_ERROR:
          ROS_INFO_STREAM( "The RFID reading operation got timeout ..." );
          break; 
      }
      break;

    case READ_STRUCT:
      switch( stg ){
        default:
          break;

        case STAGE_FREE:
          serial_write( serial, "READ_STRUCT\n" );
          std_pub_buffer.data = std::string( "READ_STRUCT" );
          pub_serial_in_mon.publish( std_pub_buffer );
          break;

        case STAGE_JSON:
          switch( mode ){
            default:
              break;

            case MODE_MRTHS:
              pub_tag_mrths.publish( jsonParseMRTHS( buf ) ); 
              std_pub_buffer.data = std::string( buf );
              pub_tag.publish( std_pub_buffer );

              break;

            case MODE_CARGO:
              pub_tag_cargo.publish( jsonParseCargo( buf ) ); 
              std_pub_buffer.data = std::string( buf );
              pub_tag.publish( std_pub_buffer );
              break;  
          }
          break;         

        case STAGE_DONE:
          ROS_INFO_STREAM( "Successfully performed an RFID operation, UID: " << UID );
          break;
      }
      break;
    
    case WRITE_STRUCT:
      switch( stg ){
        default:
          break;
        
        case STAGE_FREE:
          serial_write( serial, "WRITE_STRUCT\n" );
          std_pub_buffer.data = std::string( "WRITE_STRUCT" );
          pub_serial_in_mon.publish( std_pub_buffer );
          break;

        case STAGE_SELECT:
          switch( mode ){
            default:
              break;

            case MODE_MRTHS:
              snprintf( buf, sizeof(buf), "{\"TEXT\":\"%s\"}", struct_tag_mrths.text.c_str( ) );
              serial_write( serial, "%s\n", buf );
              std_pub_buffer.data = std::string( buf );
              pub_serial_in_mon.publish( std_pub_buffer );
              break;

            case MODE_CARGO:
              snprintf( buf, sizeof(buf),
                           "{"
                           "\"V-ID\": %d,"
                           "\"V-DIM-X\": %d,"
                           "\"V-DIM-Y\": %d,"
                           "\"V-DIM-Z\": %d,"
                           "\"C-ID\": %d,"
                           "\"C-DIM-X\": %d,"
                           "\"C-DIM-Y\": %d,"
                           "\"C-DIM-Z\": %d,"
                           "\"C-DES-F1\": %d,"
                           "\"C-DES-F2\": %d,"
                           "\"C-DES-F3\": %d,"
                           "\"C-DES-F4\": %d"
                           "}",
                           struct_tag_cargo.vehicle.id,
                           struct_tag_cargo.vehicle.dimension.x,
                           struct_tag_cargo.vehicle.dimension.y,
                           struct_tag_cargo.vehicle.dimension.z,
                           struct_tag_cargo.content.id,
                           struct_tag_cargo.content.dimension.x,
                           struct_tag_cargo.content.dimension.y,
                           struct_tag_cargo.content.dimension.z,
                           struct_tag_cargo.content.destination.f1,
                           struct_tag_cargo.content.destination.f2,
                           struct_tag_cargo.content.destination.f3,
                           struct_tag_cargo.content.destination.f4
                         );
              serial_write( serial, "%s\n", buf );
              std_pub_buffer.data = std::string( buf );
              pub_serial_in_mon.publish( std_pub_buffer );
            break;
          }

          break;

        case STAGE_DONE:
          action( WRITE_RFID, STAGE_FREE );
          break;
      }
      break;

    case WRITE_RFID:
      switch( stg ){
        default:
          break;
        
        case STAGE_FREE:
          serial_write( serial, "WRITE_RFID\n" );
          std_pub_buffer.data = std::string( "WRITE_RFID" );
          pub_serial_in_mon.publish( std_pub_buffer );
          break;

        case STAGE_SELECT:
          break;

        case STAGE_DONE:
          action( READ_STRUCT, STAGE_FREE );
          break;

        case STAGE_ERROR:
          ROS_INFO_STREAM( "The RFID reading operation got timeout ..." );
          break; 
      }
      break;

    case STOP_RFID:
      switch( stg ){
        default:
          break;
        
        case STAGE_FREE:
          serial_write( serial, "STOP_RFID\n" );
          std_pub_buffer.data = std::string( "STOP_RFID" );
          pub_serial_in_mon.publish( std_pub_buffer );
          break;

        case STAGE_DONE:
          ROS_INFO_STREAM( "Successfully stopped any RFID operation ongoing" );
          break;
      }
      break;

    case VERSION:
      switch( stg ){
        default:
          break;
        
        case STAGE_FREE:
          ROS_INFO_STREAM( "Sending VERSION command to the serial port");
          serial_write( serial, "VERSION\n" );
          std_pub_buffer.data = std::string( "VERSION" );
          pub_serial_in_mon.publish( std_pub_buffer );
          break;

        case STAGE_JSON:
          pub_version.publish( jsonParseVersion( buf ) );
          break;

        case STAGE_DONE:
          ROS_INFO_STREAM( "Version obtained successfully" );
          break;
      }
      break;

    case SET_TIMEOUT:
      switch( stg ){
        default:
          break;
        
        case STAGE_FREE:
          serial_write( serial, "SET_TIMEOUT\n" );
          std_pub_buffer.data = std::string( "SET_TIMEOUT" );
          pub_serial_in_mon.publish( std_pub_buffer );
          break;

        case STAGE_SELECT:
          flag.setTimeout = 0;
          snprintf( buf, sizeof(buf), 
                    "{"
                    "\"Timeout\":%d,"
                    "\"State\":%d"
                    "}", 
                    struct_timeout.time, struct_timeout.state );

          serial_write( serial, "%s\n", buf );
          std_pub_buffer.data = std::string( buf );
          pub_serial_in_mon.publish( std_pub_buffer );
          break;

        case STAGE_DONE:
          ROS_INFO_STREAM( "Timeout successfully updated" );
          break;
      }
      break;

    case CHANGE_MODE:
      switch( stg ){
        default:
          break;
        
        case STAGE_FREE:
          serial_write( serial, "CHANGE_MODE\n" );
          std_pub_buffer.data = std::string( "CHANGE_MODE" );
          pub_serial_in_mon.publish( std_pub_buffer );
          break;

        case STAGE_SELECT:
          snprintf( buf, sizeof(buf), "{\"Mode\":%d}", mode );
          serial_write( serial, "%s\n", buf );
          std_pub_buffer.data = std::string( buf );
          pub_serial_in_mon.publish( std_pub_buffer );
          break;

        case STAGE_DONE:
          ROS_INFO_STREAM( "Changed to mode" << mode << " successfully" );

          if( flag.write ){
            action( WRITE_STRUCT, STAGE_FREE );
            flag.write = 0;
          }
          else if( flag.read ){
            action( READ_RFID, STAGE_FREE );
            flag.read = 0;
          }
          break;
      }
      break; 
  }
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int
main( int argc, char ** argv ){
  ros::init(argc, argv, NODE_NAME);                                            // Initialize the ROS node
  ros::NodeHandle  nh;                                                         // Use for the communications between nodes
  
  if( 3 > argc ){
    ROS_ERROR_STREAM( "Not enough arguments, example: rosrun rfid_supervisor_node -p \"/dev/ttyRFID\"\n" );
    return EXIT_FAILURE;
  }

  if( 1 == initSerial( argc, argv ) )                                          // Try to initialize the serial port
    return EXIT_FAILURE;       
  
  ROS_INFO_STREAM( "---------------------------------------");
  ROS_INFO_STREAM( "ESP-PN5180-ROS-1 " << FIRMWARE_VERSION );
  ROS_INFO_STREAM( "contact: fabio.d.pacheco@inesctec.pt" );
  ROS_INFO_STREAM( "---------------------------------------");

  pub_logs = nh.advertise<std_msgs::String>( PUB_LOGS, BUF_SIZE );
  pub_version = nh.advertise<esp_pn5180_ros::FirmwareVersion>( PUB_VERSION, BUF_SIZE );
  pub_serial = nh.advertise<std_msgs::String>( PUB_SERIAL, BUF_SIZE);                   
  pub_serial_out = nh.advertise<std_msgs::String>( PUB_SERIAL_OUT, BUF_SIZE);                   
  pub_serial_in_mon = nh.advertise<std_msgs::String>( PUB_SERIAL_IN_MON, BUF_SIZE);                   
  pub_tag = nh.advertise<std_msgs::String>( PUB_TAG, BUF_SIZE);                   
  pub_tag_cargo = nh.advertise<esp_pn5180_ros::TagCargo>( PUB_TAG_CARGO, BUF_SIZE);                   
  pub_tag_mrths = nh.advertise<esp_pn5180_ros::TagMrths>( PUB_TAG_MRTHS, BUF_SIZE);                   
  
  ROS_INFO_STREAM("Publising logs in " << PUB_LOGS << " ...");
  ROS_INFO_STREAM("Publising firmware version in " << PUB_VERSION << " ...");
  ROS_INFO_STREAM("Publising configuration of the serial port in " << PUB_SERIAL << " ...");
  ROS_INFO_STREAM("Publising output of serial port in " << PUB_SERIAL_OUT << " ...");
  ROS_INFO_STREAM("Publising input monitor of serial port in " << PUB_SERIAL_IN_MON << " ...");
  ROS_INFO_STREAM("Publising content of a tag in JSON in " << PUB_TAG << " ...");
  ROS_INFO_STREAM("Publising content of a tag in Cargo format in " << PUB_TAG_CARGO << " ...");
  ROS_INFO_STREAM("Publising content of a tag in MRTHS format in " << PUB_TAG_MRTHS << " ...");
  ROS_INFO_STREAM( "---------------------------------------");
  
  sub_cargo = nh.subscribe( SUB_MODE_CARGO, BUF_SIZE, cargoCallback );
  sub_mrths = nh.subscribe( SUB_MODE_MRTHS, BUF_SIZE, mrthsCallback );
  sub_timeout = nh.subscribe( SUB_TIMEOUT, BUF_SIZE, timeoutCallback );
  sub_serial_in = nh.subscribe( SUB_SERIAL_IN, BUF_SIZE, serialInCallback ); 
  sub_stop = nh.subscribe( SUB_STOP, BUF_SIZE, stopCallback );
  sub_version = nh.subscribe( SUB_GET_VERSION, BUF_SIZE, versionCallback );

  ROS_INFO_STREAM("Subscribing to receive requests Cargo on (operation: [0-READ, 1-WRITE]) " << SUB_MODE_CARGO << " ...");
  ROS_INFO_STREAM("Subscribing to receive requests MRTHS on (operation: [0-READ, 1-WRITE])" << SUB_MODE_MRTHS << " ...");
  ROS_INFO_STREAM("Subscribing to set up the timeout on " << SUB_TIMEOUT << " ...");
  ROS_INFO_STREAM("Subscribing to perform direct link to the serial port on " << SUB_SERIAL_IN << " ...");
  ROS_INFO_STREAM( "---------------------------------------");

  ros::Rate loop_rate( SLEEP_TIME );                                           // Set the rate to check for requests
  uint8_t stage, code = NONE, oldCode = NONE;

  while( ros::ok( ) ){
    ros::spinOnce( );                                                          // Process incoming messages
    
    if( flag.request ){                                                        // Once someone request a information from the ESP-PN5180 start reading information
      memset( buf, '\0', sizeof(buf) );      

      if( 0 < serial_readLine( buf, sizeof(buf), 0, serial ) ){
        ROS_INFO_STREAM( "Coming from the serial " << std::string(buf) );
        std_pub_buffer.data = std::string( buf );
        pub_serial_out.publish( std_pub_buffer );

        code = parser( buf, &stage );

        if( LAST != code )
          oldCode = code;

        action( oldCode, stage );
      } 
      else
        flag.request = 0;
    }
    loop_rate.sleep( );
  }  

  return 0;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * End of file
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/