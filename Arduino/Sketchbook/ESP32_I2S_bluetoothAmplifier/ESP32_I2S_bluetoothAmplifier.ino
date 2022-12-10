/*!
 * @file  customBluetooth.ino
 * @brief  Completely customize Bluetooth config and use. Codes more prone to being called by underlying API(Application Programming Interface).
 * @details  Try configuring the callback functions related to Bluetooth to improve your Bluetooth speaker!
 * @note  This demo has the same function as bluetoothAmplifier.ino when there is nothing modified.
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  [qsjhyy](yihuan.huang@dfrobot.com)
 * @version  V1.0
 * @date  2022-01-27
 * @url  https://github.com/DFRobot/DFRobot_MAX98357A
 */
#include <Arduino.h>

#include <esp_bt_main.h>   // Bluetooth
#include <esp_bt_device.h>
#include <esp_gap_bt_api.h>
#include <esp_a2dp_api.h>   // Bluetooth A2DP protocol
#include <esp_avrc_api.h>   // Bluetooth AVRC protocol

#include <driver/i2s.h>   // I2S communication


uint8_t remoteAddress[6];   // address of the connected remote Bluetooth device

float _volume = 1.0;   // change the coefficient of audio signal volume
int32_t _sampleRate = 44100;   // I2S communication frequency
bool _avrcConnected = false;   // AVRC connection status

String _metadata = "";   // metadata
uint8_t _metaFlag = 0;   // metadata refresh flag

/**************************************************************
                     Setup And Loop                           
**************************************************************/

void setup(void)
{
  Serial.begin(115200);

  // Initialize I2S
  if (!initI2S(/*_bclk=*/GPIO_NUM_25, /*_lrclk=*/GPIO_NUM_26, /*_din=*/GPIO_NUM_27)){
    Serial.println("Initialize I2S failed !");
  }

  // Initialize bluetooth
  if (!initBluetooth("counstomAmplifier")){
    Serial.println("Initialize bluetooth failed !");
  }

}

void loop()
{
  // 详细参见: https://github.com/espressif/esp-idf/blob/master/components/bt/host/bluedroid/api/include/api/esp_avrc_api.h
  esp_avrc_ct_send_metadata_cmd(6, ESP_AVRC_MD_ATTR_TITLE);   // request metadata from remote Bluetooth device via AVRC command
  delay(3000);
}

/**************************************************************
                           Init                             
**************************************************************/

bool initI2S(int _bclk, int _lrclk, int _din)
{
  static const i2s_config_t i2s_config = {
    .mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_TX),   //Work as host, only transmit data
    .sample_rate = _sampleRate,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,   // 16 bits per sample
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,   // 2-channels
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,   // I2S communication I2S Philips standard, data launch at second BCK
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,   // Interrupt level 1
    .dma_buf_count = 4,   // number of buffers, 128 max.
    .dma_buf_len = 400,   // size of each buffer, AVRC communication may be affected if the value is too high.
    .use_apll = false,   // For the application of a high precision clock, select the APLL_CLK clock source in the frequency range of 16 to 128 MHz. It’s not the case here, so select false.
    .tx_desc_auto_clear = true
  };

  static const i2s_pin_config_t pin_config = {
    .bck_io_num = _bclk,   // serial clock (SCK), aka bit clock (BCK)
    .ws_io_num = _lrclk,   // word select (WS), i.e. command (channel) select, used to switch between left and right channel data
    .data_out_num = _din,   // serial data signal (SD), used to transmit audio data in two's complement format
    .data_in_num = I2S_PIN_NO_CHANGE   // Not used
  };

  if (i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL)){
    Serial.println("Install and start I2S driver failed !");
    return false;
  }
  if (i2s_set_pin(I2S_NUM_0, &pin_config)){
    Serial.println("Set I2S pin number failed !");
    return false;
  }

  return true;
}

bool initBluetooth(const char * _btName)
{
  // Initialize bluedroid
  if (!btStarted() && !btStart()){
    Serial.println("Initialize controller failed");
    return false;
  }
  esp_bluedroid_status_t bt_state = esp_bluedroid_get_status();
  if (bt_state == ESP_BLUEDROID_STATUS_UNINITIALIZED){
    if (esp_bluedroid_init()) {
      Serial.println("Initialize bluedroid failed !");
      return false;
    }
  }
  if (bt_state != ESP_BLUEDROID_STATUS_ENABLED){
    if (esp_bluedroid_enable()) {
      Serial.println("Enable bluedroid failed !");
      return false;
    }
  }
  if (esp_bt_dev_set_device_name(_btName)){
    Serial.println("Set device name failed !");
    return false;
  }

  // Initialize AVRCP
  if (esp_avrc_ct_init()){
    Serial.println("Initialize the bluetooth AVRCP controller module failed !");
    return false;
  }
  if (esp_avrc_ct_register_callback(avrcCallback)){   // non-essential, callback for Bluetooth AVRC protocol
    Serial.println("Register application callbacks to AVRCP module failed !");
    return false;
  }

  // Initialize A2DP
  if (esp_a2d_sink_init()){
    Serial.println("Initialize the bluetooth A2DP sink module failed !");
    return false;
  }
  if (esp_a2d_register_callback(a2dpCallback)){   // non-essential, callback for Bluetooth A2DP protocol
    Serial.println("Register application callbacks to A2DP module failed !");
    return false;
  }
  if (esp_a2d_sink_register_data_callback(audioDataProcessCallback)){   // required, callback for processing data received by Bluetooth
    Serial.println("Register A2DP sink data output function failed !");
    return false;
  }


  // Set discoverability and connectability mode for legacy bluetooth.
  if (esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE)){
    Serial.println("Set discoverability and connectability mode for legacy bluetooth failed !");
    return false;
  }

  return true;
}

/**************************************************************
                           Callback Function                             
**************************************************************/

// This callback function is required to transfer the audio stream data received by Bluetooth to the amplifier via I2S, the audio stream data can be processed here.
// Refer to: https://github.com/espressif/esp-idf/blob/master/examples/bluetooth/bluedroid/classic_bt/a2dp_sink/main/main.c
void audioDataProcessCallback(const uint8_t *data, uint32_t len)
{
  int16_t* data16 = (int16_t*)data;   // convert to 16-bit sample data
  int16_t processedData;   // store the processed audio data
  int count = len/2;   // the number of audio data in int16_t to be processed
  size_t i2s_bytes_write = 0;   // i2s_write() the variable storing the number of data to be written

  for(int i=0; i<count; i++){
    processedData = (int16_t)((*data16) * _volume);   // change the volume of audio data
    data16++;

    i2s_write(I2S_NUM_0, &processedData, 2, &i2s_bytes_write, 10);   // transfer audio data to the amplifier via I2S
  }
}

// Non-required, callback for Bluetooth AVRC protocol, some events are triggered by sending the corresponding command
// e.g. metadata request event: esp_avrc_ct_send_metadata_cmd(num, type);   // request metadata from remote Bluetooth device via AVRC command
void avrcCallback(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param)
{
  esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t *)(param);

  switch (event) {
    /*!< metadata response event */
    case ESP_AVRC_CT_METADATA_RSP_EVT: {
        char *attr_text = (char *) malloc (rc->meta_rsp.attr_length + 1);
        memcpy(attr_text, rc->meta_rsp.attr_text, rc->meta_rsp.attr_length);
        attr_text[rc->meta_rsp.attr_length] = 0;
        _metadata = String(attr_text);
        _metaFlag = rc->meta_rsp.attr_id;
        Serial.print("_metadata : ");
        Serial.println(_metadata);
        // Serial.println(rc->meta_rsp.attr_id);

        free(attr_text);
        break;
      }
    /*!< connection state changed event */
    case ESP_AVRC_CT_CONNECTION_STATE_EVT:{
        /*!< connection established */
        _avrcConnected = rc->conn_stat.connected;
        if(_avrcConnected){
          uint8_t * p = rc->conn_stat.remote_bda;
          Serial.print("remoteAddress : ");
          for(uint8_t i=0; i<6; i++){
            remoteAddress[i] = *(p + i);
            Serial.print(remoteAddress[i], HEX);
            Serial.print("-");
          }
        /*!< disconnecting remote device */
        }else{
          memset(remoteAddress, 0, 6);
        }
        break;
      }
    /*!< passthrough response event */
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT:
    /*!< notification event */
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
    /*!< feature of remote device indication event */
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT:
    /*!< supported notification events capability of peer device */
    case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT:
    /*!< play status response event */
    case ESP_AVRC_CT_PLAY_STATUS_RSP_EVT:
    /*!< set absolute volume response event */
    case ESP_AVRC_CT_SET_ABSOLUTE_VOLUME_RSP_EVT:
      break;
    default:
      // "unhandled AVRC event: %d", event
      break;
  }
}

// Non-required, callback for Bluetooth A2DP protocol, some events are triggered by sending the corresponding command.
void a2dpCallback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t*param)
{
  esp_a2d_cb_param_t *a2d = (esp_a2d_cb_param_t *)(param);

  switch (event) {
    /*!<
     * Audio codec is configured, only used for A2DP SINK
     *
     * @brief ESP_A2D_AUDIO_CFG_EVT
     *
     * struct a2d_audio_cfg_param {
     *     esp_bd_addr_t remote_bda;              /*!< remote bluetooth device address
     *     esp_a2d_mcc_t mcc;                     /*!< A2DP media codec capability information
     * } audio_cfg;                               /*!< media codec configuration information
     */
    case ESP_A2D_AUDIO_CFG_EVT:
    /*!<
     * Connection state changed event
     *
     * @brief  ESP_A2D_CONNECTION_STATE_EVT
     *
     * struct a2d_conn_stat_param {
     *     esp_a2d_connection_state_t state;      /*!< one of values from esp_a2d_connection_state_t
     *     esp_bd_addr_t remote_bda;              /*!< remote bluetooth device address
     *     esp_a2d_disc_rsn_t disc_rsn;           /*!< reason of disconnection for "DISCONNECTED"
     * } conn_stat;                               /*!< A2DP connection status
     */
    case ESP_A2D_CONNECTION_STATE_EVT:
    /*!< audio stream transmission state changed event */
    case ESP_A2D_AUDIO_STATE_EVT:
    /*!< acknowledge event in response to media control commands */
    case ESP_A2D_MEDIA_CTRL_ACK_EVT:
    /*!< indicate a2dp init&deinit complete */
    case ESP_A2D_PROF_STATE_EVT:
      break;
    default:
      // "a2dp invalid cb event: %d", event
      break;
  }
}
