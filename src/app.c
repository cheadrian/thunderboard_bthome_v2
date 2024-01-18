/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#include "em_common.h"
#include "sl_bluetooth.h"
#include "app.h"
#include "bthome_v2.h"

#include "sl_status.h"
#include "sl_power_supply.h"
#include "sl_simple_button_instances.h"
#include "app_log.h"
#include "sl_sleeptimer.h"
#include <stdbool.h>
#include <stdio.h>
#include "sl_mic.h"

/**************************************************************************//**
 * Include the sensors definition
 *****************************************************************************/
#ifdef SL_COMPONENT_CATALOG_PRESENT
#include "sl_component_catalog.h"
#endif // SL_COMPONENT_CATALOG_PRESENT
#ifdef SL_CATALOG_SENSOR_RHT_PRESENT
#include "sl_sensor_rht.h"
#endif // SL_CATALOG_SENSOR_RHT_PRESENT
#ifdef SL_CATALOG_SENSOR_HALL_PRESENT
#include "sl_sensor_hall.h"
#endif // SL_CATALOG_SENSOR_HALL_PRESENT
#ifdef SL_CATALOG_SENSOR_LIGHT_PRESENT
#include "sl_sensor_light.h"
#endif // SL_CATALOG_SENSOR_LIGHT_PRESENT
#ifdef SL_CATALOG_SENSOR_IMU_PRESENT
#include "sl_sensor_imu.h"
#endif // SL_CATALOG_SENSOR_IMU_PRESENT
#ifdef SL_CATALOG_SENSOR_PRESSURE_PRESENT
#include "sl_sensor_pressure.h"
#endif // SL_CATALOG_SENSOR_PRESSURE_PRESENT
#ifdef SL_CATALOG_SENSOR_GAS_PRESENT
#include "sl_sensor_gas.h"
#endif // SL_CATALOG_SENSOR_GAS_PRESENT
#ifdef SL_CATALOG_SENSOR_SOUND_PRESENT
#include "sl_sensor_sound.h"
#endif // SL_CATALOG_SENSOR_SOUND_PRESENT

/**************************************************************************//**
 * Define the name and encryption key for BTHome V2.
 * Note: encryption disabled by default in this example
 * modify the bthome_v2_init() to enable.
 *****************************************************************************/
static uint8_t name[] = "TBS2";
static uint8_t key[] = "231d39c1d7cc1ab1aee224cd096db932";

/**************************************************************************//**
 * Increase the advertising interval to longer the battery life
 *****************************************************************************/
#define ADVERTISING_INTERVAL 60000
#define BUTTON_NONE_TIMEOUT 250
#define DSLEEP_TIMEOUT 500
#define ADV_INTERVAL_TIMER_EXT_SIGNAL 0x01
#define DEEP_SLEEP_TIMER_EXT_SIGNAL 0xB2
#define EVENT_BUTTON_B0 0xA0
#define EVENT_BUTTON_B1 0xA1
#define BUTTON_NONE_EXT_SIG 0xA5
#define BLE_ADVERTISE_TIME_INTERVAL_MS 100

static sl_sleeptimer_timer_handle_t adv_interval_timer;
static sl_sleeptimer_timer_handle_t deep_sleep_timer;
static sl_sleeptimer_timer_handle_t button_none_timer;

static void sensor_init(void);
static void sensor_deinit(void);
static void bthome_add_sensor_data_and_battery(void);
static void start_button_none_timer(void);
static void start_deep_sleep_timer(void);
static void send_text_to_bthome_v2(const char text[]);
static void send_data_packet_to_bthome_v2(void);

static void adv_interval_timer_callback(sl_sleeptimer_timer_handle_t *timer,
                               void *data);
static void deep_sleep_timer_callback(sl_sleeptimer_timer_handle_t *timer,
                               void *data);
static void button_none_timer_callback(sl_sleeptimer_timer_handle_t *timer,
                                       void *data);

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
void app_init(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
  app_log_info("===== BTHome v2 - Thunderboard application =====" APP_LOG_NL);
  app_log_info("======= BTHome v2 initialization =========" APP_LOG_NL);

  // Make power management to request EM2 state
  sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM2);
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
void app_process_action(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  bthome_v2_bt_on_event(evt);

  switch (SL_BT_MSG_ID(evt->header)) {
    case sl_bt_evt_system_boot_id:
      /////////////////////////////////////////////////////////////////////////////
      // You can enable encryption here by setting the second parameter to true. //
      // Home Assistant support encryption for BTHome V2 protocol.               //
      // The key is set by the variable `key` defined above.                     //
      /////////////////////////////////////////////////////////////////////////////
      bthome_v2_init(name, false, key, false);
      bthome_add_sensor_data_and_battery();
      bthome_v2_add_measurement_state(EVENT_BUTTON, EVENT_BUTTON_NONE, 0);
      bthome_v2_send_packet();

      // Add an timer to send data every ADVERTISING_INTERVAL
      sl_sleeptimer_start_periodic_timer_ms(&adv_interval_timer,
                                            ADVERTISING_INTERVAL,
                                            adv_interval_timer_callback,
                                            NULL,
                                            0,
                                            0);
      start_deep_sleep_timer();
      break;
    // -------------------------------
    // external_signal event handler.
    case sl_bt_evt_system_external_signal_id:
      if (evt->data.evt_system_external_signal.extsignals
          == EVENT_BUTTON_B0) {
          app_log_info("Button 0 press event" APP_LOG_NL);
          bthome_v2_reset_measurement();
          bthome_v2_add_measurement_state(EVENT_BUTTON, EVENT_BUTTON_PRESS, 0);
          bthome_v2_send_packet();
          start_button_none_timer();
      }
      else if (evt->data.evt_system_external_signal.extsignals
          == EVENT_BUTTON_B1) {
          app_log_info("Button 1 press event" APP_LOG_NL);
          bthome_v2_reset_measurement();
          bthome_v2_add_measurement_state(EVENT_BUTTON, EVENT_BUTTON_NONE, 0);
          bthome_v2_add_measurement_state(EVENT_BUTTON, EVENT_BUTTON_PRESS, 0);
          bthome_v2_send_packet();
          start_button_none_timer();
      }
      else if (evt->data.evt_system_external_signal.extsignals
          == BUTTON_NONE_EXT_SIG) {
          app_log_info("[BTHome]: Send BUTTON_NONE event." APP_LOG_NL);
          bthome_v2_reset_measurement();
          bthome_v2_add_measurement_state(EVENT_BUTTON, EVENT_BUTTON_NONE, 0);
          bthome_v2_build_packet();
          start_deep_sleep_timer();
      }
      else if (evt->data.evt_system_external_signal.extsignals
          == ADV_INTERVAL_TIMER_EXT_SIGNAL) {
          app_log_info("App timer event" APP_LOG_NL);
          bthome_add_sensor_data_and_battery();
          start_deep_sleep_timer();
      }
      else if (evt->data.evt_system_external_signal.extsignals
          == DEEP_SLEEP_TIMER_EXT_SIGNAL) {
          app_log_info("Stop advertising to enter in EM2 power mode" APP_LOG_NL);
          bthome_v2_stop();
      }
      break;
    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}

/***************************************************************************//**
 * Callback on button change.
 ******************************************************************************/
void sl_button_on_change(const sl_button_t *handle)
{
  if (sl_button_get_state(handle) == SL_SIMPLE_BUTTON_PRESSED) {
    if (handle == &sl_button_btn0) {
      sl_bt_external_signal(EVENT_BUTTON_B0);
    }
    if (handle == &sl_button_btn1) {
      sl_bt_external_signal(EVENT_BUTTON_B1);
    }
  }
}

/***************************************************************************//**
 * Sensor batch init
 ******************************************************************************/
static void sensor_init(void)
{
  sl_status_t sc;
#ifdef SL_CATALOG_SENSOR_RHT_PRESENT
  sc = sl_sensor_rht_init();
  if (sc != SL_STATUS_OK) {
    app_log_warning("Relative Humidity and Temperature sensor initialization failed." APP_LOG_NL);
  }
#endif // SL_CATALOG_SENSOR_RHT_PRESENT
#ifdef SL_CATALOG_SENSOR_HALL_PRESENT
  sc = sl_sensor_hall_init();
  if (sc != SL_STATUS_OK) {
    app_log_warning("Hall sensor initialization failed." APP_LOG_NL);
  }
#endif // SL_CATALOG_SENSOR_HALL_PRESENT
#ifdef SL_CATALOG_SENSOR_LIGHT_PRESENT
  sc = sl_sensor_light_init();
  if (sc != SL_STATUS_OK) {
    app_log_warning("Ambient light and UV index sensor initialization failed." APP_LOG_NL);
  }
#endif // SL_CATALOG_SENSOR_LIGHT_PRESENT
#ifdef SL_CATALOG_SENSOR_IMU_PRESENT
  if (!sl_power_supply_is_low_power()) {
    sl_sensor_imu_init();
    sc = sl_sensor_imu_enable(true);
    if (SL_STATUS_OK != sc) {
        app_log_warning("Inertial Measurement Unit sensor sensor initialization failed." APP_LOG_NL);
    }
  }
#endif // SL_CATALOG_SENSOR_IMU_PRESENT
#ifdef SL_CATALOG_SENSOR_PRESSURE_PRESENT
  sc = sl_sensor_pressure_init();
  if (sc != SL_STATUS_OK) {
    app_log_warning("Air Pressure sensor initialization failed." APP_LOG_NL);
  }
#endif // SL_CATALOG_SENSOR_PRESSURE_PRESENT
#ifdef SL_CATALOG_SENSOR_GAS_PRESENT
  if (!sl_power_supply_is_low_power()) {
    sc = sl_sensor_gas_init();
    if (sc != SL_STATUS_OK) {
      app_log_warning("Air quality sensor initialization failed." APP_LOG_NL);
    }
  }
#endif // SL_CATALOG_SENSOR_GAS_PRESENT
#ifdef SL_CATALOG_SENSOR_SOUND_PRESENT
  sc = sl_sensor_sound_init();
  if (sc != SL_STATUS_OK) {
    app_log_warning("Sound level sensor initialization failed." APP_LOG_NL);
  }
#endif // SL_CATALOG_SENSOR_SOUND_PRESENT
}

/***************************************************************************//**
 * Sensor batch deinit
 ******************************************************************************/
static void sensor_deinit(void)
{
#ifdef SL_CATALOG_SENSOR_RHT_PRESENT
  sl_sensor_rht_deinit();
#endif // SL_CATALOG_SENSOR_RHT_PRESENT
#ifdef SL_CATALOG_SENSOR_HALL_PRESENT
  sl_sensor_hall_deinit();
#endif // SL_CATALOG_SENSOR_HALL_PRESENT
#ifdef SL_CATALOG_SENSOR_LIGHT_PRESENT
  sl_sensor_light_deinit();
#endif // SL_CATALOG_SENSOR_LIGHT_PRESENT
#ifdef SL_CATALOG_SENSOR_IMU_PRESENT
  if (!sl_power_supply_is_low_power()) {
    sl_sensor_imu_enable(false);
    sl_sensor_imu_deinit();
  }
#endif // SL_CATALOG_SENSOR_IMU_PRESENT
#ifdef SL_CATALOG_SENSOR_PRESSURE_PRESENT
  sl_sensor_pressure_deinit();
#endif // SL_CATALOG_SENSOR_PRESSURE_PRESENT
#ifdef SL_CATALOG_SENSOR_GAS_PRESENT
  if (!sl_power_supply_is_low_power()) {
    sl_sensor_gas_deinit();
  }
#endif // SL_CATALOG_SENSOR_GAS_PRESENT
#ifdef SL_CATALOG_SENSOR_SOUND_PRESENT
  sl_sensor_sound_deinit();
#endif // SL_CATALOG_SENSOR_SOUND_PRESENT
}

/***************************************************************************//**
 * Read the value of battery, sensors and add to BTHome measurement
 ******************************************************************************/
void bthome_add_sensor_data_and_battery(void) {
  /////////////////////////////////////////////////////////////////////////////
  // Initialize the sensors, power up, in order to read the values           //
  // This will introduce latency if you plan to use many or specific sensors,//
  // usually is around 300ms if you init all the sensors on Thunderboard     //
  // and just a few or under ms for the temperature and humidity             //
  /////////////////////////////////////////////////////////////////////////////
  sl_status_t sc;
  char formatted_string[22];
  sensor_init();

#if defined(SL_CATALOG_SENSOR_RHT_PRESENT)
  uint32_t rhumidity;
  int32_t temperature;
  sc = sl_sensor_rht_get(&rhumidity, &temperature);
  if (SL_STATUS_OK == sc) {
    app_log_info("Humidity = %3.2f %%RH" APP_LOG_NL, (float)rhumidity / 1000.0f);
    app_log_info("Temperature = %3.2f C" APP_LOG_NL, (float)temperature / 1000.0f);
    bthome_v2_reset_measurement();
    bthome_v2_add_measurement_float(ID_TEMPERATURE_PRECISE, (float)temperature / 1000.0f);
    bthome_v2_add_measurement_float(ID_HUMIDITY, (float)rhumidity / 1000.0f);
    send_data_packet_to_bthome_v2();
  } else if (SL_STATUS_NOT_INITIALIZED == sc) {
    app_log_info("Relative Humidity and Temperature sensor is not initialized." APP_LOG_NL);
  } else {
    app_log_status_error_f(sc, "RHT sensor measurement failed" APP_LOG_NL);
  }
#endif //SL_CATALOG_SENSOR_RHT_PRESENT

#if defined(SL_CATALOG_SENSOR_HALL_PRESENT)
  float field_strength;
  bool alert;
  bool tamper;
  sc = sl_sensor_hall_get(&field_strength, &alert, &tamper);
  if (SL_STATUS_OK == sc) {
    app_log_info("Magnetic flux = %4.3f mT" APP_LOG_NL, field_strength);
    sprintf(formatted_string, "MF: %4.3f mT;", field_strength);
    send_text_to_bthome_v2(formatted_string);
  } else if (SL_STATUS_NOT_INITIALIZED == sc) {
    app_log_info("Hall sensor is not initialized." APP_LOG_NL);
  } else {
    app_log_status_error_f(sc, "Hall sensor measurement failed" APP_LOG_NL);
  }
#endif //SL_CATALOG_SENSOR_HALL_PRESENT

#if defined(SL_CATALOG_SENSOR_LIGHT_PRESENT)
  float lux;
  float uvi;
  sc = sl_sensor_light_get(&lux, &uvi);
  if (SL_STATUS_OK == sc) {
    app_log_info("Ambient light = %f lux" APP_LOG_NL, lux);
    app_log_info("UV Index = %u" APP_LOG_NL, (unsigned int)uvi);
    bthome_v2_reset_measurement();
    bthome_v2_add_measurement_float(ID_ILLUMINANCE, lux);
    bthome_v2_add_measurement_float(ID_UV, uvi);
    send_data_packet_to_bthome_v2();
  } else if (SL_STATUS_NOT_INITIALIZED == sc) {
    app_log_info("Ambient light and UV index sensor is not initialized." APP_LOG_NL);
  } else {
    app_log_status_error_f(sc, "Light sensor measurement failed" APP_LOG_NL);
  }
#endif //SL_CATALOG_SENSOR_LIGHT_PRESENT

#if defined(SL_CATALOG_SENSOR_IMU_PRESENT)
  int16_t ovec[3];
  int16_t avec[3];
  if (!sl_power_supply_is_low_power()) {
    sc = sl_sensor_imu_get(ovec, avec);
    if (SL_STATUS_OK == sc) {
        app_log_info("IMU: ORI : %04d,%04d,%04d" APP_LOG_NL, ovec[0], ovec[1], ovec[2]);
        app_log_info("IMU: ACC : %04d,%04d,%04d" APP_LOG_NL, avec[0], avec[1], avec[2]);
        sprintf(formatted_string, "O: %04d,%04d,%04d;", ovec[0], ovec[1], ovec[2]);
        send_text_to_bthome_v2(formatted_string);
        sprintf(formatted_string, "A: %04d,%04d,%04d;", avec[0], avec[1], avec[2]);
        send_text_to_bthome_v2(formatted_string);
    } else if (SL_STATUS_NOT_INITIALIZED == sc) {
        app_log_info("Inertial Measurement Unit sensor is not initialized." APP_LOG_NL);
    }
  } else {
      app_log_info("Inertial Measurement Unit doesn't work on battery power" APP_LOG_NL);
  }
#endif //SL_CATALOG_SENSOR_IMU_PRESENT

#if defined(SL_CATALOG_SENSOR_PRESSURE_PRESENT)
  float pressure;
  sc = sl_sensor_pressure_get(&pressure);
  if (SL_STATUS_OK == sc) {
    app_log_info("Pressure = %0.3f mbar" APP_LOG_NL, pressure);
    bthome_v2_reset_measurement();
    bthome_v2_add_measurement_float(ID_PRESSURE, pressure);
    send_data_packet_to_bthome_v2();
  } else if (SL_STATUS_NOT_INITIALIZED == sc) {
    app_log_info("Air pressure sensor is not initialized" APP_LOG_NL);
  } else {
    app_log_status_error_f(sc, "Pressure sensor measurement failed" APP_LOG_NL);
  }
#endif //SL_CATALOG_SENSOR_IMU_PRESENT

#if defined(SL_CATALOG_SENSOR_GAS_PRESENT)
  uint16_t eco2;
  uint16_t tvoc;
  if (!sl_power_supply_is_low_power()) {
    sc = sl_sensor_gas_get(&eco2, &tvoc);
    if (SL_STATUS_OK == sc) {
      app_log_info("eCO2 = %u ppm" APP_LOG_NL, (uint16_t)eco2);
      app_log_info("TVOC = %u ppd" APP_LOG_NL, (uint16_t)tvoc);
      bthome_v2_reset_measurement();
      bthome_v2_add_measurement(ID_CO2, eco2);
      bthome_v2_add_measurement(ID_TVOC, tvoc);
      send_data_packet_to_bthome_v2();
    } else if (SL_STATUS_NOT_INITIALIZED == sc) {
      app_log_info("Air quality sensor is not initialized." APP_LOG_NL);
    } else if (SL_STATUS_NOT_READY != sc) {
      app_log_status_error_f(sc, "Air quality sensor measurement failed" APP_LOG_NL);
    }
  } else {
      app_log_info("Gas monitor doesn't work on battery power" APP_LOG_NL);
  }
#endif //SL_CATALOG_SENSOR_GAS_PRESENT

#if defined(SL_CATALOG_SENSOR_SOUND_PRESENT)
#define MIC_SAMPLE_BUFFER_SIZE     1000
  float sound_level;
  static int16_t buffer[MIC_SAMPLE_BUFFER_SIZE];
  sl_mic_get_n_samples(buffer, MIC_SAMPLE_BUFFER_SIZE);
  while (!sl_mic_sample_buffer_ready()) {
  }
  sc = sl_mic_calculate_sound_level(&sound_level, buffer, MIC_SAMPLE_BUFFER_SIZE, 0);
  if (SL_STATUS_OK == sc) {
    app_log_info("Sound level = %3.2f dBA" APP_LOG_NL, sound_level);
    sprintf(formatted_string, "SL: %3.2f dBA;", sound_level);
    send_text_to_bthome_v2(formatted_string);
  } else if (SL_STATUS_NOT_INITIALIZED == sc) {
    app_log_info("Sound level sensor is not initialized." APP_LOG_NL);
  }else {
    app_log_status_error_f(sc, "Sound level measurement failed" APP_LOG_NL);
  }
#endif //SL_CATALOG_SENSOR_SOUND_PRESENT

  uint8_t bat_level;
  bat_level = sl_power_supply_get_battery_level();
  app_log_info("Battery level = %d %%" APP_LOG_NL, bat_level);
  bthome_v2_add_measurement_float(ID_BATTERY, (float)bat_level);
  send_data_packet_to_bthome_v2();

  // Deinitialize the sensors to save power
  sensor_deinit();
}

/**************************************************************************//**
 * Advertise timer callback function.
 *****************************************************************************/
static void adv_interval_timer_callback(sl_sleeptimer_timer_handle_t *timer,
                               void *data)
{
  (void) timer;
  (void) data;

  sl_bt_external_signal(ADV_INTERVAL_TIMER_EXT_SIGNAL);
}


/**************************************************************************//**
 * Deep sleep timer callback function.
 *****************************************************************************/
static void deep_sleep_timer_callback(sl_sleeptimer_timer_handle_t *timer,
                                      void *data)
{
  (void) timer;
  (void) data;

  sl_bt_external_signal(DEEP_SLEEP_TIMER_EXT_SIGNAL);
}

/**************************************************************************//**
 * Button none sleeptimer callback function.
 *****************************************************************************/
static void button_none_timer_callback(sl_sleeptimer_timer_handle_t *timer,
                                       void *data)
{
  (void) timer;
  (void) data;

  sl_bt_external_signal(BUTTON_NONE_EXT_SIG);
}

/**************************************************************************//**
 * Start or restart the button none event timer
 *****************************************************************************/
static void start_button_none_timer(void)
{
  bool is_timer_running;
  sl_sleeptimer_is_timer_running(&button_none_timer, &is_timer_running);
  if (is_timer_running) {
      sl_sleeptimer_stop_timer(&button_none_timer);
  }

  sl_sleeptimer_start_timer_ms(&button_none_timer,
                               BUTTON_NONE_TIMEOUT,
                               button_none_timer_callback,
                               NULL,
                               0,
                               0);
}

/**************************************************************************//**
 * Start or restart the deep sleep timer timeout
 *****************************************************************************/
static void start_deep_sleep_timer(void){
  bool is_timer_running;
  sl_sleeptimer_is_timer_running(&deep_sleep_timer, &is_timer_running);
  if (is_timer_running) {
      sl_sleeptimer_stop_timer(&deep_sleep_timer);
   }

  sl_sleeptimer_start_timer_ms(&deep_sleep_timer,
                               DSLEEP_TIMEOUT,
                               deep_sleep_timer_callback,
                               NULL,
                               0,
                               0);
}

/**************************************************************************//**
 * Send text data using BTHome V2 protocol
 *****************************************************************************/
static void send_text_to_bthome_v2(const char text[]){
  bthome_v2_reset_measurement();
  bthome_v2_add_text(text);
  bthome_v2_send_packet();
  // Add an delay to let advertising data change, default 100ms
  sl_sleeptimer_delay_millisecond(BLE_ADVERTISE_TIME_INTERVAL_MS * 2);
}

static void send_data_packet_to_bthome_v2(void){
  bthome_v2_send_packet();
  sl_sleeptimer_delay_millisecond(BLE_ADVERTISE_TIME_INTERVAL_MS * 2);
}