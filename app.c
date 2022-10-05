/***************************************************************************//**
 * @file app.c
 * @brief Silicon Labs Empty Example Project
 *
 * This example demonstrates the bare minimum needed for a Blue Gecko C application
 * that allows Over-the-Air Device Firmware Upgrading (OTA DFU). The application
 * starts advertising after boot and restarts advertising after a connection is closed.
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
#include <stdbool.h>
#include "em_common.h"
#include <strings.h>
#include <stdio.h>
#include "app_assert.h"
#include "SEGGER.h"
#include "SEGGER_RTT.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "sl_simple_timer.h"

// Clear the Terminal 0 screen in RTT viewer
#define CLR_SCR "\033[2J\033[;H"


#ifdef SL_COMPONENT_CATALOG_PRESENT
#include "sl_component_catalog.h"
#endif // SL_COMPONENT_CATALOG_PRESENT
#ifdef SL_CATALOG_CLI_PRESENT
#include "sl_cli.h"
#endif // SL_CATALOG_CLI_PRESENT
#include "app.h"

#include "sl_simple_button.h"
#include "sl_simple_button_instances.h"

#include "btl_interface.h"
#include "btl_interface_storage.h"

// The advertising set handle allocated from Bluetooth stack
static uint8_t advertising_set_handle = 0xff;

/* Flag for indicating DFU Reset must be performed */


// Periodic timer handle.
static sl_simple_timer_t app_periodic_timer;

// Periodic timer callback.
static void app_periodic_timer_cb(sl_simple_timer_t *timer, void *data);


//Bootloader Interface
static BootloaderInformation_t bldInfo;
static BootloaderStorageSlot_t slotInfo;

/* OTA variables */
static uint32_t ota_image_position = 0;
static uint8_t ota_in_progress = 0;
static uint8_t ota_image_finished = 0;
static uint16_t ota_time_elapsed = 0;

void sl_button_on_change(const sl_button_t *handle)
{
  // Button pressed.
  if (sl_button_get_state(handle) == SL_SIMPLE_BUTTON_PRESSED) {
    if (&sl_button_btn0 == handle) {
        SEGGER_RTT_printf(0,"\r\nbtn0 pressed\r\n");
     sl_bt_external_signal(1);
    }
  }
}

static int32_t get_slot_info(uint8_t slot)
{
int32_t err;
bootloader_getInfo(&bldInfo);
SEGGER_RTT_printf(0,"\r\nGecko bootloader version: %u.%u\r\n", (bldInfo.version & 0xFF000000) >> 24, (bldInfo.version & 0x00FF0000) >> 16);

err = bootloader_getStorageSlotInfo(slot, &slotInfo);

    if(err == BOOTLOADER_OK)
    {
    SEGGER_RTT_printf(0,"\r\nSlot %u starts @ 0x%8.8x, size %u bytes\r\n", slot, slotInfo.address, slotInfo.length);
    }
    else
    {
    SEGGER_RTT_printf(0,"\r\nUnable to get storage slot info, error %x\r\n", err);
    }

    return(err);
}

static int32_t verify_application()
{
int32_t err;
err = bootloader_verifyImage(0, NULL);

  if(err != BOOTLOADER_OK)
  {
  SEGGER_RTT_printf(0,"\r\nApplication Verification Failed. err: %u \r\n", err);
  }
  else
  {
  SEGGER_RTT_printf(0,"\r\nApplication Verified \r\n");
  }

  return err;
}

static void erase_slot_if_needed(uint8_t slot)
{
uint32_t offset = 0, num_blocks = 0, i = 0;
uint8_t buffer[256];
bool dirty = false;
int32_t err = BOOTLOADER_OK;

/* check the download area content by reading it in 256-byte blocks */
num_blocks = slotInfo.length / 256;

  while((dirty == 0) && (offset < 256*num_blocks) && (err == BOOTLOADER_OK))
  {
  err = bootloader_readStorage(slot, offset, buffer, 256);
    if(err == BOOTLOADER_OK)
    {
    i = 0;
      while(i < 256)
      {
        if(buffer[i++] != 0xFF)
        {
        dirty = true;
          break;
        }
      }
    offset += 256;
    }
  }

  if(err != BOOTLOADER_OK)
  {
  SEGGER_RTT_printf(0,"\r\nerror reading flash! %x\r\n", err);
  }
  else if(dirty)
  {
  SEGGER_RTT_printf(0,"\r\ndownload slot is not empty, erasing...");
  bootloader_eraseStorageSlot(0);
  SEGGER_RTT_printf(0,"  done\r\n");
  }
  else
  {
  SEGGER_RTT_printf(0,"\r\nDownload slot %u is empty\r\n", slot);
  }
}

static void print_progress()
{
// estimate transfer speed in kbps
int kbps = ota_image_position*8/(1024*ota_time_elapsed);
SEGGER_RTT_printf(0,"pos: %u, time: %u, kbps: %u\r\n", ota_image_position, ota_time_elapsed, kbps);
}

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{

  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
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
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t* evt)
{
bd_addr address;
uint8_t address_type;
int32_t err;

  // Handle stack events
  switch (SL_BT_MSG_ID(evt->header))
  {
  case sl_bt_evt_system_boot_id:
  printf("\r\nAT+CFUN=0\r\n");

  SEGGER_RTT_WriteString(0, CLR_SCR);
  // Print boot message.
  SEGGER_RTT_printf(0,"\r\nBluetooth stack booted: v%d.%d.%d-b%d\r\n",
          evt->data.evt_system_boot.major,
          evt->data.evt_system_boot.minor,
          evt->data.evt_system_boot.patch,
          evt->data.evt_system_boot.build);

  sl_bt_system_get_identity_address(&address, &address_type);
  SEGGER_RTT_printf(0,"\r\nBluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
    address_type ? "static random" : "public device",
    address.addr[5],
    address.addr[4],
    address.addr[3],
    address.addr[2],
    address.addr[1],
    address.addr[0]);
  // 1 second timer, used for performance statistics during OTA file upload
  sl_simple_timer_start(&app_periodic_timer, 1000, app_periodic_timer_cb, NULL, true);

  // Create an advertising set.
  sl_bt_advertiser_create_set  (&advertising_set_handle);
  sl_bt_advertiser_set_timing  (advertising_set_handle, 1600, 1600, 0, 0);
  sl_bt_legacy_advertiser_start(advertising_set_handle, sl_bt_legacy_advertiser_connectable);

  SEGGER_RTT_printf(0,"\r\nBoot event - started advertising\r\nInitialize bootloader\r\n");
  bootloader_init();

    // read slot information from bootloader
    if(get_slot_info(0) == BOOTLOADER_OK)
    {
    SEGGER_RTT_printf(0,"\r\nBootloader OK");
    // the download area is erased here (if needed), prior to any connections are opened
    erase_slot_if_needed(0);
    SEGGER_RTT_printf(0,"\r\nSlot erased");
    }
    else
    {
    SEGGER_RTT_printf(0,"\r\nCheck that you have installed correct type of Gecko bootloader!\r\n");
    }
  err = bootloader_verifyImage(0, NULL);
    if(err != BOOTLOADER_OK)
    {
    SEGGER_RTT_printf(0,"\r\nSlot 0 Application Verification Failed. err: %u \r\n", err);
    }
    else
    {
    SEGGER_RTT_printf(0,"\r\nSlot 0 Application Verified \r\n");
    }

  printf("\r\nAT+CFUN=1\r\n");
  break;

  ///////////////////////////////////////////////////////////////////////////
  // This event indicates that a new connection was opened.                //
  ///////////////////////////////////////////////////////////////////////////
  case sl_bt_evt_connection_opened_id:
  SEGGER_RTT_printf(0,"\r\nConnection opened\r\n");
  break;

  ///////////////////////////////////////////////////////////////////////////
  // This event indicates that a connection was closed.                    //
  ///////////////////////////////////////////////////////////////////////////
  case sl_bt_evt_connection_closed_id:
  SEGGER_RTT_printf(0,"\r\nConnection closed, reason: 0x%2.2x\r\n", evt->data.evt_connection_closed.reason);

    if ( ota_image_finished )
    {
    SEGGER_RTT_printf(0,"\r\nVerifying new image\r\n");
    int32_t verified = verify_application();

      if(verified == 0)
      {
      SEGGER_RTT_printf(0,"\r\nInstalling new image\r\n");
      bootloader_setImageToBootload(0);
      bootloader_rebootAndInstall();
      }
      else
      {
      SEGGER_RTT_printf(0,"\r\nApplication not installed\r\n");
      erase_slot_if_needed(0);
      ota_time_elapsed = 0;
      // Restart advertising after client has disconnected.
      sl_bt_legacy_advertiser_start( advertising_set_handle, sl_bt_legacy_advertiser_connectable);
      SEGGER_RTT_printf(0,"\r\nStarted advertising\n");
      }
    }
    else
    {
    // Restart advertising after client has disconnected.
    sl_bt_legacy_advertiser_start( advertising_set_handle, sl_bt_legacy_advertiser_connectable);
    SEGGER_RTT_printf(0,"\r\nStarted advertising\n");
    }
  break;

  ///////////////////////////////////////////////////////////////////////////
  // Add additional event handlers here as your application requires!      //
  ///////////////////////////////////////////////////////////////////////////
  case sl_bt_evt_gatt_server_user_write_request_id:
  {
  uint32_t connection = evt->data.evt_gatt_server_user_write_request.connection;
  uint32_t characteristic = evt->data.evt_gatt_server_user_write_request.characteristic;
    if(characteristic == gattdb_ota_control)
    {
      switch(evt->data.evt_gatt_server_user_write_request.value.data[0])
      {
      case 0:
      //Erase and use slot 0
      // NOTE: download area is NOT erased here, because the long blocking delay would result in supervision timeout
      bootloader_eraseStorageSlot(0);
      ota_image_position=0;
      ota_in_progress=1;
      break;

      case 3://END OTA process
      //wait for connection close and then reboot
      ota_in_progress=0;
      ota_image_finished=1;
      SEGGER_RTT_printf(0,"\r\nUpload finished. received file size %u bytes\r\n", ota_image_position);
      break;

      default:
      break;
      }
    }else if(characteristic == gattdb_ota_data)
    {
      if(ota_in_progress)
      {
      bootloader_writeStorage(0, ota_image_position, evt->data.evt_gatt_server_user_write_request.value.data, evt->data.evt_gatt_server_user_write_request.value.len);
      ota_image_position+=evt->data.evt_gatt_server_user_write_request.value.len;
      }
    }
  sl_bt_gatt_server_send_user_write_response(connection,characteristic,0);
  }
  break;

/*
  ///////////////////////////////////////////////////////////////////////////
  // External Signal ID (push button to load image)                        //
  ///////////////////////////////////////////////////////////////////////////
  case sl_bt_evt_system_external_signal_id:
  SEGGER_RTT_printf(0,"Install slot 0 firmware\r\n");
  bootloader_setImageToBootload(0);
  bootloader_rebootAndInstall();
  break;

  ///////////////////////////////////////////////////////////////////////////
  // Default event handler.                                                //
  ///////////////////////////////////////////////////////////////////////////
  default:
  break;
*/
  }
}

static void app_periodic_timer_cb(sl_simple_timer_t *timer, void *data)
{
(void)data;
(void)timer;
  if(ota_in_progress)
  {
  ota_time_elapsed++;
  print_progress();
  }
}

