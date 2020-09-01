/**
 ******************************************************************************
 * @file    usbd_hid.c
 * @author  MCD Application Team
 * @brief   This file provides the HID core functions.
 *
 * @verbatim
 *
 *          ===================================================================
 *                                HID Class  Description
 *          ===================================================================
 *           This module manages the HID class V1.11 following the "Device Class
 *Definition for Human Interface Devices (HID) Version 1.11 Jun 27, 2001". This driver
 *implements the following aspects of the specification:
 *             - The Boot Interface Subclass
 *             - The Mouse protocol
 *             - Usage Page : Generic Desktop
 *             - Usage : Joystick
 *             - Collection : Application
 *
 * @note     In HS mode and when the DMA is used, all variables and data structures
 *           dealing with the DMA during the transaction process should be 32-bit aligned.
 *
 *
 *  @endverbatim
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                      www.st.com/SLA0044
 *
 ******************************************************************************
 */

/* BSPDependencies
- "stm32xxxxx_{eval}{discovery}{nucleo_144}.c"
- "stm32xxxxx_{eval}{discovery}_io.c"
EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbd_hid.h"
#include "usbd_ctlreq.h"

#include "hid_def.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
 * @{
 */

/** @defgroup USBD_HID
 * @brief usbd core module
 * @{
 */

/** @defgroup USBD_HID_Private_TypesDefinitions
 * @{
 */
/**
 * @}
 */

/** @defgroup USBD_HID_Private_Defines
 * @{
 */

/**
 * @}
 */

/** @defgroup USBD_HID_Private_Macros
 * @{
 */
/**
 * @}
 */

/** @defgroup USBD_HID_Private_FunctionPrototypes
 * @{
 */

static uint8_t USBD_HID_Init(USBD_HandleTypeDef* pdev, uint8_t cfgidx);

static uint8_t USBD_HID_DeInit(USBD_HandleTypeDef* pdev, uint8_t cfgidx);

static uint8_t USBD_HID_Setup(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req);

static uint8_t* USBD_HID_GetFSCfgDesc(uint16_t* length);

static uint8_t* USBD_HID_GetHSCfgDesc(uint16_t* length);

static uint8_t* USBD_HID_GetOtherSpeedCfgDesc(uint16_t* length);

static uint8_t* USBD_HID_GetDeviceQualifierDesc(uint16_t* length);

static uint8_t USBD_HID_DataIn(USBD_HandleTypeDef* pdev, uint8_t epnum);
/**
 * @}
 */

/** @defgroup USBD_HID_Private_Variables
 * @{
 */
USBD_ClassTypeDef USBD_HID = {
    USBD_HID_Init,
    USBD_HID_DeInit,
    USBD_HID_Setup,
    NULL,            /*EP0_TxSent*/
    NULL,            /*EP0_RxReady*/
    USBD_HID_DataIn, /*DataIn*/
    NULL,            /*DataOut*/
    NULL,            /*SOF */
    NULL,
    NULL,
    USBD_HID_GetHSCfgDesc,
    USBD_HID_GetFSCfgDesc,
    USBD_HID_GetOtherSpeedCfgDesc,
    USBD_HID_GetDeviceQualifierDesc,
};

/**
 * \brief USB HID device FS configuration descriptor
 */
__ALIGN_BEGIN static uint8_t USBD_HID_CfgFSDesc[USB_HID_CONFIG_DESC_SIZ] __ALIGN_END = {
    0x09,                           // bLength: Size of descriptor in bytes
    USB_DESC_TYPE_CONFIGURATION,    // bDescriptorType: Configuration Descriptor (0x02)
    USB_HID_CONFIG_DESC_SIZ, 0x00,  // wTotalLength: Total length in bytes of data returned
    0x01,                           // bNumInterfaces: Number of interfaces
    0x01,        // bConfigurationValue: Value to use as an argument to select this configuration
    0x00,        // iConfiguration: Index of String Descriptor describing this configuration
    0b11100000,  // bmAttributes: Self powered, remote wakeup
    0x32,        // bMaxPower: 100mA = 0x32*2mA

    /***************************** Descriptor of joystick interface *****************************/
    0x09,                     // bLength: Size of descriptor in bytes (9 bytes)
    USB_DESC_TYPE_INTERFACE,  // bDescriptorType: Interface Descriptor (0x04)
    0x00,                     // bInterfaceNumber: Number of interface
    0x00,                     // bAlternateSetting: Value used to select alternative setting
    0x01,                     // bNumEndpoints: Number of Endpoints used for this interface
    0x03,                     // bInterfaceClass: Class Code (HID = 0x03)
    0x01,                     // bInterfaceSubClass: Subclass Code (Boot interface = 0x01)
    0x00,                     // bInterfaceProtocol: Protocol Code (None = 0x00)
    0x00,                     // iInterface: Index of String Descriptor Describing this interface

    /***************************** Descriptor of joystick HID *****************************/
    0x09,                 // bLength: Size of descriptor in bytes (9 bytes)
    HID_DESCRIPTOR_TYPE,  // bDescriptorType: HID = 0x21
    0x11, 0x01,           // bcdHID: HID class specification release number 0x0111
    0x00,                 // bCountryCode: Hardware target country
    0x01,                 // bNumDescriptors: Number of descriptors
    HID_REPORT_DESC,      // bDescriptorType: Descriptor type
    HID_HEADTRACKER_REPORT_DESC_SIZE, 0x00,  // wDescriptorLength: Descriptor length

    /***************************** Descriptor of joystick endpoint *****************************/
    0x07,                    // bLength: Size of descriptor in bytes (7 bytes)
    USB_DESC_TYPE_ENDPOINT,  // bDescriptorType: Endpoint Descriptor (0x05)
    HID_EPIN_ADDR,           // bEndpointAddress: Endpoint Address
    0x03,                    // bmAttributes: Interrupt endpoint
    HID_EPIN_SIZE, 0x00,     // wMaxPacketSize: Maximum packet size (4 bytes)
    HID_FS_BINTERVAL,        // bInterval: Polling interval
};

/* USB HID device HS Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_CfgHSDesc[USB_HID_CONFIG_DESC_SIZ] __ALIGN_END = {
    0x09,                           // bLength: Size of descriptor in bytes
    USB_DESC_TYPE_CONFIGURATION,    // bDescriptorType: Configuration Descriptor (0x02)
    USB_HID_CONFIG_DESC_SIZ, 0x00,  // wTotalLength: Total length in bytes of data returned
    0x01,                           // bNumInterfaces: Number of interfaces
    0x01,        // bConfigurationValue: Value to use as an argument to select this configuration
    0x00,        // iConfiguration: Index of String Descriptor describing this configuration
    0b11100000,  // bmAttributes: Self powered, remote wakeup
    0x32,        // bMaxPower: 100mA = 0x32*2mA

    /***************************** Descriptor of joystick interface *****************************/
    0x09,                     // bLength: Size of descriptor in bytes (9 bytes)
    USB_DESC_TYPE_INTERFACE,  // bDescriptorType: Interface Descriptor (0x04)
    0x00,                     // bInterfaceNumber: Number of interface
    0x00,                     // bAlternateSetting: Value used to select alternative setting
    0x01,                     // bNumEndpoints: Number of Endpoints used for this interface
    0x03,                     // bInterfaceClass: Class Code (HID = 0x03)
    0x01,                     // bInterfaceSubClass: Subclass Code (Boot interface = 0x01)
    0x00,                     // bInterfaceProtocol: Protocol Code (None = 0x00)
    0x00,                     // iInterface: Index of String Descriptor Describing this interface

    /***************************** Descriptor of joystick HID *****************************/
    0x09,                 // bLength: Size of descriptor in bytes (9 bytes)
    HID_DESCRIPTOR_TYPE,  // bDescriptorType: HID = 0x21
    0x11, 0x01,           // bcdHID: HID class specification release number 0x0111
    0x00,                 // bCountryCode: Hardware target country
    0x01,                 // bNumDescriptors: Number of descriptors
    HID_REPORT_DESC,      // bDescriptorType: Descriptor type
    HID_HEADTRACKER_REPORT_DESC_SIZE, 0x00,  // wDescriptorLength: Descriptor length

    /***************************** Descriptor of joystick endpoint *****************************/
    0x07,                    // bLength: Size of descriptor in bytes (7 bytes)
    USB_DESC_TYPE_ENDPOINT,  // bDescriptorType: Endpoint Descriptor (0x05)
    HID_EPIN_ADDR,           // bEndpointAddress: Endpoint Address
    0x03,                    // bmAttributes: Interrupt endpoint
    HID_EPIN_SIZE, 0x00,     // wMaxPacketSize: Maximum packet size (4 bytes)
    HID_HS_BINTERVAL,        // bInterval: Polling interval
};

/* USB HID device Other Speed Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_OtherSpeedCfgDesc[USB_HID_CONFIG_DESC_SIZ] __ALIGN_END = {
    0x09,                           // bLength: Size of descriptor in bytes
    USB_DESC_TYPE_CONFIGURATION,    // bDescriptorType: Configuration Descriptor (0x02)
    USB_HID_CONFIG_DESC_SIZ, 0x00,  // wTotalLength: Total length in bytes of data returned
    0x01,                           // bNumInterfaces: Number of interfaces
    0x01,        // bConfigurationValue: Value to use as an argument to select this configuration
    0x00,        // iConfiguration: Index of String Descriptor describing this configuration
    0b11100000,  // bmAttributes: Self powered, remote wakeup
    0x32,        // bMaxPower: 100mA = 0x32*2mA

    /***************************** Descriptor of joystick interface *****************************/
    0x09,                     // bLength: Size of descriptor in bytes (9 bytes)
    USB_DESC_TYPE_INTERFACE,  // bDescriptorType: Interface Descriptor (0x04)
    0x00,                     // bInterfaceNumber: Number of interface
    0x00,                     // bAlternateSetting: Value used to select alternative setting
    0x01,                     // bNumEndpoints: Number of Endpoints used for this interface
    0x03,                     // bInterfaceClass: Class Code (HID = 0x03)
    0x01,                     // bInterfaceSubClass: Subclass Code (Boot interface = 0x01)
    0x00,                     // bInterfaceProtocol: Protocol Code (None = 0x00)
    0x00,                     // iInterface: Index of String Descriptor Describing this interface

    /***************************** Descriptor of joystick HID *****************************/
    0x09,                 // bLength: Size of descriptor in bytes (9 bytes)
    HID_DESCRIPTOR_TYPE,  // bDescriptorType: HID = 0x21
    0x11, 0x01,           // bcdHID: HID class specification release number 0x0111
    0x00,                 // bCountryCode: Hardware target country
    0x01,                 // bNumDescriptors: Number of descriptors
    HID_REPORT_DESC,      // bDescriptorType: Descriptor type
    HID_HEADTRACKER_REPORT_DESC_SIZE, 0x00,  // wDescriptorLength: Descriptor length

    /***************************** Descriptor of joystick endpoint *****************************/
    0x07,                    // bLength: Size of descriptor in bytes (7 bytes)
    USB_DESC_TYPE_ENDPOINT,  // bDescriptorType: Endpoint Descriptor (0x05)
    HID_EPIN_ADDR,           // bEndpointAddress: Endpoint Address
    0x03,                    // bmAttributes: Interrupt endpoint
    HID_EPIN_SIZE, 0x00,     // wMaxPacketSize: Maximum packet size (4 bytes)
    HID_HS_BINTERVAL,        // bInterval: Polling interval
};

/* USB HID device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_Desc[USB_HID_DESC_SIZ] __ALIGN_END = {
    0x09,                 // bLength: Size of descriptor in bytes (9 bytes)
    HID_DESCRIPTOR_TYPE,  // bDescriptorType: HID = 0x21
    0x11,
    0x01,             // bcdHID: HID class specification release number 0x0111
    0x00,             // bCountryCode: Hardware target country
    0x01,             // bNumDescriptors: Number of descriptors
    HID_REPORT_DESC,  // bDescriptorType: Descriptor type
    HID_HEADTRACKER_REPORT_DESC_SIZE,
    0x00,  // wDescriptorLength: Descriptor length
};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
    {
        USB_LEN_DEV_QUALIFIER_DESC,
        USB_DESC_TYPE_DEVICE_QUALIFIER,
        0x00,
        0x02,
        0x00,
        0x00,
        0x00,
        0x40,
        0x01,
        0x00,
};

// clang-format off
__ALIGN_BEGIN static uint8_t HID_HeadTracker_ReportDesc[HID_HEADTRACKER_REPORT_DESC_SIZE]  __ALIGN_END =
{
  HID_USAGE_PAGE (GENERIC_DESKTOP), // HID_USAGE_PAGE (VIRTUAL_REALITY),
  HID_USAGE (JOYSTICK), // HID_USAGE (HEAD_TRACKER),
  HID_COLLECTION (APPLICATION),
    HID_USAGE_PAGE (GENERIC_DESKTOP),
    HID_USAGE (HEAD_TRACKER),
    HID_COLLECTION (PHYSICAL),
      HID_USAGE_PAGE (GENERIC_DESKTOP),
      HID_USAGE (X),
      HID_USAGE (Y),
      HID_USAGE (Z),
      HID_LOGICAL_MINIMUM (2, -1000),
      HID_LOGICAL_MAXIMUM (2, 1000),
      HID_REPORT_SIZE (16),
      HID_REPORT_COUNT (3),
      HID_INPUT (DATA, VARIABLE, ABSOLUTE),
    HID_END_COLLECTION (PHYSICAL),
    HID_USAGE_PAGE (GENERIC_DESKTOP),
    HID_USAGE (HEAD_TRACKER),
    HID_COLLECTION (PHYSICAL),
      HID_USAGE_PAGE (GENERIC_DESKTOP),
      HID_USAGE (RX),
      HID_USAGE (RY),
      HID_USAGE (RZ),
      HID_LOGICAL_MINIMUM (2, -18000),
      HID_LOGICAL_MAXIMUM (2, 18000),
      HID_REPORT_SIZE (16),
      HID_REPORT_COUNT (3),
      HID_INPUT (DATA, VARIABLE, ABSOLUTE),
    HID_END_COLLECTION (PHYSICAL),
  HID_END_COLLECTION (APPLICATION),
};
// clang-format on

/**
 * @}
 */

/** @defgroup USBD_HID_Private_Functions
 * @{
 */

/**
 * @brief  USBD_HID_Init
 *         Initialize the HID interface
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t USBD_HID_Init(USBD_HandleTypeDef* pdev, uint8_t cfgidx)
{
  /* Open EP IN */
  USBD_LL_OpenEP(pdev, HID_EPIN_ADDR, USBD_EP_TYPE_INTR, HID_EPIN_SIZE);
  pdev->ep_in[HID_EPIN_ADDR & 0xFU].is_used = 1U;

  pdev->pClassData = USBD_malloc(sizeof(USBD_HID_HandleTypeDef));

  if (pdev->pClassData == NULL) {
    return USBD_FAIL;
  }

  ((USBD_HID_HandleTypeDef*)pdev->pClassData)->state = HID_IDLE;

  return USBD_OK;
}

/**
 * @brief  USBD_HID_Init
 *         DeInitialize the HID layer
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t USBD_HID_DeInit(USBD_HandleTypeDef* pdev, uint8_t cfgidx)
{
  /* Close HID EPs */
  USBD_LL_CloseEP(pdev, HID_EPIN_ADDR);
  pdev->ep_in[HID_EPIN_ADDR & 0xFU].is_used = 0U;

  /* FRee allocated memory */
  if (pdev->pClassData != NULL) {
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }

  return USBD_OK;
}

/**
 * @brief  USBD_HID_Setup
 *         Handle the HID specific requests
 * @param  pdev: instance
 * @param  req: usb requests
 * @retval status
 */
static uint8_t USBD_HID_Setup(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req)
{
  USBD_HID_HandleTypeDef* hhid        = (USBD_HID_HandleTypeDef*)pdev->pClassData;
  uint16_t                len         = 0U;
  uint8_t*                pbuf        = NULL;
  uint16_t                status_info = 0U;
  USBD_StatusTypeDef      ret         = USBD_OK;

  switch (req->bmRequest & USB_REQ_TYPE_MASK) {
    case USB_REQ_TYPE_CLASS:
      switch (req->bRequest) {
        case HID_REQ_SET_PROTOCOL:
          hhid->Protocol = (uint8_t)(req->wValue);
          break;

        case HID_REQ_GET_PROTOCOL:
          USBD_CtlSendData(pdev, (uint8_t*)(void*)&hhid->Protocol, 1U);
          break;

        case HID_REQ_SET_IDLE:
          hhid->IdleState = (uint8_t)(req->wValue >> 8);
          break;

        case HID_REQ_GET_IDLE:
          USBD_CtlSendData(pdev, (uint8_t*)(void*)&hhid->IdleState, 1U);
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;
    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest) {
        case USB_REQ_GET_STATUS:
          if (pdev->dev_state == USBD_STATE_CONFIGURED) {
            USBD_CtlSendData(pdev, (uint8_t*)(void*)&status_info, 2U);
          } else {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_GET_DESCRIPTOR:
          if (req->wValue >> 8 == HID_REPORT_DESC) {
            len  = MIN(HID_HEADTRACKER_REPORT_DESC_SIZE, req->wLength);
            pbuf = HID_HeadTracker_ReportDesc;
          } else if (req->wValue >> 8 == HID_DESCRIPTOR_TYPE) {
            pbuf = USBD_HID_Desc;
            len  = MIN(USB_HID_DESC_SIZ, req->wLength);
          } else {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
            break;
          }
          USBD_CtlSendData(pdev, pbuf, len);
          break;

        case USB_REQ_GET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED) {
            USBD_CtlSendData(pdev, (uint8_t*)(void*)&hhid->AltSetting, 1U);
          } else {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_SET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED) {
            hhid->AltSetting = (uint8_t)(req->wValue);
          } else {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
  }

  return ret;
}

/**
 * @brief  USBD_HID_SendReport
 *         Send HID Report
 * @param  pdev: device instance
 * @param  buff: pointer to report
 * @retval status
 */
uint8_t USBD_HID_SendReport(USBD_HandleTypeDef* pdev, uint8_t* report, uint16_t len)
{
  USBD_HID_HandleTypeDef* hhid = (USBD_HID_HandleTypeDef*)pdev->pClassData;

  if (pdev->dev_state == USBD_STATE_CONFIGURED) {
    if (hhid->state == HID_IDLE) {
      hhid->state = HID_BUSY;
      USBD_LL_Transmit(pdev, HID_EPIN_ADDR, report, len);
    }
  }
  return USBD_OK;
}

/**
 * @brief  USBD_HID_GetPollingInterval
 *         return polling interval from endpoint descriptor
 * @param  pdev: device instance
 * @retval polling interval
 */
uint32_t USBD_HID_GetPollingInterval(USBD_HandleTypeDef* pdev)
{
  uint32_t polling_interval = 0U;

  /* HIGH-speed endpoints */
  if (pdev->dev_speed == USBD_SPEED_HIGH) {
    /* Sets the data transfer polling interval for high speed transfers.
     Values between 1..16 are allowed. Values correspond to interval
     of 2 ^ (bInterval-1). This option (8 ms, corresponds to HID_HS_BINTERVAL */
    polling_interval = (((1U << (HID_HS_BINTERVAL - 1U))) / 8U);
  } else /* LOW and FULL-speed endpoints */
  {
    /* Sets the data transfer polling interval for low and full
    speed transfers */
    polling_interval = HID_FS_BINTERVAL;
  }

  return ((uint32_t)(polling_interval));
}

/**
 * @brief  USBD_HID_GetCfgFSDesc
 *         return FS configuration descriptor
 * @param  speed : current device speed
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t* USBD_HID_GetFSCfgDesc(uint16_t* length)
{
  *length = sizeof(USBD_HID_CfgFSDesc);
  return USBD_HID_CfgFSDesc;
}

/**
 * @brief  USBD_HID_GetCfgHSDesc
 *         return HS configuration descriptor
 * @param  speed : current device speed
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t* USBD_HID_GetHSCfgDesc(uint16_t* length)
{
  *length = sizeof(USBD_HID_CfgHSDesc);
  return USBD_HID_CfgHSDesc;
}

/**
 * @brief  USBD_HID_GetOtherSpeedCfgDesc
 *         return other speed configuration descriptor
 * @param  speed : current device speed
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t* USBD_HID_GetOtherSpeedCfgDesc(uint16_t* length)
{
  *length = sizeof(USBD_HID_OtherSpeedCfgDesc);
  return USBD_HID_OtherSpeedCfgDesc;
}

/**
 * @brief  USBD_HID_DataIn
 *         handle data IN Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t USBD_HID_DataIn(USBD_HandleTypeDef* pdev, uint8_t epnum)
{
  /* Ensure that the FIFO is empty before a new transfer, this condition could
  be caused by  a new transfer before the end of the previous transfer */
  ((USBD_HID_HandleTypeDef*)pdev->pClassData)->state = HID_IDLE;
  return USBD_OK;
}

/**
 * @brief  DeviceQualifierDescriptor
 *         return Device Qualifier descriptor
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t* USBD_HID_GetDeviceQualifierDesc(uint16_t* length)
{
  *length = sizeof(USBD_HID_DeviceQualifierDesc);
  return USBD_HID_DeviceQualifierDesc;
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
