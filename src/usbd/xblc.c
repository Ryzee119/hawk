// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2024 Ryan Wendland

#include "usb_device.h"
#include "usbd_core.h"
#include "xblc.h"

extern void Error_Handler(void);

#define USBD_VID 0x045e
#define USBD_PID 0x0283

void USBD_XBLC_Sample_Rate_Changed(USBD_HandleTypeDef *pdev, uint32_t sample_rate);
void USBD_XBLC_AGC_Changed(USBD_HandleTypeDef *pdev, uint8_t value);
void USBD_XBLC_Data_Received(USBD_HandleTypeDef *pdev, uint8_t *data, uint16_t size);
void USBD_XBLC_Get_Data(USBD_HandleTypeDef *pdev, uint8_t *data, uint16_t size);

typedef struct {
    uint8_t current_packet_size;
    uint32_t host_in_sample_count;
    uint8_t sample_rate;
    uint8_t agc;
    uint8_t host_in_buffer[USBD_XBLC_MAX_PACKET];
    uint8_t host_out_buffer[USBD_XBLC_MAX_PACKET];
} xblc_data_t;
xblc_data_t gxblc_data;

USBD_HandleTypeDef hUsbDeviceFS;

// clang-format off
__ALIGN_BEGIN uint8_t USBD_XBLC_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END = {
    0x12,
    USB_DESC_TYPE_DEVICE,
    0x00,
    0x01,
    0x00,
    0x00,
    0x00,
    USB_MAX_EP0_SIZE,
    LOBYTE(USBD_VID),
    HIBYTE(USBD_VID),
    LOBYTE(USBD_PID),
    HIBYTE(USBD_PID),
    0x00,
    0x01,
    0,
    0,
    0, 
    1
};

__ALIGN_BEGIN static uint8_t USBD_XBLC_CfgDesc[45] __ALIGN_END = {
    0x09,
    USB_DESC_TYPE_CONFIGURATION, 
    LOBYTE(45),
    HIBYTE(45),
    0x02, 
    0x01,  
    0x00,   
    0x80,    
    USBD_MAX_POWER,

    0x09,
    USB_DESC_TYPE_INTERFACE,
    0x00,
    0x00,
    0x01,
    0x78,
    0x00,
    0x00,
    0x00,

    0x09,
    USB_DESC_TYPE_ENDPOINT,
    USBD_XBLC_OUT_EP,
    0x05,
    LOBYTE(USBD_XBLC_MAX_PACKET),
    HIBYTE(USBD_XBLC_MAX_PACKET),
    0x01,
    0x00,
    0x00,

    0x09,
    USB_DESC_TYPE_INTERFACE,
    0x01,
    0x00,
    0x01,
    0x78,
    0x00,
    0x00,
    0x00,

    0x09,
    USB_DESC_TYPE_ENDPOINT,
    USBD_XBLC_IN_EP,
    0x05,
    LOBYTE(USBD_XBLC_MAX_PACKET),
    HIBYTE(USBD_XBLC_MAX_PACKET),
    0x01,
    0x00,
    0x00
};
// clang-format on

typedef struct {
    uint16_t sample_rate;
    uint16_t bytes_per_frame;
    uint8_t frames_per_extra_sample;  // This is needed when the sample rate does not divide into an integer value.
                                      // i.e for 11025, every 40 frames, send another sample. 1000 / (11025 % 1000) = 40
} xblc_sample_rates_t;

// clang-format off
typedef enum
{
    XBLC_RATE_8000  = 0,
    XBLC_RATE_11025 = 1,
    XBLC_RATE_16000 = 2,
    XBLC_RATE_22050 = 3,
    XBLC_RATE_24000 = 4,
    XBLC_RATE_MAX_NUM = 5
} xblc_samplerate_enum;

static const xblc_sample_rates_t sample_rates[5] = {
    {8000, (8000 * 2 / 1000), 0},
    {11025, (11025 * 2 / 1000), 40},
    {16000, (16000 * 2 / 1000), 0},
    {22050, (22050 * 2 / 1000), 20},
    {24000, (24000 * 2 / 1000), 0},
};
// clang-format on

static uint8_t *USBD_XBLC_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
    UNUSED(speed);
    *length = sizeof(USBD_XBLC_DeviceDesc);
    return USBD_XBLC_DeviceDesc;
}

static uint8_t USBD_XBLC_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
    UNUSED(cfgidx);
    memset(&gxblc_data, 0, sizeof(gxblc_data));
    pdev->pClassData = &gxblc_data;

    // Testing appears to indicate stock XBLC starts up at 16kHz
    xblc_data_t *xblc_data = (xblc_data_t *)pdev->pClassData;
    xblc_data->sample_rate = XBLC_RATE_16000;
    xblc_data->current_packet_size = sample_rates[XBLC_RATE_16000].bytes_per_frame;

    USBD_LL_OpenEP(pdev, USBD_XBLC_IN_EP, USBD_EP_TYPE_ISOC, USBD_XBLC_MAX_PACKET);
    USBD_LL_OpenEP(pdev, USBD_XBLC_OUT_EP, USBD_EP_TYPE_ISOC, USBD_XBLC_MAX_PACKET);
    pdev->ep_in[USBD_XBLC_IN_EP & 0xFU].is_used = 1U;
    pdev->ep_out[USBD_XBLC_OUT_EP & 0xFU].is_used = 1U;

    USBD_LL_PrepareReceive(pdev, USBD_XBLC_OUT_EP, xblc_data->host_out_buffer, USBD_XBLC_MAX_PACKET);
    USBD_LL_Transmit(pdev, USBD_XBLC_IN_EP, xblc_data->host_in_buffer, xblc_data->current_packet_size);
    return (uint8_t)USBD_OK;
}

static uint8_t USBD_XBLC_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
    USBD_LL_CloseEP(pdev, USBD_XBLC_OUT_EP);
    USBD_LL_CloseEP(pdev, USBD_XBLC_IN_EP);

    pdev->ep_out[USBD_XBLC_OUT_EP & 0xFU].is_used = 0U;
    pdev->ep_in[USBD_XBLC_IN_EP & 0xFU].is_used = 0U;

    pdev->pClassData = NULL;

    return USBD_OK;
}

static uint8_t USBD_XBLC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
    xblc_data_t *xblc_data = (xblc_data_t *)pdev->pClassData;
    if (xblc_data == NULL) {
        return USBD_FAIL;
    }

    if (req->bmRequest == (USB_REQ_TYPE_VENDOR | USB_REQ_RECIPIENT_INTERFACE)) {
        if (req->wIndex == 0 && req->wValue & 0x100) {
            xblc_data->sample_rate = req->wValue & 0xFF;
            if (xblc_data->sample_rate >= 5) {
                xblc_data->sample_rate = 0;
            }
            uint8_t new_mp = sample_rates[xblc_data->sample_rate].bytes_per_frame;
            xblc_data->current_packet_size = new_mp;

            USBD_XBLC_Sample_Rate_Changed(pdev, sample_rates[xblc_data->sample_rate].sample_rate);

            xblc_data->host_in_sample_count = 0;

            USBD_LL_FlushEP(pdev, USBD_XBLC_IN_EP);
            USBD_LL_FlushEP(pdev, USBD_XBLC_OUT_EP);

            return USBD_OK;
        } else if (req->wIndex == 1) {
            xblc_data->agc = req->wValue & 0xFF;
            USBD_XBLC_AGC_Changed(pdev, xblc_data->agc);
            return USBD_OK;
        }
    }
    USBD_CtlError(pdev, req);
    return USBD_FAIL;
}

static uint8_t USBD_XBLC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum) {
    xblc_data_t *xblc_data = (xblc_data_t *)pdev->pClassData;
    if (xblc_data == NULL) {
        return USBD_FAIL;
    }

    if (epnum == (USBD_XBLC_IN_EP & 0x7F)) {
        xblc_data->host_in_sample_count++;
        // Work out if we need to add an extra sample to this transfer.
        // Some sample rates don't divide evenly into 1ms USB frames so every n samples an extra sample is added
        uint8_t frames_per_extra_sample = sample_rates[xblc_data->sample_rate].frames_per_extra_sample;
        uint8_t extra_sample = 0;
        if (frames_per_extra_sample > 0) {
            extra_sample = (xblc_data->host_in_sample_count % frames_per_extra_sample == 0) ? 2 : 0;
        }
        xblc_data->host_in_sample_count = 0;

        USBD_XBLC_Get_Data(pdev, xblc_data->host_in_buffer, xblc_data->current_packet_size + extra_sample);
        USBD_LL_Transmit(pdev, USBD_XBLC_IN_EP, xblc_data->host_in_buffer,
                         xblc_data->current_packet_size + extra_sample);
    }
    return (uint8_t)USBD_OK;
}

static uint8_t USBD_XBLC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum) {
    xblc_data_t *xblc_data = (xblc_data_t *)pdev->pClassData;
    if (xblc_data == NULL) {
        return USBD_FAIL;
    }
    if (epnum == USBD_XBLC_OUT_EP) {
        uint16_t PacketSize = (uint16_t)USBD_LL_GetRxDataSize(pdev, epnum);
        USBD_XBLC_Data_Received(pdev, xblc_data->host_out_buffer, PacketSize);
        USBD_LL_PrepareReceive(pdev, USBD_XBLC_OUT_EP, xblc_data->host_out_buffer, USBD_XBLC_MAX_PACKET);
    }
    return USBD_OK;
}

static uint8_t *USBD_XBLC_GetCfgDesc(uint16_t *length) {
    *length = (uint16_t)sizeof(USBD_XBLC_CfgDesc);

    return USBD_XBLC_CfgDesc;
}

static uint8_t USBD_XBLC_SOF(USBD_HandleTypeDef *pdev) {
    return (uint8_t)USBD_OK;
}

static uint8_t USBD_XBLC_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum) {
    return (uint8_t)USBD_OK;
}

static uint8_t USBD_XBLC_IsoOUTIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum) {
    return (uint8_t)USBD_OK;
}

USBD_DescriptorsTypeDef XBLC_DESC = {USBD_XBLC_DeviceDescriptor, NULL, NULL, NULL, NULL, NULL, NULL};
USBD_ClassTypeDef XBLC_USBD = {
    USBD_XBLC_Init,
    USBD_XBLC_DeInit,
    USBD_XBLC_Setup,
    NULL,
    NULL,
    USBD_XBLC_DataIn,
    USBD_XBLC_DataOut,
    USBD_XBLC_SOF,
    USBD_XBLC_IsoINIncomplete,
    USBD_XBLC_IsoOUTIncomplete,
    USBD_XBLC_GetCfgDesc,
    USBD_XBLC_GetCfgDesc,
    USBD_XBLC_GetCfgDesc,
    NULL,
};

void MX_USB_Device_Init(void) {
    if (USBD_Init(&hUsbDeviceFS, &XBLC_DESC, DEVICE_FS) != USBD_OK) {
        Error_Handler();
    }
    if (USBD_RegisterClass(&hUsbDeviceFS, &XBLC_USBD) != USBD_OK) {
        Error_Handler();
    }
    if (USBD_Start(&hUsbDeviceFS) != USBD_OK) {
        Error_Handler();
    }
}
