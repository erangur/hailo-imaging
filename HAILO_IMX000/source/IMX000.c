/******************************************************************************\
|* Copyright (c) 2024 Hailo Technologies Ltd. (Hailo) All rights reserved.    *|
|*                                                                            *|
|* This proprietary software is the confidential and proprietary information  *|
|* of Hailo Technologies Ltd. and licensed to Hailo by VeriSilicon Holdings   *|
|* Co., Ltd. You may not disclose, copy, distribute, any part of this         *|
|* software without the express written permission of Hailo.                  *|
|* You may use this software only in accordance with the terms of the license *|
|* agreement provided with the software ("End User License Agreement").       *|
|*                                                                            *|
\******************************************************************************/
/******************************************************************************\
|* Copyright (c) 2020 by VeriSilicon Holdings Co., Ltd. ("VeriSilicon")       *|
|* All Rights Reserved.                                                       *|
|*                                                                            *|
|* The material in this file is confidential and contains trade secrets of    *|
|* of VeriSilicon.  This is proprietary information owned or licensed by      *|
|* VeriSilicon.  No part of this work may be disclosed, reproduced, copied,   *|
|* transmitted, or used in any way for any purpose, without the express       *|
|* written permission of VeriSilicon.                                         *|
|*                                                                            *|
\******************************************************************************/

#include <common/return_codes.h>
#include <ebase/builtins.h>
#include <ebase/types.h>
#include <common/misc.h>
#include <fcntl.h>
#include <isi/isi.h>
#include <isi/isi_iss.h>
#include <isi/isi_priv.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <sys/ioctl.h>

#include "IMX000_priv.h"
#include "vvsensor.h"

CREATE_TRACER(IMX000_INFO, "IMX000: ", INFO, 1);
CREATE_TRACER(IMX000_WARN, "IMX000: ", WARNING, 1);
CREATE_TRACER(IMX000_ERROR, "IMX000: ", ERROR, 1);
CREATE_TRACER(IMX000_DEBUG, "IMX000: ", INFO, 1);
CREATE_TRACER(IMX000_REG_INFO, "IMX000: ", INFO, 1);
CREATE_TRACER(IMX000_REG_DEBUG, "IMX000: ", INFO, 1);

#include <fcntl.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#define IMX000_I2C_ADDR 0x1a
#define IMX000_IRIS_MIN_VAL 1
#define IMX000_IRIS_MAX_VAL 1
#define IMX000_MIN_GAIN_STEP                                          \
    (0.035) /**< min gain step size used by GUI (hardware min = 1/16; \
               1/16..32/16 depending on actual gain ) */
#define IMX000_PLL_PCLK 74250000
#define IMX000_HMAX 0xaec
#define IMX000_VMAX_30FPS 4500
#define IMX000_VMAX_HDR 6750
#define IMX000_VMAX_MAX ((1 << 20) - 2)
#define IMX000_MIN_SHR 3
#define IMX000_MAX_GAIN_AEC                                                    \
    (32.0f) /**< max. gain used by the AEC (arbitrarily chosen, hardware limit \
               = 62.0, driver limit = 32.0 ) */
#define IMX000_VS_MAX_INTEGRATION_TIME (0.0018)
#define IMX000_TRANSFER_BUFFER_LENGTH 3
#define IMX000_MAX_GAIN 3981
#define IMX000_TRANSFER_BUFFER_LENGTH 3
#define IMX000_SHR0_RHS2_GAP 7
#define IMX000_SHR0_FSC_GAP 3
#define IMX000_SHR1_MIN_GAP 7
#define IMX000_SHR1_RHS1_GAP 3
#define IMX000_SHR2_RHS1_GAP 7
#define IMX000_SHR2_RHS2_GAP 3
#define IMX000_PIXEL_CLK_RATE 74.25
#define DEFAULT_RHS1 0x91
#define DEFAULT_RHS2 0xaa
#define MICRO_2_NANO 1000

#define SPI_IOC_MAGIC   'k'
#define HAILO15_IOC_GET_IRIS    _IOR(SPI_IOC_MAGIC, 1, int)
#define HAILO15_IOC_SET_IRIS    _IOW(SPI_IOC_MAGIC, 2, int)


/*****************************************************************************
 *Forward Declarations
*****************************************************************************/
RESULT IMX000_IsiSetIrisIss( IsiSensorHandle_t handle,
                    const float NewIris);

/*****************************************************************************
 *Sensor Info
*****************************************************************************/

static struct vvsensor_mode_s pimx000_mode_info[] = {
    {
        .index     = 0,
        .size      ={
			.bounds_width  = 3840,
			.bounds_height = 2160,
			.top           = 0,
			.left          = 0,
			.width         = 3840,
			.height        = 2160,
	},
	.fps       = 30 * ISI_FPS_ACCURACY,
	.hdr_mode  = SENSOR_MODE_LINEAR,
	.bit_width = 12,
	.bayer_pattern = BAYER_GBRG,
	.ae_info = {
		.one_line_exp_time_ns = 7410,
		.max_integration_time = IMX000_VMAX_30FPS - IMX000_MIN_SHR,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX000_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 30
	}
    },
    {
        .index     = 1,
        .size      ={
			.bounds_width  = 1920,
			.bounds_height = 1080,
			.top           = 0,
			.left          = 0,
			.width         = 1920,
			.height        = 1080,
	},
	.fps       = 30 * ISI_FPS_ACCURACY,
	.hdr_mode  = SENSOR_MODE_LINEAR,
	.bit_width = 12,
	.bayer_pattern = BAYER_RGGB,
	.ae_info = {
		.one_line_exp_time_ns = 7410,
		.max_integration_time = IMX000_VMAX_30FPS - IMX000_MIN_SHR,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX000_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 30
	}
    },
    {
        .index     = 2,
        .size      ={
			.bounds_width  = 1920,
			.bounds_height = 1080,
			.top           = 0,
			.left          = 0,
			.width         = 1920,
			.height        = 1080,
		},
		.fps       = 20 * ISI_FPS_ACCURACY,
		.hdr_mode  = SENSOR_MODE_HDR_STITCH,
		.stitching_mode = SENSOR_STITCHING_3DOL,
		.bit_width = 12,
		.bayer_pattern = BAYER_GBRG,
        .ae_info = {
		.one_line_exp_time_ns = 7410,
		.max_integration_time = DEFAULT_RHS1 - IMX000_SHR1_MIN_GAP,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX000_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 20
	}
    },
    {
        .index     = 3,
        .size      ={
			.bounds_width  = 3840,
			.bounds_height = 2160,
			.top           = 0,
			.left          = 0,
			.width         = 3840,
			.height        = 2160,
		},
		.fps       = 20 * ISI_FPS_ACCURACY,
		.hdr_mode  = SENSOR_MODE_HDR_STITCH,
		.stitching_mode = SENSOR_STITCHING_3DOL,
		.bit_width = 12,
		.bayer_pattern = BAYER_GBRG,
        .ae_info = {
		.one_line_exp_time_ns = 7410,
		.max_integration_time = DEFAULT_RHS1 - IMX000_SHR1_MIN_GAP,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX000_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 20
	}
    }

};

static RESULT IMX000_IsiSetPowerIss(IsiSensorHandle_t handle, bool_t on)
{
    printf("--- [INFO] %s (enter)\n", __func__); 

    RESULT result = RET_SUCCESS;

    return (result);
}

static RESULT IMX000_IsiCreateIss(IsiSensorInstanceConfig_t* pConfig) {
    RESULT result = RET_SUCCESS;
    IMX000_Context_t* pIMX000Ctx;
    char i2c_file_path[PATH_MAX];    
    memset(i2c_file_path, 0, PATH_MAX);
    printf("---\n---\n---\n"); 
    printf("--- [INFO] %s (enter)\n", __func__); 

    if (!pConfig || !pConfig->pSensor) return (RET_NULL_POINTER);

    pIMX000Ctx = (IMX000_Context_t*)malloc(sizeof(IMX000_Context_t));
    if (!pIMX000Ctx) {
        return (RET_OUTOFMEM);
    }

    MEMSET(pIMX000Ctx, 0, sizeof(IMX000_Context_t));
    result = HalAddRef(pConfig->HalHandle);
    if (result != RET_SUCCESS) {
        free(pIMX000Ctx);
        return (result);
    }

    pIMX000Ctx->IsiCtx.HalHandle = pConfig->HalHandle;
    pIMX000Ctx->IsiCtx.pSensor = pConfig->pSensor;
    pIMX000Ctx->GroupHold = BOOL_FALSE;
    pIMX000Ctx->OldGain = 1.0;
    pIMX000Ctx->OldIntegrationTime = 0.01;
    pIMX000Ctx->Configured = BOOL_FALSE;
    pIMX000Ctx->Streaming = BOOL_FALSE;
    pIMX000Ctx->TestPattern = BOOL_FALSE;
    pIMX000Ctx->isAfpsRun = BOOL_FALSE;
    pIMX000Ctx->SensorMode.index = pConfig->SensorModeIndex;
    pConfig->SensorModeIndex =
	    MAX(MIN((sizeof(pimx000_mode_info) / sizeof(pimx000_mode_info[0])),
		    pConfig->SensorModeIndex),
		0);
    memcpy(&pIMX000Ctx->SensorMode,
	   &pimx000_mode_info[pConfig->SensorModeIndex],
	   sizeof(pIMX000Ctx->SensorMode));
    pConfig->hSensor = (IsiSensorHandle_t)pIMX000Ctx;
    pIMX000Ctx->pattern = ISI_BPAT_RGRGGBGB;
    pIMX000Ctx->subdev = HalGetFdHandle(pConfig->HalHandle,
                                        HAL_MODULE_SENSOR);  // two sensors??
    pIMX000Ctx->KernelDriverFlag = 1;
    sprintf(i2c_file_path, "/dev/i2c-%d", pConfig->I2cBusNum);
    pIMX000Ctx->i2c_fd = open(i2c_file_path, O_RDWR);
    if (pIMX000Ctx->i2c_fd < 0) {
        TRACE(IMX000_INFO, "unable to open /dev/i2c-%d\n", pConfig->I2cBusNum);
        return RET_FAILURE;
    }

    if (ioctl(pIMX000Ctx->i2c_fd, I2C_SLAVE_FORCE, IMX000_I2C_ADDR) < 0) {
        TRACE(IMX000_INFO, "unable to set I2C_SLAVE_FORCE on /dev/i2c-%d\n",
              pConfig->I2cBusNum);
        return RET_FAILURE;
    }
    return (result);
}

static RESULT IMX000_IsiReleaseIss(IsiSensorHandle_t handle) {
    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    printf("--- [INFO] %s (enter)\n", __func__); 

    if (pIMX000Ctx == NULL) return (RET_WRONG_HANDLE);

    (void)IMX000_IsiSetStreamingIss(pIMX000Ctx, BOOL_FALSE);
    (void)IMX000_IsiSetPowerIss(pIMX000Ctx, BOOL_FALSE);
    (void)HalDelRef(pIMX000Ctx->IsiCtx.HalHandle);
    close(pIMX000Ctx->i2c_fd);
    MEMSET(pIMX000Ctx, 0, sizeof(IMX000_Context_t));
    free(pIMX000Ctx);
    return (result);
}

static RESULT IMX000_IsiReadRegIss(IsiSensorHandle_t handle,
                                   const uint32_t Addr, uint32_t* pValue) {
    RESULT result = RET_SUCCESS;    
    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;
    char out[IMX000_TRANSFER_BUFFER_LENGTH];
    printf("--- [INFO] %s (enter)\n", __func__); 

    if (pIMX000Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }

    memset(out, 0, IMX000_TRANSFER_BUFFER_LENGTH);
    out[0] = (Addr >> 8) & 0xff;
    out[1] = Addr & 0xff;
    if (write(pIMX000Ctx->i2c_fd, out, sizeof(uint16_t)) != sizeof(uint16_t)) {
        return RET_FAILURE;
    }

    if (read(pIMX000Ctx->i2c_fd, out, 1) != 1) return RET_FAILURE;

    *pValue = out[0];

    return (result);
}

static RESULT IMX000_IsiWriteRegIss(IsiSensorHandle_t handle,
                                    const uint32_t Addr, const uint32_t Value) {
    RESULT result = RET_SUCCESS;
    char out[IMX000_TRANSFER_BUFFER_LENGTH];

    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;
    if (pIMX000Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }
    printf("--- [INFO] %s (enter)\n", __func__); 

    memset(out, 0, IMX000_TRANSFER_BUFFER_LENGTH);
    out[0] = (Addr >> 8) & 0xff;
    out[1] = Addr & 0xff;
    out[2] = Value;
    if (write(pIMX000Ctx->i2c_fd, out, sizeof(out)) != sizeof(out))
        result = RET_FAILURE;
    return (result);
}

static RESULT IMX000_UpdateFps(IMX000_Context_t *pIMX000Ctx, uint32_t vmax) {
    float frame_time = 0;
    frame_time = (vmax * pIMX000Ctx->one_line_exp_time);
    if (frame_time == 0) return RET_FAILURE;
    printf("--- [INFO] %s (enter)\n", __func__); 

    pIMX000Ctx->CurrFps = (uint32_t)(ceil(1 / frame_time));
    return RET_SUCCESS;
}

static RESULT IMX000_ReadVmax(IsiSensorHandle_t handle, uint32_t* vmax) {
    uint32_t vmax_low = 0, vmax_mid = 0, vmax_high = 0;
    RESULT result;
    printf("--- [INFO] %s (enter)\n", __func__); 

    result = IMX000_IsiReadRegIss(handle, 0x3024, &vmax_low);
    result |= IMX000_IsiReadRegIss(handle, 0x3025, &vmax_mid);
    result |= IMX000_IsiReadRegIss(handle, 0x3026, &vmax_high);
    if (result) return RET_FAILURE;

    *vmax = (vmax_high << 16) | (vmax_mid << 8) | vmax_low;
    return result;
}

static RESULT IMX000_ReadHmax(IsiSensorHandle_t handle, uint32_t* hmax) {
    uint32_t hmax_low = 0, hmax_high = 0;
    RESULT result;
    printf("--- [INFO] %s (enter)\n", __func__); 

    result = IMX000_IsiReadRegIss(handle, 0x3028, &hmax_low);
    result |= IMX000_IsiReadRegIss(handle, 0x3029, &hmax_high);
    if (result) return RET_FAILURE;

    *hmax = (hmax_high << 8) | hmax_low;
    return result;
}

static RESULT IMX000_WriteVmax(IsiSensorHandle_t handle, uint32_t vmax) {
    RESULT result;
    printf("--- [INFO] %s (enter)\n", __func__); 

    result = IMX000_IsiWriteRegIss(handle, 0x3024, vmax & 0xff);
    result |= IMX000_IsiWriteRegIss(handle, 0x3025, (vmax >> 8) & 0xff);
    result |= IMX000_IsiWriteRegIss(handle, 0x3026, (vmax >> 16) & 0x0f);
    if (!result) {
        return IMX000_UpdateFps((IMX000_Context_t *)handle, vmax);
    }

    return result;
}

static RESULT IMX000_ReadRHS1(IsiSensorHandle_t handle, uint32_t* rhs1) {
    uint32_t rhs1_low = 0, rhs1_mid = 0, rhs1_high = 0;
    RESULT result;
    printf("--- [INFO] %s (enter)\n", __func__); 

    result = IMX000_IsiReadRegIss(handle, 0x3060, &rhs1_low);
    result |= IMX000_IsiReadRegIss(handle, 0x3061, &rhs1_mid);
    result |= IMX000_IsiReadRegIss(handle, 0x3062, &rhs1_high);
    if (result) return RET_FAILURE;

    *rhs1 = (rhs1_high << 16) | (rhs1_mid << 8) | rhs1_low;
    return result;
}

static RESULT IMX000_ReadRHS2(IsiSensorHandle_t handle, uint32_t* rhs2) {
    uint32_t rhs2_low = 0, rhs2_mid = 0, rhs2_high = 0;
    RESULT result;
    printf("--- [INFO] %s (enter)\n", __func__); 

    result = IMX000_IsiReadRegIss(handle, 0x3064, &rhs2_low);
    result |= IMX000_IsiReadRegIss(handle, 0x3065, &rhs2_mid);
    result |= IMX000_IsiReadRegIss(handle, 0x3066, &rhs2_high);
    if (result) return RET_FAILURE;

    *rhs2 = (rhs2_high << 16) | (rhs2_mid << 8) | rhs2_low;
    return result;
}

static RESULT IMX000_WriteShr0(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;
    printf("--- [INFO] %s (enter)\n", __func__); 

    result = IMX000_IsiWriteRegIss(handle, 0x3050, shr & 0xff);
    result |= IMX000_IsiWriteRegIss(handle, 0x3051, (shr >> 8) & 0xff);
    result |= IMX000_IsiWriteRegIss(handle, 0x3052, (shr >> 16) & 0x0f);

    return result;
}

static RESULT IMX000_WriteShr1(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;
    printf("--- [INFO] %s (enter)\n", __func__); 

    result = IMX000_IsiWriteRegIss(handle, 0x3054, shr & 0xff);
    result |= IMX000_IsiWriteRegIss(handle, 0x3055, (shr >> 8) & 0xff);
    result |= IMX000_IsiWriteRegIss(handle, 0x3056, (shr >> 16) & 0x0f);

    return result;
}

static RESULT IMX000_WriteShr2(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;
    printf("--- [INFO] %s (enter)\n", __func__); 

    result = IMX000_IsiWriteRegIss(handle, 0x3058, shr & 0xff);
    result |= IMX000_IsiWriteRegIss(handle, 0x3059, (shr >> 8) & 0xff);
    result |= IMX000_IsiWriteRegIss(handle, 0x305a, (shr >> 16) & 0x0f);

    return result;
}

static RESULT IMX000_WriteGain(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;
    printf("--- [INFO] %s (enter)\n", __func__); 

    result = IMX000_IsiWriteRegIss(handle, 0x3090, (gain & 0x00ff));
	result |= IMX000_IsiWriteRegIss(handle, 0x3091, (gain & 0x0700) >> 8);

    return result;
}

static RESULT IMX000_WriteGain1(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;
    printf("--- [INFO] %s (enter)\n", __func__); 

    result = IMX000_IsiWriteRegIss(handle, 0x3092, (gain & 0x00ff));
	result |= IMX000_IsiWriteRegIss(handle, 0x3093, (gain & 0x0700) >> 8);

    return result;
}

static RESULT IMX000_WriteGain2(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;
    printf("--- [INFO] %s (enter)\n", __func__); 

    result = IMX000_IsiWriteRegIss(handle, 0x3094, (gain & 0x00ff));
	result |= IMX000_IsiWriteRegIss(handle, 0x3095, (gain & 0x0700) >> 8);

    return result;
}

static RESULT IMX000_LockRegHold(IsiSensorHandle_t handle) {
    RESULT result;
    printf("--- [INFO] %s (enter)\n", __func__); 

    result = IMX000_IsiWriteRegIss(handle, 0x3001, 0x1);

    return result;
}

static RESULT IMX000_UnlockRegHold(IsiSensorHandle_t handle) {
    RESULT result;
    printf("--- [INFO] %s (enter)\n", __func__); 

    result = IMX000_IsiWriteRegIss(handle, 0x3001, 0x0);

    return result;
}

static RESULT IMX000_IsiGetModeIss(IsiSensorHandle_t handle, IsiMode_t* pMode) {
    printf("--- [INFO] %s (enter)\n", __func__); 
    
    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;
    if (pIMX000Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }
    memcpy(pMode, &(pIMX000Ctx->SensorMode), sizeof(pIMX000Ctx->SensorMode));

    printf("--- [INFO] %s (exit)\n", __func__); 
    return (RET_SUCCESS);
}

static RESULT IMX000_IsiSetModeIss(IsiSensorHandle_t handle, IsiMode_t* pMode) {
    int ret = 0;
    printf("--- [INFO] %s (enter)\n", __func__); 

    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;
    if (pIMX000Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }
    HalContext_t* pHalCtx = (HalContext_t*)pIMX000Ctx->IsiCtx.HalHandle;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_SENSOR_MODE, pMode);
    if (ret != 0) {
        return (RET_FAILURE);
    }

    return (RET_SUCCESS);
}

static RESULT IMX000_IsiHalEnumModeIss(HalHandle_t HalHandle,
                                       IsiEnumMode_t* pEnumMode) {
    HalContext_t* pHalCtx = HalHandle;
    printf("--- [INFO] %s (enter)\n", __func__); 

    if (pHalCtx == NULL) {
        return RET_NULL_POINTER;
    }

    if (pEnumMode->index >=
        (sizeof(pimx000_mode_info) / sizeof(pimx000_mode_info[0])))
        return RET_OUTOFRANGE;

    for (uint32_t i = 0;
         i < (sizeof(pimx000_mode_info) / sizeof(pimx000_mode_info[0])); i++) {
        if (pimx000_mode_info[i].index == pEnumMode->index) {
            memcpy(&pEnumMode->mode, &pimx000_mode_info[i], sizeof(IsiMode_t));            
            printf("--- [INFO] %s (exit)\n", __func__); 
            return RET_SUCCESS;
        }
    }

    return RET_NOTSUPP;
}

static RESULT IMX000_IsiEnumModeIss(IsiSensorHandle_t handle,
                                    IsiEnumMode_t* pEnumMode) {
    RESULT result = RET_SUCCESS;
    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;
    printf("--- [INFO] %s (enter)\n", __func__); 

    if (pIMX000Ctx == NULL || pIMX000Ctx->IsiCtx.HalHandle == NULL) {
        return RET_NULL_POINTER;
    }

    HalContext_t* pHalCtx = (HalContext_t*)pIMX000Ctx->IsiCtx.HalHandle;
    result = IMX000_IsiHalEnumModeIss(pHalCtx, pEnumMode);
    if (result != RET_SUCCESS) {
        TRACE(IMX000_ERROR, "%s: sensor enum mode error!\n", __func__);
        return (RET_FAILURE);
    }

    return result;
}

static RESULT IMX000_IsiGetCapsIss(IsiSensorHandle_t handle, IsiCaps_t* pCaps) {
    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;

    RESULT result = RET_SUCCESS;

    printf("--- [INFO] %s (enter)\n", __func__); 

    if (pIMX000Ctx == NULL) return (RET_WRONG_HANDLE);

    if (pCaps == NULL) {
        return (RET_NULL_POINTER);
    }

    if (!pIMX000Ctx->Configured) IMX000_IsiSetupIss(handle, pCaps);

    pCaps->BusWidth = pIMX000Ctx->SensorMode.bit_width;
    pCaps->Mode = ISI_MODE_BAYER;
    pCaps->FieldSelection = ISI_FIELDSEL_BOTH;
    pCaps->YCSequence = ISI_YCSEQ_YCBYCR;
    pCaps->Conv422 = ISI_CONV422_COSITED;
    pCaps->BPat = pIMX000Ctx->SensorMode.bayer_pattern;
    pCaps->HPol = ISI_HPOL_REFPOS;
    pCaps->VPol = ISI_VPOL_POS;
    pCaps->Edge = ISI_EDGE_RISING;
    pCaps->Resolution.width = pIMX000Ctx->SensorMode.size.width;
    pCaps->Resolution.height = pIMX000Ctx->SensorMode.size.height;
    pCaps->SmiaMode = ISI_SMIA_OFF;
    pCaps->MipiLanes = ISI_MIPI_4LANES;

    if (pCaps->BusWidth == 10) {
        pCaps->MipiMode = ISI_MIPI_MODE_RAW_10;
    } else if (pCaps->BusWidth == 12) {
        pCaps->MipiMode = ISI_MIPI_MODE_RAW_12;
    } else {
        pCaps->MipiMode = ISI_MIPI_OFF;
    }
    TRACE(IMX000_INFO, "got caps - width %d height %d buswidth %d\n",
          pCaps->Resolution.width, pCaps->Resolution.height, pCaps->BusWidth);
    printf("--- [INFO] %s (exit)\n", __func__); 
    return (result);
}

static RESULT IMX000_AecSetModeParameters(IMX000_Context_t* pIMX000Ctx,
                                          const IsiCaps_t* pConfig) {
    RESULT result = RET_SUCCESS;
    printf("--- [INFO] %s (enter)\n", __func__); 
    TRACE(IMX000_INFO, "%s%s: (enter)\n", __func__,
          pIMX000Ctx->isAfpsRun ? "(AFPS)" : "");

    pIMX000Ctx->AecIntegrationTimeIncrement = pIMX000Ctx->one_line_exp_time;
    pIMX000Ctx->AecMinIntegrationTime =
        pIMX000Ctx->one_line_exp_time * pIMX000Ctx->MinIntegrationLine;
    pIMX000Ctx->AecMaxIntegrationTime =
        pIMX000Ctx->one_line_exp_time * pIMX000Ctx->MaxIntegrationLine;

    TRACE(IMX000_DEBUG, "%s%s: AecMaxIntegrationTime = %f \n", __func__,
          pIMX000Ctx->isAfpsRun ? "(AFPS)" : "",
          pIMX000Ctx->AecMaxIntegrationTime);

    pIMX000Ctx->AecGainIncrement = IMX000_MIN_GAIN_STEP;

    // reflects the state of the sensor registers, must equal default settings
    pIMX000Ctx->AecCurGainLEF = pIMX000Ctx->AecMinGain;
	pIMX000Ctx->AecCurGainSEF1 = pIMX000Ctx->AecMinGain;
	pIMX000Ctx->AecCurGainSEF2 = pIMX000Ctx->AecMinGain;
	pIMX000Ctx->AecCurIntegrationTimeLEF = pIMX000Ctx->AecMaxIntegrationTime;
	pIMX000Ctx->AecCurIntegrationTimeSEF1 = pIMX000Ctx->AecMaxIntegrationTime;
	pIMX000Ctx->AecCurIntegrationTimeSEF2 = pIMX000Ctx->AecMaxIntegrationTime;
    pIMX000Ctx->OldGain = 1;
    pIMX000Ctx->OldIntegrationTime = 0.0f;

    TRACE(IMX000_INFO, "%s%s: (exit)\n", __func__,
          pIMX000Ctx->isAfpsRun ? "(AFPS)" : "");

    return (result);
}

static RESULT IMX000_IsiSetupIss(IsiSensorHandle_t handle,
                                 const IsiCaps_t* pCaps) {
    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    printf("--- [INFO] %s (enter)\n", __func__); 

    if (!pIMX000Ctx) {
        TRACE(IMX000_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pCaps == NULL) {
        TRACE(IMX000_ERROR,
              "%s: Invalid configuration (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (pIMX000Ctx->Streaming != BOOL_FALSE) {
        return RET_WRONG_STATE;
    }

    if (pIMX000Ctx->SensorMode.hdr_mode != SENSOR_MODE_LINEAR) {
        pIMX000Ctx->enableHdr = true;
    } else {
        pIMX000Ctx->enableHdr = false;
    }

    pIMX000Ctx->one_line_exp_time =
        (float)(pIMX000Ctx->SensorMode.ae_info.one_line_exp_time_ns) /
        1000000000;
    pIMX000Ctx->MaxIntegrationLine =
        pIMX000Ctx->SensorMode.ae_info.max_integration_time;
    TRACE(IMX000_INFO, "%s: MaxIntegrationLine %u\n", __func__,
          pIMX000Ctx->MaxIntegrationLine);
    pIMX000Ctx->MinIntegrationLine =
        pIMX000Ctx->SensorMode.ae_info.min_integration_time;
    pIMX000Ctx->gain_accuracy = pIMX000Ctx->SensorMode.ae_info.gain_accuracy;
    pIMX000Ctx->AecMaxGain = (float)(pIMX000Ctx->SensorMode.ae_info.max_gain) /
                             pIMX000Ctx->gain_accuracy;
    pIMX000Ctx->AecMinGain = (float)(pIMX000Ctx->SensorMode.ae_info.min_gain) /
                             pIMX000Ctx->gain_accuracy;

    pIMX000Ctx->AecMinIris = IMX000_IRIS_MIN_VAL;
    pIMX000Ctx->AecMaxIris = IMX000_IRIS_MAX_VAL;

    pIMX000Ctx->original_vmax = 0;
    pIMX000Ctx->unlimit_fps = 0;

    pIMX000Ctx->MaxFps = pIMX000Ctx->SensorMode.fps;
    pIMX000Ctx->CurrFps = pIMX000Ctx->MaxFps;
    TRACE(IMX000_INFO, "%s - got caps - width %d height %d buswidth %d\n",
          __func__, pIMX000Ctx->SensorMode.size.width,
          pIMX000Ctx->SensorMode.size.height, pIMX000Ctx->SensorMode.bit_width);

    TRACE(IMX000_INFO, "%s - MinGain %f MaxGain %f\n", __func__, pIMX000Ctx->AecMinGain,
          pIMX000Ctx->AecMaxGain);

    memcpy(&pIMX000Ctx->CapsConfig, pCaps, sizeof(pIMX000Ctx->CapsConfig));

    /* 1.) SW reset of image sensor (via I2C register interface)  be careful,
     * bits 6..0 are reserved, reset bit is not sticky */
    TRACE(IMX000_DEBUG, "%s: IMX000 System-Reset executed\n", __func__);
    osSleep(100);

    result = IMX000_AecSetModeParameters(pIMX000Ctx, pCaps);
    if (result != RET_SUCCESS) {
        TRACE(IMX000_ERROR, "%s: SetupOutputWindow failed.\n", __func__);
        return (result);
    }

    pIMX000Ctx->Configured = BOOL_TRUE;
    printf("--- [INFO] %s (exit)\n", __func__); 
    return 0;
}

static RESULT IMX000_IsiCheckConnectionIss(IsiSensorHandle_t handle) {
    printf("--- [INFO] %s (enter)\n", __func__); 
    RESULT result = RET_SUCCESS;
    return (result);
}

// Not tested
static RESULT IMX000_IsiGetRevisionIss(IsiSensorHandle_t handle,
                                       uint32_t* pValue) {
    RESULT result = RET_SUCCESS;
    int ret = 0;
    uint32_t reg_val;
    uint32_t sensor_id;

    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;
    if (pIMX000Ctx == NULL || pIMX000Ctx->IsiCtx.HalHandle == NULL) {
        return RET_NULL_POINTER;
    }
    HalContext_t* pHalCtx = (HalContext_t*)pIMX000Ctx->IsiCtx.HalHandle;

    if (!pValue) return (RET_NULL_POINTER);

    if (pIMX000Ctx->KernelDriverFlag) {
        ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_CHIP_ID, &sensor_id);
        if (ret != 0) {
            TRACE(IMX000_ERROR, "%s: Read Sensor ID Error! \n", __func__);
            return (RET_FAILURE);
        }
    } else {
        reg_val = 0;
        result = IMX000_IsiReadRegIss(handle, 0x3a04, &reg_val);
        sensor_id = (reg_val & 0xff) << 8;

        reg_val = 0;
        result |= IMX000_IsiReadRegIss(handle, 0x3a05, &reg_val);
        sensor_id |= (reg_val & 0xff);
    }

    *pValue = sensor_id;
    printf("--- [INFO] %s (exit)\n", __func__); 
    return (result);
}

static RESULT IMX000_IsiSetStreamingIss(IsiSensorHandle_t handle, bool_t on) {
    RESULT result = RET_SUCCESS;
    printf("--- [INFO] %s (enter)\n", __func__); 

    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;
    if (pIMX000Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }
    pIMX000Ctx->Streaming = on;

    if (pIMX000Ctx->enableHdr)
        return result;

    printf("--- [INFO] %s (exit)\n", __func__); 
    return (result);
}

static RESULT IMX000_IsiGetGainLimitsIss(IsiSensorHandle_t handle,
                                         float* pMinGain, float* pMaxGain) {
    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    printf("--- [INFO] %s (enter)\n", __func__);     

    if (pIMX000Ctx == NULL) {
        TRACE(IMX000_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinGain == NULL) || (pMaxGain == NULL)) {
        TRACE(IMX000_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    *pMinGain = pIMX000Ctx->AecMinGain;
    *pMaxGain = pIMX000Ctx->AecMaxGain;

    printf("--- [INFO] %s (exit)\n", __func__); 
    return (result);
}

static RESULT IMX000_IsiUnlimitFpsIss(IsiSensorHandle_t handle,
                                      float maxIntegrationTime) {
    RESULT result = RET_SUCCESS;
    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;
    printf("--- [INFO] %s (enter)\n", __func__); 

    if (!pIMX000Ctx) {
        return RET_NULL_POINTER;
    }

    if (pIMX000Ctx->enableHdr) {
        return result;
    }
    pIMX000Ctx->unlimit_fps = 1;

    if (maxIntegrationTime < 0)
        pIMX000Ctx->MaxIntegrationLine = IMX000_VMAX_MAX - IMX000_MIN_SHR;
    else
        pIMX000Ctx->MaxIntegrationLine =
            MIN((uint32_t)(maxIntegrationTime / pIMX000Ctx->one_line_exp_time),
                IMX000_VMAX_MAX - IMX000_MIN_SHR);
    TRACE(IMX000_INFO, "%s: set MaxIntegrationLine to %u\n", __func__,
        pIMX000Ctx->MaxIntegrationLine);
    pIMX000Ctx->AecMaxIntegrationTime =
        pIMX000Ctx->one_line_exp_time * pIMX000Ctx->MaxIntegrationLine;
    return result;
}

static RESULT IMX000_IsiLimitFpsIss(IsiSensorHandle_t handle) {
    RESULT result = RET_SUCCESS;
    uint32_t current_vmax = 0;
    
    printf("--- [INFO] %s (enter)\n", __func__); 

    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;
    if (!pIMX000Ctx) 
        return RET_NULL_POINTER;

    if (pIMX000Ctx->enableHdr)
        return result;
    
    // Working around overriding VMAX issue with reset(?) value
    if (pIMX000Ctx->Streaming == BOOL_FALSE)
        return result;

    IMX000_ReadVmax(handle, &current_vmax);

    pIMX000Ctx->unlimit_fps = 0;
    if (pIMX000Ctx->original_vmax == 0) {
        pIMX000Ctx->original_vmax = current_vmax;
    }
    if (current_vmax != pIMX000Ctx->original_vmax) {
        IMX000_WriteVmax(handle, pIMX000Ctx->original_vmax);
    }

    pIMX000Ctx->MaxIntegrationLine =
        MAX(pIMX000Ctx->original_vmax - IMX000_MIN_SHR, 1);
    TRACE(IMX000_INFO, "%s: set MaxIntegrationLine to %u\n", __func__,
          pIMX000Ctx->MaxIntegrationLine);
    pIMX000Ctx->AecMaxIntegrationTime =
        pIMX000Ctx->one_line_exp_time * pIMX000Ctx->MaxIntegrationLine;
    return result;
}

static RESULT IMX000_IsiGetIrisLimitsIss(IsiSensorHandle_t handle,
                                         float* pMinIris, float* pMaxIris) {
    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    printf("--- [INFO] %s (enter)\n", __func__); 

    if (pIMX000Ctx == NULL) {
        TRACE(IMX000_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinIris == NULL) || (pMaxIris == NULL)) {
        TRACE(IMX000_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    *pMinIris = pIMX000Ctx->AecMinIris;
    *pMaxIris = pIMX000Ctx->AecMaxIris;

    printf("--- [INFO] %s (exit)\n", __func__); 
    return (result);
}

static RESULT IMX000_IsiSetIrisLimitsIss(IsiSensorHandle_t handle,
                                         float minIris, float maxIris) {
    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    printf("--- [INFO] %s (enter)\n", __func__); 

    if (pIMX000Ctx == NULL) {
        TRACE(IMX000_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    pIMX000Ctx->AecMinIris = minIris;
    pIMX000Ctx->AecMaxIris = maxIris;

    printf("--- [INFO] %s (exit)\n", __func__); 
    return (result);
}

static RESULT IMX000_IsiGetIntegrationTimeLimitsIss(
    IsiSensorHandle_t handle, float* pMinIntegrationTime,
    float* pMaxIntegrationTime) {
    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    printf("--- [INFO] %s (enter)\n", __func__); 
    if (pIMX000Ctx == NULL) {
        TRACE(IMX000_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL)) {
        TRACE(IMX000_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    *pMinIntegrationTime = pIMX000Ctx->AecMinIntegrationTime;
    *pMaxIntegrationTime = pIMX000Ctx->AecMaxIntegrationTime;
    TRACE(IMX000_INFO, "%s: (exit) %f, %f\n", 
    __func__, *pMinIntegrationTime, *pMaxIntegrationTime);
    return (result);
}

/* Gain get functions*/

RESULT IMX000_IsiGetGainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX000_Context_t *pIMX000Ctx = (IMX000_Context_t *)handle;
	printf("--- [INFO] %s (enter)\n", __func__); 

	if (pIMX000Ctx == NULL) {
		TRACE(IMX000_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}

	if (pIMX000Ctx->enableHdr)
		return IMX000_IsiGetSEF1GainIss(handle, pSetGain);

	return IMX000_IsiGetLEFGainIss(handle, pSetGain);
}

RESULT IMX000_IsiGetLEFGainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX000_Context_t *pIMX000Ctx = (IMX000_Context_t *)handle;
	RESULT result = RET_SUCCESS;
	printf("--- [INFO] %s (enter)\n", __func__); 

	if (pIMX000Ctx == NULL) {
		TRACE(IMX000_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}
	*pSetGain = pIMX000Ctx->AecCurGainLEF;
	TRACE(IMX000_DEBUG, "%s - returning %f\n", __func__, pIMX000Ctx->AecCurGainLEF);
	printf("--- [INFO] %s (exit)\n", __func__); 
	return (result);
}

// HDR has not been tested yet
RESULT IMX000_IsiGetSEF1GainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX000_Context_t *pIMX000Ctx = (IMX000_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	printf("--- [INFO] %s (enter)\n", __func__); 

	if (pIMX000Ctx == NULL) {
		TRACE(IMX000_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}

	*pSetGain = pIMX000Ctx->AecCurGainSEF1;
	TRACE(IMX000_DEBUG, "%s - returning %f\n", __func__, pIMX000Ctx->AecCurGainSEF1);

	printf("--- [INFO] %s (exit)\n", __func__); 

	return (result);
}

// HDR has not been tested yet
RESULT IMX000_IsiGetSEF2GainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX000_Context_t *pIMX000Ctx = (IMX000_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	printf("--- [INFO] %s (enter)\n", __func__); 

	if (pIMX000Ctx == NULL) {
		TRACE(IMX000_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}

	*pSetGain = pIMX000Ctx->AecCurGainSEF2;
	TRACE(IMX000_DEBUG, "%s - returning %f\n", __func__, pIMX000Ctx->AecCurGainSEF2);

	printf("--- [INFO] %s (exit)\n", __func__); 

	return (result);
}

RESULT IMX000_IsiGetGainIncrementIss(IsiSensorHandle_t handle, float* pIncr) {
    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    printf("--- [INFO] %s (enter)\n", __func__); 

    if (pIMX000Ctx == NULL) {
        TRACE(IMX000_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pIncr == NULL) return (RET_NULL_POINTER);

    *pIncr = pIMX000Ctx->AecGainIncrement;

    printf("--- [INFO] %s (exit)\n", __func__); 

    return (result);
}

RESULT IMX000_IsiGetIrisIncrementIss(IsiSensorHandle_t handle, float* pIncr) {
    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    printf("--- [INFO] %s (enter)\n", __func__); 
    if (pIMX000Ctx == NULL) {
        TRACE(IMX000_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pIncr == NULL) return (RET_NULL_POINTER);

    *pIncr = 0.0001;

    printf("--- [INFO] %s (exit)\n", __func__); 

    return (result);
}

/* Gain set functions*/

RESULT IMX000_IsiSetGainIss(IsiSensorHandle_t handle, float NewGain,
			    float *pSetGain, float *hdr_ratio)
{
	RESULT result = RET_SUCCESS;    
	IMX000_Context_t *pIMX000Ctx = (IMX000_Context_t *)handle;
    printf("--- [INFO] %s (enter)\n", __func__); 

    if (pIMX000Ctx == NULL || pIMX000Ctx->IsiCtx.HalHandle == NULL) {
		return RET_NULL_POINTER;
	}

	if (pIMX000Ctx->enableHdr) {
		result = IMX000_IsiSetSEF1GainIss(handle, 0, NewGain, pSetGain,
						hdr_ratio);

		result |= IMX000_IsiSetSEF2GainIss(handle, 0, NewGain, pSetGain,
						hdr_ratio);
	}
	result |= IMX000_IsiSetLEFGainIss(handle, NewGain, pSetGain, hdr_ratio);
	return result;
}

static inline uint32_t _linear2sensorGain(float gain)
{    
    printf("--- [INFO] %s (enter)\n", __func__); 
    uint32_t db = 0;
    float log_gain = log10(gain);
    log_gain = (log_gain * 10 * 20) / 3;
    db = (uint32_t)(log_gain);
    return db;
}

static inline float _sensorGain2linear(uint32_t db)
{
    printf("--- [INFO] %s (enter)\n", __func__); 
    float gain = ((float)(db) * 3) / 200;
    gain = pow(10, gain);
    return gain;
}

RESULT IMX000_IsiSetLEFGainIss(IsiSensorHandle_t handle, float NewGain,
			    float *pSetGain, float *hdr_ratio)
{
	RESULT result = RET_SUCCESS;
	IMX000_Context_t *pIMX000Ctx = (IMX000_Context_t *)handle;
    printf("--- [INFO] %s (enter)\n", __func__); 

	if (pIMX000Ctx == NULL || pIMX000Ctx->IsiCtx.HalHandle == NULL) {
		return RET_NULL_POINTER;
	}    
    
	TRACE(IMX000_DEBUG, "%s: got NewGain %f\n",
	__func__, NewGain);

	uint32_t Gain = _linear2sensorGain(NewGain);
	TRACE(IMX000_DEBUG, "%s: writting 0x%x to GAIN\n", __func__, Gain);

    result |= IMX000_LockRegHold(handle);
	result |= IMX000_WriteGain(handle, Gain);
	result |= IMX000_UnlockRegHold(handle);

	if (result != 0) {
		return RET_FAILURE;
	}

	pIMX000Ctx->AecCurGainLEF = _sensorGain2linear(Gain);
	*pSetGain = pIMX000Ctx->AecCurGainLEF;
	TRACE(IMX000_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
	return (result);
}

RESULT IMX000_IsiSetSEF1GainIss(IsiSensorHandle_t handle,
				float NewIntegrationTime, float NewGain,
				float *pSetGain, float *hdr_ratio)
{
	IMX000_Context_t *pIMX000Ctx = (IMX000_Context_t *)handle;
	RESULT result = RET_SUCCESS;
	if (!pIMX000Ctx) {
		return (RET_WRONG_HANDLE);
	}
    printf("--- [INFO] %s (enter)\n", __func__); 

	if (!pSetGain || !hdr_ratio)
		return (RET_NULL_POINTER);

	TRACE(IMX000_DEBUG, "%s: got NewGain %f\n",
	__func__, NewGain);

	uint32_t Gain = _linear2sensorGain(NewGain);
	TRACE(IMX000_DEBUG, "%s: writting 0x%x to GAIN1\n", __func__, Gain);

    result |= IMX000_LockRegHold(handle);
	result |= IMX000_WriteGain1(handle, Gain);
    result |= IMX000_UnlockRegHold(handle);

	pIMX000Ctx->AecCurGainSEF1 = _sensorGain2linear(Gain);
	*pSetGain = pIMX000Ctx->AecCurGainSEF1;

	TRACE(IMX000_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
	return (result);
}

RESULT IMX000_IsiSetSEF2GainIss(IsiSensorHandle_t handle,
				float NewIntegrationTime, float NewGain,
				float *pSetGain, float *hdr_ratio)
{
	IMX000_Context_t *pIMX000Ctx = (IMX000_Context_t *)handle;
	RESULT result = RET_SUCCESS;
    printf("--- [INFO] %s (enter)\n", __func__); 

	if (!pIMX000Ctx) {
		return (RET_WRONG_HANDLE);
	}

	if (!pSetGain || !hdr_ratio)
		return (RET_NULL_POINTER);
	
	TRACE(IMX000_DEBUG, "%s: got NewGain %f\n",
	__func__, NewGain);

	uint32_t Gain = _linear2sensorGain(NewGain);
	TRACE(IMX000_DEBUG, "%s: writting 0x%x to GAIN2\n", __func__, Gain);

    result |= IMX000_LockRegHold(handle);
	result |= IMX000_WriteGain2(handle, Gain);
    result |= IMX000_UnlockRegHold(handle);

	pIMX000Ctx->AecCurGainSEF2 = _sensorGain2linear(Gain);
	*pSetGain = pIMX000Ctx->AecCurGainSEF2;

	TRACE(IMX000_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
	return (result);
}

/* Integration Time get functions*/

RESULT IMX000_IsiGetIntegrationTimeIss(IsiSensorHandle_t handle,
				       float *pSetIntegrationTime)
{
	IMX000_Context_t *pIMX000Ctx = (IMX000_Context_t *)handle;
    printf("--- [INFO] %s (enter)\n", __func__); 

	if (!pIMX000Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);
	
	if (pIMX000Ctx->enableHdr)
		return IMX000_IsiGetSEF1IntegrationTimeIss(handle, pSetIntegrationTime);
	
	return IMX000_IsiGetLEFIntegrationTimeIss(handle, pSetIntegrationTime);
}

RESULT IMX000_IsiGetLEFIntegrationTimeIss(IsiSensorHandle_t handle,
				       float *pSetIntegrationTime)
{
	IMX000_Context_t *pIMX000Ctx = (IMX000_Context_t *)handle;
	RESULT result = RET_SUCCESS;
    printf("--- [INFO] %s (enter)\n", __func__); 


	if (!pIMX000Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);
	*pSetIntegrationTime = pIMX000Ctx->AecCurIntegrationTimeLEF;
	TRACE(IMX000_DEBUG, "%s - returning %f\n", __func__, pIMX000Ctx->AecCurIntegrationTimeLEF);
	return (result);
}

// HDR has not been tested yet
RESULT IMX000_IsiGetSEF1IntegrationTimeIss(IsiSensorHandle_t handle,
					   float *pSetIntegrationTime)
{
	IMX000_Context_t *pIMX000Ctx = (IMX000_Context_t *)handle;
	RESULT result = RET_SUCCESS;
    printf("--- [INFO] %s (enter)\n", __func__); 

	if (!pIMX000Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}
	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);
	*pSetIntegrationTime = pIMX000Ctx->AecCurIntegrationTimeSEF1;
	TRACE(IMX000_DEBUG, "%s - returning %f\n", __func__, pIMX000Ctx->AecCurIntegrationTimeSEF1);
	return (result);
}

// HDR has not been tested yet
RESULT IMX000_IsiGetSEF2IntegrationTimeIss(IsiSensorHandle_t handle,
					   float *pSetIntegrationTime)
{
	IMX000_Context_t *pIMX000Ctx = (IMX000_Context_t *)handle;
	RESULT result = RET_SUCCESS;
    printf("--- [INFO] %s (enter)\n", __func__); 

	if (!pIMX000Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}
	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);

	*pSetIntegrationTime = pIMX000Ctx->AecCurIntegrationTimeSEF2;
	TRACE(IMX000_DEBUG, "%s - returning %f\n", __func__, pIMX000Ctx->AecCurIntegrationTimeSEF2);
	return (result);
}

RESULT IMX000_IsiGetIntegrationTimeIncrementIss(IsiSensorHandle_t handle,
                                                float* pIncr) {
    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;
    printf("--- [INFO] %s (enter)\n", __func__); 

    RESULT result = RET_SUCCESS;

    if (!pIMX000Ctx) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (!pIncr) return (RET_NULL_POINTER);

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders
    //in
    // the application)
    *pIncr = pIMX000Ctx->AecIntegrationTimeIncrement;
    return (result);
}

/* Integration Time set functions*/

RESULT IMX000_IsiSetIntegrationTimeIss(IsiSensorHandle_t handle,
				       float NewIntegrationTime,
				       float *pSetIntegrationTime,
				       uint8_t *pNumberOfFramesToSkip,
				       float *hdr_ratio)
{
	IMX000_Context_t *pIMX000Ctx = (IMX000_Context_t *)handle;
    printf("--- [INFO] %s (enter)\n", __func__); 

	if (!pIMX000Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pIMX000Ctx->enableHdr) {
		return IMX000_IsiSetSEF1IntegrationTimeIss(
			handle, NewIntegrationTime, pSetIntegrationTime,
			pNumberOfFramesToSkip, hdr_ratio);
	}

	return IMX000_IsiSetLEFIntegrationTimeIss(
		handle, NewIntegrationTime, pSetIntegrationTime,
		pNumberOfFramesToSkip, hdr_ratio);
}

RESULT IMX000_IsiSetLEFIntegrationTimeIss(IsiSensorHandle_t handle,
				       float NewIntegrationTime,
				       float *pSetIntegrationTime,
				       uint8_t *pNumberOfFramesToSkip,
				       float *hdr_ratio)
{
	RESULT result = RET_SUCCESS;

    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;

    int shr = 0;
    uint32_t exp = 0;
    uint32_t new_vmax = 0;
    uint32_t vmax_updated = 0;
    uint32_t current_vmax = 0;
    uint32_t rhs2;
    if (!pIMX000Ctx) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (!pSetIntegrationTime || !pNumberOfFramesToSkip) {
        printf("%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (!pIMX000Ctx->Streaming) {
        TRACE(IMX000_ERROR, "%s: sensor not streaming\n", __func__);
        return RET_FAILURE;
    }
    exp = NewIntegrationTime / pIMX000Ctx->one_line_exp_time;

    TRACE(IMX000_DEBUG, "%s: set AEC_PK_EXPO=0x%05x\n", __func__, exp);

    if (fabs(NewIntegrationTime - pIMX000Ctx->AecCurIntegrationTimeLEF) > FLT_EPSILON) {

        if (pIMX000Ctx->enableHdr){
            if (pIMX000Ctx->cur_rhs1 == 0 || pIMX000Ctx->cur_rhs2 == 0) {
                TRACE(IMX000_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
                return (RET_WRONG_CONFIG);
            }

            rhs2 = pIMX000Ctx->cur_rhs2;
            new_vmax = IMX000_VMAX_HDR;
            exp = new_vmax - exp;
            exp = exp > rhs2 + IMX000_SHR0_RHS2_GAP ? exp : rhs2 + IMX000_SHR0_RHS2_GAP;
            exp = exp < new_vmax - IMX000_SHR0_FSC_GAP? exp : new_vmax - IMX000_SHR0_FSC_GAP;
            shr = exp;
		} else {
            if (exp > pIMX000Ctx->MaxIntegrationLine || exp == 0) {
                TRACE(IMX000_ERROR, "%s: Integration time %f (exp %u) out of range (%u)\n", __func__,
                    NewIntegrationTime, exp, pIMX000Ctx->MaxIntegrationLine);
                return RET_FAILURE;
            }
            result = IMX000_ReadVmax(handle, &current_vmax);

            if (pIMX000Ctx->original_vmax == 0) {                
                pIMX000Ctx->original_vmax = current_vmax;
            }

            if (pIMX000Ctx->original_vmax - IMX000_MIN_SHR >
                pIMX000Ctx->MaxIntegrationLine) {
                pIMX000Ctx->MaxIntegrationLine =
                    pIMX000Ctx->original_vmax - IMX000_MIN_SHR;
                TRACE(IMX000_DEBUG, "%s: set MaxIntegrationLine to %u\n", __func__,
                      pIMX000Ctx->MaxIntegrationLine);
                pIMX000Ctx->AecMaxIntegrationTime =
                    pIMX000Ctx->one_line_exp_time * pIMX000Ctx->MaxIntegrationLine;
                TRACE(IMX000_DEBUG, "%s: set AecMaxIntegrationTime to %f\n", __func__,
                      pIMX000Ctx->AecMaxIntegrationTime);
            }

            shr = current_vmax - exp;

            if (shr < IMX000_MIN_SHR) {
                new_vmax = MIN(exp + IMX000_MIN_SHR,
                            pIMX000Ctx->MaxIntegrationLine + IMX000_MIN_SHR);
                shr = IMX000_MIN_SHR;
                vmax_updated = 1;
            } else if (shr > IMX000_MIN_SHR &&
                    current_vmax > pIMX000Ctx->original_vmax) {
                new_vmax = MAX(current_vmax - shr + IMX000_MIN_SHR,
                            pIMX000Ctx->original_vmax);
                shr = new_vmax - exp;
                vmax_updated = 1;
            } else {
                new_vmax = current_vmax;
            }
        }

        result |= IMX000_LockRegHold(handle);
        if (vmax_updated && pIMX000Ctx->unlimit_fps && !pIMX000Ctx->enableHdr) {
            result = IMX000_WriteVmax(handle, new_vmax);
        }

        TRACE(IMX000_DEBUG, "%s - writing 0x%x to SHR0\n", __func__, shr);
        result |= IMX000_WriteShr0(handle, shr);
        result |= IMX000_UnlockRegHold(handle);

        float configuredIntegrationTime =
            (new_vmax - shr) * pIMX000Ctx->one_line_exp_time;
        pIMX000Ctx->OldIntegrationTime = configuredIntegrationTime;
        pIMX000Ctx->AecCurIntegrationTimeLEF = configuredIntegrationTime;

        *pNumberOfFramesToSkip = 1U;
    } else {
        *pNumberOfFramesToSkip = 0U;  // no frame skip
    }

    *pSetIntegrationTime = pIMX000Ctx->AecCurIntegrationTimeLEF;
    TRACE(IMX000_DEBUG, "%s: Ti=%f\n", __func__, *pSetIntegrationTime);
    return (result);
}

RESULT IMX000_IsiSetSEF1IntegrationTimeIss(IsiSensorHandle_t handle,
					   float NewIntegrationTime,
					   float *pSetIntegrationTimeSEF1,
					   uint8_t *pNumberOfFramesToSkip,
					   float *hdr_ratio)
{
	IMX000_Context_t *pIMX000Ctx = (IMX000_Context_t *)handle;
	RESULT result = RET_SUCCESS;
	uint32_t exp = 0;
	uint32_t rhs1;
    printf("--- [INFO] %s (enter)\n", __func__); 

	if (!pIMX000Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pIMX000Ctx->cur_rhs1 == 0 || pIMX000Ctx->cur_rhs2 == 0) {
		TRACE(IMX000_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMX000Ctx->cur_rhs1;

	if (!pSetIntegrationTimeSEF1 || !pNumberOfFramesToSkip) {
		printf("%s: Invalid parameter (NULL pointer detected)\n",
		       __func__);
		return (RET_NULL_POINTER);
	}
	TRACE(IMX000_DEBUG, "%s: NewIntegrationTime = %f\n", __func__, NewIntegrationTime);

	exp = (NewIntegrationTime / pIMX000Ctx->one_line_exp_time);
	TRACE(IMX000_DEBUG, "%s - calculated IT in rows = 0x%x\n", __func__, exp);

	if (fabs(NewIntegrationTime - pIMX000Ctx->AecCurIntegrationTimeSEF1) > FLT_EPSILON) {
		exp = rhs1 - exp;
		exp = exp > IMX000_SHR1_MIN_GAP ? exp : IMX000_SHR1_MIN_GAP;
		exp = exp < rhs1 - IMX000_SHR1_RHS1_GAP ? exp : rhs1 - IMX000_SHR1_RHS1_GAP;
		TRACE(IMX000_DEBUG, "%s - writing 0x%x to SHR1\n", __func__, exp);

		result |= IMX000_LockRegHold(handle);
		result |= IMX000_WriteShr1(handle, exp);
		result |= IMX000_UnlockRegHold(handle);

		pIMX000Ctx->AecCurIntegrationTimeSEF1 = (rhs1 - exp) * pIMX000Ctx->one_line_exp_time; // in sec
		*pNumberOfFramesToSkip = 1U;
	} else {
		*pNumberOfFramesToSkip = 0U;
	}

	*pSetIntegrationTimeSEF1 = pIMX000Ctx->AecCurIntegrationTimeSEF1;

	TRACE(IMX000_DEBUG, "%s: Ti=%f\n", __func__, *pSetIntegrationTimeSEF1);
	return (result);
}

RESULT IMX000_IsiSetSEF2IntegrationTimeIss(IsiSensorHandle_t handle,
					   float NewIntegrationTime,
					   float *pSetIntegrationTimeSEF2,
					   uint8_t *pNumberOfFramesToSkip,
					   float *hdr_ratio)
{
	IMX000_Context_t *pIMX000Ctx = (IMX000_Context_t *)handle;
	RESULT result = RET_SUCCESS;
	uint32_t exp = 0;
	uint32_t rhs1;
	uint32_t rhs2;
    printf("--- [INFO] %s (enter)\n", __func__); 

	if (pIMX000Ctx->cur_rhs1 == 0 || pIMX000Ctx->cur_rhs2 == 0) {
		printf("%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMX000Ctx->cur_rhs1;
	rhs2 = pIMX000Ctx->cur_rhs2;

	if (!pIMX000Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (!pSetIntegrationTimeSEF2 || !pNumberOfFramesToSkip) {
		printf("%s: Invalid parameter (NULL pointer detected)\n",
		       __func__);
		return (RET_NULL_POINTER);
	}
	TRACE(IMX000_DEBUG, "%s: NewIntegrationTime = %f\n", __func__, NewIntegrationTime);

	exp = (NewIntegrationTime / pIMX000Ctx->one_line_exp_time);
	TRACE(IMX000_DEBUG, "%s - calculated IT in rows = 0x%x\n", __func__, exp);

	if (fabs(NewIntegrationTime - pIMX000Ctx->AecCurIntegrationTimeSEF2) > FLT_EPSILON) {
		exp = rhs2 - exp;
		exp = exp > rhs1 + IMX000_SHR2_RHS1_GAP ? exp : rhs1 + IMX000_SHR2_RHS1_GAP;
		exp = exp < rhs2 - IMX000_SHR2_RHS2_GAP ? exp : rhs2 - IMX000_SHR2_RHS2_GAP;
		TRACE(IMX000_DEBUG, "%s - writing 0x%x to SHR2\n", __func__, exp);

		result |= IMX000_LockRegHold(handle);
		result |= IMX000_WriteShr2(handle, exp);
		result |= IMX000_UnlockRegHold(handle);

		pIMX000Ctx->AecCurIntegrationTimeSEF2 = (rhs2 - exp) * pIMX000Ctx->one_line_exp_time; // in sec
		*pNumberOfFramesToSkip = 1U;
	} else {
		*pNumberOfFramesToSkip = 0U;
	}

	*pSetIntegrationTimeSEF2 = pIMX000Ctx->AecCurIntegrationTimeSEF2;

	TRACE(IMX000_DEBUG, "%s: Ti=%f\n", __func__, *pSetIntegrationTimeSEF2);
	return (result);
}

RESULT IMX000_CalculateHDRExposures(IsiSensorHandle_t handle, float NewIntegrationTime, float NewGain,
                                    float *o_long_it, float *o_short_it, float *o_very_short_it,
                                    float *o_long_gain, float *o_short_gain, float *o_very_short_gain,
                                    float *hdr_ratio) {
    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    float long_it = 0.0;
	float short_it = 0.0;
	float very_short_it = 0.0;
	float long_exp_val = 0.0;
	float short_exp_val = 0.0;
	float very_short_exp_val = 0.0;
	float long_gain = 1;
	float short_gain = 1;
	float very_short_gain = 1;
	bool calculate_gain = false;
	uint32_t rhs1;
	uint32_t rhs2;

    if (pIMX000Ctx == NULL || o_long_it == NULL || o_short_it == NULL ||
        o_very_short_it == NULL || o_long_gain == NULL || o_short_gain == NULL ||
        o_very_short_gain == NULL || hdr_ratio == NULL) {
        printf("%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

	if (pIMX000Ctx->cur_rhs1 == 0 || pIMX000Ctx->cur_rhs2 == 0) {
		TRACE(IMX000_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMX000Ctx->cur_rhs1;
	rhs2 = pIMX000Ctx->cur_rhs2;

    TRACE(IMX000_DEBUG, "%s: hdr_ratio[0] = LS Ratio = %f, hdr_ratio[1] = VS Ratio = %f\n", 
    __func__, hdr_ratio[0], hdr_ratio[1]);
    
    // Sometimes there is no actual input gain. In that case, we will read it from the sensor
    if (NewGain == 0) {
        TRACE(IMX000_DEBUG, "%s: Input NewGain is 0, reading gain from sensor\n", __func__);
        result = IMX000_IsiGetSEF1GainIss(handle, &NewGain);
        if (result != RET_SUCCESS) {
            return result;
        }
        calculate_gain = true;
    }

    // Same for integration time
    if (NewIntegrationTime == 0) {
        TRACE(IMX000_DEBUG, "%s: Input NewIntegrationTime is 0, reading integration time from sensor\n", __func__);
        result = IMX000_IsiGetSEF1IntegrationTimeIss(handle, &NewIntegrationTime);
        if (result != RET_SUCCESS) {
            return result;
        }
        calculate_gain = true;
    }

    // assume gain is 1 and see if ratio can be achieved with integration time
    long_it 		= NewIntegrationTime * hdr_ratio[0];
    short_it 		= NewIntegrationTime;
    very_short_it 	= NewIntegrationTime / hdr_ratio[1];
    
    TRACE(IMX000_DEBUG, "%s: requested IT long: %f, short: %f, very_short: %f\n", 
    __func__, long_it, short_it, very_short_it);
    long_exp_val 		= long_it / pIMX000Ctx->one_line_exp_time;
    short_exp_val 		= short_it / pIMX000Ctx->one_line_exp_time;
    very_short_exp_val 	= very_short_it / pIMX000Ctx->one_line_exp_time;

    TRACE(IMX000_DEBUG, "%s: requested IT in lines long: %f, short: %f, very_short: %f\n", 
    __func__, long_exp_val, short_exp_val, very_short_exp_val);
    long_exp_val 		= IMX000_VMAX_HDR - long_exp_val;
    short_exp_val 		= rhs1 - short_exp_val;
    very_short_exp_val 	= rhs2 - very_short_exp_val;

    TRACE(IMX000_DEBUG, "%s: requested IT in shr long: %f, short: %f, very_short: %f\n", 
    __func__, long_exp_val, short_exp_val, very_short_exp_val);
    if(long_exp_val < rhs2 + IMX000_SHR0_RHS2_GAP) {
        long_exp_val = rhs2 + IMX000_SHR0_RHS2_GAP;
        long_it = (IMX000_VMAX_HDR - long_exp_val) * pIMX000Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX000_DEBUG, "%s: long_exp_val is too long, set to %u, new long_it = %f\n",
        __func__, rhs2 + IMX000_SHR0_RHS2_GAP, long_it);
    } else if(long_exp_val > IMX000_VMAX_HDR - IMX000_SHR0_FSC_GAP) {
        long_exp_val = IMX000_VMAX_HDR - IMX000_SHR0_FSC_GAP;
        long_it = (IMX000_VMAX_HDR - long_exp_val) * pIMX000Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX000_DEBUG, "%s: long_exp_val is too short, set to %u, new long_it = %f\n",
        __func__, IMX000_VMAX_HDR - IMX000_SHR0_FSC_GAP, long_it);
    }
    if(short_exp_val < IMX000_SHR1_MIN_GAP) {
        short_exp_val = IMX000_SHR1_MIN_GAP;
        short_it = (rhs1 - short_exp_val) * pIMX000Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX000_DEBUG, "%s: short_exp_val is too long, set to %u, new short_it = %f\n",
        __func__, IMX000_SHR1_MIN_GAP, short_it);
    } else if(short_exp_val > rhs1 - IMX000_SHR1_RHS1_GAP) {
        short_exp_val = rhs1 - IMX000_SHR1_RHS1_GAP;
        short_it = (rhs1 - short_exp_val) * pIMX000Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX000_DEBUG, "%s: short_exp_val is too short, set to %u, new short_it = %f\n",
        __func__, rhs1 - IMX000_SHR1_RHS1_GAP, short_it);
    }
    if(very_short_exp_val < rhs1 + IMX000_SHR2_RHS1_GAP) {
        very_short_exp_val = rhs1 + IMX000_SHR2_RHS1_GAP;
        very_short_it = (rhs2 - very_short_exp_val) * pIMX000Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX000_DEBUG, "%s: very_short_exp_val is too long, set to %u, new very_short_it = %f\n",
        __func__, rhs2 + IMX000_SHR2_RHS1_GAP, very_short_it);
    } else if(very_short_exp_val > rhs2 - IMX000_SHR2_RHS2_GAP) {
        very_short_exp_val = rhs2 - IMX000_SHR2_RHS2_GAP;
        very_short_it = (rhs2 - very_short_exp_val) * pIMX000Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX000_DEBUG, "%s: very_short_exp_val is too short, set to %u, new very_short_it = %f\n",
        __func__, rhs2 - IMX000_SHR2_RHS2_GAP, very_short_it);
    }

    // need to use gain to achive ratio / requested gain update
    if(calculate_gain || NewGain != pIMX000Ctx->AecCurGainSEF1) {
        long_gain = (short_it * NewGain * hdr_ratio[0]) / long_it;
        short_gain = NewGain;
        very_short_gain = (short_it * NewGain) / (very_short_it * hdr_ratio[1]);
        TRACE(IMX000_DEBUG, "%s: calculated gain: long: %f, short: %f, very_short: %f\n",
        __func__, long_gain, short_gain, very_short_gain);
    }

    *o_long_it = long_it;
    *o_short_it = short_it;
    *o_very_short_it = very_short_it;
    *o_long_gain = long_gain;
    *o_short_gain = short_gain;
    *o_very_short_gain = very_short_gain;

    return RET_SUCCESS;
}

RESULT IMX000_IsiExposureControlIss(IsiSensorHandle_t handle, float NewGain,
                                    float NewIntegrationTime,
                                    uint8_t* pNumberOfFramesToSkip,
                                    float* pSetGain, float* pSetIntegrationTime,
                                    float* hdr_ratio) {
    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;    
    RESULT result = RET_SUCCESS;
    float long_it = 0.0;
	float short_it = 0.0;
	float very_short_it = 0.0;
	float long_gain = 1;
	float short_gain = 1;
	float very_short_gain = 1;
    uint32_t hmax;
    TRACE(IMX000_INFO, "%s: enter with NewIntegrationTime: %f, NewGain: %f\n",
        __func__, NewIntegrationTime, NewGain);
    if (pIMX000Ctx == NULL) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }        
    if ((pNumberOfFramesToSkip == NULL) || (pSetGain == NULL) ||
        (pSetIntegrationTime == NULL)) {
        printf("%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    // HDR has not been tested yet
    if (pIMX000Ctx->enableHdr) {
        result = IMX000_ReadRHS1(handle, &pIMX000Ctx->cur_rhs1);
        result |= IMX000_ReadRHS2(handle, &pIMX000Ctx->cur_rhs2);
        result |= IMX000_ReadHmax(handle, &hmax);
        if (result != RET_SUCCESS) {
            TRACE(IMX000_ERROR, "%s: Read RHS1, RHS2 or HMAX failed\n", __func__);
            return result;
        }

        pIMX000Ctx->SensorMode.ae_info.max_integration_time = pIMX000Ctx->cur_rhs1 - IMX000_SHR1_MIN_GAP;
        pIMX000Ctx->SensorMode.ae_info.one_line_exp_time_ns = (uint32_t)(((float)hmax / IMX000_PIXEL_CLK_RATE) * MICRO_2_NANO);
        pIMX000Ctx->one_line_exp_time =
        (float)(pIMX000Ctx->SensorMode.ae_info.one_line_exp_time_ns) / 1000000000;

        result = IMX000_CalculateHDRExposures(handle, NewIntegrationTime, NewGain,
                                    &long_it, &short_it, &very_short_it,
                                    &long_gain, &short_gain, &very_short_gain,
                                    hdr_ratio);
        if (result != RET_SUCCESS) {
            TRACE(IMX000_ERROR, "%s: CalculateHDRExposures failed\n", __func__);
            return result;
        }

		result = IMX000_IsiSetLEFIntegrationTimeIss(handle, long_it,
							pSetIntegrationTime,
							pNumberOfFramesToSkip,
							hdr_ratio);
		result |= IMX000_IsiSetLEFGainIss(handle, long_gain, pSetGain, hdr_ratio);        
		result |= IMX000_IsiSetSEF1IntegrationTimeIss(
			handle, short_it, pSetIntegrationTime,
			pNumberOfFramesToSkip, hdr_ratio);

		result |= IMX000_IsiSetSEF1GainIss(handle, NewIntegrationTime,
						  short_gain, pSetGain, hdr_ratio);
		result |= IMX000_IsiSetSEF2IntegrationTimeIss(
			handle, very_short_it, pSetIntegrationTime,
			pNumberOfFramesToSkip, hdr_ratio);
		result |= IMX000_IsiSetSEF2GainIss(handle, NewIntegrationTime,
						  very_short_gain, pSetGain, hdr_ratio);

        // Recalculate `io_hdr_ratio` according to the set values
        hdr_ratio[0] = (long_it * long_gain) / (short_it * short_gain);
        hdr_ratio[1] = (short_it * short_gain) / (very_short_it * very_short_gain);

        // Set the output values to SEF1 values
        *pSetGain = short_gain;
        *pSetIntegrationTime = short_it;

        TRACE(IMX000_DEBUG, "%s: actual hdr_ratio[0] = LS Ratio = %f, hdr_ratio[1] = VS Ratio = %f\n",
        __func__, hdr_ratio[0], hdr_ratio[1]);
    } else {
        result |= IMX000_IsiSetLEFIntegrationTimeIss(handle, NewIntegrationTime,
                                                pSetIntegrationTime,
                                                pNumberOfFramesToSkip, hdr_ratio);
        result |= IMX000_IsiSetLEFGainIss(handle, NewGain, pSetGain, hdr_ratio);
    }
    return result;
}

RESULT IMX000_IsiExposureControlExpandedIss(
    IsiSensorHandle_t handle, float NewGain, float NewIntegrationTime,
    float NewIris, uint8_t* pNumberOfFramesToSkip, float* pSetGain,
    float* pSetIntegrationTime, float* pSetIris, float* hdr_ratio) {
    printf("--- [INFO] %s (enter)\n", __func__); 

    if (pSetIris) {
        IMX000_IsiSetIrisIss(handle, NewIris);
        *pSetIris = NewIris;
    }

    return IMX000_IsiExposureControlIss(handle, NewGain, NewIntegrationTime,
                                        pNumberOfFramesToSkip, pSetGain,
                                        pSetIntegrationTime, hdr_ratio);
}

RESULT IMX000_IsiGetCurrentExposureIss(IsiSensorHandle_t handle,
                                       float* pSetGain,
                                       float* pSetIntegrationTime) {
    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    printf("--- [INFO] %s (enter)\n", __func__); 

    if (pIMX000Ctx == NULL) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pSetGain == NULL) || (pSetIntegrationTime == NULL))
        return (RET_NULL_POINTER);

    if (pIMX000Ctx->enableHdr) {
		*pSetGain = pIMX000Ctx->AecCurGainSEF1;
		*pSetIntegrationTime = pIMX000Ctx->AecCurIntegrationTimeSEF1;
	} else {
		*pSetGain = pIMX000Ctx->AecCurGainLEF;
		*pSetIntegrationTime = pIMX000Ctx->AecCurIntegrationTimeLEF;
	}

    return (result);
}

RESULT IMX000_IsiGetFpsIss(IsiSensorHandle_t handle, uint32_t* pFps) {
    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    printf("--- [INFO] %s (enter)\n", __func__); 

    if (pIMX000Ctx == NULL) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    *pFps = pIMX000Ctx->CurrFps;

    return (result);
}

RESULT IMX000_IsiSetFpsIss(IsiSensorHandle_t handle, uint32_t Fps) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT IMX000_IsiSetFlickerFpsIss(IsiSensorHandle_t handle, uint32_t flickerMode) {
    RESULT result = RET_SUCCESS;
    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;
    printf("--- [INFO] %s (enter)\n", __func__); 

    TRACE(IMX000_DEBUG, "%s: set sensor flickerMode = %d\n", __func__, flickerMode);

    if (!pIMX000Ctx) {
        return RET_NULL_POINTER;
    }
    return result;
}

RESULT IMX000_IsiGetAutoFpsInfoIss(IsiSensorHandle_t handle,
                                   IsiAutoFps_t* pAutoFpsInfo) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT IMX000_IsiGetStartEvIss(IsiSensorHandle_t handle, uint64_t* pStartEv) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT IMX000_IsiGetIspStatusIss(IsiSensorHandle_t handle,
                                 IsiIspStatus_t* pIspStatus) {
    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;
    printf("--- [INFO] %s (enter)\n", __func__); 
    if (pIMX000Ctx == NULL || pIMX000Ctx->IsiCtx.HalHandle == NULL) {
        return RET_WRONG_HANDLE;
    }

    pIspStatus->useSensorAE = false;
    pIspStatus->useSensorBLC = false;
    pIspStatus->useSensorAWB = false;

    return RET_SUCCESS;
}

RESULT IMX000_IsiSetTpgIss(IsiSensorHandle_t handle, IsiTpg_t Tpg) {
    RESULT result = RET_SUCCESS;
    printf("--- [INFO] %s (enter)\n", __func__); 
    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;
    if (pIMX000Ctx == NULL || pIMX000Ctx->IsiCtx.HalHandle == NULL) {
        return RET_NULL_POINTER;
    }

    if (pIMX000Ctx->Configured != BOOL_TRUE) return RET_WRONG_STATE;

    if (Tpg.enable == 0) {
        result = IMX000_IsiWriteRegIss(handle, 0x3253, 0x00);
    } else {
        result = IMX000_IsiWriteRegIss(handle, 0x3253, 0x80);
    }

    pIMX000Ctx->TestPattern = Tpg.enable;

    return (result);
}

RESULT IMX000_IsiGetTpgIss(IsiSensorHandle_t handle, IsiTpg_t* Tpg) {
    RESULT result = RET_SUCCESS;
    uint32_t value = 0;
    printf("--- [INFO] %s (enter)\n", __func__); 

    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;
    if (pIMX000Ctx == NULL || pIMX000Ctx->IsiCtx.HalHandle == NULL ||
        Tpg == NULL) {
        return RET_NULL_POINTER;
    }

    if (pIMX000Ctx->Configured != BOOL_TRUE) return RET_WRONG_STATE;

    if (!IMX000_IsiReadRegIss(handle, 0x5081, &value)) {
        Tpg->enable = ((value & 0x80) != 0) ? 1 : 0;
        if (Tpg->enable) {
            Tpg->pattern = (0xff & value);
        }
        pIMX000Ctx->TestPattern = Tpg->enable;
    }

    return (result);
}

RESULT IMX000_IsiFocusCreateIss(IsiSensorHandle_t handle) {
    return RET_SUCCESS;
}

RESULT IMX000_IsiFocusReleaseIss(IsiSensorHandle_t handle) {
    return RET_SUCCESS;
}

RESULT IMX000_IsiFocusGetCalibrateIss(IsiSensorHandle_t handle,
                                      IsiFocusCalibAttr_t* pFocusCalib) {
    return RET_SUCCESS;
}

RESULT IMX000_IsiFocusSetIss(IsiSensorHandle_t handle, IsiFocusPos_t* pPos) {
    return RET_SUCCESS;
}

RESULT IMX000_IsiFocusGetIss(IsiSensorHandle_t handle, IsiFocusPos_t* pPos) {
    return RET_SUCCESS;
}

// Taken from the IMX681 driver
RESULT IMX000_IsiSetAgainDgainIss(IsiSensorHandle_t handle,
                                  IsiUserGain_t Gain) {
    RESULT result = RET_SUCCESS;
    uint32_t Again = 0, Dgain = 0;
    printf("--- [INFO] %s (enter)\n", __func__); 

    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;
    if (pIMX000Ctx == NULL) {
        TRACE(IMX000_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((Gain.again < 1) | (Gain.again > 16)) {
        TRACE(IMX000_ERROR, "%s: Invalid sensor again\n", __func__);
        return (RET_OUTOFRANGE);
    }
    // Again = (uint32_t)(1024 - (1024/Gain.again));
    Again = (uint32_t)(((2048 * Gain.again) - 2048) / Gain.again);

    result = IMX000_IsiWriteRegIss(handle, 0x309c, (Again & 0x0000FF));
    result = IMX000_IsiWriteRegIss(handle, 0x309d, (Again & 0x00FF00) >> 8);

    if ((Gain.dgain < 1) | (Gain.dgain > 16)) {
        TRACE(IMX000_ERROR, "%s: Invalid sensor dgain\n", __func__);
        return (RET_OUTOFRANGE);
    }
    Dgain = Gain.dgain * 256;

    result = IMX000_IsiWriteRegIss(handle, 0x308c, (Dgain & 0x0000FF));
    result = IMX000_IsiWriteRegIss(handle, 0x308d, (Dgain & 0x00FF00) >> 8);

    printf("--- [INFO] %s (exit)\n", __func__); 
    return (result);
}

RESULT IMX000_IsiGetIrisIss( IsiSensorHandle_t handle, float *pSetIris ) {
    RESULT result = RET_SUCCESS;

    printf("--- [INFO] %s (enter)\n", __func__); 

    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;
    if (pIMX000Ctx == NULL) {
        TRACE(IMX000_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    //TODO implement getting iris here!
    *pSetIris = 1.0;

    printf("--- [INFO] %s (exit)\n", __func__); 
    return (result);
}

RESULT IMX000_IsiSetIrisIss( IsiSensorHandle_t handle, float NewIris) {
    RESULT result = RET_SUCCESS;
    printf("--- [INFO] %s (enter)\n", __func__); 

    IMX000_Context_t* pIMX000Ctx = (IMX000_Context_t*)handle;
    if (pIMX000Ctx == NULL) {
        TRACE(IMX000_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (NewIris < IMX000_IRIS_MIN_VAL || NewIris > IMX000_IRIS_MAX_VAL) {
        TRACE(IMX000_ERROR, "%s: Invalid Iris %f\n", __func__, NewIris);
    }

    //TODO implement setting iris here!

    printf("--- [INFO] %s (exit)\n", __func__); 
    return (result);
}

RESULT IMX000_IsiGetSensorIss(IsiSensor_t* pIsiSensor) {
    RESULT result = RET_SUCCESS;
    static const char SensorName[16] = "IMX000";
    printf("--- [INFO] %s (enter)\n", __func__); 

    if (pIsiSensor != NULL) {
        pIsiSensor->pszName = SensorName;
        pIsiSensor->pIsiCreateIss = IMX000_IsiCreateIss;
        pIsiSensor->pIsiReleaseIss = IMX000_IsiReleaseIss;
        pIsiSensor->pIsiReadRegIss = IMX000_IsiReadRegIss;
        pIsiSensor->pIsiWriteRegIss = IMX000_IsiWriteRegIss;
        pIsiSensor->pIsiGetModeIss = IMX000_IsiGetModeIss;
        pIsiSensor->pIsiSetModeIss = IMX000_IsiSetModeIss;
        pIsiSensor->pIsiEnumModeIss = IMX000_IsiEnumModeIss;
        pIsiSensor->pIsiGetCapsIss = IMX000_IsiGetCapsIss;
        pIsiSensor->pIsiSetupIss = IMX000_IsiSetupIss;
        pIsiSensor->pIsiCheckConnectionIss = IMX000_IsiCheckConnectionIss;
        pIsiSensor->pIsiGetRevisionIss = IMX000_IsiGetRevisionIss;
        pIsiSensor->pIsiSetStreamingIss = IMX000_IsiSetStreamingIss;

        /* AEC functions */
        pIsiSensor->pIsiGetGainLimitsIss = 					IMX000_IsiGetGainLimitsIss;
		pIsiSensor->pIsiGetIrisLimitsIss = 					IMX000_IsiGetIrisLimitsIss;
		pIsiSensor->pIsiSetIrisLimitsIss = 					IMX000_IsiSetIrisLimitsIss;
		pIsiSensor->pIsiGetIntegrationTimeLimitsIss =		IMX000_IsiGetIntegrationTimeLimitsIss;

		pIsiSensor->pIsiExposureControlIss =				IMX000_IsiExposureControlIss;
		pIsiSensor->pIsiExposureControlExpandedIss =		IMX000_IsiExposureControlExpandedIss;
		pIsiSensor->pIsiSetIntegrationTimeIss =				IMX000_IsiSetIntegrationTimeIss;

		pIsiSensor->pIsiGetLongIntegrationTimeIss =			IMX000_IsiGetLEFIntegrationTimeIss;
		pIsiSensor->pIsiGetIntegrationTimeIss =				IMX000_IsiGetIntegrationTimeIss;
		pIsiSensor->pIsiGetVSIntegrationTimeIss =			IMX000_IsiGetSEF2IntegrationTimeIss;

		pIsiSensor->pIsiGetLongGainIss = 					IMX000_IsiGetLEFGainIss;
		pIsiSensor->pIsiGetGainIss = 						IMX000_IsiGetGainIss;
		pIsiSensor->pIsiGetVSGainIss = 						IMX000_IsiGetSEF2GainIss;

		pIsiSensor->pIsiGetGainIncrementIss =				IMX000_IsiGetGainIncrementIss;
		pIsiSensor->pIsiGetIrisIncrementIss =				IMX000_IsiGetIrisIncrementIss;
		pIsiSensor->pIsiGetIntegrationTimeIncrementIss =	IMX000_IsiGetIntegrationTimeIncrementIss;
		pIsiSensor->pIsiSetGainIss = 						IMX000_IsiSetGainIss;
		pIsiSensor->pIsiGetFpsIss = 						IMX000_IsiGetFpsIss;
		pIsiSensor->pIsiSetFpsIss = 						IMX000_IsiSetFpsIss;
        pIsiSensor->pIsiSetFlickerFpsIss = 					IMX000_IsiSetFlickerFpsIss;
        pIsiSensor->pIsiUnlimitFpsIss =						IMX000_IsiUnlimitFpsIss;
        pIsiSensor->pIsiLimitFpsIss =						IMX000_IsiLimitFpsIss;
		pIsiSensor->pIsiGetAutoFpsInfoIss = 				IMX000_IsiGetAutoFpsInfoIss;
		pIsiSensor->pIsiGetStartEvIss = 					IMX000_IsiGetStartEvIss;
        pIsiSensor->pIsiGetIrisIss =						IMX000_IsiGetIrisIss;
        pIsiSensor->pIsiSetIrisIss =						IMX000_IsiSetIrisIss;

        /* SENSOR ISP */
        pIsiSensor->pIsiGetIspStatusIss = IMX000_IsiGetIspStatusIss;
        // pIsiSensor->pIsiSetBlcIss                       =
        // IMX000_IsiSetBlcIss; pIsiSensor->pIsiSetWBIss = IMX000_IsiSetWBIss;

        /* SENSOR OTHER FUNC*/
        pIsiSensor->pIsiSetPowerIss = IMX000_IsiSetPowerIss;
        pIsiSensor->pIsiSetTpgIss = IMX000_IsiSetTpgIss;
        pIsiSensor->pIsiGetTpgIss = IMX000_IsiGetTpgIss;
        // pIsiSensor->pIsiGetExpandCurveIss               =
        // IMX000_IsiGetExpandCurveIss; pIsiSensor->pIsiGetCompressCurveIss =
        // IMX000_IsiGetCompressCurveIss; pIsiSensor->pIsiExtendFuncIss =
        // IMX000_IsiExtendFuncIss; pIsiSensor->pIsiGetOtpDataIss =
        // IMX000_IsiGetOtpDataIss;

        /* AF */
        pIsiSensor->pIsiFocusCreateIss = IMX000_IsiFocusCreateIss;
        pIsiSensor->pIsiFocusReleaseIss = IMX000_IsiFocusReleaseIss;
        pIsiSensor->pIsiFocusGetCalibrateIss = IMX000_IsiFocusGetCalibrateIss;
        pIsiSensor->pIsiFocusSetIss = IMX000_IsiFocusSetIss;
        pIsiSensor->pIsiFocusGetIss = IMX000_IsiFocusGetIss;
        pIsiSensor->pIsiSetAgainDgainIss = IMX000_IsiSetAgainDgainIss;

    } else {
        result = RET_NULL_POINTER;
    }

    printf("--- [INFO] %s (exit)\n", __func__); 
    return (result);
}

/*****************************************************************************
 * each sensor driver need declare this struct for isi load
 *****************************************************************************/
IsiCamDrvConfig_t IsiCamDrvConfig = {
    // .CameraDriverID = 0x9012,
    .pIsiHalEnumModeIss = IMX000_IsiHalEnumModeIss,
    .pIsiGetSensorIss = IMX000_IsiGetSensorIss,
};
