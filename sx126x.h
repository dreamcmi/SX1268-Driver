/**
 * @file      sx126x.h
 *
 * @brief     SX126x radio driver definition
 *
 * Revised BSD License
 * Copyright Semtech Corporation 2020. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SX126X_H
#define SX126X_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/**
 * @brief Maximum value for parameter timeout_in_rtc_step in both functions \ref sx126x_set_rx_with_timeout_in_rtc_step
 * and \ref sx126x_set_tx_with_timeout_in_rtc_step
 */
#define SX126X_MAX_TIMEOUT_IN_RTC_STEP 0x00FFFFFE

/**
 * @brief  Maximum value for parameter timeout_in_ms in both functions \ref sx126x_set_rx and \ref sx126x_set_tx
 */
#define SX126X_MAX_TIMEOUT_IN_MS ( SX126X_MAX_TIMEOUT_IN_RTC_STEP / 64 )

/**
 * @brief Timeout parameter in \ref sx126x_set_rx_with_timeout_in_rtc_step to set the chip in reception until a
 * reception occurs
 */
#define SX126X_RX_SINGLE_MODE 0x00000000

/**
 * @brief Timeout parameter in \ref sx126x_set_rx_with_timeout_in_rtc_step to launch a continuous reception
 */
#define SX126X_RX_CONTINUOUS 0x00FFFFFF

#define SX126X_CHIP_MODES_POS ( 4U )
#define SX126X_CHIP_MODES_MASK ( 0x07UL << SX126X_CHIP_MODES_POS )

#define SX126X_CMD_STATUS_POS ( 1U )
#define SX126X_CMD_STATUS_MASK ( 0x07UL << SX126X_CMD_STATUS_POS )

#define SX126X_GFSK_RX_STATUS_PKT_SENT_POS ( 0U )
#define SX126X_GFSK_RX_STATUS_PKT_SENT_MASK ( 0x01UL << SX126X_GFSK_RX_STATUS_PKT_SENT_POS )

#define SX126X_GFSK_RX_STATUS_PKT_RECEIVED_POS ( 1U )
#define SX126X_GFSK_RX_STATUS_PKT_RECEIVED_MASK ( 0x01UL << SX126X_GFSK_RX_STATUS_PKT_RECEIVED_POS )

#define SX126X_GFSK_RX_STATUS_ABORT_ERROR_POS ( 2U )
#define SX126X_GFSK_RX_STATUS_ABORT_ERROR_MASK ( 0x01UL << SX126X_GFSK_RX_STATUS_ABORT_ERROR_POS )

#define SX126X_GFSK_RX_STATUS_LENGTH_ERROR_POS ( 3U )
#define SX126X_GFSK_RX_STATUS_LENGTH_ERROR_MASK ( 0x01UL << SX126X_GFSK_RX_STATUS_LENGTH_ERROR_POS )

#define SX126X_GFSK_RX_STATUS_CRC_ERROR_POS ( 4U )
#define SX126X_GFSK_RX_STATUS_CRC_ERROR_MASK ( 0x01UL << SX126X_GFSK_RX_STATUS_CRC_ERROR_POS )

#define SX126X_GFSK_RX_STATUS_ADRS_ERROR_POS ( 5U )
#define SX126X_GFSK_RX_STATUS_ADRS_ERROR_MASK ( 0x01UL << SX126X_GFSK_RX_STATUS_ADRS_ERROR_POS )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief SX126X APIs return status enumeration definition
 * SX126X API 返回状态枚举定义
 */
typedef enum sx126x_status_e
{
    SX126X_STATUS_OK = 0,
    SX126X_STATUS_UNSUPPORTED_FEATURE,
    SX126X_STATUS_UNKNOWN_VALUE,
    SX126X_STATUS_ERROR,
} sx126x_status_t;

/**
 * @brief SX126X sleep mode configurations definition
 * SX126X 睡眠模式配置定义
 */
typedef enum sx126x_sleep_cfgs_e
{
    SX126X_SLEEP_CFG_COLD_START = ( 0 << 2 ),
    SX126X_SLEEP_CFG_WARM_START = ( 1 << 2 ),
} sx126x_sleep_cfgs_t;

/**
 * @brief SX126X standby modes enumeration definition
 * SX126X 待机模式枚举定义
 */
typedef enum sx126x_standby_cfgs_e
{
    SX126X_STANDBY_CFG_RC   = 0x00,
    SX126X_STANDBY_CFG_XOSC = 0x01,
} sx126x_standby_cfgs_t;

typedef uint8_t sx126x_standby_cfg_t;

/**
 * @brief SX126X power regulator modes enumeration definition
 * SX126X 功率调节器模式枚举定义
 */
typedef enum sx126x_reg_mods_e
{
    SX126X_REG_MODE_LDO  = 0x00,  // default
    SX126X_REG_MODE_DCDC = 0x01,
} sx126x_reg_mod_t;

/**
 * @brief SX126X power amplifier configuration parameters structure definition
 * SX126X功放配置参数结构定义
 */
typedef struct sx126x_pa_cfg_params_s
{
    uint8_t pa_duty_cycle;
    uint8_t hp_max;
    uint8_t device_sel;
    uint8_t pa_lut;
} sx126x_pa_cfg_params_t;

/**
 * @brief SX126X fallback modes enumeration definition
 * SX126X 回退模式枚举定义
 */
typedef enum sx126x_fallback_modes_e
{
    SX126X_FALLBACK_STDBY_RC   = 0x20,
    SX126X_FALLBACK_STDBY_XOSC = 0x30,
    SX126X_FALLBACK_FS         = 0x40,
} sx126x_fallback_modes_t;

/**
 * @brief SX126X interrupt masks enumeration definition
 * SX126X中断屏蔽枚举定义
 */
enum sx126x_irq_masks_e
{
    SX126X_IRQ_NONE              = ( 0 << 0 ),
    SX126X_IRQ_TX_DONE           = ( 1 << 0 ),
    SX126X_IRQ_RX_DONE           = ( 1 << 1 ),
    SX126X_IRQ_PREAMBLE_DETECTED = ( 1 << 2 ),
    SX126X_IRQ_SYNC_WORD_VALID   = ( 1 << 3 ),
    SX126X_IRQ_HEADER_VALID      = ( 1 << 4 ),
    SX126X_IRQ_HEADER_ERROR      = ( 1 << 5 ),
    SX126X_IRQ_CRC_ERROR         = ( 1 << 6 ),
    SX126X_IRQ_CAD_DONE          = ( 1 << 7 ),
    SX126X_IRQ_CAD_DETECTED      = ( 1 << 8 ),
    SX126X_IRQ_TIMEOUT           = ( 1 << 9 ),
    SX126X_IRQ_ALL               = SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_PREAMBLE_DETECTED |
                     SX126X_IRQ_SYNC_WORD_VALID | SX126X_IRQ_HEADER_VALID | SX126X_IRQ_HEADER_ERROR |
                     SX126X_IRQ_CRC_ERROR | SX126X_IRQ_CAD_DONE | SX126X_IRQ_CAD_DETECTED | SX126X_IRQ_TIMEOUT,
};

typedef uint16_t sx126x_irq_mask_t;

/**
 * @brief Calibration settings
 * 校准设置
 */
enum sx126x_cal_mask_e
{
    SX126X_CAL_RC64K      = ( 1 << 0 ),
    SX126X_CAL_RC13M      = ( 1 << 1 ),
    SX126X_CAL_PLL        = ( 1 << 2 ),
    SX126X_CAL_ADC_PULSE  = ( 1 << 3 ),
    SX126X_CAL_ADC_BULK_N = ( 1 << 4 ),
    SX126X_CAL_ADC_BULK_P = ( 1 << 5 ),
    SX126X_CAL_IMAGE      = ( 1 << 6 ),
    SX126X_CAL_ALL        = SX126X_CAL_RC64K | SX126X_CAL_RC13M | SX126X_CAL_PLL | SX126X_CAL_ADC_PULSE |
                     SX126X_CAL_ADC_BULK_N | SX126X_CAL_ADC_BULK_P | SX126X_CAL_IMAGE,
};

typedef uint8_t sx126x_cal_mask_t;

/**
 * @brief SX126X TCXO control voltages enumeration definition
 * SX126X TCXO 控制电压枚举定义
 */
typedef enum sx126x_tcxo_ctrl_voltages_e
{
    SX126X_TCXO_CTRL_1_6V = 0x00,
    SX126X_TCXO_CTRL_1_7V = 0x01,
    SX126X_TCXO_CTRL_1_8V = 0x02,
    SX126X_TCXO_CTRL_2_2V = 0x03,
    SX126X_TCXO_CTRL_2_4V = 0x04,
    SX126X_TCXO_CTRL_2_7V = 0x05,
    SX126X_TCXO_CTRL_3_0V = 0x06,
    SX126X_TCXO_CTRL_3_3V = 0x07,
} sx126x_tcxo_ctrl_voltages_t;

/**
 * @brief SX126X packet types enumeration definition
 * SX126X 数据包类型枚举定义
 */
typedef enum sx126x_pkt_types_e
{
    SX126X_PKT_TYPE_GFSK = 0x00,
    SX126X_PKT_TYPE_LORA = 0x01,
} sx126x_pkt_type_t;

/**
 * @brief SX126X power amplifier ramp-up timings enumeration definition
 * SX126X 功率放大器斜升时序枚举定义
 */
typedef enum sx126x_ramp_time_e
{
    SX126X_RAMP_10_US   = 0x00,
    SX126X_RAMP_20_US   = 0x01,
    SX126X_RAMP_40_US   = 0x02,
    SX126X_RAMP_80_US   = 0x03,
    SX126X_RAMP_200_US  = 0x04,
    SX126X_RAMP_800_US  = 0x05,
    SX126X_RAMP_1700_US = 0x06,
    SX126X_RAMP_3400_US = 0x07,
} sx126x_ramp_time_t;

/**
 * @brief SX126X GFSK modulation shaping enumeration definition
 * SX126X GFSK 调制整形枚举定义
 */
typedef enum sx126x_gfsk_pulse_shape_e
{
    SX126X_GFSK_PULSE_SHAPE_OFF   = 0x00,
    SX126X_GFSK_PULSE_SHAPE_BT_03 = 0x08,
    SX126X_GFSK_PULSE_SHAPE_BT_05 = 0x09,
    SX126X_GFSK_PULSE_SHAPE_BT_07 = 0x0A,
    SX126X_GFSK_PULSE_SHAPE_BT_1  = 0x0B,
} sx126x_gfsk_pulse_shape_t;

/**
 * @brief SX126X GFSK Rx bandwidth enumeration definition
 * SX126X GFSK Rx 带宽枚举定义
 */
typedef enum sx126x_gfsk_bw_e
{
    SX126X_GFSK_BW_4800   = 0x1F,
    SX126X_GFSK_BW_5800   = 0x17,
    SX126X_GFSK_BW_7300   = 0x0F,
    SX126X_GFSK_BW_9700   = 0x1E,
    SX126X_GFSK_BW_11700  = 0x16,
    SX126X_GFSK_BW_14600  = 0x0E,
    SX126X_GFSK_BW_19500  = 0x1D,
    SX126X_GFSK_BW_23400  = 0x15,
    SX126X_GFSK_BW_29300  = 0x0D,
    SX126X_GFSK_BW_39000  = 0x1C,
    SX126X_GFSK_BW_46900  = 0x14,
    SX126X_GFSK_BW_58600  = 0x0C,
    SX126X_GFSK_BW_78200  = 0x1B,
    SX126X_GFSK_BW_93800  = 0x13,
    SX126X_GFSK_BW_117300 = 0x0B,
    SX126X_GFSK_BW_156200 = 0x1A,
    SX126X_GFSK_BW_187200 = 0x12,
    SX126X_GFSK_BW_234300 = 0x0A,
    SX126X_GFSK_BW_312000 = 0x19,
    SX126X_GFSK_BW_373600 = 0x11,
    SX126X_GFSK_BW_467000 = 0x09,
} sx126x_gfsk_bw_t;

/**
 * @brief SX126X GFSK modulation parameters structure definition
 * SX126X GFSK调制参数结构定义
 */
typedef struct sx126x_mod_params_gfsk_s
{
    uint32_t                  br_in_bps;
    uint32_t                  fdev_in_hz;
    sx126x_gfsk_pulse_shape_t pulse_shape;
    sx126x_gfsk_bw_t          bw_dsb_param;
} sx126x_mod_params_gfsk_t;

/**
 * @brief SX126X LoRa spreading factor enumeration definition
 * SX126X LoRa扩频因子枚举定义
 */
typedef enum sx126x_lora_sf_e
{
    SX126X_LORA_SF5  = 0x05,
    SX126X_LORA_SF6  = 0x06,
    SX126X_LORA_SF7  = 0x07,
    SX126X_LORA_SF8  = 0x08,
    SX126X_LORA_SF9  = 0x09,
    SX126X_LORA_SF10 = 0x0A,
    SX126X_LORA_SF11 = 0x0B,
    SX126X_LORA_SF12 = 0x0C,
} sx126x_lora_sf_t;

/**
 * @brief SX126X LoRa bandwidth enumeration definition
 * SX126X LoRa带宽枚举定义
 */
typedef enum sx126x_lora_bw_e
{
    SX126X_LORA_BW_500 = 6,
    SX126X_LORA_BW_250 = 5,
    SX126X_LORA_BW_125 = 4,
    SX126X_LORA_BW_062 = 3,
    SX126X_LORA_BW_041 = 10,
    SX126X_LORA_BW_031 = 2,
    SX126X_LORA_BW_020 = 9,
    SX126X_LORA_BW_015 = 1,
    SX126X_LORA_BW_010 = 8,
    SX126X_LORA_BW_007 = 0,
} sx126x_lora_bw_t;

/**
 * @brief SX126X LoRa coding rate enumeration definition
 * SX126X LoRa编码率枚举定义
 */
typedef enum sx126x_lora_cr_e
{
    SX126X_LORA_CR_4_5 = 0x01,
    SX126X_LORA_CR_4_6 = 0x02,
    SX126X_LORA_CR_4_7 = 0x03,
    SX126X_LORA_CR_4_8 = 0x04,
} sx126x_lora_cr_t;

/**
 * @brief SX126X LoRa modulation parameters structure definition
 * SX126X LoRa调制参数结构定义
 */
typedef struct sx126x_mod_params_lora_s
{
    sx126x_lora_sf_t sf;    //!< LoRa Spreading Factor
    sx126x_lora_bw_t bw;    //!< LoRa Bandwidth
    sx126x_lora_cr_t cr;    //!< LoRa Coding Rate
    uint8_t          ldro;  //!< Low DataRate Optimization configuration
} sx126x_mod_params_lora_t;

/**
 * @brief SX126X GFSK preamble length Rx detection size enumeration definition
 * SX126X GFSK 前导长度 Rx 检测大小枚举定义
 */
typedef enum sx126x_gfsk_preamble_detector_e
{
    SX126X_GFSK_PREAMBLE_DETECTOR_OFF        = 0x00,
    SX126X_GFSK_PREAMBLE_DETECTOR_MIN_8BITS  = 0x04,
    SX126X_GFSK_PREAMBLE_DETECTOR_MIN_16BITS = 0x05,
    SX126X_GFSK_PREAMBLE_DETECTOR_MIN_24BITS = 0x06,
    SX126X_GFSK_PREAMBLE_DETECTOR_MIN_32BITS = 0x07,
} sx126x_gfsk_preamble_detector_t;

/**
 * @brief SX126X GFSK address filtering configuration enumeration definition
 * SX126X GFSK地址过滤配置枚举定义
 */
typedef enum sx126x_gfsk_address_filtering_e
{
    SX126X_GFSK_ADDRESS_FILTERING_DISABLE                      = 0x00,
    SX126X_GFSK_ADDRESS_FILTERING_NODE_ADDRESS                 = 0x01,
    SX126X_GFSK_ADDRESS_FILTERING_NODE_AND_BROADCAST_ADDRESSES = 0x02,
} sx126x_gfsk_address_filtering_t;

/**
 * @brief SX126X GFSK packet length enumeration definition
 * SX126X GFSK 包长度枚举定义
 */
typedef enum sx126x_gfsk_pkt_len_modes_e
{
    SX126X_GFSK_PKT_FIX_LEN = 0x00,  //!< The packet length is known on both sides, no header included
    SX126X_GFSK_PKT_VAR_LEN = 0x01,  //!< The packet length is variable, header included
} sx126x_gfsk_pkt_len_modes_t;

/**
 * @brief SX126X GFSK CRC type enumeration definition
 * SX126X GFSK CRC 类型枚举定义
 */
typedef enum sx16x_gfsk_crc_types_e
{
    SX126X_GFSK_CRC_OFF         = 0x01,
    SX126X_GFSK_CRC_1_BYTE      = 0x00,
    SX126X_GFSK_CRC_2_BYTES     = 0x02,
    SX126X_GFSK_CRC_1_BYTE_INV  = 0x04,
    SX126X_GFSK_CRC_2_BYTES_INV = 0x06,
} sx126x_gfsk_crc_types_t;

/**
 * @brief SX126X GFSK whitening control enumeration definition
 * SX126X GFSK whitening control enumeration definition
 */
typedef enum sx126x_gfsk_dc_free_e
{
    SX126X_GFSK_DC_FREE_OFF       = 0x00,
    SX126X_GFSK_DC_FREE_WHITENING = 0x01,
} sx126x_gfsk_dc_free_t;

/**
 * @brief SX126X LoRa packet length enumeration definition
 * SX126X LoRa 数据包长度枚举定义
 */
typedef enum sx126x_lora_pkt_len_modes_e
{
    SX126X_LORA_PKT_EXPLICIT = 0x00,  //!< Header included in the packet
    SX126X_LORA_PKT_IMPLICIT = 0x01,  //!< Header not included in the packet
} sx126x_lora_pkt_len_modes_t;

/**
 * @brief SX126X LoRa packet parameters structure definition
 * SX126X LoRa数据包参数结构定义
 */
typedef struct sx126x_pkt_params_lora_s
{
    uint16_t                    preamble_len_in_symb;  //!< Preamble length in symbols
    sx126x_lora_pkt_len_modes_t header_type;           //!< Header type
    uint8_t                     pld_len_in_bytes;      //!< Payload length in bytes
    bool                        crc_is_on;             //!< CRC activation
    bool                        invert_iq_is_on;       //!< IQ polarity setup
} sx126x_pkt_params_lora_t;

/**
 * @brief SX126X GFSK packet parameters structure definition
 * SX126X GFSK 数据包参数结构定义
 */
typedef struct sx126x_pkt_params_gfsk_s
{
    uint16_t                        preamble_len_in_bits;   //!< Preamble length in bits
    sx126x_gfsk_preamble_detector_t preamble_detector;      //!< Preamble detection length
    uint8_t                         sync_word_len_in_bits;  //!< Sync word length in bits
    sx126x_gfsk_address_filtering_t address_filtering;      //!< Address filtering configuration
    sx126x_gfsk_pkt_len_modes_t     header_type;            //!< Header type
    uint8_t                         pld_len_in_bytes;       //!< Payload length in bytes
    sx126x_gfsk_crc_types_t         crc_type;               //!< CRC type configuration
    sx126x_gfsk_dc_free_t           dc_free;                //!< Whitening configuration
} sx126x_pkt_params_gfsk_t;

/**
 * @brief SX126X LoRa CAD number of symbols enumeration definition
 * SX126X LoRa CAD 符号数枚举定义
 *
 * @note Represents the number of symbols to be used for a CAD operation
 * 表示用于 CAD 操作的符号数
 */
typedef enum sx126x_cad_symbs_e
{
    SX126X_CAD_01_SYMB = 0x00,
    SX126X_CAD_02_SYMB = 0x01,
    SX126X_CAD_04_SYMB = 0x02,
    SX126X_CAD_08_SYMB = 0x03,
    SX126X_CAD_16_SYMB = 0x04,
} sx126x_cad_symbs_t;

/**
 * @brief SX126X LoRa CAD exit modes enumeration definition
 * SX126X LoRa CAD 退出模式枚举定义
 *
 * @note Represents the action to be performed after a CAD is done
 * 表示完成 CAD 后要执行的操作
 */
typedef enum sx126x_cad_exit_modes_e
{
    SX126X_CAD_ONLY = 0x00,
    SX126X_CAD_RX   = 0x01,
    SX126X_CAD_LBT  = 0x10,
} sx126x_cad_exit_modes_t;

/**
 * @brief SX126X CAD parameters structure definition
 * SX126X CAD参数结构定义
 */
typedef struct sx126x_cad_param_s
{
    sx126x_cad_symbs_t      cad_symb_nb;      //!< CAD number of symbols
    uint8_t                 cad_detect_peak;  //!< CAD peak detection
    uint8_t                 cad_detect_min;   //!< CAD minimum detection
    sx126x_cad_exit_modes_t cad_exit_mode;    //!< CAD exit mode
    uint32_t                cad_timeout;      //!< CAD timeout value
} sx126x_cad_params_t;

/**
 * @brief SX126X chip mode enumeration definition
 * SX126X芯片模式枚举定义
 */
typedef enum sx126x_chip_modes_e
{
    SX126X_CHIP_MODE_UNUSED    = 0,
    SX126X_CHIP_MODE_RFU       = 1,
    SX126X_CHIP_MODE_STBY_RC   = 2,
    SX126X_CHIP_MODE_STBY_XOSC = 3,
    SX126X_CHIP_MODE_FS        = 4,
    SX126X_CHIP_MODE_RX        = 5,
    SX126X_CHIP_MODE_TX        = 6,
} sx126x_chip_modes_t;

/**
 * @brief SX126X command status enumeration definition
 * SX126X 命令状态枚举定义
 */
typedef enum sx126x_cmd_status_e
{
    SX126X_CMD_STATUS_RESERVED          = 0,
    SX126X_CMD_STATUS_RFU               = 1,
    SX126X_CMD_STATUS_DATA_AVAILABLE    = 2,
    SX126X_CMD_STATUS_CMD_TIMEOUT       = 3,
    SX126X_CMD_STATUS_CMD_PROCESS_ERROR = 4,
    SX126X_CMD_STATUS_CMD_EXEC_FAILURE  = 5,
    SX126X_CMD_STATUS_CMD_TX_DONE       = 6,
} sx126x_cmd_status_t;

/**
 * @brief SX126X chip status structure definition
 * SX126X芯片状态结构定义
 */
typedef struct sx126x_chip_status_s
{
    sx126x_cmd_status_t cmd_status;  //!< Previous command status
    sx126x_chip_modes_t chip_mode;   //!< Current chip mode
} sx126x_chip_status_t;

/**
 * @brief SX126X RX buffer status structure definition
 * SX126X RX 缓冲区状态结构体定义
 */
typedef struct sx126x_rx_buffer_status_s
{
    uint8_t pld_len_in_bytes;      //!< Number of bytes available in the buffer
    uint8_t buffer_start_pointer;  //!< Position of the first byte in the buffer
} sx126x_rx_buffer_status_t;

typedef struct sx126x_rx_status_gfsk_s
{
    bool pkt_sent;
    bool pkt_received;
    bool abort_error;
    bool length_error;
    bool crc_error;
    bool adrs_error;
} sx126x_rx_status_gfsk_t;

/**
 * @brief SX126X GFSK packet status structure definition
 * SX126X GFSK 数据包状态结构定义
 */
typedef struct sx126x_pkt_status_gfsk_s
{
    sx126x_rx_status_gfsk_t rx_status;
    int8_t                  rssi_sync;  //!< The RSSI measured on last packet
    int8_t                  rssi_avg;   //!< The averaged RSSI
} sx126x_pkt_status_gfsk_t;

/**
 * @brief SX126X LoRa packet status structure definition
 * SX126X LoRa数据包状态结构定义
 */
typedef struct sx126x_pkt_status_lora_s
{
    int8_t rssi_pkt_in_dbm;         //!< RSSI of the last packet
    int8_t snr_pkt_in_db;           //!< SNR of the last packet
    int8_t signal_rssi_pkt_in_dbm;  //!< Estimation of RSSI (after despreading)
} sx126x_pkt_status_lora_t;

/**
 * @brief SX126X GFSK reception statistics structure definition
 * SX126X GFSK接收统计结构定义
 */
typedef struct sx126x_stats_gfsk_s
{
    uint16_t nb_pkt_received;
    uint16_t nb_pkt_crc_error;
    uint16_t nb_pkt_len_error;
} sx126x_stats_gfsk_t;

/**
 * @brief SX126X LoRa reception statistics structure definition
 * SX126X LoRa接收统计结构定义
 */
typedef struct sx126x_stats_lora_s
{
    uint16_t nb_pkt_received;
    uint16_t nb_pkt_crc_error;
    uint16_t nb_pkt_header_error;
} sx126x_stats_lora_t;

/**
 * @brief SX126X errors enumeration definition
 * SX126X 错误枚举定义
 */
enum sx126x_errors_e
{
    SX126X_ERRORS_RC64K_CALIBRATION = ( 1 << 0 ),
    SX126X_ERRORS_RC13M_CALIBRATION = ( 1 << 1 ),
    SX126X_ERRORS_PLL_CALIBRATION   = ( 1 << 2 ),
    SX126X_ERRORS_ADC_CALIBRATION   = ( 1 << 3 ),
    SX126X_ERRORS_IMG_CALIBRATION   = ( 1 << 4 ),
    SX126X_ERRORS_XOSC_START        = ( 1 << 5 ),
    SX126X_ERRORS_PLL_LOCK          = ( 1 << 6 ),
    SX126X_ERRORS_PA_RAMP           = ( 1 << 8 ),
};

typedef uint16_t sx126x_errors_mask_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

//
// Operational Modes Functions
//

/**
 * @brief 将芯片设置为睡眠模式
 *
 * @param [in]  context 芯片实现上下文
 * @param [in]  cfg 睡眠模式配置
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_set_sleep( const void* context, const sx126x_sleep_cfgs_t cfg );

/**
 *@brief 将芯片设置为待机模式
 *
 *@param [in] context 芯片实现上下文。
 *@param [in] cfg 待机模式配置
 *
 *@returns 操作状态
 */
sx126x_status_t sx126x_set_standby( const void* context, const sx126x_standby_cfg_t cfg );

/**
 *@brief 将芯片设置为频率合成模式
 *
 *@param [in] context 芯片实现上下文。
 *
 *@returns 操作状态
 */
sx126x_status_t sx126x_set_fs( const void* context );

/**
 *@brief 设置芯片为传输模式
 *
 *@remark 使用该命令前，需要使用@ref sx126x_set_pkt_type 配置数据包类型。
 *
 *@remark 默认情况下，一旦有数据包发送或收到数据包，芯片自动返回待机 RC 模式
 *超时前尚未完全传输。这种行为可以通过@ref 改变
 *sx126x_set_rx_tx_fallback_mode。
 *
 *@remark 如果超时参数为 0，则不使用超时。
 *
 *@param [in] context 芯片实现上下文。
 *@param [in] timeout_in_ms Tx 操作的超时配置，单位毫秒
 *
 *@returns 操作状态 **/
sx126x_status_t sx126x_set_tx( const void* context, const uint32_t timeout_in_ms );

/**
 *@brief 设置芯片为传输模式
 *
 *@remark 使用该命令前，需要使用@ref sx126x_set_pkt_type 配置数据包类型。
 *
 *@remark 默认情况下，一旦有数据包发送或收到数据包，芯片自动返回待机 RC 模式
 *超时前尚未完全传输。这种行为可以通过@ref sx126x_set_rx_tx_fallback_mode。
 *
 *@remark 超时时间可以用以下公式计算：
 *\f$ timeout\_duration\_ms = timeout_in_rtc_step \times *\frac{1}{64} \f$
 *
 *@remark 最大值为 SX126X_MAX_TIMEOUT_IN_RTC_STEP (即 262 143 ms)
 *
 *@remark 如果超时参数为 0，则不使用超时。
 *
 *@param [in] context 芯片实现上下文。
 *@param [in] timeout_in_rtc_step Tx 操作的超时配置
 *
 *@returns 操作状态
 */ 
sx126x_status_t sx126x_set_tx_with_timeout_in_rtc_step( const void* context, const uint32_t timeout_in_rtc_step );

/**
 *@brief 将芯片设置为接收模式
 *
 *@remark 使用该命令前，需要使用@ref sx126x_set_pkt_type 配置数据包类型。
 *
 *@remark 默认情况下，芯片一收到数据包就自动返回待机RC模式
 *或者如果在超时之前没有收到数据包。这种行为可以通过@ref 改变
 *sx126x_set_rx_tx_fallback_mode。
 *
 *@remark timeout 参数可以有以下特殊值：
 *
 *|特殊值 |含义 |
 *| ---------------| ---------------------------------------------------------------------------|
 *| SX126X_RX_SINGLE_MODE | Single：芯片保持RX模式直到有接收发生，然后切换到待机RC |
 *
 *@param [in] context 芯片实现上下文。
 *@param [in] timeout_in_ms 以毫秒为单位的 Rx 操作超时配置
 *
 *@returns 操作状态
 */ 
sx126x_status_t sx126x_set_rx( const void* context, const uint32_t timeout_in_ms );

/**
 *@brief 将芯片设置为接收模式
 *
 *@remark 使用该命令前，需要使用@ref sx126x_set_pkt_type 配置数据包类型。
 *
 *@remark 默认情况下，芯片一收到数据包就自动返回待机RC模式
 *或者如果在超时之前没有收到数据包。这种行为可以通过@ref 改变
 *sx126x_set_rx_tx_fallback_mode。
 *
 *@remark 超时时长通过以下方式获得：
 *\f$ timeout\_duration\_ms = timeout_in_rtc_step \times \frac{1}{64} \f$
 *
 *@remark 最大超时值为 SX126X_MAX_TIMEOUT_IN_RTC_STEP（即 262 143 毫秒）。
 *
 *@remark timeout 参数可以有以下特殊值：
 *
 *|特殊值 |含义 |
 *| ---------------| ---------------------------------------------------------------------------|
 *| SX126X_RX_SINGLE_MODE | Single：芯片保持RX模式直到有接收发生，然后切换到待机RC |
 *| SX126X_RX_CONTINUOUS |连续：芯片即使在接收到数据包后仍保持在 RX 模式 |
 *
 *@param [in] context 芯片实现上下文。
 *@param [in] timeout_in_rtc_step Rx 操作的超时配置
 *
 *@returns 操作状态
 */ 
sx126x_status_t sx126x_set_rx_with_timeout_in_rtc_step( const void* context, const uint32_t timeout_in_rtc_step );

/**
 *@brief 配置停止接收超时的事件
 *
 *@remark 这两个选项是：
 *-同步字/标头检测（默认）
 *-前导检测
 *
 *@param [in] context 芯片实现上下文。
 *@param [in] enable 如果为 true，则计时器在同步字/标头检测时停止。
 *
 *@returns 操作状态
 */ 
sx126x_status_t sx126x_stop_timer_on_preamble( const void* context, const bool enable );

/**
 *@brief 将芯片设置为带占空比的接收模式
 *
 *@param [in] context 芯片实现上下文。
 *@param [in] rx_time_in_ms Rx 周期的超时时间 -以毫秒为单位
 *@param [in] sleep_time_in_ms 睡眠周期的长度 -以毫秒为单位
 *
 *@returns 操作状态
 */ 
sx126x_status_t sx126x_set_rx_duty_cycle( const void* context, const uint32_t rx_time_in_ms,
                                          const uint32_t sleep_time_in_ms );

/**
 *@brief 将芯片设置为带占空比的接收模式
 *
 *@remark Rx 模式持续时间定义为：
 *\f$ rx\_duration\_ms = rx_time \times \frac{1}{64} \f$
 *
 *@remark 睡眠模式持续时间定义为：
 *\f$ sleep\_duration\_ms = sleep_time \times \frac{1}{64} \f$
 *
 *@remark 最大超时值为 0xFFFFFF（即 511 秒）。
 *
 *@param [in] context 芯片实现上下文。
 *@param [in] rx_time Rx 周期的超时时间
 *@param [in] sleep_time 睡眠时间长度
 *
 *@returns 操作状态
 */ 
sx126x_status_t sx126x_set_rx_duty_cycle_with_timings_in_rtc_step( const void*    context,
                                                                   const uint32_t rx_time_in_rtc_step,
                                                                   const uint32_t sleep_time_in_rtc_step );

/**
 *@brief 将芯片设置为 CAD（通道活动检测）模式
 *
 *@remark LoRa 数据包类型应在调用此函数之前通过@ref sx126x_set_pkt_type 进行选择。
 *
 *@remark 回退模式配置为@ref sx126x_set_cad_params。
 *
 *@param [in] context 芯片实现上下文。
 *
 *@returns 操作状态
 */ 
sx126x_status_t sx126x_set_cad( const void* context );

/**
 *@brief 将芯片设置为 Tx 连续波（RF 音）。
 *
 *@remark 使用该命令前，需要使用@ref sx126x_set_pkt_type 配置数据包类型。
 *
 *@param [in] context 芯片实现上下文。
 *
 *@returns 操作状态
 */ 
sx126x_status_t sx126x_set_tx_cw( const void* context );

/**
 *@brief 在 Tx 无限前导（调制信号）中设置芯片。
 *
 *@remark 使用该命令前，需要使用@ref sx126x_set_pkt_type 配置数据包类型。
 *
 *@param [in] context 芯片实现上下文。
 *
 *@returns 操作状态
 */ 
sx126x_status_t sx126x_set_tx_infinite_preamble( const void* context );

/**
 *@brief 配置要使用的调节器模式
 *
 *@remark 需要调用这个函数来设置正确的稳压器模式，具体取决于 LDO 或 DC/DC on
 *PCB 实现。
 *
 *@param [in] context 芯片实现上下文。
 *@param [in] mode 调节器模式配置。
 *
 *@returns 操作状态
 */ 
sx126x_status_t sx126x_set_reg_mode( const void* context, const sx126x_reg_mod_t mode );

/**
 *@brief 执行请求块的校准
 *
 *@remark 此函数只能在待机 RC 模式下调用
 *
 *@remark 芯片会在退出时返回待机 RC 模式。可以使用@ref 读出潜在的校准问题
 *sx126x_get_device_errors 命令。
 *
 *@param [in] context 芯片实现上下文。
 *@param [in] param 包含要校准的块的掩码
 *
 *@returns 操作状态
 */ 
sx126x_status_t sx126x_cal( const void* context, const sx126x_cal_mask_t param );

/**
 *@brief 执行设备工作频段镜像抑制校准
 *
 *@param [in] context 芯片实现上下文。
 *@param [in] freq_in_hz 用于图像校准的频率（Hz）
 *
 *@returns 操作状态
 */ 
sx126x_status_t sx126x_cal_img( const void* context, const uint32_t freq_in_hz );

/**
 *@brief 配置 PA（功率放大器）
 *
 *@details 此命令用于区分 SX1261 和 SX1262 /SX1268。使用此命令时，用户
 *选择设备使用的 PA 及其配置。
 *
 *@param [in] context 芯片实现上下文。
 *@param [in] params 功放配置参数
 *
 *@returns 操作状态
 */ 
sx126x_status_t sx126x_set_pa_cfg( const void* context, const sx126x_pa_cfg_params_t* params );

/**
 *@brief 设置成功发送或接收后使用的芯片模式。
 *
 *@remark 在 Rx Duty Cycle 模式或 Auto TxRx 期间不考虑此设置。
 *
 *@param [in] context 芯片实现上下文。
 *@param [in] fallback_mode 选择的后备模式
 *
 *@returns 操作状态
 */ 
sx126x_status_t sx126x_set_rx_tx_fallback_mode( const void* context, const sx126x_fallback_modes_t fallback_mode );

//
// Registers and Buffer Access
//

/**
 *@brief 将数据写入寄存器内存空间。
 *
 *@param [in] context 芯片实现上下文。
 *@param [in] address 开始写操作的寄存器内存地址
 *@param [in] buffer 要写入内存的字节缓冲区。
 *@param [in] size 要写入内存的字节数，从地址开始
 *
 *@see sx126x_read_register
 */ 
sx126x_status_t sx126x_write_register( const void* context, const uint16_t address, const uint8_t* buffer,
                                       const uint8_t size );

/**
 *@brief 从寄存器内存空间读取数据。
 *
 *@param [in] context 芯片实现上下文。
 *@param [in] address 开始读操作​​的寄存器内存地址
 *@param [in] buffer 要填充寄存器数据的字节缓冲区
 *@param [in] size 从内存中读取的字节数，从地址开始
 *
 *@see sx126x_write_register
 */ 
sx126x_status_t sx126x_read_register( const void* context, const uint16_t address, uint8_t* buffer,
                                      const uint8_t size );

/**
 *@brief 将数据写入无线电 Tx 缓冲区内存空间。
 *
 *@param [in] context 芯片实现上下文。
 *@param [in] offset 芯片Tx缓冲区中的起始地址
 *@param [in] buffer 要写入无线电缓冲区的字节缓冲区
 *@param [in] size 要写入 Tx 无线电缓冲区的字节数
 *
 *@returns 操作状态
 *
 *@see sx126x_read_buffer
 */ 
sx126x_status_t sx126x_write_buffer( const void* context, const uint8_t offset, const uint8_t* buffer,
                                     const uint8_t size );

/**
 *@brief 从无线电 Rx 缓冲区内存空间读取数据。
 *
 *@param [in] context 芯片实现上下文。
 *@param [in] offset 芯片Rx缓冲区中的起始地址
 *@param [in] buffer 要填充来自 Rx 无线电缓冲区的内容的字节缓冲区
 *@param [in] size 要从 Rx 无线电缓冲区读取的字节数
 *
 *@returns 操作状态
 *
 *@see sx126x_write_buffer
 */ 
sx126x_status_t sx126x_read_buffer( const void* context, const uint8_t offset, uint8_t* buffer, const uint8_t size );

//
// DIO and IRQ Control Functions
//

/**
 *@brief 设置哪些中断信号被重定向到专用的 DIO 引脚
 *
 *@remark 默认情况下，不重定向中断信号。
 *
 *@remark 中断在系统范围内启用之前不会发生，即使它被重定向到特定的 DIO。
 *
 *@remark DIO 引脚将保持有效，直到通过调用 @ref 清除所有重定向的中断信号
 *sx126x_clear_irq_status。
 *
 *@remark DIO2 和 DIO3 与其他功能共享。见@ref sx126x_set_dio2_as_rf_sw_ctrl 和@ref
 *sx126x_set_dio3_as_tcxo_ctrl
 *
 *@param [in] 上下文芯片实现上下文。
 *@param [in] irq_mask 保存系统中断掩码的变量
 *@param [in] dio1_mask 保存 dio1 中断掩码的变量
 *@param [in] dio2_mask 保存 dio2 中断掩码的变量
 *@param [in] dio3_mask 保存 dio3 中断掩码的变量
 *
 *@returns 操作状态
 *
 *@see sx126x_clear_irq_status, sx126x_get_irq_status, sx126x_set_dio2_as_rf_sw_ctrl, sx126x_set_dio3_as_tcxo_ctrl
 */ 

sx126x_status_t sx126x_set_dio_irq_params( const void* context, const uint16_t irq_mask, const uint16_t dio1_mask,
                                           const uint16_t dio2_mask, const uint16_t dio3_mask );

/**
 *@brief 获取系统中断状态
 *
 *@param [in] context 芯片实现上下文。
 *@param [out] irq 指向保存系统中断状态的变量的指针
 *
 *@returns 操作状态
 *
 *@see sx126x_clear_irq_status
 */ 
sx126x_status_t sx126x_get_irq_status( const void* context, sx126x_irq_mask_t* irq );

/**
 *@brief 清除选定的系统中断
 *
 *@param [in] context 芯片实现上下文。
 *@param [in] irq_mask 保存要清除的系统中断的变量
 *
 *@returns 操作状态
 *
 *@see sx126x_get_irq_status
 */ 
sx126x_status_t sx126x_clear_irq_status( const void* context, const sx126x_irq_mask_t irq_mask );

/**
 *@brief 清除设置的任何无线电 irq 状态标志并返回该标志
 *被清除。
 *
 *@param [in] context 芯片实现上下文。
 *@param [out] irq 指向一个变量的指针，用于保存系统中断状态。可以为 NULL。
 *
 *@returns 操作状态
 */ 
sx126x_status_t sx126x_get_and_clear_irq_status( const void* context, sx126x_irq_mask_t* irq );

/**
 *@brief 配置嵌入式射频开关控制
 *
 *@param [in] context 芯片实现上下文。
 *@param [in] enable 如果设置为 true 启用此功能
 *
 *@returns 操作状态
 */ 
sx126x_status_t sx126x_set_dio2_as_rf_sw_ctrl( const void* context, const bool enable );

/**
 *@brief 配置嵌入式 TCXO 开关控制
 *
 *@remark 该函数只能在待机 RC 模式下调用。
 *
 *@remark 在开始任何需要 TCXO 的操作之前，芯片将等待超时发生。
 *
 *@param [in] context 芯片实现上下文。
 *@param [in] tcxo_voltage 用于为 TCXO 供电的电压。
 *@param [in] timeout TCXO 稳定所需的时间。
 *
 *@returns 操作状态
 *
 */ 
sx126x_status_t sx126x_set_dio3_as_tcxo_ctrl( const void* context, const sx126x_tcxo_ctrl_voltages_t tcxo_voltage,
                                              const uint32_t timeout );

//
// RF Modulation and Packet-Related Functions
//

/**
 *@brief 为未来的无线电操作设置射频频率。
 *
 *@remark 只有在选择了数据包类型后才能调用此命令。
 *
 *@param [in] context 芯片实现上下文。
 *@param [in] freq_in_hz 为无线电操作设置的频率（Hz）
 *
 *@returns 操作状态
 */ 
sx126x_status_t sx126x_set_rf_freq( const void* context, const uint32_t freq_in_hz );

/**
 * @brief 为未来的无线电操作设置 RF 频率 - PLL 步骤中的参数
 *
 * @remark 67 / 5000
翻译结果
只有在选择了数据包类型后才能调用该命令
 *
 * @param [in] context Chip implementation context.
 * @param [in] freq 为无线电操作设置的 PLL 步骤中的频率
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_set_rf_freq_in_pll_steps( const void* context, const uint32_t freq );

/**
 * @brief 设置数据包类型
 *
 * @param [in] context Chip implementation context.
 *
 * @param [in] pkt_type 要设置的数据包类型
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_set_pkt_type( const void* context, const sx126x_pkt_type_t pkt_type );

/**
 * @brief 获取当前数据包类型
 *
 * @param [in] context Chip implementation context.
 * @param [out] pkt_type 指向保存数据包类型的变量的指针
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_get_pkt_type( const void* context, sx126x_pkt_type_t* pkt_type );

/**
 * @brief 设置 TX 功率和功率放大器斜坡时间的参数 
 *
 * @param [in] context Chip implementation context.
 * @param [in] pwr_in_dbm The Tx output power in dBm
 * @param [in] ramp_time The ramping time configuration for the PA
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_set_tx_params( const void* context, const int8_t pwr_in_dbm,
                                      const sx126x_ramp_time_t ramp_time );

/**
 * @brief 设置 GFSK 数据包的调制参数
 *
 * @remark The command @ref sx126x_set_pkt_type must be called prior to this
 * one.
 *
 * @param [in] context Chip implementation context.
 * @param [in] params The structure of GFSK modulation configuration
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_set_gfsk_mod_params( const void* context, const sx126x_mod_params_gfsk_t* params );

/**
 * @brief 设置 LoRa 数据包的调制参数
 * @remark The command @ref sx126x_set_pkt_type must be called prior to this one.
 *
 * @param [in] context Chip implementation context.
 * @param [in] params The structure of LoRa modulation configuration
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_set_lora_mod_params( const void* context, const sx126x_mod_params_lora_t* params );

/**
 * @brief 设置GFSK报文的报文参数
 *
 * @remark The command @ref sx126x_set_pkt_type must be called prior to this
 * one.
 *
 * @param [in] context Chip implementation context.
 * @param [in] params The structure of GFSK packet configuration
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_set_gfsk_pkt_params( const void* context, const sx126x_pkt_params_gfsk_t* params );

/**
 * @brief 设置 LoRa 数据包的数据包参数
 *
 * @remark The command @ref sx126x_set_pkt_type must be called prior to this one.
 *
 * @param [in] context Chip implementation context.
 * @param [in] params The structure of LoRa packet configuration
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_set_lora_pkt_params( const void* context, const sx126x_pkt_params_lora_t* params );

/**
 * @brief 设置CAD操作参数
 *
 * @remark The command @ref sx126x_set_pkt_type must be called prior to this one.
 *
 * @param [in] context Chip implementation context.
 * @param [in] params The structure of CAD configuration
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_set_cad_params( const void* context, const sx126x_cad_params_t* params );

/**
 * @brief 为 Tx 和 Rx 操作设置缓冲区起始地址
 *
 * @param [in] context Chip implementation context.
 * @param [in] tx_base_address The start address used for Tx operations
 * @param [in] rx_base_address The start address used for Rx operations
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_set_buffer_base_address( const void* context, const uint8_t tx_base_address,
                                                const uint8_t rx_base_address );

sx126x_status_t sx126x_set_lora_symb_nb_timeout( const void* context, const uint8_t nb_of_symbs );

//
// Communication Status Information
//

/**
 * @brief 获取芯片状态
 *
 * @param [in] context Chip implementation context.
 * @param [out] radio_status Pointer to a structure holding the radio status
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_get_status( const void* context, sx126x_chip_status_t* radio_status );

/**
 * @brief 获取 LoRa 和 GFSK Rx 操作的当前 Rx 缓冲区状态
 *
 * @details This function is used to get the length of the received payload and the start address to be used when
 * reading data from the Rx buffer.
 *
 * @param [in] context Chip implementation context.
 * @param [out] rx_buffer_status Pointer to a structure to store the current status
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_get_rx_buffer_status( const void* context, sx126x_rx_buffer_status_t* rx_buffer_status );

/**
 * @brief 获取最后收到的 GFSK 数据包的状态 
 *
 * @param [in] context Chip implementation context.
 * @param [out] pkt_status Pointer to a structure to store the packet status
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_get_gfsk_pkt_status( const void* context, sx126x_pkt_status_gfsk_t* pkt_status );

/**
 * @brief 获取最后收到的 LoRa 数据包的状态
 *
 * @param [in] context Chip implementation context.
 * @param [out] pkt_status Pointer to a structure to store the packet status
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_get_lora_pkt_status( const void* context, sx126x_pkt_status_lora_t* pkt_status );

/**
 * @brief 获取瞬时RSSI值。
 *
 * @remark This function shall be called when in Rx mode.
 *
 * @param [in] context Chip implementation context.
 * @param [out] rssi_in_dbm Pointer to a variable to store the RSSI value in dBm
 *
 * @returns 运行状态
 *
 * @see sx126x_set_rx
 */
sx126x_status_t sx126x_get_rssi_inst( const void* context, int16_t* rssi_in_dbm );

/**
 * @brief 获取GFSK通信的统计信息
 *
 * @param [in] context Chip implementation context.
 * @param [out] stats Pointer to a structure to store GFSK-related statistics
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_get_gfsk_stats( const void* context, sx126x_stats_gfsk_t* stats );

/**
 * @brief 获取有关 LoRa 通信的统计信息
 *
 * @param [in] context Chip implementation context.
 * @param [out] stats Pointer to a structure to store LoRa-related statistics
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_get_lora_stats( const void* context, sx126x_stats_lora_t* stats );

/**
 * @brief 重置 Lora 和 GFSK 通信的所有统计数据
 *
 * @param [in] context Chip implementation context.
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_reset_stats( const void* context );

//
// Miscellaneous
//

/**
 * @brief 执行芯片的硬复位 
 *
 * @param [in] context Chip implementation context.
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_reset( const void* context );

/**
 * @brief 从睡眠模式唤醒
 *
 * @param [in]  context Chip implementation context.
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_wakeup( const void* context );

/**
 * @brief 获取所有活动错误的列表
 *
 * @param [in] context Chip implementation context.
 * @param [out] errors Pointer to a variable to store the error list
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_get_device_errors( const void* context, sx126x_errors_mask_t* errors );

/**
 * @brief 清除所有活动错误
 *
 * @param [in] context Chip implementation context.
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_clear_device_errors( const void* context );

/**
 * @brief 获取与 GFSK Rx 带宽相对应的参数，该参数立即高于最小请求的带宽。
 *
 * @param [in] bw Minimum required bandwith in Hz
 * @param [out] param Pointer to a value to store the parameter
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_get_gfsk_bw_param( const uint32_t bw, uint8_t* param );

/**
 * @brief 获取给定 LoRa 带宽的实际值（以hz为单位） 
 *
 * @param [in] bw LoRa bandwidth parameter
 *
 * @returns 以hz为单位的实际 LoRa 带宽
 */
uint32_t sx126x_get_lora_bw_in_hz( sx126x_lora_bw_t bw );

/**
 * @brief Compute the numerator for LoRa time-on-air computation.
 * 计算 LoRa 播出时间计算的分子。
 *
 * @remark To get the actual time-on-air in second, this value has to be divided by the LoRa bandwidth in Hertz.
 * 要获得以秒为单位的实际播出时间，必须将该值除以以赫兹为单位的 LoRa 带宽。
 *
 * @param [in] pkt_p Pointer to the structure holding the LoRa packet parameters
 * @param [in] mod_p Pointer to the structure holding the LoRa modulation parameters
 *
 * @returns LoRa time-on-air numerator
 * LoRa 播出时间分子
 */
uint32_t sx126x_get_lora_time_on_air_numerator( const sx126x_pkt_params_lora_t* pkt_p,
                                                const sx126x_mod_params_lora_t* mod_p );

/**
 * @brief 获取 LoRa 传输的播出时间（以毫秒为单位） 
 *
 * @param [in] pkt_p Pointer to a structure holding the LoRa packet parameters
 * @param [in] mod_p Pointer to a structure holding the LoRa modulation parameters
 *
 * @returns LoRa 传输的空中时间值（以毫秒为单位）
 */
uint32_t sx126x_get_lora_time_on_air_in_ms( const sx126x_pkt_params_lora_t* pkt_p,
                                            const sx126x_mod_params_lora_t* mod_p );

/**
 * @brief 计算 GFSK 播出时间计算的分子。
 *
 * @remark To get the actual time-on-air in second, this value has to be divided by the GFSK bitrate in bits per
 * second.
 * 要获得以秒为单位的实际播出时间，必须将该值除以 GFSK 比特率（以每秒比特数为单位）。
 *
 * @param [in] pkt_p Pointer to the structure holding the GFSK packet parameters
 *
 * @returns GFSK 播出时间分子
 */
uint32_t sx126x_get_gfsk_time_on_air_numerator( const sx126x_pkt_params_gfsk_t* pkt_p );

/**
 * @brief 获取 GFSK 传输的播出时间（以毫秒为单位）
 *
 * @param [in] pkt_p Pointer to a structure holding the GFSK packet parameters
 * @param [in] mod_p Pointer to a structure holding the GFSK modulation parameters
 *
 * @returns GFSK 传输的空中时间值（以毫秒为单位）
 */
uint32_t sx126x_get_gfsk_time_on_air_in_ms( const sx126x_pkt_params_gfsk_t* pkt_p,
                                            const sx126x_mod_params_gfsk_t* mod_p );

/**
 * @brief 生成一个或多个 32 位随机数。
 *
 * @remark A valid packet type must have been configured with @ref sx126x_set_pkt_type
 *         before using this command.
 *
 * @param [in]  context Chip implementation context.
 * @param [out] numbers Array where numbers will be stored.
 * @param [in]  n Number of desired random numbers.
 *
 * @returns 运行状态
 *
 * This code can potentially result in interrupt generation. It is the responsibility of
 * the calling code to disable radio interrupts before calling this function,
 * and re-enable them afterwards if necessary, or be certain that any interrupts
 * generated during this process will not cause undesired side-effects in the software.
 *
 * Please note that the random numbers produced by the generator do not have a uniform or Gaussian distribution. If
 * uniformity is needed, perform appropriate software post-processing.
 * 
 * 此代码可能会导致中断生成。 调用代码有责任在调用此函数之前禁用无线电中断，然后在必要时重新启用它们，或者确保在此过程中产生的任何中断不会对软件造成不良副作用。
 *
 * 请注意，生成器产生的随机数不具有均匀分布或高斯分布。 如果需要一致性，请执行适当的软件后处理。
 * /
sx126x_status_t sx126x_get_random_numbers( const void* context, uint32_t* numbers, unsigned int n );

/**
 * @brief Get the number of PLL steps for a given frequency in Hertz
 * 以赫兹为单位获取给定频率的 PLL 步数
 *
 * @param [in] freq_in_hz Frequency in Hertz
 *
 * @returns Number of PLL steps
 */
uint32_t sx126x_convert_freq_in_hz_to_pll_step( uint32_t freq_in_hz );

/**
 * @brief Get the number of RTC steps for a given timeout in millisecond
 * 以毫秒为单位获取给定超时的 RTC 步数
 *
 * @param [in] timeout_in_ms Timeout in millisecond
 *
 * @returns Number of RTC steps
 */
uint32_t sx126x_convert_timeout_in_ms_to_rtc_step( uint32_t timeout_in_ms );

//
// Registers access
//

/**
 * @brief Configure the boost mode in reception
 * 在接收中配置升压模式
 *
 * @remark This configuration is not kept in the retention memory. Rx boosted mode shall be enabled each time the chip
 * leaves sleep mode.
 * 此配置不保存在保留内存中。 每次芯片离开睡眠模式时，都应启用 Rx 升压模式。
 *
 * @param [in] context Chip implementation context.
 * @param [in] state Boost mode activation
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_cfg_rx_boosted( const void* context, const bool state );

/**
 * @brief Configure the sync word used in GFSK packet
 * 配置 GFSK 数据包中使用的同步字
 *
 * @param [in] context Chip implementation context.
 * @param [in] sync_word Buffer holding the sync word to be configured
 * @param [in] sync_word_len Sync word length in byte
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_set_gfsk_sync_word( const void* context, const uint8_t* sync_word, const uint8_t sync_word_len );

/**
 * @brief Configure the sync word used in LoRa packet
 * 配置 LoRa 数据包中使用的同步字
 *
 * @remark In the case of a LoRaWAN use case, the two following values are specified:
 *   - 0x12 for a private LoRaWAN network (default)
 *   - 0x34 for a public LoRaWAN network
 *
 * @param [in] context Chip implementation context.
 * @param [in] sync_word Sync word to be configured
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_set_lora_sync_word( const void* context, const uint8_t sync_word );

/**
 * @brief Configure the seed used to compute CRC in GFSK packet
 * 配置用于计算 GFSK 数据包中 CRC 的种子
 *
 * @param [in] context Chip implementation context.
 * @param [in] seed Seed value used to compute the CRC value
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_set_gfsk_crc_seed( const void* context, uint16_t seed );

/**
 * @brief Configure the polynomial used to compute CRC in GFSK packet
 * 配置GFSK包中用于计算CRC的多项式
 *
 * @param [in] context Chip implementation context.
 * @param [in] polynomial Polynomial value used to compute the CRC value
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_set_gfsk_crc_polynomial( const void* context, const uint16_t polynomial );

/**
 * @brief Configure the whitening seed used in GFSK packet
 *
 * @param [in] context Chip implementation context.
 * @param [in] seed Seed value used in data whitening
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_set_gfsk_whitening_seed( const void* context, const uint16_t seed );

/**
 * @brief Configure the Tx PA clamp
 *
 * @remark Workaround - On the SX1262, during the chip initialization, calling this function optimize the PA clamping
 * threshold. The call must be done after a Power On Reset, or a wake-up from cold Start.(see DS_SX1261-2_V1.2 datasheet
 * chapter 15.2)
 *
 * @param [in] context Chip implementation context.
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_cfg_tx_clamp( const void* context );

/**
 * @brief Stop the RTC and clear the related event
 * 停止RTC并清除相关事件
 *
 * @remark Workaround - It is advised to call this function after ANY reception with timeout active sequence, which
 * stop the RTC and clear the timeout event, if any (see DS_SX1261-2_V1.2 datasheet chapter 15.4)
 * 解决方法 - 建议在任何接收超时活动序列后调用此函数，这会停止 RTC 并清除超时事件（如果有）（请参阅 DS_SX1261-2_V1.2 数据表第 15.4 章）
 *
 * @param [in] context Chip implementation context.
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_stop_rtc( const void* context );

/**
 * @brief Configure the Over Current Protection (OCP) value
 * 配置过流保护 (OCP) 值 
 *
 * @remark The maximum value that can be configured is 63 (i.e. 157.5 mA)
 *
 * @param [in] context Chip implementation context.
 * @param [in] ocp_in_step_of_2_5_ma OCP value given in steps of 2.5 mA
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_set_ocp_value( const void* context, const uint8_t ocp_in_step_of_2_5_ma );

/**
 * @brief Configure the internal trimming capacitor values
 * 配置内部微调电容值
 *
 * @remark The device is fitted with internal programmable capacitors connected independently to the pins XTA and XTB of
 * the device. Each capacitor can be controlled independently in steps of 0.47 pF added to the minimal value 11.3pF.
 *
 * @param [in] context Chip implementation context.
 * @param [in] trimming_cap_xta Value for the trimming capacitor connected to XTA pin
 * @param [in] trimming_cap_xtb Value for the trimming capacitor connected to XTB pin
 *
 * @returns 运行状态
 */
sx126x_status_t sx126x_set_trimming_capacitor_values( const void* context, const uint8_t trimming_cap_xta,
                                                      const uint8_t trimming_cap_xtb );

#ifdef __cplusplus
}
#endif

#endif  // SX126X_H

/* --- EOF ------------------------------------------------------------------ */
