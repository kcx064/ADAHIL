#include <cstdint>

#ifndef _UBX_
#define _UBX_

#define UBX_SYNC1             0xB5
#define UBX_SYNC2             0x62

/* Message Classes */
#define UBX_CLASS_NAV         0x01
#define UBX_CLASS_RXM         0x02
#define UBX_CLASS_INF         0x04
#define UBX_CLASS_ACK         0x05
#define UBX_CLASS_CFG         0x06
#define UBX_CLASS_MON         0x0A
#define UBX_CLASS_RTCM3       0xF5

/* Message IDs */
#define UBX_ID_NAV_POSLLH     0x02
#define UBX_ID_NAV_DOP        0x04
#define UBX_ID_NAV_SOL        0x06
#define UBX_ID_NAV_PVT        0x07
#define UBX_ID_NAV_VELNED     0x12
#define UBX_ID_NAV_TIMEUTC    0x21
#define UBX_ID_NAV_SVINFO     0x30
#define UBX_ID_NAV_SAT        0x35
#define UBX_ID_NAV_SVIN       0x3B
#define UBX_ID_NAV_RELPOSNED  0x3C
#define UBX_ID_RXM_SFRBX      0x13
#define UBX_ID_RXM_RAWX       0x15
#define UBX_ID_INF_DEBUG      0x04
#define UBX_ID_INF_ERROR      0x00
#define UBX_ID_INF_NOTICE     0x02
#define UBX_ID_INF_WARNING    0x01
#define UBX_ID_ACK_NAK        0x00
#define UBX_ID_ACK_ACK        0x01
#define UBX_ID_CFG_PRT        0x00 // deprecated in protocol version >= 27 -> use CFG_VALSET
#define UBX_ID_CFG_MSG        0x01 // deprecated in protocol version >= 27 -> use CFG_VALSET
#define UBX_ID_CFG_RATE       0x08 // deprecated in protocol version >= 27 -> use CFG_VALSET
#define UBX_ID_CFG_CFG        0x09 // deprecated in protocol version >= 27 -> use CFG_VALSET
#define UBX_ID_CFG_NAV5       0x24 // deprecated in protocol version >= 27 -> use CFG_VALSET
#define UBX_ID_CFG_RST        0x04
#define UBX_ID_CFG_SBAS       0x16
#define UBX_ID_CFG_TMODE3     0x71 // deprecated in protocol version >= 27 -> use CFG_VALSET
#define UBX_ID_CFG_GNSS       0x3E
#define UBX_ID_CFG_VALSET     0x8A
#define UBX_ID_CFG_VALGET     0x8B
#define UBX_ID_CFG_VALDEL     0x8C
#define UBX_ID_MON_VER        0x04
#define UBX_ID_MON_HW         0x09 // deprecated in protocol version >= 27 -> use MON_RF
#define UBX_ID_MON_RF         0x38

/* UBX ID for RTCM3 output messages */
/* Minimal messages for RTK: 1005, 1077 + (1087 or 1127) */
/* Reduced message size using MSM4: 1005, 1074 + (1084 or 1124)  */
#define UBX_ID_RTCM3_1005     0x05    /**< Stationary RTK reference station ARP */
#define UBX_ID_RTCM3_1074     0x4A    /**< GPS MSM4 */
#define UBX_ID_RTCM3_1077     0x4D    /**< GPS MSM7 */
#define UBX_ID_RTCM3_1084     0x54    /**< GLONASS MSM4 */
#define UBX_ID_RTCM3_1087     0x57    /**< GLONASS MSM7 */
#define UBX_ID_RTCM3_1094     0x5E    /**< Galileo MSM4 */
#define UBX_ID_RTCM3_1097     0x61    /**< Galileo MSM7 */
#define UBX_ID_RTCM3_1124     0x7C    /**< BeiDou MSM4 */
#define UBX_ID_RTCM3_1127     0x7F    /**< BeiDou MSM7 */
#define UBX_ID_RTCM3_1230     0xE6    /**< GLONASS code-phase biases */
#define UBX_ID_RTCM3_4072     0xFE    /**< Reference station PVT (u-blox proprietary RTCM Message) - Used for moving baseline */

/* Message Classes & IDs */
#define UBX_MSG_CFG_VALSET    ((UBX_CLASS_CFG) | UBX_ID_CFG_VALSET << 8)
#define UBX_MSG_CFG_PRT       ((UBX_CLASS_CFG) | UBX_ID_CFG_PRT << 8)

#define UBX_MSG_MON_HW        ((UBX_CLASS_MON) | UBX_ID_MON_HW << 8)
#define UBX_MSG_MON_VER       ((UBX_CLASS_MON) | UBX_ID_MON_VER << 8)
#define UBX_MSG_MON_RF        ((UBX_CLASS_MON) | UBX_ID_MON_RF << 8)

/* Key ID's for CFG-VAL{GET,SET,DEL} */
#define UBX_CFG_KEY_CFG_I2C_ENABLED             0x10510003
#define UBX_CFG_KEY_CFG_I2CINPROT_UBX           0x10710001
#define UBX_CFG_KEY_CFG_I2CINPROT_NMEA          0x10710002
#define UBX_CFG_KEY_CFG_I2CINPROT_RTCM3X        0x10710004
#define UBX_CFG_KEY_CFG_I2COUTPROT_UBX          0x10720001
#define UBX_CFG_KEY_CFG_I2COUTPROT_NMEA         0x10720002
#define UBX_CFG_KEY_CFG_I2COUTPROT_RTCM3X       0x10720004

#define UBX_CFG_KEY_CFG_UART1_BAUDRATE          0x40520001
#define UBX_CFG_KEY_CFG_UART1_STOPBITS          0x20520002
#define UBX_CFG_KEY_CFG_UART1_DATABITS          0x20520003
#define UBX_CFG_KEY_CFG_UART1_PARITY            0x20520004
#define UBX_CFG_KEY_CFG_UART1_ENABLED           0x20520005
#define UBX_CFG_KEY_CFG_UART1_REMAP             0x20520006
#define UBX_CFG_KEY_CFG_UART1INPROT_UBX         0x10730001
#define UBX_CFG_KEY_CFG_UART1INPROT_NMEA        0x10730002
#define UBX_CFG_KEY_CFG_UART1INPROT_RTCM3X      0x10730004
#define UBX_CFG_KEY_CFG_UART1OUTPROT_UBX        0x10740001
#define UBX_CFG_KEY_CFG_UART1OUTPROT_NMEA       0x10740002
#define UBX_CFG_KEY_CFG_UART1OUTPROT_RTCM3X     0x10740004

#define UBX_CFG_KEY_CFG_UART2_BAUDRATE          0x40530001
#define UBX_CFG_KEY_CFG_UART2_STOPBITS          0x20530002
#define UBX_CFG_KEY_CFG_UART2_DATABITS          0x20530003
#define UBX_CFG_KEY_CFG_UART2_PARITY            0x20530004
#define UBX_CFG_KEY_CFG_UART2_ENABLED           0x20530005
#define UBX_CFG_KEY_CFG_UART2_REMAP             0x20530006
#define UBX_CFG_KEY_CFG_UART2INPROT_UBX         0x10750001
#define UBX_CFG_KEY_CFG_UART2INPROT_NMEA        0x10750002
#define UBX_CFG_KEY_CFG_UART2INPROT_RTCM3X      0x10750004
#define UBX_CFG_KEY_CFG_UART2OUTPROT_UBX        0x10760001
#define UBX_CFG_KEY_CFG_UART2OUTPROT_NMEA       0x10760002
#define UBX_CFG_KEY_CFG_UART2OUTPROT_RTCM3X     0x10760004

#define UBX_CFG_KEY_CFG_USB_ENABLED             0x10650001
#define UBX_CFG_KEY_CFG_USBINPROT_UBX           0x10770001
#define UBX_CFG_KEY_CFG_USBINPROT_NMEA          0x10770002
#define UBX_CFG_KEY_CFG_USBINPROT_RTCM3X        0x10770004
#define UBX_CFG_KEY_CFG_USBOUTPROT_UBX          0x10780001
#define UBX_CFG_KEY_CFG_USBOUTPROT_NMEA         0x10780002
#define UBX_CFG_KEY_CFG_USBOUTPROT_RTCM3X       0x10780004

#define UBX_CFG_KEY_CFG_SPIINPROT_UBX           0x10790001
#define UBX_CFG_KEY_CFG_SPIINPROT_NMEA          0x10790002
#define UBX_CFG_KEY_CFG_SPIINPROT_RTCM3X        0x10790004
#define UBX_CFG_KEY_CFG_SPIOUTPROT_UBX          0x107a0001
#define UBX_CFG_KEY_CFG_SPIOUTPROT_NMEA         0x107a0002
#define UBX_CFG_KEY_CFG_SPIOUTPROT_RTCM3X       0x107a0004

#define UBX_CFG_KEY_NAVHPG_DGNSSMODE            0x20140011

#define UBX_CFG_KEY_NAVSPG_FIXMODE              0x20110011
#define UBX_CFG_KEY_NAVSPG_UTCSTANDARD          0x2011001c
#define UBX_CFG_KEY_NAVSPG_DYNMODEL             0x20110021

#define UBX_CFG_KEY_ODO_USE_ODO                 0x10220001
#define UBX_CFG_KEY_ODO_USE_COG                 0x10220002
#define UBX_CFG_KEY_ODO_OUTLPVEL                0x10220003
#define UBX_CFG_KEY_ODO_OUTLPCOG                0x10220004

#define UBX_CFG_KEY_ITFM_ENABLE                 0x1041000d

#define UBX_CFG_KEY_RATE_MEAS                   0x30210001
#define UBX_CFG_KEY_RATE_NAV                    0x30210002
#define UBX_CFG_KEY_RATE_TIMEREF                0x20210003

#define UBX_CFG_KEY_TMODE_MODE                  0x20030001
#define UBX_CFG_KEY_TMODE_POS_TYPE              0x20030002
#define UBX_CFG_KEY_TMODE_LAT                   0x40030009
#define UBX_CFG_KEY_TMODE_LON                   0x4003000a
#define UBX_CFG_KEY_TMODE_HEIGHT                0x4003000b
#define UBX_CFG_KEY_TMODE_LAT_HP                0x2003000c
#define UBX_CFG_KEY_TMODE_LON_HP                0x2003000d
#define UBX_CFG_KEY_TMODE_HEIGHT_HP             0x2003000e
#define UBX_CFG_KEY_TMODE_FIXED_POS_ACC         0x4003000f
#define UBX_CFG_KEY_TMODE_SVIN_MIN_DUR          0x40030010
#define UBX_CFG_KEY_TMODE_SVIN_ACC_LIMIT        0x40030011

#define UBX_CFG_KEY_MSGOUT_UBX_MON_RF_I2C       0x20910359
#define UBX_CFG_KEY_MSGOUT_UBX_NAV_SVIN_I2C     0x20910088
#define UBX_CFG_KEY_MSGOUT_UBX_NAV_SAT_I2C      0x20910015
#define UBX_CFG_KEY_MSGOUT_UBX_NAV_DOP_I2C      0x20910038
#define UBX_CFG_KEY_MSGOUT_UBX_NAV_PVT_I2C      0x20910006
#define UBX_CFG_KEY_MSGOUT_UBX_NAV_PVT_UART1    0x20910007
#define UBX_CFG_KEY_MSGOUT_UBX_NAV_RELPOSNED_I2C 0x2091008d
#define UBX_CFG_KEY_MSGOUT_UBX_RXM_SFRBX_I2C    0x20910231
#define UBX_CFG_KEY_MSGOUT_UBX_RXM_RAWX_I2C     0x209102a4
#define UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1005_I2C 0x209102bd
#define UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1077_I2C 0x209102cc
#define UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1087_I2C 0x209102d1
#define UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1097_I2C 0x20910318
#define UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1127_I2C 0x209102d6
#define UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1230_I2C 0x20910303

#define UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE4072_0_UART2  0x20910300
#define UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE4072_1_UART2  0x20910383
#define UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1077_UART2    0x209102ce
#define UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1087_UART2    0x209102d3
#define UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1097_UART2    0x2091031a
#define UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1127_UART2    0x209102d8
#define UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1230_UART2    0x20910305

#define UBX_CFG_KEY_SPI_ENABLED                 0x10640006
#define UBX_CFG_KEY_SPI_MAXFF                   0x20640001

#define UBX_CFG_KEY_SIGNAL_GPS_ENA              0x1031001f  /**< GPS enable */
#define UBX_CFG_KEY_SIGNAL_GPS_L1CA_ENA         0x10310001  /**< GPS L1C/A */
#define UBX_CFG_KEY_SIGNAL_GPS_L2C_ENA          0x10310003  /**< GPS L2C (only on u-blox F9 platform products) */
#define UBX_CFG_KEY_SIGNAL_SBAS_ENA             0x10310020  /**< SBAS enable */
#define UBX_CFG_KEY_SIGNAL_SBAS_L1CA_ENA        0x10310005  /**< SBAS L1C/A */
#define UBX_CFG_KEY_SIGNAL_GAL_ENA              0x10310021  /**< Galileo enable */
#define UBX_CFG_KEY_SIGNAL_GAL_E1_ENA           0x10310007  /**< Galileo E1 */
#define UBX_CFG_KEY_SIGNAL_GAL_E5B_ENA          0x1031000a  /**< Galileo E5b (only on u-blox F9 platform products) */
#define UBX_CFG_KEY_SIGNAL_BDS_ENA              0x10310022  /**< BeiDou Enable */
#define UBX_CFG_KEY_SIGNAL_BDS_B1_ENA           0x1031000d  /**< BeiDou B1I */
#define UBX_CFG_KEY_SIGNAL_BDS_B2_ENA           0x1031000e  /**< BeiDou B2I (only on u-blox F9 platform products) */
#define UBX_CFG_KEY_SIGNAL_QZSS_ENA             0x10310024  /**< QZSS enable */
#define UBX_CFG_KEY_SIGNAL_QZSS_L1CA_ENA        0x10310012  /**< QZSS L1C/A */
#define UBX_CFG_KEY_SIGNAL_QZSS_L1S_ENA         0x10310014  /**< QZSS L1S */
#define UBX_CFG_KEY_SIGNAL_QZSS_L2C_ENA         0x10310015  /**< QZSS L2C (only on u-blox F9 platform products) */
#define UBX_CFG_KEY_SIGNAL_GLO_ENA              0x10310025  /**< GLONASS enable */
#define UBX_CFG_KEY_SIGNAL_GLO_L1_ENA           0x10310018  /**< GLONASS L1 */
#define UBX_CFG_KEY_SIGNAL_GLO_L2_ENA           0x1031001a  /**< GLONASS L2 (only on u-blox F9 platform products) */

/*** u-blox protocol binary message and payload definitions ***/
#pragma pack(push, 1)

/* Rx MON-VER */
typedef struct ubx_payload_mon_ver
{
	uint8_t clsID;
	uint8_t msgID;
	uint16_t length;
	uint8_t swVersion[30];
	uint8_t hwVersion[10];
	uint8_t extension[30];
} ubx_payload_mon_ver_t;

/* Rx ACK-ACK */
typedef union ubx_payload_rx_ack_ack
{
	uint16_t msg;
	struct msg_struct
	{
		uint8_t clsID;
		uint8_t msgID;
	} msg_s;
}ubx_payload_rx_ack_ack_t; 

/* Rx ACK-NAK */
typedef union ubx_payload_rx_ack_nak
{
	uint16_t msg;
	struct msg_struct
	{
		uint8_t clsID;
		uint8_t msgID;
	} msg_s;
}ubx_payload_rx_ack_nak_t;

typedef union ubx_tx_ack_nak
{
	uint8_t msg_buf[6];
	struct msg_struct
	{
		uint8_t clsID;
		uint8_t msgID;
		uint16_t length;
		uint8_t payload[2];
	} msg_s;
}ubx_tx_ack_nak_t;

typedef union cfg_value
{
	uint64_t val8byte;
	uint32_t val4byte;
	uint16_t val2byte;
	uint8_t val1byte;
}cfg_value_t;

/* Rx NAV-PVT (ubx8) */
typedef union ubx_nav_pvt
{
	uint8_t msg_buf[96];
	struct msg_struct
	{
		uint8_t clsID;
		uint8_t msgID;
		uint16_t length;
		uint32_t iTOW;          /**< GPS Time of Week [ms] */
		uint16_t year;          /**< Year (UTC)*/
		uint8_t  month;         /**< Month, range 1..12 (UTC) */
		uint8_t  day;           /**< Day of month, range 1..31 (UTC) */
		uint8_t  hour;          /**< Hour of day, range 0..23 (UTC) */
		uint8_t  min;           /**< Minute of hour, range 0..59 (UTC) */
		uint8_t  sec;           /**< Seconds of minute, range 0..60 (UTC) */
		uint8_t  valid;         /**< Validity flags (see UBX_RX_NAV_PVT_VALID_...) */
		uint32_t tAcc;          /**< Time accuracy estimate (UTC) [ns] */
		int32_t  nano;          /**< Fraction of second (UTC) [-1e9...1e9 ns] */
		uint8_t  fixType;       /**< GNSSfix type: 0 = No fix, 1 = Dead Reckoning only, 2 = 2D fix, 3 = 3d-fix, 4 = GNSS + dead reckoning, 5 = time only fix */
		uint8_t  flags;         /**< Fix Status Flags (see UBX_RX_NAV_PVT_FLAGS_...) */
		uint8_t  flags2;
		uint8_t  numSV;         /**< Number of SVs used in Nav Solution */
		int32_t  lon;           /**< Longitude [1e-7 deg] */
		int32_t  lat;           /**< Latitude [1e-7 deg] */
		int32_t  height;        /**< Height above ellipsoid [mm] */
		int32_t  hMSL;          /**< Height above mean sea level [mm] */
		uint32_t hAcc;          /**< Horizontal accuracy estimate [mm] */
		uint32_t vAcc;          /**< Vertical accuracy estimate [mm] */
		int32_t  velN;          /**< NED north velocity [mm/s]*/
		int32_t  velE;          /**< NED east velocity [mm/s]*/
		int32_t  velD;          /**< NED down velocity [mm/s]*/
		int32_t  gSpeed;        /**< Ground Speed (2-D) [mm/s] */
		int32_t  headMot;       /**< Heading of motion (2-D) [1e-5 deg] */
		uint32_t sAcc;          /**< Speed accuracy estimate [mm/s] */
		uint32_t headAcc;       /**< Heading accuracy estimate (motion and vehicle) [1e-5 deg] */
		uint16_t pDOP;          /**< Position DOP [0.01] */
		uint16_t flags3;		/** flags3 */
		uint8_t reserved3[4];	/** 4 bytes */
		int32_t  headVeh;       /**< (ubx8+ only) Heading of vehicle (2-D) [1e-5 deg] */
		int16_t  magDec;        /**< (ubx8+ only) deg*/
		uint16_t magAcc;		/** deg */
	} msg_s;
}ubx_nav_pvt_t;


typedef union ubx_mon_rf
{
	uint8_t msg_buf[28];
	struct msg_struct
	{
		uint8_t version;
		uint8_t nBlocks;         /**< number of RF blocks included */
		uint8_t reserved1[2];

		struct ubx_payload_rx_mon_rf_block_t {
			uint8_t  blockId;       /**< RF block id */
			uint8_t  flags;         /**< jammingState */
			uint8_t  antStatus;     /**< Status of the antenna superior state machine */
			uint8_t  antPower;      /**< Current power status of antenna */
			uint32_t postStatus;    /**< POST status word */
			uint8_t  reserved2[4];
			uint16_t noisePerMS;    /**< Noise level as measured by the GPS core */
			uint16_t agcCnt;        /**< AGC Monitor (counts SIGI xor SIGLO, range 0 to 8191 */
			uint8_t  jamInd;        /**< CW jamming indicator, scaled (0=no CW jamming, 255=strong CW jamming) */
			int8_t   ofsI;          /**< Imbalance of I-part of complex signal */
			uint8_t  magI;          /**< Magnitude of I-part of complex signal (0=no signal, 255=max magnitude) */
			int8_t   ofsQ;          /**< Imbalance of Q-part of complex signal */
			uint8_t  magQ;          /**< Magnitude of Q-part of complex signal (0=no signal, 255=max magnitude) */
			uint8_t  reserved3[3];
		};
		ubx_payload_rx_mon_rf_block_t block[1]; ///< only read out the first block
	};
} ubx_mon_rf_t;

// typedef struct {
// 	uint8_t version;
// 	uint8_t nBlocks;         /**< number of RF blocks included */
// 	uint8_t reserved1[2];

// 	struct ubx_payload_rx_mon_rf_block_t {
// 		uint8_t  blockId;       /**< RF block id */
// 		uint8_t  flags;         /**< jammingState */
// 		uint8_t  antStatus;     /**< Status of the antenna superior state machine */
// 		uint8_t  antPower;      /**< Current power status of antenna */
// 		uint32_t postStatus;    /**< POST status word */
// 		uint8_t  reserved2[4];
// 		uint16_t noisePerMS;    /**< Noise level as measured by the GPS core */
// 		uint16_t agcCnt;        /**< AGC Monitor (counts SIGI xor SIGLO, range 0 to 8191 */
// 		uint8_t  jamInd;        /**< CW jamming indicator, scaled (0=no CW jamming, 255=strong CW jamming) */
// 		int8_t   ofsI;          /**< Imbalance of I-part of complex signal */
// 		uint8_t  magI;          /**< Magnitude of I-part of complex signal (0=no signal, 255=max magnitude) */
// 		int8_t   ofsQ;          /**< Imbalance of Q-part of complex signal */
// 		uint8_t  magQ;          /**< Magnitude of Q-part of complex signal (0=no signal, 255=max magnitude) */
// 		uint8_t  reserved3[3];
// 	};

// 	ubx_payload_rx_mon_rf_block_t block[1]; ///< only read out the first block
// } ubx_payload_rx_mon_rf_t;



#pragma pack(pop)
/*** END OF u-blox protocol binary message and payload definitions ***/

#endif