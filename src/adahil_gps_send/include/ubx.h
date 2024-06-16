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

/*** u-blox protocol binary message and payload definitions ***/
#pragma pack(push, 1)
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


#pragma pack(pop)
/*** END OF u-blox protocol binary message and payload definitions ***/

#endif