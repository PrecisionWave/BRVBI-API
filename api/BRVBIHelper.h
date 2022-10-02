/*
 * BRVBIHelper.h
 *
 *  Created on: Sep 30, 2022
 *      Author: PrecisionWave AG
 */

#pragma once
#include <stdint.h>

// sockets
#define	BRVBI_TCP_DEFAULT_PORT					0x5057
#define	BRVBI_UDP_DEFAULT_PORT					0x5058

// message types
#define	BRVBI_MESSAGE_TYPE_INVALID				0xFF
#define	BRVBI_MESSAGE_TYPE_HW_CONFIGURE			0x01
#define	BRVBI_MESSAGE_TYPE_GET_SERIAL			0x02
#define	BRVBI_MESSAGE_TYPE_RET_SERIAL			0x03
#define	BRVBI_MESSAGE_TYPE_GET_VERSION			0x04
#define	BRVBI_MESSAGE_TYPE_RET_VERSION			0x05
#define	BRVBI_MESSAGE_TYPE_TASK_CONFIGURE		0x10
#define	BRVBI_MESSAGE_TYPE_TASK_DELETE			0x11
#define	BRVBI_MESSAGE_TYPE_TASK_TRIGGER			0x12
#define	BRVBI_MESSAGE_TYPE_TASK_STATUS			0x13
#define	BRVBI_MESSAGE_TYPE_TASK_DATA			0x14
#define	BRVBI_MESSAGE_TYPE_STREAM_CONFIGURE		0x20
#define	BRVBI_MESSAGE_TYPE_STREAM_DELETE		0x21
#define	BRVBI_MESSAGE_TYPE_STREAM_START			0x22
#define	BRVBI_MESSAGE_TYPE_STREAM_STATUS		0x23
#define	BRVBI_MESSAGE_TYPE_STREAM_STOP			0x24
#define	BRVBI_MESSAGE_TYPE_RX_ACK				0x00
#define	BRVBI_MESSAGE_TYPE_RX_ERROR				0x99
#define	BRVBI_MESSAGE_TYPE_RX_KEEPALIVE			0x50

// defines for the input configuration
#define	BRVBI_INPUT_CONFIGURATION_LFPATH		128
#define BRVBI_INPUT_CONFIGURATION_RX1INPUT		64
#define BRVBI_INPUT_CONFIGURATION_PREAMP		32
#define BRVBI_INPUT_CONFIGURATION_AUTOSCALING	16
#define BRVBI_INPUT_CONFIGURATION_SINGLESCALING	8

// defines for the data packet flags
#define	BRVBI_DATA_PACKET_FLAG_SCALINGERROR		0x0001
#define	BRVBI_DATA_PACKET_FLAG_OVERFLOW			0x0002
#define	BRVBI_DATA_PACKET_FLAG_STREAMINGBREAK	0x0004

// error codes
#define	BRVBI_ERROR_CODE_INVALID_MESSAGE_TYPE		900
#define	BRVBI_ERROR_CODE_NOT_SUPPORTED				901
#define	BRVBI_ERROR_CODE_INVALID_TASK_ID			910
#define BRVBI_ERROR_CODE_INVALID_TASK_PARAMETER		911
#define	BRVBI_ERROR_CODE_INVALID_TASK_STATE			912
#define	BRVBI_ERROR_CODE_INVALID_STREAM_ID			920
#define	BRVBI_ERROR_CODE_INVALID_STREAM_PARAMETER	921
#define	BRVBI_ERROR_CODE_INVALID_STREAM_STATE		922
#define	BRVBI_ERROR_CODE_TASKLIST_OVERFLOW			990
#define	BRVBI_ERROR_CODE_UNKNOWN_ERROR				999

struct SInputConfiguration
{
	int		nLFPath;								// set to 1 when using the LF path
	int		nRX1Input;								// set to 1 when using RX1 input for the LF path, not used for the HF path
	int		nPreamp;								// preamp flag when using the LF path, not used for the HF path
	int		nAutoScaling;							// set to 1 when continuous scaling is requested
	int		nSingleScaling;							// set to 1 when a single scaling step at the beginning of the acquisition is requested
	uint8_t	bFineAttenuation;						// input attenuation for LF path, intermediate attenuation for HF path
	uint8_t	bCoarseAttenuation;						// input attenuation for HF path, not used for LF path
	uint8_t	bRxPath;								// combined path information used in the scaling handler
};


class BRVBIControlMessage
{
public:
	BRVBIControlMessage(void);
	~BRVBIControlMessage(void);

	int	Extract(int nSize, uint8_t* byteSequence);
	int	Assemble(uint8_t* byteSequence);

private:
	int	ExtractHWConfigure(int nSize, uint8_t* payload);
	int	ExtractRetSerial(int nSize, uint8_t* payload);
	int	ExtractRetVersion(int nSize, uint8_t* payload);
	int	ExtractTaskConfigure(int nSize, uint8_t* payload);
	int	ExtractTaskDelete(int nSize, uint8_t* payload);
	int	ExtractTaskTrigger(int nSize, uint8_t* payload);
	int	ExtractTaskStatus(int nSize, uint8_t* payload);
	int	ExtractTaskData(int nSize, uint8_t* payload);
	int	ExtractStreamConfigure(int nSize, uint8_t* payload);
	int	ExtractStreamDelete(int nSize, uint8_t* payload);
	int	ExtractStreamStart(int nSize, uint8_t* payload);
	int	ExtractStreamStatus(int nSize, uint8_t* payload);
	int	ExtractStreamStop(int nSize, uint8_t* payload);
	int	ExtractRxAck(int nSize, uint8_t* payload);
	int	ExtractRxError(int nSize, uint8_t* payload);
	int	ExtractRxKeepAlive(int nSize, uint8_t* payload);
	int	AssembleHWConfigure(uint8_t* payload);
	int	AssembleRetSerial(uint8_t* payload);
	int	AssembleRetVersion(uint8_t* payload);
	int	AssembleTaskConfigure(uint8_t* payload);
	int	AssembleTaskDelete(uint8_t* payload);
	int	AssembleTaskTrigger(uint8_t* payload);
	int	AssembleTaskStatus(uint8_t* payload);
	int	AssembleTaskData(uint8_t* payload);
	int	AssembleStreamConfigure(uint8_t* payload);
	int	AssembleStreamDelete(uint8_t* payload);
	int	AssembleStreamStatus(uint8_t* payload);
	int	AssembleStreamStart(uint8_t* payload);
	int	AssembleStreamStop(uint8_t* payload);
	int	AssembleRxAck(uint8_t* payload);
	int	AssembleRxError(uint8_t* payload);

public:
	uint16_t	m_wLength;
	uint32_t	m_dwIndex;
	uint8_t		m_bMessageType;
	uint32_t	m_dwStatus;
	// serial nb
	char		m_strSerialNb[256];
	// version info
	char		m_strDSPVersion[256];
	char		m_strBuildDate[256];
	// params for HW configure message
	uint8_t		m_bExtRef;
	uint8_t		m_bGPSSync;
	uint16_t	m_wUDPPort;
	// parameters for burst and stream configuration
	uint32_t	m_dwAcquisitionClockInHz;
	uint32_t	m_dwSamplingClockInHz;
	uint32_t	m_dwBurstSize;
	uint32_t	m_dwBandwidthInHz;
	uint32_t	m_dwInputConfiguration;
	double		m_dCenterFrequencyInMHz;
	// parameters for burst and stream control
	uint32_t	m_dwIdentification;
	uint32_t	m_dwMeasID;
	uint64_t	m_ulStartTime;
	// rx information
	uint32_t	m_dwRxAck;
	uint32_t	m_dwRxError;
};

class BRVBIDataMessage
{
public:
	BRVBIDataMessage(void);
	~BRVBIDataMessage(void);

	int	Extract(int nSize, uint8_t* byteSequence);
	int	Assemble(uint8_t* byteSequence);

public:
	uint16_t	m_wLength;
	uint64_t	m_ulIndex;
	uint64_t	m_ulAbsTime;
	uint32_t	m_dwIdentification;
	uint32_t	m_dwMeasID;
	uint32_t	m_dwPacketID;
	double		m_dCenterFrequencyInMHz;
	uint32_t	m_dwAcquisitionClockInHz;
	uint32_t	m_dwSamplingClockInHz;
	uint32_t	m_dwBandwidthInHz;
	double		m_dScaling;
	uint16_t	m_wFlags;
	uint16_t	m_wSampleCount;
	short*		m_SampleData;

private:
	uint16_t	m_wSampleAlloc;
};




class BRVBIHelper
{
public:
	static void 	ParseInputConfiguration(uint32_t dwInputConfiguration, double dCenterFrequency, SInputConfiguration* pInputConfig);
	static uint32_t	AssembleInputConfiguration(SInputConfiguration sInputConfig);
	static int		DirecPath(double dCenterFrequency);
};

