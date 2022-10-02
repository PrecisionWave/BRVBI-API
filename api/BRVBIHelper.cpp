/*
 * BRVBIHelper.cpp
 *
 *  Created on: Sep 30, 2022
 *      Author: PrecisionWave AG
 */

#include "BRVBIHelper.h"
#include <stdlib.h>
#include <string.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// udp packet handling

BRVBIDataMessage::BRVBIDataMessage()
{
	// reset information
	m_wSampleAlloc				= 0;
	m_SampleData				= nullptr;
	m_wFlags					= 0;
	m_dwBandwidthInHz			= 0;
	m_dwAcquisitionClockInHz	= 0;
	m_dwSamplingClockInHz		= 0;
	m_dwPacketID				= 0;
	m_dwIdentification			= 0;
	m_ulIndex					= 0;
	m_wLength					= 0;
	m_wSampleCount				= 0;
	m_dScaling					= 0;
	m_dCenterFrequencyInMHz		= 0.0;
	m_dwMeasID					= 0;
	m_ulAbsTime					= 0;
}

BRVBIDataMessage::~BRVBIDataMessage()
{
	// final clean up
	if (m_wSampleAlloc)
		free (m_SampleData);
}

int BRVBIDataMessage::Assemble(uint8_t* byteSequence)
{
	uint16_t	wLen;

	// assemble the byte sequence, returns the size of the sequence
	wLen	= 0;
	// skip length
	wLen	= 2;
	// add params
	memcpy (&byteSequence[wLen], &m_ulIndex, sizeof(uint64_t));							wLen	+= sizeof(uint64_t);
	memcpy (&byteSequence[wLen], &m_ulAbsTime, sizeof(uint64_t));								wLen	+= sizeof(uint64_t);
	memcpy (&byteSequence[wLen], &m_dwIdentification, sizeof(uint32_t));				wLen	+= sizeof(uint32_t);
	memcpy (&byteSequence[wLen], &m_dwMeasID, sizeof(uint32_t));						wLen	+= sizeof(uint32_t);
	memcpy (&byteSequence[wLen], &m_dwPacketID, sizeof(uint32_t));						wLen	+= sizeof(uint32_t);
	memcpy (&byteSequence[wLen], &m_dCenterFrequencyInMHz, sizeof(double));			wLen	+= sizeof(double);
	memcpy (&byteSequence[wLen], &m_dwAcquisitionClockInHz, sizeof(uint32_t));		wLen	+= sizeof(uint32_t);
	memcpy (&byteSequence[wLen], &m_dwSamplingClockInHz, sizeof(uint32_t));			wLen	+= sizeof(uint32_t);
	memcpy (&byteSequence[wLen], &m_dwBandwidthInHz, sizeof(uint32_t));				wLen	+= sizeof(uint32_t);
	memcpy (&byteSequence[wLen], &m_dScaling, sizeof(double));							wLen	+= sizeof(double);
	memcpy (&byteSequence[wLen], &m_wFlags, sizeof(uint16_t));							wLen	+= sizeof(uint16_t);
	memcpy (&byteSequence[wLen], &m_wSampleCount, sizeof(uint16_t));					wLen	+= sizeof(uint16_t);
	// finally copy the sample information
	memcpy (&byteSequence[wLen], m_SampleData, 2*m_wSampleCount*sizeof(short));	wLen	+= 2*m_wSampleCount*sizeof(short);
	// set the length
	m_wLength	= wLen;
	memcpy (byteSequence, &wLen, sizeof(uint16_t));
	// all done
	return wLen;
}

int BRVBIDataMessage::Extract(int nSize, uint8_t* byteSequence)
{
	// extracts all information from a byte sequence
	// returns the byte count consumed for valid information or -1
	int			nPos;
	int			nS;
	uint16_t	wLen;

	// initial size check
	if (nSize < 62)					// 2+8+8+4+4+4+8+4+4+4+8+2+2
		return 0;
	// extract params
	nPos	= 0;
	memcpy (&wLen, &byteSequence[nPos], sizeof(uint16_t));							nPos	+= sizeof(uint16_t);
	memcpy (&m_ulIndex, &byteSequence[nPos], sizeof(uint64_t));						nPos	+= sizeof(uint64_t);
	memcpy (&m_ulAbsTime, &byteSequence[nPos], sizeof(uint64_t));					nPos	+= sizeof(uint64_t);
	memcpy (&m_dwIdentification, &byteSequence[nPos], sizeof(uint32_t));			nPos	+= sizeof(uint32_t);
	memcpy (&m_dwMeasID, &byteSequence[nPos], sizeof(uint32_t));					nPos	+= sizeof(uint32_t);
	memcpy (&m_dwPacketID, &byteSequence[nPos], sizeof(uint32_t));					nPos	+= sizeof(uint32_t);
	memcpy (&m_dCenterFrequencyInMHz, &byteSequence[nPos], sizeof(double));		nPos	+= sizeof(double);
	memcpy (&m_dwAcquisitionClockInHz, &byteSequence[nPos], sizeof(uint32_t));	nPos	+= sizeof(uint32_t);
	memcpy (&m_dwSamplingClockInHz, &byteSequence[nPos], sizeof(uint32_t));		nPos	+= sizeof(uint32_t);
	memcpy (&m_dwBandwidthInHz, &byteSequence[nPos], sizeof(uint32_t));			nPos	+= sizeof(uint32_t);
	memcpy (&m_dScaling, &byteSequence[nPos], sizeof(double));						nPos	+= sizeof(double);
	memcpy (&m_wFlags, &byteSequence[nPos], sizeof(uint16_t));						nPos	+= sizeof(uint16_t);
	memcpy (&m_wSampleCount, &byteSequence[nPos], sizeof(uint16_t));				nPos	+= sizeof(uint16_t);
	// consistency check
	nS	= 2*m_wSampleCount*sizeof(short);
	if ((wLen - nPos) != nS)
		return -1;							// som tin won
	// size check
	if ((nPos + nS) > nSize)
		return 0;							// message might be valid
	// memory handling
	if (m_wSampleCount > m_wSampleAlloc) {
		if (m_wSampleAlloc)
			free (m_SampleData);
		m_wSampleAlloc	= m_wSampleCount;
		m_SampleData	= (short*) calloc(2*m_wSampleAlloc, sizeof(short));
	}
	// copy operation
	memcpy (m_SampleData, &byteSequence[nPos], nS);
	nPos	+= nS;
	// all done
	return nPos;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// tcp packet handling

BRVBIControlMessage::BRVBIControlMessage()
{
	m_wUDPPort					= 0;
	m_bExtRef					= 0;
	m_dwStatus					= 0;
	m_dwIndex					= 0;
	m_dwRxError					= 0;
	m_ulStartTime				= 0;
	m_dwIdentification			= 0;
	m_dwInputConfiguration		= 0;
	m_dwBurstSize				= 0;
	m_dwAcquisitionClockInHz	= 0;
	m_bGPSSync					= 0;
	m_bMessageType				= 0;
	m_wLength					= 0;
	m_dwRxAck					= 0;
	m_dwMeasID					= 0;
	m_dCenterFrequencyInMHz		= 0;
	m_dwBandwidthInHz			= 0;
	m_dwSamplingClockInHz		= 0;
}

BRVBIControlMessage::~BRVBIControlMessage()
{
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// message analysis

int BRVBIControlMessage::Extract(int nSize, uint8_t* byteSequence)
{
	// extracts the information from a TCP message
	//	returns the #byte consumed for valid information or -1
	//
	int			icc;
	int			nTemp;
	uint16_t	wLen;
	uint32_t	dwIndex;
	uint8_t		bType;
	uint8_t*	payload;
	// reset basic information
	m_bMessageType	= BRVBI_MESSAGE_TYPE_INVALID;
	m_wLength		= 0;
	m_dwIndex		= 0;
	// initial size check
	if (nSize < 7)			// 2 + 4 + 1
		return -1;
	// extract primary params
	icc	= 0;
	memcpy (&wLen, &byteSequence[icc], sizeof(uint16_t));		icc	+= sizeof(uint16_t);
	memcpy (&dwIndex, &byteSequence[icc], sizeof(uint32_t));	icc	+= sizeof(uint32_t);
	memcpy (&bType, &byteSequence[icc], sizeof(uint8_t));		icc	+= sizeof(uint8_t);
	nTemp	= wLen - icc;
	payload	= &byteSequence[icc];
	// further processing according type
	switch (bType) {
		case BRVBI_MESSAGE_TYPE_HW_CONFIGURE:		nTemp	= ExtractHWConfigure(nTemp, payload);		break;
		case BRVBI_MESSAGE_TYPE_GET_SERIAL:			nTemp	= 0;										break;
		case BRVBI_MESSAGE_TYPE_RET_SERIAL:			nTemp	= ExtractRetSerial(nTemp, payload);			break;
		case BRVBI_MESSAGE_TYPE_GET_VERSION:		nTemp	= 0;													break;
		case BRVBI_MESSAGE_TYPE_RET_VERSION:		nTemp	= ExtractRetVersion(nTemp, payload);		break;
		case BRVBI_MESSAGE_TYPE_TASK_CONFIGURE:		nTemp	= ExtractTaskConfigure(nTemp, payload);		break;
		case BRVBI_MESSAGE_TYPE_TASK_DELETE:		nTemp	= ExtractTaskDelete(nTemp, payload);		break;
		case BRVBI_MESSAGE_TYPE_TASK_TRIGGER:		nTemp	= ExtractTaskTrigger(nTemp, payload);		break;
		case BRVBI_MESSAGE_TYPE_TASK_STATUS:		nTemp	= ExtractTaskStatus(nTemp, payload);		break;
		case BRVBI_MESSAGE_TYPE_TASK_DATA:			nTemp	= ExtractTaskData(nTemp, payload);			break;
		case BRVBI_MESSAGE_TYPE_STREAM_CONFIGURE:	nTemp	= ExtractStreamConfigure(nTemp, payload);	break;
		case BRVBI_MESSAGE_TYPE_STREAM_DELETE:		nTemp	= ExtractStreamDelete(nTemp, payload);		break;
		case BRVBI_MESSAGE_TYPE_STREAM_START:		nTemp	= ExtractStreamStart(nTemp, payload);		break;
		case BRVBI_MESSAGE_TYPE_STREAM_STATUS:		nTemp	= ExtractStreamStatus(nTemp, payload);		break;
		case BRVBI_MESSAGE_TYPE_STREAM_STOP:		nTemp	= ExtractStreamStop(nTemp, payload);		break;
		case BRVBI_MESSAGE_TYPE_RX_ACK:				nTemp	= ExtractRxAck(nTemp, payload);				break;
		case BRVBI_MESSAGE_TYPE_RX_ERROR:			nTemp	= ExtractRxError(nTemp, payload);			break;
		case BRVBI_MESSAGE_TYPE_RX_KEEPALIVE:		nTemp	= ExtractRxKeepAlive(nTemp, payload);		break;
		default:
			nTemp = -1;		// not supported message
	}
	// check for valid information
	if (nTemp < 0)
		return -1;
	// keep basic information
	m_dwIndex		= dwIndex;
	m_wLength		= wLen;
	m_bMessageType	= bType;
	// update the byte count
	icc	+= nTemp;
	// all done
	return icc;
}

int BRVBIControlMessage::ExtractHWConfigure(int nSize, uint8_t* payload)
{
	// specific message extraction, returns the #byte consumed if the information is valid or -1
	int	icc	= 0;
	// size check
	if ((nSize != 2) && (nSize != 4))
		return -1;
	// individual parameter extraction
	m_bExtRef	= payload[icc++];
	m_bGPSSync	= payload[icc++];
	// udp port if available
	if (nSize == 4) {
		memcpy (&m_wUDPPort, &payload[icc], sizeof(uint16_t));
		icc	+= sizeof(uint16_t);
	}
	else
		m_wUDPPort	= BRVBI_UDP_DEFAULT_PORT;
	// all done
	return icc;
}

int BRVBIControlMessage::ExtractRetSerial(int nSize, uint8_t* payload)
{
	// specific message extraction, returns the #byte consumed if there is valid information or -1
	int	icc	= 0;
	int	nLen;

	// reset the serial info in any case
	memset (m_strSerialNb, 0, 256);
	// initial size check
	if (nSize < 1)
		return -1;
	// get the length
	nLen	= payload[icc++];
	// second length check
	if (nSize != (nLen+1))
		return -1;
	// copy operation
	memcpy (m_strSerialNb, &payload[icc], nLen);
	// all done
	return nSize;
}

int BRVBIControlMessage::ExtractRetVersion(int nSize, uint8_t* payload)
{
	// specific message extraction, returns the #byte consumed if there is valid information or -1
	int	icc	= 0;
	int	nLen;

	// reset version info in any case
	memset (m_strDSPVersion, 0, 256);
	memset (m_strBuildDate, 0, 256);
	// initial size check
	if (nSize < 2)
		return -1;
	// get DSPVersion length
	nLen	= payload[icc++];
	// second length check
	if (nSize < (nLen+1))
		return -1;
	// copy operation
	memcpy (m_strDSPVersion, &payload[icc], nLen);
	icc	+= nLen;
	// length check
	if (icc >= nSize)
		return -1;
	// get the build date length
	nLen	= payload[icc++];
	// final length check
	if ((icc+nLen) != nSize)
		return -1;
	// copy operation
	memcpy (m_strBuildDate, &payload[icc], nLen);
	// all done
	return nSize;
}


int BRVBIControlMessage::ExtractTaskConfigure(int nSize, uint8_t* payload)
{
	// specific message extraction, returns the #byte consumed if the information is valid or -1
	int	icc	= 0;
	int	nLen	= 16;		// 4+4+4+4
	// size check
	if (nSize != nLen)
		return -1;
	// individual parameter extraction
	memcpy (&m_dwAcquisitionClockInHz, &payload[icc], sizeof(uint32_t));		icc	+= sizeof(uint32_t);
	memcpy (&m_dwSamplingClockInHz, &payload[icc], sizeof(uint32_t));			icc	+= sizeof(uint32_t);
	memcpy (&m_dwBandwidthInHz, &payload[icc], sizeof(uint32_t));				icc	+= sizeof(uint32_t);
	memcpy (&m_dwBurstSize, &payload[icc], sizeof(uint32_t));					icc	+= sizeof(uint32_t);
	// all done
	return nLen;
}

int BRVBIControlMessage::ExtractTaskDelete(int nSize, uint8_t* payload)
{
	// specific message extraction, returns the #byte consumed if the information is valid or -1
	int	icc	= 0;
	int	nLen	= 4;
	// size check
	if (nSize != nLen)
		return -1;
	// individual parameter extraction
	memcpy (&m_dwIdentification, &payload[icc], sizeof(uint32_t));		icc	+= sizeof(uint32_t);
	// all done
	return nLen;
}

int BRVBIControlMessage::ExtractTaskStatus(int nSize, uint8_t* payload)
{
	// specific message extraction, returns the #byte consumed if the information is valid or -1
	int	icc	= 0;
	int	nLen	= 4;
	// size check
	if (nSize != nLen)
		return -1;
	// individual parameter extraction
	memcpy (&m_dwIdentification, &payload[icc], sizeof(uint32_t));		icc	+= sizeof(uint32_t);
	// all done
	return nLen;
}

int BRVBIControlMessage::ExtractTaskData(int nSize, uint8_t* payload)
{
	// specific message extraction, returns the #byte consumed if the information is valid or -1
	int	icc	= 0;
	int	nLen	= 8;
	// size check
	if (nSize != nLen)
		return -1;
	// individual parameter extraction
	memcpy (&m_dwIdentification, &payload[icc], sizeof(uint32_t));		icc	+= sizeof(uint32_t);
	memcpy (&m_dwMeasID, &payload[icc], sizeof(uint32_t));				icc	+= sizeof(uint32_t);
	// all done
	return nLen;
}

int BRVBIControlMessage::ExtractTaskTrigger(int nSize, uint8_t* payload)
{
	// specific message extraction, returns the #byte consumed if the information is valid or -1
	int	icc	= 0;
	int	nLen	= 24;		// 4 + 8 + 4 + 8
	// size check
	if (nSize != nLen)
		return -1;
	// individual parameter extraction
	memcpy (&m_dwIdentification, &payload[icc], sizeof(uint32_t));			icc	+= sizeof(uint32_t);
	memcpy (&m_dCenterFrequencyInMHz, &payload[icc], sizeof(double));		icc	+= sizeof(double);
	memcpy (&m_dwInputConfiguration, &payload[icc], sizeof(uint32_t));		icc	+= sizeof(uint32_t);
	memcpy (&m_ulStartTime, &payload[icc], sizeof(uint64_t));				icc	+= sizeof(uint64_t);
	// all done
	return nLen;
}

int BRVBIControlMessage::ExtractStreamConfigure(int nSize, uint8_t* payload)
{
	// specific message extraction, returns the #byte consumed if the information is valid or -1
	int	icc	= 0;
	int	nLen	= 30;		// 8+4+4+4+4+2+4
	// size check
	if (nSize != nLen)
		return -1;
	// individual parameter extraction
	memcpy (&m_dCenterFrequencyInMHz, &payload[icc], sizeof(double));			icc	+= sizeof(double);
	memcpy (&m_dwAcquisitionClockInHz, &payload[icc], sizeof(uint32_t));		icc	+= sizeof(uint32_t);
	memcpy (&m_dwSamplingClockInHz, &payload[icc], sizeof(uint32_t));			icc	+= sizeof(uint32_t);
	memcpy (&m_dwBandwidthInHz, &payload[icc], sizeof(uint32_t));				icc	+= sizeof(uint32_t);
	memcpy (&m_dwInputConfiguration, &payload[icc], sizeof(uint32_t));			icc	+= sizeof(uint32_t);
	memcpy (&m_wUDPPort, &payload[icc], sizeof(uint16_t));						icc	+= sizeof(uint16_t);
	memcpy (&m_dwBurstSize, &payload[icc], sizeof(uint32_t));					icc += sizeof(uint32_t);
	// all done
	return nLen;
}

int BRVBIControlMessage::ExtractStreamDelete(int nSize, uint8_t* payload)
{
	// specific message extraction, returns the #byte consumed if the information is valid or -1
	int	icc	= 0;
	int	nLen	= 4;
	// size check
	if (nSize != nLen)
		return -1;
	// individual parameter extraction
	memcpy (&m_dwIdentification, &payload[icc], sizeof(uint32_t));		icc	+= sizeof(uint32_t);
	// all done
	return nLen;
}

int BRVBIControlMessage::ExtractStreamStart(int nSize, uint8_t* payload)
{
	// specific message extraction, returns the #byte consumed if the information is valid or -1
	int	icc	= 0;
	int	nLen	= 4;
	// size check
	if (nSize != nLen)
		return -1;
	// individual parameter extraction
	memcpy (&m_dwIdentification, &payload[icc], sizeof(uint32_t));		icc	+= sizeof(uint32_t);
	// all done
	return nLen;
}

int BRVBIControlMessage::ExtractStreamStatus(int nSize, uint8_t* payload)
{
	// specific message extraction, returns the #byte consumed if the information is valid or -1
	int	icc	= 0;
	int	nLen	= 4;
	// size check
	if (nSize != nLen)
		return -1;
	// individual parameter extraction
	memcpy (&m_dwIdentification, &payload[icc], sizeof(uint32_t));		icc	+= sizeof(uint32_t);
	// all done
	return nLen;
}

int BRVBIControlMessage::ExtractStreamStop(int nSize, uint8_t* payload)
{
	// specific message extraction, returns the #byte consumed if the information is valid or -1
	int	icc	= 0;
	int	nLen	= 4;
	// size check
	if (nSize != nLen)
		return -1;
	// individual parameter extraction
	memcpy (&m_dwIdentification, &payload[icc], sizeof(uint32_t));		icc	+= sizeof(uint32_t);
	// all done
	return nLen;
}

int BRVBIControlMessage::ExtractRxAck(int nSize, uint8_t* payload)
{
	// specific message extraction, returns the #byte consumed if the information is valid or -1
	int	icc	= 0;
	int	nLen	= 4;
	// size check
	if (nSize != nLen)
		return -1;
	// individual parameter extraction
	memcpy (&m_dwRxAck, &payload[icc], sizeof(uint32_t));		icc	+= sizeof(uint32_t);
	// all done
	return nLen;
}

int BRVBIControlMessage::ExtractRxError(int nSize, uint8_t* payload)
{
	// specific message extraction, returns the #byte consumed if the information is valid or -1
	int	icc	= 0;
	int	nLen	= 4;
	// size check
	if (nSize != nLen)
		return -1;
	// individual parameter extraction
	memcpy (&m_dwRxError, &payload[icc], sizeof(uint32_t));		icc	+= sizeof(uint32_t);
	// all done
	return nLen;
}

int BRVBIControlMessage::ExtractRxKeepAlive(int nSize, uint8_t* payload)
{
	// no payload expected
	int	nLen	= 0;
	// size check
	if (nSize != nLen)
		return -1;
	// all done
	return nLen;
}

int BRVBIControlMessage::Assemble(uint8_t* byteSequence)
{
	// assembles the message, returns the size of the byte sequence
	uint16_t	wLen;
	int		nTemp;
	uint8_t*	payload;

	// skip length information
	wLen	= sizeof(uint16_t);
	// basic params
	memcpy (&byteSequence[wLen], &m_dwIndex, sizeof(uint32_t));		wLen	+= sizeof(uint32_t);
	byteSequence[wLen++]	= m_bMessageType;
	// specific part
	payload	= &byteSequence[wLen];
	switch (m_bMessageType) {
		case BRVBI_MESSAGE_TYPE_HW_CONFIGURE:		nTemp	= AssembleHWConfigure(payload);		break;
		case BRVBI_MESSAGE_TYPE_GET_SERIAL:			nTemp	= 0;								break;
		case BRVBI_MESSAGE_TYPE_RET_SERIAL:			nTemp	= AssembleRetSerial(payload);		break;
		case BRVBI_MESSAGE_TYPE_GET_VERSION:		nTemp	= 0;											break;
		case BRVBI_MESSAGE_TYPE_RET_VERSION:		nTemp	= AssembleRetVersion(payload);		break;
		case BRVBI_MESSAGE_TYPE_TASK_CONFIGURE:	nTemp	= AssembleTaskConfigure(payload);	break;
		case BRVBI_MESSAGE_TYPE_TASK_DELETE:		nTemp	= AssembleTaskDelete(payload);		break;
		case BRVBI_MESSAGE_TYPE_TASK_TRIGGER:		nTemp	= AssembleTaskTrigger(payload);		break;
		case BRVBI_MESSAGE_TYPE_TASK_STATUS:		nTemp	= AssembleTaskStatus(payload);		break;
		case BRVBI_MESSAGE_TYPE_TASK_DATA:			nTemp	= AssembleTaskData(payload);			break;
		case BRVBI_MESSAGE_TYPE_STREAM_CONFIGURE:	nTemp	= AssembleStreamConfigure(payload);	break;
		case BRVBI_MESSAGE_TYPE_STREAM_START:		nTemp	= AssembleStreamStart(payload);		break;
		case BRVBI_MESSAGE_TYPE_STREAM_STOP:		nTemp	= AssembleStreamStop(payload);		break;
		case BRVBI_MESSAGE_TYPE_STREAM_STATUS:		nTemp	= AssembleStreamStatus(payload);		break;
		case BRVBI_MESSAGE_TYPE_RX_ACK:				nTemp	= AssembleRxAck(payload);				break;
		case BRVBI_MESSAGE_TYPE_RX_ERROR:			nTemp	= AssembleRxError(payload);			break;
		case BRVBI_MESSAGE_TYPE_RX_KEEPALIVE:		nTemp	= 0;											break;		// no additional information
	}
	// update length information
	wLen	+= nTemp;
	memcpy (byteSequence, &wLen, sizeof(uint16_t));
	// all done
	return wLen;
}

int BRVBIControlMessage::AssembleHWConfigure(uint8_t* payload)
{
	// assembles the specific part, returns the payload size
	int	icc	= 0;

	payload[icc++]	= m_bExtRef;
	payload[icc++]	= m_bGPSSync;
	memcpy (&payload[icc], &m_wUDPPort, sizeof(uint16_t));	icc	+= sizeof(uint16_t);
	return icc;
}

int BRVBIControlMessage::AssembleRetSerial(uint8_t* payload)
{
	int		icc		= 0;
	int		nLen	= strlen(m_strSerialNb);
	payload[icc++]	= nLen;
	if (nLen) {
		memcpy (&payload[icc], m_strSerialNb, nLen);
		icc	+= nLen;
	}
	return icc;
}

int BRVBIControlMessage::AssembleRetVersion(uint8_t* payload)
{
	int		icc		= 0;
	int		nLen;
	// DSP version
	nLen = strlen(m_strDSPVersion);
	payload[icc++]	= nLen;
	if (nLen) {
		memcpy (&payload[icc], m_strDSPVersion, nLen);
		icc	+= nLen;
	}
	// build date
	nLen				= strlen(m_strBuildDate);
	payload[icc++]	= nLen;
	if (nLen) {
		memcpy(&payload[icc], m_strBuildDate, nLen);
		icc	+= nLen;
	}
	return icc;
}

int BRVBIControlMessage::AssembleTaskConfigure(uint8_t* payload)
{
	// assembles the specific part, returns the payload size
	int	icc	= 0;

	memcpy (&payload[icc], &m_dwAcquisitionClockInHz, sizeof(uint32_t));	icc	+= sizeof(uint32_t);
	memcpy (&payload[icc], &m_dwSamplingClockInHz, sizeof(uint32_t));		icc	+= sizeof(uint32_t);
	memcpy (&payload[icc], &m_dwBandwidthInHz, sizeof(uint32_t));			icc	+= sizeof(uint32_t);
	memcpy (&payload[icc], &m_dwBurstSize, sizeof(uint32_t));				icc	+= sizeof(uint32_t);
	return icc;
}

int BRVBIControlMessage::AssembleTaskDelete(uint8_t* payload)
{
	// assembles the specific part, returns the payload size
	int	icc	= 0;

	memcpy (&payload[icc], &m_dwIdentification, sizeof(uint32_t));	icc	+= sizeof(uint32_t);
	return icc;
}

int BRVBIControlMessage::AssembleTaskStatus(uint8_t* payload)
{
	// assembles the specific part, returns the payload size
	int	icc	= 0;

	memcpy (&payload[icc], &m_dwIdentification, sizeof(uint32_t));	icc	+= sizeof(uint32_t);
	return icc;
}

int BRVBIControlMessage::AssembleTaskData(uint8_t* payload)
{
	// assembles the specific part, returns the payload size
	int	icc	= 0;

	memcpy (&payload[icc], &m_dwIdentification, sizeof(uint32_t));	icc	+= sizeof(uint32_t);
	memcpy (&payload[icc], &m_dwMeasID, sizeof(uint32_t));			icc	+= sizeof(uint32_t);
	return icc;
}

int BRVBIControlMessage::AssembleTaskTrigger(uint8_t* payload)
{
	// assembles the specific part, returns the payload size
	int	icc	= 0;

	memcpy (&payload[icc], &m_dwIdentification, sizeof(uint32_t));		icc	+= sizeof(uint32_t);
	memcpy (&payload[icc], &m_dCenterFrequencyInMHz, sizeof(double));	icc	+= sizeof(double);
	memcpy (&payload[icc], &m_dwInputConfiguration, sizeof(uint32_t));	icc	+= sizeof(uint32_t);
	memcpy (&payload[icc], &m_ulStartTime, sizeof(uint64_t));	icc	+= sizeof(uint64_t);
	return icc;
}

int BRVBIControlMessage::AssembleStreamConfigure(uint8_t* payload)
{
	// assembles the specific part, returns the payload size
	int	icc	= 0;

	memcpy (&payload[icc], &m_dCenterFrequencyInMHz, sizeof(double));		icc	+= sizeof(double);
	memcpy (&payload[icc], &m_dwAcquisitionClockInHz, sizeof(uint32_t));	icc	+= sizeof(uint32_t);
	memcpy (&payload[icc], &m_dwSamplingClockInHz, sizeof(uint32_t));		icc	+= sizeof(uint32_t);
	memcpy (&payload[icc], &m_dwBandwidthInHz, sizeof(uint32_t));			icc	+= sizeof(uint32_t);
	memcpy (&payload[icc], &m_dwInputConfiguration, sizeof(uint32_t));		icc	+= sizeof(uint32_t);
	memcpy (&payload[icc], &m_wUDPPort, sizeof(uint16_t));	icc	+= sizeof(uint16_t);
	memcpy (&payload[icc], &m_dwBurstSize, sizeof(uint32_t));	icc	+= sizeof(uint32_t);
	return icc;
}

int BRVBIControlMessage::AssembleStreamStart(uint8_t* payload)
{
	// assembles the specific part, returns the payload size
	int	icc	= 0;

	memcpy (&payload[icc], &m_dwIdentification, sizeof(uint32_t));	icc	+= sizeof(uint32_t);
	return icc;
}

int BRVBIControlMessage::AssembleStreamStop(uint8_t* payload)
{
	// assembles the specific part, returns the payload size
	int	icc	= 0;

	memcpy (&payload[icc], &m_dwIdentification, sizeof(uint32_t));	icc	+= sizeof(uint32_t);
	return icc;
}

int BRVBIControlMessage::AssembleStreamStatus(uint8_t* payload)
{
	// assembles the specific part, returns the payload size
	int	icc	= 0;

	memcpy (&payload[icc], &m_dwIdentification, sizeof(uint32_t));	icc	+= sizeof(uint32_t);
	return icc;
}

int BRVBIControlMessage::AssembleRxAck(uint8_t* payload)
{
	// assembles the specific part, returns the payload size
	int	icc	= 0;

	memcpy (&payload[icc], &m_dwRxAck, sizeof(uint32_t));	icc	+= sizeof(uint32_t);
	return icc;
}

int BRVBIControlMessage::AssembleRxError(uint8_t* payload)
{
	// assembles the specific part, returns the payload size
	int	icc	= 0;

	memcpy (&payload[icc], &m_dwRxError, sizeof(uint32_t));	icc	+= sizeof(uint32_t);
	return icc;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// input configuration handling

void BRVBIHelper::ParseInputConfiguration(uint32_t dwInputConfiguration, double dCenterFrequency, SInputConfiguration* pInputConfig)
{
	uint32_t	dwVal;

	// get the RxPath configuration byte
	dwVal	= (dwInputConfiguration >> 24) & 0xFF;
	// keep general pathn information for the scaling handler
	pInputConfig->bRxPath	= dwVal;
	// check the LF path flag
	if (dCenterFrequency < 100.0)
		pInputConfig->nLFPath	= 1;
	else {
		if (dCenterFrequency > 116.0)
			pInputConfig->nLFPath	= 0;
		else
			pInputConfig->nLFPath	= (dwVal & BRVBI_INPUT_CONFIGURATION_LFPATH) > 0;
	}
	// check the RX1Input flag
	if (pInputConfig->nLFPath)
		pInputConfig->nRX1Input	= (dwVal & BRVBI_INPUT_CONFIGURATION_RX1INPUT) > 0;
	else
		pInputConfig->nRX1Input	= 0;
	// check the preamp flag in any case
	pInputConfig->nPreamp	= (dwVal & BRVBI_INPUT_CONFIGURATION_PREAMP) > 0;
	// remove the preamp flag for the HF path
	if (!pInputConfig->nLFPath)
		pInputConfig->nPreamp	= 0;
	// check the auto-scaling
	pInputConfig->nAutoScaling	= (dwVal & BRVBI_INPUT_CONFIGURATION_AUTOSCALING) > 0;
	// check single scaling
	if (pInputConfig->nAutoScaling)
		pInputConfig->nSingleScaling	= 0;
	else
		pInputConfig->nSingleScaling	= (dwVal & BRVBI_INPUT_CONFIGURATION_SINGLESCALING) > 0;
	// get the coarse attenuation
	dwVal	= (dwInputConfiguration >> 16) & 0xFF;
	pInputConfig->bCoarseAttenuation	= dwVal & 127;
	// reset the input attenuation for autoscaling
	if (pInputConfig->nAutoScaling)
		pInputConfig->bCoarseAttenuation	= 0;
	if (pInputConfig->nSingleScaling)
		pInputConfig->bCoarseAttenuation	= 0;
	// no coarse attenuation for LF path
	if (pInputConfig->nLFPath)
		pInputConfig->bCoarseAttenuation	= 0;
	// get the fine attenuation
	dwVal	= (dwInputConfiguration >> 8) & 0xFF;
	pInputConfig->bFineAttenuation	= dwVal & 127;
	// reset the fine attenuation for auto scaling
	if (pInputConfig->nAutoScaling)
		pInputConfig->bFineAttenuation	= 0;
	if (pInputConfig->nSingleScaling)
		pInputConfig->bFineAttenuation	= 0;
}

uint32_t BRVBIHelper::AssembleInputConfiguration(SInputConfiguration sInputConfig)
{
	uint32_t	dwOut;
	uint32_t	dwVal;

	// reset output value
	dwOut	= 0;
	// assemble the RxPath byte
	dwVal	= 0;
	dwVal	|= sInputConfig.nLFPath * BRVBI_INPUT_CONFIGURATION_LFPATH;
	dwVal	|= sInputConfig.nRX1Input * BRVBI_INPUT_CONFIGURATION_RX1INPUT;
	dwVal	|= sInputConfig.nPreamp * BRVBI_INPUT_CONFIGURATION_PREAMP;
	dwVal	|= sInputConfig.nAutoScaling * BRVBI_INPUT_CONFIGURATION_AUTOSCALING;
	dwVal	|= sInputConfig.nSingleScaling * BRVBI_INPUT_CONFIGURATION_SINGLESCALING;
	dwOut	= dwVal;
	// assemble the input attenuation
	dwVal	= sInputConfig.bCoarseAttenuation & 127;
	dwOut	= (dwOut << 8) + dwVal;
	// assemble mixer configuration
	dwVal	= sInputConfig.bFineAttenuation & 127;
	dwOut	= (dwOut << 8) + dwVal;
	// spare byte
	dwOut	= (dwOut << 8);
	// all done
	return dwOut;
}

int BRVBIHelper::DirecPath(double dCenterFrequency)
{
	if ((dCenterFrequency > 174.927) && (dCenterFrequency < 237.201))
		return 1;
	else
		return 0;
}
