/*
 * BRVBIControl.cpp
 *
 *  Created on: Sep 30, 2022
 *      Author: PrecisionWave AG
 */

#include "BRVBIControl.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <algorithm>
#include <math.h>

BRVBIControl::BRVBIControl() {
	m_nControlSocket 			= -1;
	m_nTransferSocket 			= -1;
	m_nStreamRunning			= 0;
	m_pRXStreamThread 			= nullptr;
	m_nStreamBufferSize 		= 0;
	m_nStreamBlockSizeInSamples	= 0;
	m_nStreamBlockAlloc 		= 0;
	m_StreamBlockFlag 			= nullptr;
	m_StreamBlockSamplePos 		= nullptr;
	m_StreamBlockScaling 		= nullptr;
	m_StreamBlockIQTime 		= nullptr;
	m_dwAcquisitionBufferAlloc	= 0;
	m_AcquisitionBuffer 		= nullptr;
	m_dwMessageIndex			= 0;
	m_nStreamReset				= 0;
	m_nStreamBlockSize			= 0;
	m_nStreamBlockWritePosition	= 0;
	m_nStreamBlockReadPosition	= 0;
}

BRVBIControl::~BRVBIControl() {
	cleanUp();
}

uint32_t getTick() {
    struct timespec ts;
    clock_gettime( CLOCK_REALTIME, &ts );
    return ts.tv_nsec / 1000000 + ts.tv_sec * 1000;
}

int RxThreadFunction(void* pParam)
{
	// module access
	int	nFlag;
	int	nMask;
	int	nRet;
	BRVBIControl* pControl = (BRVBIControl*)pParam;

	// thread loop
	while(1)
	{
		nRet = pControl->m_StopEvent.CheckEvent(0);
		if(nRet)
		{
			break;
		}

		pControl->streamFunction();
	}

	// notify thread completion
	pControl->m_StoppedEvent.SetEvent(1);
	return 0;
}

void BRVBIControl::streamFunction() {
	// Stream function used in RX thread
	// DO NOT CALL THIS FUNCTION FROM OUTSIDE!

	BRVBIDataMessage	rxMessage;
	int 			nRet;
	int 			nP;
	timeval 		sTimeout;
	fd_set 			sReadSet;

	sTimeout.tv_sec = 0;
	sTimeout.tv_usec = 200000;
	// prepare the select command
	FD_ZERO(&sReadSet);
	FD_SET(m_nTransferSocket, &sReadSet);
	// select command
	nRet = select(m_nTransferSocket + 1, &sReadSet, NULL, NULL, &sTimeout);
	if (nRet != 1) // means error
	{
		std::cout << "Data reception failed" << std::endl;
	}

	// read operation
	nRet = recv(m_nTransferSocket, &m_StreamBuffer[m_nStreamBufferSize], 32768, 0);
	// all done
	if (nRet > 0) {
		// extraction loop
		nP = 0;
		m_nStreamBufferSize += nRet;
		while (1) {
			int nC = rxMessage.Extract(m_nStreamBufferSize - nP, &m_StreamBuffer[nP]);
			if (nC < 1)
				break;
			{
				std::lock_guard<std::mutex> localLock(m_StreamMutex);
				// copy operation
				m_StreamBlockScaling[m_nStreamBlockWritePosition] = rxMessage.m_dScaling;
				m_StreamBlockIQTime[m_nStreamBlockWritePosition] = rxMessage.m_ulAbsTime;
				m_StreamBlockFlag[m_nStreamBlockWritePosition] = rxMessage.m_wFlags;
				memcpy(&m_AcquisitionBuffer[m_StreamBlockSamplePos[m_nStreamBlockWritePosition]],
					rxMessage.m_SampleData,
					2 * rxMessage.m_wSampleCount * sizeof(short));
				m_nStreamBlockWritePosition = (m_nStreamBlockWritePosition + 1) % m_nStreamBlockAlloc;
				if (m_nStreamBlockSize == m_nStreamBlockAlloc) {
					m_nStreamReset = 1;
					m_nStreamBlockReadPosition = m_nStreamBlockWritePosition;// last valid block
				} else
				m_nStreamBlockSize++;
			}
			// prepare next packet
			nP += nC;
		}
		// re-arrange the stream buffer
		if (nP == m_nStreamBufferSize)
			m_nStreamBufferSize = 0;
		else {
			m_nStreamBufferSize = m_nStreamBufferSize - nP;
			memcpy(m_StreamBuffer, &m_StreamBuffer[nP], m_nStreamBufferSize);
		}
	}
}

int BRVBIControl::processCommand(int nSocket, BRVBIControlMessage* pMsg)
{
	// handles the command processing including response
	int			nRet;
	int			nPos;
	int			nSize;
	uint8_t		buffer[8192];
	uint32_t	dwTime;
	uint32_t	dwIndex;
	uint32_t 	dwTimeout;
	timeval		sTimeout;
	fd_set		sWriteSet;
	fd_set		sReadSet;

	// assemble the command
	nSize	= pMsg->Assemble(buffer);
	// send operation
	// create the timeout
	sTimeout.tv_sec	= 0;
	sTimeout.tv_usec	= 200000;
	// prepare the select command
	FD_ZERO(&sWriteSet);
	FD_SET(nSocket, &sWriteSet);
	// select command
	nRet	= select(nSocket + 1, NULL, &sWriteSet, NULL, &sTimeout);
	if (nRet != 1) {
		std::cout << "Process command: select failed" << std::endl;
		return -1;
	}
	// write operation
	nRet	= send(nSocket, buffer, nSize, 0);
	if (nRet != nSize){
		std::cout << "Process command: send failed" << std::endl;
		return -1;
	}

	// keep the index of the message
	dwIndex	= pMsg->m_dwIndex;
	// receive loop
	dwTime	= getTick() + 2000;
	while (getTick() < dwTime) {
		// reception of a single message on the TCP port, returns the received size (0 in case of error)
		dwTimeout = 500;
		// assemble the timeout
		sTimeout.tv_sec	= dwTimeout / 1000;
		sTimeout.tv_usec	= 1000 * (dwTimeout % 1000);
		// prepare the select command
		FD_ZERO(&sReadSet);
		FD_SET(nSocket, &sReadSet);
		// select command
		nRet	= select(nSocket + 1, &sReadSet, NULL, NULL, &sTimeout);
		if (nRet != 1)
		{
				std::cout << "Process command: select failed at receive" << std::endl;
				//continue;
				return 0;		// means error
		}
		// read operation
		nRet	= recv(nSocket, buffer, 8192, 0);
		nRet = nRet < 0 ? 0 :nRet;
		nPos	= 0;
		while(nPos < nRet) {
			nSize	= pMsg->Extract(nRet-nPos, &buffer[nPos]);
			if (nSize >= 0) {
				if (pMsg->m_dwIndex == dwIndex) {
					switch (pMsg->m_bMessageType) {
						case BRVBI_MESSAGE_TYPE_RX_ACK:
							return 0;
						case BRVBI_MESSAGE_TYPE_RX_ERROR:
							std::cout << "Process command: error code returned: " << pMsg->m_dwRxError << std::endl;
							return -1;
					}
				}
				nPos	+= nSize;
			}
			else
				nPos	= nRet;	// force exit
		}
	}

	return -1;
}

void BRVBIControl::cleanUp()
{
	// close any open sockets
	if(m_nControlSocket != -1)
		close(m_nControlSocket);
	m_nControlSocket = -1;
	if(m_nTransferSocket != -1)
		close(m_nTransferSocket);
	m_nTransferSocket = -1;

	m_dwMessageIndex = 0;
	m_nStreamRunning	= 0;
}

int BRVBIControl::init(std::string ipStr, double dCenterFrequencyInMHz, uint32_t dwSamplingClockInHz, uint32_t dwAcquisitionSize) {
	SInputConfiguration sInputConfig;
	sockaddr	socketAddr;
	uint16_t	wPort;
	uint32_t	dwAddr;
	uint32_t 	dwAddress;
	uint32_t 	dwStartTick;
	int 		nRet;
	int 		nSize;
	int 		nP;
	int 		nRxBufSize;
	int 		nOptLen;
	timeval 	sTimeout;
	fd_set 		workSet;

	// if streaming is running, stop first
	if (m_nStreamRunning)
		stopStream();

	// clean up all existing connections and buffers
	cleanUp();

	// create the control socket
	m_nControlSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (m_nControlSocket == -1) {
		return -1;
	}

	// connect to the receiver
	inet_pton(AF_INET, ipStr.c_str(), &dwAddress);
	wPort = BRVBI_TCP_DEFAULT_PORT;
	dwAddr = dwAddress;
	wPort = htons(wPort);
	socketAddr.sa_family = AF_INET;
	memcpy(socketAddr.sa_data, &wPort, 2);
	memcpy(&socketAddr.sa_data[2], &dwAddr, 4);
	// connect attempt
	nRet = connect(m_nControlSocket, &socketAddr, sizeof(socketAddr));
	if (nRet) {
		std::cout << "Connection error: " << errno << std::endl;
		cleanUp();
		return -1;
	}

	// set HW configuration
	BRVBIControlMessage msg;
	msg.m_dwIndex = m_dwMessageIndex++;
	msg.m_bMessageType = BRVBI_MESSAGE_TYPE_HW_CONFIGURE;
	msg.m_bExtRef = 0;
	msg.m_bGPSSync = 0;
	msg.m_wUDPPort = 0;		// use default port
	nRet = processCommand(m_nControlSocket, &msg);
	if (nRet) {
		std::cout << "Config failed" << std::endl;
		cleanUp();
		return -1;
	}

	// set Stream configuration
	msg.m_dwIndex = m_dwMessageIndex++;
	msg.m_bMessageType = BRVBI_MESSAGE_TYPE_STREAM_CONFIGURE;
	msg.m_dwAcquisitionClockInHz = dwSamplingClockInHz;
	msg.m_dwSamplingClockInHz = dwSamplingClockInHz;
	msg.m_dwBandwidthInHz = dwSamplingClockInHz;		// no resampling
	msg.m_dwBurstSize = dwAcquisitionSize;
	msg.m_dCenterFrequencyInMHz = dCenterFrequencyInMHz;
	sInputConfig.nLFPath = 0;
	sInputConfig.nRX1Input = 0;
	sInputConfig.nAutoScaling = 1;
	sInputConfig.nPreamp = 1;
	sInputConfig.nSingleScaling = 0;
	sInputConfig.bCoarseAttenuation = 0;
	sInputConfig.bFineAttenuation = 0;
	msg.m_dwInputConfiguration = BRVBIHelper::AssembleInputConfiguration(sInputConfig);
	msg.m_ulStartTime = 0;
	nRet = processCommand(m_nControlSocket, &msg);
	if (nRet) {
		std::cout << "Setup failed" << std::endl;
		cleanUp();
		return -1;
	}

	// set up buffers
	m_nStreamBlockSizeInSamples = 2048;
	nSize = 3 * dwAcquisitionSize / m_nStreamBlockSizeInSamples;
	m_nStreamBlockAlloc = nSize;
	m_StreamBlockFlag = (int*) calloc(m_nStreamBlockAlloc, sizeof(int));
	m_StreamBlockIQTime = (uint64_t*) calloc(m_nStreamBlockAlloc, sizeof(uint64_t));
	m_StreamBlockSamplePos = (int*) calloc(m_nStreamBlockAlloc, sizeof(int));
	m_StreamBlockScaling = (double*) calloc(m_nStreamBlockAlloc, sizeof(double));
	m_dwAcquisitionBufferAlloc = m_nStreamBlockAlloc * m_nStreamBlockSizeInSamples * 2;		// iq pairs
	m_AcquisitionBuffer = (short*) calloc(m_dwAcquisitionBufferAlloc, sizeof(short));

	// fill the sample position array
	nP = 0;
	for (int i = 0; i < m_nStreamBlockAlloc; i++) {
		m_StreamBlockSamplePos[i] = nP;
		nP += 2 * m_nStreamBlockSizeInSamples;	// iq pairs;
	}

	// prepare the socket for data transfer
	m_nTransferSocket = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, IPPROTO_TCP);
	if (m_nTransferSocket == -1) {
		std::cout << "Transfer Socket setup failed" << std::endl;
		cleanUp();
		return -1;
	}

	// set the rx buffer on the transfer socket
	nRxBufSize = 8388608;
	nOptLen = sizeof(int);
	nRet = setsockopt(m_nTransferSocket, SOL_SOCKET, SO_RCVBUF,	(char*) &nRxBufSize, nOptLen);
	if (nRet) {
		std::cout << "Transfer Socket option setting failed" << std::endl;
		cleanUp();
		return -1;
	}

	// connect to the transfer port of the receiver
	dwAddr = dwAddress;
	wPort = BRVBI_UDP_DEFAULT_PORT;
	wPort = htons(wPort);
	socketAddr.sa_family = AF_INET;
	memcpy(socketAddr.sa_data, &wPort, 2);
	memcpy(&socketAddr.sa_data[2], &dwAddr, 4);
	nRet = connect(m_nTransferSocket, &socketAddr, sizeof(socketAddr));
	if (nRet == -1) {
		nRet = errno;
		if (nRet == EINPROGRESS) {
			// since the connection is not set up immediately (SOCK_NONBLOCK), allow some time to complete
			dwStartTick = getTick();
			do {
				sTimeout.tv_sec = 0;
				sTimeout.tv_usec = 200000;
				FD_ZERO(&workSet);
				FD_SET(m_nTransferSocket, &workSet);
				int nSelect = select(m_nTransferSocket + 1, NULL, &workSet, NULL, &sTimeout);
				if (nSelect > 0) {
					nRet = 0;
					break;
				}
			} while (getTick() < (dwStartTick + 5000));
		}

		if (nRet != 0) {
			std::cout << "Transfer socket preparation failed: " << nRet	<< std::endl;
			cleanUp();
			return -1;
		}
	}

	return 0;
}

int BRVBIControl::startStream()
{
	BRVBIControlMessage msg;
	int nRet;

	// if stream is already running, there's nothing to do
	if (m_nStreamRunning)
		return 0;

	msg.m_dwIndex		= m_dwMessageIndex++;
	msg.m_bMessageType	= BRVBI_MESSAGE_TYPE_STREAM_START;
	nRet = processCommand(m_nControlSocket, &msg);
	if (nRet) {
		std::cout << "Start Streaming failed" << std::endl;
		return -1;
	}

	// reset all information
	m_StopEvent.ResetEvent();
	m_StoppedEvent.ResetEvent();

	m_nStreamBlockSize				= 0;
	m_nStreamBlockReadPosition		= 0;
	m_nStreamBlockWritePosition	= 0;
	m_nStreamBufferSize				= 0;
	m_nStreamReset						= 0;

	m_pRXStreamThread = new std::thread(RxThreadFunction, this);
	m_pRXStreamThread->detach();
	if(m_pRXStreamThread) {
		return 0;
	}
	else
		return -1;
}

int BRVBIControl::stopStream()
{
	BRVBIControlMessage msg;
	int nRet;

	if (m_pRXStreamThread) {
		// set the event for completion
		m_StopEvent.SetEvent(1);
		// wait for notification
		nRet = m_StoppedEvent.CheckEvent(2000);
		if(!nRet)
		{
			std::cout << "Stopping RX Thread failed" << std::endl;
		}

		// reset all information
		m_pRXStreamThread	= nullptr;
		m_nStreamRunning	= 0;

		// stop streaming
		msg.m_dwIndex			= m_dwMessageIndex++;
		msg.m_bMessageType		= BRVBI_MESSAGE_TYPE_STREAM_STOP;
		msg.m_dwIdentification	= 12345;	// stream
		msg.m_dwMeasID			= 0;
		nRet = processCommand(m_nControlSocket, &msg);
		if (nRet) {
			std::cout << "Stop Streaming failed" << std::endl;
			return -1;
		}
	}

	return 0;
}

uint32_t BRVBIControl::getStreamData(uint32_t dwTimeout, uint32_t dwAcquisitionSize, short* iData, short* qData, int nFlagMask)
{
	// retrieves the samples from the last measurement, returns the data size (0 in case of error)
	int			i;
	int			nBlockWanted;
	int			nBlockReceived;
	int			nDiff;
	uint32_t	dwS;
	uint32_t	dwTime;
	uint32_t	dwL;
	uint32_t	dwP;
	uint32_t	dw;
	short		iqData[4096];
	double		dIQTime;
	uint64_t	ulIQTime;
	double		dScaling;
	int			nFlag;
	double*		scalingArray;
	double		dMinScale;
	double		dMaxScale;
	double		dDiff;
	uint64_t*	timeArray;
	timeval 	tv;

	// set the time bound
	dwTime	= getTick() + dwTimeout;
	// get the block count
	if (dwAcquisitionSize % m_nStreamBlockSizeInSamples)
		nBlockWanted	= dwAcquisitionSize/m_nStreamBlockSizeInSamples + 1;
	else
		nBlockWanted	= dwAcquisitionSize / m_nStreamBlockSizeInSamples;
	// prepare the containers to hold the information
	scalingArray	= (double*) calloc(nBlockWanted, sizeof(double));
	timeArray		= (uint64_t*) calloc(nBlockWanted, sizeof(uint64_t));
	// adjust the positions for spot mode
	// block copy loop
	dwS				= 0;
	nBlockReceived	= 0;
	while (dwS < dwAcquisitionSize) {
		// lock section
		{
			std::lock_guard<std::mutex> localLock(m_StreamMutex);
			nFlag	= -1;		// skip indicator
			if (m_nStreamReset) {
				dwS				= 0;
				nBlockReceived	= 0;
				m_nStreamReset	= 0;
			}
			if (m_nStreamBlockSize) {
				ulIQTime	= m_StreamBlockIQTime[m_nStreamBlockReadPosition];
				dScaling	= m_StreamBlockScaling[m_nStreamBlockReadPosition];
				nFlag		= m_StreamBlockFlag[m_nStreamBlockReadPosition] & nFlagMask;
				memcpy (iqData, &m_AcquisitionBuffer[m_StreamBlockSamplePos[m_nStreamBlockReadPosition]], 2*m_nStreamBlockSizeInSamples*sizeof(short));
				m_nStreamBlockReadPosition	= (m_nStreamBlockReadPosition + 1) % m_nStreamBlockAlloc;
				m_nStreamBlockSize--;
			}
			else {
				// check for timeout
				if (getTick() > dwTime) {
					// clean up
					free (scalingArray);
					free (timeArray);
					// report error
					return 0;
				}
			}
		}
		// handle the flag information
		if (nFlag > 0) {				// negative value indicates missing information
			dwS				= 0;		// remove all existing information in any case
			nBlockReceived	= 0;
			nFlag				%= BRVBI_DATA_PACKET_FLAG_STREAMINGBREAK;		// message with overflow indicator contains valid information
		}
		// check for valid information
		if (nFlag == 0) {
			// meta information
			scalingArray[nBlockReceived]	= dScaling;
			timeArray[nBlockReceived]		= ulIQTime;
			// copy operation
			dwL	= dwAcquisitionSize - dwS;
			dwL	= std::min((uint32_t)2048, dwL);
			dwP	= 0;
			for (dw=0; dw<dwL; dw++) {
				iData[dwS]	= iqData[dwP++];
				qData[dwS]	= iqData[dwP++];
				dwS++;
			}
			// next block
			nBlockReceived++;
		}
		else {
			// allow idle time (10ms)
			tv.tv_sec = 0;
			tv.tv_usec = 10000;
			select(0, NULL, NULL, NULL, &tv);
		}
	}
	// scaling handling
	dScaling		= scalingArray[0];
	dMinScale	= dScaling;
	dMaxScale	= dScaling;
	for (i=1; i<nBlockReceived; i++) {
		dScaling		= scalingArray[i];
		dMaxScale	= std::max(dMaxScale, dScaling);
		dMinScale	= std::min(dMinScale, dScaling);
	}
	if (dMaxScale - dMinScale) {
		dScaling	= dMaxScale;
		for (i=0; i<nBlockReceived; i++) {
			dDiff	= scalingArray[i] - dScaling;
			if (dDiff) {
				dDiff	= pow(10.0, dDiff/20.0);
				dwP	= i * 2048;
				dwL	= std::min(dwP + 2048, dwAcquisitionSize);
				while (dwP < dwL) {
					iData[dwP]	= short(iData[dwP]*dDiff + 0.5);
					qData[dwP]	= short(qData[dwP]*dDiff + 0.5);
					dwP++;
				}
			}
		}
	}
	// clean up
	free (scalingArray);
	free (timeArray);
	// all done
	return dwAcquisitionSize;
}

