/*
 * BRVBIControl.h
 *
 *  Created on: Sep 30, 2022
 *      Author: PrecisionWave AG
 */

#pragma once
#include "BRVBIHelper.h"
#include <unistd.h>
#include <stdint.h>
#include <iostream>
#include <thread>
#include "WaitableEvent.h"

class BRVBIControl {
public:
	BRVBIControl();
	virtual ~BRVBIControl();
	
	int checkParams(double dCenterFrequencyInMHz, uint32_t dwSamplingClockInHz, uint32_t dwRxSize);
	int init(std::string ipStr, double dCenterFrequencyInMHz, uint32_t dwSamplingClockInHz, uint32_t dwAcquisitionSize);
	int startStream();
	int stopStream();
	uint32_t getStreamData(uint32_t dwTimeout, uint32_t dwRxSize, short* iqData);

public:
	// public because used in thread function
	void streamFunction();
	WaitableEvent m_StopEvent;
	WaitableEvent m_StoppedEvent;

private:
	int processCommand(int nSocket, BRVBIControlMessage* pMsg);
	bool tryGetDataFromBuffer(short* pData);
	void cleanUp();

	int 		m_nControlSocket;
	int 		m_nTransferSocket;
	uint32_t 	m_dwMessageIndex;
	uint8_t		m_StreamBuffer[65536];
	int			m_nStreamBufferSize;
	int			m_nStreamBlockSizeInSamples;
	int			m_nStreamBlockAlloc;
	int*		m_StreamBlockSamplePos;
	uint32_t 	m_dwAcquisitionBufferAlloc;
	short*		m_AcquisitionBuffer;
	std::thread* m_pRXStreamThread;
	std::mutex 	m_StreamMutex;

	int			m_nStreamBlockWritePosition;
	int			m_nStreamBlockReadPosition;
	int			m_nStreamBlockSize;
	int			m_nStreamRunning;
};
