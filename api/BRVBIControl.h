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
	int init(std::string ipStr, double dCenterFrequencyInMHz, uint32_t dwSamplingClockInHz, uint32_t dwAcquisitionSize);
	int startStream();
	int stopStream();
	uint32_t getStreamData(uint32_t dwTimeout, uint32_t dwAcquisitionSize, short* iqData, int nFlagMask = 7);

public:
	// public because used in thread function
	void streamFunction();
	WaitableEvent m_StopEvent;
	WaitableEvent m_StoppedEvent;

private:
	int processCommand(int nSocket, BRVBIControlMessage* pMsg);
	void cleanUp();

	int 		m_nControlSocket;
	int 		m_nTransferSocket;
	uint32_t 	m_dwMessageIndex;
	uint8_t		m_StreamBuffer[65536];
	int			m_nStreamBufferSize;
	int			m_nStreamBlockSizeInSamples;
	int			m_nStreamBlockAlloc;
	int*		m_StreamBlockFlag;
	int*		m_StreamBlockSamplePos;
	double*		m_StreamBlockScaling;
	uint64_t*	m_StreamBlockIQTime;
	uint32_t 	m_dwAcquisitionBufferAlloc;
	short*		m_AcquisitionBuffer;
	std::thread* m_pRXStreamThread;
	std::mutex 	m_StreamMutex;

	int			m_nStreamBlockWritePosition;
	int			m_nStreamBlockReadPosition;
	int			m_nStreamBlockSize;
	int			m_nStreamReset;
	int			m_nStreamRunning;
};
