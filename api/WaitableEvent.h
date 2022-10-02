/*
 * WaitableEvent.h
 *
 *  Created on: Sep 30, 2022
 *      Author: PrecisionWave AG
 */

#pragma once
#include <mutex>
#include <condition_variable>
#include <unistd.h>

using namespace std::chrono_literals;

class WaitableEvent
{
public:
	WaitableEvent()
	{
		ResetEvent();
	}

	~WaitableEvent()
	{
	}

	void SetEvent(int nType)
	{
		std::lock_guard<std::mutex> localLock(m_Mutex);
		m_nReady = nType;
		m_cvCondVar.notify_one();
	}

	void ResetEvent()
	{
		m_Mutex.lock();
		m_nReady = 0;
		m_Mutex.unlock();
	}

	int CheckEvent(uint32_t dwTimeoutInMs)
	{
		std::unique_lock<std::mutex> localLock(m_Mutex);
		if (m_cvCondVar.wait_for(localLock, dwTimeoutInMs*1000us, [this] {return m_nReady>0;})) {
			m_nOut	= m_nReady;
			m_nReady = 0;
			return m_nOut;
		}
		else
			return 0;
	}

private:
	std::condition_variable	m_cvCondVar;
	std::mutex				m_Mutex;
	int						m_nReady;
	int						m_nOut;
};
