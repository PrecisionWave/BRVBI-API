/*
 * Example.cpp
 *
 *  Created on: Sep 30, 2022
 *      Author: PrecisionWave AG
 */

#include <csignal>
#include <fstream>
#include <iostream>
#include "cxxopts.hpp"
#include "api/BRVBIControl.h"

static bool cancelFlag;

void CancelHandler(int)
{
	std::cout << std::endl << "Abort..." << std::endl;
	cancelFlag = true;
}

int main(int argc, char **argv) {
	BRVBIControl 	control;
	int				nRet;
	uint32_t		dwMessageIndex;
	uint32_t 		dwMessageCount;
	short*			iqData;

	std::string 	ipAddress;
	std::string		iqFilename;
	double 			dCenterFrequencyMHz;
	uint32_t 		dwSamplingClock;
	uint32_t 		dwAcquisitionSize;
	uint32_t		dwIQDataSize;
	uint32_t		dwTimeout;

	// command line arguments handling
	cxxopts::Options options("BRVBI2File", "BRVBI IQ Stream Demo");
	options.add_options()
		("i,ip-address", "IPv4 Address of the receiver", cxxopts::value<std::string>())
		("c,center-frequency", "Center Frequency [MHz]", cxxopts::value<double>())
		("s,sampling-clock", "Sampling Clock [Hz]", cxxopts::value<uint32_t>())
		("a,acquisition-size", "Acquisition Size [#Samples]", cxxopts::value<uint32_t>())
		("f,iq-file", "Target IQ data file", cxxopts::value<std::string>())
		("b,block-size", "Block size for file writing [#Samples]", cxxopts::value<uint32_t>()->default_value("10000"))
	    ("h,help", "Print usage");

	try
	{
		auto arguments = options.parse(argc, argv);
		if (arguments.count("help"))
		{
			std::cout << options.help() << std::endl;
			exit(EXIT_SUCCESS);
		}

		ipAddress = arguments["ip-address"].as<std::string>();
		dCenterFrequencyMHz = arguments["center-frequency"].as<double>();
		dwSamplingClock = arguments["sampling-clock"].as<uint32_t>();
		dwAcquisitionSize = arguments["acquisition-size"].as<uint32_t>();
		iqFilename = arguments["iq-file"].as<std::string>();
		dwIQDataSize = arguments["block-size"].as<uint32_t>();
	}
	catch(cxxopts::OptionException& e)
	{
		std::cout << "Argument error:" << std::endl << e.what() << std::endl << std::endl << options.help() << std::endl;
		exit(EXIT_FAILURE);
	}

	std::cout << std::endl << "### BRVBI2File ###" << std::endl << std::endl;
	
	nRet = control.checkParams(dCenterFrequencyMHz, dwSamplingClock, dwIQDataSize);
	if(nRet)
	{
		std::cout << "Parameter check failed!" << std::endl;
		exit(EXIT_FAILURE);
	}

	// connect to receiver
	nRet = control.init(ipAddress, dCenterFrequencyMHz, dwSamplingClock, dwAcquisitionSize);
	if(nRet) {
		std::cout << "Initialization failed!" << std::endl;
		exit(EXIT_FAILURE);
	}
	std::cout << "Init complete" << std::endl;

	// start stream
	nRet = control.startStream();
	if(nRet) {
		std::cout << "Start stream failed!" << std::endl;
		exit(EXIT_FAILURE);
	}
	std::cout << "Streaming started" << std::endl;

	/////////////////////////////
	// Stream reception Loop
	/////////////////////////////

	// open file
	std::ofstream outputFile(iqFilename , std::ios::binary );

	// set up cancellation option
	cancelFlag = false;
	signal(SIGINT, CancelHandler);

	// prepare IQ data buffer
	iqData = (short*)calloc(dwIQDataSize*2, sizeof(short));

	// start RX
	dwMessageCount = 0;
	dwTimeout = dwIQDataSize < dwSamplingClock ? 2000 : (dwIQDataSize / dwSamplingClock + 2) * 1000;
	std::cout << "Data reception started, press 'Ctrl+C' to terminate..." << std::endl;
	while(!cancelFlag)
	{
		nRet = control.getStreamData(dwTimeout, dwIQDataSize, iqData);
		if(nRet == dwIQDataSize)
		{
			outputFile.write((const char*)&iqData[0], dwIQDataSize*2*sizeof(short));

			dwMessageCount++;
			if(!cancelFlag)
			{
				std::cout << "Data blocks written: " << dwMessageCount << '\r' << std::flush;
			}
		}
		if(nRet == 0)
			std::cout << "GetStreamData timeout" << std::endl;
	}

	std::cout << std::endl;
	
	// stop stream
	nRet = control.stopStream();
	if(nRet) {
		std::cout << "Stop stream failed!" << std::endl;
		exit(EXIT_FAILURE);
	}

	// close iq file
	outputFile.flush();
	outputFile.close();
	free(iqData);
	
	// reset cancellation procedure to default
	signal(SIGINT, SIG_DFL);

	std::cout << "Terminated successfully" << std::endl;
	return EXIT_SUCCESS;
}
