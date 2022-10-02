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

#define IQ_BUFFER_SIZE 100000

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
	short			iData[IQ_BUFFER_SIZE];
	short			qData[IQ_BUFFER_SIZE];
	short			iqData[IQ_BUFFER_SIZE*2];

	// command line arguments handling
	cxxopts::Options options("BRVBIExample", "BRVBI IQ Stream Demo");
	options.add_options()
		("i,ip-address", "IPv4 Address of the receiver", cxxopts::value<std::string>())
		("c,center-frequency", "Center Frequency [MHz]", cxxopts::value<double>())
		("s,sampling-clock", "Sampling Clock [Hz]", cxxopts::value<uint32_t>())
		("a,acquisition-size", "Acquisition Size [#Samples]", cxxopts::value<uint32_t>())
		("f,iq-file", "Target IQ data file", cxxopts::value<std::string>())
	    ("h,help", "Print usage");

	auto arguments = options.parse(argc, argv);
	if (arguments.count("help"))
	{
		std::cout << options.help() << std::endl;
		exit(EXIT_SUCCESS);
	}

	if (!arguments.count("center-frequency") || !arguments.count("sampling-clock") || !arguments.count("acquisition-size") || !arguments.count("iq-file"))
	{
		std::cout << "Missing argument(s)!" << std::endl;
		exit(EXIT_FAILURE);
	}

	// connect to receiver
	nRet = control.init(
			arguments["ip-address"].as<std::string>(),
			arguments["center-frequency"].as<double>(),
			arguments["sampling-clock"].as<uint32_t>(),
			arguments["acquisition-size"].as<uint32_t>());
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
	std::ofstream outputFile(arguments["iq-file"].as<std::string>() , std::ios::binary );

	// set up cancellation option
	cancelFlag = false;
	signal(SIGINT, CancelHandler);

	// start RX
	dwMessageCount = 0;
	std::cout << "Data reception started, press 'Ctrl+C' to terminate..." << std::endl;
	while(!cancelFlag)
	{
		nRet = control.getStreamData(500, IQ_BUFFER_SIZE, iData, qData);
		if(nRet == IQ_BUFFER_SIZE)
		{
			// interleave IQ
			for(int i = 0; i < IQ_BUFFER_SIZE; i++)
			{
				iqData[2*i] = iData[i];
				iqData[2*i+1] = qData[i];
			}

			outputFile.write((char const*)&iqData[0], IQ_BUFFER_SIZE*2*sizeof(short));

			dwMessageCount++;
			std::cout << "Acquisitions received: " << dwMessageCount << '\r'	<< std::flush;
		}
		else
		{
			std::cout << "Data RX failed" << std::endl;
		}
	}

	// stop stream
	nRet = control.stopStream();
	if(nRet) {
		std::cout << "Stop stream failed!" << std::endl;
		exit(EXIT_FAILURE);
	}

	// close iq file
	outputFile.close();

	// reset cancellation procedure to default
	signal(SIGINT, SIG_DFL);

	std::cout << "Terminated successfully" << std::endl;
	return EXIT_SUCCESS;
}
