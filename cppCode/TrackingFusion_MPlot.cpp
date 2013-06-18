////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// This file is part of CAMPCom, a client/server based communication module   //
//                                                                            //
// Copyright (C) 2013 Alexander Schoch                                        //    
//                                                                            //    
///////////////////////////////////////////////////////////////////////////////

///
/// @file main.cpp
/// @brief Contains the a console application that can be used to controle the connector manually
/// @author Alexander Schoch
/// @date Mar 13,2013 - First creation
/// @package CAMPComConnector
///

#define CAMPCOM_DEF_OT
#define CAMPCOM_DEF_EM

#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <conio.h>

#include "engine.h"

#include "TTypeHandler.hpp"
#include "OTTypeHandler.hpp"
#include "EMTypeHandler.hpp"
#include "CAMPComClient.hpp"
#include "CampLog.hpp"

std::string addr = "127.0.0.1";

typedef boost::shared_ptr<boost::asio::ip::tcp::socket> socket_ptr;
using namespace campcom;
FILE *fileOpticalTracking, *fileEMTracking;

/// Polling flag
bool pollingDoContinue = true;

/// Polling mutex
boost::mutex pollingMutex;
boost::mutex EMTMutex;
boost::mutex OTMutex;

// global buffers for matlab plotting
double positionEMT[] = { -50, -50, -50 };
double orientationEMT[] = { 0, 0, 0, 0 };

double positionOT[] = { -100, -100, -100 };
double orientationOT[] = { 0, 0, 0, 0 };

bool OTOkay = true;
bool EMTOkay = true;
int sensor_number = 0;

double em_error = 0;
int em_sensorID_int = 0;


void printOutOpticalTracking(std::vector<unsigned char> &msg)
{

    static bool finished = false;

    if (finished)
    {
        return;
    }

	STD_LOG(logINFO) << "Optical Tracking data received.";
	
	{
	boost::mutex::scoped_lock OTtrackerlock(OTMutex);

	OptTrackingData td;
    TypeHandler<OptTrackingData>::deserializePayload(&msg[0],msg.size(),td);
		// lock shared variables
		if (abs(td.error) < 1)
		{
			OTOkay = true;

			if (pollingDoContinue == true)
			{
				positionOT[0] = static_cast<double>(td.position[0]);
				positionOT[1] = static_cast<double>(td.position[1]);
				positionOT[2] = static_cast<double>(td.position[2]);

				orientationOT[0] = static_cast<double>(td.orientation[0]);
				orientationOT[1] = static_cast<double>(td.orientation[1]);
				orientationOT[2] = static_cast<double>(td.orientation[2]);
				orientationOT[3] = static_cast<double>(td.orientation[3]);
			
			}
			else
			{
				STD_LOG(logINFO) << "OT Recording finished!";
				finished = true;
			}
		
		}
		else
		{
			OTOkay = false;
		}
	// end of lock
	}
}

void printOutEMTracking(std::vector<unsigned char> &msg)
{
    static bool finished = false;

    if (finished)
    {
        return;
    }

	STD_LOG(logINFO) << "EM Tracking data received.";

	{
	// lock shared variables
	boost::mutex::scoped_lock EMtrackerlock(EMTMutex);
	
	EMTrackingData td;
    TypeHandler<EMTrackingData>::deserializePayload(&msg[0],msg.size(),td);
	em_error = td.error;
	em_sensorID_int = td.sensorID;
	if (td.isValid)
	{
		if (td.error < 10)
		{
				
			EMTOkay = true;

			if (pollingDoContinue == true)
			{
			
				sensor_number = static_cast<int>(td.sensorID);

				positionEMT[0] = static_cast<double>(td.position[0]);
				positionEMT[1] = static_cast<double>(td.position[1]);
				positionEMT[2] = static_cast<double>(td.position[2]);

				
				orientationEMT[0] = static_cast<double>(td.orientation[0]);
				orientationEMT[1] = static_cast<double>(td.orientation[1]);
				orientationEMT[2] = static_cast<double>(td.orientation[2]);
				orientationEMT[3] = static_cast<double>(td.orientation[3]);
			
			}
			else
			{
				STD_LOG(logINFO) << "EMT Recording finished!";
				finished = true;
			}
			
		}
		else
		{
			EMTOkay = false;
		}
	}
	// end of lock
	}
}


void dummyPrint(std::vector<unsigned char> &msg)
{
    STD_LOG(logDEBUG) <<  "New data arrived...";
}


int main(int argc, char* argv[]){

    STD_LOG(logINFO) << "Starting client...";

    CAMPComClient client("TrackingFusionClient",Device_TrackingFusion,addr);

    bool con = client.connect();

    if (con)
    {
        STD_LOG(logDEBUG) << "Client connected. Starting suscriptions...";
    } 
    else
    {
        STD_LOG(logERROR) << "Couldn't connect the client. Exiting program now...";
        return 0;
    }

	// local buffers for matlab plotting
	double positionEMT_main[] = { -50, -50, -50 };
	double orientationEMT_main[] = { 0, 0, 0, 0 };

	double positionOT_main[] = { -100, -100, -100 };
	double orientationOT_main[] = { 0, 0, 0, 0 };

	bool OTOkay_main = true;
	bool EMTOkay_main = true;
	int sensor_number_main = 0;

	// Matlab pointer variables
	Engine *ep;
	mxArray *pos_ptr = NULL;
	mxArray *orient_ptr = NULL;

	/*
	 * Start the MATLAB engine 
	 */
	if (!(ep = engOpen(NULL))) {
		MessageBox ((HWND)NULL, (LPSTR)"Can't start MATLAB engine", 
			(LPSTR) "Engwindemo.c", MB_OK);
		exit(-1);
	}

	/* 
	 * Set target structure for Matlab data pointers
	 */
	pos_ptr = mxCreateDoubleMatrix(1, 3, mxREAL);
	orient_ptr = mxCreateDoubleMatrix(1, 4, mxREAL);

	/*
	 * Load important .mat files, Execute environment plot
	 */
	engEvalString(ep, "prepareMatlabData");

	pollingMutex.lock();
	pollingDoContinue = true;
	pollingMutex.unlock();
    // Subscribe to OT
	con &= client.subscribe(Type_OptTracking,&printOutOpticalTracking);

	if (con)
    {
        STD_LOG(logDEBUG) << "Client subscribed to OT...";
    } 
    else
    {
        STD_LOG(logWARN) << "Failed attempt when subscribing to OT...";
    }

	// MPlot_OpticalTracking(ep, pos_ptr);

	// Subscribe to EMT
	con &= client.subscribe(Type_EMTracking,&printOutEMTracking);

	if (con)
    {
        STD_LOG(logDEBUG) << "Client subscribed to EMT...";
    } 
    else
    {
        STD_LOG(logWARN) << "Failed attempt when subscribing to EMT...";
    }

	// MPlot_EMTracking(ep, pos_ptr);


	STD_LOG(logALWAYS) << "Press ESC to stop the tracking...";

	//while(_getch() != 27)
	bool exitcondition=false;
	char c;
	while(!exitcondition)
	{
		if(kbhit()) // only when a key is pressed
		{ 
			c = getch(); // does not need to wait for input, key was already pressed
			if(c == 27)
			{
				exitcondition = true;
			}
		}
		{
		// lock and read shared variables
		boost::mutex::scoped_lock EMtrackerlock(EMTMutex);
		boost::mutex::scoped_lock OTtrackerlock(OTMutex);
			
		// copy global values to local variables
		std::copy(std::begin(positionEMT), std::end(positionEMT), std::begin(positionEMT_main));
		std::copy(std::begin(orientationEMT), std::end(orientationEMT), std::begin(orientationEMT_main));

		std::copy(std::begin(positionOT), std::end(positionOT), std::begin(positionOT_main));
		std::copy(std::begin(orientationOT), std::end(orientationOT), std::begin(orientationOT_main));

		OTOkay_main = OTOkay;
		EMTOkay_main = EMTOkay;
		sensor_number_main = sensor_number;

		if (EMTOkay_main)
		{
		std::cout << "locked data: SensorID " << sensor_number_main << " Position: " << positionEMT_main[0] << " " << positionEMT_main[1] << " " << positionEMT_main[2] << std::endl;
		std::cout << " OKAY? " << EMTOkay_main << " Error: " << em_error << " simple sensorID: " << em_sensorID_int << std::endl;
		}
		}
			

		if (OTOkay_main)
		{
			memcpy((void *) mxGetPr(pos_ptr), (void *) positionOT_main, 3*sizeof(double));
			engPutVariable(ep, "positionOT_OCS", pos_ptr);

			memcpy((void *) mxGetPr(orient_ptr), (void *) orientationOT_main, 4*sizeof(double));
			engPutVariable(ep, "orientationOT_OCS", orient_ptr);

			engEvalString(ep, "plotByOT");
		}
		else if (!OTOkay_main && EMTOkay_main) // it emt data arrive but optical is not available
		{
			memcpy((void *) mxGetPr(pos_ptr), (void *) positionEMT_main, 3*sizeof(double));
			engPutVariable(ep, "positionEMT", pos_ptr);

			memcpy((void *) mxGetPr(orient_ptr), (void *) orientationEMT_main, 4*sizeof(double));
			engPutVariable(ep, "orientationEMT", orient_ptr);

			switch(sensor_number_main)
			{
			//case 0:
				// use EMT1 to map it to missing OT
				//engEvalString(ep, "plotByEMT1");

			case 1:
				// use EMT2 to map it to missing OT
				engEvalString(ep, "plotByEMT2");

			//case 2:
			//	engEvalString(ep, "plotByEMT3");
			}
		}
		else if (!OTOkay_main && !EMTOkay_main && sensor_number_main == 0)// plot a red sphere at last OT position if no tracking input is available
		{
			engEvalString(ep, "plotRedSphere");
		}

		Sleep(50); // 50ms for maximum 20Hz update rate of the Matlab plot
	}


	pollingMutex.lock();
	pollingDoContinue = false;
	pollingMutex.unlock();
	Sleep(1);

	con &= client.unsubscribe(Type_OptTracking);
	if (con)
    {
        STD_LOG(logDEBUG) << "Client unsubscribed from OT...";
    } 
    else
    {
        STD_LOG(logWARN) << "Failed attempt when to subscribe to OT...";
    }
	con &= client.unsubscribe(Type_EMTracking);
	if (con)
    {
        STD_LOG(logDEBUG) << "Client unsubscribed from EMT...";
    } 
    else
    {
        STD_LOG(logWARN) << "Failed attempt when unsubscribing from EMT...";
    }

	return 0;
}
