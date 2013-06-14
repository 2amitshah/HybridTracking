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

void printOutOpticalTracking(std::vector<unsigned char> &msg)
{
	//static bool OTOkay = true;
	//static double positionOT[] = { 0, 0, 0 };
	
	static int numbSamples = 0;
    static bool finished = false;

    if (finished)
    {
        return;
    }

	STD_LOG(logINFO) << "Optical Tracking data received.";
	
	OptTrackingData td;
    TypeHandler<OptTrackingData>::deserializePayload(&msg[0],msg.size(),td);
	{
	boost::mutex::scoped_lock OTtrackerlock(OTMutex);
		if (abs(td.error) < 1)
		{
		
			OTOkay = true;
			//std::stringstream ss;
			//ss << "Position: " << td.position << " Orientation: " << td.orientation << " Time: " << td.timestamp;
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
	}
}

void printOutEMTracking(std::vector<unsigned char> &msg)
{

	//static double positionEMT[] = { 0, 0, 0 };
	//static bool EMTOkay = true;
	//static int sensor_number = 0;

	static int numbSamples = 0;
    static bool finished = false;

    if (finished)
    {
        return;
    }

	STD_LOG(logINFO) << "EM Tracking data received.";

	EMTrackingData td;
    TypeHandler<EMTrackingData>::deserializePayload(&msg[0],msg.size(),td);

	//std::cout << "Validity of EMT sensor: " << td.isValid << std::endl;
	{
	boost::mutex::scoped_lock EMtrackerlock(EMTMutex);
		if (abs(td.error) < 10)
		{
		
		
			EMTOkay = true;
			//std::stringstream ss;

			//char id = td.sensorID;
			//int int_id = static_cast<int>(id);
			//std::cout << "id: " << int_id << " " << toolInformation.position << std::endl;
			//std::cout << "sensor_id: " << td.sensorID << " " << static_cast<int>(td.sensorID)  << " " << static_cast<unsigned>(td.sensorID) << std::endl;
			//ss << "SensorID" << int_id << " Position: " << td.position << " Orientation: " << td.orientation << " Time: " << td.timestamp;
        
			//ss <<  "SensorID: " << static_cast<int>(td.sensorID) << " Position: " << td.position << " Orientation: " << td.orientation << " Time: " << td.timestamp;
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
			
				//std::cout << "td data: SensorID " << int_id << " Position: " << td.position << std::endl;
				//std::cout << "our data: SensorID " << sensor_number << " Position: " << positionEMT[0] << " " << positionEMT[1] << " " << positionEMT[2] << std::endl;

				//fprintf(fileEMTracking,"%s\n",ss.str().c_str());fflush(fileEMTracking);
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


	Engine *ep;
	mxArray *pos_ptr = NULL;
	mxArray *orient_ptr = NULL;
	//static double positionEMT[] = { 50, 50, 50 };
	//static double positionOT[] = { 0, 0, 0 };
	//static bool OTOkay = true;
	//static bool EMTOkay = true;
	//static int sensor_number;

	/*
	 * Start the MATLAB engine 
	 */
	if (!(ep = engOpen(NULL))) {
		MessageBox ((HWND)NULL, (LPSTR)"Can't start MATLAB engine", 
			(LPSTR) "Engwindemo.c", MB_OK);
		exit(-1);
	}
	/*
	 * Load important .mat files
	 */
	engEvalString(ep, "load(which('H_OT_to_EMT.mat')); load(which('Y.mat')); c = colormap('lines'); [x,y,z] = sphere(20); load(which('H_EMT1_to_EMT2.mat'));");// load(which('H_EMT1_to_EMT3.mat'));");

	// as long as we dont have the matrix yet
	//engEvalString(ep, "H_EMT1_to_EMT2 = H_OT_to_EMT");
	/* 
	 * Create a variable for our data
	 */
	pos_ptr = mxCreateDoubleMatrix(1, 3, mxREAL);
	orient_ptr = mxCreateDoubleMatrix(1, 4, mxREAL);

	/*
	 * Execute environment plot
	 */
	engEvalString(ep, "realtime_plot_figure = figure('Position', get(0,'ScreenSize')); plotEnvironment(realtime_plot_figure, [], Y); view(3)" );



    // Creates datafiles
	const char *filenameOT = "OpticalTracking.txt";
	const char *filenameEMT = "EMTracking.txt";
	fopen_s(&fileOpticalTracking,filenameOT,"w");
	fopen_s(&fileEMTracking,filenameEMT, "w");

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

	while(true)
	{
		//{
			//boost::mutex::scoped_lock trackerlock(pollingMutex);
			
			//static double positionEMT[3];
			//static double positionOT[3];
			//static int sensor_number;

			//std::cout << OTOkay << " " << EMTOkay << std::endl;
			{
			boost::mutex::scoped_lock EMtrackerlock(EMTMutex);
			boost::mutex::scoped_lock OTtrackerlock(OTMutex);
			
			std::cout << "locked data: SensorID " << sensor_number << " Position: " << positionEMT[0] << " " << positionEMT[1] << " " << positionEMT[2] << " OKAY? " << EMTOkay << std::endl;

			if (OTOkay)
			{

				//double* pa = mxGetPr(pos_ptr);
				//
				//	memcpy((void *) pa, (void *) positionEMT, sizeof(positionEMT));
				//	engPutVariable(ep, "position", pos_ptr);

				//	//engEvalString(ep, "if exist('emt1Obj', 'var'), delete(emt1Obj); end");
				//	//engEvalString(ep, "hold on; emt1Obj = plot3(position(1), position(2), position(3), 'x', 'Color', c(2,:) ); hold off;");

				//	switch(sensor_number)
				//	{
				//	case 0:
				//		engEvalString(ep, "if exist('emt1Obj', 'var'), delete(emt1Obj); end");
				//		engEvalString(ep, "hold on; emt1Obj = plot3(position(1), position(2), position(3), 'x', 'Color', c(2,:) ); hold off;");
				//	case 1:
				//		engEvalString(ep, "if exist('emt2Obj', 'var'), delete(emt2Obj); end");
				//		engEvalString(ep, "hold on; emt2Obj = plot3(position(1), position(2), position(3), 'x', 'Color', c(3,:) ); hold off;");
				//	case 2:
				//		engEvalString(ep, "if exist('emt3Obj', 'var'), delete(emt3Obj); end");
				//		engEvalString(ep, "hold on; emt3Obj = plot3(position(1), position(2), position(3), 'x', 'Color', c(4,:) ); hold off;");

				//	}
				


				memcpy((char *) mxGetPr(pos_ptr), (char *) positionOT, 3*sizeof(double));
				engPutVariable(ep, "positionOT", pos_ptr);

				memcpy((char *) mxGetPr(orient_ptr), (char *) orientationOT, 4*sizeof(double));
				engPutVariable(ep, "orientationOT", orient_ptr);


				engEvalString(ep, "H_OT_to_OCS = quat2rot(orientationOT(1:3)'); H_OT_to_OCS = transl(positionOT') * H_OT_to_OCS;  H_OT_to_EMCS = Y * H_OT_to_OCS;");
				engEvalString(ep, "positionOT = H_OT_to_EMCS(1:3,4);");
				engEvalString(ep, "if exist('otObj', 'var'), delete(otObj); end");
				engEvalString(ep, "if exist('redsphere', 'var'), delete(redsphere); end");
				engEvalString(ep, "hold on; otObj = plot3(positionOT(1), positionOT(2), positionOT(3), 'o', 'Color', c(1,:) ); hold off;");
				// plot cylinder
				engEvalString(ep, "if exist('cylinderObj', 'var'), delete(cylinderObj); end");
				engEvalString(ep, "H_EMT_to_EMCS = H_OT_to_EMCS*inv(H_OT_to_EMT); cylinderObj = Plot_cylinder(H_EMT_to_EMCS)");
				

			}
			else if (!OTOkay && EMTOkay) // it emt data arrive but optical is not available
			{
			
				memcpy((char *) mxGetPr(pos_ptr), (char *) positionEMT, 3*sizeof(double));
				engPutVariable(ep, "positionEMT", pos_ptr);

				memcpy((char *) mxGetPr(orient_ptr), (char *) orientationEMT, 4*sizeof(double));
				engPutVariable(ep, "orientationEMT", orient_ptr);

				switch(sensor_number)
				{
				case 0:
				//	//(ep, "if exist('emt1Obj', 'var'), delete(emt1Obj); end");
				//	//engEvalString(ep, "hold on; emt1Obj = plot3(position(1), position(2), position(3), 'x', 'Color', c(2,:) ); hold off;");
					 //use EMT to map it to missing OT
						 //map EMT1 to OT
					engEvalString(ep, "H_EMT_to_EMCS = quat2rot(orientationEMT(1:3)'); H_EMT_to_EMCS = transl(positionEMT') * H_EMT_to_EMCS;  H_OT_to_EMCS = H_EMT_to_EMCS * H_OT_to_EMT;");
					engEvalString(ep, "OTposition = H_OT_to_EMCS(1:3,4);");
					engEvalString(ep, "if exist('otObj', 'var'), delete(otObj); end");
					engEvalString(ep, "if exist('redsphere', 'var'), delete(redsphere); end");
					engEvalString(ep, "hold on; otObj = plot3(OTposition(1), OTposition(2), OTposition(3), 'o', 'Color', c(3,:) ); hold off;");
									 //plot cylinder
				engEvalString(ep, "if exist('cylinderObj', 'var'), delete(cylinderObj); end");
				engEvalString(ep, "cylinderObj = Plot_cylinder(H_EMT_to_EMCS)");

				//case 1:
					//engEvalString(ep, "if exist('emt2Obj', 'var'), delete(emt2Obj); end");
					//engEvalString(ep, "hold on; emt2Obj = plot3(position(1), position(2), position(3), 'x', 'Color', c(3,:) ); hold off;");
					// use EMT to map it to missing OT
						// first map EMT2 to EMT1
				//	engEvalString(ep, "H_EMT2_to_EMCS = quat2rot(orientationEMT(1:3)'); H_EMT2_to_EMCS = transl(positionEMT') * H_EMT2_to_EMCS; H_EMT_to_EMCS = H_EMT2_to_EMCS * H_EMT1_to_EMT2;");
				//		// then map EMT1 to OT
				//	engEvalString(ep, "H_OT_to_EMCS = H_EMT_to_EMCS * H_OT_to_EMT;");
				//	engEvalString(ep, "OTposition = H_OT_to_EMCS(1:3,4);");
				//	engEvalString(ep, "if exist('otObj', 'var'), delete(otObj); end");
				//	engEvalString(ep, "hold on; otObj = plot3(OTposition(1), OTposition(2), OTposition(3), 'o', 'Color', c(2,:) ); hold off;");
				//					// plot cylinder
				//engEvalString(ep, "if exist('cylinderObj', 'var'), delete(cylinderObj); end");
				//engEvalString(ep, "cylinderObj = Plot_cylinder(H_EMT_to_EMCS)");

				//case 2:
				//	engEvalString(ep, "if exist('emt3Obj', 'var'), delete(emt3Obj); end");
				//	engEvalString(ep, "hold on; emt3Obj = plot3(position(1), position(2), position(3), 'x', 'Color', c(4,:) ); hold off;");
				//	// use EMT to map it to missing OT
				//		// first map EMT3 to EMT1
				//	engEvalString(ep, " H_EMT3_to_EMCS = transl(position'); H_EMT_to_EMCS = H_EMT3_to_EMCS * H_EMT1_to_EMT3;");
				//		// then map EMT1 to OT
				//	engEvalString(ep, "H_OT_to_EMCS = H_EMT_to_EMCS * H_OT_to_EMT;");
				//	engEvalString(ep, "OTposition = H_OT_to_EMCS(1:3,4);");
				//	engEvalString(ep, "if exist('otObj', 'var'), delete(otObj); end");
				//	engEvalString(ep, "hold on; otObj = plot3(OTposition(1), OTposition(2), OTposition(3), 'o', 'Color', c(1,:) ); hold off;");

				}
			}
			//else if (OTOkay && !EMTOkay) // if optical is available but emt data do not arrive 
			//{
			//	memcpy((char *) mxGetPr(pos_ptr), (char *) positionOT, 3*sizeof(double));
			//	engPutVariable(ep, "position", pos_ptr);

			//	engEvalString(ep, "H_OT_to_OCS = transl(position'); H_OT_to_EMCS = Y * H_OT_to_OCS;");
			//	engEvalString(ep, "OTposition = H_OT_to_EMCS(1:3,4);");
			//	engEvalString(ep, "if exist('otObj', 'var'), delete(otObj); end");
			//	engEvalString(ep, "hold on; otObj = plot3(OTposition(1), OTposition(2), OTposition(3), 'o', 'Color', c(1,:) ); hold off;");

			//	// use OT to map it to missing EMT
			//	engEvalString(ep, "H_EMT_to_EMCS = Y * H_OT_to_OCS * inv(H_OT_to_EMT);");
			//	engEvalString(ep, "EMTposition = H_EMT_to_EMCS(1:3,4);");
			//	engEvalString(ep, "if exist('emt1Obj', 'var'), delete(emt1Obj); end");
			//	engEvalString(ep, "hold on; emt1Obj = plot3(EMTposition(1), EMTposition(2), EMTposition(3), 'x', 'Color', c(2,:) ); hold off;");
			//}
			else if (!OTOkay && !EMTOkay)// plot a red sphere at last OT position if none is available
			{
				//memcpy((char *) mxGetPr(pos_ptr), (char *) positionOT, 3*sizeof(double));
				//engPutVariable(ep, "position", pos_ptr);

				//engEvalString(ep, "H_OT_to_OCS = transl(position'); H_OT_to_EMCS = Y * H_OT_to_OCS;");
				//engEvalString(ep, "OTposition = H_OT_to_EMCS(1:3,4);");

				//engEvalString(ep, "if exist('redsphere', 'var'), delete(redsphere); end");
				//engEvalString(ep, "xsp = 50*x + OTposition(1); ysp = 50*y + OTposition(2); zsp = 50*z + OTposition(3); hold on; redsphere = surf(xsp,ysp,zsp, 'EdgeColor' , 'none', 'FaceColor', 'r', 'FaceLighting', 'gouraud'); hold off; ");
			}


		}
		Sleep(40); // 50ms for maximum 20Hz update rate of the Matlab plot
	}

	while(_getch() != 27 || _getch() != 27)
	{
		//idle
	}

	pollingMutex.lock();
	pollingDoContinue = false;
	pollingMutex.unlock();
	Sleep(1);

	fclose(fileOpticalTracking);
	fclose(fileEMTracking);
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
