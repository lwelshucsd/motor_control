#include "YEI_functions.hpp"
#include "general_functions.hpp"
#include <Windows.h>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <chrono>


void accelerometer::initialize_f() {
	YEI_port.port_name = new char[64];
	TSS_ERROR error = TSS_NO_ERROR;

	printf("====Creating a Three Space Device from Search====\n");
	tss_findSensorPorts(TSS_FIND_ALL_KNOWN ^ TSS_DONGLE);

	error = tss_getNextSensorPort(YEI_port.port_name, &YEI_port.device_type, &YEI_port.connection_Type);
	if (error == TSS_NO_ERROR)
	{
		error = tss_createSensor(YEI_port.port_name, &YEI_device_id);

		if (error)
		{
			printf("Failed to create TSS Sensor on %s!\n", YEI_port.port_name);
			tss_deinitAPI();
		}
		else
		{
			printf("====Starting Streaming====\n");
			error = tss_sensor_startStreamingWired(YEI_device_id, TSS_STREAM_CORRECTED_SENSOR_DATA, 500, TSS_STREAM_DURATION_INFINITE, 0);
			error = tss_sensor_enableTimestampsWired(YEI_device_id);

			Sleep(10);
			TSS_Stream_Packet packet;
			std::vector<std::vector<double>> threeSpaceData;
			using namespace std::chrono;
			long long start = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

			for (int i = 0; i < 10; i++)
			{
				long long ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
				ms = ms - start;
				threeSpaceData.push_back({ double(ms) });
				Sleep(50);
				printf("Press Enter to get next packet.\n");
				error = tss_sensor_getLastStreamingPacket(YEI_device_id, &packet);
				printf("Gyro	: (%.03f,%.03f,%.03f)\n", packet.correctedSensorData[0], packet.correctedSensorData[1], packet.correctedSensorData[2]);
				printf("Accel	: (%.03f,%.03f,%.03f)\n", packet.correctedSensorData[3], packet.correctedSensorData[4], packet.correctedSensorData[5]);
				printf("Magnet	: (%.03f,%.03f,%.03f)\n", packet.correctedSensorData[6], packet.correctedSensorData[7], packet.correctedSensorData[8]);
				for (int j = 0; j < 9; j++) {
					//threeSpaceData[j][i] = packet.correctedSensorData[j];
					threeSpaceData[i].push_back(packet.correctedSensorData[j]);
				}
			}

			save_array_f(threeSpaceData);
			tss_sensor_stopStreamingWired(YEI_device_id);

			tss_removeSensor(YEI_device_id);
		}
	}
	else
	{
		printf("Failed to get the port!\n");
		tss_deinitAPI();
	}

	tss_deinitAPI();

	printf("Finished press Enter to continue");
	getchar();

}

//std::vector<double> accelerometer::measure_all_f() {
//
//}
