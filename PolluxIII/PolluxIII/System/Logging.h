/*
 * Logging.h
 *
 *  Created on: 10 Aug 2022
 *      Author: arion
 */

#ifndef LIBRARIES_ROCKETFS_LOGGING_H_
#define LIBRARIES_ROCKETFS_LOGGING_H_

#include "Thread.h"
#include "Libraries/RoCo/RoCo.h"
#include "Libraries/RocketFS/FileSystem.h"

class Logging : public Thread {
public:
	Logging(MessageBus* bus);

	void init();
	void loop();

	void pauseLogging();
	void resumeLogging();

	void newMission();
	bool missionUploadBegin(uint32_t target_mission_id);
	void missionUploadNext();
	void missionUploadProcess();
	uint32_t getMissionID();

	void updateTimestamp(uint64_t timestamp);
	void beginBlock();
	void writeData(float data);
	void endBlock();
	float getFlashUsage();

private:
	MessageBus* bus;
	FileSystem fs;
	Stream mission_stream;
	uint32_t seq_number;
	uint64_t timestamp;
	uint32_t mission_id;
	xSemaphoreHandle logging_mutex;
	xSemaphoreHandle process_semaphore;
	Stream stream;
	bool next;
	bool enabled;

	Stream getMission(uint32_t target_mission_id);
};


#endif /* LIBRARIES_ROCKETFS_LOGGING_H_ */
