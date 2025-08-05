/*
 * Logging.cpp
 *
 *  Created on: 10 Aug 2022
 *      Author: arion
 */

#include "Logging.h"

#include "Libraries/FlashAPI/flash.h"
#include "Debug/Debug.h"


static void __debug(const char *message) {
	console.printf("%s\r\n", message);
}


Logging::Logging(MessageBus* bus) : Thread("LoggingThread"),
									bus(bus),
									fs("NOR Flash", 4096 * 4096, 4096),
									mission_stream(nullptr, 0, EMPTY),
									seq_number(0),
									mission_id(0xFFFFFFFF),
									stream(nullptr, 0, EMPTY),
									next(false),
									enabled(true)
{
    this->logging_mutex = xSemaphoreCreateMutex();
    this->process_semaphore = xSemaphoreCreateCounting(3, 0);
}

void Logging::init() {
	xSemaphoreTake(logging_mutex, portMAX_DELAY);

	osDelay(10000 / portTICK_PERIOD_MS); // Avoid damage to memory if in watchdog loop

	fs.setLogger(&__debug);
	fs.bind(&flash_read, &flash_write, &flash_erase_subsector);

	fs.mount();

	println("Logging ready");

	xSemaphoreGive(logging_mutex);
}

void Logging::loop() {
	PayloadPacket packet;

	if(stream.getFileType() != EMPTY) {
		uint32_t total_length = 0;
		int32_t length = 0;

		while(!stream.isEOF()) {
			if(xSemaphoreTake(process_semaphore, portMAX_DELAY)) {
				if(xSemaphoreTake(logging_mutex, portMAX_DELAY)) {
					if(next) {
						length = stream.read(packet.payload, 512);
						this->next = false;
					}

					if(length >= 0) {
						packet.length = length;
						total_length += length;
						MAKE_RELIABLE(packet);
						bus->send(&packet);
					}

					xSemaphoreGive(logging_mutex);
				}
			}

			console.printf("Uploaded %d bytes\r\n", total_length);
		}

		console.printf("Upload finished!\r\n", total_length);

		packet.length = 0;
		MAKE_RELIABLE(packet);
		bus->send(&packet);

		stream = Stream(nullptr, 0, EMPTY);
	}
}

void Logging::pauseLogging() {
	enabled = false;
}

void Logging::resumeLogging() {
	enabled = true;
}

void Logging::newMission() {
	if(enabled && xSemaphoreTake(logging_mutex, portMAX_DELAY)) {
		File* mission_status = fs.getFile("MissionStatus");
		File* mission_file;

		uint8_t mission_slot;

		osDelay(1000);


		Stream mission_status_stream(nullptr, 0, EMPTY);


		if(mission_status == nullptr) {
			mission_status = fs.newFile("MissionStatus", RAW);
			mission_status_stream = fs.openStream(mission_status, OVERWRITE);

			this->mission_id = 0;
			mission_slot = 0;
		} else {
			mission_status_stream = fs.openStream(mission_status, OVERWRITE);

			this->mission_id = 0xFFFFFFFF;

			while(1) {
				uint32_t new_mission_id = mission_status_stream.read32();

				if(new_mission_id != mission_id + 1) {
					break;
				} else {
					this->mission_id = new_mission_id;
					mission_status_stream.write32(0xFFFFFFFF); // Equivalent to writing nothing
				}
			}

			this->mission_id++;

			mission_slot = mission_id % 10;
		}

		mission_status_stream.write32(mission_id);

		mission_status_stream.close();

		console.printf("Mission UID: %d\r\n", mission_id);

		uint8_t filename[] = "Mission#";
		filename[7] = 48 + mission_slot;

		mission_file = fs.getFile((const char*) filename);

		if(mission_file != nullptr) {
			fs.delFile(mission_file);
		}

		mission_file = fs.newFile((const char*) filename, RAW);

		this->mission_stream = fs.openStream(mission_file, APPEND);

		xSemaphoreGive(logging_mutex);
	}
}

void Logging::updateTimestamp(uint64_t timestamp) {
	this->timestamp = timestamp;
}

void Logging::beginBlock() {
	if(enabled && xSemaphoreTake(logging_mutex, portMAX_DELAY)) {
		mission_stream.write32(0xC0FFEE);
		mission_stream.write32(seq_number++);
		mission_stream.write64(timestamp);
		xSemaphoreGive(logging_mutex);
	}
}

void Logging::writeData(float data) {
	if(enabled && xSemaphoreTake(logging_mutex, portMAX_DELAY)) {
		uint32_t *temp;
		temp = (uint32_t*) &data;
		mission_stream.write32(*temp);
		xSemaphoreGive(logging_mutex);
	}
}

void Logging::endBlock() {
	if(enabled && xSemaphoreTake(logging_mutex, portMAX_DELAY)) {
		mission_stream.write32(0xCAFE);

		if(seq_number % 100 == 0) {
			fs.flush();
		}

		xSemaphoreGive(logging_mutex);
	}
}

float Logging::getFlashUsage() {
	return (float) fs.getTotalUsedBlocks() / NUM_BLOCKS;
}

uint32_t Logging::getMissionID() {
	return mission_id;
}

Stream Logging::getMission(uint32_t target_mission_id) {
	uint8_t filename[] = "Mission#";
	filename[7] = 48 + (target_mission_id % 10);

	if(xSemaphoreTake(logging_mutex, portMAX_DELAY)) {
		File* mission_status = fs.getFile("MissionStatus");
		File* mission = fs.getFile((const char*) filename);

		if(mission_status != nullptr && mission != nullptr) {
			Stream mission_status_stream = fs.openStream(mission_status, OVERWRITE);

			uint32_t last_mission_id = 0xFFFFFFFF;

			while(1) {
				uint32_t new_mission_id = mission_status_stream.read32();

				if(new_mission_id != last_mission_id + 1) {
					break;
				} else if(new_mission_id == target_mission_id) {
					Stream mission_stream = fs.openStream(mission, OVERWRITE);

					xSemaphoreGive(logging_mutex);

					return mission_stream;
				}

				last_mission_id = new_mission_id;
			}
		}

		xSemaphoreGive(logging_mutex);
	}

	return Stream(nullptr, 0, EMPTY);
}

bool Logging::missionUploadBegin(uint32_t target_mission_id) {
	this->stream = getMission(target_mission_id);
	return stream.getFileType() != EMPTY;
}

void Logging::missionUploadNext() {
	this->next = true;
}

void Logging::missionUploadProcess() {
	xSemaphoreGive(process_semaphore);
	xSemaphoreGive(process_semaphore);
	xSemaphoreGive(process_semaphore); // Give three semaphores in case one gets lost
}
