/*
 * System.cpp
 *
 *  Created on: 23 Oct 2020
 *      Author: AV Team 2020
 */

#include "System.h"

#include "Utils/Operators.h"
#include "Utils/ExceptionTracker.h"
#include "Debug/Debug.h"

#include "Misc/WatchdogThread.h"
#include "Sensors/PowerThread.h"
#include "Sensors/IMUThread.h"
#include "Supplies/SupplyThread.h"
#include "GUI/GUIThread.h"

#include "Logging.h"
#include "RoCo/RoCo.h"
#include "FlashAPI/flash.h"

#include <iwdg.h>
#include <usart.h>
#include <spi.h>
#include <i2c.h>
#include <tim.h>
#include <gpio.h>

#define BATTERY_CHARGE 600.0f
#define AUTO_SUMMARY false

#define CTA_UID 5111854
#define CTB_UID 5111850


/*
 * BEGIN
 * Please sync these definitions with HealthManager.h and SensorManager.h in the supervisor source code
 */
#define STATE_RESET 0
#define STATE_BOOTING 1
#define STATE_READY 2
#define STATE_SYNC 3
#define STATE_CRASH 255


typedef enum {
	SUPERVISOR,
	CTA,
	CTB
} controller_t;
/*
 * END
 */

static Logging* logging;

void handle_request(uint8_t sender, RequestPacket* packet, MessageBus* bus) {
	if(IS_RELIABLE((*packet))) {
		if(packet->uid != PS_UID_SUPERVISOR) {
			return;
		}

		uint32_t uid = HAL_GetUIDw0();

		if(packet->action_id == PS_ACTION_EXE) {
			if(packet->target_id == PS_TARGET_RESET) {
				if((uid == CTA_UID && packet->payload == CTA) || (uid == CTB_UID && packet->payload == CTB)) {
					Power_ControllerState state;
					state.state = STATE_RESET;
					MAKE_RELIABLE(state);
					bus->send(&state);

					NVIC_SystemReset();
				}
			} else if(packet->target_id == PS_TARGET_FLASH_ERASE) {
				flash_erase_all();
				NVIC_SystemReset();
			}
		} else if(packet->action_id == PS_ACTION_GET) {
			if(packet->target_id == PS_TARGET_MISSION_ID) {
				uint32_t mission_id = logging->getMissionID();

				ResponsePacket resp;
				resp.action_id = packet->action_id;
				resp.target_id = packet->target_id;
				resp.payload = mission_id;
				MAKE_RELIABLE(resp);

				bus->send(&resp);
			} else if(packet->target_id == PS_TARGET_DOWNLOAD) {

				uint32_t mission_id = packet->payload;

				if(logging->missionUploadBegin(mission_id)) {
					console.printf("Requested upload for mission ID %d...\r\n", mission_id);
					logging->missionUploadNext();
					logging->missionUploadProcess();
				} else {
					console.printf("Requested mission ID %d does not exist\r\n", mission_id);
				}
			} else if(packet->target_id == PS_TARGET_ACK) {
				logging->missionUploadNext();
				logging->missionUploadProcess();
			} else if(packet->target_id == PS_TARGET_NACK) {
				logging->missionUploadProcess();
			}
		}
	}
}

void handle_response(uint8_t sender, ResponsePacket* packet) {
	if(IS_RELIABLE((*packet))) {
		if(packet->uid != PS_UID_SUPERVISOR) {
			return;
		}

		if(packet->action_id == PS_ACTION_GET) {
			if(packet->target_id == PS_TARGET_ACK) {
				logging->missionUploadNext();
				logging->missionUploadProcess();
				console.printf("ACK\r\n");
			} else if(packet->target_id == PS_TARGET_NACK) {
				logging->missionUploadProcess();
				console.printf("NACK\r\n");
			}
		}
	}
}

void fetch_transmit_log(PowerThread* monitor, PowerMonitor monitorID, MessageBus* bus) {

	if(!monitor->isRunning()) {
		logging->writeData(0.0f);
		logging->writeData(0.0f);
		logging->writeData(0.0f);
		logging->writeData(0.0f);
		return;
	}

	float power = 0.001f * monitor->getPower() / 16384.0f;
	float energy = 0.001f * monitor->getEnergy() / 16384.0f / 3600000000.0f;
	float voltage = 0.001f * monitor->getVoltage() / 1000.0f;
	float temperature = 0.001f * monitor->getTemperature();



	logging->writeData(power);
	logging->writeData(energy);
	logging->writeData(voltage);
	logging->writeData(temperature);
}

static void send_status(MessageBus* bus, uint8_t status);
static void handle_ping(uint8_t sender, PingPacket* packet);

void systemd_init() {
	//static IMUThread imu(&hi2c4, 0b11010000, "IMU");

	static Terminal terminal;
	static Shell shell(&hlpuart1, &terminal);
	// static WatchdogThread watchdog(&hiwdg1);

	flash_init();

	//flash_erase_all();

	static STMUARTDriver uart_driver(&huart2);
	static LoopbackDriver loopback_driver;

	static PowerBus main_bus(&uart_driver);
	static PowerBus internal_bus(&loopback_driver);

	logging = new Logging(&main_bus);

	main_bus.forward<PingPacket>(&main_bus); // Loopback pings to the WiFi module
	main_bus.handle<PingPacket>(&handle_ping);
	main_bus.handle<ResponsePacket>(&handle_response);
	main_bus.handle<RequestPacket>(std::bind(&handle_request, std::placeholders::_1, std::placeholders::_2, &main_bus));

	internal_bus.forward<Power_BusInfo>(&main_bus);
	internal_bus.forward<Power_SupplyInfo>(&main_bus);
	internal_bus.forward<Power_BatteryInfo>(&main_bus);
	internal_bus.forward<Power_SupplyControl>(&main_bus);
	main_bus.forward<Power_SupplyControl>(&internal_bus);

	static GUIThread gui(&hspi4, &internal_bus);

	send_status(&main_bus, STATE_BOOTING);

	console.printf("Controller UID: %d\r\n", HAL_GetUIDw0());

	static PowerThread power_monitor0(&hspi1, &htim2, LVA, 0, &internal_bus);
	static PowerThread power_monitor1(&hspi1, &htim2, LVB, 4, &internal_bus);
	static PowerThread power_monitor2(&hspi1, &htim2, HVA, 2, &internal_bus);
	static PowerThread power_monitor3(&hspi1, &htim2, HVB, 6, &internal_bus);
	static PowerThread power_monitor4(&hspi1, &htim2, MOTORS, 1, &internal_bus);
	static PowerThread power_monitor5(&hspi1, &htim2, BATTERY, 5, &internal_bus);
	PowerThread* general_monitor = &power_monitor5;

	static SupplyThread lva_supply(&hi2c4, LVASupply, &power_monitor5, &power_monitor0, &internal_bus);
	static SupplyThread lvb_supply(&hi2c4, LVBSupply, &power_monitor5, &power_monitor1, &internal_bus);
	static SupplyThread hva_supply(&hi2c4, HVASupply, &power_monitor5, &power_monitor2, &internal_bus);
	static SupplyThread hvb_supply(&hi2c4, HVBSupply, &power_monitor5, &power_monitor3, &internal_bus);

	logging->pauseLogging(); // To remove for mission
	logging->newMission();

	send_status(&main_bus, STATE_READY);

	console.printf("Power system V4 is now ready\r\n", HAL_GetUIDw0());

	while(true) {

		float energy = general_monitor->getEnergy();
		float soc = 100.0f * (1.0f - energy / BATTERY_CHARGE);

		if(general_monitor->isRunning()) {
			Power_BatteryInfo batt_info;
			batt_info.charge = soc / 100.0f;
			batt_info.estimated_runtime = soc * 1.2f;
			MAKE_RELIABLE(batt_info);
			internal_bus.send(&batt_info);
		}

		logging->beginBlock();

		fetch_transmit_log(&power_monitor0, LVA, &internal_bus);
		fetch_transmit_log(&power_monitor1, LVB, &internal_bus);
		fetch_transmit_log(&power_monitor2, HVA, &internal_bus);
		fetch_transmit_log(&power_monitor3, HVB, &internal_bus);
		fetch_transmit_log(&power_monitor4, MOTORS, &internal_bus);
		fetch_transmit_log(&power_monitor5, BATTERY, &internal_bus);

		logging->endBlock();


		Power_ControllerHealth health;
		health.heap = (float) (configTOTAL_HEAP_SIZE - xPortGetFreeHeapSize()) / configTOTAL_HEAP_SIZE;
		health.flash = logging->getFlashUsage();
		MAKE_RELIABLE(health);
		main_bus.send(&health);

		if(general_monitor->isRunning() && (AUTO_SUMMARY || monitor.enter(SUMMARY_MONITOR))) {
			float general_power = general_monitor->getPower();
			float general_energy = general_monitor->getEnergy();
			float general_voltage = general_monitor->getVoltage();
			float general_temperature = general_monitor->getTemperature();

			uint8_t range = (uint8_t) (soc * 0.78f);

			char battery_charge[78];
			for(uint8_t i = 0; i < range; i++) {
				battery_charge[i] = '=';
			}

			for(uint8_t i = range; i < 78; i++) {
				battery_charge[i] = ' ';
			}

			if(AUTO_SUMMARY) {
				console.printf("\e7");
				console.printf("\x1b[17;0H");
			}

			console.printf("Power: %.3fW\x1b[K\r\n", general_power);
			console.printf("Energy: %.3fWh\x1b[K\r\n", general_energy);
			console.printf("Bus voltage: %.3fV\x1b[K\r\n", general_voltage);
			console.printf("Bus temperature: %.3f°C\x1b[K\r\n", general_temperature);
			console.printf("LVA status: %s\x1b[K\r\n", lva_supply.getStatus());
			console.printf("LVB status: %s\x1b[K\r\n", lvb_supply.getStatus());
			console.printf("HVA status: %s\x1b[K\r\n", hva_supply.getStatus());
			console.printf("HVB status: %s\x1b[K\r\n", hvb_supply.getStatus());
			console.printf("Battery: %.3f%%\x1b[K\r\n", soc);
			console.printf("\x1b[K\r\n");
			console.printf("[%s]\n", battery_charge);

			if(AUTO_SUMMARY) {
				console.printf("\e8");
			} else {
				monitor.exit(SUMMARY_MONITOR);
			}
		}

		osDelay(1000);
	}
}

static void handle_ping(uint8_t sender, PingPacket* packet) {
	if(IS_RELIABLE((*packet))) {
		HAL_IWDG_Refresh(&hiwdg1);
		logging->updateTimestamp(packet->time);
	} else {
		console.printf("Pong failed! %x\r\n", packet->time);
	}
}

static void send_status(MessageBus* bus, uint8_t status) {
	Power_ControllerState state;
	state.state = status;
	MAKE_RELIABLE(state);
	bus->send(&state);
}
