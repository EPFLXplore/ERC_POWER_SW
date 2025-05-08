#ifndef BMS_CONFIG
#define BMS_CONFIG
#endif

enum OV_HW{ // la protection hardware possède uniquement les valeures suivantes pour la tension de cellules max,
				//combinaisons pour les signaux A1_1 A0_1 A1_2 A0_2 concatenés 00=Default,01=Vreg,10=Vref,11=V-
		OV45 = 0b0101,  //4.498
		OV44 = 0b0110,  //4.403
		OV43 = 0b0111,  //4.307
		OV42 = 0b1001,  //4.211
		OV41 = 0b1010,  //4.116
		OV40 = 0b1011,  //4.020
		OV39 = 0b1101,  //3.924
		OV38 = 0b1110,  //3.828
		OV37 = 0b1111   //3.733
};

enum UV_HW{ // la protection hardware possède uniquement les valeures suivantes pour la tension de cellules min,
			//combinaisons pour les signaux A1_3 A0_3 A1_4 A0_4 concatenés 00=Default,01=Vreg,10=Vref,11=V-
		UV29 = 0b0101,  //2.871
		UV27 = 0b0110,  //2.680
		UV25 = 0b0111,  //2.489
		UV23 = 0b1001,  //2.297
		UV21 = 0b1010,  //2.106
		UV19 = 0b1011,  //1.914
		UV17 = 0b1101,  //1.723
		UV15 = 0b1110,  //1.531
		UV07 = 0b1111   //0.766
};

enum Cell_CNT{ // konfigurer le nombre de cellules pour la protection hardware
	Cell10 = 0b0111,
	Cell9 = 0b1001,
	Cell8 = 0b1010,
	Cell7 = 0b1011,
	Cell6 = 0b1101,
	Cell5 = 0b1110,
	Cell4 = 0b1111
};

enum Hyst{ // Hysterisis Selection
	H2 = 0b01, //UV=500mV OV=200mV
	H1 = 0b10, //UV=250mV OV=100mV
	H0 = 0b11 //UV=0mV OV=0mV
};

enum DC{ //Cycle time Selection
	DC2 = 0b01, //~15.5ms
	DC1 = 0b10, //~130ms
	DC0 = 0b11  //~500ms
};

//enum BMS_State{
//	Fault,
//	Running
//}

//List of Critical events that should be considered including but not limited to:
uint8_t HardwareOVP = OV42;
uint8_t HardwareUVP = UV21;
uint8_t HardwareCellCnt = Cell7;  //number of cells in Serie for hardware protection
uint8_t HardwareHysterysis = H0;
uint8_t HardwareCycleTime = DC2;
float SoftwareOVP = 4.18;
float SoftwareUVP = 2.2;
float DschgOCP = 60;
float ChgOCP = 5;
float OverTemp = 80;
float ChgUnderTemp = 0;

//List of Cell settings that should be considered including but not limited to:
float BalThresVolt;
float MaxDisbal = 0.015;
uint8_t CellsNbS = 7; //number of cells in Serie for Software protection
float ChgEndVolt ;
float ChgEndCurr;
float MinDischargeVoltage;
float IntBlockRes; // mOhm, individual cell resistance divided by number of parallel cells
float Capacity; // Ah
float SOC; // in %
uint16_t ThermB; // Thermistor B coefficient
uint16_t ThermR; // Thermistor Resistance
uint16_t



