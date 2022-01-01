//============================================================================
// Name        : GCodeParser.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>
#include <vector>
#include <math.h>
#include <chrono>
#include <regex>
#include <sstream>
#include <string>
#include <fstream>


const char* STOP_RESET_FILE = "/sys/class/cncController/cncC0/stop_reset";
const char* STOP_FILE = "/sys/class/cncController/cncC0/stop_clear";
const char* WRITE_DMA_FILE = "/sys/class/dma_class/dma-0/write_dma";
const char* GCODEFILE = "/media/sd-mmcblk0p2/Part2.txt";
const char* DMA_BUFFER = "/dev/dma-0";
const int axi_clock_freq = 50000000;
const int ns_per_clock = 1000000000/axi_clock_freq;
enum DIRECTION {DIRECTION_POS, DIRECTION_NEG}; //determined as hardware switch so we don't have to keep track
enum MOTORNUMBERS {MOTORNUMBERS_M1, MOTORNUMBERS_M2, MOTORNUMBERS_M3, MOTORNUMBERS_M4, MOTORNUMBERS_M5};
#define MOTOR_MOVE_BIT 0
#define MOTOR_DIR_BIT 1
#define bufflength 2048
#define MAXTOOLCOUNT 20
#define MAXSPINDLESPEED 10000



struct CNC_OP_DataStructure {
	unsigned int timer : 24;
	unsigned int m1 : 8;
	unsigned int m2 : 8;
	unsigned int m3 : 8;
	unsigned int m4 : 8;
	unsigned int m5 : 8;
};
class cncHardwareControl {
	public:
	static double total_time;
	
	static void write_Data() {
		//todo add buffering so i don't have to open write every time
		buffer.push_back(CNC_op);
		total_time += CNC_op.timer;
		if(buffer.size() == bufflength) {
			flush();
		}
		
	}
	static void flush() {
		if(buffer.empty()) return;
		int ret;
		struct pollfd pfd;
		int fd = open(DMA_BUFFER, O_WRONLY | O_NONBLOCK);
		if(fd == -1) {
			printf("Failed to open file\n");
			return;
		}
		pfd.fd = fd;
		pfd.events = ( POLLOUT | POLLWRNORM );
		ret = poll(&pfd, (unsigned long)1, 20000);
//			printf("%x:%x\n", ret, pfd.revents);
		if(ret < 0) {
			printf("Error in polling\n");
			assert(0);
		}
		if((pfd.revents & POLLOUT) == POLLOUT) {
			int wcount = write(fd, (void*)&buffer[0], buffer.size() * sizeof(CNC_OP_DataStructure));
		}
		buffer.clear();
		close(fd);
	}
	static int ForceWriteDMA() {
		int ret;
		int fd = open(WRITE_DMA_FILE, O_WRONLY | O_NONBLOCK);
		if(fd == -1) {
			printf("Failed to open sysfs file\n");
			return -1;
		}
		ret = write(fd, "1", 1);
		close(fd);

		return ret;
	}
	static int Reset_Stop_Bit(bool _r) {
		int ret;
		int rst_fd = open(STOP_RESET_FILE, O_WRONLY | O_NONBLOCK);
		if(rst_fd == -1) {
			printf("Failed to open sysfs file\n");
		}
		ret = write(rst_fd, _r?"1":"0", 1);
		close(rst_fd);

		return ret;
	}
	static int Stop_Bit() {
		int ret;
		int rst_fd = open(STOP_FILE, O_WRONLY | O_NONBLOCK);
		if(rst_fd == -1) {
			printf("Failed to open sysfs file\n");
		}
		ret = write(rst_fd, "1", 1);
		close(rst_fd);

		return ret;
	}

	static void setTimer(unsigned int nanoSeconds) {
		CNC_op.timer = (nanoSeconds/ns_per_clock) & 0x00FFFFFF;
	} 
	static void unset_motors() {
		CNC_op.m1 = 0;
		CNC_op.m2 = 0;
		CNC_op.m3 = 0;
		CNC_op.m4 = 0;
		CNC_op.m5 = 0;
	}
	static void setMotor(enum MOTORNUMBERS _m, bool move, enum DIRECTION dir) {
		switch(_m) {
			case(MOTORNUMBERS_M1):{
				CNC_op.m1 = ParstMotorControl(move, dir);
				break;
			}
			case(MOTORNUMBERS_M2):{
				CNC_op.m2 = ParstMotorControl(move, dir);
				break;
			}
			case(MOTORNUMBERS_M3):{
				CNC_op.m3 = ParstMotorControl(move, dir);
				break;
			}
			case(MOTORNUMBERS_M4):{
				CNC_op.m4 = ParstMotorControl(move, dir);
				break;
			}
			case(MOTORNUMBERS_M5):{
				CNC_op.m5 = ParstMotorControl(move, dir);
				break;
			}
			default:
			assert(0);
		}
	}


	private:
	static struct CNC_OP_DataStructure CNC_op;
	static std::vector<CNC_OP_DataStructure> buffer;
	static uint8_t ParstMotorControl(bool move, enum DIRECTION dir) {
		return (uint8_t)((move << MOTOR_MOVE_BIT) + (dir << MOTOR_DIR_BIT));
	}

	
};
double cncHardwareControl::total_time;
struct CNC_OP_DataStructure cncHardwareControl::CNC_op;
std::vector<CNC_OP_DataStructure> cncHardwareControl::buffer;

// cncOperationControl cnc; 
struct coordinate_s {
	double x;
	double y;
	double z;
};

enum CUBEPOSITIONS {XN1Y1Z1, X0Y1Z1, X1Y1Z1,
					XN1Y0Z1, X0Y0Z1, X1Y0Z1,
					XN1YN1Z1,X0YN1Z1,X1YN1Z1,
					XN1Y1Z0, X0Y1Z0, X1Y1Z0,
					XN1Y0Z0, X0Y0Z0, X1Y0Z0,
					XN1YN1Z0,X0YN1Z0,X1YN1Z0, 
					XN1Y1ZN1, X0Y1ZN1, X1Y1ZN1,
					XN1Y0ZN1, X0Y0ZN1, X1Y0ZN1,
					XN1YN1ZN1,X0YN1ZN1,X1YN1ZN1,
					POS_LENGTH };



enum GCODE_INSTRUCTION_TYPES {G_00, G_01, G_02, G_03, G_17, G_21, G_40, G_54, G_80, G_90, G_91, G_94, S_setspeed, M_03, M_06, G_NOTELINE, G_UNKOWN_STOP};
struct TOOL_DATA {
	std::string name;
};
class MachineParams {
	public:
	static constexpr const int encoder_precision = 6400;
	static constexpr const double TPI = 10;
	static constexpr const double Max_rpm = 1130;
	static constexpr const double x_mm_per_tick = 1.0 / (encoder_precision * (TPI / 25.4));
	static constexpr const double y_mm_per_tick = 1.0 / (encoder_precision * (TPI / 25.4));
	static constexpr const double z_mm_per_tick = 1.0 / (encoder_precision * (TPI / 25.4));
	static double xy_mm_per_tick;
	static double Max_Move_Distance;
	static double time_x_move;
	static double time_y_move;
	static double time_z_move;
	static double time_xy_move;
	static struct TOOL_DATA Tools[MAXTOOLCOUNT];
	static struct TOOL_DATA *currentTool;
	static int spindle_rpm;

	static bool useMM;

	static constexpr const double maxFeedrate_mm_sec = Max_rpm * 25.4 / (60.0 * 10.0);
	static double Feedrate_mm_sec;

	static void init() {
		//apparently can't constepr these math functions
		xy_mm_per_tick = std::sqrt( std::pow(x_mm_per_tick,2) + std::pow(y_mm_per_tick,2));
		Max_Move_Distance = xy_mm_per_tick;
		useMM = true;
		setFeedrate(10);
	}
	static void setSpindleSpeed(int _rpm) {
		if(_rpm > MAXSPINDLESPEED) _rpm = MAXSPINDLESPEED;
		spindle_rpm = _rpm;
	}
	static void setFeedrate(double feed_dist_min) {
		if(useMM) {
			Feedrate_mm_sec = feed_dist_min / 60;
		} else {
			Feedrate_mm_sec = feed_dist_min * 25.4 / 60.0;
		}
		if(Feedrate_mm_sec > maxFeedrate_mm_sec) Feedrate_mm_sec = maxFeedrate_mm_sec;

		time_x_move = x_mm_per_tick * 1000000000 / Feedrate_mm_sec;
		time_y_move = y_mm_per_tick * 1000000000 / Feedrate_mm_sec;
		time_z_move = y_mm_per_tick * 1000000000 / Feedrate_mm_sec;
		time_xy_move = xy_mm_per_tick * 1000000000 / Feedrate_mm_sec;
	}
	static double getFeedrate() {
		if(useMM) {
			return Feedrate_mm_sec * 60.0;
		} else {
			return Feedrate_mm_sec * 60.0 / 25.4;
		}
	}
	static double getMAXFeedrate() {
		if(useMM) {
			return maxFeedrate_mm_sec * 60.0;
		} else {
			return maxFeedrate_mm_sec * 60.0 / 25.4;
		}
	}
	static void setAbsoluteMode() {
		std::cout << "todo: add absolute mode\n";
	}
	static void setRelativeMode() {
		std::cout << "todo: add relative mode\n";
	}
	static void setTool(int toolnumber) {
		if(toolnumber < MAXTOOLCOUNT) {
			currentTool = &Tools[toolnumber];
		}
	}
	static void writemotors(enum CUBEPOSITIONS moveDirection) {
		cncHardwareControl::unset_motors();
		cncHardwareControl::setTimer(0);
		switch(moveDirection) {
			case (XN1Y1Z1): {
				// cncHardwareControl::setTimer(time_xy_move);
				// cncHardwareControl::setMotor(MOTORNUMBERS_M1, true, DIRECTION_NEG);
				// cncHardwareControl::setMotor(MOTORNUMBERS_M2, true, DIRECTION_POS);
				break;
			}
			case (X0Y1Z1): {
				// cncHardwareControl::setTimer(time_y_move);
				// cnc.setMotor(MOTORNUMBERS_M1, true, DIRECTION_NEG);
				// cncHardwareControl::setMotor(MOTORNUMBERS_M2, true, DIRECTION_POS);
				break;
			}
			case (X1Y1Z1): {
				// cncHardwareControl::setTimer(time_xy_move);
				// cncHardwareControl::setMotor(MOTORNUMBERS_M1, true, DIRECTION_POS);
				// cncHardwareControl::setMotor(MOTORNUMBERS_M2, true, DIRECTION_POS);
				break;
			}
			case (XN1Y0Z1): {
				// cncHardwareControl::setTimer(time_x_move);
				// cncHardwareControl::setMotor(MOTORNUMBERS_M1, true, DIRECTION_NEG);
				// cnc.setMotor(MOTORNUMBERS_M2, true, DIRECTION_POS);
				break;
			}
			case (X0Y0Z1): {
				cncHardwareControl::setTimer(time_z_move);
				// cnc.setMotor(MOTORNUMBERS_M1, true, DIRECTION_NEG);
				// cnc.setMotor(MOTORNUMBERS_M2, true, DIRECTION_POS);
				cncHardwareControl::setMotor(MOTORNUMBERS_M3, true, DIRECTION_POS);
				break;
			}
			case (X1Y0Z1): {
				// cncHardwareControl::setTimer(time_x_move);
				// cncHardwareControl::setMotor(MOTORNUMBERS_M1, true, DIRECTION_POS);
				// cnc.setMotor(MOTORNUMBERS_M2, true, DIRECTION_POS);
				break;
			}
			case (XN1YN1Z1): {
				// cncHardwareControl::setTimer(time_xy_move);
				// cncHardwareControl::setMotor(MOTORNUMBERS_M1, true, DIRECTION_NEG);
				// cncHardwareControl::setMotor(MOTORNUMBERS_M2, true, DIRECTION_NEG);
				break;
			}
			case (X0YN1Z1): {
				// cncHardwareControl::setTimer(time_y_move);
				// cnc.setMotor(MOTORNUMBERS_M1, true, DIRECTION_NEG);
				// cncHardwareControl::setMotor(MOTORNUMBERS_M2, true, DIRECTION_NEG);
				break;
			}
			case (X1YN1Z1): {
				// cncHardwareControl::setTimer(time_xy_move);
				// cncHardwareControl::setMotor(MOTORNUMBERS_M1, true, DIRECTION_POS);
				// cncHardwareControl::setMotor(MOTORNUMBERS_M2, true, DIRECTION_NEG);
				break;
			}




			case (XN1Y1Z0): {
				cncHardwareControl::setTimer(time_xy_move);
				cncHardwareControl::setMotor(MOTORNUMBERS_M1, true, DIRECTION_NEG);
				cncHardwareControl::setMotor(MOTORNUMBERS_M2, true, DIRECTION_POS);
				break;
			}
			case (X0Y1Z0): {
				cncHardwareControl::setTimer(time_y_move);
				// cnc.setMotor(MOTORNUMBERS_M1, true, DIRECTION_NEG);
				cncHardwareControl::setMotor(MOTORNUMBERS_M2, true, DIRECTION_POS);
				break;
			}
			case (X1Y1Z0): {
				cncHardwareControl::setTimer(time_xy_move);
				cncHardwareControl::setMotor(MOTORNUMBERS_M1, true, DIRECTION_POS);
				cncHardwareControl::setMotor(MOTORNUMBERS_M2, true, DIRECTION_POS);
				break;
			}
			case (XN1Y0Z0): {
				cncHardwareControl::setTimer(time_x_move);
				cncHardwareControl::setMotor(MOTORNUMBERS_M1, true, DIRECTION_NEG);
				// cnc.setMotor(MOTORNUMBERS_M2, true, DIRECTION_POS);
				break;
			}
			case (X0Y0Z0): {
				std::cout << "move center error\n";
				// cnc.setTimer(time_xy_move);
				// cnc.setMotor(MOTORNUMBERS_M1, true, DIRECTION_NEG);
				// cnc.setMotor(MOTORNUMBERS_M2, true, DIRECTION_POS);
				break;
			}
			case (X1Y0Z0): {
				cncHardwareControl::setTimer(time_x_move);
				cncHardwareControl::setMotor(MOTORNUMBERS_M1, true, DIRECTION_POS);
				// cnc.setMotor(MOTORNUMBERS_M2, true, DIRECTION_POS);
				break;
			}
			case (XN1YN1Z0): {
				cncHardwareControl::setTimer(time_xy_move);
				cncHardwareControl::setMotor(MOTORNUMBERS_M1, true, DIRECTION_NEG);
				cncHardwareControl::setMotor(MOTORNUMBERS_M2, true, DIRECTION_NEG);
				break;
			}
			case (X0YN1Z0): {
				cncHardwareControl::setTimer(time_y_move);
				// cnc.setMotor(MOTORNUMBERS_M1, true, DIRECTION_NEG);
				cncHardwareControl::setMotor(MOTORNUMBERS_M2, true, DIRECTION_NEG);
				break;
			}
			case (X1YN1Z0): {
				cncHardwareControl::setTimer(time_xy_move);
				cncHardwareControl::setMotor(MOTORNUMBERS_M1, true, DIRECTION_POS);
				cncHardwareControl::setMotor(MOTORNUMBERS_M2, true, DIRECTION_NEG);
				break;
			}



			case (XN1Y1ZN1): {
				// cncHardwareControl::setTimer(time_xy_move);
				// cncHardwareControl::setMotor(MOTORNUMBERS_M1, true, DIRECTION_NEG);
				// cncHardwareControl::setMotor(MOTORNUMBERS_M2, true, DIRECTION_POS);
				break;
			}
			case (X0Y1ZN1): {
				// cncHardwareControl::setTimer(time_y_move);
				// cnc.setMotor(MOTORNUMBERS_M1, true, DIRECTION_NEG);
				// cncHardwareControl::setMotor(MOTORNUMBERS_M2, true, DIRECTION_POS);
				break;
			}
			case (X1Y1ZN1): {
				// cncHardwareControl::setTimer(time_xy_move);
				// cncHardwareControl::setMotor(MOTORNUMBERS_M1, true, DIRECTION_POS);
				// cncHardwareControl::setMotor(MOTORNUMBERS_M2, true, DIRECTION_POS);
				break;
			}
			case (XN1Y0ZN1): {
				// cncHardwareControl::setTimer(time_x_move);
				// cncHardwareControl::setMotor(MOTORNUMBERS_M1, true, DIRECTION_NEG);
				// cnc.setMotor(MOTORNUMBERS_M2, true, DIRECTION_POS);
				break;
			}
			case (X0Y0ZN1): {
				cncHardwareControl::setTimer(time_z_move);
				// cnc.setMotor(MOTORNUMBERS_M1, true, DIRECTION_NEG);
				// cnc.setMotor(MOTORNUMBERS_M2, true, DIRECTION_POS);
				cncHardwareControl::setMotor(MOTORNUMBERS_M3, true, DIRECTION_NEG);
				break;
			}
			case (X1Y0ZN1): {
				// cncHardwareControl::setTimer(time_x_move);
				// cncHardwareControl::setMotor(MOTORNUMBERS_M1, true, DIRECTION_POS);
				// cnc.setMotor(MOTORNUMBERS_M2, true, DIRECTION_POS);
				break;
			}
			case (XN1YN1ZN1): {
				// cncHardwareControl::setTimer(time_xy_move);
				// cncHardwareControl::setMotor(MOTORNUMBERS_M1, true, DIRECTION_NEG);
				// cncHardwareControl::setMotor(MOTORNUMBERS_M2, true, DIRECTION_NEG);
				break;
			}
			case (X0YN1ZN1): {
				// cncHardwareControl::setTimer(time_y_move);
				// cnc.setMotor(MOTORNUMBERS_M1, true, DIRECTION_NEG);
				// cncHardwareControl::setMotor(MOTORNUMBERS_M2, true, DIRECTION_NEG);
				break;
			}
			case (X1YN1ZN1): {
				// cncHardwareControl::setTimer(time_xy_move);
				// cncHardwareControl::setMotor(MOTORNUMBERS_M1, true, DIRECTION_POS);
				// cncHardwareControl::setMotor(MOTORNUMBERS_M2, true, DIRECTION_NEG);
				break;
			}


			default:
				break;
		}	
		cncHardwareControl::write_Data();
	}

};

double MachineParams::xy_mm_per_tick;
double MachineParams::Max_Move_Distance;
bool MachineParams::useMM = true;
double MachineParams::Feedrate_mm_sec = 100*60;
double MachineParams::time_x_move;
double MachineParams::time_y_move;
double MachineParams::time_z_move;
double MachineParams::time_xy_move;
struct TOOL_DATA MachineParams::Tools[MAXTOOLCOUNT];
struct TOOL_DATA *MachineParams::currentTool = nullptr;
int MachineParams::spindle_rpm = 100;

#define MAX_WORK_COORDINATE_OFFSETS 6
class CNC_GCODE_PROCESSING {
	public:
	//these coordinate are in reference to machine datum - offsets are added to the gcode location to correct to machine coordinates
	static struct coordinate_s MotorQuadrantPosition;
	static struct coordinate_s VirtualPosition;
	static struct coordinate_s last_exact_gcode_point;


	static struct coordinate_s WorkCoordinateOffsets[MAX_WORK_COORDINATE_OFFSETS];
	static struct coordinate_s *current_work_offset;

	static struct coordinate_s PossiblePositions[POS_LENGTH];
	static struct coordinate_s VirtualOutcomePositions[POS_LENGTH];
	static bool PossiblePositions_bool[POS_LENGTH];
	static double PossiblePositions_dist[POS_LENGTH];

	static std::vector<std::string> fileContents;
	static void LoadFile(const char* filename) {
		std::string line;
		std::ifstream gcodefile(filename);
		if(gcodefile.is_open()) {
			while(std::getline(gcodefile, line)) {
				fileContents.push_back(line);
			}
			gcodefile.close();
		}

		// fileContents.push_back("G00 Z5.000000");
		// fileContents.push_back("G00 X5 Y5");
		// fileContents.push_back("G01 X5 Y25 F200.0000");
		// fileContents.push_back("G01 X25 Y25");
		// fileContents.push_back("G02 X35 Y15 I0 J-10");
		// fileContents.push_back("G02 X31 Y7 I-10 J0");
		// fileContents.push_back("G01 X5 Y5");
		// fileContents.push_back("G03 X-5 Y-5 I-5 J-5");

	}
	static void emergency_stop() {
		cncHardwareControl::Stop_Bit();
		cncHardwareControl::flush();
		cncHardwareControl::ForceWriteDMA();
		return;
	}
	static void runGcodeFile() {
		std::vector<enum GCODE_INSTRUCTION_TYPES> gcode_setup_list;
		for(auto line_it = fileContents.begin(); line_it < fileContents.end(); line_it++) {
			// gcode_list.clear();
			enum GCODE_INSTRUCTION_TYPES *gcode_move_ptr = nullptr;
			struct coordinate_s *endpoint_ptr = nullptr;
			struct coordinate_s *centerpoint_ptr = nullptr;
			double *feedrate_ptr = nullptr;
			int *toolNumber_ptr = nullptr;
			int *aux_data_ptr = nullptr;
			std::cout << *line_it << "\n";

			GCODE_PARSE(*line_it, gcode_setup_list, &gcode_move_ptr, &endpoint_ptr, &centerpoint_ptr, &feedrate_ptr, &toolNumber_ptr, &aux_data_ptr);
			//process non movement gcode
			//correct for machine offsets
			//process movement gcode 
			for(auto gcode_it = gcode_setup_list.begin(); gcode_it < gcode_setup_list.end(); gcode_it++) {
				switch(*gcode_it) {
					case(G_17): {
						std::cout << "Set Machine in XY plane(other planes not supported so no action)\n";
						break;
					}
					case(G_21): {
						std::cout << "Set Machine to MM\n";
						MachineParams::useMM = true;
						break;
					}
					case(G_40): {
						std::cout << "Turn off cutter compensation\n";
						break;
					}
					case(G_54): {
						current_work_offset = &WorkCoordinateOffsets[0];
						break;
					}
					case(G_80): {
						std::cout << "Cancel canned cycles(no action supported)\n";
						break;
					}
					case(G_90): {
						MachineParams::setAbsoluteMode();
						break;
					}
					case(G_91): {
						MachineParams::setRelativeMode();
						break;
					}
					case(G_94): {
						std::cout << "Set feed/minute scale\n";
						break;
					}
					case(M_06): {
						if(toolNumber_ptr) {
							MachineParams::setTool(*toolNumber_ptr);
						} else {
							emergency_stop();
						}
						break;
					}
					case(M_03): {
						std::cout << "todo: turn on spindle\n";
						break;
					}
					case(S_setspeed): {
						if(aux_data_ptr) {
							MachineParams::setSpindleSpeed(*aux_data_ptr);
						} else {
							emergency_stop();
						}
						break;
					}
					case(G_UNKOWN_STOP): {
						//emergency stop because we don't know how to handle this code yet
						emergency_stop();
						return;
					}
					default: {
						std::cout << "No Op\n";
						break;
					}
					
				}
			}
			if(current_work_offset) {
				if(endpoint_ptr) {
					endpoint_ptr->x += current_work_offset->x;
					endpoint_ptr->y += current_work_offset->y;
					endpoint_ptr->z += current_work_offset->z;
				}
				if(centerpoint_ptr) {
					centerpoint_ptr->x += current_work_offset->x;
					centerpoint_ptr->y += current_work_offset->y;
					centerpoint_ptr->z += current_work_offset->z;

				}
			}
			if(gcode_move_ptr) {
				switch(*gcode_move_ptr) {
					case(G_00): {
						if(endpoint_ptr) {
							double oldFeedrate = MachineParams::getFeedrate();
							MachineParams::setFeedrate(MachineParams::getMAXFeedrate());
							moveLine(*endpoint_ptr);
							MachineParams::setFeedrate(oldFeedrate);
						} else {
							assert(0);
						}
						break;
					}
					case(G_01): {
						if(feedrate_ptr) {
							MachineParams::setFeedrate(*feedrate_ptr);
						}
						if(endpoint_ptr) {
							moveLine(*endpoint_ptr);
						}
						break;
					}
					case(G_02): {
						if(feedrate_ptr) {
							MachineParams::setFeedrate(*feedrate_ptr);
						}
						if(endpoint_ptr && centerpoint_ptr) {
							moveCircle(*endpoint_ptr, *centerpoint_ptr, true);
						}
						break;
					}
					case(G_03): {
						if(feedrate_ptr) {
							MachineParams::setFeedrate(*feedrate_ptr);
						}
						if(endpoint_ptr && centerpoint_ptr) {
							moveCircle(*endpoint_ptr, *centerpoint_ptr, false);
						}
						break;
					}
					default: {
						std::cout << "Move code not supported\n";
						emergency_stop();
						break;
					}
				}
			}
			if(endpoint_ptr) {
				last_exact_gcode_point = *endpoint_ptr;
			}
		}
		cncHardwareControl::flush();
		cncHardwareControl::ForceWriteDMA();
	}

	static void GCODE_PARSE(std::string line, std::vector<enum GCODE_INSTRUCTION_TYPES> &_gcode_setup_list, GCODE_INSTRUCTION_TYPES **_gcode_move, 
							struct coordinate_s **_endpoint, struct coordinate_s **_centerpoint, double **_feedrate, int **_toolnumber, int **_aux_data) {
		static enum GCODE_INSTRUCTION_TYPES gcode_move;
		static struct coordinate_s endpoint;
		static struct coordinate_s centerpoint;
		static double feedrate;
		static int toolnumber;
		static int aux_data;

		_gcode_setup_list.clear();
		*_gcode_move = nullptr;
		*_endpoint = nullptr;
		*_centerpoint = nullptr;
		*_feedrate = nullptr;
		*_toolnumber = nullptr;
		*_aux_data = nullptr;
		
		char space_del = ' ';
		std::vector<std::string> words;
		std::stringstream ss(line);
		std::string temp_s;
		while(std::getline(ss, temp_s, space_del)) {
			words.push_back(temp_s);
		}

		endpoint = last_exact_gcode_point;
		
		for(auto word = words.begin(); word < words.end(); word++) {
			if(word->length() == 1) {
				//this is mainly to deal with a comment word with a single ( ex. N1 G00 ( comment )
				return;
			}
			if(word->length() >= 2) {
				char first_char = (*word)[0];
				// std::cout << first_char << "\n";
				word->erase(word->begin());

				switch(first_char) {
					case('G'):{
						int temp_i = std::stoi(*word);
						switch(temp_i) {
							case(0): {
								//rapid
								*_gcode_move = &gcode_move;
								gcode_move = G_00;
								break;
							}
							case(1): {
								//linear
								*_gcode_move = &gcode_move;
								gcode_move = G_01;
								break;
							}
							case(2): {
								//clockwise circle
								*_gcode_move = &gcode_move;
								gcode_move = G_02;
								break;
							}
							case(3): {
								//counterclockwise circle
								*_gcode_move = &gcode_move;
								gcode_move = G_03;
								break;
							}
							case(17): {
								//set XY plane
								_gcode_setup_list.push_back(G_17);
								break;
							}
							case(21): {
								//set mm
								_gcode_setup_list.push_back(G_21);
								break;
							}
							case(40): {
								//disable tool radius compensation
								_gcode_setup_list.push_back(G_40);
								break;
							}
							case(54): {
								//coordinate offset for workpiece
								_gcode_setup_list.push_back(G_54);
								break;
							}
							case(80): {
								//cancel canned cycles
								_gcode_setup_list.push_back(G_80);
								break;
							}
							case(90): {
								//set absolute
								_gcode_setup_list.push_back(G_90);
								break;
							}
							case(91): {
								//set relative
								_gcode_setup_list.push_back(G_91);
								break;
							}
							case(94): {
								//set feed to feed/minute (mm/in set somewhere else)
								_gcode_setup_list.push_back(G_94);
								break;
							}
							default: {
								_gcode_setup_list.push_back(G_UNKOWN_STOP);
								break;
							}
						}
						break;
					}
					case('M'): {
						int temp_i = std::stoi(*word);
						switch(temp_i) {
							case(6): {
								//tool change
								_gcode_setup_list.push_back(M_06);
								break;
							}
							case(3): {
								//turn on spindle
								_gcode_setup_list.push_back(M_03);
								break;
							}
							default: {
								_gcode_setup_list.push_back(G_UNKOWN_STOP);
								break;
							}
						}
						break;
					}
					case('S'): {
						//set spindle speed
						int temp_i = std::stoi(*word);
						*_aux_data = &aux_data;
						aux_data = temp_i;
						_gcode_setup_list.push_back(S_setspeed);
						break;
					}
					case('X'): {
						double temp_d = std::stod(*word);
						*_endpoint = &endpoint;
						endpoint.x = temp_d;
						break;
					}
					case('Y'): {
						double temp_d = std::stod(*word);
						*_endpoint = &endpoint;
						endpoint.y = temp_d;
						break;
					}
					case('Z'): {
						double temp_d = std::stod(*word);
						*_endpoint = &endpoint;
						endpoint.z = temp_d;
						break;
					}
					case('I'): {
						double temp_d = std::stod(*word);
						*_centerpoint = &centerpoint;
						centerpoint.x = last_exact_gcode_point.x + temp_d;
						break;
					}
					case('J'): {
						double temp_d = std::stod(*word);
						*_centerpoint = &centerpoint;
						centerpoint.y = last_exact_gcode_point.y + temp_d;
						break;
					}
					case('F'): {
						double temp_d = std::stod(*word);
						*_feedrate = &feedrate;
						feedrate = temp_d;
						break;
					}
					case('N'): {
						break;
					}
					case('T'): {
						int temp_int = std::stoi(*word);
						*_toolnumber = &toolnumber;
						toolnumber = temp_int;
						break;
					}
					case('%'):
					case('O'):
					case('('):
					{
						_gcode_setup_list.push_back(G_NOTELINE);
						return;
					}
					default: {
						_gcode_setup_list.push_back(G_UNKOWN_STOP);
					}
				}
			}
		}

		if(*_endpoint && _gcode_setup_list.empty()) {
			//if we have an enpoint set and no other setup paramaters then run with the last move command
			*_gcode_move = &gcode_move;
		}
		return;
	}
	static void moveCircle(struct coordinate_s end, struct coordinate_s center, bool clockwise) {
		bool keeprunning = true;
		enum CUBEPOSITIONS moveDirection;

		while (keeprunning) {
			moveDirection = X0Y0Z0;
			fillPossibleLocations(MotorQuadrantPosition, MachineParams::x_mm_per_tick, MachineParams::y_mm_per_tick, MachineParams::z_mm_per_tick);

			// check for Y vector only(no slope)
			if(center.x == VirtualPosition.x) {
				if((VirtualPosition.y - center.y) > 0) {
					//mark x only directions;
					mark_XOnly_Direction(clockwise?true:false);
				} else {
					mark_XOnly_Direction(clockwise?false:true);
				}
			} else if(center.y == VirtualPosition.y) {
				if((VirtualPosition.x - center.x) > 0) {
					mark_YOnly_Direction(clockwise?false:true);
				} else {
					mark_YOnly_Direction(clockwise?true:false);
				}
			} else{
				double slope = (center.y - VirtualPosition.y) / (center.x - VirtualPosition.x);
				double p_slope = -1.0/slope;
				//is circle vector pointing to pos x direction
				bool center_to_circ_posx = (VirtualPosition.x - center.x) > 0;
				mark_XYcircle_PossibleLocations(clockwise?center_to_circ_posx:!center_to_circ_posx, p_slope);

			}

			for(int i = 0; i < POS_LENGTH; i++) {
				// std::cout << "made it here" << i << "\n";
				if(PossiblePositions_bool[i]) {
					PossiblePositions_dist[i] = get_XY_DistanceToArc(PossiblePositions[i], end, center, &VirtualOutcomePositions[i]);
				}
			}
			double lastDist = 1000000;
			for(int i = 0; i < POS_LENGTH; i++) {
				if(PossiblePositions_bool[i] && PossiblePositions_dist[i] < lastDist) {
					lastDist = PossiblePositions_dist[i];
					moveDirection = (CUBEPOSITIONS)i;
				}
			}
			double current_distance = getDistanceBetweenPoints(VirtualPosition, end);
			//if current loc > max move dist, move or if next move is closer then move
			if(current_distance > MachineParams::Max_Move_Distance || getDistanceBetweenPoints(VirtualOutcomePositions[moveDirection], end) < getDistanceBetweenPoints(VirtualPosition, end)) {
				MachineParams::writemotors(moveDirection);
				VirtualPosition.x = VirtualOutcomePositions[moveDirection].x;
				VirtualPosition.y = VirtualOutcomePositions[moveDirection].y;
				VirtualPosition.z = VirtualOutcomePositions[moveDirection].z;
				MotorQuadrantPosition.x = PossiblePositions[moveDirection].x;
				MotorQuadrantPosition.y = PossiblePositions[moveDirection].y;
				MotorQuadrantPosition.z = PossiblePositions[moveDirection].z;
				// std::cout << "X:" << VirtualPosition.x << "    Y:" << VirtualPosition.y << "     RX:" << RealPosition.x << "     RY:" << RealPosition.y << "\n";
			} else {
				keeprunning = false;
			}
		}
	}

	static void moveLine(struct coordinate_s end) {
		bool Xdir = (last_exact_gcode_point.x - end.x) != 0;
		bool Ydir = (last_exact_gcode_point.y - end.y) != 0;
		bool Zdir = (last_exact_gcode_point.z - end.z) != 0;
		bool keeprunning = true;
		enum CUBEPOSITIONS moveDirection;
		while(keeprunning){
			moveDirection = X0Y0Z0;
			fillPossibleLocations(MotorQuadrantPosition, MachineParams::x_mm_per_tick, MachineParams::y_mm_per_tick, MachineParams::z_mm_per_tick);

			if(Xdir == true && Ydir == false && Zdir == false) {
				//move x only
				if((end.x - VirtualPosition.x) > 0) {
					moveDirection = X1Y0Z0;
					VirtualOutcomePositions[X1Y0Z0].x = VirtualPosition.x + MachineParams::x_mm_per_tick;
					VirtualOutcomePositions[X1Y0Z0].y = VirtualPosition.y;
					VirtualOutcomePositions[X1Y0Z0].z = VirtualPosition.z;
				} else {
					moveDirection = XN1Y0Z0;
					VirtualOutcomePositions[XN1Y0Z0].x = VirtualPosition.x - MachineParams::x_mm_per_tick;
					VirtualOutcomePositions[XN1Y0Z0].y = VirtualPosition.y;
					VirtualOutcomePositions[XN1Y0Z0].z = VirtualPosition.z;
				}
			} else if(Xdir == false && Ydir == true && Zdir == false) {
				//move y only
				if((end.y - VirtualPosition.y) > 0) {
					moveDirection = X0Y1Z0;
					VirtualOutcomePositions[X0Y1Z0].x = VirtualPosition.x;
					VirtualOutcomePositions[X0Y1Z0].y = VirtualPosition.y + MachineParams::y_mm_per_tick;
					VirtualOutcomePositions[X0Y1Z0].z = VirtualPosition.z;
				} else {
					moveDirection = X0YN1Z0;
					VirtualOutcomePositions[X0YN1Z0].x = VirtualPosition.x;
					VirtualOutcomePositions[X0YN1Z0].y = VirtualPosition.y - MachineParams::y_mm_per_tick;
					VirtualOutcomePositions[X0YN1Z0].z = VirtualPosition.z;
				}
			} else if(Xdir == false && Ydir == false && Zdir == true) {
				//move Z only
				if((end.z - VirtualPosition.z) > 0) {
					moveDirection = X0Y0Z1;
					VirtualOutcomePositions[X0Y0Z1].x = VirtualPosition.x;
					VirtualOutcomePositions[X0Y0Z1].y = VirtualPosition.y;
					VirtualOutcomePositions[X0Y0Z1].z = VirtualPosition.z + MachineParams::z_mm_per_tick;
				} else {
					moveDirection = X0Y0ZN1;
					VirtualOutcomePositions[X0Y0ZN1].x = VirtualPosition.x;
					VirtualOutcomePositions[X0Y0ZN1].y = VirtualPosition.y;
					VirtualOutcomePositions[X0Y0ZN1].z = VirtualPosition.z - MachineParams::z_mm_per_tick;
				}
				
			} else if(Xdir == true && Ydir == true && Zdir == false) {
				//move xy plane
				double slope = (end.y - VirtualPosition.y) / (end.x - VirtualPosition.x);
				//y=mx+b
				double b = end.y - (slope*end.x);
				mark_XY_PossibleLocations(VirtualPosition, end);
				for(int i = 0; i < POS_LENGTH; i++) {
					// std::cout << "made it here" << i << "\n";
					if(PossiblePositions_bool[i]) {
						PossiblePositions_dist[i] = get_XY_DistanceToLine(slope, b, PossiblePositions[i], &VirtualOutcomePositions[i]);
					}
				}
				double lastDist = 1000000;
				for(int i = 0; i < POS_LENGTH; i++) {
					if(PossiblePositions_dist[i] < lastDist && PossiblePositions_bool[i]) {
						lastDist = PossiblePositions_dist[i];
						moveDirection = (CUBEPOSITIONS)i;
					}
				}
	
			} else if(Xdir == true && Ydir == false && Zdir == true) {
				//move xz plane
			} else if(Xdir == false && Ydir == true && Zdir == true) {
				//move yz plane
			} else if(Xdir == true && Ydir == true && Zdir == true) {
				//move xyz plane
			}

			double current_distance = getDistanceBetweenPoints(VirtualPosition, end);
			//if current loc > max move dist, move or if next move is closer then move
			if(current_distance > MachineParams::Max_Move_Distance || getDistanceBetweenPoints(VirtualOutcomePositions[moveDirection], end) < getDistanceBetweenPoints(VirtualPosition, end)) {
				MachineParams::writemotors(moveDirection);
				VirtualPosition.x = VirtualOutcomePositions[moveDirection].x;
				VirtualPosition.y = VirtualOutcomePositions[moveDirection].y;
				VirtualPosition.z = VirtualOutcomePositions[moveDirection].z;
				MotorQuadrantPosition.x = PossiblePositions[moveDirection].x;
				MotorQuadrantPosition.y = PossiblePositions[moveDirection].y;
				MotorQuadrantPosition.z = PossiblePositions[moveDirection].z;
			} else {
				keeprunning = false;
			}
		}

	}

	static double getDistanceBetweenPoints(const struct coordinate_s &p1, const struct coordinate_s &p2) {
		return std::sqrt(std::pow((p1.x - p2.x),2) + std::pow((p1.y - p2.y), 2) + std::pow((p1.z - p2.z), 2));
	}
	static double get_XY_DistanceToLine(const double &slope, const double &b, const struct coordinate_s &point, struct coordinate_s *closest_point) {
		double parallel_slope = -1/slope;
		double parallel_b = point.y - (parallel_slope*point.x);
		closest_point->x = (b - parallel_b) / (parallel_slope - slope);
		closest_point->y = slope * closest_point->x + b;
		closest_point->z = point.z;

		return getDistanceBetweenPoints(point, *closest_point);
	}
	static double get_XY_DistanceToArc(const struct coordinate_s &point, const struct coordinate_s &arc_point, const struct coordinate_s &center, struct coordinate_s *closest_point) {
		double radius = getDistanceBetweenPoints(arc_point, center);
		double radius_point = getDistanceBetweenPoints(point, center);
		double distance = std::abs(radius_point - radius);

		double uvec_x = (point.x - center.x) / radius_point;
		double uvec_y = (point.y - center.y) / radius_point;
		closest_point->x = center.x + (uvec_x * radius);
		closest_point->y = center.y + (uvec_y * radius);
		closest_point->z = point.z;

		return distance;

	}

	static void fillPossibleLocations(const struct coordinate_s &current_real_loc, const double &x_per_tick, const double &y_per_tick, const double &z_per_tick) {
		PossiblePositions[XN1Y1Z1].x = current_real_loc.x - x_per_tick;
		PossiblePositions[XN1Y1Z1].y = current_real_loc.y + y_per_tick;
		PossiblePositions[XN1Y1Z1].z = current_real_loc.z + z_per_tick;

		PossiblePositions[X0Y1Z1].x = current_real_loc.x;
		PossiblePositions[X0Y1Z1].y = current_real_loc.y + y_per_tick;
		PossiblePositions[X0Y1Z1].z = current_real_loc.z + z_per_tick;

		PossiblePositions[X1Y1Z1].x = current_real_loc.x + x_per_tick;
		PossiblePositions[X1Y1Z1].y = current_real_loc.y + y_per_tick;
		PossiblePositions[X1Y1Z1].z = current_real_loc.z + z_per_tick;


		PossiblePositions[XN1Y0Z1].x = current_real_loc.x - x_per_tick;
		PossiblePositions[XN1Y0Z1].y = current_real_loc.y;
		PossiblePositions[XN1Y0Z1].z = current_real_loc.z + z_per_tick;

		PossiblePositions[X0Y0Z1].x = current_real_loc.x;
		PossiblePositions[X0Y0Z1].y = current_real_loc.y;
		PossiblePositions[X0Y0Z1].z = current_real_loc.z + z_per_tick;

		PossiblePositions[X1Y0Z1].x = current_real_loc.x + x_per_tick;
		PossiblePositions[X1Y0Z1].y = current_real_loc.y;
		PossiblePositions[X1Y0Z1].z = current_real_loc.z + z_per_tick;


		PossiblePositions[XN1YN1Z1].x = current_real_loc.x - x_per_tick;
		PossiblePositions[XN1YN1Z1].y = current_real_loc.y - y_per_tick;
		PossiblePositions[XN1YN1Z1].z = current_real_loc.z + z_per_tick;

		PossiblePositions[X0YN1Z1].x = current_real_loc.x;
		PossiblePositions[X0YN1Z1].y = current_real_loc.y - y_per_tick;
		PossiblePositions[X0YN1Z1].z = current_real_loc.z + z_per_tick;

		PossiblePositions[X1YN1Z1].x = current_real_loc.x + x_per_tick;
		PossiblePositions[X1YN1Z1].y = current_real_loc.y - y_per_tick;
		PossiblePositions[X1YN1Z1].z = current_real_loc.z + z_per_tick;



		PossiblePositions[XN1Y1Z0].x = current_real_loc.x - x_per_tick;
		PossiblePositions[XN1Y1Z0].y = current_real_loc.y + y_per_tick;
		PossiblePositions[XN1Y1Z0].z = current_real_loc.z;

		PossiblePositions[X0Y1Z0].x = current_real_loc.x;
		PossiblePositions[X0Y1Z0].y = current_real_loc.y + y_per_tick;
		PossiblePositions[X0Y1Z0].z = current_real_loc.z;

		PossiblePositions[X1Y1Z0].x = current_real_loc.x + x_per_tick;
		PossiblePositions[X1Y1Z0].y = current_real_loc.y + y_per_tick;
		PossiblePositions[X1Y1Z0].z = current_real_loc.z;


		PossiblePositions[XN1Y0Z0].x = current_real_loc.x - x_per_tick;
		PossiblePositions[XN1Y0Z0].y = current_real_loc.y;
		PossiblePositions[XN1Y0Z0].z = current_real_loc.z;

		PossiblePositions[X0Y0Z0].x = current_real_loc.x;
		PossiblePositions[X0Y0Z0].y = current_real_loc.y;
		PossiblePositions[X0Y0Z0].z = current_real_loc.z;

		PossiblePositions[X1Y0Z0].x = current_real_loc.x + x_per_tick;
		PossiblePositions[X1Y0Z0].y = current_real_loc.y;
		PossiblePositions[X1Y0Z0].z = current_real_loc.z;


		PossiblePositions[XN1YN1Z0].x = current_real_loc.x - x_per_tick;
		PossiblePositions[XN1YN1Z0].y = current_real_loc.y - y_per_tick;
		PossiblePositions[XN1YN1Z0].z = current_real_loc.z;

		PossiblePositions[X0YN1Z0].x = current_real_loc.x;
		PossiblePositions[X0YN1Z0].y = current_real_loc.y - y_per_tick;
		PossiblePositions[X0YN1Z0].z = current_real_loc.z;

		PossiblePositions[X1YN1Z0].x = current_real_loc.x + x_per_tick;
		PossiblePositions[X1YN1Z0].y = current_real_loc.y - y_per_tick;
		PossiblePositions[X1YN1Z0].z = current_real_loc.z;


		PossiblePositions[XN1Y1ZN1].x = current_real_loc.x - x_per_tick;
		PossiblePositions[XN1Y1ZN1].y = current_real_loc.y + y_per_tick;
		PossiblePositions[XN1Y1ZN1].z = current_real_loc.z - z_per_tick;

		PossiblePositions[X0Y1ZN1].x = current_real_loc.x;
		PossiblePositions[X0Y1ZN1].y = current_real_loc.y + y_per_tick;
		PossiblePositions[X0Y1ZN1].z = current_real_loc.z - z_per_tick;

		PossiblePositions[X1Y1ZN1].x = current_real_loc.x + x_per_tick;
		PossiblePositions[X1Y1ZN1].y = current_real_loc.y + y_per_tick;
		PossiblePositions[X1Y1ZN1].z = current_real_loc.z - z_per_tick;


		PossiblePositions[XN1Y0ZN1].x = current_real_loc.x - x_per_tick;
		PossiblePositions[XN1Y0ZN1].y = current_real_loc.y;
		PossiblePositions[XN1Y0ZN1].z = current_real_loc.z - z_per_tick;

		PossiblePositions[X0Y0ZN1].x = current_real_loc.x;
		PossiblePositions[X0Y0ZN1].y = current_real_loc.y;
		PossiblePositions[X0Y0ZN1].z = current_real_loc.z - z_per_tick;

		PossiblePositions[X1Y0ZN1].x = current_real_loc.x + x_per_tick;
		PossiblePositions[X1Y0ZN1].y = current_real_loc.y;
		PossiblePositions[X1Y0ZN1].z = current_real_loc.z - z_per_tick;


		PossiblePositions[XN1YN1ZN1].x = current_real_loc.x - x_per_tick;
		PossiblePositions[XN1YN1ZN1].y = current_real_loc.y - y_per_tick;
		PossiblePositions[XN1YN1ZN1].z = current_real_loc.z - z_per_tick;

		PossiblePositions[X0YN1ZN1].x = current_real_loc.x;
		PossiblePositions[X0YN1ZN1].y = current_real_loc.y - y_per_tick;
		PossiblePositions[X0YN1ZN1].z = current_real_loc.z - z_per_tick;

		PossiblePositions[X1YN1ZN1].x = current_real_loc.x + x_per_tick;
		PossiblePositions[X1YN1ZN1].y = current_real_loc.y - y_per_tick;
		PossiblePositions[X1YN1ZN1].z = current_real_loc.z - z_per_tick;



	}
	static void mark_YOnly_Direction(bool positive) {
		for(int i = 0; i < POS_LENGTH; i++) {
			PossiblePositions_bool[i] = false;
		}
		if(positive) {
				PossiblePositions_bool[XN1Y1Z0] = true;
				PossiblePositions_bool[X0Y1Z0] = true;
				PossiblePositions_bool[X1Y1Z0] = true;
		} else {
				PossiblePositions_bool[XN1YN1Z0] = true;
				PossiblePositions_bool[X0YN1Z0] = true;
				PossiblePositions_bool[X1YN1Z0] = true;
		}
	}
	static void mark_XOnly_Direction(bool positive) {
		for(int i = 0; i < POS_LENGTH; i++) {
			PossiblePositions_bool[i] = false;
		}
		if(positive) {
				PossiblePositions_bool[X1Y1Z0] = true;
				PossiblePositions_bool[X1Y0Z0] = true;
				PossiblePositions_bool[X1YN1Z0] = true;
		} else {
				PossiblePositions_bool[XN1Y1Z0] = true;
				PossiblePositions_bool[XN1Y0Z0] = true;
				PossiblePositions_bool[XN1YN1Z0] = true;
		}
	}
	static void mark_XY_PossibleLocations(const struct coordinate_s &virt_loc, const struct coordinate_s &new_loc) {
		for(int i = 0; i < POS_LENGTH; i++) {
			PossiblePositions_bool[i] = false;
		}
		// double vert = std::abs(new_loc.y - virt_loc.y);
		// double hor = std::abs(new_loc.x - virt_loc.x);
		if(new_loc.y > virt_loc.y) {
			if(new_loc.x < virt_loc.x ) {
				PossiblePositions_bool[XN1Y1Z0] = true;
			}

			//todo this should match machine paramaters if x/y moves in different amounts per tick(diagonal move might not be 45degrees)
			// if(vert > hor) {
				PossiblePositions_bool[X0Y1Z0] = true;
			// }

			if(new_loc.x > virt_loc.x) {
				PossiblePositions_bool[X1Y1Z0] = true;
			}
		}
		// if(hor > vert) {
			if(new_loc.x < virt_loc.x) {
				PossiblePositions_bool[XN1Y0Z0] = true;
			}
			if(new_loc.x > virt_loc.x) {
				PossiblePositions_bool[X1Y0Z0] = true;
			}
		// }

		if(new_loc.y < virt_loc.y) {
			if(new_loc.x < virt_loc.x) {
					PossiblePositions_bool[XN1YN1Z0] = true;
				}

			// if(vert > hor) {
			PossiblePositions_bool[X0YN1Z0] = true;
			// }

			if(new_loc.x > virt_loc.x) {
					PossiblePositions_bool[X1YN1Z0] = true;
				}
		}
	}
	static void mark_XYcircle_PossibleLocations(bool negY_direction, double slope) {
		for(int i = 0; i < POS_LENGTH; i++) {
			PossiblePositions_bool[i] = false;
		}
		//
		if(negY_direction == true) {
			PossiblePositions_bool[X0YN1Z0] = true;
			if(slope < 0) {
				PossiblePositions_bool[X1Y0Z0] = true;
				PossiblePositions_bool[X1YN1Z0] = true;
			} else {
				PossiblePositions_bool[XN1YN1Z0] = true;
				PossiblePositions_bool[XN1Y0Z0] = true;
			}
		}
		else {
			PossiblePositions_bool[X0Y1Z0] = true;
			if(slope < 0) {
				PossiblePositions_bool[XN1Y0Z0] = true;
				PossiblePositions_bool[XN1Y1Z0] = true;
			} else {
				PossiblePositions_bool[X1Y1Z0] = true;
				PossiblePositions_bool[X1Y0Z0] = true;
			}
		}
	}
	// static void mark_XOnly_PossibleLocations(const struct coordinate_s &virt_loc, const struct coordinate_s &new_loc) {
	// 	for(int i = 0; i < POS_LENGTH; i++) {
	// 		PossiblePositions_bool[i] = false;
	// 	}
	// 	if(new_loc.x < virt_loc.x) {
	// 		PossiblePositions_bool[XN1Y0Z0] = true;
	// 	}
	// 	if(new_loc.x > virt_loc.x) {
	// 		PossiblePositions_bool[X1Y0Z0] = true;
	// 	}
	// }
	// static void mark_YOnly_PossibleLocations(const struct coordinate_s &virt_loc, const struct coordinate_s &new_loc) {
	// 	for(int i = 0; i < POS_LENGTH; i++) {
	// 		PossiblePositions_bool[i] = false;
	// 	}
	// 	if(new_loc.y > virt_loc.y) {
	// 		PossiblePositions_bool[X0Y1Z0] = true;
	// 	}
	// 	if(new_loc.y < virt_loc.y) {
	// 		PossiblePositions_bool[X0YN1Z0] = true;
	// 	}
	// }

};

std::vector<std::string> CNC_GCODE_PROCESSING::fileContents;
	
struct coordinate_s CNC_GCODE_PROCESSING::MotorQuadrantPosition = {0};
struct coordinate_s CNC_GCODE_PROCESSING::VirtualPosition = {0};
struct coordinate_s CNC_GCODE_PROCESSING::last_exact_gcode_point = {0};
struct coordinate_s CNC_GCODE_PROCESSING::PossiblePositions[POS_LENGTH];
struct coordinate_s CNC_GCODE_PROCESSING::VirtualOutcomePositions[POS_LENGTH];
bool CNC_GCODE_PROCESSING::PossiblePositions_bool[POS_LENGTH];
double CNC_GCODE_PROCESSING::PossiblePositions_dist[POS_LENGTH];
struct coordinate_s CNC_GCODE_PROCESSING::WorkCoordinateOffsets[MAX_WORK_COORDINATE_OFFSETS];
struct coordinate_s *CNC_GCODE_PROCESSING::current_work_offset = nullptr;

int main() {
	std::cout << "!!!Hello World!!!" << std::endl; // prints !!!Hello World!!!
	cncHardwareControl::total_time = 0;
	MachineParams::init();
	// struct pollfd pfd;
    // int ret;
    // std::cout << sizeof(CNC_OP_DataStructure) << "\n";
	// assert(sizeof(CNC_OP_DataStructure) == 8);


	cncHardwareControl::Reset_Stop_Bit(true);

	CNC_GCODE_PROCESSING::LoadFile(GCODEFILE);
	// double total_time = 0;
	std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
	CNC_GCODE_PROCESSING::runGcodeFile();

	std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> diff = end - start;
	std::cout << "time is " << diff.count() << "\n";
	std::cout << "time loaded is " << cncHardwareControl::total_time*ns_per_clock/1000000 << "\n";
	//zero
	// VirtualPosition.x = 0;
	// VirtualPosition.y = 0;
	// VirtualPosition.z = 0;
	// RealPosition.x = 0;
	// RealPosition.y = 0;
	// RealPosition.z = 0;



// std::cout << "made it here 1\n";
	// bool keepgoing = true;

	//read gcodeMoveTypes
	// std::string next_move = "G01 X5 Y12 F200";
	//get code, xyz endpoint, feedrate

	//speed = 200mm/min
	// double feedrate = 17000.0 / 60.0; //mm/seconds
	// double time_x_move = x_mm_per_tick * 1000000000 / feedrate;
	// double time_y_move = y_mm_per_tick * 1000000000 / feedrate;
	// double time_xy_move = xy_mm_per_tick * 1000000000 / feedrate;

	// struct coordinate_s newPos;
	// newPos.x = 50*25.4;
	// newPos.y = 120*25.4;
	// newPos.z = 0;

	// double slope = (newPos.y - VirtualPosition.y) / (newPos.x - VirtualPosition.x);
	//y=mx+b
	//b = y - (mx)
	// double b = newPos.y - (slope*newPos.x);
	//possible move +y +x+y +x 00
	



	// while(keepgoing){
	// 	//what type of gcode
	// 	fillPossibleLocations(RealPosition, x_mm_per_tick, y_mm_per_tick);
	// 	markPossibleLocations(VirtualPosition, newPos);

	// 	for(int i = 0; i < POS_LENGTH; i++) {
	// 		// std::cout << "made it here" << i << "\n";
	// 		if(PossiblePositions_bool[i]) {
	// 			PossiblePositions_dist[i] = getDistanceToLine(slope, b, PossiblePositions[i], &VirtualOutcomePositions[i]);
	// 		}
	// 	}
	// 	enum CUBEPOSITIONS moveDirection = X0Y0Z0;
	// 	double lastDist = 1000000;
	// 	for(int i = 0; i < POS_LENGTH; i++) {
	// 		if(PossiblePositions_dist[i] < lastDist && PossiblePositions_bool[i]) {
	// 			lastDist = PossiblePositions_dist[i];
	// 			moveDirection = (CUBEPOSITIONS)i;
	// 		}
	// 	}
	// 	if(getDistanceBetweenPoints(PossiblePositions[moveDirection], newPos) > getDistanceBetweenPoints(VirtualPosition, newPos)) {
	// 		keepgoing = false;
	// 	}

	// 	cncHardwareControl::unset_motors();
	// 	cncHardwareControl::setTimer(0);
	// 	switch(moveDirection) {
	// 		case (XN1Y1Z0): {
	// 			cncHardwareControl::setTimer(time_xy_move);
	// 			cncHardwareControl::setMotor(MOTORNUMBERS_M1, true, DIRECTION_NEG);
	// 			cncHardwareControl::setMotor(MOTORNUMBERS_M2, true, DIRECTION_POS);
	// 			break;
	// 		}
	// 		case (X0Y1Z0): {
	// 			cncHardwareControl::setTimer(time_y_move);
	// 			// cnc.setMotor(MOTORNUMBERS_M1, true, DIRECTION_NEG);
	// 			cncHardwareControl::setMotor(MOTORNUMBERS_M2, true, DIRECTION_POS);
	// 			break;
	// 		}
	// 		case (X1Y1Z0): {
	// 			cncHardwareControl::setTimer(time_xy_move);
	// 			cncHardwareControl::setMotor(MOTORNUMBERS_M1, true, DIRECTION_POS);
	// 			cncHardwareControl::setMotor(MOTORNUMBERS_M2, true, DIRECTION_POS);
	// 			break;
	// 		}
	// 		case (XN1Y0Z0): {
	// 			cncHardwareControl::setTimer(time_x_move);
	// 			cncHardwareControl::setMotor(MOTORNUMBERS_M1, true, DIRECTION_NEG);
	// 			// cnc.setMotor(MOTORNUMBERS_M2, true, DIRECTION_POS);
	// 			break;
	// 		}
	// 		case (X0Y0Z0): {
	// 			std::cout << "move center error\n";
	// 			// cnc.setTimer(time_xy_move);
	// 			// cnc.setMotor(MOTORNUMBERS_M1, true, DIRECTION_NEG);
	// 			// cnc.setMotor(MOTORNUMBERS_M2, true, DIRECTION_POS);
	// 			break;
	// 		}
	// 		case (X1Y0Z0): {
	// 			cncHardwareControl::setTimer(time_x_move);
	// 			cncHardwareControl::setMotor(MOTORNUMBERS_M1, true, DIRECTION_POS);
	// 			// cnc.setMotor(MOTORNUMBERS_M2, true, DIRECTION_POS);
	// 			break;
	// 		}
	// 		case (XN1YN1Z0): {
	// 			cncHardwareControl::setTimer(time_xy_move);
	// 			cncHardwareControl::setMotor(MOTORNUMBERS_M1, true, DIRECTION_NEG);
	// 			cncHardwareControl::setMotor(MOTORNUMBERS_M2, true, DIRECTION_NEG);
	// 			break;
	// 		}
	// 		case (X0YN1Z0): {
	// 			cncHardwareControl::setTimer(time_y_move);
	// 			// cnc.setMotor(MOTORNUMBERS_M1, true, DIRECTION_NEG);
	// 			cncHardwareControl::setMotor(MOTORNUMBERS_M2, true, DIRECTION_NEG);
	// 			break;
	// 		}
	// 		case (X1YN1Z0): {
	// 			cncHardwareControl::setTimer(time_xy_move);
	// 			cncHardwareControl::setMotor(MOTORNUMBERS_M1, true, DIRECTION_POS);
	// 			cncHardwareControl::setMotor(MOTORNUMBERS_M2, true, DIRECTION_NEG);
	// 			break;
	// 		}
	// 		default:
	// 			break;
	// 	}	
	// 	cncHardwareControl::write_Data();
	// 	// total_time += cnc.CNC_op.timer;

	// 	//update pos
	// 	VirtualPosition.x = VirtualOutcomePositions[moveDirection].x;
	// 	VirtualPosition.y = VirtualOutcomePositions[moveDirection].y;
	// 	VirtualPosition.z = VirtualOutcomePositions[moveDirection].z;
	// 	RealPosition.x = PossiblePositions[moveDirection].x;
	// 	RealPosition.y = PossiblePositions[moveDirection].y;
	// 	RealPosition.z = PossiblePositions[moveDirection].z;


	// }

	// std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
	// std::chrono::duration<double, std::milli> diff = end - start;
	// std::cout << "time is " << diff.count() << "\n";
	// // std::cout << "time loaded is " << total_time*ns_per_clock/1000000 << "\n";


	return 0;
}
