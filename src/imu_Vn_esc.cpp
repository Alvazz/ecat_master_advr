/*

        Copyright (C) 2015 Italian Institute of Technology

        Developer:
        Alessio Margan (2017 , alessio.margan@iit.it)

        Generated with ./c_gen.py :
        2017-02-16 11:37:36
*/

#include <iit/ecat/advr/imu_Vn_esc.h>

using namespace iit::ecat;
using namespace iit::ecat::advr;


static const iit::ecat::objd_t source_SDOs[] = {

	{ 0x6000, 1, DTYPE_REAL32,		32, ATYPE_RO, "x_rate",		0},
	{ 0x6000, 2, DTYPE_REAL32,		32, ATYPE_RO, "y_rate",		0},
	{ 0x6000, 3, DTYPE_REAL32,		32, ATYPE_RO, "z_rate",		0},
	{ 0x6000, 4, DTYPE_REAL32,		32, ATYPE_RO, "x_acc",		0},
	{ 0x6000, 5, DTYPE_REAL32,		32, ATYPE_RO, "y_acc",		0},
	{ 0x6000, 6, DTYPE_REAL32,		32, ATYPE_RO, "z_acc",		0},
	{ 0x6000, 7, DTYPE_REAL32,		32, ATYPE_RO, "x_quat",		0},
	{ 0x6000, 8, DTYPE_REAL32,		32, ATYPE_RO, "y_quat",		0},
	{ 0x6000, 9, DTYPE_REAL32,		32, ATYPE_RO, "z_quat",		0},
	{ 0x6000, 10, DTYPE_REAL32,		32, ATYPE_RO, "w_quat",		0},
	{ 0x6000, 11, DTYPE_UNSIGNED32,		32, ATYPE_RO, "imu_ts",		0},
	{ 0x6000, 12, DTYPE_UNSIGNED16,		16, ATYPE_RO, "temperature",		0},
	{ 0x6000, 13, DTYPE_UNSIGNED16,		16, ATYPE_RO, "digital_in",		0},
	{ 0x6000, 14, DTYPE_UNSIGNED16,		16, ATYPE_RO, "fault",		0},
	{ 0x6000, 15, DTYPE_UNSIGNED16,		16, ATYPE_RO, "rtt",		0},

	{ 0x7000, 1, DTYPE_UNSIGNED16,		16, ATYPE_RW, "fault_ack",		0},
	{ 0x7000, 2, DTYPE_UNSIGNED16,		16, ATYPE_RW, "digital_out",		0},
	{ 0x7000, 3, DTYPE_UNSIGNED16,		16, ATYPE_RW, "ts",		0},

	{ 0x8000, 1, DTYPE_INTEGER16,		16, ATYPE_RW, "config_flags",		0},
	{ 0x8000, 2, DTYPE_INTEGER16,		16, ATYPE_RW, "config_flags2",		0},
	{ 0x8000, 3, DTYPE_INTEGER16,		16, ATYPE_RW, "digital_out_cfg",		0},
	{ 0x8000, 4, DTYPE_INTEGER16,		16, ATYPE_RW, "digital_in_cfg",		0},
	{ 0x8000, 5, DTYPE_INTEGER16,		16, ATYPE_RW, "digital_ain_cfg",		0},
	{ 0x8000, 6, DTYPE_INTEGER16,		16, ATYPE_RW, "function_cfg",		0},
	{ 0x8000, 7, DTYPE_INTEGER16,		16, ATYPE_RW, "sensor_num",		0},
	{ 0x8000, 8, DTYPE_INTEGER16,		16, ATYPE_RW, "joint_robot_id",		0},

	{ 0x8001, 1, DTYPE_VISIBLE_STRING,		64, ATYPE_RO, "fw_ver",		0},
	{ 0x8001, 2, DTYPE_UNSIGNED32,		32, ATYPE_RW, "ack_board_faults",		0},
	{ 0x8001, 3, DTYPE_REAL32,		32, ATYPE_RW, "dbg_1",		0},
	{ 0x8001, 4, DTYPE_REAL32,		32, ATYPE_RW, "dbg_2",		0},
	{ 0x8001, 5, DTYPE_REAL32,		32, ATYPE_RW, "dbg_3",		0},
	{ 0x8001, 6, DTYPE_UNSIGNED16,		16, ATYPE_RW, "flash_params_cmd",		0},
	{ 0x8001, 7, DTYPE_UNSIGNED16,		16, ATYPE_RO, "flash_params_cmd_ack",		0},

	{0, 0, 0, 0, 0, 0, 0 }
};

void ImuVnESC::init_SDOs(void) {                                         

    int objd_num, i = 0;                                              

    objd_num = sizeof(source_SDOs)/sizeof(objd_t);                    
    SDOs = new objd_t [objd_num];                                     

    memcpy((void*)SDOs, source_SDOs, sizeof(source_SDOs));

    //0x6000
    SDOs[i++].data = (void*)&ImuVnESC::rx_pdo.x_rate;
    SDOs[i++].data = (void*)&ImuVnESC::rx_pdo.y_rate;
    SDOs[i++].data = (void*)&ImuVnESC::rx_pdo.z_rate;
    SDOs[i++].data = (void*)&ImuVnESC::rx_pdo.x_acc;
    SDOs[i++].data = (void*)&ImuVnESC::rx_pdo.y_acc;
    SDOs[i++].data = (void*)&ImuVnESC::rx_pdo.z_acc;
    SDOs[i++].data = (void*)&ImuVnESC::rx_pdo.x_quat;
    SDOs[i++].data = (void*)&ImuVnESC::rx_pdo.y_quat;
    SDOs[i++].data = (void*)&ImuVnESC::rx_pdo.z_quat;
    SDOs[i++].data = (void*)&ImuVnESC::rx_pdo.w_quat;
    SDOs[i++].data = (void*)&ImuVnESC::rx_pdo.imu_ts;
    SDOs[i++].data = (void*)&ImuVnESC::rx_pdo.temperature;
    SDOs[i++].data = (void*)&ImuVnESC::rx_pdo.digital_in;
    SDOs[i++].data = (void*)&ImuVnESC::rx_pdo.fault;
    SDOs[i++].data = (void*)&ImuVnESC::rx_pdo.rtt;

    //0x7000
    SDOs[i++].data = (void*)&ImuVnESC::tx_pdo.fault_ack;
    SDOs[i++].data = (void*)&ImuVnESC::tx_pdo.digital_out;
    SDOs[i++].data = (void*)&ImuVnESC::tx_pdo.ts;

    //0x8000
    SDOs[i++].data = (void*)&ImuVnESC::sdo.config_flags;
    SDOs[i++].data = (void*)&ImuVnESC::sdo.config_flags2;
    SDOs[i++].data = (void*)&ImuVnESC::sdo.digital_out_cfg;
    SDOs[i++].data = (void*)&ImuVnESC::sdo.digital_in_cfg;
    SDOs[i++].data = (void*)&ImuVnESC::sdo.digital_ain_cfg;
    SDOs[i++].data = (void*)&ImuVnESC::sdo.function_cfg;
    SDOs[i++].data = (void*)&ImuVnESC::sdo.sensor_num;
    SDOs[i++].data = (void*)&ImuVnESC::sdo.joint_robot_id;

    //0x8001
    SDOs[i++].data = (void*)&ImuVnESC::sdo.fw_ver;
    SDOs[i++].data = (void*)&ImuVnESC::sdo.ack_board_faults;
    SDOs[i++].data = (void*)&ImuVnESC::sdo.dbg_1;
    SDOs[i++].data = (void*)&ImuVnESC::sdo.dbg_2;
    SDOs[i++].data = (void*)&ImuVnESC::sdo.dbg_3;
    SDOs[i++].data = (void*)&ImuVnESC::sdo.flash_params_cmd;
    SDOs[i++].data = (void*)&ImuVnESC::sdo.flash_params_cmd_ack;


    // end marker
    SDOs[i++].data = 0;

    assert ( objd_num == i );
}

// kate: indent-mode cstyle; indent-width 4; replace-tabs on;
