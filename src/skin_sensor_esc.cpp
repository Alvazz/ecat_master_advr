#include <iit/ecat/advr/skin_sensor_esc.h>

using namespace iit::ecat;
using namespace iit::ecat::advr;

static const char acName6000[] = "Inputs";
static const char acName6000_0[] = "Number of Elements";

static const char acName7000[] = "Outputs";
static const char acName7000_0[] = "Number of Elements";
static const char acName7000_1[] = "pos_ref";
static const char acName7000_2[] = "tor_ref";
static const char acName7000_3[] = "direct_ref";
static const char acName7000_4[] = "ts";

static const char acName8000[] = "Flash Parameter";
static const char acName8000_0[] = "Number of Elements";
static const char acName8000_1[] = "Block control";
static const char acName8000_2[] = "Num Av Samples";
static const char acName8000_3[] = "ConfigFlags";
static const char acName8000_4[] = "Sensor_number";
static const char acName8000_5[] = "Sensor_robot_id";

static const char acName8001[] = "Parameter";
static const char acName8001_1[] = "fw_ver";
static const char acName8001_2[] = "ack_board_faults";
static const char acName8001_3[] = "flash_params_cmd";
static const char acName8001_4[] = "flash_params_cmd_ack";




static const iit::ecat::objd_t source_SDOs[] = {

    // SDO6000[] =
    {0x6000, 0x1,  DTYPE_UNSIGNED8,   8, ATYPE_RO, "force1A", 0},
    {0x6000, 0x2,  DTYPE_UNSIGNED8,   8, ATYPE_RO, "force1B", 0},
    {0x6000, 0x3,  DTYPE_UNSIGNED8,   8, ATYPE_RO, "force1C", 0},
    {0x6000, 0x4,  DTYPE_UNSIGNED8,   8, ATYPE_RO, "force2A", 0},
    {0x6000, 0x5,  DTYPE_UNSIGNED8,   8, ATYPE_RO, "force2B", 0},
    {0x6000, 0x6,  DTYPE_UNSIGNED8,   8, ATYPE_RO, "force2C", 0},
    {0x6000, 0x7,  DTYPE_UNSIGNED8,   8, ATYPE_RO, "force3A", 0},
    {0x6000, 0x8,  DTYPE_UNSIGNED8,   8, ATYPE_RO, "force3B", 0},
    {0x6000, 0x9,  DTYPE_UNSIGNED8,   8, ATYPE_RO, "force3C", 0},
    {0x6000, 0xa,  DTYPE_UNSIGNED8,   8, ATYPE_RO, "force4A", 0},
    {0x6000, 0xb,  DTYPE_UNSIGNED8,   8, ATYPE_RO, "force4B", 0},
    {0x6000, 0xc,  DTYPE_UNSIGNED8,   8, ATYPE_RO, "force4C", 0},
    {0x6000, 0xd,  DTYPE_UNSIGNED8,   8, ATYPE_RO, "force5A", 0},
    {0x6000, 0xe,  DTYPE_UNSIGNED8,   8, ATYPE_RO, "force5B", 0},
    {0x6000, 0xf,  DTYPE_UNSIGNED8,   8, ATYPE_RO, "force5C", 0},
    {0x6000, 0x10, DTYPE_UNSIGNED8,   8, ATYPE_RO, "force6A", 0},
    {0x6000, 0x11, DTYPE_UNSIGNED8,   8, ATYPE_RO, "force6B", 0},
    {0x6000, 0x12, DTYPE_UNSIGNED8,   8, ATYPE_RO, "force6C", 0},
    {0x6000, 0x13, DTYPE_UNSIGNED8,   8, ATYPE_RO, "force7A", 0},
    {0x6000, 0x14, DTYPE_UNSIGNED8,   8, ATYPE_RO, "force7B", 0},
    {0x6000, 0x15, DTYPE_UNSIGNED8,   8, ATYPE_RO, "force7C", 0},
    {0x6000, 0x16, DTYPE_UNSIGNED8,   8, ATYPE_RO, "force8A", 0},
    {0x6000, 0x17, DTYPE_UNSIGNED8,   8, ATYPE_RO, "force8B", 0},
    {0x6000, 0x18, DTYPE_UNSIGNED8,   8, ATYPE_RO, "force8C", 0},
    {0x6000, 0x19, DTYPE_UNSIGNED16, 16, ATYPE_RO, "fault", 0},
    {0x6000, 0x1a, DTYPE_UNSIGNED16, 16, ATYPE_RO, "rtt", 0},
    // SDO7000[] =
    {0x7000, 0x1,  DTYPE_UNSIGNED16, 16, ATYPE_RW, "ts", 0},
    // SDO8000[] =
    {0x8000, 0x1, DTYPE_INTEGER32,   32, ATYPE_RW, "Block_control",  0},
    {0x8000, 0x2, DTYPE_INTEGER32,   32, ATYPE_RW, "NumAvSamples",  0},
    {0x8000, 0x3, DTYPE_INTEGER16,   16, ATYPE_RW, "ConfigFlags",  0},
    {0x8000, 0x4, DTYPE_INTEGER16,   16, ATYPE_RW, "sensor_number",  0},
    {0x8000, 0x5, DTYPE_INTEGER16,   16, ATYPE_RW, "sensor_robot_id",  0},
    // SDO8001[] =
    {0x8001, 0x1, DTYPE_VISIBLE_STRING,   64, ATYPE_RO, "fw_ver", 0},
    {0x8001, 0x2, DTYPE_INTEGER16,        16, ATYPE_RW, "ack_board_faults", 0},
    {0x8001, 0x3, DTYPE_UNSIGNED16,       16, ATYPE_RW, "flash_params_cmd", 0},
    {0x8001, 0x4, DTYPE_UNSIGNED16,       16, ATYPE_RO, "flash_params_cmd_ack", 0},

    {0, 0, 0, 0, 0, 0, 0 }
};



void SkinSensorESC::init_SDOs ( void ) {

    int objd_num, i = 0;

    objd_num = sizeof ( source_SDOs ) /sizeof ( objd_t );
    SDOs = new objd_t [objd_num];

    memcpy ( ( void* ) SDOs, source_SDOs, sizeof ( source_SDOs ) );

    // 0x6000
    for(int j = 0; j < SKIN_SENSOR_NUMBER; j++) {
        SDOs[i++].data = ( void* ) &SkinSensorESC::rx_pdo.forceXY[j];
    }
    SDOs[i++].data = ( void* ) &SkinSensorESC::rx_pdo.fault;
    SDOs[i++].data = ( void* ) &SkinSensorESC::rx_pdo.rtt;
    // 0x7000
    SDOs[i++].data = ( void* ) &SkinSensorESC::tx_pdo.ts;
    // 0x8000
    SDOs[i++].data = ( void* ) &SkinSensorESC::sdo.Block_control;
    SDOs[i++].data = ( void* ) &SkinSensorESC::sdo.NumAvSamples;
    SDOs[i++].data = ( void* ) &SkinSensorESC::sdo.ConfigFlags;
    SDOs[i++].data = ( void* ) &SkinSensorESC::sdo.sensor_number;
    SDOs[i++].data = ( void* ) &SkinSensorESC::sdo.sensor_robot_id;
    // 0x8001
    SDOs[i++].data = ( void* ) &SkinSensorESC::sdo.firmware_version;
    SDOs[i++].data = ( void* ) &SkinSensorESC::sdo.ack_board_fault;
    SDOs[i++].data = ( void* ) &SkinSensorESC::sdo.flash_params_cmd;
    SDOs[i++].data = ( void* ) &SkinSensorESC::sdo.flash_params_cmd_ack;

    SDOs[i++].data = 0;

    assert ( objd_num == i );
}
