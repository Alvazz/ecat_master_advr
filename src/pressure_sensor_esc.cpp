#include <iit/ecat/advr/pressure_sensor_esc.h>

using namespace iit::ecat::advr;


static const iit::ecat::objd_t source_SDOs[] = {

    // SDO6000[] =
    //
    // {0x6000, 0x1, DTYPE_UNSIGNED8,   8, ATYPE_RO, forceXY.c_str(), 0};
    //
    {0x6000, 0x1, DTYPE_UNSIGNED16,    16, ATYPE_RO, fault.c_str(),    0},
    {0x6000, 0x2, DTYPE_UNSIGNED16,    16, ATYPE_RO, rtt.c_str(),      0},
    // SDO7000[] =
    {0x7000, 0x1, DTYPE_UNSIGNED16,     16, ATYPE_RW, ts.c_str(),       0},
    // SDO8000[] =
    {0x8000, 0x1, DTYPE_INTEGER32,      32, ATYPE_RW, BlockControl.c_str(),         0},
    {0x8000, 0x2, DTYPE_INTEGER32,      32, ATYPE_RW, NumAvSamples.c_str(),         0},
    {0x8000, 0x3, DTYPE_INTEGER16,      16, ATYPE_RW, ConfigFlags.c_str(),          0},
    {0x8000, 0x4, DTYPE_INTEGER16,      16, ATYPE_RW, Sensor_number.c_str(),        0},
    {0x8000, 0x5, DTYPE_INTEGER16,      16, ATYPE_RW, Sensor_robot_id.c_str(),      0},
    // SDO8001[] =
    {0x8001, 0x1, DTYPE_VISIBLE_STRING, 64, ATYPE_RO, fw_ver.c_str(),               0},
    {0x8001, 0x2, DTYPE_INTEGER16,      16, ATYPE_RW, ack_board_faults.c_str(),     0},
    {0x8001, 0x3, DTYPE_UNSIGNED16,     16, ATYPE_RW, flash_params_cmd.c_str(),     0},
    {0x8001, 0x4, DTYPE_UNSIGNED16,     16, ATYPE_RO, flash_params_cmd_ack.c_str(), 0},

    {0, 0, 0, 0, 0, 0, 0 }
};

int  iit::ecat::advr::get_SDOs_size( void ) { return sizeof ( source_SDOs ) /sizeof ( iit::ecat::objd_t ); };
    
void iit::ecat::advr::copy_source_SDOs(iit::ecat::objd_t * dest) {  memcpy ( ( void* ) dest, source_SDOs, sizeof ( source_SDOs ) ); }

