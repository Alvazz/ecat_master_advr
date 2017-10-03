#include <iit/ecat/advr/lxm32i_esc.h>

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

    // 0x1A00 slave TxPDO 
    {0x6041, 0x0,  DTYPE_UNSIGNED16, 16, ATYPE_RO, "_DCOMstatus",   0 },
    {0x6064, 0x0,  DTYPE_INTEGER32,  32, ATYPE_RO, "_p_act",        0 },
    {0x603F, 0x0,  DTYPE_UNSIGNED16, 16, ATYPE_RO, "_LastError",    0 },
    {0x3008, 0x1,  DTYPE_UNSIGNED16, 16, ATYPE_RO, "_IO_act",       0 },
    // 0x1600 slave RxPDO 
    {0x6040, 0x0,  DTYPE_UNSIGNED16, 16, ATYPE_RW, "DCOMcontrol",   0 },
    {0x607A, 0x0,  DTYPE_INTEGER32,  32, ATYPE_RW, "PPp_target",    0 },
    {0x3008, 0x11, DTYPE_UNSIGNED16, 16, ATYPE_RW, "IO_DQ_set",     0 },
    // --> 0x1600
    {0x1C12, 0x0,  DTYPE_UNSIGNED8,   8, ATYPE_RW, "RxPdoMapCnt",   0 },
    {0x1C12, 0x1,  DTYPE_UNSIGNED16, 16, ATYPE_RW, "RxPdoMap",      0 },
    //
    {0x1600, 0x0,  DTYPE_UNSIGNED8,   8, ATYPE_RW, "RxElemCnt",     0 },
    {0x1600, 0x1,  DTYPE_UNSIGNED32, 32, ATYPE_RW, "Rx1",           0 },
    {0x1600, 0x2,  DTYPE_UNSIGNED32, 32, ATYPE_RW, "Rx2",           0 },
    {0x1600, 0x3,  DTYPE_UNSIGNED32, 32, ATYPE_RW, "Rx3",           0 },
    // --> 0x1A00
    {0x1C13, 0x0,  DTYPE_UNSIGNED8,   8, ATYPE_RW, "TxPdoMapCnt",   0 },
    {0x1C13, 0x1,  DTYPE_UNSIGNED16, 16, ATYPE_RW, "TxPdoMap",      0 },
    //
    {0x1A00, 0x0,  DTYPE_UNSIGNED8,   8, ATYPE_RW, "TxElemCnt",     0 },
    {0x1A00, 0x1,  DTYPE_UNSIGNED32, 32, ATYPE_RW, "Tx1",           0 },
    {0x1A00, 0x2,  DTYPE_UNSIGNED32, 32, ATYPE_RW, "Tx2",           0 },
    {0x1A00, 0x3,  DTYPE_UNSIGNED32, 32, ATYPE_RW, "Tx3",           0 },
    {0x1A00, 0x4,  DTYPE_UNSIGNED32, 32, ATYPE_RW, "Tx4",           0 },
    
    
    {0, 0, 0, 0, 0, 0, 0 }
};



void LXM32iESC::init_SDOs ( void ) {

    int objd_num, i = 0;

    objd_num = sizeof ( source_SDOs ) /sizeof ( objd_t );
    SDOs = new objd_t [objd_num];

    memcpy ( ( void* ) SDOs, source_SDOs, sizeof ( source_SDOs ) );

    // 
    SDOs[i++].data = ( void* ) &LXM32iESC::rx_pdo._DCOMstatus;
    SDOs[i++].data = ( void* ) &LXM32iESC::rx_pdo._p_act;
    SDOs[i++].data = ( void* ) &LXM32iESC::rx_pdo._LastError;
    SDOs[i++].data = ( void* ) &LXM32iESC::rx_pdo._IO_act;
    // 
    SDOs[i++].data = ( void* ) &LXM32iESC::tx_pdo.DCOMcontrol;
    SDOs[i++].data = ( void* ) &LXM32iESC::tx_pdo.PPp_target;
    SDOs[i++].data = ( void* ) &LXM32iESC::tx_pdo.IO_DQ_set;
    //
    SDOs[i++].data = ( void* ) &LXM32iESC::sdo.rxPdoMapCnt;
    SDOs[i++].data = ( void* ) &LXM32iESC::sdo.rxPdoMap;
    SDOs[i++].data = ( void* ) &LXM32iESC::sdo.rxElemCnt;
    for(int j=0;j<=2;j++) SDOs[i++].data = ( void* ) &LXM32iESC::sdo.rxMap[j];
    
    //
    SDOs[i++].data = ( void* ) &LXM32iESC::sdo.txPdoMapCnt;
    SDOs[i++].data = ( void* ) &LXM32iESC::sdo.txPdoMap;
    SDOs[i++].data = ( void* ) &LXM32iESC::sdo.txElemCnt;
    for(int j=0;j<=3;j++) SDOs[i++].data = ( void* ) &LXM32iESC::sdo.txMap[j];
    
    SDOs[i++].data = 0;

    assert ( objd_num == i );
}
