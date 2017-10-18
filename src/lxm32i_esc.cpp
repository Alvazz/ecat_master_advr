#include <iit/ecat/advr/lxm32i_esc.h>

//using namespace iit::ecat;
using namespace iit::ecat::advr;


static const iit::ecat::objd_t source_SDOs[] = {

    // --> 0x1600
    {0x1C12, 0x0,  DTYPE_UNSIGNED8,   8, ATYPE_RW, RxPdoMapCnt.c_str(),   0 },
    {0x1C12, 0x1,  DTYPE_UNSIGNED16, 16, ATYPE_RW, RxPdoMap.c_str(),      0 },
    // --> 0x1A00
    {0x1C13, 0x0,  DTYPE_UNSIGNED8,   8, ATYPE_RW, TxPdoMapCnt.c_str(),   0 },
    {0x1C13, 0x1,  DTYPE_UNSIGNED16, 16, ATYPE_RW, TxPdoMap.c_str(),      0 },
    //
    {0x1600, 0x0,  DTYPE_UNSIGNED8,   8, ATYPE_RW, "RxElemCnt",     0 },
    {0x1600, 0x1,  DTYPE_UNSIGNED32, 32, ATYPE_RW, RPdo1.c_str(),   0 },
    {0x1600, 0x2,  DTYPE_UNSIGNED32, 32, ATYPE_RW, RPdo2.c_str(),   0 },
    {0x1600, 0x3,  DTYPE_UNSIGNED32, 32, ATYPE_RW, RPdo3.c_str(),   0 },
    {0x1600, 0x4,  DTYPE_UNSIGNED32, 32, ATYPE_RW, RPdo4.c_str(),   0 },
    {0x1600, 0x5,  DTYPE_UNSIGNED32, 32, ATYPE_RW, RPdo5.c_str(),   0 },
    {0x1600, 0x6,  DTYPE_UNSIGNED32, 32, ATYPE_RW, RPdo6.c_str(),   0 },
    {0x1600, 0x7,  DTYPE_UNSIGNED32, 32, ATYPE_RW, RPdo7.c_str(),   0 },
    {0x1600, 0x8,  DTYPE_UNSIGNED32, 32, ATYPE_RW, RPdo8.c_str(),   0 },
    //
    {0x1A00, 0x0,  DTYPE_UNSIGNED8,   8, ATYPE_RW, "TxElemCnt",     0 },
    {0x1A00, 0x1,  DTYPE_UNSIGNED32, 32, ATYPE_RW, TPdo1.c_str(),   0 },
    {0x1A00, 0x2,  DTYPE_UNSIGNED32, 32, ATYPE_RW, TPdo2.c_str(),   0 },
    {0x1A00, 0x3,  DTYPE_UNSIGNED32, 32, ATYPE_RW, TPdo3.c_str(),   0 },
    {0x1A00, 0x4,  DTYPE_UNSIGNED32, 32, ATYPE_RW, TPdo4.c_str(),   0 },
    {0x1A00, 0x5,  DTYPE_UNSIGNED32, 32, ATYPE_RW, TPdo5.c_str(),   0 },
    {0x1A00, 0x6,  DTYPE_UNSIGNED32, 32, ATYPE_RW, TPdo6.c_str(),   0 },
    {0x1A00, 0x7,  DTYPE_UNSIGNED32, 32, ATYPE_RW, TPdo7.c_str(),   0 },
    {0x1A00, 0x8,  DTYPE_UNSIGNED32, 32, ATYPE_RW, TPdo8.c_str(),   0 },
    
    // 0x1A00 slave TxPDO 
    {0x6041, 0x0,  DTYPE_UNSIGNED16, 16, ATYPE_RO, _DCOMstatus.c_str(),   0 },
    {0x6061, 0x0,  DTYPE_INTEGER8,    8, ATYPE_RO, _DCOMopmd_act.c_str(), 0 },
    {0x6064, 0x0,  DTYPE_INTEGER32,  32, ATYPE_RO, _p_act.c_str(),        0 },
    {0x60F4, 0x0,  DTYPE_INTEGER32,  32, ATYPE_RO, _p_dif.c_str(),        0 },
    {0x6077, 0x0,  DTYPE_INTEGER32,  16, ATYPE_RO, _tq_act.c_str(),        0 },
    {0x603F, 0x0,  DTYPE_UNSIGNED16, 16, ATYPE_RO, _LastError.c_str(),    0 },
    {0x3008, 0x1,  DTYPE_UNSIGNED16, 16, ATYPE_RO, _IO_act.c_str(),       0 },    
    // 0x1600 slave RxPDO 
    {0x6040, 0x0,  DTYPE_UNSIGNED16, 16, ATYPE_RW, DCOMcontrol.c_str(),   0 },
    {0x6060, 0x0,  DTYPE_INTEGER8,    8, ATYPE_RW, DCOMopmode.c_str(),    0 },
    {0x607A, 0x0,  DTYPE_INTEGER32,  32, ATYPE_RW, PPp_target.c_str(),    0 },
    {0x60FF, 0x0,  DTYPE_INTEGER32,  32, ATYPE_RW, PVv_target.c_str(),    0 },
    {0x6071, 0x0,  DTYPE_INTEGER16,  16, ATYPE_RW, PTtq_target.c_str(),   0 },
    {0x3008, 0x11, DTYPE_UNSIGNED16, 16, ATYPE_RW, IO_DQ_set.c_str(),     0 },
    {0x301B, 0x9,  DTYPE_UNSIGNED16, 16, ATYPE_RW, JOGactivate.c_str(),   0 },
    
    {0x6081, 0x0,  DTYPE_UNSIGNED32, 32, ATYPE_RW, PPv_target.c_str(),   0 },
    {0x6083, 0x0,  DTYPE_UNSIGNED32, 32, ATYPE_RW, RAMP_v_acc.c_str(),   0 },
    {0x6084, 0x0,  DTYPE_UNSIGNED32, 32, ATYPE_RW, RAMP_v_dec.c_str(),   0 },
    // position scaling 
    {0x3006, 0x7,  DTYPE_INTEGER32,  32, ATYPE_RW, ScalePOSdenom.c_str(),0 },
    {0x3006, 0x8,  DTYPE_INTEGER32,  32, ATYPE_RW, ScalePOSnum.c_str(),  0 },
    
    {0, 0, 0, 0, 0, 0, 0 }
};



void LXM32iESC::init_SDOs ( void ) {

    int objd_num, i = 0;

    objd_num = sizeof ( source_SDOs ) /sizeof ( objd_t );
    SDOs = new objd_t [objd_num];

    memcpy ( ( void* ) SDOs, source_SDOs, sizeof ( source_SDOs ) );

    //
    SDOs[i++].data = ( void* ) &LXM32iESC::sdo.rxPdoMapCnt;
    SDOs[i++].data = ( void* ) &LXM32iESC::sdo.rxPdoMap;
    
    SDOs[i++].data = ( void* ) &LXM32iESC::sdo.rxElemCnt;
    for(int j=0;j<8;j++) SDOs[i++].data = ( void* ) &LXM32iESC::sdo.rxMap[j];
    
    //
    SDOs[i++].data = ( void* ) &LXM32iESC::sdo.txPdoMapCnt;
    SDOs[i++].data = ( void* ) &LXM32iESC::sdo.txPdoMap;
    
    SDOs[i++].data = ( void* ) &LXM32iESC::sdo.txElemCnt;
    for(int j=0;j<8;j++) SDOs[i++].data = ( void* ) &LXM32iESC::sdo.txMap[j];
    
    // 
    SDOs[i++].data = ( void* ) &LXM32iESC::rx_pdo._DCOMstatus;
    SDOs[i++].data = ( void* ) &LXM32iESC::rx_pdo._DCOMopmd_act;
    SDOs[i++].data = ( void* ) &LXM32iESC::rx_pdo._p_act;
    SDOs[i++].data = ( void* ) &LXM32iESC::rx_pdo._p_dif;
    SDOs[i++].data = ( void* ) &LXM32iESC::rx_pdo._tq_act;
    SDOs[i++].data = ( void* ) &LXM32iESC::rx_pdo._LastError;
    SDOs[i++].data = ( void* ) &LXM32iESC::rx_pdo._IO_act;
    // 
    SDOs[i++].data = ( void* ) &LXM32iESC::tx_pdo.DCOMcontrol;
    SDOs[i++].data = ( void* ) &LXM32iESC::tx_pdo.DCOMopmode;
    SDOs[i++].data = ( void* ) &LXM32iESC::tx_pdo.PPp_target;
    SDOs[i++].data = ( void* ) &LXM32iESC::tx_pdo.PVv_target;
    SDOs[i++].data = ( void* ) &LXM32iESC::tx_pdo.PTtq_target;
    SDOs[i++].data = ( void* ) &LXM32iESC::tx_pdo.IO_DQ_set;
    SDOs[i++].data = ( void* ) &LXM32iESC::tx_pdo.JOGactivate;
    
    SDOs[i++].data = ( void* ) &LXM32iESC::sdo.PPv_target;
    SDOs[i++].data = ( void* ) &LXM32iESC::sdo.RAMP_v_acc;
    SDOs[i++].data = ( void* ) &LXM32iESC::sdo.RAMP_v_dec;
    
    SDOs[i++].data = ( void* ) &LXM32iESC::sdo.ScalePOSdenom;
    SDOs[i++].data = ( void* ) &LXM32iESC::sdo.ScalePOSnum;
    
    SDOs[i++].data = 0;

    assert ( objd_num == i );
}
