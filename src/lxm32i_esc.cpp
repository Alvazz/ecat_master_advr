#include <iit/ecat/advr/lxm32i_esc.h>

//using namespace iit::ecat;
using namespace iit::ecat::advr;


static const iit::ecat::objd_t source_SDOs[] = {

    // --> 0x1600
    {0x1C12, 0x0,  DTYPE_UNSIGNED8,   8, ATYPE_RW, RxPdoMapCnt.c_str(),   0 },
    {0x1C12, 0x1,  DTYPE_UNSIGNED16, 16, ATYPE_RW, RxPdoMap.c_str(),      0 },
    //
    {0x1600, 0x0,  DTYPE_UNSIGNED8,   8, ATYPE_RW, RxElemCnt.c_str(),     0 },
    {0x1600, 0x1,  DTYPE_UNSIGNED32, 32, ATYPE_RW, RPdo1.c_str(),   0 },
    {0x1600, 0x2,  DTYPE_UNSIGNED32, 32, ATYPE_RW, RPdo2.c_str(),   0 },
    {0x1600, 0x3,  DTYPE_UNSIGNED32, 32, ATYPE_RW, RPdo3.c_str(),   0 },
    {0x1600, 0x4,  DTYPE_UNSIGNED32, 32, ATYPE_RW, RPdo4.c_str(),   0 },
    {0x1600, 0x5,  DTYPE_UNSIGNED32, 32, ATYPE_RW, RPdo5.c_str(),   0 },
    {0x1600, 0x6,  DTYPE_UNSIGNED32, 32, ATYPE_RW, RPdo6.c_str(),   0 },
    // --> 0x1A00
    {0x1C13, 0x0,  DTYPE_UNSIGNED8,   8, ATYPE_RW, TxPdoMapCnt.c_str(),   0 },
    {0x1C13, 0x1,  DTYPE_UNSIGNED16, 16, ATYPE_RW, TxPdoMap.c_str(),      0 },
    //
    {0x1A00, 0x0,  DTYPE_UNSIGNED8,   8, ATYPE_RW, TxElemCnt.c_str(),     0 },
    {0x1A00, 0x1,  DTYPE_UNSIGNED32, 32, ATYPE_RW, TPdo1.c_str(),   0 },
    {0x1A00, 0x2,  DTYPE_UNSIGNED32, 32, ATYPE_RW, TPdo2.c_str(),   0 },
    {0x1A00, 0x3,  DTYPE_UNSIGNED32, 32, ATYPE_RW, TPdo3.c_str(),   0 },
//    {0x1A00, 0x4,  DTYPE_UNSIGNED32, 32, ATYPE_RW, TPdo4.c_str(),   0 },
//    {0x1A00, 0x5,  DTYPE_UNSIGNED32, 32, ATYPE_RW, TPdo5.c_str(),   0 },
    // 0x1A00 slave TxPDO 
    {0x6041, 0x0,  DTYPE_UNSIGNED16, 16, ATYPE_RO, _DCOMstatus.c_str(),   0 },
    {0x6061, 0x0,  DTYPE_INTEGER8,    8, ATYPE_RO, _DCOMopmd_act.c_str(), 0 },
    {0x6064, 0x0,  DTYPE_INTEGER32,  32, ATYPE_RO, _p_act.c_str(),        0 },
//    {0x60F4, 0x0,  DTYPE_INTEGER32,  32, ATYPE_RO, _p_dif.c_str(),        0 },
//    {0x6077, 0x0,  DTYPE_INTEGER16,  16, ATYPE_RO, _tq_act .c_str(),      0 },
    // 0x1600 slave RxPDO 
    {0x6040, 0x0,  DTYPE_UNSIGNED16, 16, ATYPE_RW, DCOMcontrol.c_str(),   0 },
    {0x6060, 0x0,  DTYPE_INTEGER8,    8, ATYPE_RW, DCOMopmode.c_str(),    0 },
    {0x607A, 0x0,  DTYPE_INTEGER32,  32, ATYPE_RW, PPp_target.c_str(),    0 },
    {0x6081, 0x0,  DTYPE_UNSIGNED32, 32, ATYPE_RW, PPv_target.c_str(),    0 },
    {0x6083, 0x0,  DTYPE_UNSIGNED32, 32, ATYPE_RW, RAMP_v_acc.c_str(),    0 },
    {0x6084, 0x0,  DTYPE_UNSIGNED32, 32, ATYPE_RW, RAMP_v_dec.c_str(),    0 },
    // position scaling  startup params 
//     {0x3006, 0x3D, DTYPE_INTEGER16,  16, ATYPE_RW, CompParSyncMot.c_str(),0 }, 
//     {0x3006, 0x38, DTYPE_UNSIGNED16, 16, ATYPE_RW, MOD_enable.c_str(),0 }, 
//     {0x3006, 0x18, DTYPE_INTEGER16,  16, ATYPE_RW, LimQStopReact.c_str(),0 }, 
//     {0x3006, 0x6 , DTYPE_UNSIGNED16, 16, ATYPE_RW, IOsigRespOfPS.c_str(),0 }, 
//     {0x3012, 0x6 , DTYPE_UNSIGNED16, 16, ATYPE_RW, CTRL1_KFPp.c_str(),0 }, 
//     {0x3013, 0x6 , DTYPE_UNSIGNED16, 16, ATYPE_RW, CTRL2_KFPp.c_str(),0 },  
    {0x3006, 0x7,  DTYPE_INTEGER32,  32, ATYPE_RW, ScalePOSdenom.c_str(), 0 },
    {0x3006, 0x8,  DTYPE_INTEGER32,  32, ATYPE_RW, ScalePOSnum.c_str(),   0 },
    // homing
    {0x6098, 0x0,  DTYPE_INTEGER8,    8, ATYPE_RW, HMmethod.c_str(),      0 },
    {0x301B, 0x16, DTYPE_INTEGER32,  32, ATYPE_RW, HMp_setP.c_str(),      0 },

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
    for(int j=0;j<RX_ELEM_CNT;j++) SDOs[i++].data = ( void* ) &LXM32iESC::sdo.rxMap[j];
    //
    SDOs[i++].data = ( void* ) &LXM32iESC::sdo.txPdoMapCnt;
    SDOs[i++].data = ( void* ) &LXM32iESC::sdo.txPdoMap;
    SDOs[i++].data = ( void* ) &LXM32iESC::sdo.txElemCnt;
    for(int j=0;j<TX_ELEM_CNT;j++) SDOs[i++].data = ( void* ) &LXM32iESC::sdo.txMap[j];
    // 
    SDOs[i++].data = ( void* ) &LXM32iESC::rx_pdo._DCOMstatus;
    SDOs[i++].data = ( void* ) &LXM32iESC::rx_pdo._DCOMopmd_act;
    SDOs[i++].data = ( void* ) &LXM32iESC::rx_pdo._p_act;
//     SDOs[i++].data = ( void* ) &LXM32iESC::rx_pdo._p_dif;
//     SDOs[i++].data = ( void* ) &LXM32iESC::rx_pdo._tq_act;
    // 
    SDOs[i++].data = ( void* ) &LXM32iESC::tx_pdo.DCOMcontrol;
    SDOs[i++].data = ( void* ) &LXM32iESC::tx_pdo.DCOMopmode;
    SDOs[i++].data = ( void* ) &LXM32iESC::tx_pdo.PPp_target;
    SDOs[i++].data = ( void* ) &LXM32iESC::tx_pdo.PPv_target;
    SDOs[i++].data = ( void* ) &LXM32iESC::tx_pdo.RAMP_v_acc;
    SDOs[i++].data = ( void* ) &LXM32iESC::tx_pdo.RAMP_v_dec;
    // startup params position scaling 
//     SDOs[i++].data = ( void* ) &LXM32iESC::sdo.CompParSyncMot; 
//     SDOs[i++].data = ( void* ) &LXM32iESC::sdo.MOD_enable; 
//     SDOs[i++].data = ( void* ) &LXM32iESC::sdo.LimQStopReact; 
//     SDOs[i++].data = ( void* ) &LXM32iESC::sdo.IOsigRespOfPS; 
//     SDOs[i++].data = ( void* ) &LXM32iESC::sdo.CTRL1_KFPp; 
//     SDOs[i++].data = ( void* ) &LXM32iESC::sdo.CTRL2_KFPp; 
    SDOs[i++].data = ( void* ) &LXM32iESC::sdo.ScalePOSdenom;
    SDOs[i++].data = ( void* ) &LXM32iESC::sdo.ScalePOSnum;
    // homing
    SDOs[i++].data = ( void* ) &LXM32iESC::sdo.HMmethod;
    SDOs[i++].data = ( void* ) &LXM32iESC::sdo.HMp_setP;
    
    SDOs[i++].data = 0;

    assert ( objd_num == i );
}

static struct LXM32iEscSdoTypes yifu_pdomap = {
    // RxPdo
    1,
    0x1600,
    RX_ELEM_CNT,
    0x60400010, 0x60600008, 0x607A0020, 0x60810020, 0x60830020, 0x60840020,  
    // TxPdo
    1,
    0x1A00,
    TX_ELEM_CNT,
    0x60410010, 0x60610008, 0x60640020, // 0x60F40020, 0x60770010,
    // startup param 
    //1, 0, 6, 1, 1000, 1000,
    // denom num
    //32768, 1,
    131072, 1,
    //0x1000000, 0x200,
    // homing
    35, 0,
    //
};

/*
 * 
 * 
 */

int LXM32iESC::init ( const YAML::Node & root_cfg ) {

    std::cout << yifu_pdomap << "\n";
    
    std::string robot_name("void");
//     try { robot_name = root_cfg["ec_boards_base"]["robot_name"].as<std::string>(); }
//     catch ( YAML::Exception &e ) { DPRINTF ( "No robot name in config ... %s\n", e.what() ); }
    
    try {
        std::string motor_node_name ( "Lxm32i_"+std::to_string ( position ) );
        const auto motor_node = root_cfg[motor_node_name];
        tx_pdo.PPv_target = motor_node["PPv_target"].as<uint32_t>();
        //tx_pdo.PVv_target = motor_node["PVv_target"].as<uint32_t>();
        tx_pdo.RAMP_v_acc = motor_node["RAMP_v_acc"].as<uint32_t>();
        tx_pdo.RAMP_v_dec = motor_node["RAMP_v_dec"].as<uint32_t>();
        set_points = motor_node["set_points"].as<std::vector<int32_t>>();
        op_mode = motor_node["OpMode"].as<std::string>();
        sign_dir = motor_node["sign_dir"].as<int32_t>();
        mmXturn = motor_node["mmXturn"].as<int32_t>();
        trj_Xs = motor_node["trj_Xs"].as<std::vector<double>>();
        trj_Ys = motor_node["trj_Ys"].as<std::vector<double>>();
        assert( trj_Xs.size() == trj_Ys.size() );
        //yifu_pdomap.ScalePOSnum = mmXturn;
        
    } catch ( YAML::Exception &e ) {
        DPRINTF ( "Catch Exception in %s ... %s\n", __PRETTY_FUNCTION__, e.what() );
        return EC_BOARD_INIT_SDO_FAIL;
    }

    sp_it = set_points.begin();
                
    try {
        init_SDOs();
        init_sdo_lookup();
        //readSDO_byname ( "sensor_robot_id" );
        
        for ( auto const name : rd_pdos ) { readSDO_byname ( name ); }
        rx_pdo.dump(std::cout, "\n"); std::cout << "\n\n";
        
        for ( auto const name : rd_sdos ) { readSDO_byname ( name ); }
        std::cout << sdo << "\n";
        
        { 
            int i = 0; 
            for ( auto const name : rpdos ) { writeSDO_byname( name, yifu_pdomap.rxMap[i++] ); }  
            writeSDO_byname( RxElemCnt, yifu_pdomap.rxElemCnt ); 
        } 
        { 
            int i = 0; 
            for ( auto const name : tpdos ) { writeSDO_byname( name, yifu_pdomap.txMap[i++] ); }  
            writeSDO_byname( TxElemCnt, yifu_pdomap.txElemCnt ); 
        }
        
        writeSDO_byname( RxPdoMap,    yifu_pdomap.rxPdoMap );
        writeSDO_byname( RxPdoMapCnt, yifu_pdomap.rxPdoMapCnt );
        
        writeSDO_byname( TxPdoMap,    yifu_pdomap.txPdoMap );        
        writeSDO_byname( TxPdoMapCnt, yifu_pdomap.txPdoMapCnt );

//         writeSDO_byname( CompParSyncMot, yifu_pdomap.CompParSyncMot );
//         writeSDO_byname( MOD_enable, yifu_pdomap.MOD_enable );
//         writeSDO_byname( LimQStopReact, yifu_pdomap.LimQStopReact );
//         writeSDO_byname( IOsigRespOfPS, yifu_pdomap.IOsigRespOfPS );
//         writeSDO_byname( CTRL1_KFPp, yifu_pdomap.CTRL1_KFPp );
//         writeSDO_byname( CTRL2_KFPp, yifu_pdomap.CTRL2_KFPp );
        writeSDO_byname( ScalePOSdenom, yifu_pdomap.ScalePOSdenom );
        writeSDO_byname( ScalePOSnum, yifu_pdomap.ScalePOSnum );
        
        writeSDO_byname( HMmethod, yifu_pdomap.HMmethod );
        writeSDO_byname( HMp_setP, yifu_pdomap.HMp_setP );

        
        writeSDO_byname( PPv_target, tx_pdo.PPv_target );
        writeSDO_byname( RAMP_v_acc, tx_pdo.RAMP_v_acc );
        writeSDO_byname( RAMP_v_dec, tx_pdo.RAMP_v_dec );
        
        uint32_t tmp;
        readSDO_byname(PPv_target, tmp);
        DPRINTF("PPV_target readback %d\n", tmp);
        
        //sdo.dump(std::cout, " "); std::cout << "\n";
        
    } catch ( EscWrpError &e ) {

        DPRINTF ( "Catch Exception in %s ... %s\n", __PRETTY_FUNCTION__, e.what() );
        return EC_BOARD_INIT_SDO_FAIL;
    }

    // set filename with robot_id
    log_filename = std::string ( "/tmp/LXM32iESC_pos_"+std::to_string ( position ) +"_log.txt" );

    // we log when receive PDOs
    Log::start_log ( true );

    XDDP_pipe::init (robot_name+"@LXM32i_pos_"+std::to_string ( position ) );
        
    return EC_BOARD_OK;

}


void LXM32iESC::handle_fault ( void ) {

    if ( rx_pdo._DCOMstatus.dcom_status.b3_Fault ) {
        tx_pdo.DCOMcontrol.dcom_control.b7_FaultReset = 1;
        //DPRINTF("[%s]: reset fault 0x%X\n", __FUNCTION__, rx_pdo._LastError);
    } else {
        tx_pdo.DCOMcontrol.dcom_control.b7_FaultReset = 0;                            
    }

}

int32_t LXM32iESC::set_pos_target( float pos_mm ) {
    
    int32_t pos_mm_pulses;
    pos_mm_pulses = pos_mm *  yifu_pdomap.ScalePOSdenom / mmXturn;
    //tx_pdo.PPp_target = rx_pdo._p_act + (sign_dir * pos_mm_pulses);
    tx_pdo.PPp_target = (sign_dir * pos_mm_pulses);
    
    //tx_pdo.PPp_target = rx_pdo._p_act + (sign_dir * pos_mm);
    //tx_pdo.PPp_target = (sign_dir * pos_mm);
    
    DPRINTF("<%d>[%s]: %d = %d %d %d %f\n", position, __FUNCTION__,
            tx_pdo.PPp_target, rx_pdo._p_act, sign_dir, pos_mm_pulses, pos_mm);
            
}


int LXM32iESC::user_loop( const char &cmd ) {

    std::ostringstream oss;
    auto rx_pdo_update = 0;
    
    if ( rx_pdo._DCOMstatus.all != prev_rx_pdo._DCOMstatus.all ) {
    
        prev_rx_pdo = rx_pdo;
        //rx_pdo.dump(std::cout, "\n");
        rx_pdo_update = 1;
    }
    
    if ( rx_pdo._DCOMopmd_act == HOMING ) {

        if ( rx_pdo._DCOMstatus.dcom_status.b10_target_reached &&
             rx_pdo._DCOMstatus.dcom_status.b12_operating_mode_specific &&
             rx_pdo._DCOMstatus.dcom_status.b14_x_end &&
             rx_pdo._DCOMstatus.dcom_status.b15_ref_ok ) {
            
            std::cout << "... homing reached " << rx_pdo._p_act << "\n";
        }
    }

    if ( rx_pdo._DCOMstatus.dcom_status.b0_ReadyToSwitchOn &&
         rx_pdo._DCOMstatus.dcom_status.b1_SwitchedOn &&
         rx_pdo._DCOMstatus.dcom_status.b2_OperationEnabled &&
         rx_pdo._DCOMstatus.dcom_status.b5_QuickStop ) {
        

        if ( rx_pdo._DCOMopmd_act == PPOS ) {
            if ( rx_pdo._DCOMstatus.dcom_status.b12_operating_mode_specific ) {                
                tx_pdo.DCOMcontrol.dcom_control.b4_ = 0x0;
            }
            if ( rx_pdo_update &&
                rx_pdo._DCOMstatus.dcom_status.b10_target_reached &&
                ! rx_pdo._DCOMstatus.dcom_status.b12_operating_mode_specific &&
                rx_pdo._DCOMstatus.dcom_status.b14_x_end ) {
                
                std::cout << position << "... target reached " << rx_pdo._p_act << "\n";
                tx_pdo.DCOMcontrol.dcom_control.b4_ = 0x1;
                if ( sp_it == set_points.end() ) {
                    sp_it = set_points.begin();
                }   
                set_pos_target( *sp_it );
                sp_it ++;
            }
        }
    }
    
    if ( cmd ) {
        
        std::cout << cmd << "\n";
        rx_pdo.dump(std::cout, "\n"); std::cout << "\n\n";

        if ( rx_pdo._DCOMstatus.dcom_status.b3_Fault ) {
            tx_pdo.DCOMcontrol.dcom_control.b7_FaultReset = 1;
            DPRINTF("[%s]: reset fault\n", __FUNCTION__);
        } else {
            tx_pdo.DCOMcontrol.dcom_control.b7_FaultReset = 0;                            
        }

        // 
        switch (cmd) {
            case 'a' :
                tx_pdo.DCOMcontrol.all = 0x0;
                break;
            case 'b' :
                //tx_pdo.DCOMcontrol.all = 0x6;
                tx_pdo.DCOMcontrol.dcom_control.b0_SwitchOn = 0x0;
                tx_pdo.DCOMcontrol.dcom_control.b1_EnableVoltage = 0x1;
                tx_pdo.DCOMcontrol.dcom_control.b2_QuickStop = 0x1;
                tx_pdo.DCOMcontrol.dcom_control.b3_EnableOperation = 0x0;
                break;
            case 'c' :
                //tx_pdo.DCOMcontrol.all = 0xF;
                tx_pdo.DCOMcontrol.dcom_control.b0_SwitchOn = 0x1;
                tx_pdo.DCOMcontrol.dcom_control.b1_EnableVoltage = 0x1;
                tx_pdo.DCOMcontrol.dcom_control.b2_QuickStop = 0x1;
                tx_pdo.DCOMcontrol.dcom_control.b3_EnableOperation = 0x1;
                tx_pdo.DCOMcontrol.dcom_control.b8_Halt = 0;
                tx_pdo.DCOMopmode = PPOS;
                break;

            case 'j' :
                tx_pdo.DCOMopmode = JOG;
                break;
            case 'h' :
                // set op mode 
                tx_pdo.DCOMopmode = HOMING;
                // start homing
                tx_pdo.DCOMcontrol.dcom_control.b4_ = 0x1;
                break;
            case 'p' :
                // position
                tx_pdo.DCOMopmode = PPOS;
                tx_pdo.DCOMcontrol.dcom_control.b4_ = 0x1;
                tx_pdo.DCOMcontrol.dcom_control.b9_ChangeOnSetpoint = 0x1;
                // starts a movement to a target position
                // Target values transmitted during a movement become immediately effective and are executed at the target.
                // The movement is not stopped at the current target position
                //tx_pdo.DCOMcontrol.dcom_control.b5_ = 0x0;
                // Target values transmitted during a movement become immediately effective and are immediately executed.
                tx_pdo.DCOMcontrol.dcom_control.b5_ = 0x1;                
                // relative movement
                //tx_pdo.DCOMcontrol.dcom_control.b6_ = 0x1;
                // absolute movement
                tx_pdo.DCOMcontrol.dcom_control.b6_ = 0x0;
                // set pos using set_points
                sp_it = set_points.begin();
                set_pos_target( *sp_it );
                sp_it ++;
                // 
                //trj->start_time();
                //set_pos_target( (float)(*trj)() );
                break;
            case 'q' :
                // cyclic sync position
                tx_pdo.DCOMopmode = CSPOS;
                //trj->start_time();
                //set_pos_target( (float)(*trj)() );
                break;
            case 'n' :
                tx_pdo.DCOMcontrol.dcom_control.b4_ = 0x1;
                break;                
            case 'x' :
                tx_pdo.PPv_target -= 100;
                tx_pdo.dump(oss,"\t");
                DPRINTF ( "\ttx_pdo %s\n", oss.str().c_str() );
                break;
            case 'X' :
                tx_pdo.PPv_target += 100;
                tx_pdo.dump(oss,"\t");
                DPRINTF ( "\ttx_pdo %s\n", oss.str().c_str() );
                break;
                
            default :
                // halt
                tx_pdo.DCOMcontrol.dcom_control.b8_Halt = 1;
                break;
        }
    }
        
}


