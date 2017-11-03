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
    {0x6077, 0x0,  DTYPE_INTEGER32,  16, ATYPE_RO, _tq_act.c_str(),       0 },
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
    // homing
    {0x6098, 0x0,  DTYPE_INTEGER8,    8, ATYPE_RW, HMmethod.c_str(),     0 },
    {0x301B, 0x16, DTYPE_INTEGER32,  32, ATYPE_RW, HMp_setP.c_str(),     0 },
    
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
    // position scaling 
    SDOs[i++].data = ( void* ) &LXM32iESC::sdo.ScalePOSdenom;
    SDOs[i++].data = ( void* ) &LXM32iESC::sdo.ScalePOSnum;
    // homing
    SDOs[i++].data = ( void* ) &LXM32iESC::sdo.HMmethod;
    SDOs[i++].data = ( void* ) &LXM32iESC::sdo.HMp_setP;
    
    SDOs[i++].data = 0;

    assert ( objd_num == i );
}


/*
 * 
 * 
 */

int LXM32iESC::init ( const YAML::Node & root_cfg ) {

    
    std::string robot_name("void");
    try {
        robot_name = root_cfg["ec_boards_base"]["robot_name"].as<std::string>();
    } catch ( YAML::Exception &e ) {
        DPRINTF ( "No robot name in config ... %s\n", e.what() );
    }
    try {
        std::string motor_node_name ( "Lxm32i_"+std::to_string ( position ) );
        const auto motor_node = root_cfg[motor_node_name];
        yifu_pdomap.PPv_target = motor_node["PPv_target"].as<uint32_t>();
        yifu_pdomap.RAMP_v_acc = motor_node["RAMP_v_acc"].as<uint32_t>();
        yifu_pdomap.RAMP_v_dec = motor_node["RAMP_v_dec"].as<uint32_t>();
        set_points = motor_node["set_points"].as<std::vector<int32_t>>();
        op_mode = motor_node["OpMode"].as<std::string>();
        sign_dir = motor_node["sign_dir"].as<int32_t>();
        mmXturn = motor_node["mmXturn"].as<int32_t>();
        //yifu_pdomap.ScalePOSnum = mmXturn;
        
    } catch ( YAML::Exception &e ) {
        DPRINTF ( "Catch Exception in %s ... %s\n", __PRETTY_FUNCTION__, e.what() );
        return EC_BOARD_INIT_SDO_FAIL;
    }

    sp_it = set_points.begin();
                
    try {
        init_SDOs();
        init_sdo_lookup(false);
        //readSDO_byname ( "sensor_robot_id" );
        
        for ( auto const name : rd_pdos ) { readSDO_byname ( name ); }
        rx_pdo.dump(std::cout, "\n"); std::cout << "\n\n";
        
        for ( auto const name : rd_sdos ) { readSDO_byname ( name ); }
        for ( auto const name : pdo_map ) { readSDO_byname ( name ); }
        sdo.dump(std::cout, " "); std::cout << "\n";
        
        {
            int i = 0;
            for ( auto const name : rpdos ) { writeSDO_byname( name, yifu_pdomap.rxMap[i++] ); } 
            writeSDO_byname( "RxElemCnt", yifu_pdomap.rxElemCnt );
        }
        {
            int i = 0;
            for ( auto const name : tpdos ) { writeSDO_byname( name, yifu_pdomap.txMap[i++] ); } 
            writeSDO_byname( "TxElemCnt", yifu_pdomap.txElemCnt );
        }
        
        writeSDO_byname( "RxPdoMap",    yifu_pdomap.rxPdoMap );
        writeSDO_byname( "RxPdoMapCnt", yifu_pdomap.rxPdoMapCnt );
        
        writeSDO_byname( "TxPdoMap",    yifu_pdomap.txPdoMap );        
        writeSDO_byname( "TxPdoMapCnt", yifu_pdomap.txPdoMapCnt );

        writeSDO_byname( PPv_target, yifu_pdomap.PPv_target );
        writeSDO_byname( RAMP_v_acc, yifu_pdomap.RAMP_v_acc );
        writeSDO_byname( RAMP_v_dec, yifu_pdomap.RAMP_v_dec );
        
        writeSDO_byname( ScalePOSdenom, yifu_pdomap.ScalePOSdenom );
        writeSDO_byname( ScalePOSnum, yifu_pdomap.ScalePOSnum );
        
        writeSDO_byname( HMmethod, yifu_pdomap.HMmethod );
        writeSDO_byname( HMp_setP, yifu_pdomap.HMp_setP );
        
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
        DPRINTF("[%s]: reset fault 0x%X\n", __FUNCTION__, rx_pdo._LastError);
    } else {
        tx_pdo.DCOMcontrol.dcom_control.b7_FaultReset = 0;                            
    }

}

int32_t LXM32iESC::set_pos_target( int32_t pos_mm ) {
    
    int32_t pos_mm_pulses = pos_mm * 32768 / mmXturn;
    tx_pdo.PPp_target = rx_pdo._p_act + (sign_dir * pos_mm_pulses);
    DPRINTF("<%d>[%s]: %d = %d %d %d %d\n", position, __FUNCTION__, tx_pdo.PPp_target, rx_pdo._p_act, sign_dir, pos_mm_pulses, pos_mm);
            
}


int LXM32iESC::user_loop( const char &cmd ) {

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
                // b12 == 1 new target pos accepted
                tx_pdo.DCOMcontrol.dcom_control.b4_ = 0x0;
            } else {
                // b12 == 0 new position possible
                tx_pdo.DCOMcontrol.dcom_control.b4_ = 0x1;
                set_pos_target( (float)(*trj)() );
            }
        }

        if ( 0 && rx_pdo._DCOMopmd_act == PPOS ) {
            if ( rx_pdo._DCOMstatus.dcom_status.b12_operating_mode_specific ) {                
                tx_pdo.DCOMcontrol.dcom_control.b4_ = 0x0;
            }
            if ( rx_pdo_update &&
                rx_pdo._DCOMstatus.dcom_status.b10_target_reached &&
                ! rx_pdo._DCOMstatus.dcom_status.b12_operating_mode_specific &&
                rx_pdo._DCOMstatus.dcom_status.b14_x_end ) {
                
                std::cout << "... target reached " << rx_pdo._p_act << "\n";
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
            DPRINTF("[%s]: reset fault 0x%X\n", __FUNCTION__, rx_pdo._LastError);
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
                tx_pdo.DCOMopmode = JOG;
                tx_pdo.JOGactivate = STOP;
                break;

            case 'j' :
                tx_pdo.DCOMopmode = JOG;
                tx_pdo.JOGactivate = POS_SLOW;
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
                // set pos
                //set_pos_target( *sp_it );
                //sp_it ++;
                //
                trj->start_time();
                set_pos_target( (float)(*trj)() );
                break;
            case 'q' :
                // cyclic sync position
                tx_pdo.DCOMopmode = CSPOS;
                trj->start_time();
                set_pos_target( (float)(*trj)() );
                break;
            case 'n' :
                tx_pdo.DCOMcontrol.dcom_control.b4_ = 0x1;
                break;                
            case 'v' :
                // velocity
                tx_pdo.DCOMopmode = PVEL;
                tx_pdo.PVv_target = 50;
                break;
            case 't' :
                // torque
                tx_pdo.DCOMopmode = PTOR;
                tx_pdo.PTtq_target = 30;
                break;
            case 'x' :
                if ( tx_pdo.JOGactivate == POS_SLOW ) {
                    tx_pdo.JOGactivate = NEG_SLOW;                    
                } else {
                    tx_pdo.JOGactivate = POS_SLOW;
                }
                tx_pdo.PVv_target *= -1;
                tx_pdo.PTtq_target *= -1;
                break;
            case 'X' :
                tx_pdo.PVv_target += 10;
                tx_pdo.PTtq_target += 10;
                break;
                
            default :
                // halt
                tx_pdo.DCOMcontrol.dcom_control.b8_Halt = 1;
                break;
        }
    }
        
}


