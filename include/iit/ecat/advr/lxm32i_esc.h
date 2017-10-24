/*
 *  Schneider LXM32i
 *
 *  Created on: Sept 2017
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_LXM32I_ESC_H__
#define __IIT_ECAT_ADVR_LXM32I_ESC_H__

#include <iit/ecat/slave_wrapper.h>
#include <iit/ecat/advr/esc.h>
#include <iit/ecat/advr/log_esc.h>
#include <iit/ecat/advr/pipes.h>
#include <iit/ecat/utils.h>
#include <protobuf/ecat_pdo.pb.h>

#include <map>
#include <iostream>


namespace iit {
namespace ecat {
namespace advr {

#define _MK_STR(a) static const std::string a ( #a )

//static const std::string _DCOMstatus( "_DCOMstatus");
_MK_STR(_DCOMstatus);
_MK_STR(_DCOMopmd_act);
_MK_STR(_p_act);
_MK_STR(_p_dif);
_MK_STR(_tq_act);
_MK_STR(_LastError);
_MK_STR(_IO_act);


std::vector<std::string> const rd_pdos = { _DCOMstatus, _DCOMopmd_act, _p_act, _p_dif, _tq_act, _LastError, _IO_act };
std::vector<std::string> const rd_sdos = { };

_MK_STR(DCOMcontrol);
_MK_STR(DCOMopmode);
_MK_STR(PPp_target);
_MK_STR(PVv_target);
_MK_STR(PTtq_target);
_MK_STR(IO_DQ_set);
_MK_STR(JOGactivate);

_MK_STR(PPv_target);
_MK_STR(RAMP_v_acc);
_MK_STR(RAMP_v_dec);
_MK_STR(ScalePOSdenom);
_MK_STR(ScalePOSnum);

_MK_STR(HMmethod);
_MK_STR(HMp_setP);


std::vector<std::string> const wr_pdos = { DCOMcontrol, DCOMopmode, PPp_target, PVv_target, PTtq_target, IO_DQ_set, JOGactivate };
std::vector<std::string> const wr_sdos = { PPv_target, RAMP_v_acc, RAMP_v_dec, HMmethod, HMp_setP};

_MK_STR(RxPdoMapCnt);
_MK_STR(RxPdoMap);
_MK_STR(RPdo1);
_MK_STR(RPdo2);
_MK_STR(RPdo3);
_MK_STR(RPdo4);
_MK_STR(RPdo5);
_MK_STR(RPdo6);
_MK_STR(RPdo7);
_MK_STR(RPdo8);
std::vector<std::string> const rpdos = { RPdo1, RPdo2, RPdo3, RPdo4, RPdo5, RPdo6, RPdo7, RPdo8 };


_MK_STR(TxPdoMapCnt);
_MK_STR(TxPdoMap);
_MK_STR(TPdo1);
_MK_STR(TPdo2);
_MK_STR(TPdo3);
_MK_STR(TPdo4);
_MK_STR(TPdo5);
_MK_STR(TPdo6);
_MK_STR(TPdo7);
_MK_STR(TPdo8);
std::vector<std::string> const tpdos = { TPdo1, TPdo2, TPdo3, TPdo4, TPdo5, TPdo6, TPdo7, TPdo8 };


std::vector<std::string> const pdo_map = { RxPdoMapCnt, RxPdoMap, TxPdoMapCnt, TxPdoMap };


#define DUMPWNAME(os, name, a, d) \
    do { (os) << (name) << "=" << (a) << d; } while(0)
        
#define DUMP(os, a, d) DUMPWNAME((os), #a, (a), (d))  

        
// Bits 0, 1, 2, 3, and 5, 6 of the DCOMstatus parameter provide information on the operating state.
        
struct DCOM_STATUS {

    uint16_t  b0_ReadyToSwitchOn:1;
    uint16_t  b1_SwitchedOn:1;
    uint16_t  b2_OperationEnabled:1;
    uint16_t  b3_Fault:1;
    uint16_t  b4_VoltageEnabled:1;
    uint16_t  b5_QuickStop:1;
    uint16_t  b6_SwtchedOnDisabled:1;
    uint16_t  b7_warning:1;
    uint16_t  b8_halt_request_active:1;
    uint16_t  b9_remote:1;
    uint16_t  b10_target_reached:1;
    uint16_t  b11:1;
    uint16_t  b12_operating_mode_specific:1;
    uint16_t  b13_x_err:1;
    uint16_t  b14_x_end:1;
    uint16_t  b15_ref_ok:1;
    
    std::ostream& dump ( std::ostream& os, const std::string delim ) const {
    
        DUMP( os, b0_ReadyToSwitchOn, delim );
        DUMP( os, b1_SwitchedOn, delim );
        DUMP( os, b2_OperationEnabled, delim );
        DUMP( os, b3_Fault, delim );
        DUMP( os, b4_VoltageEnabled, delim );
        DUMP( os, b5_QuickStop, delim );
        DUMP( os, b6_SwtchedOnDisabled, delim );
        DUMP( os, b7_warning, delim );
        DUMP( os, b8_halt_request_active, delim );
        DUMP( os, b9_remote, delim );
        DUMP( os, b10_target_reached, delim );
        DUMP( os, b11, delim );
        DUMP( os, b12_operating_mode_specific, delim );
        DUMP( os, b13_x_err, delim );
        DUMP( os, b14_x_end, delim );
        DUMP( os, b15_ref_ok, delim );
        return os;
        
    }
    
    void fprint ( FILE *fp ) {
        std::ostringstream oss;
        dump(oss,"\t");
        fprintf ( fp, "%s", oss.str().c_str() );
    }
    int sprint ( char *buff, size_t size ) {
        std::ostringstream oss;
        dump(oss,"\t");
        return snprintf ( buff, size, "%s", oss.str().c_str() );
    }
       
};

// Bits 0, 1, 2, 3 and 7 of the parameter DCOMcontrol allow you to switch between the operating states.

struct DCOM_CONTROL {

    uint16_t  b0_SwitchOn:1;
    uint16_t  b1_EnableVoltage:1;
    uint16_t  b2_QuickStop:1;
    uint16_t  b3_EnableOperation:1;
    uint16_t  b4_:1;
    uint16_t  b5_:1;
    uint16_t  b6_:1;
    uint16_t  b7_FaultReset:1;
    uint16_t  b8_Halt:1;
    uint16_t  b9_ChangeOnSetpoint:1;
    uint16_t  _reserved:6;
    
    std::ostream& dump ( std::ostream& os, const std::string delim ) const {
    
        DUMP( os, b0_SwitchOn, delim );
        DUMP( os, b1_EnableVoltage, delim );
        DUMP( os, b2_QuickStop, delim );
        DUMP( os, b3_EnableOperation, delim );
        DUMP( os, b7_FaultReset, delim );
        DUMP( os, b8_Halt, delim );
        DUMP( os, b9_ChangeOnSetpoint, delim );
        return os;
        
    }    
    void fprint ( FILE *fp ) {
        std::ostringstream oss;
        dump(oss,"\t");
        fprintf ( fp, "%s", oss.str().c_str() );
    }
    int sprint ( char *buff, size_t size ) {
        std::ostringstream oss;
        dump(oss,"\t");
        return snprintf ( buff, size, "%s", oss.str().c_str() );
    }
};


typedef union{
    uint16_t            all;
    struct DCOM_CONTROL  dcom_control;
} dcom_control_t;

typedef union{
    uint16_t            all;
    struct DCOM_STATUS  dcom_status;
} dcom_status_t;


class LXM32iEscPdoTypes {
   
public:
    
    // TX  slave_input -- master output
    struct pdo_tx {
        //uint8_t _pdo_tx[64];
        dcom_control_t  DCOMcontrol;
        uint8_t         DCOMopmode;
        int32_t         PPp_target;
        int32_t         PVv_target;
        int16_t         PTtq_target;
        uint16_t        IO_DQ_set;
        uint16_t        JOGactivate;
        
    }  __attribute__ ( ( __packed__ ) );

    // RX  slave_output -- master input
    struct pdo_rx {
        //uint8_t _pdo_rx[64];
        dcom_status_t   _DCOMstatus;
        uint8_t         _DCOMopmd_act;
        int32_t         _p_act;
        int32_t         _p_dif;
        int32_t         _tq_act;
        uint16_t        _LastError;
        uint16_t        _IO_act;
        
    std::ostream& dump ( std::ostream& os, const std::string delim ) const {
        _DCOMstatus.dcom_status.dump(os,delim);
        os << "0x" << std::hex << _DCOMstatus.all << std::dec << delim;
        os << "_DCOMopmd_act=" << (int)_DCOMopmd_act << std::dec << delim;
        //DUMP( os, _DCOMopmd_act, delim );
        DUMP( os, _p_act, delim );
        DUMP( os, _p_dif, delim );
        DUMP( os, _tq_act, delim );
        DUMP( os, _LastError, delim );
        DUMP( os, _IO_act, delim );
        //os << std::endl;
        return os;
    }
    
    void fprint ( FILE *fp ) {
        std::ostringstream oss;
        dump(oss,"\t");
        fprintf ( fp, "%s", oss.str().c_str() );
    }
    int sprint ( char *buff, size_t size ) {
        std::ostringstream oss;
        dump(oss,"\t");
        return snprintf ( buff, size, "%s", oss.str().c_str() );
    }
    void pb_toString( std::string * pb_str ) {
        // !!! NO static declaration
        iit::advr::Ec_slave_pdo pb_rx_pdo;
        static struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        // Header
        pb_rx_pdo.mutable_header()->mutable_stamp()->set_sec(ts.tv_sec);
        pb_rx_pdo.mutable_header()->mutable_stamp()->set_nsec(ts.tv_nsec);
        // Type
        pb_rx_pdo.set_type(iit::advr::Ec_slave_pdo::RX_SKIN_SENS);
        //
        pb_rx_pdo.SerializeToString(pb_str);
    }

    }  __attribute__ ( ( __packed__ ) );
};

typedef enum : uint16_t {
    STOP = 0,
    POS_SLOW = 1,
    NEG_SLOW = 2,
    POS_FAST = 5,
    NEG_FAST = 6,
} jogact_t;

typedef enum : int8_t {
    AUTO = -6,
    JOG = -1,
    PPOS = 1,
    PVEL = 3,
    PTOR = 4,
    HOMING = 6,
    IPOS = 7,
    CSPOS = 8,
    CSVEL = 9,
    CSTOR = 10,
} opmode_t;

struct LXM32iEscSdoTypes {
    
    uint8_t     rxPdoMapCnt;
    uint16_t    rxPdoMap;
    
    uint8_t     rxElemCnt;
    uint32_t    rxMap[8];
    
    uint8_t     txPdoMapCnt;
    uint16_t    txPdoMap;

    uint8_t     txElemCnt;
    uint32_t    txMap[8];

#if 0
    /////////////////////////////////////////////
    // TX  slave_input -- master output
    dcom_control_t  DCOMcontrol;
    uint16_t        JOGactivate;
    uint8_t         DCOMopmode;
    int32_t         PPp_target;
    uint16_t        IO_DQ_set;
    // RX  slave_output -- master input
    dcom_status_t   _DCOMstatus;
    uint8_t         _DCOMopmd_act;
    int32_t         _p_act;
    uint16_t        _LastError;
    uint16_t        _IO_act;
#endif
    
    uint32_t    PPv_target;
    uint32_t    RAMP_v_acc;
    uint32_t    RAMP_v_dec;
        
    int32_t     ScalePOSdenom;
    int32_t     ScalePOSnum;
    
    int8_t      HMmethod;
    int32_t     HMp_setP;
    
   std::ostream& dump ( std::ostream& os, const std::string delim ) const {
       os << "struct LXM32iEscSdoTypes : ";
       os << (int)rxPdoMapCnt << delim;
       os << "0x" << std::hex << rxPdoMap << std::dec << delim;
       os << (int)txPdoMapCnt << delim;
       os << "0x" << std::hex << txPdoMap << std::dec << delim;
       os << PPv_target << delim;
       os << RAMP_v_acc << delim;
       os << RAMP_v_dec << delim;
       os << HMmethod << delim;
       os << HMmethod << delim;
    }

};


// PDO is suitable for the operating mode JOG 
static struct LXM32iEscSdoTypes yifu_pdomap = {
    // RxPdo
    1,
    0x1600,
    //6,
    //{ 0x60400010, 0x60600008, 0x607A0020, 0x60FF0020, 0x60710010, 0x30081110, 0, 0 },
    7,
    { 0x60400010, 0x60600008, 0x607A0020, 0x60FF0020, 0x60710010, 0x30081110, 0x301B0910, 0 }, 
    // TxPdo
    1,
    0x1A00,
    7,
    { 0x60410010, 0x60610008, 0x60640020, 0x60F40020, 0x60770010, 0x603F0010, 0x30080110, 0 },
    //
    2500, 50000, 50000,
    // denom num
    32768, 1,
    // homing
    35, 0,
};


struct LXM32iLogTypes {

    uint64_t                    ts;     // ns
    LXM32iEscPdoTypes::pdo_rx   rx_pdo;

    void fprint ( FILE *fp ) {
        fprintf ( fp, "%lu\t", ts );
        rx_pdo.fprint ( fp );
    }
    int sprint ( char *buff, size_t size ) {
        int l = snprintf ( buff, size, "%lu\t", ts );
        return l + rx_pdo.sprint ( buff+l,size-l );
    }
};


template <typename T>
void fprint ( T t, FILE *fp ) {
    std::ostringstream oss;
    t.dump(oss,"\t");
    fprintf ( fp, "%s", oss.str().c_str() );
}
template <typename T>
int sprint ( T t, char *buff, size_t size ) {
    std::ostringstream oss;
    t.dump(oss,"\t");
    return snprintf ( buff, size, "%s", oss.str().c_str() );
}

/*
 *
 * 
 * 
 */

class LXM32iESC :
    public BasicEscWrapper<LXM32iEscPdoTypes, LXM32iEscSdoTypes>,
    public PDO_log<LXM32iLogTypes>,
    public XDDP_pipe
{
public:
    typedef BasicEscWrapper<LXM32iEscPdoTypes,LXM32iEscSdoTypes>    Base;

    typedef PDO_log<LXM32iLogTypes>                                 Log;

public:
    LXM32iESC ( const ec_slavet& slave_descriptor ) :
        Base ( slave_descriptor ),
        Log ( std::string ( "/tmp/LXM32iESC_pos"+std::to_string ( position ) +"_log.txt" ),DEFAULT_LOG_SIZE ),
        XDDP_pipe ()
    { }

    virtual ~LXM32iESC ( void ) {
        delete [] SDOs;
        DPRINTF ( "~%s pos %d\n", typeid ( this ).name(), position );
        print_stat ( s_rtt );
    }

    ///////////////////////////////////////////////////////////////////////////
    // Overrides functions from iit::ecat::BasicEscWrapper
    virtual void on_readPDO ( void );
    virtual void on_writePDO ( void );
    virtual const objd_t * get_SDOs( void );
    virtual uint32_t get_ESC_type( void );
    virtual void init_SDOs ( void );
    
    int16_t get_robot_id( void );
    void print_info ( void );
    int init ( const YAML::Node & root_cfg );
    void handle_fault ( void );

    template<class C>
    int user_loop( C &c );

    int32_t set_pos_target( int32_t pos );
    
private:
    stat_t      s_rtt;
    objd_t *    SDOs;
  
    pdo_rx_t    prev_rx_pdo;
    std::vector<int32_t> set_points; 
    std::vector<int32_t>::iterator sp_it;
    int32_t     sign_dir;
    int32_t     mmXturn;
    std::string op_mode;
};

/*
 *
 * 
 */

inline const objd_t * LXM32iESC::get_SDOs() {
    return SDOs;
}
inline uint32_t LXM32iESC::get_ESC_type() {
    return LXM32I;
}
inline void LXM32iESC::on_writePDO ( void ) {
}
inline void LXM32iESC::on_readPDO ( void ) {

//     if ( rx_pdo._DCOMstatus.dcom_status.b3_Fault ) {
//         handle_fault();
//     }

    if ( _start_log ) {
        Log::log_t log;
        log.ts = get_time_ns() - _start_log_ts ;
        log.rx_pdo  = rx_pdo;
        push_back ( log );
    }

    xddp_write ( rx_pdo );

}

inline int16_t LXM32iESC::get_robot_id() {
    //return sdo.sensor_robot_id;
    return position;
}

inline void LXM32iESC::print_info ( void ) {
}

inline int LXM32iESC::init ( const YAML::Node & root_cfg ) {

    
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


inline void LXM32iESC::handle_fault ( void ) {

    if ( rx_pdo._DCOMstatus.dcom_status.b3_Fault ) {
        tx_pdo.DCOMcontrol.dcom_control.b7_FaultReset = 1;
        DPRINTF("[%s]: reset fault 0x%X\n", __FUNCTION__, rx_pdo._LastError);
    } else {
        tx_pdo.DCOMcontrol.dcom_control.b7_FaultReset = 0;                            
    }

}

inline int32_t LXM32iESC::set_pos_target( int32_t pos ) {
    
    int32_t pos_mm_pulses = pos * 32768 / mmXturn;
    tx_pdo.PPp_target = rx_pdo._p_act + (sign_dir * pos_mm_pulses);
    DPRINTF("<%d>[%s]: %d = %d %d %d %d\n", position, __FUNCTION__, tx_pdo.PPp_target, rx_pdo._p_act, sign_dir, pos_mm_pulses, pos);
            
}


template<class C>
inline int LXM32iESC::user_loop( C &cmd ) {

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

    if ( rx_pdo._DCOMopmd_act == PPOS ) {

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
    
    if ( cmd ) {
        
        std::cout << cmd << "\n";
        //rx_pdo.dump(std::cout, "\n"); std::cout << "\n\n";

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
                //tx_pdo.DCOMcontrol.dcom_control.b9_ChangeOnSetpoint = 0x1;
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
                set_pos_target( *sp_it );
                sp_it ++;
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
                break;
        }
    }
        
}


typedef std::map<int, LXM32iESC*>  LXM32iSlavesMap;




}
}
}
#endif /* __IIT_ECAT_ADVR_SKIN_SENSOR_ESC_H__ */
