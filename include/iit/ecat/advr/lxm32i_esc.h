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

#define DUMPWNAME(os, name, a, d) \
    do { (os) << (name) << "=" << (a) << d; } while(0)
        
#define DUMP(os, a, d) DUMPWNAME((os), #a, (a), (d))  

struct CTRL_STATUS {

    uint16_t  status_bits1:4;
    uint16_t  voltage_enabled:1;
    uint16_t  status_bits2:2;
    uint16_t  warning:1;
    uint16_t  halt_request_active:1;
    uint16_t  remote:1;
    uint16_t  target_reached:1;
    uint16_t  bit11:1;
    uint16_t  operating_mode_specific:1;
    uint16_t  x_err:1;
    uint16_t  x_end:1;
    uint16_t  ref_ok:1;
    
    std::ostream& dump ( std::ostream& os, const std::string delim ) const {
    
        DUMP( os, status_bits1, delim );
        DUMP( os, voltage_enabled, delim );
        DUMP( os, status_bits2, delim );
        DUMP( os, warning, delim );
        DUMP( os, remote, delim );
        DUMP( os, target_reached, delim );
        DUMP( os, bit11, delim );
        DUMP( os, operating_mode_specific, delim );
        DUMP( os, x_err, delim );
        DUMP( os, x_end, delim );
        DUMP( os, ref_ok, delim );
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
    struct CTRL_STATUS  ctrl_status;
} control_status_t;
    

struct LXM32iEscPdoTypes {
    // TX  slave_input -- master output
    struct pdo_tx {
        uint16_t    DCOMcontrol;
        int32_t     PPp_target;
        uint16_t    IO_DQ_set;
    }  __attribute__ ( ( __packed__ ) );

    // RX  slave_output -- master input
    struct pdo_rx {
        control_status_t    _DCOMstatus;
        int32_t             _p_act;
        uint16_t            _LastError;
        uint16_t            _IO_act;
        
        std::ostream& dump ( std::ostream& os, const std::string delim ) const {
            _DCOMstatus.ctrl_status.dump(os,",");
            os << _p_act << delim;
            os << _LastError << delim;
            os << _IO_act << delim;
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
        void to_map ( jmap_t & jpdo ) {
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


struct LXM32iEscSdoTypes {
    
    uint8_t     rxPdoMapCnt;
    uint16_t    rxPdoMap;
    
    uint8_t     rxElemCnt;
    uint32_t    rxMap[3];
    
    uint8_t     txPdoMapCnt;
    uint16_t    txPdoMap;

    uint8_t     txElemCnt;
    uint32_t    txMap[4];

};

static struct LXM32iEscSdoTypes dflt_sdo = {
    1, 0x1600,
    3, 0x60400010, 0x607A0020, 0x30081110,
    1, 0x1A00,
    4, 0x60410010, 0x60640020, 0x603F0010, 0x30080110, 
        
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
    
private:
    stat_t  s_rtt;
    objd_t * SDOs;
};

/*
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

    if ( rx_pdo._LastError ) {
        handle_fault();
    }

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
    return 1;
}

inline void LXM32iESC::print_info ( void ) {
}

inline int LXM32iESC::init ( const YAML::Node & root_cfg ) {

    sdo = dflt_sdo;
    
    std::string robot_name("void");
    try {
        robot_name = root_cfg["ec_boards_base"]["robot_name"].as<std::string>();
    } catch ( YAML::Exception &e ) {
    }

    uint8_t zero = 0;
    try {
        init_SDOs();
        init_sdo_lookup(false);
        //readSDO_byname ( "sensor_robot_id" );
        
        //writeSDO_byname( "RxElemCnt", zero );
        writeSDO_byname( "Rx1",    sdo.rxMap[0] );
        writeSDO_byname( "Rx2",    sdo.rxMap[1] );
        writeSDO_byname( "Rx3",    sdo.rxMap[2] );
        writeSDO_byname( "RxElemCnt", sdo.rxElemCnt );

        //writeSDO_byname( "TxElemCnt", zero );
        writeSDO_byname( "Tx1",    sdo.txMap[0] );
        writeSDO_byname( "Tx2",    sdo.txMap[1] );
        writeSDO_byname( "Tx3",    sdo.txMap[2] );
        writeSDO_byname( "Tx4",    sdo.txMap[3] );
        writeSDO_byname( "TxElemCnt", sdo.txElemCnt );

        //writeSDO_byname( "RxPdoMapCnt", zero );
        writeSDO_byname( "RxPdoMap",    sdo.rxPdoMap );
        writeSDO_byname( "RxPdoMapCnt", sdo.rxPdoMapCnt );
        
        //writeSDO_byname( "TxPdoMapCnt", zero );
        writeSDO_byname( "TxPdoMap",    sdo.txPdoMap );        
        writeSDO_byname( "TxPdoMapCnt", sdo.txPdoMapCnt );
        
    } catch ( EscWrpError &e ) {

        DPRINTF ( "Catch Exception in %s ... %s\n", __PRETTY_FUNCTION__, e.what() );
        return EC_BOARD_INIT_SDO_FAIL;
    }

    // set filename with robot_id
    log_filename = std::string ( "/tmp/LXM32iESC_"+std::to_string ( 1 ) +"_log.txt" );

    // we log when receive PDOs
    Log::start_log ( true );

    XDDP_pipe::init (robot_name+"@LXM32i_id_"+std::to_string ( get_robot_id() ) );
        
    return EC_BOARD_OK;

}

inline void LXM32iESC::handle_fault ( void ) {

    //DPRINTF("_oXo_ TODO _oXo_ %d\n", rx_pdo.driverErr);
}


typedef std::map<int, LXM32iESC*>  LXM32iSlavesMap;




}
}
}
#endif /* __IIT_ECAT_ADVR_SKIN_SENSOR_ESC_H__ */
