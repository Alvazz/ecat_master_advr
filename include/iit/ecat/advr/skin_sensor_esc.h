/*
 * skin_send_esc.h
 *
 *  based on TI TM4C123x - Tiva Microcontroller MCUs
 *  
 *  /http://www.ti.com/product/tm4c123ah6pm
 *
 *  Created on: Sept 2017
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_SKIN_SENSOR_ESC_H__
#define __IIT_ECAT_ADVR_SKIN_SENSOR_ESC_H__

#include <iit/ecat/slave_wrapper.h>
#include <iit/ecat/advr/esc.h>
#include <iit/ecat/advr/log_esc.h>
#include <iit/ecat/advr/pipes.h>
#include <iit/ecat/utils.h>
#include <protobuf/ecat_pdo.pb.h>

#include <map>
#include <iostream>

//#define SENSOR_NUMBER 10*5
#define SKIN_SENSOR_NUMBER  8*3

namespace iit {
namespace ecat {
namespace advr {


struct SkinSensorEscPdoTypes {
    // TX  slave_input -- master output
    struct pdo_tx {
        uint16_t    ts;
    }  __attribute__ ( ( __packed__ ) );

    // RX  slave_output -- master input
    struct pdo_rx {
        uint8_t    forceXY[SKIN_SENSOR_NUMBER];
        uint16_t   fault;
        uint16_t   rtt;                
        
        std::ostream& dump ( std::ostream& os, const std::string delim ) const {
            for (int i=0; i<SKIN_SENSOR_NUMBER; i++) {
                os << (unsigned)forceXY[i] << delim;
            }
            os << std::hex << fault << std::dec << delim;
            os << rtt << delim;
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
            for(int i = 0; i < SKIN_SENSOR_NUMBER; i++) {
                jpdo["forceXY_"+std::to_string(i)] = std::to_string( forceXY[i] );
            }
            JPDO ( fault );
            JPDO ( rtt );
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
            //pb_rx_pdo.set_type(iit::advr::Ec_slave_pdo::RX_SKIN_SENS);
            pb_rx_pdo.SerializeToString(pb_str);
        }

    }  __attribute__ ( ( __packed__ ) );
};


struct SkinSensorEscSdoTypes {

    // flash

    unsigned long Block_control;
    long NumAvSamples;
    
    int16_t ConfigFlags;

    int16_t sensor_number;
    int16_t sensor_robot_id;

    // ram

    char        firmware_version[8];
    uint16_t    ack_board_fault;
    uint16_t    flash_params_cmd;
    uint16_t    flash_params_cmd_ack;
};

struct SkinLogTypes {

    uint64_t                        ts;     // ns
    SkinSensorEscPdoTypes::pdo_rx   rx_pdo;

    void fprint ( FILE *fp ) {
        fprintf ( fp, "%lu\t", ts );
        rx_pdo.fprint ( fp );
    }
    int sprint ( char *buff, size_t size ) {
        int l = snprintf ( buff, size, "%lu\t", ts );
        return l + rx_pdo.sprint ( buff+l,size-l );
    }
};

/**
*
**/

class SkinSensorESC :
    public BasicEscWrapper<SkinSensorEscPdoTypes, SkinSensorEscSdoTypes>,
    public PDO_log<SkinLogTypes>,
    public XDDP_pipe
{
public:
    typedef BasicEscWrapper<SkinSensorEscPdoTypes,SkinSensorEscSdoTypes>    Base;
    typedef PDO_log<SkinLogTypes>                                           Log;

public:
    SkinSensorESC ( const ec_slavet& slave_descriptor ) :
        Base ( slave_descriptor ),
        Log ( std::string ( "/tmp/SkinSensorESC_pos"+std::to_string ( position ) +"_log.txt" ),DEFAULT_LOG_SIZE ),
        XDDP_pipe ()
    { }

    virtual ~SkinSensorESC ( void ) {
        delete [] SDOs;
        DPRINTF ( "~%s pos %d\n", typeid ( this ).name(), position );
        print_stat ( s_rtt );
    }

    ///////////////////////////////////////////////////////////////////////////
    // Overrides functions from iit::ecat::BasicEscWrapper
    virtual void on_readPDO ( void );
    virtual void on_writePDO ( void );
    virtual const objd_t * get_SDOs( void );
    virtual uint16_t get_ESC_type( void );
    virtual void init_SDOs ( void );
    
    int16_t get_robot_id( void );
    void print_info ( void );
    int init ( const YAML::Node & root_cfg );
    void handle_fault ( void );
    
private:
    stat_t  s_rtt;
    objd_t * SDOs;
};



inline const objd_t * SkinSensorESC::get_SDOs() {
    return SDOs;
}
inline uint16_t SkinSensorESC::get_ESC_type() {
    return SKIN_SENSOR;
}
inline void SkinSensorESC::on_writePDO ( void ) {

    tx_pdo.ts = ( uint16_t ) ( get_time_ns() /1000 );
}
inline void SkinSensorESC::on_readPDO ( void ) {

    if ( rx_pdo.rtt ) {
        rx_pdo.rtt = ( uint16_t ) ( get_time_ns() /1000 ) - rx_pdo.rtt;
        s_rtt ( rx_pdo.rtt );
    }

    if ( rx_pdo.fault ) {
        handle_fault();
    } else {
        // clean any previuos fault ack !!
        //tx_pdo.fault_ack = 0;
    }

    if ( _start_log ) {
        Log::log_t log;
        log.ts = get_time_ns() - _start_log_ts ;
        log.rx_pdo  = rx_pdo;
        push_back ( log );
    }

    xddp_write ( rx_pdo );

}

inline int16_t SkinSensorESC::get_robot_id() {
    //assert(sdo.Joint_robot_id != -1);
    return sdo.sensor_robot_id;
}

inline void SkinSensorESC::print_info ( void ) {
    DPRINTF ( "\tSensor id %d\tSensor robot id %d\n", sdo.sensor_number, sdo.sensor_robot_id );
    DPRINTF ( "\tfw_ver %s\n", sdo.firmware_version );
}

inline int SkinSensorESC::init ( const YAML::Node & root_cfg ) {

    try {
        init_SDOs();
        init_sdo_lookup(true);
        readSDO_byname ( "sensor_robot_id" );
//             set_flash_cmd_X ( this, CTRL_REMOVE_TORQUE_OFFS );

    } catch ( EscWrpError &e ) {

        DPRINTF ( "Catch Exception in %s ... %s\n", __PRETTY_FUNCTION__, e.what() );
        return EC_BOARD_INIT_SDO_FAIL;
    }

    // set filename with robot_id
    log_filename = std::string ( "/tmp/SkinSensorESC_"+std::to_string ( sdo.sensor_robot_id ) +"_log.txt" );

    // we log when receive PDOs
    Log::start_log ( true );

    XDDP_pipe::init ( "Skin_id_"+std::to_string ( get_robot_id() ) );
        
    return EC_BOARD_OK;

}

inline void SkinSensorESC::handle_fault ( void ) {

    fault_t fault;
    fault.all = rx_pdo.fault;
    //fault.bit.
    //ack_faults_X(this, fault.all);

}


typedef std::map<int, SkinSensorESC*>  SkinSensorSlavesMap;




}
}
}
#endif /* __IIT_ECAT_ADVR_SKIN_SENSOR_ESC_H__ */
