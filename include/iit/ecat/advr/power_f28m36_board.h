/*
 *
 *  Created on: May, 2017
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_POW_F28M36_ESC_H__
#define __IIT_ECAT_ADVR_POW_F28M36_ESC_H__

#include <iit/ecat/advr/esc.h>
#include <iit/ecat/advr/log_esc.h>
#include <iit/ecat/advr/pipes.h>
#include <protobuf/ecat_pdo.pb.h>

#include <map>
#include <string>


namespace iit {
namespace ecat {
namespace advr {

namespace pow_f28m36 {
struct status_bits {
    uint16_t  m3_fan_1_status:1;
    uint16_t  m3_fan_2_status:1;
    uint16_t  m3_spare_1:1;
    uint16_t  m3_spare_2:1;

    uint16_t  c28_key_status:1;
    uint16_t  c28_emr_sw_status:1;
    uint16_t  c28_pow_12V_status:1;
    uint16_t  c28_pow_19V_status:1;
    uint16_t  c28_pow_24V_status:1;
    uint16_t  c28_prech_rel_status:1;
    uint16_t  c28_main_rel_status:1;
    uint16_t  c28_spare_1:1;
    uint16_t  c28_spare_2:1;
    uint16_t  c28_spare_3:1;
    uint16_t  c28_spare_4:1;
    uint16_t  c28_spare_5:1;};

typedef union {
    uint16_t all;
    struct status_bits bit;
} status_t;
}

struct PowF28M36EscPdoTypes {
    // TX  slave_input -- master output
    struct pdo_tx {
        uint16_t    master_command;
        uint16_t    fault_ack;
        uint16_t    ts;
        
    } __attribute__ ( ( __packed__ ) );

    // RX  slave_output -- master input
    struct pdo_rx {
        uint16_t    v_batt;
        uint16_t    v_load;
        int16_t     i_load;
        uint16_t    temp_pcb;
        uint16_t    temp_heatsink;
        uint16_t    temp_batt;
        pow_f28m36::status_t    status;
        uint16_t    fault;
        uint16_t    rtt;                // us
        int sprint ( char *buff, size_t size ) {
            return snprintf ( buff, size, "0x%02X\t%d\t%d\t%d\t%d", status.all,temp_pcb,temp_heatsink,temp_batt,rtt );
        }
        void fprint ( FILE *fp ) {
            fprintf ( fp, "0x%02X\t%d\t%d\t%d\t%d", status.all,temp_pcb,temp_heatsink,temp_batt,rtt );
        }
        void to_map ( jmap_t & jpdo ) {
            JPDO ( status.all );
            JPDO ( rtt );
        }
        void pb_toString( std::string * pb_str ) {
            static iit::advr::Ec_slave_pdo pb_rx_pdo;
            static struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            // Header
            pb_rx_pdo.mutable_header()->mutable_stamp()->set_sec(ts.tv_sec);
            pb_rx_pdo.mutable_header()->mutable_stamp()->set_nsec(ts.tv_nsec);
            // Type
            pb_rx_pdo.set_type(iit::advr::Ec_slave_pdo::RX_POW_WLK);
            // PowWalkman_tx_pdo
            pb_rx_pdo.mutable_powwalkman_rx_pdo()->set_status(status.all);
            pb_rx_pdo.mutable_powwalkman_rx_pdo()->set_fault(fault);
            pb_rx_pdo.mutable_powwalkman_rx_pdo()->set_rtt(rtt);
            pb_rx_pdo.SerializeToString(pb_str);
        }
    } __attribute__ ( ( __packed__ ) );
};


struct PowF28M36EscSdoTypes {
    // flash param
    uint16_t    Hardware_configuration;
    int16_t     Serial_number_A;
    //...
    // ram param
    char        m3_fw_ver[8];
    char        c28_fw_ver[8];
    uint16_t    ctrl_status_cmd;
    uint16_t    ctrl_status_cmd_ack;
    uint16_t    flash_params_cmd;
    uint16_t    flash_params_cmd_ack;
    float       v_batt;
    float       v_load;
    float       i_load;
    float       t_board;
    float       t_heat;
    float       t_batt;
    uint16_t    board_status;
    uint16_t    board_fault;
    uint16_t    FSM;
};

struct PowF28M36LogTypes {

    uint64_t                        ts;     // ns
    PowF28M36EscPdoTypes::pdo_rx    rx_pdo;

    void fprint ( FILE *fp ) {
        fprintf ( fp, "%lu\t", ts );
        rx_pdo.fprint ( fp );
    }
    int sprint ( char *buff, size_t size ) {
        int l = snprintf ( buff, size, "%lu\t", ts );
        return l + rx_pdo.sprint ( buff+l,size-l );
    }
};


class PowF28M36ESC :
    public BasicEscWrapper<PowF28M36EscPdoTypes,PowF28M36EscSdoTypes>,
    public PDO_log<PowF28M36LogTypes>,
    public XDDP_pipe
{
public:
    typedef BasicEscWrapper<PowF28M36EscPdoTypes,PowF28M36EscSdoTypes>  Base;
    typedef PDO_log<PowF28M36LogTypes>                                  Log;

    PowF28M36ESC ( const ec_slavet& slave_descriptor ) :
        Base ( slave_descriptor ),
        Log ( std::string ( "/tmp/PowF28M36ESC_pos"+std::to_string ( position ) +"_log.txt" ),DEFAULT_LOG_SIZE ),
        XDDP_pipe()
    {
        _start_log = false;
    }

    virtual ~PowF28M36ESC ( void ) {
        delete [] SDOs;
        DPRINTF ( "~%s %d\n", typeid ( this ).name(), position );
        print_stat ( s_rtt );
    }

    ///////////////////////////////////////////////////////////////////////////
    // Overrides functions from iit::ecat::BasicEscWrapper
    virtual void on_readPDO ( void );
    virtual void on_writePDO ( void );
    virtual const objd_t * get_SDOs();
    virtual uint32_t get_ESC_type();
    virtual void init_SDOs ( void );
    
    void print_info ( void );
    int init ( const YAML::Node & root_cfg );
    void handle_status ( void );
    int power_on_ok ( void );

private:

    YAML::Node node_cfg;
    objd_t * SDOs;
    stat_t  s_rtt;

};

inline void PowF28M36ESC::print_info ( void ) {
    DPRINTF ( "\tfw_ver m3 %s\tc28 %s\n", 
              std::string((const char *)sdo.m3_fw_ver,8).c_str(),
              std::string((const char *)sdo.c28_fw_ver,8).c_str() );
}

inline void PowF28M36ESC::on_readPDO ( void ) {

    if ( rx_pdo.rtt ) {
        rx_pdo.rtt = ( uint16_t ) ( get_time_ns() /1000 - rx_pdo.rtt );
        s_rtt ( rx_pdo.rtt );
    }

    handle_status();

    if ( _start_log ) {
        Log::log_t log;
        log.ts = get_time_ns() - _start_log_ts ;
        log.rx_pdo  = rx_pdo;
        push_back ( log );
    }

    xddp_write ( rx_pdo );
}

inline void PowF28M36ESC::on_writePDO ( void ) {
    tx_pdo.ts = ( uint16_t ) ( get_time_ns() /1000 );
}

inline const objd_t * PowF28M36ESC::get_SDOs() {
    return SDOs;
}

inline uint32_t PowF28M36ESC::get_ESC_type() {
    return POW_F28M36_BOARD;
}

inline int PowF28M36ESC::init ( const YAML::Node & root_cfg ) {

    try {
        init_SDOs();
        init_sdo_lookup(true);

    } catch ( EscWrpError &e ) {

        DPRINTF ( "Catch Exception in %s ... %s\n", __PRETTY_FUNCTION__, e.what() );
        return EC_BOARD_INIT_SDO_FAIL;
    }

    // we log when receive PDOs
    start_log ( true );

    XDDP_pipe::init( "PowF28M36ESC_pos_"+std::to_string ( position ) );
    
    try {
        power_on_ok();
    } catch ( EscWrpError &e ) {
        DPRINTF ( "Catch Exception in %s ... %s\n", __PRETTY_FUNCTION__, e.what() );
        return EC_BOARD_NOK;
    }
        
    set_ctrl_status_X ( this, CTRL_FAN_1_ON );
    set_ctrl_status_X ( this, CTRL_FAN_2_ON );

    return EC_BOARD_OK;
}

inline void PowF28M36ESC::handle_status ( void ) {

    static pow_f28m36::status_t status;

    status.all = rx_pdo.status.all;
}

inline int PowF28M36ESC::power_on_ok ( void ) {
    readSDO_byname ( "status" );
    handle_status();
    return rx_pdo.status.bit.c28_main_rel_status == 1;
}


}
}
}

#endif


// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
