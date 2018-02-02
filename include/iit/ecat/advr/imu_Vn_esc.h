/*
 *
 *  Created on: May, 2015
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_IMU_ESC_H__
#define __IIT_ECAT_ADVR_IMU_ESC_H__

#include <iit/ecat/advr/esc.h>
#include <iit/ecat/advr/log_esc.h>
#include <iit/ecat/advr/pipes.h>
#include <protobuf/ecat_pdo.pb.h>

#include <map>
#include <string>


namespace iit {
namespace ecat {
namespace advr {


struct ImuEscPdoTypes {

    // TX  slave_input -- master output
    struct pdo_tx {
        uint16_t    fault_ack;
        uint16_t    digital_out;
        uint16_t    ts;
    } __attribute__ ( ( __packed__ ) );

    // RX  slave_output -- master input
    struct pdo_rx {
        float       x_rate;       
        float       y_rate;       
        float       z_rate;       
        float       x_acc;       
        float       y_acc;       
        float       z_acc;       
        float       x_quat;       
        float       y_quat;       
        float       z_quat;       
        float       w_quat;       
        uint32_t    imu_ts;          
        uint16_t    temperature;          
        uint16_t    digital_in;          
        uint16_t    fault;
        uint16_t    rtt;                // us
        
        std::ostream& dump ( std::ostream& os, const std::string delim ) const {
            os << x_rate << delim;
            os << y_rate << delim;
            os << z_rate << delim;
            os << x_acc << delim;
            os << y_acc << delim;
            os << z_acc << delim;
            os << x_quat << delim;
            os << y_quat << delim;
            os << z_quat << delim;
            os << w_quat << delim;
            os << imu_ts << delim;
            os << temperature << delim;
            os << digital_in << delim;
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
            assert(0);
        }
        void pb_toString( std::string * pb_str ) {
            static iit::advr::Ec_slave_pdo pb_rx_pdo;
            static struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            // Header
            pb_rx_pdo.mutable_header()->mutable_stamp()->set_sec(ts.tv_sec);
            pb_rx_pdo.mutable_header()->mutable_stamp()->set_nsec(ts.tv_nsec);
            // Type
            pb_rx_pdo.set_type(iit::advr::Ec_slave_pdo::RX_IMU_VN);
            // ImuVN_rx_pdo
            pb_rx_pdo.mutable_imuvn_rx_pdo()->set_x_rate(x_rate);
            pb_rx_pdo.mutable_imuvn_rx_pdo()->set_y_rate(y_rate);
            pb_rx_pdo.mutable_imuvn_rx_pdo()->set_z_rate(z_rate);
            pb_rx_pdo.mutable_imuvn_rx_pdo()->set_x_acc(x_acc);
            pb_rx_pdo.mutable_imuvn_rx_pdo()->set_y_acc(y_acc);
            pb_rx_pdo.mutable_imuvn_rx_pdo()->set_z_acc(z_acc);
            pb_rx_pdo.mutable_imuvn_rx_pdo()->set_x_quat(x_quat);
            pb_rx_pdo.mutable_imuvn_rx_pdo()->set_y_quat(y_quat);
            pb_rx_pdo.mutable_imuvn_rx_pdo()->set_z_quat(z_quat);
            pb_rx_pdo.mutable_imuvn_rx_pdo()->set_imu_ts(imu_ts);
            pb_rx_pdo.mutable_imuvn_rx_pdo()->set_temperature(temperature);
            pb_rx_pdo.mutable_imuvn_rx_pdo()->set_digital_in(digital_in);
            pb_rx_pdo.mutable_imuvn_rx_pdo()->set_fault(fault);
            pb_rx_pdo.mutable_imuvn_rx_pdo()->set_rtt(rtt);
            pb_rx_pdo.SerializeToString(pb_str);
        }
    } __attribute__ ( ( __packed__ ) );
};


struct ImuEscSdoTypes {
    
    // flash param
    short config_flags;
    short config_flags2;
    short digital_out_cfg;
    short digital_in_cfg;
    short digital_ain_cfg;
    short function_cfg;
    short sensor_num;
    short joint_robot_id;
    // ram param
    char fw_ver[8];
    unsigned int ack_board_faults;
    float dbg_1;
    float dbg_2;
    float dbg_3;
    unsigned short flash_params_cmd;
    unsigned short flash_params_cmd_ack;
};


struct ImuEscLogTypes {
    
    uint64_t                ts;     // ns
    ImuEscPdoTypes::pdo_rx  rx_pdo;

    std::ostream& dump ( std::ostream& os, const std::string delim ) const {
            os << ts << delim;
            rx_pdo.dump(os,delim);
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

// Vn stand for VectorNav
class ImuVnESC :
    public BasicEscWrapper<ImuEscPdoTypes,ImuEscSdoTypes>,
    public PDO_log<ImuEscLogTypes>,
    public XDDP_pipe
{
public:
    typedef BasicEscWrapper<ImuEscPdoTypes,ImuEscSdoTypes>  Base;
    typedef PDO_log<ImuEscLogTypes>                         Log;

    ImuVnESC ( const ec_slavet& slave_descriptor ) :
        Base ( slave_descriptor ),
        Log ( std::string ( "/tmp/ImuESC_pos"+std::to_string ( position ) +"_log.txt" ),DEFAULT_LOG_SIZE ),
        XDDP_pipe()
    {
        _start_log = false;
    }

    virtual ~ImuVnESC ( void ) {
        delete [] SDOs;
        DPRINTF ( "~%s %d\n", typeid ( this ).name(), position );
        print_stat ( s_rtt );
    }

    virtual int16_t get_robot_id() {
        return sdo.joint_robot_id;
    }
    
    virtual void on_readPDO ( void ) {

        if ( rx_pdo.rtt ) {
            rx_pdo.rtt = ( uint16_t ) ( get_time_ns() /1000 - rx_pdo.rtt );
            s_rtt ( rx_pdo.rtt );
        }

        if ( _start_log ) {
            Log::log_t log;
            log.ts = get_time_ns() - _start_log_ts ;
            log.rx_pdo = rx_pdo;
            push_back ( log );
        }

        if( use_pipes ) {
            xddp_write ( rx_pdo );
        }
    }

    virtual void on_writePDO ( void ) {
        tx_pdo.ts = ( uint16_t ) ( get_time_ns() /1000 );
    }

    virtual const objd_t * get_SDOs() {
        return SDOs;
    }
    virtual uint32_t get_ESC_type() {
        return IMU_VECTORNAV;
    }

    void print_info ( void ) { return; }

    void init_SDOs ( void );

    int init ( const YAML::Node & root_cfg );


private:

    YAML::Node node_cfg;

    objd_t * SDOs;

    stat_t  s_rtt;

};

///////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////

inline int ImuVnESC::init ( const YAML::Node & root_cfg )     {

    try {
        init_SDOs();
        init_sdo_lookup();
        readSDO_byname ( "joint_robot_id");

    } catch ( EscWrpError &e ) {

        DPRINTF ( "Catch Exception in %s ... %s\n", __PRETTY_FUNCTION__, e.what() );
        return EC_BOARD_INIT_SDO_FAIL;
    }

    if ( sdo.joint_robot_id <= 0 ) {
        return EC_BOARD_INVALID_ROBOT_ID;
    }
    
    // set filename with robot_id
    log_filename = std::string ( "/tmp/ImuESC_"+std::to_string ( sdo.joint_robot_id ) +"_log.txt" );

    // we log when receive PDOs
    start_log ( true );
    
    // set use pipe variable NOTE true by default
    if(root_cfg["ec_board_ctrl"]["use_pipes"]) {
        use_pipes = root_cfg["ec_board_ctrl"]["use_pipes"].as<bool>();
    }

    if( use_pipes ) {
        XDDP_pipe::init ( "Imu_id_"+std::to_string ( sdo.joint_robot_id ) );
    }
    

    return EC_BOARD_OK;
}


    

}
}
}

#endif


// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
