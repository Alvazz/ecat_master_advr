/*
 * mc_hand_esc.h
 *
 *  LowPower Motor Controlleer
 *  based on TI TM4C123AH6PM - Tiva Microcontroller
 *  High performance 32-Bit ARM Cortex M4F
 *
 *  http://www.ti.com/product/tm4c123ah6pm
 *
 *  Created on: Jan 2017
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_MC_HAND_ESC_H__
#define __IIT_ECAT_ADVR_MC_HAND_ESC_H__

#include <map>

#include <iit/ecat/advr/mc_lowpwr_esc.h>

namespace iit {
namespace ecat {
namespace advr {

struct McHandEscPdoTypes {
                    
    // TX  slave_input -- master output
    struct pdo_tx {
        float       pos_ref;    // rad   
        int16_t     vel_ref;    // mrad/s 
        int16_t     tor_ref;    // mNm
        uint16_t    gain_0;      
        uint16_t    gain_1;     
        uint16_t    gain_2;     
        uint16_t    gain_3;     
        uint16_t    gain_4;     
        uint16_t    fault_ack;
        uint16_t    ts;
        uint16_t    op_idx_aux;  // op [get/set] , idx
        float       aux;         // set value

        std::ostream& dump ( std::ostream& os, const std::string delim ) const {
            os << pos_ref << delim;
            os << vel_ref << delim;
            os << tor_ref << delim;
            os << gain_0 << delim;
            os << gain_1 << delim;
            os << gain_2 << delim;
            os << gain_3 << delim;
            os << gain_4 << delim;
            os << std::hex << fault_ack << std::dec << delim;
            os << ts << delim;
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

    }  __attribute__ ( ( __packed__ ) ); // 28 bytes

    // RX  slave_output -- master input
    struct pdo_rx {
        float        link_pos;          // rad
        float        motor_pos;         // rad
        float        link_vel;          // mrad/s      TO CHECK ?!?!
        //int16_t      link_vel;          // mrad/s 
        int16_t      motor_vel;         // mrad/s
        //float        torque;            // Nm
        int16_t      analog1;           // 1st touch sensor (previously called torque)
        uint16_t     temperature;       // C
        uint16_t     fault;
        uint16_t     rtt;               // us
        //uint16_t     op_idx_ack;        // op [ack/nack] , idx
        //float        aux;               // get value or nack erro code
        int16_t      analog2;             // 2nd touch sensor
        int16_t      analog3;             // 3rd touch sensor

        std::ostream& dump ( std::ostream& os, const std::string delim ) const {
            os << link_pos << delim;
            os << motor_pos << delim;
            os << (float)link_vel/1000 << delim;
            os << (float)motor_vel/1000 << delim;
            os << analog1 << delim;
            os << temperature << delim;
            os << fault << delim;
            os << rtt << delim;
            os << analog2 << delim;
            os << analog3 << delim;
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
            JPDO ( link_pos );
            JPDO ( motor_pos );
            JPDO ( (float)link_vel/1000 );
            JPDO ( (float)motor_vel/1000 );
            JPDO ( analog1 );
            JPDO ( temperature );
            JPDO ( fault );
            JPDO ( rtt );
            JPDO ( analog2 );
            JPDO ( analog3 );
        }
        void pb_toString( std::string * pb_str ) {
            static iit::advr::Ec_slave_pdo pb_rx_pdo;
            static struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            // Header
            pb_rx_pdo.mutable_header()->mutable_stamp()->set_sec(ts.tv_sec);
            pb_rx_pdo.mutable_header()->mutable_stamp()->set_nsec(ts.tv_nsec);
            // Type
            pb_rx_pdo.set_type(iit::advr::Ec_slave_pdo::RX_MC_HAND);
            // Motor_xt_tx_pdo
            pb_rx_pdo.mutable_mchand_rx_pdo()->set_link_pos(link_pos);
            pb_rx_pdo.mutable_mchand_rx_pdo()->set_motor_pos(motor_pos);
            pb_rx_pdo.mutable_mchand_rx_pdo()->set_link_vel((float)link_vel/1000);
            pb_rx_pdo.mutable_mchand_rx_pdo()->set_motor_vel((float)motor_vel/1000);
            pb_rx_pdo.mutable_mchand_rx_pdo()->set_analog_1(analog1);
            pb_rx_pdo.mutable_mchand_rx_pdo()->set_temperature(temperature);
            pb_rx_pdo.mutable_mchand_rx_pdo()->set_fault(fault);
            pb_rx_pdo.mutable_mchand_rx_pdo()->set_rtt(rtt);
            pb_rx_pdo.mutable_mchand_rx_pdo()->set_analog_2(analog2);
            pb_rx_pdo.mutable_mchand_rx_pdo()->set_analog_3(analog3);
            pb_rx_pdo.SerializeToString(pb_str);
        }
    }  __attribute__ ( ( __packed__ ) ); // ... bytes

}; // 28 + ... bytes

inline std::ostream& operator<< (std::ostream& os, const McHandEscPdoTypes::pdo_tx& tx_pdo ) {
    return tx_pdo.dump(os,"\t");
}

inline std::ostream& operator<< (std::ostream& os, const McHandEscPdoTypes::pdo_rx& rx_pdo ) {
    return rx_pdo.dump(os,"\t");
}


struct LoPwrHandLogTypes {

    uint64_t                ts;     // ns
    float                   pos_ref;
    McHandEscPdoTypes::pdo_rx   rx_pdo;

    void fprint ( FILE *fp ) {
        fprintf ( fp, "%lu\t%f\t", ts, pos_ref );
        rx_pdo.fprint ( fp );
    }
    int sprint ( char *buff, size_t size ) {
        int l = snprintf ( buff, size, "%lu\t%f\t", ts, pos_ref );
        return l + rx_pdo.sprint ( buff+l,size-l );
    }
};

/**
 *
 **/

class LpHandESC :
    public BasicEscWrapper<McHandEscPdoTypes,LoPwrEscSdoTypes>,
    public PDO_log<LoPwrHandLogTypes>,
    public XDDP_pipe
    //public Motor
{
public:
    typedef BasicEscWrapper<McHandEscPdoTypes,LoPwrEscSdoTypes> 	Base;
    typedef PDO_log<LoPwrHandLogTypes>                   		Log;

    LpHandESC ( const ec_slavet& slave_descriptor ) :
        Base ( slave_descriptor ),
        Log ( std::string ( "/tmp/LpHandESC_pos"+std::to_string ( position ) +"_log.txt" ),DEFAULT_LOG_SIZE ),
        XDDP_pipe ()
    {
         _start_log = false;
        //_actual_state = EC_STATE_PRE_OP;
    }

    virtual ~LpHandESC ( void ) {
        delete [] SDOs;
        DPRINTF ( "~%s %d\n", typeid ( this ).name(), position );
        print_stat ( s_rtt );
    }

    virtual uint16_t get_ESC_type() {
        return LO_PWR_HAND_MC;
    }

    virtual int16_t get_robot_id() {
        return sdo.Joint_robot_id;
    }

    void print_info ( void ) {
        DPRINTF ( "\tJoint id %d\tJoint robot id %d\n", sdo.Joint_number, sdo.Joint_robot_id );
        DPRINTF ( "\tmin pos %f\tmax pos %f\tmax vel %f\n", sdo.Min_pos, sdo.Max_pos, sdo.Target_velocity );
        DPRINTF ( "\tPosGainP: %f PosGainI: %f PosGainD: %f I lim: %f\n", sdo.PosGainP, sdo.PosGainI, sdo.PosGainD, sdo.Pos_I_lim );
        DPRINTF ( "\tImpPosGainP :%f ImpPosGainD:%f\n", sdo.ImpedancePosGainP, sdo.ImpedancePosGainD );
        DPRINTF ( "\tTorGainP:%f TorGainI:%f Tor_I_lim:%f\n", sdo.TorGainP, sdo.TorGainI, sdo.Tor_I_lim );
        DPRINTF ( "\tfw_ver %s\n", std::string((const char *)sdo.firmware_version,8).c_str() );
    }

    virtual const objd_t * get_SDOs() {
        return SDOs;
    }

    virtual void init_SDOs ( void );

protected :
    
    virtual void on_readPDO ( void ) {

        if ( rx_pdo.rtt ) {
            rx_pdo.rtt = ( uint16_t ) ( get_time_ns() /1000 ) - rx_pdo.rtt;
            //DPRINTF(">> %s >> %d\n", __PRETTY_FUNCTION__, rx_pdo.rtt);
            s_rtt ( rx_pdo.rtt );
        }

        if ( rx_pdo.fault ) {
            handle_fault();
        } else {
            // clean any previuos fault ack !!
            tx_pdo.fault_ack = 0;
        }

        // apply transformation from Motor to Joint
        rx_pdo.link_pos = lopwr_esc::M2J ( rx_pdo.link_pos,_sgn,_offset );
        rx_pdo.motor_pos = lopwr_esc::M2J ( rx_pdo.motor_pos,_sgn,_offset );
        //rx_pdo.pos_ref_fb  = lopwr_esc::M2J ( rx_pdo.pos_ref_fb,_sgn,_offset );

        if ( _start_log ) {
            Log::log_t log;
            log.ts = get_time_ns() - _start_log_ts ;
            log.pos_ref = lopwr_esc::M2J ( tx_pdo.pos_ref,_sgn,_offset );
            log.rx_pdo = rx_pdo;
            push_back ( log );
        }
        
        xddp_write ( rx_pdo );
    }

    virtual void on_writePDO ( void ) {

        tx_pdo.ts = ( uint16_t ) ( get_time_ns() /1000 );
        // NOOOOOOOOOOOO
        // NOT HERE !!! use set_posRef to apply transformation from Joint to Motor
        //tx_pdo.pos_ref = lopwr_esc::J2M(tx_pdo.pos_ref,_sgn,_offset);

    }

    virtual int on_readSDO ( const objd_t * sdobj )  {

        if ( ! strcmp ( sdobj->name, "link_pos" ) ) {
            rx_pdo.link_pos = lopwr_esc::M2J ( rx_pdo.link_pos,_sgn,_offset );
            //DPRINTF("on_getSDO M2J link_pos %f\n", rx_pdo.position);
        } else if ( ! strcmp ( sdobj->name, "motor_pos" ) ) {
            rx_pdo.motor_pos = lopwr_esc::M2J ( rx_pdo.motor_pos,_sgn,_offset );
        } else if ( ! strcmp ( sdobj->name, "Min_pos" ) ) {
            sdo.Min_pos = lopwr_esc::M2J ( sdo.Min_pos,_sgn,_offset );
        } else if ( ! strcmp ( sdobj->name, "Max_pos" ) ) {
            sdo.Max_pos = lopwr_esc::M2J ( sdo.Max_pos,_sgn,_offset );
        }
        return EC_BOARD_OK;
    }

    virtual int on_writeSDO ( const objd_t * sdo ) {

        // do not allow to write sdo that map txPDO
        //if ( _actual_state == EC_STATE_OPERATIONAL && sdo->index == 0x7000 ) {
        //    return EC_WRP_SDO_WRITE_CB_FAIL;
        //}
        if ( ! strcmp ( sdo->name, "pos_ref" ) ) {
            tx_pdo.pos_ref = lopwr_esc::J2M ( tx_pdo.pos_ref,_sgn,_offset );
            //DPRINTF("on_setSDO J2M pos_ref %f\n", tx_pdo.pos_ref);
        }
        return EC_BOARD_OK;
    }

public:        
    ///////////////////////////////////////////////////////
    ///
    /// Motor method implementation
    ///
    ///////////////////////////////////////////////////////

    virtual const pdo_rx_t& getRxPDO() const {
        return Base::getRxPDO();
    }
    virtual const pdo_tx_t& getTxPDO() const {
        return Base::getTxPDO();
    }
    virtual void setTxPDO ( const pdo_tx_t & pdo_tx ) {
        Base::setTxPDO ( pdo_tx );
    }

    virtual int init ( const YAML::Node & root_cfg ) {

        std::string esc_conf_key;
        Joint_robot_id = -1;

        try {
            // !! sgn and offset must set before init_sdo_lookup !!
            init_SDOs();
            init_sdo_lookup();
            readSDO_byname ( "Joint_robot_id", Joint_robot_id );
            readSDO_byname ( "Joint_number" );
            readSDO_byname ( "fw_ver" );
            
        } catch ( EscWrpError &e ) {

            DPRINTF ( "Catch Exception in %s ... %s\n", __PRETTY_FUNCTION__, e.what() );
            return EC_BOARD_INIT_SDO_FAIL;
        }

        if ( Joint_robot_id > 0 ) {
            try {
                esc_conf_key = std::string ( "LpHandESC_"+std::to_string ( Joint_robot_id ) );
                if ( read_conf ( esc_conf_key, root_cfg ) != EC_WRP_OK ) {
                    esc_conf_key = std::string ( "LpHandESC_X" );
                    if ( read_conf ( esc_conf_key, root_cfg ) != EC_WRP_OK ) {
                        DPRINTF ( "NO config for LpHandESC_%d in %s\n", Joint_robot_id, __PRETTY_FUNCTION__ );
                        return EC_BOARD_KEY_NOT_FOUND;
                    }
                }
            } catch ( YAML::KeyNotFound &e ) {
                DPRINTF ( "Catch Exception in %s ... %s\n", __PRETTY_FUNCTION__, e.what() );
                return EC_BOARD_KEY_NOT_FOUND;
            }
        } else {
            return EC_BOARD_INVALID_ROBOT_ID;
        }

        // redo read SDOs so we can apply _sgn and _offset to transform Min_pos Max_pos to Joint Coordinate
        readSDO_byname ( "Min_pos" );
        readSDO_byname ( "Max_pos" );
        readSDO_byname ( "Target_velocity" );
        readSDO_byname ( "link_pos" );
        readSDO_byname ( "pos_gain_P");
        readSDO_byname ( "pos_gain_I");
        readSDO_byname ( "pos_gain_D");
        readSDO_byname ( "pos_integral_limit");
        readSDO_byname ( "ImpedancePosGainP");
        readSDO_byname ( "ImpedancePosGainD");
        readSDO_byname ( "tor_gain_P");
        readSDO_byname ( "tor_gain_I");
        readSDO_byname ( "tor_integral_limit");

        log_filename = std::string ( "/tmp/LpHandESC_"+std::to_string ( sdo.Joint_robot_id ) +"_log.txt" );

        // we log when receive PDOs
        start_log ( true );

        XDDP_pipe::init ("Motor_id_"+std::to_string ( get_robot_id() ));
        
        return EC_WRP_OK;

    }
    
    //virtual int start ( int controller_type, float _p, float _i, float _d ) {
    virtual int start ( int controller_type, const std::vector<float> &gains ) {

        std::ostringstream oss;
        float act_position;
        uint16_t fault;
        //uint16_t gain;
        float gain;
        
        try {
            set_ctrl_status_X ( this, CTRL_POWER_MOD_OFF );
            
            if ( controller_type == CTRL_SET_POS_MODE ) {
                
                writeSDO_byname( "pos_gain_P", gains[0] );
                writeSDO_byname( "pos_gain_I", gains[1] );
                writeSDO_byname( "pos_gain_D", gains[2] );
                DPRINTF ( "\tPosGain %f %f %f\n", gains[0], gains[1], gains[2] );
            
            } else if ( controller_type == CTRL_SET_IMPED_MODE ) {

                // Impedance gains : position PD torque PI 
                // PosGainP
                writeSDO_byname( "gain_0", (uint16_t)(gains[0]/100.0));
                // TorGainP
                writeSDO_byname( "gain_1", (uint16_t)(sdo.TorGainP*1000));
                // PosGainD
                writeSDO_byname( "gain_2", (uint16_t)gains[2]);
                // TorGainD
                //writeSDO_byname( "gain_3", (uint16_t)(sdo.TorGainP));
                // TorGainIs
                writeSDO_byname( "gain_4", (uint16_t)(sdo.TorGainI*1000));
                
                oss << tx_pdo;
                DPRINTF ( "\ttx_pdo %s\n", oss.str().c_str() );
            
            }
            else if ( controller_type == CTRL_SET_VOLT_MODE ) {
            
                int16_t configFlags;
                readSDO_byname("ConfigFlags", configFlags);
                // #define CONFIGFLAG_USE_VOLTAGE_MODE_LIMITS_BIT    8
                configFlags |= (0x1 << 8); 
                writeSDO_byname("ConfigFlags", configFlags);
            }
            
            // set actual position as reference
            readSDO_byname ( "link_pos", act_position );
            writeSDO_byname ( "pos_ref", act_position );
            DPRINTF ( "%s\n\tlink_pos %f pos_ref %f\n", __PRETTY_FUNCTION__,
                      act_position,
                      lopwr_esc::M2J(tx_pdo.pos_ref,_sgn,_offset) );
            oss << tx_pdo;
            DPRINTF ( "\ttx_pdo %s\n", oss.str().c_str() );
            // set direct mode and power on modulator
            set_ctrl_status_X ( this, CTRL_SET_DIRECT_MODE );
            set_ctrl_status_X ( this, CTRL_POWER_MOD_ON );

            readSDO_byname ( "fault", fault );
            handle_fault();

            // set controller mode
            set_ctrl_status_X ( this, controller_type );


        } catch ( EscWrpError &e ) {

            DPRINTF ( "Catch Exception %s ... %s\n", __PRETTY_FUNCTION__, e.what() );
            return EC_BOARD_NOK;
        }

        return EC_BOARD_OK;

    }

    virtual int start ( int controller_type ) {
        
        std::vector<float> gains = {0.0, 0.0, 0.0};
        
        if ( controller_type == CTRL_SET_POS_MODE ) {
            // use default value read in init()
            gains[0] = sdo.PosGainP;
            gains[1] = sdo.PosGainI;
            gains[2] = sdo.PosGainD;
            
            if ( node_cfg["pid"]["position"] ) {
                try {
                    gains = node_cfg["pid"]["position"].as<std::vector<float>>();
                    assert ( gains.size() == 3 );
                } catch ( std::exception &e ) {
                    DPRINTF ( "Catch Exception in %s ... %s\n", __PRETTY_FUNCTION__, e.what() );
                }
            }
        } 
        if ( controller_type == CTRL_SET_IMPED_MODE ) {
            // Impedance gains : [ Pos_P, Pos_D torque PI 
            // use default value read in init()
            gains[0] = sdo.ImpedancePosGainP;
            gains[1] = 0,0; // not used
            gains[2] = sdo.ImpedancePosGainD;

            if ( node_cfg["pid"]["impedance"] ) {
                try {
                    gains = node_cfg["pid"]["impedance"].as<std::vector<float>>();
                    assert ( gains.size() == 3 );
                    DPRINTF ( "using yaml values\n");  
                } catch ( std::exception &e ) {
                    DPRINTF ( "Catch Exception in %s ... %s\n", __PRETTY_FUNCTION__, e.what() );
                }
            }
        } 

        //return start ( controller_type, gains[0], gains[1], gains[2] );
        return start ( controller_type, gains );
    }

    virtual int stop ( void ) {
        return set_ctrl_status_X ( this, CTRL_POWER_MOD_OFF );
    }

    virtual void start_log ( bool start ) {
        Log::start_log ( start );
    }

    virtual void handle_fault ( void ) {

        fault_t fault;
        fault.all = rx_pdo.fault;
        //fault.bit.
        tx_pdo.fault_ack = fault.all & 0xFFFF;
        //ack_faults_X(this, fault.all);

    }

    /////////////////////////////////////////////
    // set pdo data
    virtual int set_posRef ( float joint_pos ) {
        tx_pdo.pos_ref = lopwr_esc::J2M(joint_pos,_sgn,-_offset);
    }
    virtual int set_velRef ( float joint_vel ) {
        tx_pdo.vel_ref = joint_vel;
    }
    virtual int set_torRef ( float joint_tor ) {
        tx_pdo.tor_ref = joint_tor;
    }
#if 0
    virtual int set_torOffs ( float tor_offs ) {
        /*tx_pdo.tor_offs = tor_offs;*/
    }
    virtual int set_posGainP ( float p_gain )  {
        tx_pdo.gain_0 = p_gain;
    }
    virtual int set_posGainI ( float i_gain )  {
        tx_pdo.gain_2 = i_gain;
    }
    virtual int set_posGainD ( float d_gain )  {
        tx_pdo.gain_1 = d_gain;
    }
#endif
    virtual int move_to ( float pos_ref, float step ) {

        float pos, tx_pos_ref;

        try {
            readSDO_byname ( "link_pos", pos );
            readSDO_byname ( "pos_ref", tx_pos_ref );
            //tx_pos_ref = pos_ref;
            if ( fabs ( pos - pos_ref ) > step ) {
                if ( pos > pos_ref ) {
                    tx_pos_ref -= step*2;
                } else {
                    tx_pos_ref += step*2;
                }

                writeSDO_byname ( "pos_ref", tx_pos_ref );
                DPRINTF ( "%d move to %f %f %f\n", Joint_robot_id, pos_ref, tx_pos_ref, pos );
                return 0;
            } else {
                DPRINTF ( "%d move to %f %f %f\n", Joint_robot_id, pos_ref, tx_pos_ref, pos );
                return 1;
            }

        } catch ( EscWrpError &e ) {
            DPRINTF ( "Catch Exception %s ... %s\n", __FUNCTION__, e.what() );
            return 0;
        }

    }


private:

    int read_conf ( std::string conf_key, const YAML::Node & root_cfg ) {

        if ( ! root_cfg[conf_key] ) {
            return EC_BOARD_KEY_NOT_FOUND;
        }

        DPRINTF ( "Using config %s\n", conf_key.c_str() );
        
        node_cfg = root_cfg[conf_key];
        
        _sgn = node_cfg["sign"].as<int>();
        _offset = node_cfg["pos_offset"].as<float>();
        _offset = DEG2RAD ( _offset );

        return EC_WRP_OK;
    }

    int16_t Joint_robot_id;

    YAML::Node node_cfg;

    float   _offset;
    int     _sgn;

    stat_t  s_rtt;

    objd_t * SDOs;

};



}
}
}
#endif /* IIT_ECAT_ADVR_ESC_H_ */
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
