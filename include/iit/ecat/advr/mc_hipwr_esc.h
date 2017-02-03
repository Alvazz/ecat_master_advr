/*
 * mc_hipwr_esc.h
 *
 *  HiPower Motor Controlleer
 *  based on TI TMS320F28335 - Delfino Microcontroller
 *  High-Performance 32-Bit CPU 150 Mhz
 *
 *  http://www.ti.com/product/tms320f28335
 *
 *  Created on: Dec 2014
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_MC_HIPWR_ESC_H__
#define __IIT_ECAT_ADVR_MC_HIPWR_ESC_H__

#include <iit/ecat/ec_master_iface.h>
#include <iit/ecat/slave_wrapper.h>
#include <iit/ecat/advr/esc.h>
#include <iit/ecat/advr/motor_iface.h>
#include <iit/ecat/advr/log_esc.h>
#include <iit/ecat/advr/pipes.h>
#include <iit/ecat/utils.h>
#include <map>


namespace iit {
namespace ecat {
namespace advr {

namespace hipwr_esc {
static float J2M ( float p, int s, float o ) {
    return ( M_PI - ( s*o ) + ( s*p ) );
}
static float M2J ( float p, int s, float o ) {
    return ( ( p - M_PI + ( s*o ) ) /s );
}
//static float J2M(float p, int s, float o) { return p; }
//static float M2J(float p, int s, float o) { return p; }
}

struct HiPwrEscSdoTypes {

    ////////////////////////////
    // flash
    unsigned long Sensor_type;      // Sensor type: NOT USED

    float PosGainP;
    float PosGainI;
    float PosGainD;
    float TorGainP;
    float TorGainI;
    float TorGainD;
    float TorGainFF;
    float Pos_I_lim;                // Integral limit: NOT USED
    float Tor_I_lim;                // Integral limit: NOT USED

    float Min_pos;
    float Max_pos;
    float Max_vel;
    float Max_tor;
    float Max_cur;

    float Enc_offset;
    float Enc_relative_offset;
    float Phase_angle;
    float Torque_lin_coeff;

    uint64_t    Enc_mot_nonius_calib;
    uint64_t    Enc_load_nonius_calib;

    int16_t     Joint_number;
    int16_t     Joint_robot_id;

    // parameters added 
    float MotorInertia;
    float InvMotorInertia;
    float ObserverCutOff;
    float InvGearedTorqueConstant;
    float GearedTorqueConstant;
    float WindingResistance;
    float VoltageFeedforward;
    float BackEmfCompensation;
    float HasDeflectionEncoder;
    float Analog_motor;
    
    ////////////////////////////
    // ram
    char        firmware_version[8];
    uint32_t    board_enable_mask;
    float       Direct_ref;
    float       V_batt_filt_100ms;
    float       Board_Temperature;
    float       T_mot1_filt_100ms;
    uint16_t    ctrl_status_cmd;
    uint16_t    ctrl_status_cmd_ack;
    uint16_t    flash_params_cmd;
    uint16_t    flash_params_cmd_ack;
    int32_t     abs_enc_mot;
    int32_t     abs_enc_2;
    float       angle_enc_mot;  
    float       angle_enc_link;
    float       angle_enc_deflection;
    float       motor_ref;
    // aux
    float       pos_ref_fb;
    float       motor_ref_fb;
    float       motor_out_fb;
    float       torque_ref_fb;
};


struct HiPwrLogTypes {

    uint64_t                ts;     // ns
    float                   pos_ref;
    McEscPdoTypes::pdo_rx   rx_pdo;

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


class HpESC :
    public BasicEscWrapper<McEscPdoTypes,HiPwrEscSdoTypes>,
    public PDO_log<HiPwrLogTypes>,
    public XDDP_pipe,
    public Motor
{

public:
    typedef BasicEscWrapper<McEscPdoTypes,HiPwrEscSdoTypes> Base;
    typedef PDO_log<HiPwrLogTypes>                          Log;

    HpESC ( const ec_slavet& slave_descriptor ) :
        Base ( slave_descriptor ),
        Log ( std::string ( "/tmp/HpESC_pos"+std::to_string ( position ) +"_log.txt" ),DEFAULT_LOG_SIZE ),
        XDDP_pipe ()
    {
        _start_log = false;
        //_actual_state = EC_STATE_PRE_OP;
    }

    virtual ~HpESC ( void ) {

        delete [] SDOs;
        DPRINTF ( "~%s pos %d\n", typeid ( this ).name(), position );
        print_stat ( s_rtt );
    }

    virtual int16_t get_robot_id() {
        //assert(sdo.Joint_robot_id != -1);
        return sdo.Joint_robot_id;
    }

    void print_info ( void ) {

        DPRINTF ( "\tJoint id %c%d\tJoint robot id %d\n", ( char ) ( sdo.Joint_number>>8 ), sdo.Joint_number&0xFF, sdo.Joint_robot_id );
        DPRINTF ( "\tmin pos %f\tmax pos %f\n", sdo.Min_pos, sdo.Max_pos );
        DPRINTF ( "\tfw_ver %s\n", std::string((const char *)sdo.firmware_version,8).c_str() );
    }

    virtual const objd_t * get_SDOs() {
        return SDOs;
    }

    void init_SDOs ( void );

protected :

    virtual void on_readPDO ( void ) {

        if ( rx_pdo.rtt ) {
            rx_pdo.rtt = ( uint16_t ) ( get_time_ns() /1000 - rx_pdo.rtt );
            s_rtt ( rx_pdo.rtt );
        }

        if ( rx_pdo.fault & 0x7FFF ) {
            handle_fault();
        } else {
            // clean any previuos fault ack !!
            tx_pdo.fault_ack = 0;
        }

        // apply transformation from Motor to Joint
        rx_pdo.link_pos = hipwr_esc::M2J ( rx_pdo.link_pos,_sgn,_offset );
        rx_pdo.motor_pos = hipwr_esc::M2J ( rx_pdo.motor_pos,_sgn,_offset );
        //rx_pdo.pos_ref_fb  = hipwr_esc::M2J ( rx_pdo.pos_ref_fb,_sgn,_offset );

        if ( _start_log ) {
            Log::log_t log;
            log.ts      = get_time_ns() - _start_log_ts ;
            log.pos_ref = hipwr_esc::M2J ( tx_pdo.pos_ref,_sgn,_offset );
            log.rx_pdo  = rx_pdo;
            push_back ( log );
        }
        
        xddp_write( rx_pdo );

    }

    virtual void on_writePDO ( void ) {

        tx_pdo.ts = ( uint16_t ) ( get_time_ns() /1000 );
        // NOOOOOOOOOOOO
        // NOT HERE !!! use set_posRef to apply transformation from Joint to Motor
        //tx_pdo.pos_ref = hipwr_esc::J2M(tx_pdo.pos_ref,_sgn,_offset);
    }

    virtual int on_readSDO ( const objd_t * sdobj )  {

        if ( ! strcmp ( sdobj->name, "link_pos" ) ) {
            rx_pdo.link_pos = hipwr_esc::M2J ( rx_pdo.link_pos,_sgn,_offset );
            //DPRINTF("on_getSDO M2J link_pos %f\n", rx_pdo.link_pos);
        } else if ( ! strcmp ( sdobj->name, "motor_pos" ) ) {
            rx_pdo.motor_pos = hipwr_esc::M2J ( rx_pdo.motor_pos,_sgn,_offset );
        } else if ( ! strcmp ( sdobj->name, "Min_pos" ) ) {
            sdo.Min_pos = hipwr_esc::M2J ( sdo.Min_pos,_sgn,_offset );
        } else if ( ! strcmp ( sdobj->name, "Max_pos" ) ) {
            sdo.Max_pos = hipwr_esc::M2J ( sdo.Max_pos,_sgn,_offset );
        }
        return EC_BOARD_OK;
    }

    virtual int on_writeSDO ( const objd_t * sdo ) {

        // do not allow to write sdo that map txPDO
        //if ( _actual_state == EC_STATE_OPERATIONAL && sdo->index == 0x7000 ) {
        //    return EC_WRP_SDO_WRITE_CB_FAIL;
        //}
        if ( ! strcmp ( sdo->name, "pos_ref" ) ) {
            tx_pdo.pos_ref = hipwr_esc::J2M ( tx_pdo.pos_ref,_sgn,_offset );
            //DPRINTF("on_setSDO J2M pos_ref %f\n", tx_pdo.pos_ref);
        }
        return EC_BOARD_OK;
    }

    virtual const pdo_rx_t& getRxPDO() const        {
        return Base::getRxPDO();
    }
    virtual const pdo_tx_t& getTxPDO() const        {
        return Base::getTxPDO();
    }
    virtual void setTxPDO ( const pdo_tx_t & pdo_tx )  {
        Base::setTxPDO ( pdo_tx );
    }


public :
    ///////////////////////////////////////////////////////
    ///
    /// Motor method implementation
    ///
    ///////////////////////////////////////////////////////
    virtual uint16_t get_ESC_type() {
        if ( product_code == HI_PWR_AC_MC ) return HI_PWR_AC_MC;
        if ( product_code == HI_PWR_DC_MC ) return HI_PWR_DC_MC;
        return NO_TYPE;
    }

    virtual int init ( const YAML::Node & root_cfg ) {

        Joint_robot_id = -1;

        try {
            // !! sgn and offset must set before init_sdo_lookup !!
            init_SDOs();
            init_sdo_lookup();
            readSDO_byname ( "Joint_robot_id", Joint_robot_id );
            readSDO_byname ( "Joint_number" );
            readSDO_byname ( "firmware_version" );

        } catch ( EscWrpError &e ) {

            DPRINTF ( "Catch Exception in %s ... %s\n", __PRETTY_FUNCTION__, e.what() );
            return EC_BOARD_INIT_SDO_FAIL;
        }

        if ( Joint_robot_id > 0 ) {
            try {
                std::string esc_conf_key = std::string ( "HpESC_"+std::to_string ( Joint_robot_id ) );
                if ( read_conf ( esc_conf_key, root_cfg ) != EC_WRP_OK ) {
                    esc_conf_key = std::string ( "HpESC_X" );
                    if ( read_conf ( esc_conf_key, root_cfg ) != EC_WRP_OK ) {
                        DPRINTF ( "NO config for HpESC_%d in %s\n", Joint_robot_id, __PRETTY_FUNCTION__ );
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
        readSDO_byname ( "Max_vel" );
        readSDO_byname ( "Max_tor" );
        readSDO_byname ( "Max_cur" );
        readSDO_byname ( "link_pos" );

        // set filename with robot_id
        log_filename = std::string ( "/tmp/HpESC_"+std::to_string ( sdo.Joint_robot_id ) +"_log.txt" );

        // Paranoid Direct_ref
        float direct_ref = 0.0;
        writeSDO_byname ( "Direct_ref", direct_ref );
        readSDO_byname ( "Direct_ref", direct_ref );
        assert ( direct_ref == 0.0 );

        // we log when receive PDOs
        start_log ( true );

        XDDP_pipe::init ( "Motor_id_"+std::to_string ( get_robot_id() ) );
        
        return EC_BOARD_OK;

    }
    ///////////////////////////////////////////////////////
    /**
     * all done with mailbox
     * !! in OPERATIONAL DO NOT ALLOW to set_SDO that maps TX_PDO !!
     * @return int
     */
    virtual int start ( int controller_type, const std::vector<float> &gains ) {

        std::ostringstream oss;
        float actual_position;
        float actual_torque;
        uint16_t fault;
        uint32_t enable_mask = 0x0;
        uint16_t gain;
        float max_vel = 3.0;

        DPRINTF ( "Start motor[%d] 0x%02X %.2f %.2f %.2f\n",
                  Joint_robot_id, controller_type, gains[0], gains[1], gains[2] );

        try {
            set_ctrl_status_X ( this, CTRL_POWER_MOD_OFF );
            
            if ( controller_type == CTRL_SET_POS_MODE ||
                 controller_type == CTRL_SET_MIX_POS_MODE ) {

                // pdo gains will be used in OP
                writeSDO_byname ( "PosGainP", gains[0] );
                writeSDO_byname ( "PosGainI", gains[1] );
                writeSDO_byname ( "PosGainD", gains[2] );
                // this will SET tx_pdo.gain_x
                // pdo gains will be used in OP
                // pos_Kp
                gain = (uint16_t)gains[0];
                writeSDO_byname ( "gain_0", gain );
                // pos_Kd
                gain = (uint16_t)gains[2];
                writeSDO_byname ( "gain_1", gain );
                
                
            } else if ( controller_type == CTRL_SET_IMPED_MODE ) {
            
                // pos_Kp
                gain = (uint16_t)gains[0];
                writeSDO_byname ( "gain_0", gain );
                // pos_Kd
                gain = (uint16_t)gains[1];
                writeSDO_byname ( "gain_1", gain );
                // tor_Kp
                gain = (uint16_t)(gains[2] * 10000);
                writeSDO_byname ( "gain_2", gain );
                // tor_Kd
                gain = (uint16_t)(gains[3] * 10000);
                writeSDO_byname ( "gain_3", gain );
                // tor_Ki
                gain = (uint16_t)(gains[4] * 10000);
                writeSDO_byname ( "gain_4", gain );

            }
            
            // set actual position as reference
            //readSDO_byname ( "link_pos", act_position );
            readSDO_byname ( "motor_pos", actual_position );
            readSDO_byname ( "torque", actual_torque );
            writeSDO_byname ( "pos_ref", actual_position );
            writeSDO_byname ( "Max_vel", max_vel );

            DPRINTF ( "%s\n\tlink_pos %f torque %f pos_ref %f\n", __PRETTY_FUNCTION__,
                      actual_position, actual_torque,
                      hipwr_esc::M2J(tx_pdo.pos_ref,_sgn,_offset) );
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

            DPRINTF ( "Catch Exception %s ... %s\n", __FUNCTION__, e.what() );
            return EC_BOARD_NOK;
        }

        return EC_BOARD_OK;

    }

    virtual int start ( int controller_type ) {

        std::vector<float> gains;
        
        if ( controller_type == CTRL_SET_POS_MODE ) {
            if ( node_cfg["pid"]["position"] ) {
                try {
                    gains = node_cfg["pid"]["position"].as<std::vector<float>>();
                    assert ( gains.size() == 3 );
                } catch ( std::exception &e ) {
                    DPRINTF ( "Catch Exception in %s ... %s\n", __PRETTY_FUNCTION__, e.what() );
                    return EC_BOARD_NOK;
                }
            }
        } 
        if ( controller_type == CTRL_SET_MIX_POS_MODE ) {
            if ( node_cfg["pid"]["mix_position"] ) {
                try {
                    gains = node_cfg["pid"]["mix_position"].as<std::vector<float>>();
                    assert ( gains.size() == 3 );
                } catch ( std::exception &e ) {
                    DPRINTF ( "Catch Exception in %s ... %s\n", __PRETTY_FUNCTION__, e.what() );
                    return EC_BOARD_NOK;
                }
            }
        } 
        if ( controller_type == CTRL_SET_IMPED_MODE ) {
            if ( node_cfg["pid"]["impedance"] ) {
                try {
                    gains = node_cfg["pid"]["impedance"].as<std::vector<float>>();
                    assert ( gains.size() == 5 );
                    DPRINTF ( "using yaml values\n");  
                } catch ( std::exception &e ) {
                    DPRINTF ( "Catch Exception in %s ... %s\n", __PRETTY_FUNCTION__, e.what() );
                    return EC_BOARD_NOK;
                }
            }
        } 

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
        //DPRINTF("[%d]fault 0x%04X\n", Joint_robot_id, fault.all );
        //fault.bit.
        tx_pdo.fault_ack = fault.all & 0x7FFF;
    }

    /////////////////////////////////////////////
    // set pdo data
    // TODO check valid range
    virtual int set_posRef ( float joint_pos ) {
        tx_pdo.pos_ref = hipwr_esc::J2M(joint_pos,_sgn,_offset);
        return EC_BOARD_OK;
    }
    virtual int set_velRef ( float joint_vel ) {
        tx_pdo.vel_ref = (int16_t)(joint_vel*1000);
        return EC_BOARD_OK;
    }
    virtual int set_torRef ( float joint_tor ) {
        tx_pdo.tor_ref = (int16_t)(joint_tor*100);
        return EC_BOARD_OK;
    }

#if 0
    virtual int set_torOffs ( float tor_offs ) {
        /*tx_pdo.tor_offs = tor_offs;*/
    }
    virtual int set_posGainP ( float p_gain )  {
        tx_pdo.gain_kp_l = p_gain;
    }
    virtual int set_posGainI ( float i_gain )  {
        tx_pdo.gain_ki = i_gain;
    }
    virtual int set_posGainD ( float d_gain )  {
        tx_pdo.gain_kd_l = d_gain;
    }
#endif
    virtual int move_to ( float pos_ref, float step ) {

    float       pos, link_pos, motor_pos, tx_pos_ref;
    uint16_t    fault;

    try {
        readSDO_byname ( "fault", fault );
        handle_fault();
        readSDO_byname ( "link_pos", link_pos );
        readSDO_byname ( "motor_pos", motor_pos );
        readSDO_byname ( "pos_ref", tx_pos_ref );
        tx_pos_ref = hipwr_esc::M2J ( tx_pos_ref,_sgn,_offset );

        // use motor_position !!!
        pos = motor_pos;
        
        if ( fabs ( pos - pos_ref ) > step ) {
            if ( pos > pos_ref ) {
                tx_pos_ref -= step;
            } else {
                tx_pos_ref += step;
            }

            writeSDO_byname ( "pos_ref", tx_pos_ref );
            DPRINTF("%d move to %f %f %f\n", Joint_robot_id, pos_ref, tx_pos_ref, pos);
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

    /**
     * - set "Enc_offset" = 0
     * - set "Calibration_angle" motor coordinate : Motor 3.14159 --> Joint 0
     * - set "ctrl_status_cmd" =  0x00AB CTRL_SET_ZERO_POSITION
     * - save params to flash
     * -
     */
    int set_zero_position ( float calibration_angle ) {

        float enc_offset;

        // do it in PREOP
        enc_offset = 0.0;
        DPRINTF ( "%d : set zero pos %f\n", position, calibration_angle );

        try {
            writeSDO_byname ( "Enc_offset", enc_offset );
            writeSDO_byname ( "Calibration_angle", calibration_angle );
        } catch ( EscWrpError &e ) {
            DPRINTF ( "Catch Exception %s ... %s\n", __FUNCTION__, e.what() );
            return EC_BOARD_ZERO_POS_FAIL;
        }
        set_ctrl_status_X ( this, CTRL_SET_ZERO_POSITION );
        set_flash_cmd_X ( this, FLASH_SAVE );

        return EC_BOARD_OK;
    }


private:

    /*
     * set node_cfg member and read from it
     */ 
    int read_conf ( std::string conf_key, const YAML::Node & root_cfg );
    
    int16_t Joint_robot_id;

    YAML::Node node_cfg;

    float   _offset;
    int     _sgn;

    stat_t  s_rtt;

    objd_t * SDOs;

};

///////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////


inline int HpESC::read_conf ( std::string conf_key, const YAML::Node & root_cfg ) {

        if ( ! root_cfg[conf_key] ) {
            return EC_BOARD_KEY_NOT_FOUND;
        }

        DPRINTF ( "Using config %s\n", conf_key.c_str() );
        
        node_cfg = root_cfg[conf_key];
        
        _sgn = node_cfg["sign"].as<int>();
        _offset = node_cfg["pos_offset"].as<float>();
        _offset = DEG2RAD ( _offset );

#if 0        
        if ( node_cfg["gear_ratio"] ) {
            std::vector<std::string> upg_par_names = std::initializer_list<std::string> {
                "Motor_Inertia", "Inv_Motor_Inertia", "Observer_Cut_Off",
                "Inv_Geared_Torque_Constant", "Geared_Torque_Constant",
                "Winding_Resistance", "Voltage_Feedforward", "BackEmf_Compensation"
            };
            for ( auto const par_name : upg_par_names ) {
                float upgPar;
                const YAML::Node & gr_cfg = root_cfg[node_cfg["gear_ratio"].as<std::string>()];
                upgPar = gr_cfg[par_name].as<float>();
                writeSDO_byname ( par_name.c_str(), upgPar );
                DPRINTF("writeSDO_byname ( %s, %f )\n", par_name.c_str(), upgPar);
            }
        }
        
        int16_t tmp_par;
        if ( get_ESC_type() == HI_PWR_AC_MC ) { tmp_par = 1; }
        else { tmp_par = 0; }
        writeSDO_byname ( "Analog_motor", tmp_par );
        DPRINTF("writeSDO_byname ( %s, %d )\n", "Analog_motor", tmp_par);
        
        std::vector<int> upg_rids = std::initializer_list<int> {
            32,33,41,42,43,44,45,46,51,52,53,54,55,56
        };
        auto found = std::find(std::begin(upg_rids), std::end(upg_rids), Joint_robot_id);
        if ( found != std::end(upg_rids) ) { tmp_par = 1; }
        else { tmp_par = 0; }
        writeSDO_byname ( "Has_Deflection_Encoder", tmp_par );
        DPRINTF("writeSDO_byname ( %s, %d )\n", "Has_Deflection_Encoder", tmp_par);
        
        set_flash_cmd_X ( this, FLASH_SAVE );

#else        
        std::vector<std::string> upg_par_names = std::initializer_list<std::string> {
            "Torque_lin_coeff",
            "Motor_Inertia", "Inv_Motor_Inertia", "Observer_Cut_Off",
            "Inv_Geared_Torque_Constant", "Geared_Torque_Constant",
            "Winding_Resistance", "Voltage_Feedforward", "BackEmf_Compensation"
        };
        float f_par;
        int16_t i16_par;
        readSDO_byname ( "Has_Deflection_Encoder" , i16_par );
        DPRINTF("readSDO_byname ( %s, %d )\n", "Has_Deflection_Encoder", i16_par);
        readSDO_byname ( "Analog_motor" , i16_par );
        DPRINTF("readSDO_byname ( %s, %d )\n", "Analog_motor", i16_par);
        for ( auto const par_name : upg_par_names ) {
            readSDO_byname ( par_name.c_str(), f_par );
            DPRINTF("readSDO_byname ( %s, %f )\n", par_name.c_str(), f_par);
        }

#endif        
        return EC_WRP_OK;
    }



}
}
}
#endif /* __IIT_ECAT_ADVR_MC_HIPWR_ESC_H__ */
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
