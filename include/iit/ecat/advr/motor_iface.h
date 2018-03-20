/*
 *
 *  Created on: Dec, 2014
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_MOTOR_H__
#define __IIT_ECAT_ADVR_MOTOR_H__

#include <iit/ecat/advr/esc.h>

namespace iit {
namespace ecat {
namespace advr {

class EcBoardsError;
class HpESC;
class LpESC;
class CentAcESC;

template <typename MotorPdoTypes>
class AbsMotor {
public:
    typedef typename MotorPdoTypes::pdo_rx    motor_pdo_rx_t;
    typedef typename MotorPdoTypes::pdo_tx    motor_pdo_tx_t;

private:
    template <class C, typename T>
    int writeSDO_impl ( std::string const & name, T value ) {
        C *c = dynamic_cast<C*> ( this );
        if ( !c ) {
            return EC_WRP_NOK;
        }
        return c->writeSDO_byname ( name.c_str(), value );
    }

    template <class C, typename T>
    int readSDO_impl ( std::string const & name, T & value ) {
        C *c = dynamic_cast<C*> ( this );
        if ( !c ) {
            return EC_WRP_NOK;
        }
        return c->readSDO_byname ( name.c_str(), value );
    }

    template <class C, typename T>
    int getSDO_impl(std::string const & name, T & value ) {
        C *c = dynamic_cast<C*>(this);
        if (!c) {
            return EC_WRP_NOK;
        }
        return c->getSDO_byname( name.c_str(), value );
    }

    std::string _control_mode;

public:
    
    const std::string& get_control_mode() const { return _control_mode; }
    void set_control_mode(const std::string& ctrl_mode) { _control_mode = ctrl_mode; }
    
    template<typename T>
    int writeSDO ( std::string const & name, T value ) {
        
        if ( get_ESC_type() == HI_PWR_AC_MC || get_ESC_type() == HI_PWR_DC_MC ) { 
            return writeSDO_impl<HpESC> ( name, value );

        } else if ( get_ESC_type() == LO_PWR_DC_MC ) {
            return writeSDO_impl<LpESC> ( name, value );
            
        } else if ( get_ESC_type() == CENT_AC ) {
            return writeSDO_impl<CentAcESC> ( name, value );
            
        } else {
            throw EcBoardsError ( EC_BOARD_NOK, "writeSDO_impl" );
        }


        return EC_WRP_NOK;
    }

    template<typename T>
    int readSDO ( std::string const & name, T & value ) {
        
        if ( get_ESC_type() == HI_PWR_AC_MC || get_ESC_type() == HI_PWR_DC_MC ) { 
            return readSDO_impl<HpESC> ( name, value );
                    
        } else if ( get_ESC_type() == LO_PWR_DC_MC ) {
            return readSDO_impl<LpESC> ( name, value );
            
        } else if ( get_ESC_type() == CENT_AC ) {
            return readSDO_impl<CentAcESC> ( name, value );
            
        } else {
            throw EcBoardsError ( EC_BOARD_NOK, "readSDO_impl" );
        }

        return EC_WRP_NOK;
    }

    template<typename T>
    int getSDO(std::string const & name, T & value ) {
        if ( get_ESC_type() == HI_PWR_AC_MC || get_ESC_type() == HI_PWR_DC_MC ) { 
            return getSDO_impl<HpESC> ( name, value );
                    
        } else if ( get_ESC_type() == LO_PWR_DC_MC ) {
            return getSDO_impl<LpESC> ( name, value );
            
        } else if ( get_ESC_type() == CENT_AC ) {
            return getSDO_impl<CentAcESC> ( name, value );
            
        } else {
            throw EcBoardsError ( EC_BOARD_NOK, "readSDO_impl" );
        }

        return EC_WRP_NOK;
    }

    virtual int init( const YAML::Node & ) = 0;
    int start ( void );
    virtual int start ( int controller_type ) = 0;
    virtual int start ( int controller_type, const std::vector<float> &gains ) = 0;
    virtual int stop ( void ) = 0;

    virtual const motor_pdo_rx_t & getRxPDO() const = 0;
    virtual const motor_pdo_tx_t & getTxPDO() const = 0;
    virtual void setTxPDO ( const motor_pdo_tx_t & ) = 0;

    virtual int set_posRef ( float joint_pos ) = 0;
    virtual int set_velRef ( float joint_vel ) = 0;
    virtual int set_torRef ( float joint_tor ) = 0;
    //virtual int set_torOffs ( float tor_offs ) = 0;
    
    //virtual int set_posGainP ( float p_gain )  = 0;
    //virtual int set_posGainI ( float i_gain )  = 0;
    //virtual int set_posGainD ( float d_gain )  = 0;

    virtual int move_to ( float pos, float step ) = 0;

    //virtual int get_pos(float &joint_pos)   = 0;
    //virtual int get_posGainP(float &p_gain) = 0;
    //virtual int get_posGainI(float &i_gain) = 0;
    //virtual int get_posGainD(float &d_gain) = 0;

    //virtual void handle_fault(void) = 0;


    //void set_state(ec_state state) { _actual_state = state; }

    virtual uint32_t get_ESC_type(void) = 0;
    
protected:

    //ec_state     _actual_state;
};

typedef AbsMotor<McEscPdoTypes> Motor;

template <typename MotorPdoTypes>
inline int AbsMotor<MotorPdoTypes>::start(void) {
    
    auto ctrl_str = get_control_mode();
    try {
        uint32_t ctrl_code = controller_type_map.at(ctrl_str);
        //DPRINTF("ctrl_code %d %s\n", ctrl_code, ctrl_str);
        // 00_idle go ahead without start controller
        if ( ! ctrl_code ) return EC_BOARD_OK;
        else return start( ctrl_code );
    } catch (const std::out_of_range &e ) {
        // ctrl_str invalid
        return EC_BOARD_NOK;
    }
};

}
}
}

#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
