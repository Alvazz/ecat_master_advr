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

#include <iit/ecat/advr/lxm32i_reg.h>

#include <protobuf/ecat_pdo.pb.h>

#include <map>
#include <iostream>

namespace iit {
namespace ecat {
namespace advr {

#define RX_ELEM_CNT 6
#define TX_ELEM_CNT 3  

    
#define _MK_STR(a) static const std::string a ( #a )

//static const std::string _DCOMstatus( "_DCOMstatus");
_MK_STR(_DCOMstatus);
_MK_STR(_DCOMopmd_act);
_MK_STR(_p_act);
_MK_STR(_p_dif);
_MK_STR(_tq_act);
_MK_STR(_LastError);
_MK_STR(_IO_act);

_MK_STR(DCOMcontrol);
_MK_STR(DCOMopmode);
_MK_STR(PPp_target);
_MK_STR(PPv_target);
_MK_STR(RAMP_v_acc);
_MK_STR(RAMP_v_dec);
_MK_STR(PVv_target);
_MK_STR(PTtq_target);
_MK_STR(IO_DQ_set);
_MK_STR(JOGactivate);

_MK_STR(CompParSyncMot); 
_MK_STR(MOD_enable); 
_MK_STR(LimQStopReact); 
_MK_STR(IOsigRespOfPS); 
_MK_STR(CTRL1_KFPp); 
_MK_STR(CTRL2_KFPp);
_MK_STR(ScalePOSdenom);
_MK_STR(ScalePOSnum);

_MK_STR(HMmethod);
_MK_STR(HMp_setP);

_MK_STR(RxPdoMapCnt);
_MK_STR(RxPdoMap);
_MK_STR(RxElemCnt);
_MK_STR(RPdo1);
_MK_STR(RPdo2);
_MK_STR(RPdo3);
_MK_STR(RPdo4);
_MK_STR(RPdo5);
_MK_STR(RPdo6);
_MK_STR(TxPdoMapCnt);
_MK_STR(TxPdoMap);
_MK_STR(TxElemCnt);
_MK_STR(TPdo1);
_MK_STR(TPdo2);
_MK_STR(TPdo3);
_MK_STR(TPdo4);
_MK_STR(TPdo5);

std::vector<std::string> const rpdos = {
    RPdo1, RPdo2, RPdo3, RPdo4, RPdo5, RPdo6
};
std::vector<std::string> const tpdos = {
    TPdo1, TPdo2, TPdo3,
    //TPdo4, TPdo5,
};

std::vector<std::string> const rd_pdos = { 
    _DCOMstatus, _DCOMopmd_act, _p_act,
    //_p_dif, _tq_act  
};

std::vector<std::string> const wr_pdos = {
    DCOMcontrol, DCOMopmode,
    PPp_target, PPv_target, RAMP_v_acc, RAMP_v_dec,
};



std::vector<std::string> const rd_sdos = { 
    RxPdoMapCnt, RxPdoMap, TxPdoMapCnt, TxPdoMap, 
//     CompParSyncMot, MOD_enable, LimQStopReact,
//     IOsigRespOfPS, CTRL1_KFPp, CTRL2_KFPp,
    ScalePOSdenom, ScalePOSnum,
    HMmethod, HMp_setP,  
};

std::vector<std::string> const wr_sdos= { };



        


class LXM32iEscPdoTypes {
   
public:
    
    // TX  slave_input -- master output
    struct pdo_tx {
        dcom_control_t  DCOMcontrol;
        uint8_t         DCOMopmode;
        int32_t         PPp_target;
        uint32_t        PPv_target;
        uint32_t        RAMP_v_acc;
        uint32_t        RAMP_v_dec;
        // 19 bytes
        //int32_t         PVv_target;
        //int16_t         PTtq_target;
        // 25 bytes
        //uint16_t        IO_DQ_set;
        //uint16_t        JOGactivate;
        // 29  bytes
        
    std::ostream& dump ( std::ostream& os, const std::string delim ) const {
        DUMP( os, PPp_target, delim );
        DUMP( os, PPv_target, delim );
        DUMP( os, RAMP_v_acc, delim );
        DUMP( os, RAMP_v_dec, delim );
        DUMP( os, PVv_target, delim );
        DUMP( os, PTtq_target, delim );
        //os << std::endl;
        return os;
    }
        
    }  __attribute__ ( ( __packed__ ) );

    // RX  slave_output -- master input
    struct pdo_rx {
        dcom_status_t   _DCOMstatus;
        uint8_t         _DCOMopmd_act;
        int32_t         _p_act;
        // 7 bytes
        //int32_t         _p_dif;
        //int16_t         _tq_act;
        // 13 bytes
        //uint16_t        _LastError;
        //uint16_t        _IO_act;
        // 17 bytes
        
    std::ostream& dump ( std::ostream& os, const std::string delim ) const {
        _DCOMstatus.dcom_status.dump(os,delim);
        os << "0x" << std::hex << _DCOMstatus.all << std::dec << delim;
        os << "_DCOMopmd_act=" << (int)_DCOMopmd_act << std::dec << delim;
        DUMP( os, _p_act, delim );
//        DUMP( os, _p_dif, delim );
//        DUMP( os, _tq_act, delim );
//         DUMP( os, _LastError, delim );
//         DUMP( os, _IO_act, delim );
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
    }

    }  __attribute__ ( ( __packed__ ) );
};


struct LXM32iEscSdoTypes {
    
    uint8_t     rxPdoMapCnt;
    uint16_t    rxPdoMap;
    uint8_t     rxElemCnt;
    uint32_t    rxMap[RX_ELEM_CNT];
    uint8_t     txPdoMapCnt;
    uint16_t    txPdoMap;
    uint8_t     txElemCnt;
    uint32_t    txMap[TX_ELEM_CNT];

//     int16_t     CompParSyncMot; 
//     uint16_t    MOD_enable; 
//     int16_t     LimQStopReact; 
//     uint16_t    IOsigRespOfPS; 
//     uint16_t    CTRL1_KFPp; 
//     uint16_t    CTRL2_KFPp; 

    int32_t     ScalePOSdenom;
    int32_t     ScalePOSnum;
    
    int8_t      HMmethod;
    int32_t     HMp_setP;

    
   std::ostream& dump ( std::ostream& os, const std::string delim ) const {
       int j;
       os << "struct LXM32iEscSdoTypes : ";
       os << "rxPdoMapCnt="  << (int)rxPdoMapCnt << delim;
       os << "rxPdoMap="     << "0x" << std::hex << rxPdoMap << std::dec << delim;
       os << "rxElemCnt="    << (int)rxElemCnt << delim;
       for(j=0;j<RX_ELEM_CNT;j++) {  os << j << "_0x" << std::hex << rxMap[j] << std::dec << delim; }
       os << "txPdoMapCnt="  << (int)txPdoMapCnt << delim;
       os << "txPdoMap="     << "0x" << std::hex << txPdoMap << std::dec << delim;
       os << "txElemCnt="    << (int)txElemCnt << delim;
       for(j=0;j<TX_ELEM_CNT;j++) { os << j << "_0x" << std::hex << txMap[j] << std::dec << delim; }
//        os << CompParSyncMot << delim;
//        os << MOD_enable << delim;
//        os << LimQStopReact << delim;
//        os << IOsigRespOfPS << delim;
//        os << CTRL1_KFPp << delim;
//        os << CTRL2_KFPp << delim;
       os << ScalePOSdenom << delim;
       os << ScalePOSnum << delim;
       os << (int)HMmethod << delim;
       os << HMp_setP << delim;
       return os;
    }

};


struct LXM32iLogTypes {

    uint64_t    ts;     // ns
    int32_t     pRef;
    int32_t     pAct;
    int32_t     pDif;
    int16_t     tqAct;
        
    void fprint ( FILE *fp ) {
        fprintf ( fp, "%lu\t%d\t%d\t%d\t%d", ts, pRef, pAct, pDif, tqAct );
    }
    int sprint ( char *buff, size_t size ) {
        int l = snprintf ( buff, size, "%lu\t%d\t%d\t%d\t%d", ts, pRef, pAct, pDif, tqAct );
        return l;
    }
};


inline std::ostream& operator<< (std::ostream& os, const LXM32iEscSdoTypes& sdo ) {
    return sdo.dump(os," ");
}



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

    int user_loop( const char &c );

    int32_t set_pos_target( float pos );
    
    std::vector<double> trj_Xs, trj_Ys;
    
private:
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
        log.pAct  = rx_pdo._p_act;
        log.pRef = tx_pdo.PPp_target;
        //log.pDif = rx_pdo._p_dif;
        //log.tqAct = rx_pdo._tq_act;
        push_back ( log );
    }
 
//     xddp_write ( rx_pdo );

}

inline int16_t LXM32iESC::get_robot_id() {
    //return sdo.sensor_robot_id;
    return position;
}

inline void LXM32iESC::print_info ( void ) {
}



typedef std::map<int, LXM32iESC*>  LXM32iSlavesMap;




}
}
}
#endif /* __IIT_ECAT_ADVR_SKIN_SENSOR_ESC_H__ */
