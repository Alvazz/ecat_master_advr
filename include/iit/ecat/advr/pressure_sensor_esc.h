/*
 * skin_sensor_esc.h
 *
 *  based on TI TM4C123x - Tiva Microcontroller MCUs
 *  
 *  /http://www.ti.com/product/tm4c123ah6pm
 *
 *  Created on: Sept 2017
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_PRESSURE_SENSOR_ESC_H__
#define __IIT_ECAT_ADVR_PRESSURE_SENSOR_ESC_H__

#include <iit/ecat/slave_wrapper.h>
#include <iit/ecat/advr/esc.h>
#include <iit/ecat/advr/log_esc.h>
#include <iit/ecat/advr/pipes.h>
#include <iit/ecat/utils.h>
#include <protobuf/ecat_pdo.pb.h>

#include <map>
#include <iostream>

//#define SENSOR_NUMBER 16*8
//#define SENSOR_NUMBER 10*5
//#define SENSOR_NUMBER  8*8


namespace iit {
namespace ecat {
namespace advr {

// 0x6000
_MK_STR(force);
_MK_STR(fault);
_MK_STR(rtt);


// 0x7000
_MK_STR(ts);

// 0x8000
_MK_STR(BlockControl);
_MK_STR(NumAvSamples);
_MK_STR(ConfigFlags);
_MK_STR(Sensor_number);
_MK_STR(Sensor_robot_id);


// 0x8001
_MK_STR(fw_ver);
_MK_STR(ack_board_faults);
_MK_STR(flash_params_cmd);
_MK_STR(flash_params_cmd_ack);


int  get_SDOs_size( void );
void copy_source_SDOs(iit::ecat::objd_t * dest);


template <int _Rows, int _Cols>
struct PressSensEscPdoTypes {
    
    static int const rows = _Rows;
    static int const cols = _Cols;
    static int const fxy_size = rows*cols;
    
    // TX  slave_input -- master output
    struct pdo_tx {
        uint16_t    ts;
    }  __attribute__ ( ( __packed__ ) );

    // RX  slave_output -- master input
    struct pdo_rx {
        uint8_t    forceXY[fxy_size];
        uint16_t   fault;
        uint16_t   rtt;                
        
        std::ostream& dump ( std::ostream& os, const std::string delim ) const {
            for (int i=0; i<fxy_size; i++) {
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
            for(int i = 0; i<fxy_size; i++) {
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
            pb_rx_pdo.set_type(iit::advr::Ec_slave_pdo::RX_FOOT_SENS);
            // footWalkman_rx_pdo
            pb_rx_pdo.mutable_footwalkman_rx_pdo()->set_fault(fault);
            pb_rx_pdo.mutable_footwalkman_rx_pdo()->set_rtt(rtt);
            //for (int i=0; i<fxy_size; i++) { pb_rx_pdo.mutable_footwalkman_rx_pdo()->set_forcexy(i,(unsigned)forceXY[i]); }
            //pb_rx_pdo.mutable_footwalkman_rx_pdo()->clear_forcexy();
            for (int i=0; i<fxy_size; i++) { pb_rx_pdo.mutable_footwalkman_rx_pdo()->add_forcexy((unsigned)forceXY[i]); }
            //std::cout << "FootXY " << pb_rx_pdo.mutable_footwalkman_rx_pdo()->forcexy_size() << std::endl;
            pb_rx_pdo.SerializeToString(pb_str);
        }

    }  __attribute__ ( ( __packed__ ) );
};

struct PressSensEscSdoTypes {

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

template <typename T>
struct PressSensLogTypes {

    uint64_t    ts;     // ns
    T           rx_pdo;

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


template <int _Rows, int _Cols>
class PressSensESC :
    public BasicEscWrapper<PressSensEscPdoTypes<_Rows,_Cols>, PressSensEscSdoTypes>,
    public PDO_log<PressSensLogTypes<typename PressSensEscPdoTypes<_Rows,_Cols>::pdo_rx>>,
    public XDDP_pipe
{
public:
    typedef PressSensEscPdoTypes<_Rows,_Cols>                           BasePdoTypes;
    typedef BasicEscWrapper<BasePdoTypes, PressSensEscSdoTypes>         Base;
    typedef PDO_log<PressSensLogTypes<typename BasePdoTypes::pdo_rx>>   Log;

public:
    PressSensESC<_Rows,_Cols>( const ec_slavet& slave_descriptor );
    virtual ~PressSensESC ( void );

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


#define TEMPL template <int _Rows, int _Cols>
#define CLASS PressSensESC<_Rows,_Cols>
#define SIGNATURE(type) TEMPL inline type CLASS


TEMPL
CLASS::PressSensESC( const ec_slavet& slave_descriptor ) :
    Base ( slave_descriptor ),
    Log ( std::string ( "/tmp/FootSensorESC_pos"+std::to_string ( Base::position ) +"_log.txt" ),DEFAULT_LOG_SIZE ),
    XDDP_pipe ()
    { }

TEMPL
CLASS::~PressSensESC ( void ) {
    // only free strdup'ed name
    for(int j=0; j<BasePdoTypes::fxy_size; j++) {
        free((void*)SDOs[j].name);
    }
    delete [] SDOs;
    DPRINTF ( "~%s pos %d\n", typeid ( this ).name(), Base::position );
    print_stat ( s_rtt );
}

SIGNATURE(const objd_t *)::get_SDOs() {
    return SDOs;
}

SIGNATURE(uint32_t)::get_ESC_type() {
    if ( BasePdoTypes::fxy_size == 16*8 ) return FOOT_SENSOR;
    if ( BasePdoTypes::fxy_size == 10*5 ) return FOOT_SENS_10x5;
    if ( BasePdoTypes::fxy_size ==  8*3 ) return SKIN_SENSOR;
    return NO_TYPE;
}


SIGNATURE(void)::on_writePDO ( void ) {
    Base::tx_pdo.ts = ( uint16_t ) ( get_time_ns() /1000 );
}

SIGNATURE(void)::on_readPDO ( void ) {

    if ( Base::rx_pdo.rtt ) {
        Base::rx_pdo.rtt = ( uint16_t ) ( get_time_ns() /1000 ) - Base::rx_pdo.rtt;
        s_rtt ( Base::rx_pdo.rtt );
    }

    if ( Base::rx_pdo.fault ) {
        handle_fault();
    } else {
        // clean any previuos fault ack !!
        //tx_pdo.fault_ack = 0;
    }

    if ( Log::_start_log ) {
        typename Log::log_t log;
        memcpy(log.rx_pdo.forceXY, Base::rx_pdo.forceXY, sizeof(Base::rx_pdo.forceXY));
        log.rx_pdo.fault       = Base::rx_pdo.fault;
        log.rx_pdo.rtt         = Base::rx_pdo.rtt;
        Log::push_back ( log );
    }
    
    xddp_write ( Base::rx_pdo );

}

SIGNATURE(void)::init_SDOs ( void ) {

    int objd_num, i, subidx;
    const objd_t force_entry = {0x6000, 0x1, DTYPE_UNSIGNED8, 8, ATYPE_RO, force.c_str(), 0};
    objd_t * tmp_obj;
    
    //objd_num = sizeof ( source_SDOs ) /sizeof ( objd_t );
    objd_num = get_SDOs_size() + BasePdoTypes::fxy_size;
    
    SDOs = new objd_t [objd_num];

    //memcpy ( ( void* ) SDOs, source_SDOs, sizeof ( source_SDOs ) );
    copy_source_SDOs( SDOs + BasePdoTypes::fxy_size );
    
    // 0x6000
    subidx = 1;
    //for(j = 0; j < BasePdoTypes::fxy_size; j++) {
    for(int r = 0; r < BasePdoTypes::rows; r++) {
        for(int c = 0; c < BasePdoTypes::cols; c++) {
            tmp_obj = &SDOs[i++];
            memcpy(tmp_obj, &force_entry, sizeof(objd_t));
            tmp_obj->name = strdup(std::string("force"+std::to_string(r+1)+char('A'+c)).c_str());
            tmp_obj->subindex = subidx;
            tmp_obj->data = ( void* ) &Base::rx_pdo.forceXY[subidx-1];
            subidx++;
        }
    }
    tmp_obj = &SDOs[i++];
    tmp_obj->subindex = subidx;
    tmp_obj->data = ( void* ) &Base::rx_pdo.fault;
    tmp_obj = &SDOs[i++];
    tmp_obj->subindex = subidx+1;
    tmp_obj->data = ( void* ) &Base::rx_pdo.rtt;

    // 0x7000
    SDOs[i++].data = ( void* ) &Base::tx_pdo.ts;
    // 0x8000
    SDOs[i++].data = ( void* ) &Base::sdo.Block_control;
    SDOs[i++].data = ( void* ) &Base::sdo.NumAvSamples;
    SDOs[i++].data = ( void* ) &Base::sdo.ConfigFlags;
    SDOs[i++].data = ( void* ) &Base::sdo.sensor_number;
    SDOs[i++].data = ( void* ) &Base::sdo.sensor_robot_id;
    // 0x8001
    SDOs[i++].data = ( void* ) &Base::sdo.firmware_version;
    SDOs[i++].data = ( void* ) &Base::sdo.ack_board_fault;
    SDOs[i++].data = ( void* ) &Base::sdo.flash_params_cmd;
    SDOs[i++].data = ( void* ) &Base::sdo.flash_params_cmd_ack;

    SDOs[i++].data = 0;

    DPRINTF("%d %d %d\n", objd_num , i, BasePdoTypes::fxy_size);
    assert ( objd_num >= i );
}


SIGNATURE(int16_t)::get_robot_id() {
    return Base::sdo.sensor_robot_id;
}

SIGNATURE(void)::print_info ( void ) {
    DPRINTF ( "\tSensor id %d\tSensor robot id %d\n", Base::sdo.sensor_number, Base::sdo.sensor_robot_id );
    DPRINTF ( "\tfw_ver %s\n", Base::sdo.firmware_version );
}

SIGNATURE(int)::init ( const YAML::Node & root_cfg ) {

    std::string robot_name("void");
    try {
        robot_name = root_cfg["ec_boards_base"]["robot_name"].as<std::string>();
    } catch ( YAML::Exception &e ) {
    }

    try {
        init_SDOs();
        Base::init_sdo_lookup(true);
        Base::readSDO_byname ( "Sensor_robot_id" );

    } catch ( EscWrpError &e ) {

        DPRINTF ( "Catch Exception in %s ... %s\n", __PRETTY_FUNCTION__, e.what() );
        return EC_BOARD_INIT_SDO_FAIL;
    }

    // set filename with robot_id
    Log::log_filename = std::string ( "/tmp/PressSensESC_"+std::to_string ( Base::sdo.sensor_robot_id ) +"_log.txt" );

    // we log when receive PDOs
    Log::start_log ( true );
    
    // set use pipe variable NOTE true by default
    if(root_cfg["ec_board_ctrl"]["use_pipes"]) {
        Base::use_pipes = root_cfg["ec_board_ctrl"]["use_pipes"].as<bool>();
    }

    if ( Base::use_pipes ) {
        XDDP_pipe::init (robot_name+"@PressSens_id_"+std::to_string ( get_robot_id() ) );
    }
    
    return EC_BOARD_OK;

}

SIGNATURE(void)::handle_fault ( void ) {

    fault_t fault;
    fault.all = Base::rx_pdo.fault;
    //fault.bit.
    //ack_faults_X(this, fault.all);

}


// walkman
typedef PressSensESC<16,8>  FootSensor_16x8;
// cogimon
typedef PressSensESC<10,5>  FootSensor_10x5;
// skin
typedef PressSensESC<8,3>   SkinSensor_8x3;

typedef std::map<int, FootSensor_16x8*>  FootSensor_16x8_SlavesMap;
typedef std::map<int, FootSensor_10x5*>  FootSensor_10x5_SlavesMap;
typedef std::map<int, SkinSensor_8x3*>   SkinSensor_8x3_SlavesMap;


#undef TEMPL
#undef CLASS
#undef SIGNATURE


}
}
}
#endif /* __IIT_ECAT_ADVR_FOOT_SENSOR_ESC_H__ */
