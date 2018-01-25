#ifndef __IIT_ECAT_ADVR_LXM32I_REG_H__
#define __IIT_ECAT_ADVR_LXM32I_REG_H__


#define DUMPWNAME(os, name, a, d) \
    do { (os) << (name) << "=" << (a) << d; } while(0)
        
#define DUMP(os, a, d) DUMPWNAME((os), #a, (a), (d))  

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


#endif