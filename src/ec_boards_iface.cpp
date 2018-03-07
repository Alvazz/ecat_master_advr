#include <iit/ecat/advr/ec_boards_iface.h>

#include <iostream>
#include <fstream>

using namespace iit::ecat::advr;


///////////////////////////////////////////////////////////////////////////////


Ec_Boards_ctrl::Ec_Boards_ctrl ( std::string config_file ) {

#ifdef __COBALT__
    rd_mtx = PTHREAD_MUTEX_INITIALIZER;
    wr_mtx = PTHREAD_MUTEX_INITIALIZER;
#endif

    std::ifstream fin ( config_file );
    if ( fin.fail() ) {
        DPRINTF ( "Can not open %s\n", config_file.c_str() );
        assert ( 0 );
    }

    root_cfg = YAML::LoadFile ( config_file );

    const YAML::Node& board_ctrl = root_cfg["ec_board_ctrl"];

    eth_if = board_ctrl["eth_iface"].as<std::string>();
    sync_cycle_time_ns = board_ctrl["sync_cycle_time_ns"].as<uint32_t>();
    sync_cycle_offset_ns = board_ctrl["sync_cycle_offset_ns"].as<uint32_t>();

}

Ec_Boards_ctrl::~Ec_Boards_ctrl() {

    bool do_power_off = false;
    std::cout << "~" << typeid ( this ).name() << std::endl;
    // std::shared_ptr slaves
    if ( root_cfg["ec_board_ctrl"]["power_off_boards"].as<bool>() == true ) {
        do_power_off = true;
    }
    shutdown(do_power_off);
}

int Ec_Boards_ctrl::init ( void ) {

    if ( iit::ecat::initialize ( eth_if.c_str() ) > 0 ) {
        factory_board();
        return EC_BOARD_OK;
    }
    return EC_BOARD_NOK;

}

int Ec_Boards_ctrl::shutdown ( bool do_power_off ) {

    // slave shared_ptr Dtor
    // destructs the owned object if no more shared_ptrs link to it 
    
    for ( auto const& item : slaves ) { DPRINTF(" shared_ptr use_count %ld\n", item.second.use_count()); }
    slaves.clear();
    iit::ecat::finalize ( do_power_off );

}

template <typename T>
void Ec_Boards_ctrl::make_board ( int ec_slave_idx ) {

    int16_t rid;
    
    T * slave = new T ( ec_slave[ec_slave_idx] );
    if ( slave->init ( root_cfg ) != EC_BOARD_OK ) {
        // skip this slave
        zombies[ec_slave_idx] = iit::ecat::ESCPtr ( slave );
        return;
    }
    slaves[ec_slave_idx] = iit::ecat::ESCPtr ( slave );
    
    rid = slave->get_robot_id();
    slave->print_info();
    if ( rid != -1 ) {
        rid2pos[rid] = ec_slave_idx;
        pos2rid[ec_slave_idx] = rid;
    }
    
}

void Ec_Boards_ctrl::factory_board ( void ) {

    int i;
    int16_t rid;
    slave_cnt = ec_slavecount;


    for ( i=1; i<=slave_cnt; i++ ) {

        ///////////////////////////////////////////////////
        // BigMotor and MediumMotor
        if ( ec_slave[i].eep_id == HI_PWR_AC_MC ||
             ec_slave[i].eep_id == HI_PWR_DC_MC ) {
            
            make_board<HpESC>(i);
        }
        ///////////////////////////////////////////////////
        // LowPwr Motor
        else if ( ec_slave[i].eep_id == LO_PWR_DC_MC ) {

            make_board<LpESC>(i);
        }
        ///////////////////////////////////////////////////
        // LowPwr Hand
        else if ( ec_slave[i].eep_id == LO_PWR_HAND_MC ) {

            make_board<LpHandESC>(i);
        }
        ///////////////////////////////////////////////////
        // Centauro AC
        else if ( ec_slave[i].eep_id == CENT_AC ) {

            make_board<CentAcESC>(i);
        }
        ///////////////////////////////////////////////////
        // FT6 Sensor
        else if ( ec_slave[i].eep_id == FT6 ) {

            make_board<Ft6ESC>(i);
        }
        ///////////////////////////////////////////////////
        // Foot Sensor
        else if ( ec_slave[i].eep_id == FOOT_SENSOR ) {

            make_board<FootSensor_16x8>(i);
        }
        ///////////////////////////////////////////////////
        // Foot Sensor 10x5
        else if ( ec_slave[i].eep_id == FOOT_SENS_10x5 ) {

            make_board<FootSensor_10x5>(i);
        }
        ///////////////////////////////////////////////////
        // Skin Sensor
        else if ( ec_slave[i].eep_id == SKIN_SENSOR ) {

            make_board<SkinSensor_8x3>(i);
        }
        ///////////////////////////////////////////////////
        // IMU VN Sensor
        else if ( ec_slave[i].eep_id == IMU_VECTORNAV ) {

            make_board<ImuVnESC>(i);
        }
        ///////////////////////////////////////////////////
        // Pow board
        else if ( ec_slave[i].eep_id == POW_BOARD ) {

            make_board<PowESC>(i);
        }
        ///////////////////////////////////////////////////
        // Pow F28M36 board
        else if ( ec_slave[i].eep_id == POW_F28M36_BOARD ) {

            make_board<PowF28M36ESC>(i);
        }
        ///////////////////////////////////////////////////
        // Pow coman board
        else if ( ec_slave[i].eep_id == POW_CMN_BOARD ) {

            make_board<PowComanESC>(i);
        }
        ///////////////////////////////////////////////////
        // Hubs
        else if ( ec_slave[i].eep_id == HUB ) {

            make_board<HubESC>(i);
        } else if ( ec_slave[i].eep_id == HUB_IO ) {

            make_board<HubIoESC>(i);
        }
        ///////////////////////////////////////////////////
        // Test
        else if ( ec_slave[i].eep_id & TEST_MASK  ) {

            make_board<TestESC>(i);
        }
        ///////////////////////////////////////////////////
        // Test
        else if ( ec_slave[i].eep_id = LXM32I  ) {

            make_board<LXM32iESC>(i);
        }
        ///////////////////////////////////////////////////
        else {

            DPRINTF ( "Warning product code %d not handled !!!\n", ec_slave[i].eep_id );
        }

    }

    iit::ecat::setExpectedSlaves ( slaves );

    if ( zombies.size() > 0 ) {
        DPRINTF ( "Warning you got %lu zombies !!!\n", zombies.size() );
        for ( auto it = zombies.begin(); it != zombies.end(); it++ ) {
            DPRINTF ( "\tpos %d ", it->first );
        }
        DPRINTF ( "\n" );
    }
}

int Ec_Boards_ctrl::configure_boards ( void ) {

    factory_board();
    
    return slaves.size();

}

int Ec_Boards_ctrl::set_operative ( void ) {

    expected_wkc = iit::ecat::operational ( sync_cycle_time_ns, sync_cycle_offset_ns );
    return expected_wkc;
}

int Ec_Boards_ctrl::set_pre_op ( void ) {

    return iit::ecat::pre_operational();
}

int Ec_Boards_ctrl::recv_from_slaves ( ec_timing_t &timing ) {

    /////////////////////////////////////////////
    // wait for cond_signal
    // ecat_thread sync with DC
    rd_LOCK();
    int ret = iit::ecat::recv_from_slaves ( timing );
    rd_UNLOCK();
    if ( ret < 0 ) {
        DPRINTF ( "fail recv_from_slaves\n" );
        return EC_BOARD_RECV_FAIL;
    }

    return EC_BOARD_OK;
}

int Ec_Boards_ctrl::send_to_slaves(bool write_slaves_pdo) {

    return iit::ecat::send_to_slaves(write_slaves_pdo);
}


static int esc_gpio_ll_wr ( uint16_t configadr, uint16_t gpio_val ) {

    int wc = ec_FPWR ( configadr, 0x0F10, sizeof ( gpio_val ), &gpio_val, EC_TIMEOUTRET3 );
    if ( wc <= 0 ) {
        DPRINTF ( "ERROR FPWR(%x, 0x0F10, %d)\n", configadr, gpio_val );
    }
    return wc;
}

/**
 * bit0 power_on
 * bit1 reset
 * bit2 boot
 */
int Ec_Boards_ctrl::update_board_firmware ( uint16_t slave_pos, std::string firmware, uint32_t passwd_firm, std::string mcu_info = "none" ) {

    int wc, ret = 0;
    char * ec_err_string;
    bool go_ahead = true;
    uint16_t configadr;
    uint16_t flash_cmd;
    uint16_t flash_cmd_ack = 0x0;
    uint16_t sub_idx;
    int size;
    int tries;
    char firm_ver[16];

    // all slaves in INIT state
    req_state_check ( 0, EC_STATE_INIT );

    EscWrapper * sWrp = slave_as_EscWrapper ( slave_pos );
    if ( ! sWrp ) {
        sWrp = slave_as_Zombie ( slave_pos );
        if ( ! sWrp ) {
            return 0;
        }
    }

    configadr = sWrp->get_configadr();

    // check slave type ... some uses ET1100 GPIO to force/release bootloader
    if ( esc_gpio_boot_set.find( sWrp->get_ESC_type() ) != esc_gpio_boot_set.end() ) {

        // pre-update
        if ( esc_gpio_ll_wr ( configadr, GPIO_PW_OFF ) <= 0 ) { return 0; }
        sleep ( 1 );
        if ( esc_gpio_ll_wr ( configadr, GPIO_PW_ON|GPIO_RESET|GPIO_BOOT ) <= 0 ) { return 0; }
        usleep ( 300000 );
        if ( esc_gpio_ll_wr ( configadr, GPIO_PW_ON|GPIO_BOOT ) <= 0 ) { return 0; }
        sleep ( 1 );

    } else {
        DPRINTF ( "Slave %d is NOT a XL or a MD motor\n", slave_pos );
    }

    // first boot state request is handled by application that jump to bootloader
    // we do NOT have a state change in the slave
    req_state_check ( slave_pos, EC_STATE_BOOT );

    sleep ( 3 );

    // second boot state request is handled by bootloader
    // now the slave should go in BOOT state
    if ( req_state_check ( slave_pos, EC_STATE_BOOT ) != EC_STATE_BOOT ) {
        DPRINTF ( "Slave %d not changed to BOOT state.\n", slave_pos );
        return 0;
    }

    if ( esc_gpio_boot_set.find( sWrp->get_ESC_type() ) != esc_gpio_boot_set.end() ) {

        // erase flash
        flash_cmd = 0x00EE;
        // fw_ver sub index
        sub_idx = 0x4;
        if ( mcu_info.compare("m3") == 0 ) { flash_cmd = 0x00E1; }
        if ( mcu_info.compare("c28") == 0 ) { flash_cmd = 0x00E2; sub_idx = 0x5; }

        memset ( ( void* ) firm_ver, 0, sizeof ( firm_ver ) );
        wc = ec_SDOread ( slave_pos, 0x8000, sub_idx, false, &size, &firm_ver, EC_TIMEOUTRXM * 30 );
        DPRINTF ( "Slave %d bl fw %s\n", slave_pos, firm_ver );

        // write sdo flash_cmd
        DPRINTF ( "erasing flash ...\n" );
        wc = ec_SDOwrite ( slave_pos, 0x8000, 0x1, false, sizeof ( flash_cmd ), &flash_cmd, EC_TIMEOUTRXM * 30 ); // 21 secs
        if ( wc <= 0 ) {
            DPRINTF ( "ERROR writing flash_cmd\n" );
            ec_err_string =  ec_elist2string();
            DPRINTF ( "Ec_error : %s\n", ec_err_string );
            go_ahead = false;
        } else {
// to test
#if 0
            flash_cmd_ack = 0x0;
            tries = 30;
            while ( tries -- ) {
                sleep ( 1 );
                // read flash_cmd_ack
                wc = ec_SDOread ( slave_pos, 0x8000, 0x2, false, &size, &flash_cmd_ack, EC_TIMEOUTRXM * 30 );
                DPRINTF ( "Slave %d wc %d flash_cmd_ack 0x%04X\n", slave_pos, wc, flash_cmd_ack );
                if ( wc <= 0 ) {
                    DPRINTF ( "ERROR reading flash_cmd_ack\n" );
                    ec_err_string =  ec_elist2string();
                    DPRINTF ( "Ec_error : %s\n", ec_err_string );
                    go_ahead = false;
                } else if ( flash_cmd_ack != CTRL_CMD_DONE ) {
                    DPRINTF ( "ERROR erasing flash\n" );
                    go_ahead = false;
                } else {
                    //
                    break;
                }
            }
#endif
        }
    }

    if ( go_ahead ) {
        ret = send_file ( slave_pos, firmware, passwd_firm );
    }

    // post-update ... restore
    if ( esc_gpio_ll_wr ( configadr, GPIO_PW_OFF ) <= 0 ) { return 0; }
    sleep ( 1 );
    if ( esc_gpio_ll_wr ( configadr, GPIO_PW_ON|GPIO_RESET ) <= 0 ) { return 0; }
    usleep ( 300000 );
    if ( esc_gpio_ll_wr ( configadr, GPIO_PW_ON ) <= 0 ) { return 0; }

    //INIT state request is handled by bootloader that jump to application that start from INIT
    req_state_check ( slave_pos, EC_STATE_INIT );

    return go_ahead && ( ret > 0 ) ;
}


int Ec_Boards_ctrl::upload_flash ( uint16_t slave_pos, std::string bin_file, uint32_t bin_passwd, uint32_t size_byte, std::string save_as ) {

    int wc, ret = 0;
    char * ec_err_string;
    uint16_t configadr;
    int size;
    char firm_ver[16];

    // all slaves in INIT state
    req_state_check ( 0, EC_STATE_INIT );

    EscWrapper * sWrp = slave_as_EscWrapper ( slave_pos );
    if ( ! sWrp ) {
        sWrp = slave_as_Zombie ( slave_pos );
        if ( ! sWrp ) {
            return 0;
        }
    }

    configadr = sWrp->get_configadr();

    // check slave type ... some uses ET1100 GPIO to force/release bootloader
    if ( esc_gpio_boot_set.find( sWrp->get_ESC_type() ) != esc_gpio_boot_set.end() ) {

        // pre-update
        if ( esc_gpio_ll_wr ( configadr, GPIO_PW_OFF ) <= 0 ) { return 0; }
        sleep ( 1 );
        if ( esc_gpio_ll_wr ( configadr, GPIO_PW_ON|GPIO_RESET|GPIO_BOOT ) <= 0 ) { return 0; }
        usleep ( 300000 );
        if ( esc_gpio_ll_wr ( configadr, GPIO_PW_ON|GPIO_BOOT ) <= 0 ) { return 0; }
        sleep ( 1 );

    } else {
        DPRINTF ( "Slave %d is NOT a XL or a MD motor\n", slave_pos );
    }

    // first boot state request is handled by application that jump to bootloader
    // we do NOT have a state change in the slave
    req_state_check ( slave_pos, EC_STATE_BOOT );

    sleep ( 3 );

    // second boot state request is handled by bootloader
    // now the slave should go in BOOT state
    if ( req_state_check ( slave_pos, EC_STATE_BOOT ) != EC_STATE_BOOT ) {
        DPRINTF ( "Slave %d not changed to BOOT state.\n", slave_pos );
        return 0;
    }

    memset ( ( void* ) firm_ver, 0, sizeof ( firm_ver ) );
    wc = ec_SDOread ( slave_pos, 0x8000, 0x4, false, &size, &firm_ver, EC_TIMEOUTRXM );
    DPRINTF ( "Slave %d bl fw %s\n", slave_pos, firm_ver );

    //int iit::ecat::recv_file(uint16_t slave, std::string filename, uint32_t passwd_firm, uint32_t byte_count, std::string save_as);
    ret = recv_file ( slave_pos, bin_file, bin_passwd, size_byte, save_as );

    // post-update ... restore
    if ( esc_gpio_ll_wr ( configadr, GPIO_PW_OFF ) <= 0 ) { return 0; }
    sleep ( 1 );
    if ( esc_gpio_ll_wr ( configadr, GPIO_PW_ON|GPIO_RESET ) <= 0 ) { return 0; }
    usleep ( 300000 );
    if ( esc_gpio_ll_wr ( configadr, GPIO_PW_ON ) <= 0 ) { return 0; }

    //INIT state request is handled by bootloader that jump to application that start from INIT
    req_state_check ( slave_pos, EC_STATE_INIT );

    return ret;
}





// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
