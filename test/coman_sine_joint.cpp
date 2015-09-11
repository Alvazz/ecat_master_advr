#include <signal.h>
#include <sys/mman.h>
#include <execinfo.h>
#include <stdio.h>
#include <stdint.h>
#include <yaml-cpp/yaml.h>

#ifdef __XENO__
    #include <rtdk.h>
#endif

#include <memory>

#include <iit/ecat/advr/ec_boards_iface.h>

#define MID_POS(m,M)    (m+(M-m)/2)

using namespace iit::ecat::advr;
using namespace iit::ecat;

extern int run_loop;
extern int main_common(void);

Ec_Boards_ctrl * ec_boards_ctrl; 

std::vector<int> right_arm = {

    
    Robot_IDs::RA_SH_1,
    Robot_IDs::RA_SH_2,
    Robot_IDs::RA_SH_3,
    Robot_IDs::RA_EL,
    Robot_IDs::RA_WR_1,
    Robot_IDs::RA_WR_2,
    Robot_IDs::RA_WR_3,
    //Robot_IDs::RA_FT,
    //Robot_IDs::RA_HA,

};

std::vector<int> left_arm = {

    Robot_IDs::LA_SH_1,
    Robot_IDs::LA_SH_2,
    Robot_IDs::LA_SH_3,
    Robot_IDs::LA_EL,
    Robot_IDs::LA_WR_1,
    Robot_IDs::LA_WR_2,
    Robot_IDs::LA_WR_3,
    //Robot_IDs::LA_FT,
    //Robot_IDs::LA_HA, 
};

std::vector<int> left_leg = {

    //Robot_IDs::LL_H_Y,
    Robot_IDs::LL_H_R,
    Robot_IDs::LL_H_P,
    Robot_IDs::LL_K,
    Robot_IDs::LL_A_P,
    Robot_IDs::LL_A_R,
    //Robot_IDs::LL_FT,
};


std::vector<int> body = {

    Robot_IDs::WAIST_R,
    Robot_IDs::WAIST_Y,
    Robot_IDs::WAIST_P,

    Robot_IDs::RA_SH_2,
    Robot_IDs::RA_SH_1,
    Robot_IDs::RA_SH_3,
    Robot_IDs::RA_EL,
    Robot_IDs::RA_WR_1,
    Robot_IDs::RA_WR_2,
    Robot_IDs::RA_WR_3,
    //Robot_IDs::RA_FT,
    Robot_IDs::RA_HA,
    
    Robot_IDs::LA_SH_2,
    Robot_IDs::LA_SH_1,
    Robot_IDs::LA_SH_3,
    Robot_IDs::LA_EL,
    Robot_IDs::LA_WR_1,
    Robot_IDs::LA_WR_2,
    Robot_IDs::LA_WR_3,
    //Robot_IDs::LA_FT,
    Robot_IDs::LA_HA,     
    
    Robot_IDs::RL_H_R,
    Robot_IDs::LL_H_R,

    Robot_IDs::RL_H_Y,
    Robot_IDs::LL_H_Y,    
    
    Robot_IDs::RL_H_P,
    Robot_IDs::LL_H_P,
    
    Robot_IDs::RL_K,
    Robot_IDs::LL_K,
    
    Robot_IDs::RL_A_P,
    Robot_IDs::LL_A_P,
    
    Robot_IDs::RL_A_R,
    Robot_IDs::LL_A_R,
    
    //Robot_IDs::RL_FT,
    //Robot_IDs::LL_FT,

    
};


//std::vector<int> motors = body; 
//std::vector<int> motors = { 1000 }; 
std::vector<int> motors = { 1 }; 

std::map<int,float> home;
std::map<int,float> start_pos;
    
int main(int argc, char **argv)
{
    main_common();

    if ( argc != 2) {
    printf("Usage: %s config.yaml\n", argv[0]);
        return 0;
    }


    Ec_Boards_ctrl * ec_boards_ctrl;

    PowEscPdoTypes::pdo_rx pow_pdo_rx;
    McEscPdoTypes::pdo_rx mc_pdo_rx;
    McEscPdoTypes::pdo_tx mc_pdo_tx;
    char buffer[1024];
    std::map<int, PowESC*> pow_boards;

    ///////////////////////////////////////////////////////////////////////////

    ec_boards_ctrl = new Ec_Boards_ctrl(argv[1]); 

    if ( ec_boards_ctrl->init() != EC_BOARD_OK) {
        std::cout << "Error in boards init()... cannot proceed!" << std::endl;		
        delete ec_boards_ctrl;
        return 0;
    }

    Rid2PosMap  rid2pos = ec_boards_ctrl->get_Rid2PosMap();
    
    ec_boards_ctrl->get_esc_map_bytype(POW_BOARD, pow_boards);
    
    float min_pos, max_pos, fault;
    for ( auto it = motors.begin(); it != motors.end(); it++ ) {
        Motor * moto = ec_boards_ctrl->slave_as_Motor(rid2pos[*it]);
        if (moto) {
            // start motor controller
            moto->start(CTRL_SET_MIX_POS_MODE);
            moto->getSDO("Min_pos", min_pos);
            moto->getSDO("Max_pos", max_pos);
            moto->getSDO("link_pos", start_pos[*it]); 
            // set home to mid pos
            home[*it] = MID_POS(min_pos,max_pos);
#if 0
            // special case home
            if (*it == Robot_IDs::RA_SH_2 ) { home[*it] = start_pos[*it] + 1.0; }
            if (*it == Robot_IDs::LA_SH_2 ) { home[*it] = start_pos[*it] - 1.0; }
            //if (*it == Robot_IDs::LA_SH_2 ) { home[*it] = 0; }
            if (*it == Robot_IDs::WAIST_Y ) { home[*it] = 0; }
            if (*it == Robot_IDs::WAIST_P ) { home[*it] = start_pos[*it]; }
            //if (*it == Robot_IDs::WAIST_P ) { home[*it] = 0; }
            if (*it == Robot_IDs::WAIST_R ) { home[*it] = 0; }
            if (*it == Robot_IDs::RL_H_R ) { home[*it] = 0; }
            if (*it == Robot_IDs::LL_H_R ) { home[*it] = 0; }
            if (*it == Robot_IDs::RL_H_Y ) { home[*it] = 0; }
            if (*it == Robot_IDs::LL_H_Y ) { home[*it] = 0; }

            if (*it == 1000 ) { home[*it] = M_PI/4; }   
#endif
            DPRINTF("%d : start pos %f home pos %f\n", *it, start_pos[*it], home[*it]);
                        
        } else {
            DPRINTF("NOT a MOTOR %d\n", rid2pos[*it]);
        }
    }

    
    for ( auto it = motors.begin(); it != motors.end(); it++ ) {
        Motor * moto = ec_boards_ctrl->slave_as_Motor(rid2pos[*it]); 
        if (! moto) { continue; }
        while ( ! moto->move_to(home[*it], 0.003) && run_loop ) {
            osal_usleep(2000);    
        }
    }
    
    
    /////////////////////////////////////////////
    // set state OP
    /////////////////////////////////////////////
    if ( ec_boards_ctrl->set_operative() <= 0 ) {
        std::cout << "Error in boards set_operative()... cannot proceed!" << std::endl; 
        delete ec_boards_ctrl;
        return 0;
    }

    /////////////////////////////////////////////
    //
    /////////////////////////////////////////////
    uint64_t start_time = get_time_ns();
    uint64_t tNow, tPre = start_time;
    stat_t  s_loop;
    int fails = 0;
    int cnt = 0;
    double time = 0;
    
    try {
        sleep(3);
        DPRINTF("STARTING\n");
        
        while (run_loop) {
            
            tNow = get_time_ns();
            s_loop(tNow - tPre);
            tPre = tNow;
            
            if ( ec_boards_ctrl->recv_from_slaves() != EC_BOARD_OK ) { break; }
    
            for ( auto it = motors.begin(); it != motors.end(); it++ ) {
                Motor * moto = ec_boards_ctrl->slave_as_Motor(rid2pos[*it]);
                if (moto) {
                    moto->set_posRef(home[*it] + 3.0 * sinf(2*M_PI*time));
                } else {
                    continue;
                }
                
            }
    
            ec_boards_ctrl->send_to_slaves();

            //time += 0.001;    // dc sync 2 ms
            //time += 0.0005;   // dc sync 1 ms
            //time += 0.0002;   // dc sync 0.5 ms
            time += 0.0001;   // really slow

        }
        

    } catch (EscWrpError &e) {
            std::cout << e.what() << std::endl;
    }

    //
    req_state_check(0, EC_STATE_PRE_OP);
    
    for ( auto it = motors.begin(); it != motors.end(); it++ ) {
        Motor * moto = ec_boards_ctrl->slave_as_Motor(rid2pos[*it]); 
        if (! moto) { continue; }
        while ( ! moto->move_to(home[*it], 0.003) ) {
            osal_usleep(2000);    
        }
    }

    
    DPRINTF("elapsed secs %d\n", (int)(get_time_ns() - start_time/1000000000L));
    print_stat(s_loop);

    ec_boards_ctrl->stop_motors();

    delete ec_boards_ctrl;

    return 1;
}
