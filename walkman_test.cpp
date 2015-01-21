#include <signal.h>
#include <sys/mman.h>
#include <execinfo.h>
#include <stdio.h>
#include <stdint.h>

#ifdef __XENO__
    #include <rtdk.h>
#endif

#include <boost/circular_buffer.hpp>

#include <iit/ecat/advr/ec_boards_iface.h>
#include <ati_iface.h>

using namespace iit::ecat::advr;

static int run_loop = 1;


typedef struct {
    uint64_t    ts;
    float       ati[6];
    float       iit[6];
    float       dummy[6];
    void sprint(char *buff, size_t size) {
        snprintf(buff, size, "%lld\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t0\t0\t0\t0\t0\t0\n", ts,
                 iit[0],iit[1],iit[2],iit[3],iit[4],iit[5],
                 -ati[0],ati[1],ati[2],-ati[3],ati[4],ati[5]);
    }
    void fprint(FILE *fp) {
        fprintf(fp, "%lld\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t0\t0\t0\t0\t0\t0\n", ts,
                iit[0],iit[1],iit[2],iit[3],iit[4],iit[5],
                -ati[0],ati[1],ati[2],-ati[3],ati[4],ati[5]);
    }
} sens_data_t ; 



static void warn_upon_switch(int sig __attribute__((unused)))
{
    // handle rt to nrt contex switch
    void *bt[3];
    int nentries;

    /* Dump a backtrace of the frame which caused the switch to
       secondary mode: */
    nentries = backtrace(bt,sizeof(bt)/sizeof(bt[0]));
    // dump backtrace 
    backtrace_symbols_fd(bt,nentries,fileno(stdout));
}

static void shutdown(int sig __attribute__((unused)))
{
    run_loop = 0;
    DPRINTF("got signal .... Shutdown\n");
}

static void set_signal_handler(void)
{
    signal(SIGINT, shutdown);
    signal(SIGINT, shutdown);
    signal(SIGKILL, shutdown);
#ifdef __XENO__
    // call pthread_set_mode_np(0, PTHREAD_WARNSW) to cause a SIGXCPU
    // signal to be sent when the calling thread involontary switches to secondary mode
    signal(SIGXCPU, warn_upon_switch);
#endif
}

///////////////////////////////////////////////////////////////////////////////

using namespace iit::ecat;

Ec_Boards_ctrl * ec_boards_ctrl; 


int main(int argc, char **argv)
{
    int ret;

    boost::circular_buffer<sens_data_t> sens_log;
    sens_log.set_capacity(LOG_SIZE);

    set_signal_handler();

#ifdef __XENO__
    
    int policy = SCHED_FIFO;
    struct sched_param  schedparam;
    schedparam.sched_priority = sched_get_priority_max(policy);
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &schedparam);

    /* Prevent any memory-swapping for this program */
    ret = mlockall(MCL_CURRENT | MCL_FUTURE);
    if ( ret < 0 ) {
        printf("mlockall failed (ret=%d) %s\n", ret, strerror(ret));
        return 0;
    }
    /*
     * This is a real-time compatible printf() package from
     * Xenomai's RT Development Kit (RTDK), that does NOT cause
     * any transition to secondary (i.e. non real-time) mode when
     * writing output.
     */
    rt_print_auto_init(1);
#endif

    if ( argc != 2 ) {
    printf("Usage: %s ifname\nifname = {eth0,rteth0} for example\n", argv[0]);
        return 0;
    }

    ///////////////////////////////////////////////////////////////////////

    Ec_Boards_ctrl * ec_boards_ctrl;

    ec_boards_ctrl = new Ec_Boards_ctrl(argv[1]); 

    if ( ec_boards_ctrl->init() <= 0) {
        std::cout << "Error in boards init()... cannot proceed!" << std::endl;		
        delete ec_boards_ctrl;
        return 0;
    }
    ec_boards_ctrl->configure_boards();

    if ( ec_boards_ctrl->set_operative() <= 0) {
        std::cout << "Error in boards set_operative()... cannot proceed!" << std::endl;	
        delete ec_boards_ctrl;
        return 0;
    }
    
    int cnt = 0;
    uint16_t  cmd = CTRL_POWER_MOD_OFF;

    static double time;
    McESCTypes::pdo_rx mc_pdo_rx;
    McESCTypes::pdo_tx mc_pdo_tx;

    FtESCTypes::pdo_rx ft_pdo_rx;
    FtESCTypes::pdo_tx ft_pdo_tx;

    uint64_t    start = get_time_ns();

    ec_boards_ctrl->set_ctrl_status(7,CTRL_SET_DIRECT_MODE);
    ec_boards_ctrl->set_ctrl_status(8,CTRL_SET_DIRECT_MODE);
    ec_boards_ctrl->set_ctrl_status(7,CTRL_POWER_MOD_ON);
    ec_boards_ctrl->set_ctrl_status(8,CTRL_POWER_MOD_ON);

    ec_boards_ctrl->recv_from_slaves();

    ec_boards_ctrl->getRxPDO(7, mc_pdo_rx);
    mc_pdo_tx.pos_ref = mc_pdo_rx.position;
    mc_pdo_tx.PosGainP = 200;
    mc_pdo_tx.PosGainI = 0;
    mc_pdo_tx.PosGainD = 10;
    mc_pdo_tx.ts = get_time_ns();
    ec_boards_ctrl->setTxPDO(7, mc_pdo_tx);

    ec_boards_ctrl->getRxPDO(8, mc_pdo_rx);
    mc_pdo_tx.pos_ref = mc_pdo_rx.position;
    mc_pdo_tx.PosGainP = 200;
    mc_pdo_tx.PosGainI = 0;
    mc_pdo_tx.PosGainD = 10;
    mc_pdo_tx.ts = get_time_ns();
    ec_boards_ctrl->setTxPDO(8, mc_pdo_tx);

    ec_boards_ctrl->send_to_slaves();

    ec_boards_ctrl->set_ctrl_status(7,CTRL_SET_POS_MODE);
    ec_boards_ctrl->set_ctrl_status(8,CTRL_SET_POS_MODE);


    sleep(1);

    ec_boards_ctrl->recv_from_slaves();

    mc_pdo_tx.pos_ref = M_PI;
    mc_pdo_tx.ts = get_time_ns();
    ec_boards_ctrl->setTxPDO(7, mc_pdo_tx);
    ec_boards_ctrl->setTxPDO(8, mc_pdo_tx);

    ec_boards_ctrl->send_to_slaves();


    while ( run_loop ) {
        
        ec_boards_ctrl->recv_from_slaves();

        if ( (cnt % 10) == 0) {
            ec_boards_ctrl->check_sanity();
        }
        cnt++;

        ///////////////////////////////////////////////////////////////////////
        time += 0.002;
        mc_pdo_tx.pos_ref = M_PI + sinf(2*M_PI*time);
        mc_pdo_tx.ts = get_time_ns();
        ec_boards_ctrl->setTxPDO(7, mc_pdo_tx);
        ec_boards_ctrl->setTxPDO(8, mc_pdo_tx);

        ///////////////////////////////////////////////////////////////////////

        ec_boards_ctrl->send_to_slaves();
   
    }

    /////////////////////////////////////////////////////////////////

    delete ec_boards_ctrl;

    
    return 0;
}
