/*
 *
 *  Created on: Apr, 2015
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_PIPE_H__
#define __IIT_ECAT_ADVR_PIPE_H__

#include <cstring>

#ifdef __COBALT__
    #include <iit/ecat/advr/rt_ipc.h>
#else
    #include <sys/types.h>
    #include <sys/stat.h>
    #include <fcntl.h>
#endif


#ifdef __COBALT__
    static const std::string pipe_prefix ( "/proc/xenomai/registry/rtipc/xddp/" );
#else
    static const std::string pipe_prefix ( "/tmp/" );
#endif


class XDDP_pipe {

public:

    XDDP_pipe ( int _pool_size = 8192 ) : pool_size ( _pool_size ) { fd = 0; }

    int get_fd() { return fd; }
    
    void init ( const std::string pipe_name ) {

        std::string pipe = pipe_prefix + pipe_name;

#ifdef __COBALT__
        fd = xddp_bind ( pipe_name.c_str(), pool_size );
#else
        mkfifo ( pipe.c_str(), S_IRWXU|S_IRWXG );
        fd = open ( pipe.c_str(), O_RDWR | O_NONBLOCK );
#endif
        DPRINTF ( " .... open %s\n", pipe.c_str() );
        assert ( fd > 0 );
    }

    virtual ~XDDP_pipe() {
        if ( ! fd ) {
            return;
        }

        close ( fd );
#ifndef __COBALT__
        std::string pipe = pipe_prefix + pipe_name;
        unlink ( pipe.c_str() );
#endif
    }

    template<typename XddpTxTypes>
    int xddp_write ( const XddpTxTypes & tx ) {
        if ( fd <= 0 ) { return 0; }
        return write ( fd, ( void* ) &tx, sizeof ( tx ) );
    }

    int xddp_write ( const uint8_t * buffer, int size ) {
        if ( fd <= 0 ) { return 0; }
        return write ( fd, ( void* ) buffer, size );
    }

    template<typename XddpRxTypes>
    int xddp_read ( const XddpRxTypes & rx ) {

        if ( fd <= 0 ) { return 0; }
        /////////////////////////////////////////////////////////
        // NON-BLOCKING, read buff_size byte from pipe or cross domain socket
#if __COBALT__
        return recvfrom ( fd, ( void* ) &rx, sizeof ( rx ), MSG_DONTWAIT, NULL, 0 );
#else
        // NON-BLOCKING
        return read ( fd, ( void* ) &rx, sizeof ( rx ) );
#endif
    }

    int xddp_read ( const uint8_t * buffer, int size ) {

        if ( fd <= 0 ) { return 0; }
        /////////////////////////////////////////////////////////
        // NON-BLOCKING, read buff_size byte from pipe or cross domain socket
#if __COBALT__
        return recvfrom ( fd, (void*)buffer, size, MSG_DONTWAIT, NULL, 0 );
#else
        // NON-BLOCKING
        return read ( fd, (void*)buffer, size );
#endif
    }

protected:
    std::string pipe_name;

private:
    int fd;
    int pool_size;

};

#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
