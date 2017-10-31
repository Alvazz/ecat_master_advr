/*
 *
 *  Created on: Mar, 2017
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_AUX_PDO_H__
#define __IIT_ECAT_ADVR_AUX_PDO_H__

#include <iit/ecat/advr/esc.h>
#include <iostream>

namespace iit {
namespace ecat {
namespace advr {

#define MK_PDO_AUX(kls,arg)             std::make_shared<kls>(kls(getSDObjd(arg)))
#define MK_PDO_AUX_WRD(kls,arg1,arg2)   std::make_shared<kls>(kls(getSDObjd(arg1),getSDObjd(arg2)))

#define AUX_PDO_OP_SET  0xA
#define AUX_PDO_OP_GET  0xB
#define AUX_PDO_OP_WRD  0xC
#define AUX_PDO_OP_ACK  0x0
#define AUX_PDO_OP_NACK 0xE

#define AUX_PDO_EE_INVALID_OP   0xE1
#define AUX_PDO_EE_INVALID_IDX  0xE2
#define AUX_PDO_EE_READONLY     0xE3
    
///////////////////////////////////////////////////////////////////////////////
//

class PDO_aux {
public:
    
    int check_ackNack( uint16_t op_idx_ack, uint32_t code) {
        if ( (op_idx_ack >> 12) == 0xE ) {
            DPRINTF("[PDO_aux %s] Fail %s errno 0x%02X\n", get_objd()->name, __FUNCTION__, code);
            return 1;
        }
        return 0;
    }
    int check_idx( uint16_t op_idx_ack, uint16_t idx, uint32_t code) {
        if ( (op_idx_ack & 0xFFF) != idx ) {
            DPRINTF("[PDO_aux %s] Fail %s errno 0x%02X\n", get_objd()->name, __FUNCTION__, code);
            return 1;
        }
        return 0;
    }
    
    template<class T>
    void on_tx( T& tx_pdo );

    template<class T>
    void on_rx( T& rx_pdo );
    
    virtual const objd_t * get_objd(void)   { throw std::runtime_error("pure virtual"); }
    
};

    
///////////////////////////////////////////////////////////////////////////////
//
class PDO_rd_aux : public PDO_aux {
public:
    PDO_rd_aux(): sdo_objd(NULL) {}
    PDO_rd_aux( const objd_t * sdo_obj_data ): sdo_objd( sdo_obj_data )     { assert( sdo_objd != 0 ); } 
    PDO_rd_aux( const PDO_rd_aux& rhs ): sdo_objd( rhs.sdo_objd )           { assert( sdo_objd != 0 ); }
    //
    // these template methods expect [rx/tx]_pdo struct with op_idx_aux/op_idx_ack and aux fields
    // 
    template<class T>
    void on_tx_impl( T& tx_pdo ) {
        // get op
        tx_pdo.op_idx_aux = AUX_PDO_OP_GET << 12 | sdo_objd->subindex & 0xFFF;
        //DPRINTF("PDO_aux 0x%04X\n", tx_pdo.op_idx_aux);
    };
    
    template<class T>
    void on_rx_impl( T& rx_pdo) {
        int ret;
        // check nack
        ret = check_ackNack(rx_pdo.op_idx_ack, (uint32_t)rx_pdo.aux);
        // check idx
        ret |= check_idx(rx_pdo.op_idx_ack, sdo_objd->subindex, (uint32_t)rx_pdo.aux);
        if ( ! ret ) {
            *(float*)sdo_objd->data = rx_pdo.aux;
        }
    }
    
    virtual const objd_t * get_objd(void)   { return (sdo_objd == 0) ? 0 : sdo_objd; }
    
private:
    const objd_t *  sdo_objd; 
};


///////////////////////////////////////////////////////////////////////////////
//
class PDO_wr_aux : public PDO_aux {
public:
    PDO_wr_aux(): sdo_objd(NULL) {}
    PDO_wr_aux( const objd_t * sdo_obj_data ): sdo_objd( sdo_obj_data )     { assert( sdo_objd != 0 && sdo_objd->access == ATYPE_RW ); }
    PDO_wr_aux( const PDO_wr_aux& rhs ): sdo_objd( rhs.sdo_objd )           { assert( sdo_objd != 0 && sdo_objd->access == ATYPE_RW ); } 
    //
    // these template methods expect [rx/tx]_pdo struct with op_idx_aux/op_idx_ack and aux fields
    // 
    template<class T>
    void on_tx_impl( T& tx_pdo ) {
        // set op
        tx_pdo.op_idx_aux = AUX_PDO_OP_SET << 12 | sdo_objd->subindex & 0xFFF;
        tx_pdo.aux = *(float*)sdo_objd->data;
        //DPRINTF("PDO_aux 0x%04X\n", tx_pdo.op_idx_aux);
    };
    
    template<class T>
    void on_rx_impl( T& rx_pdo) {
        static uint64_t prev_err_ts;
        // check nack
        check_ackNack(rx_pdo.op_idx_ack, (uint32_t)rx_pdo.aux);
        // check idx
        check_idx(rx_pdo.op_idx_ack, sdo_objd->subindex, (uint32_t)rx_pdo.aux);
    }
    
    virtual const objd_t * get_objd(void)   { return (sdo_objd == 0) ? 0 : sdo_objd; }
    
private:
    const objd_t *  sdo_objd; 
};


///////////////////////////////////////////////////////////////////////////////
//
class PDO_wrd_aux : public PDO_aux {
public:
    PDO_wrd_aux(): sdo_objd_wr(NULL),sdo_objd_rd(NULL) {}
    PDO_wrd_aux( const objd_t * sdo_objd_wr, const objd_t * sdo_objd_rd ):
        sdo_objd_wr( sdo_objd_wr ),
        sdo_objd_rd( sdo_objd_rd )      { assert ( sdo_objd_wr != 0 && sdo_objd_rd != 0 ); }
    PDO_wrd_aux( const PDO_wrd_aux& rhs ):
        sdo_objd_wr( rhs.sdo_objd_wr ),
        sdo_objd_rd( rhs.sdo_objd_rd )  { assert ( sdo_objd_wr != 0 && sdo_objd_rd != 0 ); }
    //
    // these template methods expect [rx/tx]_pdo struct with op_idx_aux/op_idx_ack and aux fields
    // 
    template<class T>
    void on_tx_impl( T& tx_pdo ) {
        idxs = ( (sdo_objd_wr->subindex & 0x3F) << 6) | (sdo_objd_rd->subindex & 0x3F);
        tx_pdo.op_idx_aux = AUX_PDO_OP_WRD << 12 | idxs ;
        tx_pdo.aux = *(float*)sdo_objd_wr->data;
        //DPRINTF("PDO_wrd_aux 0x%04X\n", tx_pdo.op_idx_aux);
    };
    
    template<class T>
    void on_rx_impl( T& rx_pdo) {
        int ret;
        // check nack
        ret = check_ackNack(rx_pdo.op_idx_ack, (uint32_t)rx_pdo.aux);
        // check idx
        ret |= check_idx(rx_pdo.op_idx_ack, idxs, (uint32_t)rx_pdo.aux);
        if ( ! ret ) {
            *(float*)sdo_objd_rd->data = rx_pdo.aux;
        }
    }

    const objd_t * get_objd_wr(void)        { return (sdo_objd_wr == 0) ? 0 : sdo_objd_wr; }
    const objd_t * get_objd_rd(void)        { return (sdo_objd_rd == 0) ? 0 : sdo_objd_rd; }
    virtual const objd_t * get_objd(void)   { return get_objd_rd(); }

private:
    uint16          idxs;
    const objd_t    *sdo_objd_wr, *sdo_objd_rd; 
};



template<class T>
void PDO_aux::on_tx( T& tx_pdo ) {
    
    PDO_rd_aux  * k_rd  = dynamic_cast<PDO_rd_aux*> ( this );
    PDO_wr_aux  * k_wr  = dynamic_cast<PDO_wr_aux*> ( this );
    PDO_wrd_aux * k_wrd = dynamic_cast<PDO_wrd_aux*>( this );
    if ( k_rd )         { k_rd->on_tx_impl<T>(tx_pdo); }
    else if ( k_wr )    { k_wr->on_tx_impl<T>(tx_pdo); }
    else if ( k_wrd )   { k_wrd->on_tx_impl<T>(tx_pdo); }
    else {
        throw std::runtime_error("PDO_aux::on_tx invalid class");
    }
    
}

template<class T>
void PDO_aux::on_rx( T& rx_pdo ) {

    PDO_rd_aux  * k_rd  = dynamic_cast<PDO_rd_aux*>  ( this );
    PDO_wr_aux  * k_wr  = dynamic_cast<PDO_wr_aux*>  ( this );
    PDO_wrd_aux * k_wrd = dynamic_cast<PDO_wrd_aux*> ( this );
    if ( k_rd )         { k_rd->on_rx_impl<T>(rx_pdo); }
    else if ( k_wr )    { k_wr->on_rx_impl<T>(rx_pdo); }
    else if ( k_wrd )   { k_wrd->on_rx_impl<T>(rx_pdo); }
    else {
        throw std::runtime_error("PDO_aux::on_rx invalid class");
    }

}


}
}
}

#endif /* IIT_ECAT_ADVR_PDO_AUX_H_ */

// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
