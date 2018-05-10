/* -*-	Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*- */

#include "ip.h"
#include "tcp-xpass.h"
#include "flags.h"
#include "random.h"
#include "template.h"

/*
 * Tcl Linkage for the following:
 *	Agent/TCP/FullTcp, Agent/TCP/FullTcp/Tahoe,
 *	Agent/TCP/FullTcp/Newreno, Agent/TCP/FullTcp/Sack
 *
 * See tcl/lib/ns-default.tcl for init methods for
 *	Tahoe, Newreno, and Sack
 */

static class XPassFullTcpClass : public TclClass { 
public:
	XPassFullTcpClass() : TclClass("Agent/TCP/FullTcp/XPass") {}
	TclObject* create(int, const char*const*) { 
		return (new XPassTcpAgent());
	}
} class_xpass_full;

/*
 * Delayed-binding variable linkage
 */

void
XPassTcpAgent::delay_bind_init_all()
{
  // delay_bind_init_one()
  xpass_agent_->delay_bind_init_all();
  FullTcpAgent::delay_bind_init_all();
}

int
XPassTcpAgent::delay_bind_dispatch(const char *varName, const char *localName, TclObject *tracer)
{
  printf("%s\n", varName);
  
  //if (delay_bind(varName, localName, "segsperack_", &segs_per_ack_, tracer)) return TCL_OK;
  FullTcpAgent::delay_bind_dispatch(varName, localName, tracer);
  return xpass_agent_->delay_bind_dispatch(varName, localName, tracer);
}

/*
 * sendpacket: 
 *	allocate a packet, fill in header fields, and send
 *	also keeps stats on # of data pkts, acks, re-xmits, etc
 *
 * fill in packet fields.  Agent::allocpkt() fills
 * in most of the network layer fields for us.
 * So fill in tcp hdr and adjust the packet size.
 *
 * Also, set the size of the tcp header.
 */
void
XPassTcpAgent::sendpacket(seq_t seqno, seq_t ackno, int pflags, int datalen, int reason, Packet *p)
{
  printf("Call new sendpacket\n");
  if (!p) p = allocpkt();
        hdr_tcp *tcph = hdr_tcp::access(p);
	hdr_flags *fh = hdr_flags::access(p);

	/* build basic header w/options */

        tcph->seqno() = seqno;
        tcph->ackno() = ackno;
        tcph->flags() = pflags;
        tcph->reason() |= reason; // make tcph->reason look like ns1 pkt->flags?
	tcph->sa_length() = 0;    // may be increased by build_options()
        tcph->hlen() = tcpip_base_hdr_size_;
	tcph->hlen() += build_options(tcph);

	/*
	 * Explicit Congestion Notification (ECN) related:
	 * Bits in header:
	 * 	ECT (EC Capable Transport),
	 * 	ECNECHO (ECHO of ECN Notification generated at router),
	 * 	CWR (Congestion Window Reduced from RFC 2481)
	 * States in TCP:
	 *	ecn_: I am supposed to do ECN if my peer does
	 *	ect_: I am doing ECN (ecn_ should be T and peer does ECN)
	 */

	if (datalen > 0 && ecn_ ){
	        // set ect on data packets 
		fh->ect() = ect_;	// on after mutual agreement on ECT
        } else if (ecn_ && ecn_syn_ && ecn_syn_next_ && (pflags & TH_SYN) && (pflags & TH_ACK)) {
                // set ect on syn/ack packet, if syn packet was negotiating ECT
               	fh->ect() = ect_;
	} else {
		/* Set ect() to 0.  -M. Weigle 1/19/05 */
		fh->ect() = 0;
	}

  // For DCTCP, ect should be set on all packets
  if (dctcp_)
    fh->ect() = ect_;

	if (ecn_ && ect_ && recent_ce_ ) { 
		// This is needed here for the ACK in a SYN, SYN/ACK, ACK
		// sequence.
		pflags |= TH_ECE;
	}
        // fill in CWR and ECE bits which don't actually sit in
        // the tcp_flags but in hdr_flags
        if ( pflags & TH_ECE) {
                fh->ecnecho() = 1;
        } else {
                fh->ecnecho() = 0;
        }
        if ( pflags & TH_CWR ) {
                fh->cong_action() = 1;
        }
	else {
		/* Set cong_action() to 0  -M. Weigle 1/19/05 */
		fh->cong_action() = 0;
	}

	/* actual size is data length plus header length */

        hdr_cmn *ch = hdr_cmn::access(p);
        ch->size() = datalen + tcph->hlen();

        if (datalen <= 0)
                ++nackpack_;
        else {
                ++ndatapack_;
                ndatabytes_ += datalen;
		last_send_time_ = now();	// time of last data
        }
        if (reason == REASON_TIMEOUT || reason == REASON_DUPACK || reason == REASON_SACK) {
                ++nrexmitpack_;
                nrexmitbytes_ += datalen;
        }

	last_ack_sent_ = ackno;

//if (state_ != TCPS_ESTABLISHED) {
//printf("%f(%s)[state:%s]: sending pkt ", now(), name(), statestr(state_));
//prpkt(p);
//}
  if (pflags & (TH_SYN | TH_FIN) || datalen == 0){
    // later should be moved to esablish checker
    if(pflags & (TH_SYN)){
     // delete xpass_agent_;
     // xpass_agent_ = new XPassQueueAgent();
    }
    send(p, 0);
    printf("Send RAW, len:%d, ACK:%d, SYN:%d, FIN:%d\n", datalen, pflags & TH_ACK, pflags & TH_SYN, pflags & TH_FIN);
  } else {
     printf("Send advance, len:%d\n", datalen);
     xpass_agent_->target(getTarget());
     xpass_agent_->add_datalen(0);
     xpass_agent_->advance_packets(p);
   }
	return;
}

void
XPassTcpAgent::recv(Packet *pkt, Handler*)
{
  hdr_tcp *tcph = hdr_tcp::access(pkt);
	int pflags = tcph->flags();
  printf("Recv RAW, ACK:%d, SYN:%d, FIN:%d\n", pflags & TH_ACK, pflags & TH_SYN, pflags & TH_FIN);
  if(xpass_agent_->recv_bool(pkt))
    FullTcpAgent::recv(pkt, NULL);
}


/*
 * the byte-oriented interface: advance_bytes(int nbytes)
 */

void
XPassTcpAgent::advance_bytes(seq_t nb)
{

	//
	// state-specific operations:
	//	if CLOSED or LISTEN, reset and try a new active open/connect
	//	if ESTABLISHED, queue and try to send more
	//	if SYN_SENT or SYN_RCVD, just queue
	//	if above ESTABLISHED, we are closing, so don't allow
	//
  printf("State = %s\n", statestr(state_));
	switch (state_) {

	case TCPS_CLOSED:
	case TCPS_LISTEN:
    reset();
    curseq_ = iss_ + nb;
    connect();              // initiate new connection
    xpass_agent_->add_datalen(nb);
    break;

	case TCPS_ESTABLISHED:
	case TCPS_SYN_SENT:
	case TCPS_SYN_RECEIVED:
                if (curseq_ < iss_) 
                        curseq_ = iss_; 
                curseq_ += nb;

    xpass_agent_->add_datalen(nb);

		break;

	default:
            if (debug_) 
	            fprintf(stderr, "%f: XPassTcpAgent::advance(%s): cannot advance while in state %s\n",
		         now(), name(), statestr(state_));

	}

	if (state_ == TCPS_ESTABLISHED)
		send_much(0, REASON_NORMAL, maxburst_);

  	return;
}



/* XPassQueueAgent */

int XPassQueueAgent::delay_bind_dispatch(const char *varName, const char *localName,
                                    TclObject *tracer) {
  if (delay_bind(varName, localName, "max_credit_rate_", &max_credit_rate_,
                tracer)) {
    return TCL_OK;
  }
  if (delay_bind(varName, localName, "alpha_", &alpha_, tracer)) {
    return TCL_OK;
  }
  if (delay_bind(varName, localName, "min_credit_size_", &min_credit_size_,
                 tracer)) {
    return TCL_OK;
  }
  if (delay_bind(varName, localName, "max_credit_size_", &max_credit_size_,
                 tracer)) {
    return TCL_OK;
  }
  if (delay_bind(varName, localName, "min_ethernet_size_", &min_ethernet_size_,
                 tracer)) {
    return TCL_OK;
  }
  if (delay_bind(varName, localName, "max_ethernet_size_", &max_ethernet_size_,
                 tracer)) {
    return TCL_OK;
  }
  if (delay_bind(varName, localName, "xpass_hdr_size_", &xpass_hdr_size_,
                 tracer)) {
    printf("xpass_hdr_size_ = %d\n", xpass_hdr_size_);
    return TCL_OK;
  }
  if (delay_bind(varName, localName, "target_loss_scaling_", &target_loss_scaling_,
                 tracer)) {
    return TCL_OK;
  }
  if (delay_bind(varName, localName, "w_init_", &w_init_, tracer)) {
    return TCL_OK;
  }
  if (delay_bind(varName, localName, "min_w_", &min_w_, tracer)) {
    return TCL_OK;
  }
  if (delay_bind(varName, localName, "retransmit_timeout_", &retransmit_timeout_,
                 tracer)) {
    return TCL_OK;
  }
  if (delay_bind(varName, localName, "default_credit_stop_timeout_", &default_credit_stop_timeout_,
                 tracer)) {
    return TCL_OK;
  }
  if (delay_bind(varName, localName, "max_jitter_", &max_jitter_, tracer)) {
    return TCL_OK;
  }
  if (delay_bind(varName, localName, "min_jitter_", &min_jitter_, tracer)) {
    return TCL_OK;
  }
  return 0;
}


bool XPassQueueAgent::recv_bool(Packet *pkt) {
  hdr_cmn *cmnh = hdr_cmn::access(pkt);

  switch (cmnh->ptype()) {
    case PT_XPASS_CREDIT_REQUEST:
      recv_credit_request(pkt);
      break;
    case PT_XPASS_CREDIT:
      recv_credit(pkt);
      break;
    case PT_XPASS_DATA:
      recv_data(pkt);
      return true;
    case PT_XPASS_CREDIT_STOP:
      recv_credit_stop(pkt);
      break;
    case PT_XPASS_NACK:
      recv_nack(pkt);
      break;
    default:
      return true;
  }
  Packet::free(pkt);
  return false;
}

Packet* XPassQueueAgent::construct_credit_request() {
  Packet *p = allocpkt();
  if (!p) {
    fprintf(stderr, "ERROR: allockpkt() failed\n");
    exit(1);
  }

//  hdr_tcp *tcph = hdr_tcp::access(p);
  hdr_cmn *cmnh = hdr_cmn::access(p);
  hdr_xpass *xph = hdr_xpass::access(p);

//  tcph->seqno() = t_seqno_;
//  tcph->ackno() = recv_next_;
//  tcph->hlen() = xpass_hdr_size_;

  cmnh->size() = min_ethernet_size_;
  cmnh->ptype() = PT_XPASS_CREDIT_REQUEST;

  xph->credit_seq() = 0;
  xph->credit_sent_time_ = now();
  xph->sendbuffer_ = pkt_remaining();

  // to measure rtt between credit request and first credit
  // for sender.
  rtt_ = now();

  return p;
}

Packet* XPassQueueAgent::construct_credit_stop() {
  Packet *p = allocpkt();
  if (!p) {
    fprintf(stderr, "ERROR: allockpkt() failed\n");
    exit(1);
  }
//  hdr_tcp *tcph = hdr_tcp::access(p);
  hdr_cmn *cmnh = hdr_cmn::access(p);
  hdr_xpass *xph = hdr_xpass::access(p);

//  tcph->seqno() = t_seqno_;
//  tcph->ackno() = recv_next_;
//  tcph->hlen() = xpass_hdr_size_;
  
  cmnh->size() = min_ethernet_size_;
  cmnh->ptype() = PT_XPASS_CREDIT_STOP;

  xph->credit_seq() = 0;

  return p;
}

Packet* XPassQueueAgent::construct_credit() {
  Packet *p = allocpkt();
  if (!p) {
    fprintf(stderr, "ERROR: allockpkt() failed\n");
    exit(1);
  }
//  hdr_tcp *tcph = hdr_tcp::access(p);
  hdr_cmn *cmnh = hdr_cmn::access(p);
  hdr_xpass *xph = hdr_xpass::access(p);
  int credit_size = min_credit_size_;

  if (min_credit_size_ < max_credit_size_) {
    // variable credit size
    credit_size += rand()%(max_credit_size_ - min_credit_size_ + 1);
  } else {
    // static credit size
    if (min_credit_size_ != max_credit_size_){
      fprintf(stderr, "ERROR: min_credit_size_ should be less than or equal to max_credit_size_\n");
      exit(1);
    }
  }

//  tcph->seqno() = t_seqno_;
//  tcph->ackno() = recv_next_;
//  tcph->hlen() = credit_size;

  cmnh->size() = credit_size;
  cmnh->ptype() = PT_XPASS_CREDIT;

  xph->credit_sent_time() = now();
  xph->credit_seq() = c_seqno_;

  c_seqno_ = max(1, c_seqno_+1);

  return p;
}


void XPassQueueAgent::recv_data(Packet *pkt) {
  hdr_xpass *xph = hdr_xpass::access(pkt);
  // distance between expected sequence number and actual sequence number.
  int distance = xph->credit_seq() - c_recv_next_;

  if (distance < 0) {
    // credit packet reordering or credit sequence number overflow happend.
    fprintf(stderr, "ERROR: Credit Sequence number is reverted.\n");
    exit(1);
  }
  credit_total_ += (distance + 1);
  credit_dropped_ += distance;

  c_recv_next_ = xph->credit_seq() + 1;

  // no ack?
  //process_ack(pkt);
  update_rtt(pkt);
}
Packet* XPassQueueAgent::construct_data(Packet *credit) {
  if (packet_queue_.size() == 0) {
    fprintf(stderr, "ERROR: packet queue is already empty\n");
    exit(1);
  }

  Packet *p = packet_queue_.front();

//  hdr_tcp *tcph = hdr_tcp::access(p);
  hdr_cmn *cmnh = hdr_cmn::access(p);
  hdr_xpass *xph = hdr_xpass::access(p);
  hdr_xpass *credit_xph = hdr_xpass::access(credit);
 
  int datalen = (int)p->datalen();

  if (datalen <= 0) {
    fprintf(stderr, "ERROR: datapacket has length of less than zero\n");
    exit(1);
  }

  if (datalen > max_segment()) {
    fprintf(stderr, "ERROR: datapacket exceeds max_segment size\n");
    exit(1);
  }
  //tcph->seqno() = t_seqno_;
  //tcph->ackno() = recv_next_;
  //tcph->hlen() = xpass_hdr_size_;

  cmnh->size() = max(min_ethernet_size_, xpass_hdr_size_ + datalen);
  cmnh->ptype() = PT_XPASS_DATA;

  xph->credit_sent_time() = credit_xph->credit_sent_time();
  xph->credit_seq() = credit_xph->credit_seq();
  
  t_seqno_ += datalen;
  packet_queue_.pop();
  return p;
}

Packet* XPassQueueAgent::construct_nack(seq_t seq_no) {
  Packet *p = allocpkt();
  if (!p) {
    fprintf(stderr, "ERROR: allockpkt() failed\n");
    exit(1);
  }
//  hdr_tcp *tcph = hdr_tcp::access(p);
  hdr_cmn *cmnh = hdr_cmn::access(p);

  //tcph->ackno() = seq_no;
  //tcph->hlen() = xpass_hdr_size_; 

  cmnh->size() = min_ethernet_size_;
  cmnh->ptype() = PT_XPASS_NACK;

  return p;
}

void XPassQueueAgent::advance_packets(Packet *p) {
// drop packet if exceed max_packet_queue
  if (packet_queue_.size() < MAX_PACKET_QUEUE) {
    packet_queue_.push(p);
    printf("Added to Packet Queue, cnt=%d\n", packet_queue_.size());

  } else {
    printf("Dropped from Packet Queue\n");
    return;
  }

  curseq_ += p->datalen();

  switch (credit_recv_state_) {
    case XPASS_RECV_CREDIT_REQUEST_SENT:
    case XPASS_RECV_CREDIT_RECEIVING:
      return;
    case XPASS_RECV_CREDIT_STOP_SENT:
    case XPASS_RECV_CLOSE_WAIT:
    case XPASS_RECV_CLOSED:
      break;
  }
 
  // send credit request
  send(construct_credit_request(), 0);
  sender_retransmit_timer_.sched(retransmit_timeout_);

  // XPASS_RECV_CLOSED -> XPASS_RECV_CREDIT_REQUEST_SENT
  credit_recv_state_ = XPASS_RECV_CREDIT_REQUEST_SENT;

}

void XPassQueueAgent::add_datalen(seq_t nb) {
  curseq_ += nb;
  printf("Add datalen, curseq = %ld, seqno = %ld\n", (long) curseq_, (long)t_seqno_);
}
