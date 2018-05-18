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


void XTSendCreditTimer::expire(Event *) {
  a_->send_credit();
}

void XTCreditStopTimer::expire(Event *) {
  a_->send_credit_stop();
}

void XTSenderRetransmitTimer::expire(Event *) {
  a_->handle_sender_retransmit();
}

void XTReceiverRetransmitTimer::expire(Event *) {
  a_->handle_receiver_retransmit();
}



/*
 * Delayed-binding variable linkage
 */

void
XPassTcpAgent::delay_bind_init_all()
{
  // delay_bind_init_one()
  delay_bind_init_one("max_credit_rate_");
  delay_bind_init_one("alpha_");
  delay_bind_init_one("min_credit_size_");
  delay_bind_init_one("max_credit_size_");
  delay_bind_init_one("min_ethernet_size_");
  delay_bind_init_one("max_ethernet_size_");
  delay_bind_init_one("xpass_hdr_size_");
  delay_bind_init_one("target_loss_scaling_");
  delay_bind_init_one("w_init_");
  delay_bind_init_one("min_w_");
  delay_bind_init_one("retransmit_timeout_");
  delay_bind_init_one("default_credit_stop_timeout_");
  delay_bind_init_one("min_jitter_");
  delay_bind_init_one("max_jitter_");
  FullTcpAgent::delay_bind_init_all();
}

int
XPassTcpAgent::delay_bind_dispatch(const char *varName, const char *localName, TclObject *tracer)
{
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
  return FullTcpAgent::delay_bind_dispatch(varName, localName, tracer);
}
static FILE *f_log;
void XPassTcpAgent::init() {
  w_ = w_init_;
//  cur_credit_rate_ = (int)(alpha_ * max_credit_rate_);
  last_credit_rate_update_ = now();
  if(!f_log)
    f_log = fopen("traces/log.txt", "a");
}


void
XPassTcpAgent::recv(Packet *pkt, Handler* h)
{
  hdr_cmn *cmnh = hdr_cmn::access(pkt);

  switch (cmnh->ptype()) {
    case PT_XPASS_CREDIT_REQUEST:
      recv_credit_request(pkt);
      Packet::free(pkt);
      break;
    case PT_XPASS_CREDIT:
      recv_credit(pkt);
      Packet::free(pkt);
      break;
    case PT_XPASS_DATA:
      recv_data(pkt);
      FullTcpAgent::recv(pkt, h);
      break;
    case PT_XPASS_CREDIT_STOP:
      recv_credit_stop(pkt);
      Packet::free(pkt);
      break;
    case PT_XPASS_NACK:
      recv_nack(pkt);
      Packet::free(pkt);
      break;
    default:
      FullTcpAgent::recv(pkt, h);
      break;
  }
}

void XPassTcpAgent::recv_credit_request(Packet *pkt) {
  hdr_xpass *xph = hdr_xpass::access(pkt);

  switch (credit_send_state_) {
    case XPASS_SEND_CLOSED:
      double lalpha;
      init();
      if (xph->sendbuffer_ >= 40) {
        lalpha = alpha_;
      } else {
        lalpha = alpha_ * xph->sendbuffer_ / 40.0;
      }
      
      cur_credit_rate_ = (int)(lalpha * max_credit_rate_);
      fst_ = xph->credit_sent_time();
      // need to start to send credits.
      send_credit();
        
      // XPASS_SEND_CLOSED -> XPASS_SEND_CREDIT_REQUEST_RECEIVED
      credit_send_state_ = XPASS_SEND_CREDIT_SENDING;     
      break;
  }
}

void XPassTcpAgent::recv_credit(Packet *pkt) {
  credit_recved_rtt_++;
  switch (credit_recv_state_) {
    case XPASS_RECV_CREDIT_REQUEST_SENT:
      sender_retransmit_timer_.force_cancel();
      credit_recv_state_ = XPASS_RECV_CREDIT_RECEIVING;
      // first sender RTT.
      rtt_ = now() - rtt_;
      last_credit_recv_update_ = now();
    case XPASS_RECV_CREDIT_RECEIVING:
      // send data
      if (datalen_remaining() > 0) {
        Packet *pkt_data = construct_data(pkt);
        if(pkt_data)
          send(pkt_data, 0);
      }
 
      if (datalen_remaining() == 0) {
        if (credit_stop_timer_.status() != TIMER_IDLE) {
          fprintf(stderr, "Error: CreditStopTimer seems to be scheduled more than once.\n");
          exit(1);
        }
        // Because ns2 does not allow sending two consecutive packets, 
        // credit_stop_timer_ schedules CREDIT_STOP packet with no delay.
        credit_stop_timer_.sched(0);
      } else if (now() - last_credit_recv_update_ >= rtt_) {
        if (credit_recved_rtt_ >= pkt_remaining()) {
          // Early credit stop
          if (credit_stop_timer_.status() != TIMER_IDLE) {
            fprintf(stderr, "Error: CreditStopTimer seems to be scheduled more than once.\n");
            exit(1);
          }
          // Because ns2 does not allow sending two consecutive packets, 
          // credit_stop_timer_ schedules CREDIT_STOP packet with no delay.
          credit_stop_timer_.sched(0);
        }
        credit_recved_rtt_ = 0;
        last_credit_recv_update_ = now();
      }
      break;
    case XPASS_RECV_CREDIT_STOP_SENT:
      if (datalen_remaining() > 0) {
        send(construct_data(pkt), 0);
      } else {
        credit_wasted_++;
      }
      credit_recved_++;
      break;
    case XPASS_RECV_CLOSE_WAIT:
      // accumulate credit count to check if credit stop has been delivered
      credit_wasted_++;
      break;
    case XPASS_RECV_CLOSED:
      credit_wasted_++;
      break;
  }
}

void XPassTcpAgent::recv_data(Packet *pkt) {
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

  // no ack
  update_rtt(pkt);
}

void XPassTcpAgent::recv_nack(Packet *pkt) {
  hdr_tcp *tcph = hdr_tcp::access(pkt);
  switch (credit_recv_state_) {
    case XPASS_RECV_CREDIT_STOP_SENT:
    case XPASS_RECV_CLOSE_WAIT:
    case XPASS_RECV_CLOSED:
      send(construct_credit_request(), 0);
      credit_recv_state_ = XPASS_RECV_CREDIT_REQUEST_SENT;
      sender_retransmit_timer_.resched(retransmit_timeout_);
    case XPASS_RECV_CREDIT_REQUEST_SENT:
    case XPASS_RECV_CREDIT_RECEIVING:
      // set t_seqno_ for retransmission
      t_seqno_ = tcph->ackno();
  }
}

void XPassTcpAgent::recv_credit_stop(Packet *pkt) {
  FILE *fct_out = fopen("outputs/fct.out","a");

  send_credit_timer_.force_cancel();
  fprintf(fct_out,"%d,%ld,%.10lf\n", fid_, recv_next_-1, now()-fst_);
  fclose(fct_out);
  credit_send_state_ = XPASS_SEND_CLOSED;
}


void XPassTcpAgent::handle_sender_retransmit() {
  switch (credit_recv_state_) {
    case XPASS_RECV_CREDIT_REQUEST_SENT:
      send(construct_credit_request(), 0);
      sender_retransmit_timer_.resched(retransmit_timeout_);
      break;
    case XPASS_RECV_CREDIT_STOP_SENT:
      if (datalen_remaining() > 0) {
        credit_recv_state_ = XPASS_RECV_CREDIT_REQUEST_SENT;
        send(construct_credit_request(), 0);
        sender_retransmit_timer_.resched(retransmit_timeout_);
      } else {
        credit_recv_state_ = XPASS_RECV_CLOSE_WAIT;
        credit_recved_ = 0;
        sender_retransmit_timer_.resched((rtt_ > 0) ? rtt_ : default_credit_stop_timeout_); 
      }
      break;
    case XPASS_RECV_CLOSE_WAIT:
      if (credit_recved_ == 0) {
        FILE *waste_out = fopen("outputs/waste.out","a");

        credit_recv_state_ = XPASS_RECV_CLOSED;
        sender_retransmit_timer_.force_cancel();
        fprintf(waste_out, "%d,%ld,%d\n", fid_, curseq_-1, credit_wasted_);
        fclose(waste_out); 
        return;
      }
      // retransmit credit_stop
      send_credit_stop();
      break;
    case XPASS_RECV_CLOSED:
      fprintf(stderr, "Sender Retransmit triggered while connection is closed.");
      exit(1);
  }
}

void XPassTcpAgent::handle_receiver_retransmit() {
  if (wait_retransmission_) {
    send(construct_nack(recv_next_), 0);
    receiver_retransmit_timer_.resched(retransmit_timeout_);
  }
}

Packet* XPassTcpAgent::construct_credit_request() {
  Packet *p = allocpkt();
  if (!p) {
    fprintf(stderr, "ERROR: allockpkt() failed\n");
    exit(1);
  }

  hdr_tcp *tcph = hdr_tcp::access(p);
  hdr_cmn *cmnh = hdr_cmn::access(p);
  hdr_xpass *xph = hdr_xpass::access(p);

  //tcph->seqno() = t_seqno_;
  //tcph->ackno() = recv_next_;
  //tcph->hlen() = xpass_hdr_size_;

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

Packet* XPassTcpAgent::construct_credit_stop() {
  Packet *p = allocpkt();
  if (!p) {
    fprintf(stderr, "ERROR: allockpkt() failed\n");
    exit(1);
  }
  hdr_tcp *tcph = hdr_tcp::access(p);
  hdr_cmn *cmnh = hdr_cmn::access(p);
  hdr_xpass *xph = hdr_xpass::access(p);

  //tcph->seqno() = t_seqno_;
  //tcph->ackno() = recv_next_;
  //tcph->hlen() = xpass_hdr_size_;
  
  cmnh->size() = min_ethernet_size_;
  cmnh->ptype() = PT_XPASS_CREDIT_STOP;

  xph->credit_seq() = 0;

  return p;
}

Packet* XPassTcpAgent::construct_credit() {
  Packet *p = allocpkt();
  if (!p) {
    fprintf(stderr, "ERROR: allockpkt() failed\n");
    exit(1);
  }
  hdr_tcp *tcph = hdr_tcp::access(p);
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

  //tcph->seqno() = t_seqno_;
  //tcph->ackno() = recv_next_;
  //tcph->hlen() = credit_size;

  cmnh->size() = credit_size;
  cmnh->ptype() = PT_XPASS_CREDIT;

  xph->credit_sent_time() = now();
  xph->credit_seq() = c_seqno_;

  c_seqno_ = max(1, c_seqno_+1);

  fprintf(f_log, "%lf,%d,CREDIT_SENT\n", now(), fid_);
  fflush(f_log);
  return p;
}

Packet* XPassTcpAgent::construct_data(Packet *credit) {
  if (packet_queue_.size() == 0) {
    //fprintf(stderr, "ERROR: packet queue is already empty\n");
   // exit(1);
//  printf("Notice: Queue is empty, but received credit.\n");
    return NULL;
  }

  Packet *p = packet_queue_.front();

  hdr_tcp *tcph = hdr_tcp::access(p);
  hdr_cmn *cmnh = hdr_cmn::access(p);
  hdr_xpass *xph = hdr_xpass::access(p);
  hdr_xpass *credit_xph = hdr_xpass::access(credit);

  int datalen = cmnh->size() - tcph->hlen();
  if (datalen <= 0) {
    fprintf(stderr, "ERROR: datapacket has length of less than zero\n");
    exit(1);
  }

  if (datalen > max_ethernet_size_) {
    fprintf(stderr, "ERROR: datapacket exceeds max_segment size\n");
    exit(1);
  }
  //tcph->seqno() = t_seqno_;
  //tcph->ackno() = recv_next_;
  //tcph->hlen() = xpass_hdr_size_;
  // not calculate xpass_hdr_size (temp)
 //cmnh->size() = max(min_ethernet_size_, xpass_hdr_size_ + datalen);
  cmnh->ptype() = PT_XPASS_DATA;
 
  xph->credit_sent_time() = credit_xph->credit_sent_time();
  xph->credit_seq() = credit_xph->credit_seq();
  t_seqno_ += datalen;
 // printf("Sending datalen=%d, now t_seqno_ = %d\n", datalen, t_seqno_);
  packet_queue_.pop();
 // printf("Process Queue Item\n");
  return p;
}

Packet* XPassTcpAgent::construct_nack(seq_t seq_no) {
  Packet *p = allocpkt();
  if (!p) {
    fprintf(stderr, "ERROR: allockpkt() failed\n");
    exit(1);
  }
  hdr_tcp *tcph = hdr_tcp::access(p);
  hdr_cmn *cmnh = hdr_cmn::access(p);

  //tcph->ackno() = seq_no;
  //tcph->hlen() = xpass_hdr_size_; 

  cmnh->size() = min_ethernet_size_;
  cmnh->ptype() = PT_XPASS_NACK;

  return p;
}

void XPassTcpAgent::send_credit() {
  double avg_credit_size = (min_credit_size_ + max_credit_size_)/2.0;
  double delay;

  credit_feedback_control();

  // send credit.
  send(construct_credit(), 0);

  // calculate delay for next credit transmission.
  delay = avg_credit_size / cur_credit_rate_;
  // add jitter
  if (max_jitter_ > min_jitter_) {
    double jitter = (double)rand()/(double)RAND_MAX;
    jitter = jitter * (max_jitter_ - min_jitter_) + min_jitter_;
    // jitter is in the range between min_jitter_ and max_jitter_
    delay = delay*(1+jitter);
  }else if (max_jitter_ < min_jitter_) {
    fprintf(stderr, "ERROR: max_jitter_ should be larger than min_jitter_");
    exit(1);
  }

  send_credit_timer_.resched(delay);
}

void XPassTcpAgent::send_credit_stop() {
  send(construct_credit_stop(), 0);
  // set on timer
  sender_retransmit_timer_.resched(rtt_ > 0 ? (2. * rtt_) : default_credit_stop_timeout_);    
  credit_recv_state_ = XPASS_RECV_CREDIT_STOP_SENT; //Later changes to XPASS_RECV_CLOSE_WAIT -> XPASS_RECV_CLOSED
}

void XPassTcpAgent::update_rtt(Packet *pkt) {
  hdr_xpass *xph = hdr_xpass::access(pkt);

  double rtt = now() - xph->credit_sent_time();
  if (rtt_ > 0.0) {
    rtt_ = 0.8*rtt_ + 0.2*rtt;
  }else {
    rtt_ = rtt;
  }
}

void XPassTcpAgent::credit_feedback_control() {
  if (rtt_ <= 0.0) {
    return;
  }
  if ((now() - last_credit_rate_update_) < rtt_) {
    return;
  }
  if (credit_total_ == 0) {
    return;
  }

  int old_rate = cur_credit_rate_;
  double loss_rate = credit_dropped_/(double)credit_total_;
  double target_loss = (1.0 - cur_credit_rate_/(double)max_credit_rate_) * target_loss_scaling_;
  int min_rate = (int)(avg_credit_size() / rtt_);

  if (loss_rate > target_loss) {
    // congestion has been detected!
    if (loss_rate >= 1.0) {
      cur_credit_rate_ = (int)(avg_credit_size() / rtt_);
    } else {
      cur_credit_rate_ = (int)(avg_credit_size()*(credit_total_ - credit_dropped_)
                         / (now() - last_credit_rate_update_)
                         * (1.0+target_loss));
    }
    if (cur_credit_rate_ > old_rate) {
      cur_credit_rate_ = old_rate;
    }

    w_ = max(w_/2.0, min_w_);
    can_increase_w_ = false;
  }else {
    // there is no congestion.
    if (can_increase_w_) {
      w_ = min(w_ + 0.05, 0.5);
    }else {
      can_increase_w_ = true;
    }
    if (cur_credit_rate_ < max_credit_rate_) {
      cur_credit_rate_ = (int)(w_*max_credit_rate_ + (1-w_)*cur_credit_rate_);
    }
  }

  if (cur_credit_rate_ > max_credit_rate_) {
    cur_credit_rate_ = max_credit_rate_;
  }
  if (cur_credit_rate_ < min_rate) {
    cur_credit_rate_ = min_rate;
  }

  credit_total_ = 0;
  credit_dropped_ = 0;
  last_credit_rate_update_ = now();
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
    send(p, 0);
    //printf("Send RAW, len:%d, ACK:%d, SYN:%d, FIN:%d\n", datalen, pflags & TH_ACK, pflags & TH_SYN, pflags & TH_FIN);
  } else {
    //printf("Send ADV, len:%d, seqno:%d, t_seqno_:%d\n", datalen, seqno, t_seqno_);
    advance_packets(p);
  }
}


void XPassTcpAgent::advance_packets(Packet *p) {
// drop packet if exceed max_packet_queue
    // later change to numbering
 // hdr_tcp *tcph = hdr_tcp::access(p);
 // tcph->seqno() = seqno;


//  if (packet_queue_.size() < MAX_PACKET_QUEUE) {
    packet_queue_.push(p);
    //printf("Added to Packet Queue\n");
 // } else {
  //  printf("Dropped from Packet Queue\n");
  //  return;
 // }

  //curseq_ += p->datalen();

  switch (credit_recv_state_) {
    case XPASS_RECV_CREDIT_REQUEST_SENT:
    case XPASS_RECV_CREDIT_RECEIVING:
    case XPASS_RECV_CREDIT_STOP_SENT:
     return;
    case XPASS_RECV_CLOSE_WAIT:
    case XPASS_RECV_CLOSED:
      break;
  }
 // printf("Credit_recv_state: %d, curseq_ = %d, t_seqno_ = %d\n", credit_recv_state_, curseq_, t_seqno_); 
  // send credit request
  send(construct_credit_request(), 0);
  sender_retransmit_timer_.sched(retransmit_timeout_);

  // XPASS_RECV_CLOSED -> XPASS_RECV_CREDIT_REQUEST_SENT
  credit_recv_state_ = XPASS_RECV_CREDIT_REQUEST_SENT;

}
