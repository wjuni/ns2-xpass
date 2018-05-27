#include "xpass.h"
#include "../tcp/template.h"

#ifdef XPASS_CFC_BIC
#define ABS(a) ((a) >= 0 ? (a) : -(a))
#define UPDATE_WITH_LIMIT(target, value, minval, maxval) target = (ABS((target)-(value))>(maxval)? (((target) > (value)) ? (target)-(maxval) : (target) + (maxval) ) : (ABS((target)-(value)) < (minval) ? \
           ( ((target) > (value)) ? (target)-(minval) : (target)+(minval) ) : (value)))
#endif

int hdr_xpass::offset_;
static class XPassHeaderClass: public PacketHeaderClass {
public:
  XPassHeaderClass(): PacketHeaderClass("PacketHeader/XPass", sizeof(hdr_xpass)) {
    bind_offset (&hdr_xpass::offset_);
  }
} class_xpass_hdr;

static class XPassClass: public TclClass {
public:
  XPassClass(): TclClass("Agent/XPass") {}
  TclObject* create(int, const char*const*) {
    return (new XPassAgent());
  }
} class_xpass;

void SendCreditTimer::expire(Event *) {
  a_->send_credit();
}

void CreditStopTimer::expire(Event *) {
  a_->send_credit_stop();
}

void SenderRetransmitTimer::expire(Event *) {
  a_->handle_sender_retransmit();
}

void ReceiverRetransmitTimer::expire(Event *) {
  a_->handle_receiver_retransmit();
}

void FCTTimer::expire(Event *) {
  a_->handle_fct();
}

void XPassAgent::delay_bind_init_all() {
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
  delay_bind_init_one("early_credit_stop_");
  delay_bind_init_one("adaptive_initial_rate_");
  delay_bind_init_one("dynamic_target_loss_");
  delay_bind_init_one("initial_credit_rate_");
  delay_bind_init_one("exp_id_");
  delay_bind_init_one("credit_queue_count_");
  Agent::delay_bind_init_all();
}

int XPassAgent::delay_bind_dispatch(const char *varName, const char *localName,
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
  if (delay_bind(varName, localName, "early_credit_stop_", (&early_credit_stop_), tracer)) {
    return TCL_OK;
  }
  if (delay_bind(varName, localName, "adaptive_initial_rate_", (&adaptive_initial_rate_), tracer)) {
    return TCL_OK;
  }
  if (delay_bind(varName, localName, "dynamic_target_loss_", (&dynamic_target_loss_), tracer)) {
    return TCL_OK;
  }
  if (delay_bind(varName, localName, "initial_credit_rate_", &initial_credit_rate_, tracer)) {
    return TCL_OK;
  }
  if (delay_bind(varName, localName, "exp_id_", &exp_id_, tracer)) {
    return TCL_OK;
  }
  if (delay_bind(varName, localName, "credit_queue_count_", &credit_queue_count_, tracer)) {
    return TCL_OK;
  }
  return Agent::delay_bind_dispatch(varName, localName, tracer);
}

void XPassAgent::init() {
  w_ = w_init_;
#ifdef XPASS_CFC_BIC
  cur_credit_rate_ = (int)(alpha_ * initial_credit_rate_);
  bic_target_rate_ =  initial_credit_rate_/2;
#endif
  last_credit_rate_update_ = now();
}

int XPassAgent::command(int argc, const char*const* argv) {
  if (argc == 2) {
    if (strcmp(argv[1], "listen") == 0) {
      listen();
      return TCL_OK;
    } else if (strcmp(argv[1], "stop") == 0) {
      //on_transmission_ = false;
      return TCL_OK;
    }
  } else if (argc == 3) {
    if (strcmp(argv[1], "advance-bytes") == 0) {
      if (credit_recv_state_ == XPASS_RECV_CLOSED) {
        advance_bytes(atol(argv[2]));
        return TCL_OK;
      } else {
        return TCL_ERROR;
      }
    }
  }
  return Agent::command(argc, argv);
}

void XPassAgent::recv(Packet* pkt, Handler*) {
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
      break;
    case PT_XPASS_CREDIT_STOP:
      recv_credit_stop(pkt);
      break;
    case PT_XPASS_NACK:
      recv_nack(pkt);
      break;
    default:
      break;
  }
  Packet::free(pkt);
}

void XPassAgent::recv_credit_request(Packet *pkt) {
  hdr_xpass *xph = hdr_xpass::access(pkt);

  switch (credit_send_state_) {
    case XPASS_SEND_CLOSE_WAIT:
      fct_timer_.force_cancel();
    case XPASS_SEND_CLOSED:
      double lalpha;
      init();
      if (!adaptive_initial_rate_ || xph->sendbuffer_ >= 40) {
        lalpha = alpha_;
      } else {
        lalpha = alpha_ * xph->sendbuffer_ / 40.0;
      } 
      cur_credit_rate_ = (int)(lalpha * initial_credit_rate_);
      
      fst_ = xph->credit_sent_time();
      // need to start to send credits.
      send_credit();
        
      // XPASS_SEND_CLOSED -> XPASS_SEND_CREDIT_REQUEST_RECEIVED
      credit_send_state_ = XPASS_SEND_CREDIT_SENDING;     
      break;
  }
}

void XPassAgent::recv_credit(Packet *pkt) {
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
        send(construct_data(pkt), 0);
      }
 
      if (datalen_remaining() == 0) {
        if (credit_stop_timer_.status() != TIMER_IDLE) {
          fprintf(stderr, "Error: CreditStopTimer seems to be scheduled more than once.\n");
          exit(1);
        }
        // Because ns2 does not allow sending two consecutive packets, 
        // credit_stop_timer_ schedules CREDIT_STOP packet with no delay.
        credit_stop_timer_.sched(0);
      } else if (early_credit_stop_ && now() - last_credit_recv_update_ >= rtt_) {
        if (credit_recved_rtt_ >= pkt_remaining()*1 ) {
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

void XPassAgent::recv_data(Packet *pkt) {
  hdr_xpass *xph = hdr_xpass::access(pkt);
  seq_t new_seq = xph->credit_seq();

  //printf("new_seq: %d, c_recv_next_:%d, c_recv_next_queue_:%X, credit_total_:%d, credit_dropped:%d, filled:%d, now:%f \n", new_seq, c_recv_next_, c_recv_next_queue_, credit_total_, credit_dropped_, num_c_queue_filled_, now());
  // distance between expected sequence number and actual sequence number.
  int distance = xph->credit_seq() - c_recv_next_;

  if (distance < 0) {
    // credit packet reordering or credit sequence number overflow happend.
    printf("new_seq: %d, c_recv_next_:%d, credit_total_:%d, credit_dropped:%d \n", new_seq, c_recv_next_, credit_total_, credit_dropped_);
    return;
    fprintf(stderr, "ERROR: Credit Sequence number is reverted.\n");
    exit(1);
  }
 
  if (new_seq == c_recv_next_) {
    c_recv_next_ += 1;
    credit_total_ += 1;
    shift_c_seq_queue(1);
      printf("Filling queue at pos=%d, new_seq=%d, filled=%d, credit_total=%d, credit_dropped=%d, c_recv_next=%d\n", new_seq - c_recv_next_, new_seq, num_c_queue_filled_, credit_total_, credit_dropped_, c_recv_next_);
   while(c_recv_next_queue_ & 0x01) {
     shift_c_seq_queue(1);
     num_c_queue_filled_ -= 1;
     c_recv_next_ += 1;
     credit_total_ += 1;
     printf("Auto-emptying(1), filled=%d\n", num_c_queue_filled_);
   }
      
      process_ack(pkt);
  update_rtt(pkt);
    return;
  }

  int num_dropped = 0;
  // if there are credit burst drops
  if (new_seq - c_recv_next_ >= CREDIT_BURST_SIZE*2) {
   printf("CREDIT BURST CHECK : new_seq = %d, c_recv_next_ = %d, diff=%d, CBS=%d\n", new_seq, c_recv_next_, new_seq - c_recv_next_, CREDIT_BURST_SIZE*2);
    seq_t new_c_recv_next = new_seq + 1;//- CREDIT_BURST_SIZE*2 + 1;
    int jump = new_c_recv_next - c_recv_next_;
    if (jump > CREDIT_BURST_SIZE*2 - 1) {
       num_dropped = (new_c_recv_next - c_recv_next_ - num_c_queue_filled_);
       credit_dropped_ += num_dropped;
       credit_total_ += (num_dropped + 1);
       c_recv_next_ = new_c_recv_next;
       c_recv_next_queue_ = 0;
       num_c_queue_filled_ = 0;
   process_ack(pkt);
  update_rtt(pkt);
  printf("Credit dropped = %d, empty queue\n", credit_dropped_);
    return;
    }
    
    for(int i=0; i<jump; i++) {
      if(c_seq_queue_item(i)==0) {
        num_dropped++;
      }
    }
    credit_dropped_ += num_dropped;
    credit_total_ += (num_dropped + 1);
   
  printf("Credit dropped = %d\n", credit_dropped_);
    int c=0;
    for(int i=jump;i<CREDIT_BURST_SIZE*2;i++) {
      if(c_seq_queue_item(i)==0){
        c_recv_next_ += i;
        //jump++;
        c = 1;
        break;
      }
    }
    if (c==0) {
      c_recv_next_ += CREDIT_BURST_SIZE*2;
      jump = CREDIT_BURST_SIZE*2;
    }
    shift_c_seq_queue(jump);
    assert(!(0x00000001 << (new_seq - c_recv_next_)));
    c_recv_next_queue_ = c_recv_next_queue_ | (0x00000001 << (new_seq - c_recv_next_));
    cal_c_queue_filled();
  }
  else {
    assert(!(0x00000001 << (new_seq - c_recv_next_)));
    c_recv_next_queue_ = c_recv_next_queue_ | (0x00000001 << (new_seq - c_recv_next_));
    num_c_queue_filled_++;
    credit_total_++;
     printf("Filling queue at pos=%d, new_seq=%d, filled=%d, credit_total=%d, credit_dropped=%d, c_recv_next=%d\n", new_seq - c_recv_next_, new_seq, num_c_queue_filled_, credit_total_, credit_dropped_, c_recv_next_);
  }
 while(c_recv_next_queue_ & 0x01) {
     shift_c_seq_queue(1);
     num_c_queue_filled_ -= 1;
     c_recv_next_ += 1;
     credit_total_ += 1;
     printf("Auto-emptying(2), filled=%d, c_recv_next=%d\n", num_c_queue_filled_, c_recv_next_);
   }
  process_ack(pkt);
  update_rtt(pkt);
}

void XPassAgent::recv_nack(Packet *pkt) {
  hdr_tcp *tcph = hdr_tcp::access(pkt);
  switch (credit_recv_state_) {
    case XPASS_RECV_CREDIT_STOP_SENT:
    case XPASS_RECV_CLOSE_WAIT:
    case XPASS_RECV_CLOSED:
      if (datalen_remaining() > 0){
        send(construct_credit_request(), 0);
        credit_recv_state_ = XPASS_RECV_CREDIT_REQUEST_SENT;
        sender_retransmit_timer_.resched(retransmit_timeout_);
      } else {
        /////
      }
    case XPASS_RECV_CREDIT_REQUEST_SENT:
    case XPASS_RECV_CREDIT_RECEIVING:
      // set t_seqno_ for retransmission
      t_seqno_ = tcph->ackno();
  }
}

void XPassAgent::recv_credit_stop(Packet *pkt) {
  fct_ = now() - fst_;
  fct_timer_.sched(default_credit_stop_timeout_);
  send_credit_timer_.force_cancel();
  credit_send_state_ = XPASS_SEND_CLOSE_WAIT;
}

void XPassAgent::handle_fct() {
  char foname[40];
  sprintf(foname, "outputs/fct_%d.out", exp_id_);

  FILE *fct_out = fopen(foname,"a");

  fprintf(fct_out, "%d,%ld,%.10lf\n", fid_, recv_next_-1, fct_);
  fclose(fct_out);
  credit_send_state_ = XPASS_SEND_CLOSED;
}

void XPassAgent::handle_sender_retransmit() {
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
        char foname[40];
        sprintf(foname, "outputs/waste_%d.out", exp_id_);

        FILE *waste_out = fopen(foname,"a");

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

void XPassAgent::handle_receiver_retransmit() {
  if (wait_retransmission_) {
    send(construct_nack(recv_next_), 0);
    receiver_retransmit_timer_.resched(retransmit_timeout_);
  }
}

Packet* XPassAgent::construct_credit_request() {
  Packet *p = allocpkt();
  if (!p) {
    fprintf(stderr, "ERROR: allockpkt() failed\n");
    exit(1);
  }

  hdr_tcp *tcph = hdr_tcp::access(p);
  hdr_cmn *cmnh = hdr_cmn::access(p);
  hdr_xpass *xph = hdr_xpass::access(p);

  tcph->seqno() = t_seqno_;
  tcph->ackno() = recv_next_;
  tcph->hlen() = xpass_hdr_size_;

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

Packet* XPassAgent::construct_credit_stop() {
  Packet *p = allocpkt();
  if (!p) {
    fprintf(stderr, "ERROR: allockpkt() failed\n");
    exit(1);
  }
  hdr_tcp *tcph = hdr_tcp::access(p);
  hdr_cmn *cmnh = hdr_cmn::access(p);
  hdr_xpass *xph = hdr_xpass::access(p);

  tcph->seqno() = t_seqno_;
  tcph->ackno() = recv_next_;
  tcph->hlen() = xpass_hdr_size_;
  
  cmnh->size() = min_ethernet_size_;
  cmnh->ptype() = PT_XPASS_CREDIT_STOP;

  xph->credit_seq() = 0;

  return p;
}

Packet* XPassAgent::construct_credit() {
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

  tcph->seqno() = t_seqno_;
  tcph->ackno() = recv_next_;
  tcph->hlen() = credit_size;

  cmnh->size() = credit_size;
  cmnh->ptype() = PT_XPASS_CREDIT;

  xph->credit_sent_time() = now();
  xph->credit_seq() = c_seqno_;

  xph->cos() = randomize_cos(c_seqno_);
  c_seqno_ = max(1, c_seqno_+1);

  return p;
}

Packet* XPassAgent::construct_data(Packet *credit) {
  Packet *p = allocpkt();
  if (!p) {
    fprintf(stderr, "ERROR: allockpkt() failed\n");
    exit(1);
  }
  hdr_tcp *tcph = hdr_tcp::access(p);
  hdr_cmn *cmnh = hdr_cmn::access(p);
  hdr_xpass *xph = hdr_xpass::access(p);
  hdr_xpass *credit_xph = hdr_xpass::access(credit);
  int datalen = (int)min(max_segment(),
                         datalen_remaining());

  if (datalen <= 0) {
    fprintf(stderr, "ERROR: datapacket has length of less than zero\n");
    exit(1);
  }
  tcph->seqno() = t_seqno_;
  tcph->ackno() = recv_next_;
  tcph->hlen() = xpass_hdr_size_;

  cmnh->size() = max(min_ethernet_size_, xpass_hdr_size_ + datalen);
  cmnh->ptype() = PT_XPASS_DATA;

  xph->credit_sent_time() = credit_xph->credit_sent_time();
  xph->credit_seq() = credit_xph->credit_seq();
  
  t_seqno_ += datalen;

  return p;
}

Packet* XPassAgent::construct_nack(seq_t seq_no) {
  Packet *p = allocpkt();
  if (!p) {
    fprintf(stderr, "ERROR: allockpkt() failed\n");
    exit(1);
  }
  hdr_tcp *tcph = hdr_tcp::access(p);
  hdr_cmn *cmnh = hdr_cmn::access(p);

  tcph->ackno() = seq_no;
  tcph->hlen() = xpass_hdr_size_; 

  cmnh->size() = min_ethernet_size_;
  cmnh->ptype() = PT_XPASS_NACK;

  return p;
}

void XPassAgent::send_credit() {
  //double avg_credit_size = (min_credit_size_ + max_credit_size_)/2.0;
  double delay;
  credit_feedback_control();
  if(credit_cnt_timing_ % CREDIT_BURST_SIZE == 0) {
   // send credit.
    send(construct_credit(), 0);
    credit_cnt_++;

    if(credit_cnt_ % CREDIT_BURST_SIZE != 0) {
      // resend credit without delay
      send_credit_timer_.resched(0);
      return;
    }
  }

  // calculate delay for next credit transmission.
  delay = avg_credit_size() / cur_credit_rate_;
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
  credit_cnt_timing_++;
  send_credit_timer_.resched(delay);
}

void XPassAgent::send_credit_stop() {
  send(construct_credit_stop(), 0);
  // set on timer
  sender_retransmit_timer_.resched(rtt_ > 0 ? (2. * rtt_) : default_credit_stop_timeout_);    
  credit_recv_state_ = XPASS_RECV_CREDIT_STOP_SENT; //Later changes to XPASS_RECV_CLOSE_WAIT -> XPASS_RECV_CLOSED
}

void XPassAgent::advance_bytes(seq_t nb) {
  if(credit_recv_state_ != XPASS_RECV_CLOSED) {
    fprintf(stderr, "ERROR: tried to advance_bytes without XPASS_RECV_CLOSED\n");
  }
  if (nb <= 0) {
    fprintf(stderr, "ERROR: advanced bytes are less than or equal to zero\n");
  }

  // advance bytes
  curseq_ += nb;

  // send credit request
  send(construct_credit_request(), 0);
  sender_retransmit_timer_.sched(retransmit_timeout_);

  // XPASS_RECV_CLOSED -> XPASS_RECV_CREDIT_REQUEST_SENT
  credit_recv_state_ = XPASS_RECV_CREDIT_REQUEST_SENT;
}

void XPassAgent::process_ack(Packet *pkt) {
  hdr_cmn *cmnh = hdr_cmn::access(pkt);
  hdr_tcp *tcph = hdr_tcp::access(pkt);
  int datalen = cmnh->size() - tcph->hlen();
  if (datalen < 0) {
    fprintf(stderr, "ERROR: negative length packet has been detected.\n");
    exit(1);
  }
  if (tcph->seqno() > recv_next_) {
   ///// printf("[%d] %lf: data loss detected. (expected = %ld, received = %ld)\n",
   ////        fid_, now(), recv_next_, tcph->seqno());
    if (!wait_retransmission_) {
      wait_retransmission_ = true;
      send(construct_nack(recv_next_), 0);
      receiver_retransmit_timer_.resched(retransmit_timeout_);
    }
  } else if (tcph->seqno() == recv_next_) {
    if (wait_retransmission_) {
      wait_retransmission_ = false;
      receiver_retransmit_timer_.force_cancel();
    }
    recv_next_ += datalen;
  }
}

void XPassAgent::update_rtt(Packet *pkt) {
  hdr_xpass *xph = hdr_xpass::access(pkt);

  double rtt = now() - xph->credit_sent_time();
  if (rtt_ > 0.0) {
    rtt_ = 0.8*rtt_ + 0.2*rtt;
  }else {
    rtt_ = rtt;
  }
}

int XPassAgent::randomize_cos(int seqno) {
  srand(seqno);
  int temp = (int)(now()*10000);
  srand(rand()^temp);
//  return rand() % credit_queue_count_;
  return seqno % credit_queue_count_;
}

// set num_c_queue_filled as the number of 1 bits in c_recv_next_queue_
void XPassAgent::cal_c_queue_filled() {
  uint32_t temp = c_recv_next_queue_ - ((c_recv_next_queue_ >> 1) & 0x55555555);
  temp = (temp & 0x33333333) + ((temp >> 2) & 0x33333333);
  num_c_queue_filled_ = (((temp + (temp >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;
}

#ifdef XPASS_CFC_ORIGINAL
void XPassAgent::credit_feedback_control() {
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
    printf("Congestion detected. dropped credit = %d, credit_total = %d\n", credit_dropped_, credit_total_);
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
    printf("Congestion not detected. dropped credit = %d, credit_total = %d\n", credit_dropped_, credit_total_);
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
#endif

#ifdef XPASS_CFC_BIC
void XPassAgent::credit_feedback_control() {
  int old_rate = cur_credit_rate_;
  if (rtt_ <= 0.0) {
    return;
  }
  if ((now() - last_credit_rate_update_) < rtt_) {
    return;
  }
  if (credit_total_ == 0) {
    return;
  }

  double loss_rate = credit_dropped_/(double)credit_total_;
  int min_rate = (int)(avg_credit_size() / rtt_);
  double data_received_rate = 0;
  if (dynamic_target_loss_)
    bic_target_loss_ =  (1.0 - cur_credit_rate_/(double) initial_credit_rate_) * target_loss_scaling_;
  else
    bic_target_loss_ = target_loss_scaling_;

  if (loss_rate > bic_target_loss_){
    // congestion has been detected!
    if (loss_rate >= 1.0) {
      data_received_rate = (int)(avg_credit_size() / rtt_);
    } else {
      data_received_rate = (int)(avg_credit_size()*(credit_total_ - credit_dropped_)
          / (now() - last_credit_rate_update_) * (1. + bic_target_loss_));
    }
/*    if (bic_prev_credit_rate_ <= bic_target_rate_) {
      // normal situation
      UPDATE_WITH_LIMIT(bic_target_rate_, bic_prev_credit_rate_, bic_s_min_, bic_s_max_);
    } else {
      // double loss
      UPDATE_WITH_LIMIT(bic_target_rate_, data_received_rate, bic_s_min_, bic_s_max_);
    }
    UPDATE_WITH_LIMIT(cur_credit_rate_, (bic_target_rate_ + cur_credit_rate_)/2 , bic_s_min_, bic_s_max_);
*/
      bic_target_rate_ = cur_credit_rate_;
      if (cur_credit_rate_ > data_received_rate)
        UPDATE_WITH_LIMIT(cur_credit_rate_, data_received_rate, bic_s_min_, bic_s_max_);
  } else {
    // there is no congestion.
    if (bic_target_rate_ - cur_credit_rate_ <= 0.05 * bic_target_rate_) {
      // exponential
      if(cur_credit_rate_ < bic_target_rate_) {
        UPDATE_WITH_LIMIT(cur_credit_rate_, bic_target_rate_, bic_s_min_, bic_s_max_);
      } else {
        int new_rate = cur_credit_rate_ + (cur_credit_rate_ - bic_target_rate_) * (1.0+bic_increase_rate_);
        UPDATE_WITH_LIMIT(cur_credit_rate_, new_rate, bic_s_min_, bic_s_max_);
      }
    } else {
      // binary
      UPDATE_WITH_LIMIT(cur_credit_rate_, (cur_credit_rate_ + bic_target_rate_)/2 , bic_s_min_, bic_s_max_);
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
  bic_prev_credit_rate_ = old_rate;
}
#endif
