#include "xpass-drop-tail.h"
#include <algorithm>

static class XPassDropTailClass: public TclClass {
public:
  XPassDropTailClass(): TclClass("Queue/XPassDropTail") {}
  TclObject* create(int, const char*const*) {
    return (new XPassDropTail);
  }
} class_xpass_drop_tail;

void CreditTimer::expire (Event *) {
  a_->expire();
}

void XPassDropTail::expire() {
  Packet *p;
  double now = Scheduler::instance().clock();

  // The code below is excerpt from Queue::recv() function.
  if (!blocked_) {
    p = deque();
    if (p) {
      utilUpdate(last_change_, now, blocked_);
      last_change_ = now;
      blocked_ = 1;
      target_->recv(p, &qh_);
    }
  }
}

// Update token bucket with current wallclock.
void XPassDropTail::updateTokenBucket() {
  double now = Scheduler::instance().clock();
  double elapsed_time = now - token_bucket_clock_;
  int new_tokens;

  if (elapsed_time <= 0.0) {
    return;
  }

  new_tokens = (int)(elapsed_time * token_refresh_rate_);
  tokens_ += new_tokens;
  tokens_ = min(tokens_, max_tokens_);

  token_bucket_clock_ += new_tokens / token_refresh_rate_;
}

// Enqueue when a new packet has arrived.
void XPassDropTail::enque(Packet* p) {
  if (p == NULL) {
    return;
  }
  // parsing headers.
  hdr_cmn* cmnh = hdr_cmn::access(p);
  hdr_xpass *xph = hdr_xpass::access(p);
  int cos = xph->cos();
  // enqueue packet: store and forward.
  if (cmnh->ptype() == PT_XPASS_CREDIT) {
    // p is credit packet.
    credit_q_[cos].enque(p);
    if (credit_q_[cos].byteLength() > credit_q_limit_) {
       printf("Credit drop from queue=%d, bytelen=%d, q_lim=%d\n", cos,
          credit_q_[cos].byteLength(), credit_q_limit_);
      credit_q_[cos].remove(p);
      drop(p);
   }
  }else {
    // p is data packet.
    data_q_->enque(p);
    if (data_q_->byteLength() > data_q_limit_) {
      printf("Data packet will be dropped.\n");
      data_q_->remove(p);
      drop(p);
    }
  }
}

// Dequeue the packets.
// Data has higher priority than credit packets.
Packet* XPassDropTail::deque() {
  Packet* packet = NULL;

  credit_timer_.force_cancel();
  updateCreditQueue();
  updateTokenBucket();

  // Credit packet
  packet = credit_q_[c_queue_num_].head();
  if (packet && tokens_ >= hdr_cmn::access(packet)->size()) {
    // Credit packet should be forwarded.
    packet = credit_q_[c_queue_num_].deque();
    tokens_ -= hdr_cmn::access(packet)->size();
    return packet;
  }
  
  // Data packet
  if (data_q_->length() > 0) {
    // Data packet should be forwarded.
    packet = data_q_->deque();
    return packet;
  }

  // Switch cannot forward credit nor data.
  // If there is credit in credit queue, set timer for the next credit.
  if (packet) {
    double delay = (hdr_cmn::access(packet)->size() - tokens_) / token_refresh_rate_;
    credit_timer_.resched(delay);
  }else if (credit_q_[c_queue_num_].byteLength() > 0 && data_q_->byteLength() > 0) {
    fprintf(stderr,"Switch has non-zero queue, but timer was not set.\n");
    exit(1);
  }

  return NULL;
}

void XPassDropTail::updateCreditQueue() {
  double now = Scheduler::instance().clock();
  double elapsed_time = now - c_queue_clock_;

  if(elapsed_time > 0.00001) {
    c_queue_num_ = (c_queue_num_+1) % credit_queue_count_;
    c_queue_clock_ = now;
  }
  for(int i=0; i<credit_queue_count_; i++) {
    if(credit_q_[(c_queue_num_ + i)%credit_queue_count_].length() != 0) {
      break;
    }
    c_queue_num_ = (c_queue_num_+1) % credit_queue_count_;
    c_queue_clock_ = now;
  }
}
