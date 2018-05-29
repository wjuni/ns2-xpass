/* -*-  Mode:C++; c-basic-offset:4; tab-width:8; indent-tabs-mode:t -*- */
/*
 * Copyright (c) 2000  International Computer Science Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by ACIRI, the AT&T 
 *      Center for Internet Research at ICSI (the International Computer
 *      Science Institute).
 * 4. Neither the name of ACIRI nor of ICSI may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ICSI AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL ICSI OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef lint
static const char rcsid[] =
    "@(#) $Header: /cvsroot/nsnam/ns-2/queue/red-pd.cc,v 1.7 2002/01/01 00:05:54 sfloyd Exp $ (ACIRI)";
#endif

#include "red-xpass.h"
#include <algorithm>

static class RedXPassClass : public TclClass {
public:
	RedXPassClass() : TclClass("Queue/RED/XPass") {}
	TclObject* create(int argc, const char*const* argv) {
		//		printf("creating REDPD %d\n", argc);
		if (argc==4) {
			return (new RedXPassQueue("Drop"));
		}
		else  {
			char args[100];
			strcpy(args, argv[4]);
			//strtok used for compatibility reasons
			char * arg1 = strtok(args," ");
			return (new RedXPassQueue(arg1));
		}
		
	}
} red_xpass_class;

void RXCreditTimer::expire (Event *) {
  a_->expire();
}

void RedXPassQueue::expire() {
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
void RedXPassQueue::updateTokenBucket() {
  double now = Scheduler::instance().clock();
  double elapsed_time = now - token_bucket_clock_;
  int64_t new_tokens;

  if (elapsed_time <= 0.0) {
    return;
  }

  new_tokens = (int64_t)(elapsed_time * token_refresh_rate_);
  tokens_ += new_tokens;
  tokens_ = min(tokens_, max_tokens_);

  token_bucket_clock_ += new_tokens / token_refresh_rate_;
}

// Enqueue when a new packet has arrived.
void RedXPassQueue::enque(Packet* p) {
  if (p == NULL) {
    return;
  }
  // parsing headers.
  hdr_cmn* cmnh = hdr_cmn::access(p);

  // enqueue packet: store and forward.
  if (cmnh->ptype() == PT_XPASS_CREDIT) {
    // p is credit packet.
    credit_q_->enque(p);
    if (credit_q_->byteLength() > credit_q_limit_) {
      credit_q_->remove(p);
      drop(p);
    }
  } else {
    // p is data packet.
    REDQueue::enque(p);
  }
}

// Dequeue the packets.
// Data has higher priority than credit packets.
Packet* RedXPassQueue::deque() {
  Packet* packet = NULL;

  credit_timer_.force_cancel();
  updateTokenBucket();

  // Credit packet
  packet = credit_q_->head();
  if (packet && tokens_ >= hdr_cmn::access(packet)->size()) {
    // Credit packet should be forwarded.
    packet = credit_q_->deque();
    tokens_ -= hdr_cmn::access(packet)->size();
    return packet;
  }
  
  // Data packet
  if (q_->length() > 0) {
    // Data packet should be forwarded.
    packet =  REDQueue::deque();
    return packet;
  }

  // Switch cannot forward credit nor data.
  // If there is credit in credit queue, set timer for the next credit.
  if (packet) {
    double delay = (hdr_cmn::access(packet)->size() - tokens_) / token_refresh_rate_;
    credit_timer_.resched(delay);
  }else if (credit_q_->byteLength() > 0 && q_->byteLength() > 0) {
    fprintf(stderr,"Switch has non-zero queue, but timer was not set.\n");
    exit(1);
  }

  return NULL;
}
