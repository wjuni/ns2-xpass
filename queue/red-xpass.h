/* -*-  Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*- */
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
 *
 * @(#) $Header: /cvsroot/nsnam/ns-2/queue/red-pd.h,v 1.4 2001/01/10 23:30:14 sfloyd Exp $ (ACIRI)
 */


#ifndef ns_red_xpass_h
#define ns_red_xpass_h

#include "red.h"

class RedXPassQueue;
class RXCreditTimer: public TimerHandler {
public:
  RXCreditTimer(RedXPassQueue *a): TimerHandler(), a_(a) {}
protected:
  virtual void expire (Event *);
  RedXPassQueue *a_;
};

class RedXPassQueue : public REDQueue {
 public:	
  RedXPassQueue(const char * edtype): REDQueue(edtype),
	credit_timer_(this) {
    credit_q_ = new PacketQueue;

    // bind with TCL
    bind("credit_limit_", &credit_q_limit_);
    bind("data_limit_", &data_q_limit_);
    bind("max_tokens_", &max_tokens_);
    bind("token_refresh_rate_", &token_refresh_rate_);

    // Init variables
    tokens_ = 0;
    token_bucket_clock_ = 0;
  }
  
  ~RedXPassQueue() {
    delete credit_q_;
    
  }
  void expire();


protected:
  void enque(Packet*);
  Packet* deque();
  void updateTokenBucket();
    // Queue Related Varaibles
  //
  // Queue for data packets (High priority)
  PacketQueue *data_q_;
  // Queue for credit packets (Low priority)
  PacketQueue *credit_q_;
  // Maximum size of credit queue (in bytes)
  int credit_q_limit_;
  // Maximum size of data queue (in bytes)
  int data_q_limit_;

  // Token Bucket Related Varaibles
  //
  // Number of tokens remaning (in bytes)
  int64_t tokens_;
  // Maximum number of tokens (in bytes)
  int64_t max_tokens_;
  // Token Bucket clock
  double token_bucket_clock_;
  // Token Refresh Rate (in bytes per sec)
  // == Credit Throttling Rate
  double token_refresh_rate_;

  // Credit timer to trigger next credit transmission
  RXCreditTimer credit_timer_;
};

#endif
