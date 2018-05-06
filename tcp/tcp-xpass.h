#ifndef ns_tcp_xpass_h
#define ns_tcp_xpass_h

#include "../xpass/xpass.h"
#include "tcp-full.h"
#include <queue>
#define MAX_PACKET_QUEUE 64

class XPassQueueAgent;

class XPassTcpAgent : public FullTcpAgent {
public:
  XPassTcpAgent() : FullTcpAgent() { }
  ~XPassTcpAgent() { }
  virtual void recv(Packet *pkt, Handler*);
protected:
	virtual void delay_bind_init_all();
	virtual int delay_bind_dispatch(const char *varName, const char *localName, TclObject *tracer);

  void sendpacket(seq_t seqno, seq_t ackno, int pflags, int datalen, int reason, Packet *p);
  XPassQueueAgent* xpass_agent_;
};

class XPassQueueAgent : public XPassAgent {
public:
  std::queue<Packet *> packet_queue_;
  XPassQueueAgent() : XPassAgent() {}
  bool recv_bool(Packet *pkt);
  void recv_data(Packet *pkt); 
  void advance_packets(Packet *p);
  Packet* construct_credit_request();
  Packet* construct_credit_stop();
  Packet* construct_credit();
  Packet* construct_data(Packet *credit); 
  Packet* construct_nack(seq_t seq_no); 
};

#endif
