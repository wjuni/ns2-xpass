set ns [new Simulator]

if {$argc < 1} {
  puts "USAGE: ./ns scripts/dumbell-topology.tcl {expriment_id} {N}"
  exit 1
}

# Configurations
#set N 10
set K 1
set B 250
set B_host 1000
set ALPHA 0.5
set w_init 0.0625
set linkBW 10Gb
set inputlinkBW 10Gb
set linkLatency 10us ;# 5us
set creditQueueCapacity [expr 84*10]  ;# bytes
set dataQueueCapacity [expr 1538*$B] ;# bytes
set hostQueueCapacity [expr 1538*$B_host] ;# bytes
set maxCrditBurst [expr 84*2] ;# bytes
set baseCreditRate 64734895
set creditRate 64734895
set interFlowDelay 0.000 ;# secs
set expID [expr int([lindex $argv 0])]
set N [expr int([lindex $argv 1])]

# Output file
file mkdir "outputs"
set nt [open outputs/trace_$expID.out w]
set fct_out [open outputs/fct_$expID.out w]
puts $fct_out "Flow ID,Flow Size (bytes),Flow Completion Time (secs)"
close $fct_out

set vt [open outputs/var_$expID.out w]


set packetSize 1454
proc finish {} {
  global ns nt vt sender N
  $ns flush-trace 
  for {set i 1} {$i < $N} {incr i} {
   # $sender($i) flush-trace
  }
  close $nt
  close $vt
  puts "Simulation terminated successfully."
  exit 0
}
$ns trace-all $nt

puts "Creating Nodes..."
set left_gateway [$ns node]
set right_gateway [$ns node]

for {set i 0} {$i < $N} {incr i} {
  set left_node($i) [$ns node]
}

for {set i 0} {$i < $N} {incr i} {
  set right_node($i) [$ns node]
}

puts "Creating Links..."
Queue/DropTail set mean_pktsize_ 1538
Queue/DropTail set limit_ $B_host

#Credit Setting
Queue/XPassDropTail set credit_limit_ $creditQueueCapacity
Queue/XPassDropTail set data_limit_ $dataQueueCapacity
Queue/XPassDropTail set token_refresh_rate_ $creditRate


Queue/XPassRED set credit_limit_ $creditQueueCapacity
Queue/XPassRED set max_tokens_ [expr 2*84] 
Queue/XPassRED set token_refresh_rate_ $creditRate
#RED Setting
Queue/XPassRED set bytes_ true
Queue/XPassRED set queue_in_bytes_ true
Queue/XPassRED set mean_pktsize_ 1538
Queue/XPassRED set setbit_ true
Queue/XPassRED set gentle_ false
Queue/XPassRED set q_weight_ 1.0
Queue/XPassRED set use_mark_p true
Queue/XPassRED set mark_p_ 2.0
Queue/XPassRED set thresh_ $K
Queue/XPassRED set maxthresh_ $K
Queue/XPassRED set limit_ $B

Agent/TCP set ecn_ 1
Agent/TCP set old_ecn_ 1
Agent/TCP set dctcp_ true
Agent/TCP set dctcp_g_ 0.0625
Agent/TCP set ecn_syn_ true


Queue/RED set bytes_ false
Queue/RED set queue_in_bytes_ true
Queue/RED set mean_pktsize_ $packetSize
Queue/RED set setbit_ true
Queue/RED set gentle_ false
Queue/RED set q_weight_ 1.0

Queue/RED set use_mark_p true
Queue/RED set mark_p_ 2.0

Queue/RED set thresh_ [expr $K]
Queue/RED set maxthresh_ [expr $K]



for {set i 0} {$i < $N} {incr i} {
  $ns simplex-link $left_node($i) $left_gateway $inputlinkBW $linkLatency DropTail
  $ns simplex-link $left_gateway $left_node($i) $inputlinkBW $linkLatency XPassRED

  $ns simplex-link $right_node($i) $right_gateway $inputlinkBW $linkLatency DropTail
  $ns simplex-link $right_gateway $right_node($i) $inputlinkBW $linkLatency XPassRED
}

$ns duplex-link $left_gateway $right_gateway $linkBW $linkLatency XPassRED

puts "Creating Agents..."
Agent/XPass set min_credit_size_ 84
Agent/XPass set max_credit_size_ 84
Agent/XPass set min_ethernet_size_ 84
Agent/XPass set max_ethernet_size_ 1538
Agent/XPass set max_credit_rate_ $creditRate
Agent/XPass set base_credit_rate_ $baseCreditRate
Agent/XPass set target_loss_scaling_ 0.125
Agent/XPass set alpha 1.0
Agent/XPass set w_init 0.0625
Agent/XPass set min_w_ 0.01
Agent/XPass set retransmit_timeout_ 0.0001
Agent/XPass set min_jitter_ -0.1
Agent/XPass set max_jitter_ 0.1
Agent/XPass set exp_id_ $expID

Agent/TCP set packetSize_ 1454
Agent/TCP set window_ 180
Agent/TCP/FullTcp set nodelay_ true
Agent/TCP set slow_start_restart_ false
Agent/TCP set tcpTick_ 0.000001
Agent/TCP set minrto_ 0.0004
Agent/TCP set rtxcur_init_ 0.001
Agent/TCP set windowOption_ 0
Agent/TCP set ssthres_ 1
Agent/TCP set max_ssthresh_ 1
Agent/TCP set tcpip_base_hdr_size_ 84

Agent/TCP/FullTcp set segsize_ 1454
Agent/TCP/FullTcp set segsperack_ 1
Agent/TCP/FullTcp set spa_thresh_ 3000
Agent/TCP/FullTcp set interval_ 0
Agent/TCP/FullTcp set state_ 0

DelayLink set avoidReordering_ true

for {set i 0} {$i < $N} {incr i} {
  if {$i < 1} {
    puts "XPass Agent"
    set sender($i) [new Agent/XPass]
    set receiver($i) [new Agent/XPass]

    $receiver($i) attach $vt
     $receiver($i) trace cur_credit_rate_tr_
     $receiver($i) listen
  } else {
    puts "TCP Agent"
    set sender($i) [new Agent/TCP/FullTcp/Sack]
    set receiver($i) [new Agent/TCP/FullTcp/Sack]
     $sender($i) attach $vt
    $sender($i) trace cwnd_
   $receiver($i) listen
  }

  $ns attach-agent $left_node($i) $sender($i)
  $ns attach-agent $right_node($i) $receiver($i)

  $sender($i) set fid_ [expr $i]
  $receiver($i) set fid_ [expr $i]

  $ns connect $sender($i) $receiver($i)
}

puts "Simulation started."
set nextTime 0.0
#for {set i 0} {$i < $N} {incr i} {
#  $ns at $nextTime "$sender($i) advance-bytes 100000000"
#  set nextTime [expr $nextTime + $interFlowDelay]
#}

#$ns at 0.1 "$sender(0) advance-bytes 200000000" 
$ns at 0.00 "$sender(1) advance-bytes 1000000000" 
$ns at 0.40 "$sender(0) advance-bytes 1000000000" 

$ns at 1.0 "finish"
$ns run
