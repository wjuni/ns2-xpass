set N 1 
set B 250
set B_host 1000

set switchAlg RED/XPass
set lineRate 10Gb
set inputlineRate 10Gb
set RTT 0.00001

set DCTCP_g_ 0.0625
set K 65
set ackRatio 1 
set packetSize 1454
 
set traceSamplingInterval 0.0001
set throughputSamplingInterval 0.01

set ns [new Simulator]

set nt [open traces/out_dctcp.tr w]
set winfile [open traces/win.tr w]
set nq [open traces/queue_dctcp.tr w]

Agent/TCP set ecn_ 1
Agent/TCP set old_ecn_ 1
Agent/TCP set packetSize_ $packetSize
Agent/TCP/FullTcp set segsize_ $packetSize
#Agent/TCP set window_ 2512 
Agent/TCP set window_ 1256
Agent/TCP set slow_start_restart_ false
Agent/TCP set tcpTick_ 0.000001
Agent/TCP set minrto_ 0.0004 ; # minRTO = 200ms
Agent/TCP set rtxcur_init_ 0.001
Agent/TCP set windowOption_ 0
Agent/TCP set tcpip_base_hdr_size_ 84
#Agent/TCP set timestamps_ true
#Agent/TCP set windowInit_ 75

Agent/TCP set dctcp_ false
Agent/TCP set dctcp_g_ $DCTCP_g_;
Agent/TCP set maxcwnd_ 1500

Agent/TCP/FullTcp set segsperack_ $ackRatio; 
Agent/TCP/FullTcp set spa_thresh_ 3000;
Agent/TCP/FullTcp set interval_ 0 ; #delayed ACK interval = 40ms
Agent/TCP/FullTcp set nodelay_ true;
Agent/TCP/FullTcp set state_ 0

Queue set limit_ $B

Queue/RED/XPass set bytes_ false
Queue/RED/XPass set queue_in_bytes_ true
Queue/RED/XPass set mean_pktsize_ $packetSize
Queue/RED/XPass set setbit_ true
Queue/RED/XPass set gentle_ false
Queue/RED/XPass set q_weight_ 1.0
Queue/RED/XPass set mark_p_ 1.0
Queue/RED/XPass set thresh_ [expr $K]
Queue/RED/XPass set maxthresh_ [expr $K]
			 
DelayLink set avoidReordering_ true

$ns trace-all $nt

set rng0 [new RNG]
$rng0 seed 94762103

#set e [new RandomVariable/Exponential]
#$e use-rng $rng0
#$e set avg_ 0.00435513383

set e [new RandomVariable/Uniform]
$e use-rng $rng0
$e set min_ 0
$e set max_ 0.01

proc finish {} {
    global ns nt namfile winfile nq
    $ns flush-trace
	close $nt
	close $winfile
	close $nq
	exit 0
}

set left_gw [$ns node]
set right_gw [$ns node]
for {set i 0} {$i < $N} {incr i} {
	set left_n($i) [$ns node]
}

for {set i 0} {$i < $N} {incr i} {
    set right_n($i) [$ns node]
}

for {set i 0} {$i < $N} {incr i} {
    $ns duplex-link $left_n($i) $left_gw $inputlineRate [expr $RTT/6] DropTail
    $ns duplex-link $left_gw $left_n($i) $inputlineRate [expr $RTT/6] DropTail;#RED/XPass

	$ns queue-limit $left_n($i) $left_gw $B
	$ns queue-limit $left_gw $left_n($i) $B

    $ns duplex-link $right_n($i) $right_gw $inputlineRate [expr $RTT/6] DropTail
    $ns duplex-link $right_gw $right_n($i) $inputlineRate [expr $RTT/6] DropTail;#RED/XPass
	$ns queue-limit $right_n($i) $right_gw $B
	$ns queue-limit $right_gw $right_n($i) $B
}

$ns simplex-link $left_gw $right_gw $lineRate [expr $RTT/6] DropTail;#RED/XPass
$ns simplex-link $right_gw $left_gw $lineRate [expr $RTT/6] DropTail;#RED/XPass
set templ [$ns link $right_gw $left_gw]
set tempq [$templ queue]
$tempq set trace_ 1
$ns queue-limit $left_gw $right_gw $B
$ns queue-limit $right_gw $left_gw $B

$ns trace-queue $left_gw $right_gw $nt

for {set i 0} {$i < $N} {incr i} { 
  if {$i % 2} {
    # send xpass-dctcp when i=1 
    set tcp($i) [new Agent/TCP/FullTcp/XPass]
	  set sink($i) [new Agent/TCP/FullTcp/XPass]
  } else {
    # send dctcp when i=0
    set tcp($i) [new Agent/TCP/FullTcp]
	  set sink($i) [new Agent/TCP/FullTcp]
  }
  
  $sink($i) listen

    $ns attach-agent $left_n($i) $tcp($i)
    $ns attach-agent $right_n($i) $sink($i)
    
    $tcp($i) set fid_ [expr $i]
    $sink($i) set fid_ [expr $i]

    $ns connect $tcp($i) $sink($i)       
}

proc plotWindow {file} {
	global ns tcp N
	set ttime 0.0001
	set now [$ns now]
	for {set i 0} {$i < $N} {incr i} {
		set cwnd($i) [$tcp($i) set cwnd_]
	}
	puts $file "$now $cwnd(0) $cwnd(1) $cwnd(2)"
	$ns at [expr $now+$ttime] "plotWindow $file"
}

set qmon [$ns monitor-queue $right_gw $left_gw $nq 0.0001]

proc plotQueue {qmonitor file} {
	global ns packetSize
	set ttime 0.0001
	set now [$ns now]
	set qsize [$qmonitor set size_]
	puts $file "$now $qsize"
	$ns at [expr $now+$ttime] "plotQueue $qmonitor $file"
}

proc printWindow {} {
	global ns tcp RTT
	set now [$ns now]
	puts "$now [$tcp(0) set cwnd_] [$tcp(1) set cwnd_]"
	$ns at [expr $now+$RTT] "printWindow"
}

set tt 0.0
set interFlowDelay 0.01
set nextTime 0.0
for {set i 0} {$i < $N} {incr i} {
  set tt [expr $tt + [$e value]]
	$ns at $nextTime "$tcp($i) advance-bytes 1000000000"
  set nextTime [expr $nextTime + $interFlowDelay]
}


#$ns at 0.0001 "plotQueue $qmon $nq"

#set tt 0.0
#
#for {set i 0} {$i < 100} {incr i} {
#	set tt [expr $tt + [$e value]]
#	$ns at $tt "$tcp($i) advance-bytes 1000000000"
#}
#
#set tt 0.1
#
#for {set i 0} {$i < 1000} {incr i} {
#	set tt [expr $tt + [$e value]]
#	$ns at $tt "$tcp([expr $i+100]) advance-bytes 100000"
#}

#$ns at 0.0 "$tcp(0) advance-bytes 11505"
#$ns at 0.09 "$sink(0) listen"
#$ns at 0.1 "$tcp(0) advance-bytes 11505"
#$ns at 0.6 "$tcp(3) advance-bytes 715058000"
#$ns at 0.8 "$tcp(4) advance-bytes 715058000"
#$ns at 0.05 "printWindow"
#$ns at 0.002 "$tcp(2) advance-bytes 11505800"
#$ns at 0.1 "$tcp(1) advance-bytes 215058000"
#$ns at 0.2 "$tcp(2) advance-bytes 215058000"
#$ns at 0.4 "$tcp(1) advance-bytes 1150580000"
#$ns at 0.8 "$tcp(2) advance-bytes 1150580000"
#$ns at 0.2 "$tcp(1) advance-bytes 1000000"
#$ns at 0.4 "$tcp(2) advance-bytes 2000000"
#$ns at 0.6 "$tcp(3) advance-bytes 4000000"
#$ns at 0.2 "$ftp(1) start"
#$ns at 0.0001 "plotWindow $winfile"
#$ns at 0.4 "$ftp(1) start"
#$ns at 0.8 "$ftp(2) start"
#$ns at $RTT "printWindow"
$ns at 100.0 "finish"
$ns run
