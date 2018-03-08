set numCore 16
set numAggr 32
set numTor 32
set numNode 128

set N 2048

set ns [new Simulator]
set nt [open traces/out_dctcp_based.tr w]

set REQUEST_SIZE 200
set linkBW 10Gb
set linkHostBW 11Gb
set linkBWReal [expr 10*1000*1000*1000]
set linkDelay 0.000005 ;# 5us
set RTT 0.00005 ;# 50us
set maxRTT [expr $linkDelay*12]
set B 10000
set delay [expr $REQUEST_SIZE*8.0/$linkBWReal]

set hostAlg DropTail
set switchAlg RED

set DCTCP_g_ 0.0625
set K 65
set ackRatio 1
set packetSize 1454

Agent/TCP set ecn_ 1
Agent/TCP set old_ecn_ 1
Agent/TCP set dctcp_ true
Agent/TCP set dctcp_g_ 0.0625
Agent/TCP set ecn_syn_ true
Agent/TCP set window_ 180
Agent/TCP set slow_start_restart_ false
Agent/TCP set packetSize 1454
Agent/TCP set tcpTick_ 0.000001
Agent/TCP set minrto_ 0.0002
Agent/TCP set windowOption_ 0
Agent/TCP set ssthresh_ 1
Agent/TCP set max_ssthresh_ 1
Agent/TCP set tcpip_base_hdr_size_ 84

Agent/TCP/FullTcp set nodelay_ true
Agent/TCP/FullTcp set segsize_ 1454
Agent/TCP/FullTcp set segsperack_ 1
Agent/TCP/FullTcp set interval_ 0
Agent/TCP/FullTcp set state_ 0

Queue/RED set bytes_ false
Queue/RED set queue_in_bytes_ true
Queue/RED set mean_pktsize_ 1538
Queue/RED set setbit_ true
Queue/RED set gentle_ flase
Queue/RED set q_weight_ 1.0

Queue/RED set use_mark_p true
Queue/RED set mark_p_ 2.0

Queue/RED set thresh_ $K
Queue/RED set maxthresh_ $K

Queue/REDTrace set bytes_ false
Queue/REDTrace set queue_in_bytes_ true
Queue/REDTrace set mean_pktsize_ 1538
Queue/REDTrace set setbit_ true
Queue/REDTrace set gentle_ flase
Queue/REDTrace set q_weight_ 1.0

Queue/REDTrace set use_mark_p true
Queue/REDTrace set mark_p_ 2.0

Queue/REDTrace set thresh_ $K
Queue/REDTrace set maxthresh_ $K

Queue set limit_ 2000
DelayLink set avoidReordering_ true

set rng0 [new RNG]
$rng0 seed 0

set rng1 [new RNG]
$rng1 seed 43259601

set u [new RandomVariable/Uniform]
$u use-rng $rng0
$u set min_ 0
$u set max_ [expr $maxRTT*100]

set rn [new RandomVariable/Uniform]
$rn use-rng $rng1
$rn set min_ 1
$rn set max_ [expr $numNode]

set e [new RandomVariable/Exponential]
$e use-rng $rng1
$e set avg_ 0.02

$ns trace-all $nt

proc finish {} {
    global ns nt
    $ns flush-trace
    close $nt
    exit 0
}

# construct fat-tree network
$ns rtproto DV
Agent/rtProto/DV set advertInterval 50
Node set multiPath_ 1
Classifier/MultiPath set perflow_ 1

# nodes
for {set i 0} {$i < $numNode} {incr i} {
    set dcNode($i) [$ns node]
}
for {set i 0} {$i < $numTor} {incr i} {
    set dcTor($i) [$ns node]
}
for {set i 0} {$i < $numAggr} {incr i} {
    set dcAggr($i) [$ns node]
}
for {set i 0} {$i < $numCore} {incr i} {
    set dcCore($i) [$ns node]
}

# links
for {set i 0} {$i < $numAggr} {incr i} {
    set coreIndex [expr ($i%4)*4]
    for {set j $coreIndex} {$j < $coreIndex+4} {incr j} {
        $ns duplex-link $dcAggr($i) $dcCore($j) $linkBW $linkDelay RED
        $ns queue-limit $dcAggr($i) $dcCore($j) $B
        $ns queue-limit $dcCore($j) $dcAggr($i) $B
    }
}

for {set i 0} {$i < $numTor} {incr i} {
    set aggrIndex [expr $i/4*4]
    for {set j $aggrIndex} {$j < $aggrIndex+4} {incr j} {
        $ns duplex-link $dcTor($i) $dcAggr($j) $linkBW $linkDelay RED
        $ns queue-limit $dcTor($i) $dcAggr($j) $B
        $ns queue-limit $dcAggr($j) $dcTor($i) $B
    }
}

$ns simplex-link $dcNode(0) $dcTor(0) $linkHostBW $linkDelay DropTail
$ns simplex-link $dcTor(0) $dcNode(0) $linkBW $linkDelay REDTrace
$ns queue-limit $dcNode(0) $dcTor(0) $B
$ns queue-limit $dcTor(0) $dcNode(0) $B

$ns trace-queue $dcTor(0) $dcNode(0) $nt

for {set i 1} {$i < $numNode} {incr i} {
    set torIndex [expr $i/4]
    $ns simplex-link $dcNode($i) $dcTor($torIndex) $linkHostBW $linkDelay DropTail
    $ns simplex-link $dcTor($torIndex) $dcNode($i) $linkBW $linkDelay RED
    $ns queue-limit $dcNode($i) $dcTor($torIndex) $B
    $ns queue-limit $dcTor($torIndex) $dcNode($i) $B
}

for {set i 0} {$i < $numNode-1} {incr i} {lappend nums $i}
for {set i 0} {$i < $numNode-1} {incr i} {
    set j [expr int(rand()*[expr $numNode-1])]
    set temp [lindex $nums $j]
    set nums [lreplace $nums $j $j [lindex $nums $i]]
    set nums [lreplace $nums $i $i $temp]
}

for {set i 0} {$i < $N} {incr i} {
    set receiver($i) [new Agent/TCP/FullTcp/Sack]
	$ns attach-agent $dcNode(0) $receiver($i)
	$receiver($i) set fid_ [expr $i+1]
	$receiver($i) listen
}

for {set i 0} {$i < $N} {incr i} {
    set sidx [lindex $nums [expr $i%($numNode-1)]]
    set sender($i) [new Agent/TCP/FullTcp/Sack]

    $ns attach-agent $dcNode($sidx) $sender($i)

    $sender($i) set fid_ [expr $i+1]

    $ns connect $sender($i) $receiver($i)
}

set tt [expr 0.05 + $delay]
set i 0
while {$tt < 0.3} {
  $ns at $tt "$sender($i) advance-bytes 1000"
  set tt [expr $tt+$delay]
  set i [expr ($i+1)%$N]
}


$ns at 0.3 "finish"
$ns run
