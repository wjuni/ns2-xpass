set ns [new Simulator]

if {$argc < 2} {
  puts "USAGE: ./ns scripts/dumbell-topology.tcl {N} {exp_id}"
  exit 1
}

# Configurations
set N [expr int([lindex $argv 0])]
set ALPHA 0.5
set w_init 0.5
set linkBW 10Gb
set inputlinkBW 10Gb
set linkLatency 10us
set creditQueueCapacity [expr 84*32]  ;# bytes
set dataQueueCapacity [expr 1538*100] ;# bytes
set hostQueueCapacity [expr 1538*100] ;# bytes
set maxCrditBurst [expr 84*2] ;# bytes
set creditRate 64734895 ;# bytes / sec
set expID [expr int([lindex $argv 1])]

set avgFlowInterval 0.0005 ;#500us
set RNGFlowInterval [new RNG]
$RNGFlowInterval seed 94762103

set randomFlowInterval [new RandomVariable/Exponential]
$randomFlowInterval use-rng $RNGFlowInterval
$randomFlowInterval set avg_ $avgFlowInterval

# Output file
file mkdir "outputs"
set nt [open "outputs/trace_$expID.out" w]
set fct_out [open "outputs/fct_$expID.out" w]
puts $fct_out "Flow ID,Flow Size (bytes),Flow Completion Time (secs)"
close $fct_out

proc finish {} {
  global ns nt
  $ns flush-trace
  close $nt
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
Queue/DropTail set qlim_ [expr $hostQueueCapacity/1538]

Queue/XPassDropTail set credit_limit_ $creditQueueCapacity
Queue/XPassDropTail set data_limit_ $dataQueueCapacity
Queue/XPassDropTail set token_refresh_rate_ $creditRate

for {set i 0} {$i < $N} {incr i} {
  $ns simplex-link $left_node($i) $left_gateway $inputlinkBW $linkLatency DropTail
  $ns simplex-link $left_gateway $left_node($i) $inputlinkBW $linkLatency XPassDropTail

  $ns simplex-link $right_node($i) $right_gateway $inputlinkBW $linkLatency DropTail
  $ns simplex-link $right_gateway $right_node($i) $inputlinkBW $linkLatency XPassDropTail
}

$ns duplex-link $left_gateway $right_gateway $linkBW $linkLatency XPassDropTail
#$ns trace-queue $left_gateway $right_gateway $nt

puts "Creating Agents..."
Agent/XPass set max_credit_rate_ $creditRate
Agent/XPass set cur_credit_rate_ [expr $ALPHA*$creditRate]
Agent/XPass set w_ $w_init
Agent/XPass set exp_id_ $expID

for {set i 0} {$i < $N} {incr i} {
  set sender($i) [new Agent/XPass]
  set receiver($i) [new Agent/XPass]

  $ns attach-agent $left_node($i) $sender($i)
  $ns attach-agent $right_node($i) $receiver($i)

  $sender($i) set fid_ [expr $i]
  $receiver($i) set fid_ [expr $i]

  $ns connect $sender($i) $receiver($i)
}

puts "Simulation started."
set nextTime 0.0
for {set i 0} {$i < $N} {incr i} {
  $ns at $nextTime "$sender($i) advance-bytes 500000000"
  set nextTime [expr $nextTime+[$randomFlowInterval value]]
}

$ns at 10.0 "finish"
$ns run
