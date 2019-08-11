#!/bin/sh

ssb=$(get.site 0 ssb)
rtm_translen=$(get.site 1 rtm_translen)
echo "rtm_translen has been auto detected as $rtm_translen samples."
trigger_rate=$1 # trigger rate in Hz

if [ $# -eq 0 ]; then
    trigger_rate=$(get.site 0 SIG:TRG_EXT:FREQ)
    trigger_rate=$(echo $trigger_rate | awk '{ print $2 }')
    if [ '$trigger_rate' = '0' ]; then
        echo 'Please provide the estimated trigger rate in Hz'
        echo 'So if trigger rate is 100Hz the appropriate cmd is:'
        echo './est_buf_size.sh 100'
        echo ''
        exit
    else
        echo "Trigger has been auto-detected as $trigger_rate Hz"
        echo ""
    fi
fi

buf_size=$(echo 5 $ssb $rtm_translen $trigger_rate | awk '{ print $1 * $2 * $3 * $4}')

A=$buf_size
C=1
while [ $A -ne 1 ]; do
    A=$((A>>1))
    C=$((C<<1))
done
NEXT=$((C<<1))
DIFF1=$((buf_size-C))
DIFF2=$((NEXT-ORIG))
if [  "$DIFF1" -ge "$DIFF2" ]; then
    echo "$NEXT"
else
    if [ $C -ge 1048576 ]; then
        C=1048576
    fi
    echo "Recommended buffer size: $C"
    echo "Setting now."
    set.site 0 bufferlen $C
    echo "bufferlen has been set to $C. Check with:"
    echo "get.site 0 bufferlen"
fi


