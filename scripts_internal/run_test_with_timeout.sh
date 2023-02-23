#!/bin/bash

#  Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#
# USAGE run_test_with_timeout.sh <timeout> <loop_count> <command_to_run>
#

# Validate the commandline argument count
if [[ $# -lt 3 ]]
then
    printf "USAGE: $0 <timeout> <loop_count> <command_to_run>\n"
    exit 1
fi

BLINK_BEGIN='\033[33m'
BLINK_END='\033[0m'
BRIGHTWHITE='\033[0;37;1m'
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NOCOLOR='\033[0m'

args=($@)
len=${#args[@]}

# First argument is a timeout value
timeout=$1

# Second argument is a loop count
loopCnt=$2

# The rest of the arguments forms the command
cmd=${args[@]:2:$len-1}

printf "$BLUE"
printf "TEST TIMEOUT VALUE: $timeout \n"
printf "TEST LOOP COUNT   : $loopCnt \n"
printf "TEST CMD          : $cmd \n"
printf "$NOCOLOR\n"

printf "The command $BLINK_BEGIN[$cmd]$BLINK_END will be run for "
printf "$BLINK_BEGIN[$timeout seconds]$BLINK_END for "
printf "a total of $BLINK_BEGIN[$loopCnt]$BLINK_END times.\n"

name=""
case $cmd in
  *"semseg"*)
    name=".semseg"
    ;;
  *"objdet"*)
    name=".objdet"
    ;;
  *"sde"*)
    name=".sde"
    ;;
  *"estop"*)
    name=".estop"
    ;;
  *"visloc"*)
    name=".visloc"
    ;;
esac

# Get the current time
cur_time=$(date "+%Y_%m_%d-%H.%M.%S")

# Setup log files for stout and stderr for each test run
stdout="stdout$name.$cur_time.log"
stderr="stderr$name.$cur_time.log"

echo "Directing STDOUT to: $stdout"
echo "Directing STDERR to: $stderr"

for ((i=1; i<=$loopCnt; i++))
do
    printf "$BLINK_BEGIN[TEST RUN $i] RUNNING $BLINK_END $NOCOLOR\n"
    timeout -s SIGINT -k $(($timeout + 30)) $timeout $cmd >> $stdout 2> $stderr
    test_status=$?

    if [ "$test_status" -ne "0" ] && [ "$test_status" -ne "124" ]
    then
       # Test failed while running
       printf "$BRIGHTWHITE[TEST RUN $i]$RED FAIL $NOCOLOR\n"
    else
       printf "$BRIGHTWHITE[TEST RUN $i]$GREEN PASS $NOCOLOR\n"
    fi
done

# Make sure to write the file content to the card
sync

cnt=`grep "APP: Deinit ... Done" $stdout | wc -l`

if [ "$cnt" -ne "$loopCnt" ]
then
	printf "$BRIGHTWHITE[OVERALL STATUS]$RED EXPECTING $loopCnt SUCCESSFUL RUNS. FOUND ONLY $cnt RUNS. $NOCOLOR\n"
else
	printf "$BRIGHTWHITE[OVERALL STATUS]$GREEN EXPECTING $loopCnt SUCCESSFUL RUNS. FOUND ALL $cnt RUNS. $NOCOLOR\n"
fi

