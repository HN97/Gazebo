#!/usr/bin/env bash
# This file:
#   Demos for the status message
#

## function
#########################################
# print_status () {

SEC=30
sleep ${2}
# The "\033[0K" will delete to the end of the line
printf '\033[99D\033[0K'

# Calculate the print_status
declare -i SEC_HASH
SEC_HASH=$((${1}*${SEC}/100))

printf "[%03d%%][" ${1}
for j in `seq 0 ${SEC_HASH}`;
do
printf '#'
done
declare -i SEC_BLANK
SEC_BLANK=${SEC}-${SEC_HASH}
for k in `seq 0 ${SEC_BLANK}`;
do
printf ' '
done

printf "] ${3}"


# print_status 10 0.75 'activity 1'
# print_status 30 0.75 'configuration files '
# print_status 90 0.75 'update registery'
# print_status 100 2 'done'
# printf "\n"

# print_status 0 0 'Init'
# print_status 30 1 'Configuration'
# print_status 60 1 'download file'
# print_status 100 1 'done'
# printf "\n"

# printf "\u2620 Setup Completed. \n"