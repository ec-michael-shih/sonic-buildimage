#!/bin/bash
set -x

SYNCD_SOCKET_FILE=/var/run/sswsyncd/sswsyncd.socket

# Function: wait until syncd has created the socket for bcmcmd to connect to
wait_syncd() {
    while true; do
        docker exec -t syncd ls -al ${SYNCD_SOCKET_FILE} >/dev/null 2>&1
        rv=$?
        if [ $rv -eq 0 ]; then
            break
        fi
        sleep 1
    done

    # wait until bcm sdk is ready to get a request
    counter=0
    while true; do
        /usr/bin/bcmcmd -t 1 "show unit" | grep BCM >/dev/null 2>&1
        rv=$?
        if [ $rv -eq 0 ]; then
            break
        fi
        counter=$((counter+1))
        if [ $counter -ge 120 ]; then
            echo "syncd is not ready to take commands after $counter re-tries; Exiting!"
            break
        fi
        sleep 1
    done
}

wait_syncd
/usr/local/bin/extphy_cmd -f /usr/local/bin/as7926_40xfb_phy_init.soc > /tmp/accton_as7926_40xfb_xphy.log
rv=$?
if [ $rv -eq 0 ]; then
    echo "extphy_cmd configureation finished ...!"
fi
