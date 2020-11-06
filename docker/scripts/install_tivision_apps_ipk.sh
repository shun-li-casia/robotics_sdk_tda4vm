#!/bin/bash
IPK_PKG=tivision_apps

CHECK_INSTALLED=$(opkg list-installed | grep $IPK_PKG)
echo "opkg list-installed | grep $IPK_PKG"
echo $CHECK_INSTALLED

opkg update
if [[ $CHECK_INSTALLED = ${IPK_PKG}* ]]; then
    echo "opkg remove $IPK_PKG"
    opkg remove $IPK_PKG
fi
echo "opkg install $IPK_PKG --force-overwrite"
opkg install $IPK_PKG --force-overwrite

echo "$0: Completed"