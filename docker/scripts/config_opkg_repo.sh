#!/bin/bash
# Should run on PSDK host linux
OPKG_REPO_DIR=/opkg_repo

FILE=/etc/opkg/base-feeds.conf
if [ -f "${FILE}_org" ]
then
	echo "It looks like $FILE already updated. Here is content:"
	echo "cat $FILE"
	cat $FILE
else
	mv "$FILE" "${FILE}_org"
	# Update base-feeds.conf to use a local OPKG repository
	echo "# Original $FILE file was copied to ${FILE}_org" > $FILE
	echo "Updated $FILE"
	echo "cat $FILE"
	cat $FILE
fi

FILE=/etc/opkg/opkg.conf
if [ -f "${FILE}_org" ]
then
	echo ""
	echo "It looks like $FILE already updated. Here is content:"
	echo "cat $FILE"
	cat $FILE
else
	mv "$FILE" "${FILE}_org"
	# Update opkg.conf to use a local OPKG repository
	echo "# Original $FILE file was copied to ${FILE}_org" > $FILE
	echo "# Settings for a local OPKG repository" >> $FILE
	echo "src/gz local file://$OPKG_REPO_DIR"     >> $FILE
	echo "dest root /"                            >> $FILE
	echo "option lists_dir /var/lib/opkg/lists"   >> $FILE

	echo ""
	echo "Updated $FILE"
	echo "cat $FILE"
	cat $FILE
fi

echo ""
echo "$0: Completed"