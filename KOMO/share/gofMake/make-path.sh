#/bin/sh

cd $1
LASTFILE=`ls -Art | tail -n 1`
if [ "$LASTFILE" = '.lastMake' ]; then
    echo "         * UpToDate " $1
else
    if mkdir Make.lock 2> /dev/null
    then    # lock did not exist and was created successfully
	echo "       *** Make     " $1
	make --no-print-directory -f Makefile.gof $2
	if [ $? -eq 0 ] ; then # success
	    date +'%y-%m-%d-%T' > .lastMake
	else # fail
	    tput bold; tput setaf 1
	    echo "     ***** FAILED   " $1
	    tput sgr0
	fi
	rm -rf Make.lock
    else
	echo "         * Waiting  " $1
	while [ -d $1/Make.lock ]
	do
	    sleep 0.2
	done
    fi
fi
