ps aux | grep '.*ros.*1to2.*'| grep -v grep | awk '{ print "kill -9", $2 }' | bash