#!/usr/bin/env python

from datetime import datetime

"""
Clock API class. Uses datetime library (https://docs.python.org/2/library/datetime.html).
"""

__author__ = "barbanas"


class Clock(object):
    
    def __init__(self):
        pass
    
    def get_datetime(self):
        """
        Gets current datetime.

        @rtype: datetime
        @return: Current datetime.
        """
        return datetime.now()

    def get_date(self):
        """
        Gets current date.

        @rtype: date
        @return: Current date.
        """
        return datetime.now().date()

    def get_time(self):
        """
        Gets current time.

        @rtype: datetime
        @return: Current time.
        """
        return datetime.now().time()
