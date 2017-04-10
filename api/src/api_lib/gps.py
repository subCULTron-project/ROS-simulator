#!/usr/bin/env python

"""
GPS API class.
"""

class GPS(object):
    
    def __init__(self):
        pass
    
    def has_fix(self):
        '''
        Returns:
           hasFix: bool
        '''
        pass
    
    def get_data(self):
        '''
        Get GPS data.
        Returns:
           [latitude, longitude, altitude]: [float, float, float]
        '''
        pass
    
    def get_accuracy(self):
        '''
        Get GPS accuracy.
        Returns:
           accuracy: float, hdop
        '''
        pass