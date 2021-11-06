# Reference:
# http://code.activestate.com/recipes/278258/
from functools import reduce
import random

class ListTools:
    def normListSumTo(self, L, sumTo=1):
        '''normalize values of a list to make it sum = sumTo'''

        sum = reduce(lambda x,y:x+y, L)
        return [ x/(sum*1.0)*sumTo for x in L]

    