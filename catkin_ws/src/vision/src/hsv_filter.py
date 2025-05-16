#!/usr/bin/env python3

class HSVFilter:
    def _init_(self,
               hMin = None, sMin = None, vMin = None,
               hMax = None, sMax = None, vMax = None,
               sAdd = None, vAdd = None, sSub = None, vSub = None):
        self.hMin = hMin
        self.sMin = sMin
        self.vMin = vMin
        self.hMax = hMax
        self.sMax = sMax
        self.vMax = vMax
        self.sAdd = sAdd
        self.vAdd = vAdd
        self.sSub = sSub
        self.vSub = vSub
