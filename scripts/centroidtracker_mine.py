from scipy.spatial import distance as dist
import numpy as np
from collections import OrderedDict


class CentroidTracker():
    def __init__(self, maxDisappeared=30, minShowup=5):
        self.nextObjectID = 1
        self.objects = {}#OrderedDict()
        self.disappeared = {}#OrderedDict()
        self.showup = {}#OrderedDict()
        self.maxDisappeared = maxDisappeared
        
    def register(self, centroid):
        self.objects[self.nextObjectID] = centroid
        self.disappeared[self.nextObjectID] = 0 # #of time the object disappeared
        self.showup[self.nextObjectID] = 1 
        self.nextObjectID += 1
    
    def deregister(self, objectID):
        del self.objects[objectID]
        del self.disappeared[objectID]
        del self.showup[objectID]
        
    def update(self, inputCentroids): # each frame call this once
        
        if len(inputCentroids)==0: # no object
            for objectID in list(self.disappeared.keys()): # if no registed object it won't run
                self.disappeared[objectID] += 1
                if self.disappeared[objectID] > self.maxDisappeared:
                    self.deregister(objectID)
            
            return self.objects, self.showup, self.disappeared
                    
        if len(self.objects)==0: # havn't track any object
            for i in range(len(inputCentroids)):
                self.register(inputCentroids[i])
        else:
            objectIDs = list(self.objects.keys())
            objectCentroids = list(self.objects.values())
            D = dist.cdist(np.array(objectCentroids), inputCentroids)
            # print('D: ',D)
            rows = D.min(axis=1).argsort()
            cols = D.argmin(axis=1)[rows]
            usedRows = set()
            usedCols = set()
            
            for (row, col) in zip(rows, cols):
                if row in usedRows or col in usedCols:
                    continue
                
                objectID = objectIDs[row]
                if D[row][col] > 1.2:
                    self.register(inputCentroids[col])
                else:
                    self.objects[objectID] = inputCentroids[col]
                    self.disappeared[objectID] = 0
                    self.showup[objectID] += 1
                usedRows.add(row)
                usedCols.add(col)
                
            unusedRows = set(range(0, D.shape[0])).difference(usedRows)
            unusedCols = set(range(0, D.shape[1])).difference(usedCols)
            
            if D.shape[0] >= D.shape[1]:
                for row in unusedRows:
                    objectID = objectIDs[row]
                    self.disappeared[objectID] += 1
                    
                    if self.disappeared[objectID] > self.maxDisappeared:
                        self.deregister(objectID)
            else:
                for col in unusedCols:
                    self.register(inputCentroids[col])
            # print('centroid...')
            # print(self.objects)
            # print(self.disappeared)
            # print(self.showup)
        return self.objects, self.showup, self.disappeared

