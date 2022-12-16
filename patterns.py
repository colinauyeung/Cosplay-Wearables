import math
from scipy import signal
import copy
import csv
import time
import RPi.GPIO as GPIO
import numpy as np
import cython
import normalizep
from numpy import fft

class corrolation:

    smallestmodel = 0

    def __init__(self, modelskip=5, lockon=0.7):
        self.models = []
        self.modelskip = modelskip
        self.lockon = lockon

    def dotproduct(self, f, g):
        if(len(f) != len(g)):
            return NULL
        else:
            sum = 0
            for i in range(len(f)):
                sum = sum + (f[i] * g[i])
            return sum

    def dotdotproduct(self, f, g):
        sum = 0
        for i in range(len(f)):
            sum = sum + self.dotproduct(f[i],g[i])
        return sum

    def scalevector(self, scale, f):
        newf = []
        for i in range(len(f)):
            newf.append(scale*f[i])
        return newf

    def normalize(self, f):
        base = 0
        for i in range(len(f)):
            base = base + self.dotproduct(f[i], f[i])
        base = 1/math.sqrt(base)
        ret = []
        for i in range(len(f)):
            ret.append(self.scalevector(base, f[i]))
        return ret

    def rotate(self, f, r):
        for i in range(0,r):
            f.append(f.pop(0))

    def resample(self, f, nsample):
        # track1 = []
        # track2 = []
        # track3 = []
        # track4 = []
        # for i in range(len(f)):
        #     track1.append(f[i][0])
        #     track2.append(f[i][1])
        #     track3.append(f[i][2])
        #     track4.append(f[i][3])

        
        # track1 = signal.resample(track1, nsample)
        # track2 = signal.resample(track2, nsample)
        # track3 = signal.resample(track3, nsample)
        # track4 = signal.resample(track4, nsample)
        

        rot = np.rot90(f)
        res = []
        for i in range(len(rot)):
            res.append(signal.resample(rot[i], nsample))

        return np.ndarray.tolist(np.rot90(res, -1));

        # res = []
        # for i in range(len(track1)):
        #     res.append([track1[i], track2[i], track3[i], track4[i]])
        # return res

    def bruterotation(self, f, g, nsample):
        max = 0
        maxi = -1
        for i in range(0,nsample+1,1):
            val = self.dotdotproduct(f, g)
            self.rotate(g, 1)
            if(val > max):
                max = val
                maxi = i
        return (maxi, max)


    def lockedrotation(self, f, g, nsample, target):
        low = target - 5
        high = target + 5
    
        if(low<=0):
            low = 0

        max = 0
        maxi = -1

        if(high > nsample):
            high = nsample
            gcopy = copy.deepcopy(g)
            for i in range(0, 5):
                val = self.dotdotproduct(f, gcopy)
                self.rotate(gcopy, 1)
                if(val > max):
                    max = val
                    maxi = i

        self.rotate(g, low)
        for i in range(low,high+1,1):
            val = self.dotdotproduct(f, g)
            self.rotate(g, 1)
            if(val > max):
                max = val
                maxi = i
        return (maxi, max)

    def old_brutecadance(self, f):
        # minp = math.floor(len(g) * minpercent)
        # maxp = math.ceil(len(g) * maxpercent)
        max = 0;
        state = (0,0,0)
        # for i in range(minp, maxp+1):
        #     gn = self.resample(g, i)
        #     ghat = self.normalize(gn)
        for z in range(len(self.models)):
            ghat = copy.deepcopy(self.models[z])
            i = len(ghat)
            fhat = self.normalize(f[-i:])
            rotation, maxr = self.bruterotation(fhat,ghat, i)
            if(maxr > max):
                max = maxr
                state = (maxr, rotation, z)
        return state

    def lockedcadance(self, f, targetcadence, targetrotation):
        max = 0;
        state = (0,0,0)
        # for i in range(minp, maxp+1):
        #     gn = self.resample(g, i)
        #     ghat = self.normalize(gn)

        high = targetcadence + 5
        low = targetcadence - 5
        if(low<=0):
            low = 0

        if(high >= len(self.models)):
            high = len(self.models)
            for z in range(0,5):
                ghat = copy.deepcopy(self.models[z])
                i = len(ghat)
                fhat = self.normalize(f[-i:])
                rotation, maxr = self.lockedrotation(fhat,ghat, i, targetrotation)
                if(maxr > max):
                    max = maxr
                    state = (maxr, rotation, z)


        for z in range(low, high):
            ghat = copy.deepcopy(self.models[z])
            i = len(ghat)
            fhat = self.normalize(f[-i:])
            rotation, maxr = self.lockedrotation(fhat,ghat, i, targetrotation)
            if(maxr > max):
                max = maxr
                state = (maxr, rotation, z)
        return state

    # https://stackoverflow.com/questions/28284257/circular-cross-correlation-python
    def periodic_corr(self, x, y):
        """Periodic correlation, implemented using the FFT.

        x and y must be real sequences with the same length.
        """
        return fft.ifft(np.fft.fft(x) * fft.fft(y).conj()).real

    def brutecadance(self,f):
        # minp = math.floor(len(g) * minpercent)
        # maxp = math.ceil(len(g) * maxpercent)
        max = 0;
        state = (0,0,0)
        # for i in range(minp, maxp+1):
        #     gn = self.resample(g, i)
        #     ghat = self.normalize(gn)

        # fnormal = normalizep.normalize(f)
        for z in range(0,len(self.models)):
            i = len(self.models[z][0])
            if(len(f) < i):
                continue

            # fhat = fnormal[-i:]
            fhat = normalizep.normalize(f[-i:])
            # print(fhat)

            # grot = np.rot90(self.models[z])
            grot = self.models[z]
            frot = np.rot90(fhat)

            # resn = signal.correlate(grot[0], frot[0], "full")
            # res = resn[0]
            # for i in range(1, len(res)):
            #     res = np

            # res = np.correlate(grot[0], frot[0], "full")
            res = self.periodic_corr(grot[0], frot[0])
            for i in range(1, len(grot)):
                # print(res)
                res = np.add(res, 
                self.periodic_corr(grot[i], frot[i])
                # np.correlate(grot[i], frot[i], "full")
                )

           
            

            maxr = np.argmax(res)
            maxm = res[maxr]

            if(maxm > max):
                max = maxm
                state = (res[maxr], maxr, z)

            # rotation, maxr = self.bruterotation(fhat,ghat, i)
            # if(maxr > max):
            #     max = maxr
            #     state = (maxr, rotation, z)
        return state

    def getresults(self, f):
        return self.brutecadance(f)
        # if(previous < self.lockon):
        #     return self.brutecadance(f)
        
        # maxr, rotation, z = self.lockedcadance(f, targetcadence, targetrotation)
        # if(maxr >= self.lockon):
        #     return (maxr, rotation, z)
        # else:
        #     return self.brutecadance(f)
        

    def prerendermodels(self, g, minpercent, maxpercent):
        minp = math.floor(len(g) * minpercent)
        self.smallestmodel = minp
        maxp = math.ceil(len(g) * maxpercent)
        for i in range(minp, maxp+1, self.modelskip):
            print(i)
            myg = copy.deepcopy(g)
            gn = self.resample(myg, i)
            ghat = self.normalize(gn)
            self.models.append(np.rot90(copy.deepcopy(ghat)))

    def convertresults(self, maxr, rotation, z):
        return (maxr, rotation, len(self.models[z][0]))

    def dumpmodels(self):
        for i in range(len(self.models)):
            self.dumpmodel(i)

    def dumpmodel(self, i):
        print(self.models[i])
        # res = np.rot90(np.rot90(np.rot90(self.models[i])))
        # for line in res:
        #     print(line)

        with open("model{0}.csv".format(i), mode="w") as file:
            writer = csv.writer(file)
            res = np.rot90(np.rot90(np.rot90(self.models[i])))
            for line in res:
                print(line)
                writer.writerow([line[0], line[1]])
    # def normalize(self, f):

if __name__ == "__main__":
    c = corrolation(10, 0.4)

    model = []
    with open("walkingsample.csv") as m:
        csv_reader = csv.reader(m)
        for row in csv_reader:
            # print( "gx:{:0.2f} gy:{:0.2f} gz:{:0.2f}".format( float(row[0]), float(row[1]), float(row[2]) ) )
            # time.sleep(0.01)
            model.append([float(row[0]), float(row[1])
            # , float(row[2])
            # , float(row[3])
            ])
            
    c.prerendermodels(model, 0.50, 1.5)
    print("Prerendering")

    c2 = corrolation(10, 0.4)

    model2 = []
    with open("squatsample.csv") as m:
        csv_reader = csv.reader(m)
        for row in csv_reader:
            # print( "gx:{:0.2f} gy:{:0.2f} gz:{:0.2f}".format( float(row[0]), float(row[1]), float(row[2]) ) )
            # time.sleep(0.01)
            model2.append([float(row[0]), float(row[1])
            # , float(row[2])
            # , float(row[3])
            ])
            
    c2.prerendermodels(model2, 0.50, 1.5)
    print("Prerendering")

    data = []
    periods = []
    prev = 0
    last = (0,0,0,0)

    print(c.smallestmodel)

    with open("test21.csv") as d:
        csv_reader = csv.reader(d)
        counter = 0
        for row in csv_reader:
            # print( "gx:{:0.2f} gy:{:0.2f} gz:{:0.2f}".format( float(row[0]), float(row[1]), float(row[2]) ) )
            # time.sleep(0.01)

            # if(counter < 300):
            #     counter +=1
            #     continue


            t1 = time.perf_counter()
            data.append([float(row[0]), float(row[1])
                # , float(row[2])
                # ,float(row[3])
                ])

            if(len(data) < c.smallestmodel):
                continue

            diff  = abs(last[0] - float(row[0])) +  abs(last[1] - float(row[1]))
                # +  abs(last[2] - float(row[2]))
                # +  abs(last[3] - float(row[3]))
        
            # if(diff < 10):
            #     counter +=1
            #     continue
            res = c.getresults(data)

            res2 = c.convertresults(res[0], res[1], res[2])


            res21 = c2.getresults(data)
            res22 = c2.convertresults(res21[0], res21[1], res21[2])

            if(res2[0] > res22[0]):
                if(0.1<=res2[1]/res2[2]<=0.6):
                    print(row[2], res2, res2[1]/res2[2], "left")
                else:
                    print(row[2], res2, res2[1]/res2[2], "right")
            else:
                print(row[2], res22, res22[1]/res22[2], "squat")
            prev = res
        

            if(len(data) > 111):
                data.pop(0)

            last = (float(row[0]), float(row[1])
                # , float(row[2])
                # , float(row[3])
                )

            counter += 1
            t2 = time.perf_counter()
            periods.append(t2-t1)
            # print(1/(t2-t1))
            # print(t2-t1)
            # if(counter > 200):
            #     break
            # if(res[0] > 0.8):
            #     q = res[1]/res[2]
            #     angle = 180 * q
            #     servo1.ChangeDutyCycle(2+(angle/18))
            #     time.sleep(0.5)
            #     servo1.ChangeDutyCycle(0)

    total = 0
    for i in range(len(periods)):
        total = total + periods[i]
    print(1/(total/len(periods)))

    print("Done")
    c.dumpmodels()





# print(data)


# print(c.dotproduct([1,2,3], [1,5,7]))
# print(c.normalize([[1,2],[1,2],[1,2]]))

# f = [1,2,3]
# c.rotate(f, 2)
# print(f)

# print(signal.resample([1,2,3,2], 8))