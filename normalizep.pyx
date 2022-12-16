# distutils: language=c++

from libcpp.vector cimport vector
from math import sqrt
from numpy import dot
from numpy import multiply

def dotproduct(f, g):
    cdef float res
    res = 0
    if(len(f) != len(g)):
        return 0
    else:
        for i in range(len(f)):
            res = res + (f[i] * g[i])
        return res

def dotdotproduct(f, g):
    cdef float res
    res = 0
    for i in range(f.size()):
        res = res + dotproduct(f[i],g[i])
    return res

def scalevector(float scale, f):
    newf = []
    for i in range(len(f)):
        newf.append(scale*f[i])
    return newf

def normalize(f):
    cdef float base
    base = 0
    for i in range(len(f)):
        base = base + dotproduct(f[i], f[i])
    base = 1/sqrt(base)
    ret = []
    for i in range(len(f)):
        ret.append(scalevector(base, f[i]))
    return ret


# def brutecadance(self,f):
#     # minp = math.floor(len(g) * minpercent)
#     # maxp = math.ceil(len(g) * maxpercent)
#     max = 0;
#     state = (0,0,0)
#     # for i in range(minp, maxp+1):
#     #     gn = self.resample(g, i)
#     #     ghat = self.normalize(gn)
#     for z in range(0,len(self.models)):
#         i = len(self.models[z])
#         if(len(f) < i):
#             continue
#         fhat = normalize(f[-i:])

#         grot = np.rot90(self.models[z])
#         frot = np.rot90(fhat)

#         # resn = signal.correlate(grot[0], frot[0], "full")
#         # res = resn[0]
#         # for i in range(1, len(res)):
#         #     res = np

#         res = np.correlate(grot[0], frot[0], "full")
#         for i in range(1, len(grot)):
#             res = np.add(res, np.correlate(grot[i], frot[i], "full"))

        

#         maxr = np.argmax(res)
#         maxm = res[maxr]

#         if(maxm > max):
#             max = maxm
#             state = (res[maxr], maxr, z)

#         # rotation, maxr = self.bruterotation(fhat,ghat, i)
#         # if(maxr > max):
#         #     max = maxr
#         #     state = (maxr, rotation, z)
#     return state

# def getresults(self, f, previous, targetrotation, targetcadence):
#     return self.brutecadance(f)
#     # if(previous < self.lockon):
#     #     return self.brutecadance(f)
    
#     # maxr, rotation, z = self.lockedcadance(f, targetcadence, targetrotation)
#     # if(maxr >= self.lockon):
#     #     return (maxr, rotation, z)
#     # else:
#     #     return self.brutecadance(f)
    

# def prerendermodels(self, g, minpercent, maxpercent):
#     minp = math.floor(len(g) * minpercent)
#     self.smallestmodel = minp
#     maxp = math.ceil(len(g) * maxpercent)
#     for i in range(minp, maxp+1, self.modelskip):
#         print(i)
#         myg = copy.deepcopy(g)
#         gn = self.resample(myg, i)
#         ghat = self.normalize(gn)
#         self.models.append(copy.deepcopy(ghat))

# def convertresults(self, maxr, rotation, z):
#     return (maxr, rotation, len(self.models[z]))