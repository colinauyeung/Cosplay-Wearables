from numpy.fft import fft, ifft
import numpy
import time
import csv
import math

def scalevector(scale, f):
    newf = []
    for i in range(len(f)):
        newf.append(scale*f[i])
    return newf

def normalize(f):
    base = 0
    for i in range(len(f)):
        base = base + numpy.dot(f[i], f[i])
    base = 1/math.sqrt(base)
    ret = []
    for i in range(len(f)):
        ret.append(scalevector(base, f[i]))
    return ret

def periodic_corr(x, y):
    """Periodic correlation, implemented using the FFT.

    x and y must be real sequences with the same length.
    """
    return ifft(fft(x) * fft(y).conj()).real


l1 = []
l2 = []


model = []
model2 = []
with open("test8.csv") as m:
    csv_reader = csv.reader(m)
    for row in csv_reader:
        # print( "gx:{:0.2f} gy:{:0.2f} gz:{:0.2f}".format( float(row[0]), float(row[1]), float(row[2]) ) )
        # time.sleep(0.01)
        model.append([float(row[0]), float(row[1]), float(row[2]), float(row[3])])

modeln = []
with open("test9.csv") as m:
    csv_reader = csv.reader(m)
    for row in csv_reader:
        # print( "gx:{:0.2f} gy:{:0.2f} gz:{:0.2f}".format( float(row[0]), float(row[1]), float(row[2]) ) )
        # time.sleep(0.01)
        modeln.append([float(row[0]), float(row[1]), float(row[2]), float(row[3])])


for n in range(192, 192+50):
    model2 = modeln[n:n+45]

    modelnormal = normalize(model)
    modelnormal2 = normalize(model2)
    # for i in range(0,100):
    #     l1.append(i/100);
    #     l2.append(((i+1)%100)/100)

    rmodel = numpy.rot90(modelnormal)
    rmodel2 = numpy.rot90(modelnormal2)


    res = periodic_corr(rmodel[0],rmodel2[0])
    for(j) in range(1,4):
        res = numpy.add(res, periodic_corr(rmodel[j],rmodel2[j]))
    # print(res)
    print(numpy.argmax(res))



# l3= [[1,2,3], [4,5,6], [7,8,9]]
# print(numpy.ndarray.tolist(numpy.rot90(numpy.rot90(l3,1), -1)))
# print(l3)

# print(t2-t1)
# print(periodic_corr(l1, l2))