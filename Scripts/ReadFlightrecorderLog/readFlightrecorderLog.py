from __future__ import print_function, division
import struct
import numpy as np

import os.path

# params from cpp:
imu = {}
imu['t'] = []
imu['accelerometer'] = []
imu['rateGyro'] = []

uwbRequest = {}
uwbRequest['t'] = []
uwbRequest['id'] = []

uwbResponse = {}
uwbResponse['t'] = []
uwbResponse['id'] = []
uwbResponse['range'] = []

filenameCounter = -1
lines = 0
randomID = 0
t = 0
lastTime = 0
timeWrap = 0
while True:
    filenameCounter += 1
    fileName = 'files/flightrecorder' + str(filenameCounter).zfill(3) + '.log'
    if not os.path.isfile(fileName):
        print('Cannot find file', fileName)
        break

    print('Opening file:', fileName)
    # time wraps every 256 ms

    with open(fileName, mode='rb') as file:  # b is important -> binary
        if filenameCounter == 0:
            print('Reading contents')
            vehId = struct.unpack('i', file.read(4))[0]
            print('Vehicle ID =', vehId)
            
            test_uint64 = struct.unpack('Q', file.read(8))[0]
            test_float32 = struct.unpack('f', file.read(4))[0]
            
            target_uint64 = 3141592653589793238
            if not test_uint64 == target_uint64:
                print('Test uint64_t failed. Equals', test_uint64, 'instead of', target_uint64)
                
            if np.abs(test_float32 - np.pi) > 1e-6:
                print('Test float failed. Equals', test_float32, 'instead of', np.pi)

            randomID = struct.unpack('H', file.read(2))[0]
        else:
            try:
                fileID = struct.unpack('H', file.read(2))[0]
            except:
                print('Log file',fileName,'appears to be empty, stopping')
                break

            if fileID != randomID:
                print('Log file',fileName,'does not match ID, stopping ('+str(fileID)+' vs '+str(randomID)+')')
                break
            
     
        while True:
            try:
                # get format strings here: https://docs.python.org/3/library/struct.html#format-characters
                lines += 1

                [r_t] = struct.unpack('B', file.read(1))
                [r_a_x, r_a_y, r_a_z, r_g_x, r_g_y, r_g_z] = struct.unpack(2 * 3 * 'h', file.read(2 * 3 * 2))
                [uwbRequestToID, uwbResponderID] = struct.unpack(2 * 'B', file.read(2))
                [uwbResponderRange_mm] = struct.unpack('H', file.read(2))
                
                if r_t < lastTime:
                    timeWrap += 256
                    
                lastTime = r_t
                    
                t = (r_t + timeWrap) / 1e3  # s
                
                imu['t'].append(t)

                def decode_int16(valIn, range):
                    if valIn == 32767:
                        return np.NaN

                    return valIn * range / 32768
                
                imu['accelerometer'].append([decode_int16(v, 16 * 9.81) for v in [r_a_x, r_a_y, r_a_z]])
                imu['rateGyro'].append([decode_int16(v, 36) for v in [r_g_x, r_g_y, r_g_z]])
                
                if uwbRequestToID:
                    uwbRequest['t'].append(t)
                    uwbRequest['id'].append(uwbRequestToID)
                    
                if uwbResponderID:
                    uwbResponse['t'].append(t)
                    uwbResponse['id'].append(uwbResponderID)
                    uwbResponse['range'].append(uwbResponderRange_mm / 1e3)
                
            except :  # whatever reader errors you care about   
#                 print('exception!')
                break;


print('Read', lines, 'lines covering', t, 's')
# convert to convenient np arrays:
for d in [imu, uwbRequest, uwbResponse]:
    for k in d.keys():
        d[k] = np.array(d[k])

import pickle
outFile = 'flightrecorder_log_veh' + str(vehId).zfill(3) + '.pickle'
print('Saving to file: <' + outFile + '>')
pickleFile = open(outFile, 'wb')
pickle.dump([vehId, imu, uwbRequest, uwbResponse], pickleFile)
pickleFile.close()

print('Complete.')

