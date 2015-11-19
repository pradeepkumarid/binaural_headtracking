

from __future__ import print_function 
from Scientific.IO.NetCDF import *
from wave import open
import numpy as np
import pylab as pl
import matplotlib.pyplot as plt
import socket as soc


UDP_IP = "192.168.0.12"; 
UDP_PORT = 12345; #int(raw_input ("Enter Port "))
sock = soc.socket.socket(socket.AF_INET, # Internet
                    socket.SOCK_DGRAM) # UDP
soc.sock.bind((UDP_IP, UDP_PORT))

f = NetCDFFile('/home/pradeep/Q4/SpatialAudio/HW/4/subject_020.sofa', 'r')


#Input audio object
obj1File =  open('drumloop.wav', 'r');
obj1 = frontleftfile.readframes(-1);
obj1 = np.fromstring(obj1, 'Int16');
#plt.plot(obj1)
#plt.show();



print(np.shape(f.variables['Data.IR']), np.shape(f.variables['SourcePosition']))

print(np.shape(frontLeft)[0])

f.variables['Data.Delay'][0]


np.shape(f.variables['SourcePosition']);
id_obj1 = -1;



index = -1;
for spos in f.variables['SourcePosition'][:]:
    index += 1;
    if (round(spos[1])==0 and spos[0]==0):
       id_FC = index;
    elif (round(spos[1])==0 and spos[0]==30):
        id_FL = index;
    elif (round(spos[1])==0 and spos[0]==115): 
        id_SL = index;
    elif (round(spos[1])==0 and spos[0]==(360-115)): 
        id_SR = index;
    elif (round(spos[1])==0 and spos[0]==(360-30)): 
        id_FR = index;
        

print(id_FL, id_FR, id_FC, id_SL, id_SR)

# <codecell>

import pyaudio

nchannels = 2 #2
sampwidth = 2
framerate = 44100
nframes = 200 
#nframes = np.shape(frontLeft)[0] 
comptype = "NONE"
compname = "not compressed"


chunk = nframes 
p = pyaudio.PyAudio() ;
stream = p.open(format = p.get_format_from_width(sampwidth),  
                channels = nchannels,  
                rate = framerate,  
                output = True) 

start = 0
finalOutput = np.array([]);
while start<=(np.shape(frontLeft)[0]- chunk):
    frontLeftSample = frontLeft[start:start+chunk];
    frontRightSample = frontRight[start:start+chunk];
    frontCenterSample = frontCenter[start:start+chunk];
    subSample = sub[start:start+chunk];
    leftSurrSample = leftSurr[start:start+chunk];
    rightSurrSample = rightSurr[start:start+chunk];
    
#    l1Output = np.convolve(frontLeftSample, f.variables['Data.IR'][id_FL][0][:]);
#    l2Output = np.convolve(frontRightSample, f.variables['Data.IR'][id_FR][0][:]);
#    l3Output = np.convolve(leftSurrSample, f.variables['Data.IR'][id_SL][0][:]);
#    l4Output = np.convolve(rightSurrSample, f.variables['Data.IR'][id_SR][0][:]);
#    l5Output = np.convolve(frontCenterSample, f.variables['Data.IR'][id_FC][0][:]);

#    r1Output = np.convolve(frontLeftSample, f.variables['Data.IR'][id_FL][1][:]);
#    r2Output = np.convolve(frontRightSample, f.variables['Data.IR'][id_FR][1][:]);
#    r3Output = np.convolve(leftSurrSample, f.variables['Data.IR'][id_SL][1][:]);
#    r4Output = np.convolve(rightSurrSample, f.variables['Data.IR'][id_SR][1][:]);
#    r5Output = np.convolve(frontCenterSample, f.variables['Data.IR'][id_FC][1][:]);

    l1Output = np.convolve(frontLeftSample, f.variables['Data.IR'][id_FL][0][:], 'same');
    l2Output = np.convolve(frontRightSample, f.variables['Data.IR'][id_FR][0][:], 'same');
    l3Output = np.convolve(leftSurrSample, f.variables['Data.IR'][id_SL][0][:], 'same');
    l4Output = np.convolve(rightSurrSample, f.variables['Data.IR'][id_SR][0][:], 'same');
    l5Output = np.convolve(frontCenterSample, f.variables['Data.IR'][id_FC][0][:], 'same');

    r1Output = np.convolve(frontLeftSample, f.variables['Data.IR'][id_FL][1][:], 'same');
    r2Output = np.convolve(frontRightSample, f.variables['Data.IR'][id_FR][1][:], 'same');
    r3Output = np.convolve(leftSurrSample, f.variables['Data.IR'][id_SL][1][:], 'same');
    r4Output = np.convolve(rightSurrSample, f.variables['Data.IR'][id_SR][1][:], 'same');
    r5Output = np.convolve(frontCenterSample, f.variables['Data.IR'][id_FC][1][:], 'same');
    
    loutput = l1Output + l2Output + l3Output + l4Output + l5Output + subSample/(np.sqrt(2)) ;
    routput = r1Output + r2Output + r3Output + r4Output + r5Output + subSample/(np.sqrt(2)) ;
    
   # if start==0:
	#plt.plot(frontLeftSample);
	#plt.plot(l1Output);
	#plt.show();

    finalLOutput = loutput.astype('int16');
    finalROutput = routput.astype('int16');
    interleavedOutput = np.zeros([2 * finalLOutput.shape[0]]);
    for i in range(finalLOutput.shape[0]):
        interleavedOutput[2*i] = finalLOutput[i];#if finalLOutput[i]<1 else 1;
        interleavedOutput[2*i+1] = finalROutput[i]; #  if finalROutput[i]<1 else 1;
    
    print('Input size: ',frontLeftSample.shape[0],' HRIR size:',f.variables['Data.IR'][id_FL][0][:].shape[0]);
    print('Writing OP:',finalLOutput.shape[0],' frames');
    
    #stream.write([finalROutput,finalROutput])  
    #stream.write(np.concatenate([finalROutput,finalROutput]).astype('int16')) 
    stream.write(interleavedOutput.astype('int16')) ;
    start = start + chunk/2;
    finalOutput = np.concatenate((finalOutput, loutput), axis=0);
    
#stop stream  
stream.stop_stream()  
stream.close()  

#close PyAudio  
p.terminate()


# <codecell>



# <codecell>

finalOutput = finalOutput.astype('int16');

#plot(finalOutput);

# <codecell>


# <codecell>

#figure;
#plot(loutput);
#plot(routput);

# <codecell>

finalLOutput = loutput.astype('int16');
finalROutput = routput.astype('int16');

# <codecell>

interleavedOutput = np.zeros([2 * finalLOutput.shape[0]]);
for i in range(finalLOutput.shape[0]):
    interleavedOutput[2*i] = finalLOutput[i];
    interleavedOutput[2*i+1] = finalROutput[i];
    
interleavedOutput = interleavedOutput.astype('int16');

# <codecell>

wav_file = open('/home/pradeep/Q4/SpatialAudio/HW/4/output.wav', 'w');
nchannels = 2
sampwidth = 2
framerate = 48000
nframes = 1250
comptype = "NONE"
compname = "not compressed"

wav_file.setparams((nchannels, sampwidth, framerate, nframes,comptype, compname));
wav_file.writeframes(interleavedOutput);
wav_file.close;

