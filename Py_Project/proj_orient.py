from __future__ import print_function 
from Scientific.IO.NetCDF import *
from wave import open
import numpy as np
import pylab as pl
import matplotlib.pyplot as plt
import socket


UDP_IP = "192.168.0.12"; 
UDP_PORT = 12345; #int(raw_input ("Enter Port "))
sock = socket.socket(socket.AF_INET, # Internet
                    socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

f = NetCDFFile('/home/pradeep/Q4/SpatialAudio/HW/4/subject_020.sofa', 'r')


#Input audio object
obj1File =  open('drumloop.wav', 'r');
obj1 = obj1File.readframes(-1);
obj1 = np.fromstring(obj1, 'Int16');
#plt.plot(obj1)
#plt.show();



print(np.shape(f.variables['Data.IR']), np.shape(f.variables['SourcePosition']))

print(np.shape(obj1)[0])

f.variables['Data.Delay'][0]

np.shape(f.variables['SourcePosition']);
obj1_id = -1;
obj1_pos = 0; # 0 degrees azimuth

#index = -1;
#for spos in f.variables['SourcePosition'][:]:
#     index += 1;
#     if (round(spos[1])==0 ):
#        print('Elevation 0: Value=',spos)
#WAIT = int(raw_input ("WAIT"));



#Audio playback

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

obj1_id = 1; #temp
while start<=(np.shape(obj1)[0]- chunk):


    #Getting head tracking information
    #data, addr = sock.recvfrom(1024);
    #mylist = data.split(",");
    #oZ = int(float(mylist[3]));  #Azimuth
    #oX = int(float(mylist[4]));
    #oY = int(float(mylist[5]));
    oZ=0;

    #Finding HRTF index for object 
    index = -1;
    for spos in f.variables['SourcePosition'][:]:
        index += 1;
        if (round(spos[1])==0 and 0 == spos[0]):  #  (360 + obj1_pos - oZ)
            obj1_id = index;
      
    print("ObjPos:", oZ, " SOFA id for obj1 :",obj1_id);


    obj1_Sample = obj1[start:start+chunk];
 

    l_obj1_Output = np.convolve(obj1_Sample, f.variables['Data.IR'][obj1_id][0][:], 'same');

    r_obj1_Output = np.convolve(obj1_Sample, f.variables['Data.IR'][obj1_id][1][:], 'same');

    
    loutput = l_obj1_Output ; #Sum of L OPs from all objects
    routput = r_obj1_Output ; #Sum of R OPs from all objects
    
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
    
    #print('Input size: ',obj1_Sample.shape[0],' HRIR size:',f.variables['Data.IR'][obj1_id][0][:].shape[0]);
    #print('Writing OP:',finalLOutput.shape[0],' frames');
    

    stream.write(interleavedOutput.astype('int16')) ;
    start = start + chunk/2;
    finalOutput = np.concatenate((finalOutput, loutput), axis=0);
    
#stop stream  
stream.stop_stream()  
stream.close()  

#close PyAudio  
p.terminate()





##Writing final output to a wav file

finalOutput = finalOutput.astype('int16');
#plt.plot(finalOutput);
#plt.show();

finalLOutput = loutput.astype('int16');
finalROutput = routput.astype('int16');

interleavedOutput = np.zeros([2 * finalLOutput.shape[0]]);
for i in range(finalLOutput.shape[0]):
    interleavedOutput[2*i] = finalLOutput[i];
    interleavedOutput[2*i+1] = finalROutput[i];
    
interleavedOutput = interleavedOutput.astype('int16');

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

