from __future__ import print_function 
from Scientific.IO.NetCDF import *
import numpy as np


fo = open("foo.txt", "wb")

f = NetCDFFile('/home/pradeep/Q4/SpatialAudio/HW/4/subject_020.sofa', 'r')

print(np.shape(f.variables['Data.IR']), np.shape(f.variables['SourcePosition']))
#(1250, 2, 200) (1250, 3)


index = -1;
for spos in f.variables['SourcePosition'][:]:
    index += 1;
    print(index,(f.variables['SourcePosition'][index][:]).astype('int'), file=fo)



# l1Output = np.convolve(frontLeftSample, f.variables['Data.IR'][id_FL][0][:], 'same');
   

fo.close()

