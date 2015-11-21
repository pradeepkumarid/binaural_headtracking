#include <stdio.h>
#include "Gamma/AudioIO.h"
#include "Gamma/Domain.h"
#include "Gamma/Oscillator.h"
#include "Gamma/Envelope.h"
#include "Gamma/Noise.h"
#include "Gamma/Delay.h"
#include "Gamma/Filter.h"
#include "Gamma/SamplePlayer.h"

#include <sys/socket.h>
#include <arpa/inet.h>

#include "../libsofa/src/SOFA.h"
#include "utilities.h"

#include <pthread.h>

using namespace gam;
using namespace std;

int frameCount    = CHUNK;
int samplingRate  = 44100;
int channelsIn      = 0;
int channelsOut     = 2;

float input  = 0.0;
float dryWet = 0.5;           // 0.0 for dry signal and 1.0 for wet signal
float level  = 0.1;  		  // Audio output level

NoiseWhite<> white;           // White Noise
AD<> env;                     // Attack/Decay envelope
Accum<> tmr;                  // Timer to reset AD envelope
SamplePlayer<> player;

sofa::SimpleFreeFieldHRIR HRIR("/home/pradeep/Q4/SpatialAudio/Project/CPP/subject_020.sofa");
vector< double > values;
double set[500000];

vector< size_t > source_dims;  //1250 x 3
vector< double > source_pos;     //Source Positions


unsigned int M,R,N;

float inputBuff[CHUNK];
float outputBuff_L[CHUNK];
float outputBuff_R[CHUNK];
float overlapBuff_L[200-1];
float overlapBuff_R[200-1];
int aziTimer = 0;
int azimuth = 0;


//UDP params
struct sockaddr_in si_me, si_other;
int s, i, slen = sizeof(si_other) , recv_len;
char buf[BUFLEN];
float oX, oY, oZ;
pthread_t orienListenerthread;
bool enableListen;

static void TestFileConvention(const string & filename,
		ostream & output = cout)
{

	const bool validnetCDF                  = sofa::IsValidNetCDFFile( filename );
	const bool validSOFA                    = sofa::IsValidSOFAFile( filename );
	const bool validSimpleFreeFieldHRIR     = sofa::IsValidSimpleFreeFieldHRIRFile( filename );
	const bool validSimpleFreeFieldSOS      = sofa::IsValidSimpleFreeFieldSOSFile( filename );
	const bool validSimpleHeadphoneIR       = sofa::IsValidSimpleHeadphoneIRFile( filename );
	const bool validGeneralFIR              = sofa::IsValidGeneralFIRFile( filename );
	const bool validGeneralTF               = sofa::IsValidGeneralTFFile( filename );

	output << "netCDF               = " << sofa::String::bool2yesorno( validnetCDF ) << endl;
	output << "SOFA                 = " << sofa::String::bool2yesorno( validSOFA ) << endl;
	output << "SimpleFreeFieldHRIR  = " << sofa::String::bool2yesorno( validSimpleFreeFieldHRIR ) << endl;
	output << "SimpleFreeFieldSOS   = " << sofa::String::bool2yesorno( validSimpleFreeFieldSOS ) << endl;
	output << "SimpleHeadphoneIR    = " << sofa::String::bool2yesorno( validSimpleHeadphoneIR ) << endl;
	output << "GeneralFIR           = " << sofa::String::bool2yesorno( validGeneralFIR ) << endl;
	output << "GeneralTF            = " << sofa::String::bool2yesorno( validGeneralTF ) << endl;
}

static void DisplayInformations(const string & filename,
		ostream & output = cout)
{
	///@n this doesnt check whether the file corresponds to SOFA conventions...
	const sofa::NetCDFFile file( filename );

	const string tabSeparator = "\t";

	//==============================================================================
	// global attributes
	//==============================================================================
	{
		vector< string > attributeNames;
		file.GetAllAttributesNames( attributeNames );

		output << endl;
		output << "Global Attributes:" << endl;

		for( size_t i = 0; i < attributeNames.size(); i++ )
		{
			const string name = attributeNames[i];
			const string value= file.GetAttributeValueAsString( name );

			output << tabSeparator << sofa::String::PadWith( attributeNames[i] ) << " = " << value << endl;
		}
	}

	//==============================================================================
	// dimensions
	//==============================================================================
	{
		vector< string > dimensionNames;
		file.GetAllDimensionsNames( dimensionNames );

		output << endl;
		output << "Dimensions:" << endl;

		for( size_t i = 0; i < dimensionNames.size(); i++ )
		{
			const string name = dimensionNames[i];
			const size_t dim  = file.GetDimension( name );
			output << tabSeparator << name << " = " << dim << endl;
		}
	}

	//==============================================================================
	// variables
	//==============================================================================
	{
		vector< string > variableNames;
		file.GetAllVariablesNames( variableNames );

		output << endl;
		output << "Variables:" << endl;

		for( size_t i = 0; i < variableNames.size(); i++ )
		{
			const string name      = variableNames[i];
			const string typeName  = file.GetVariableTypeName( name );

			const string dimsNames = file.GetVariableDimensionsNamesAsString( name );
			const string dims      = file.GetVariableDimensionsAsString( name );

			output << tabSeparator << name << endl;
			output << tabSeparator << tabSeparator << sofa::String::PadWith( "Datatype: " ) << typeName << endl;
			output << tabSeparator << tabSeparator << sofa::String::PadWith( "Dimensions: ") << dimsNames << endl;;
			output << tabSeparator << tabSeparator << sofa::String::PadWith( "Size: ") << dims << endl;;

			vector< string > attributeNames;
			vector< string > attributeValues;
			file.GetVariablesAttributes( attributeNames, attributeValues, name );

			SOFA_ASSERT( attributeNames.size() == attributeValues.size() );

			if( attributeNames.size() > 0 )
			{
				output << tabSeparator << tabSeparator << sofa::String::PadWith( "Attributes: ") << dims << endl;;
			}

			for( size_t j = 0; j < attributeNames.size(); j++ )
			{
				output << tabSeparator << tabSeparator << tabSeparator;
				output << sofa::String::PadWith( attributeNames[j] ) << " = " << attributeValues[j] << endl;
			}
		}

	}
}



void initHRTF()
{
	//Initializing Source Positions
	HRIR.GetVariableDimensions( source_dims, "SourcePosition" );
	SOFA_ASSERT( source_dims.size() == 2 );
	source_pos.resize( source_dims[0] * source_dims[1] );
	HRIR.GetSourcePosition( &source_pos[0], source_dims[0], source_dims[1] );

	M = (unsigned int) HRIR.GetNumMeasurements(); // 1250 Measurements
	R = (unsigned int) HRIR.GetNumReceivers();    // 2 Receivers
	N = (unsigned int) HRIR.GetNumDataSamples();  // 200 Num Samples

	HRIR.GetDataIR(values);  //1250  x 2 x 200
	cout << "Init : Data Size: " << values.size() << endl;
}

void initUDP()
{
	//create a UDP socket
	if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
	{
		cout<<"socket error";
	}

	// zero out the structure
	memset((char *) &si_me, 0, sizeof(si_me));

	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(PORT);
	si_me.sin_addr.s_addr = htonl(INADDR_ANY);

	//bind socket to port
	if( bind(s , (struct sockaddr*)&si_me, sizeof(si_me) ) == -1)
	{
		cout<<"bind error";
	}

}



void binauralCalculation()
{

	cout<<"\nOrientation: oX="<<oX<<":  oY="<<oY<<":  oZ="<<oZ;

	//
	//	if((++aziTimer)>=10)
	//	{
	//		aziTimer = 0;
	//		azimuth+=5;
	//		if(azimuth>=360)
	//			azimuth = 0;
	//
	//		cout<<"\nAzimuth = "<<azimuth;
	//	}

	azimuth = oZ;
	if(azimuth>=360)
		azimuth-=360;
	else if (azimuth<0)
		azimuth+=360;

	cout<<"\n Azimuth="<<azimuth;

	//Finding HRTF index for object
	int index = -1;
	int tmpIndex = -1;
	int IR_index = -1;

	float prevMinDiff = 999;
	for( size_t i = 0; i < source_dims[0]; i++ ) //idx
	{
		tmpIndex = array2DIndex(i, 0, source_dims[0], source_dims[1]);
		//tmpIndex => azimuth ; tmpIndex+1 =>elevation ; tmpIndex+2 =>height

		if (abs(source_pos[tmpIndex+1]) < 0.1) //check for elevation = 0 ; 0.1 is tolerance
		{
			if (abs(source_pos[tmpIndex] - azimuth) < prevMinDiff)
			{
				prevMinDiff = abs(source_pos[tmpIndex] - azimuth) ;
				index=i;
			}
		}
	}

	if(index ==-1)
	{
		cout<<"\nNo index found!!!";
		return;
	}

	//cout<<"\nIndex = "<<index<<" with diff="<<prevMinDiff<<" found azi="<<source_pos[index]<<" ele="<<source_pos[index+1];

	IR_index = array3DIndex(index, 0, 0, M, R, N);  //Left channel
	convolve(inputBuff, values, IR_index, outputBuff_L, overlapBuff_L);

	IR_index = array3DIndex(index, 1, 0, M, R, N);  //Right channel
	convolve(inputBuff, values, IR_index, outputBuff_R, overlapBuff_R);

}

void audioCallBack(AudioIOData& io)
{
	//cout<<"\n Callback";
	float outL, outR;
	int itr = 0;

	//Load IP buffer
	for (int i=0;i<frameCount;i++)
	{
		if(tmr()) env.reset();                // Reset AD envelope

		input = player(); //white() * env();              // Apply envelope to white noise

		inputBuff[i] = input;

	}

	//Binaural calculations
	binauralCalculation();


	while(io())
	{
		io.out(0) = outputBuff_R[itr];
		io.out(1) = outputBuff_L[itr];

		itr++;
	}
}


//Orientation listener thread

void *getOrientations(void *x_void_ptr)
{
	while(enableListen)
	{
		//try to receive some data, this is a blocking call
		if ((recv_len = recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *) &si_other, (socklen_t*)&slen)) == -1)
		{
			cout<<"recvfrom() error";
		}

		//Parsing buf
		string sBuf = buf;
		string delimiter = ",";
		size_t pos = 0;
		string token;

		int itr = 0;
		while ((pos = sBuf.find(delimiter)) != string::npos) {
			token = sBuf.substr(0, pos);
			if(itr==3) oZ = atof(token.c_str());
			else if(itr==4) oX = atof(token.c_str());
			sBuf.erase(0, pos + delimiter.length());
			itr++;
		}

		oY = atof(sBuf.c_str());

		//cout<<"\nThread : oZ="<<oZ;
	}
	return NULL;
}



int main()
{

	tmr.period(0.5);

	env.attack(0.01);           // Attack time
	env.decay(0.05);            // Decay time

	//Initializing Orientation listener
	initUDP();
	int rc;
	enableListen = true;
	rc = pthread_create(&orienListenerthread, NULL, getOrientations,NULL );

	if (rc){
		cout << "Error:unable to create orientation listener thread," << rc << endl;
		exit(-1);
	}



	const string filename = "/home/pradeep/Q4/SpatialAudio/Project/CPP/subject_020.sofa";

	TestFileConvention( filename );

	DisplayInformations( filename );

	initHRTF();


	//Intializing audio

	string filePath = WAV_FILE_NAME;
	player.load(filePath.c_str());
	player.loop();



	AudioIO audioIO(frameCount, samplingRate, audioCallBack, NULL, channelsOut, channelsIn);
	Sync::master().spu(audioIO.framesPerSecond());
	audioIO.start();
	printf("Press 'enter' to quit...\n");
	getchar();

	/* wait for the second thread to finish */
	enableListen = false;
	if(pthread_join(orienListenerthread, NULL)) {
		fprintf(stderr, "Error joining thread\n");
		return -2;
	}

	return 0;
}
