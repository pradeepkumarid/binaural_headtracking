#include <stdio.h>
#include "Gamma/AudioIO.h"
#include "Gamma/Domain.h"
#include "Gamma/Oscillator.h"
#include "Gamma/Envelope.h"
#include "Gamma/Noise.h"
#include "Gamma/Delay.h"
#include "Gamma/Filter.h"
#include "Gamma/SamplePlayer.h"


#include "../libsofa/src/SOFA.h"
#include "utilities.h"

using namespace gam;


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
std::vector< double > values;
double set[500000];

std::vector< std::size_t > source_dims;  //1250 x 3
std::vector< double > source_pos;     //Source Positions


unsigned int M,R,N;

float inputBuff[CHUNK];
float outputBuff_L[CHUNK];
float outputBuff_R[CHUNK];
float overlapBuff_L[200-1];
float overlapBuff_R[200-1];
int aziTimer = 0;
int azimuth = 0;


static void TestFileConvention(const std::string & filename,
		std::ostream & output = std::cout)
{

	const bool validnetCDF                  = sofa::IsValidNetCDFFile( filename );
	const bool validSOFA                    = sofa::IsValidSOFAFile( filename );
	const bool validSimpleFreeFieldHRIR     = sofa::IsValidSimpleFreeFieldHRIRFile( filename );
	const bool validSimpleFreeFieldSOS      = sofa::IsValidSimpleFreeFieldSOSFile( filename );
	const bool validSimpleHeadphoneIR       = sofa::IsValidSimpleHeadphoneIRFile( filename );
	const bool validGeneralFIR              = sofa::IsValidGeneralFIRFile( filename );
	const bool validGeneralTF               = sofa::IsValidGeneralTFFile( filename );

	output << "netCDF               = " << sofa::String::bool2yesorno( validnetCDF ) << std::endl;
	output << "SOFA                 = " << sofa::String::bool2yesorno( validSOFA ) << std::endl;
	output << "SimpleFreeFieldHRIR  = " << sofa::String::bool2yesorno( validSimpleFreeFieldHRIR ) << std::endl;
	output << "SimpleFreeFieldSOS   = " << sofa::String::bool2yesorno( validSimpleFreeFieldSOS ) << std::endl;
	output << "SimpleHeadphoneIR    = " << sofa::String::bool2yesorno( validSimpleHeadphoneIR ) << std::endl;
	output << "GeneralFIR           = " << sofa::String::bool2yesorno( validGeneralFIR ) << std::endl;
	output << "GeneralTF            = " << sofa::String::bool2yesorno( validGeneralTF ) << std::endl;
}

static void DisplayInformations(const std::string & filename,
		std::ostream & output = std::cout)
{
	///@n this doesnt check whether the file corresponds to SOFA conventions...
	const sofa::NetCDFFile file( filename );

	const std::string tabSeparator = "\t";

	//==============================================================================
	// global attributes
	//==============================================================================
	{
		std::vector< std::string > attributeNames;
		file.GetAllAttributesNames( attributeNames );

		output << std::endl;
		output << "Global Attributes:" << std::endl;

		for( std::size_t i = 0; i < attributeNames.size(); i++ )
		{
			const std::string name = attributeNames[i];
			const std::string value= file.GetAttributeValueAsString( name );

			output << tabSeparator << sofa::String::PadWith( attributeNames[i] ) << " = " << value << std::endl;
		}
	}

	//==============================================================================
	// dimensions
	//==============================================================================
	{
		std::vector< std::string > dimensionNames;
		file.GetAllDimensionsNames( dimensionNames );

		output << std::endl;
		output << "Dimensions:" << std::endl;

		for( std::size_t i = 0; i < dimensionNames.size(); i++ )
		{
			const std::string name = dimensionNames[i];
			const std::size_t dim  = file.GetDimension( name );
			output << tabSeparator << name << " = " << dim << std::endl;
		}
	}

	//==============================================================================
	// variables
	//==============================================================================
	{
		std::vector< std::string > variableNames;
		file.GetAllVariablesNames( variableNames );

		output << std::endl;
		output << "Variables:" << std::endl;

		for( std::size_t i = 0; i < variableNames.size(); i++ )
		{
			const std::string name      = variableNames[i];
			const std::string typeName  = file.GetVariableTypeName( name );

			const std::string dimsNames = file.GetVariableDimensionsNamesAsString( name );
			const std::string dims      = file.GetVariableDimensionsAsString( name );

			output << tabSeparator << name << std::endl;
			output << tabSeparator << tabSeparator << sofa::String::PadWith( "Datatype: " ) << typeName << std::endl;
			output << tabSeparator << tabSeparator << sofa::String::PadWith( "Dimensions: ") << dimsNames << std::endl;;
			output << tabSeparator << tabSeparator << sofa::String::PadWith( "Size: ") << dims << std::endl;;

			std::vector< std::string > attributeNames;
			std::vector< std::string > attributeValues;
			file.GetVariablesAttributes( attributeNames, attributeValues, name );

			SOFA_ASSERT( attributeNames.size() == attributeValues.size() );

			if( attributeNames.size() > 0 )
			{
				output << tabSeparator << tabSeparator << sofa::String::PadWith( "Attributes: ") << dims << std::endl;;
			}

			for( std::size_t j = 0; j < attributeNames.size(); j++ )
			{
				output << tabSeparator << tabSeparator << tabSeparator;
				output << sofa::String::PadWith( attributeNames[j] ) << " = " << attributeValues[j] << std::endl;
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
	std::cout << "Init : Data Size: " << values.size() << std::endl;
}


void binauralCalculation()
{

	if((++aziTimer)>=10)
	{
		aziTimer = 0;
		azimuth+=5;
		if(azimuth>=360)
			azimuth = 0;

		std::cout<<"\nAzimuth = "<<azimuth;
	}

	//Finding HRTF index for object
	int index = -1;
	int tmpIndex = -1;
	int IR_index = -1;

	float prevMinDiff = 999;
	for( std::size_t i = 0; i < source_dims[0]; i++ ) //idx
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
		std::cout<<"\nNo index found!!!";
		return;
	}

	//std::cout<<"\nIndex = "<<index<<" with diff="<<prevMinDiff<<" found azi="<<source_pos[index]<<" ele="<<source_pos[index+1];

	IR_index = array3DIndex(index, 0, 0, M, R, N);  //Left channel
	convolve(inputBuff, values, IR_index, outputBuff_L, overlapBuff_L);

	IR_index = array3DIndex(index, 1, 0, M, R, N);  //Right channel
	convolve(inputBuff, values, IR_index, outputBuff_R, overlapBuff_R);

}

void audioCallBack(AudioIOData& io)
{
	//std::cout<<"\n Callback";
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

int main()
{

	tmr.period(0.5);

	env.attack(0.01);           // Attack time
	env.decay(0.05);            // Decay time


	const std::string filename = "/home/pradeep/Q4/SpatialAudio/Project/CPP/subject_020.sofa";

	TestFileConvention( filename );

	DisplayInformations( filename );

	initHRTF();


	//Intializing audio

	std::string filePath = WAV_FILE_NAME;
	player.load(filePath.c_str());
	player.loop();



	AudioIO audioIO(frameCount, samplingRate, audioCallBack, NULL, channelsOut, channelsIn);
	Sync::master().spu(audioIO.framesPerSecond());
	audioIO.start();
	printf("Press 'enter' to quit...\n");
	getchar();
	return 0;
}
