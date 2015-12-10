

#include "allocore/io/al_App.hpp"
#include <time.h>
#include <stdio.h>
#include "Gamma/AudioIO.h"
#include "Gamma/Domain.h"
#include "Gamma/Oscillator.h"
#include "Gamma/Envelope.h"
#include "Gamma/Noise.h"
#include "Gamma/Delay.h"
#include "Gamma/Filter.h"
#include "Gamma/SamplePlayer.h"
#include "allocore/math/al_Mat.hpp"
#include "allocore/math/al_Vec.hpp"
#include "allocore/spatial/al_HashSpace.hpp"

#include <math.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "../libsofa/src/SOFA.h"
#include "../ann_1.1.2/include/ANN/ANN.h"

#include "utilities.h"

#include <pthread.h>




using namespace gam;
using namespace al;
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

sofa::SimpleFreeFieldHRIR HRIR("/home/pradeep/Q4/SpatialAudio/Project/CPP/subject_020.sofa");   //020 / 003/ 051
//SOFA download from http://sofacoustics.org/data/database/cipic/

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

float azimuth = 0;
float elevation = 0;


//UDP params
struct sockaddr_in si_me, si_other;
int s, i, slen = sizeof(si_other) , recv_len;
char buf[BUFLEN];
float oX, oY, oZ;
pthread_t orienListenerthread;
bool enableListen;


al::Vec3f virtSourcePos; //X,Y,Z
al::Vec3f relSourcePos; //X' , Y', Z'
al::Mat3f rotMatrix; //R



//For k-nearest neighbours

int				k				= 3;			// number of nearest neighbors
int				dim				= 3;			// dimension
double			errBound		= 1;			// error bound
int				maxPts			= 1250;			// maximum number of data points; 1250 in this case

int					nPts;					// actual number of data points
ANNpointArray		dataPts;				// data points
ANNpoint			queryPt;				// query point
ANNidxArray			nnIdx;					// near neighbor indices
ANNdistArray		dists;					// near neighbor distances
ANNkd_tree*			kdTree;					// search structure



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

void initVirtualSource()
{
	//Audio Source Position
	virtSourcePos = al::Vec3f(0.0f,0.0f,1.0f);
	relSourcePos = al::Vec3f(0.0f,0.0f,1.0f);
}

void initNearestNeighbourDS()
{
	std::cout<<"\n initNearestNeighbourDS";
	queryPt = annAllocPt(dim);					    // allocate query point
	dataPts = annAllocPts(maxPts, dim);			// allocate data points
	nnIdx = new ANNidx[k];						// allocate near neigh indices
	dists = new ANNdist[k];						// allocate near neighbor dists

	nPts = 0; //source_dims.x

	//read data points
	int tmpIndex = -1;
	for( size_t i = 0; i < source_dims[0]; i++ )
	{

		tmpIndex = array2DIndex(i, 0, source_dims[0], source_dims[1]);  //source_dims = 1250 x 3

		//x = sin φ cos θ, y = sin θ, z = cos φ cos θ.  φ-azi   θ-ele
		float phi = source_pos[tmpIndex] * PI / 180.0f;
		float theta = source_pos[tmpIndex+1] * PI / 180.0f;
		//float length = source_pos[tmpIndex+2]; //Useless as of now. =1 always

		//Store the points in cartesian coords
		dataPts[i][0] = sin (phi) * cos(theta);
		dataPts[i][1] = sin (theta);
		dataPts[i][2] = cos (phi) * cos (theta);

		//cout<<i<<" :"<<tmpIndex<<":  "<<dataPts[i][0]<<":"<<dataPts[i][1]<<":"<<dataPts[i][2]<<endl;

	}

	kdTree = new ANNkd_tree(					// build search structure
			dataPts,					// the data points
			source_dims[0],						// number of points
			dim);						// dimension of space

	cout<<"\nCreated kdtree with nPts= "<<kdTree->nPoints();

}

void binauralCalculation()
{

	////Hardcoding Headtracking data
		//oX = 0; oY=0; oZ=0;

		//cout<<"\n oX="<<oX<<" :oY="<<oY<<" :oZ="<<oZ;
		loadRotMatrix(rotMatrix, oX, oZ,oY);
	//	relSourcePos = inverse(rotMatrix) * virtSourcePos;
		relSourcePos = (rotMatrix.transpose()) * virtSourcePos;  //Transpose of rot mat is same as inv
		//cout<<"\n RelSourcePos = "<<relSourcePos;

	//	rotPoseR.quat().fromEuler(al::Vec3f(oX,oY,oZ));
	//	relSourcePos = inverse(rotPoseR.))) * virtSourcePos;


		//cout<<"\nRelSourcePos:"<<relSourcePos.x<<":"<<relSourcePos.y<<":"<<relSourcePos.z;
		float R1 = sqrt(relSourcePos.x * relSourcePos.x + relSourcePos.y * relSourcePos.y + relSourcePos.z* relSourcePos.z);
		elevation = asin(relSourcePos.y / R1);
		azimuth = asin(relSourcePos.x / (R1*elevation));

		//float theta = elevation * 180 / PI;
		//float phi = azimuth * 180 / PI;

		////Hardcoding theta, phi
		//	phi = 150 ;  theta = 0;
		//
		//	cout<<"\nRel Azi="<<phi<<" Ele="<<theta;
		//	phi = phi * PI / 180;
		//	theta = theta * PI / 180;

		//Finding HRTF index for object
		int index = -1;
		int tmpIndex = -1;
		int IR_index = -1;

		queryPt[0] = relSourcePos.x;//sin (azimuth) * cos(elevation);
		queryPt[1] = relSourcePos.y;//sin (elevation);
		queryPt[2] = relSourcePos.z;//cos (azimuth) * cos (elevation);

		//cout<<"\n New Source Pos : "<<queryPt[0]<<":"<<queryPt[1]<<":"<<queryPt[2];

		kdTree->annkSearch(						// search
				queryPt,						// query point
				k,								// number of near neighbors
				nnIdx,							// nearest neighbors (returned)
				dists,							// distance (returned)
				errBound);						// error bound

		//cout<<"\n Indices : "<<nnIdx[0]<<","<<nnIdx[1]<<","<<nnIdx[2];
		index =nnIdx[0];
		tmpIndex = array2DIndex(index, 0, source_dims[0], source_dims[1]);

		//cout<<"\nKD : Index = "<<index<<" with diff="<<dists[0]<<" found azi="<<source_pos[tmpIndex]<<" ele="<<source_pos[tmpIndex+1];

		IR_index = array3DIndex(index, 0, 0, M, R, N);  //Left channel
		convolve(inputBuff, values, IR_index, outputBuff_L, overlapBuff_L);

		IR_index = array3DIndex(index, 1, 0, M, R, N);  //Right channel
		convolve(inputBuff, values, IR_index, outputBuff_R, overlapBuff_R);

}


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







struct SphereObject {
	int id;
	Vec3f pos;
	Vec3f origPos;
	float rad;
};





struct AlloApp: App {
	Material material;
	Light light;
	Mesh m, m2;

	SphereObject ball;

	SphereObject audioSrc;

	float snd;

	AlloApp() {
		cout << "Created AlloApp" << endl;

		snd = 0;

		nav().pos(0, 0, -10);
		nav().faceToward(Vec3d(0,0,0));
		light.pos(2, 3, 10);


		ball.pos = Vec3f(0,0,0);
		ball.rad = 0.5;


		addSphere(m, 2);
		m.primitive(Graphics::LINES);


		audioSrc.pos = Vec3f(0,0,0);
		audioSrc.rad = 0.05;
		addSphere(m2, 2);

		m.generateNormals();
		m2.generateNormals();


		initWindow();
		initAudio(44100, 1024);


		tmr.period(0.5);

		env.attack(0.01);           // Attack time
		env.decay(0.05);            // Decay time

		//Initializing Orientation listener
		initUDP();
		initVirtualSource();

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

		initNearestNeighbourDS();
		//Intializing audio

		string filePath = WAV_FILE_NAME;
		player.load(filePath.c_str());
		player.loop();


		gam::Sync::master().spu(audioIO().fps());
	}


	virtual void onAnimate(double dt) {
		audioSrc.pos = relSourcePos;
		audioSrc.origPos = virtSourcePos;
	}

	virtual void onDraw(Graphics& g, const Viewpoint& v) {
		material();
		light();


		//Reder sphere mesh 1
		g.pushMatrix();
		g.color(HSV(1,1,0.5));
		g.translate(ball.pos + Vec3f(1.5f, 0.,0.));
		g.scale(ball.rad);
		g.draw(m);
		g.popMatrix();


		//Reder sphere mesh 2
		g.pushMatrix();
		g.color(HSV(0,1,0.5));
		g.translate(ball.pos + Vec3f(-1.5f, 0.,0.));
		g.scale(ball.rad);
		g.draw(m);
		g.popMatrix();

		//Render audio obj
		g.pushMatrix();
		g.color(HSV(0.2+(1+audioSrc.pos.z)*0.8/2.0, 0.5,1));
		g.translate(audioSrc.pos + Vec3f(1.5f, 0.,0.));
		g.scale(audioSrc.rad);
		g.draw(m2);
		g.popMatrix();

		g.pushMatrix();
		g.color(HSV(0.2+(1+audioSrc.origPos.z)*0.8/2.0, 0.5,1));
		g.translate(audioSrc.origPos + Vec3f(-1.5f, 0.,0.));
		g.scale(audioSrc.rad);
		g.draw(m2);
		g.popMatrix();



	}

	virtual void onSound(al::AudioIOData& io) {

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

	virtual void onKeyDown(const ViewpointWindow&, const Keyboard& k) {
		if (k.key() == ' ') {

		}
	}

	virtual void onMouseDown(const Mouse &m) {
		if (m.button() == m.LEFT) {

		}
	}

};

int main(int argc, char* argv[]) {
	AlloApp app;
	app.start();

	/* wait for the second thread to finish */
	enableListen = false;
	if(pthread_join(orienListenerthread, NULL)) {
		fprintf(stderr, "Error joining thread\n");
		return -2;
	}

	return 0;
}
