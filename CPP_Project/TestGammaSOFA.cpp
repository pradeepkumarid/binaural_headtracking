#include <stdio.h>
#include "Gamma/AudioIO.h"
#include "Gamma/Domain.h"
#include "Gamma/Oscillator.h"
#include "Gamma/Envelope.h"
#include "Gamma/Noise.h"
#include "Gamma/Delay.h"
#include "Gamma/Filter.h"

using namespace gam;

int frameCount    = 512;
int samplingRate  = 44100;
int channelsIn    = 0;
int channelsOut   = 2;

float input  = 0.0;
float output = 0.0;
float dryWet = 0.5;           // 0.0 for dry signal and 1.0 for wet signal
float level  = 0.1;           // Audio output level

float decayTime = 0.5;        // Decay time of the reverb (Play with this)

NoiseWhite<> white;           // White Noise
AD<> env;                     // Attack/Decay envelope
Accum<> tmr;                  // Timer to reset AD envelope

//Filters

class NestedAllPass
{
	float prevOutput;
	float D1, D2, K1, K2;
	Comb<> apf;
	Delay<> delay1;
public:
	NestedAllPass(float _D1, float _D2, float _K1, float _K2):
		D1(_D1),D2(_D2),K1(_K1),K2(_K2),prevOutput(0.0f)
	{
		apf.set(D2,K2,-K2);
		apf.maxDelay(D2);
		delay1.delay(D1);
		delay1.maxDelay(D1);
	}

	float operator()(float in)
	{
		float op = apf(delay1(in + (-K1*prevOutput))) + K1*in;
		prevOutput = op;
		return op;
	}

};

class DoubleNestedAllPass
{
	float prevOutput;
	float D1, D2, D3, K1, K2, K3;
	Comb<> apf1, apf2;
	Delay<> delay1;
public:
	DoubleNestedAllPass(float _D1, float _D2, float _D3,float _K1, float _K2, float _K3):
		D1(_D1),D2(_D2),D3(_D3),K1(_K1),K2(_K2),K3(_K3),prevOutput(0.0f)
	{
		apf1.set(D2,K2,-K2);
		apf1.maxDelay(D2);
		apf2.set(D3,K3,-K3);
		apf2.maxDelay(D3);
		delay1.delay(D1);
		delay1.maxDelay(D1);
	}

	float operator()(float in)
	{
		float op = apf2(apf1(delay1(in + (-K1*prevOutput)))) + K1*in;
		prevOutput = op;
		return op;
	}


};

OnePole<> lpf;
Delay<> firstDelay(0.024,0.024);
DoubleNestedAllPass dnap1(0.035,0.022,0.0083,0.3,0.4,0.6);
NestedAllPass nap1(0.066,0.030,0.1,0.4);
float prevTap3;



void roomReverb(float in, float& out, float decayTime)
{
    float gain = decayTime;    // Gain value to control decay time
    
    // Reverb implementation

    float tap0, tap1, tap2;

    tap0 = firstDelay(in+ gain* lpf(prevTap3));

    tap1 = dnap1(tap0);

    tap2 = nap1(tap1);

    out = 0.5 * tap1 + 0.5 * tap2;

    prevTap3 = tap2;

}

// DO NOT MODIFY THE AUDIO CALLBACK FUNCTION
void audioCallBack(AudioIOData& io)
{
    while(io())
    {
        if(tmr()) env.reset();                // Reset AD envelope

        input = white() * env();              // Apply envelope to white noise
        roomReverb(input, output, decayTime); // Apply reverb
        
        for (int i = 0; i < channelsOut; i++)
        {
            io.out(i) = ( (1.0 - dryWet) * input + dryWet * output ) * level;
        }
    }
}

int main()
{
    tmr.period(3);              // Reset the envelope every 3 seconds
    env.attack(0.01);           // Attack time
    env.decay(0.05);            // Decay time
    
    // Delays, lowpass, and allpass filters setup code goes here

	lpf.type(LOW_PASS);
	lpf.freq(4200);
	prevTap3 = 0;

    AudioIO audioIO(frameCount, samplingRate, audioCallBack, NULL, channelsOut, channelsIn);
    Sync::master().spu(audioIO.framesPerSecond());
    audioIO.start();
    printf("Press 'enter' to quit...\n");
    getchar();
    return 0;
}

