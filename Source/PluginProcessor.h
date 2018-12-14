/*
  ==============================================================================

    This file was auto-generated!

    It contains the basic framework code for a JUCE plugin processor.

  ==============================================================================
*/

#pragma once

#include "../JuceLibraryCode/JuceHeader.h"
#include "./OscillatorParams.h"

// tom's spectral additions
#if _WINDOWS
#define FFT_USED_FFTW 0
#define FFT_USED_APPLEVECLIB 0
#define FFT_USED_INTEL 1
#include <malloc.h>
#else
#define FFT_USED_FFTW 0
#define FFT_USED_APPLEVECLIB 1
#define FFT_USED_INTEL 0
#endif
#if FFT_USED_FFTW
#include <rfftw.h>
#elif FFT_USED_APPLEVECLIB
#define VIMAGE_H
#include <Accelerate/Accelerate.h>
#elif FFT_USED_INTEL
#include <ipps.h>
#include <ippcore.h>
#endif



const int TABLE_SIZE = 2048;
const int START_FREQ = 57;
//==============================================================================
/**
*/
class YawnGeneratorAudioProcessor  : public AudioProcessor
{
public:
    //==============================================================================
    YawnGeneratorAudioProcessor();
    ~YawnGeneratorAudioProcessor();

    //==============================================================================
    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override;

   #ifndef JucePlugin_PreferredChannelConfigurations
    bool isBusesLayoutSupported (const BusesLayout& layouts) const override;
   #endif

    void processBlock (AudioBuffer<float>&, MidiBuffer&) override;

    //==============================================================================
    AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override;

    //==============================================================================
    const String getName() const override;

    bool acceptsMidi() const override;
    bool producesMidi() const override;
    bool isMidiEffect() const override;
    double getTailLengthSeconds() const override;

    //==============================================================================
    int getNumPrograms() override;
    int getCurrentProgram() override;
    void setCurrentProgram (int index) override;
    const String getProgramName (int index) override;
    void changeProgramName (int index, const String& newName) override;

    //==============================================================================
    void getStateInformation (MemoryBlock& destData) override;
    void setStateInformation (const void* data, int sizeInBytes) override;
	AudioProcessorValueTreeState parameters;

	void createWavetables();
	double getNextSample();
	void setFrequency(int frequency);
	double linearInterpolateTableValues(int tableNumber);
	double midiToFrequency(int m);
	double ramp(double param, double start, double stop, bool up);
	double expRamp(double param, double start, double stop, bool up);
	double logRamp(double param, double start, double stop, bool up);
	double envelope(double input);
	double getNoise(double time);
	void updateCutoff();
	void updateFilter(double cutoff);
	double filter(double input);

	double tableSizeOverSampleRate;
	AudioSampleBuffer wavetablesA[8];

	//AudioSampleBuffer wavetableB750,
	//AudioSampleBuffer wavetableB1000,
	//AudioSampleBuffer wavetableB1250,
	//AudioSampleBuffer wavetableB1500,
	//AudioSampleBuffer wavetableB1750,
	//AudioSampleBuffer wavetableB2000
	double gain;
	bool trigger;
	int inputFreq;
	double t;
	double releaseT;
	double tDelta;
	double speed;
	double theSampleRate;

	double currentIndices[8];
	double currentPhases[8];
	double tableDeltas[8];

	Random r;
	double noise;

	// For filter
	double cutoff;
	double in1, in2, out1, out2;
	double a0, a1, a2, b1, b2;

	Reverb reverb;
	//Array<double> temp;
	AudioSampleBuffer temp;


	// tom's spectral additions
	enum
	{
		kSizeFFT = 1024,
		kHalfSizeFFT = 512,
		kMaxSizeFFT = 16384,
		kHalfMaxSizeFFT = 8192
	};
	enum
	{
		kMonoMode,
		kStereoMode,
		kMono2StereoMode
	};
	enum
	{
		kHamming,
		kVonHann,
		kBlackman,
		kKaiser
	};
	// spectral methods
	void initspect(int32_t numInputs);
	void termspect();

	bool allocateMemory();
	void clearMemory();   // clears memory
	void freeMemory();
	void setFFTSize(int32_t newSize);
	virtual void updateFFT(void);
	virtual void processActual(AudioSampleBuffer& buffer);
	virtual void processFFTBlock();
	virtual void processSpect();
	void cartToPolar(float *spectrum, float *polarSpectrum);
	void polarToCart(float *polarSpectrum, float *spectrum);
	void scaleWindows(void);
	void initHammingWindows(void);
	void initVonHannWindows(void);
	void initBlackmanWindows(void);
	void initKaiserWindows(void);
	float kaiserIno(float x);

	// spectral variables
	int32_t numInputs, numOutputs;
	int32_t windowSelected;
	int32_t inputTimeL, outputTimeL;
	int32_t inputTimeR, outputTimeR;
	int32_t bufferPosition;
	int32_t channelMode;
	int32_t blockSizeFFT, overLapFFT, halfSizeFFT, sizeFFT, log2n;
	float oneOverBlockSize;
	float pi, twoPi;

	// spectral buffers
	float *inShiftL, *inShiftR, *outShiftL, *outShiftR;
	float *inSpectraL, *inSpectraR, *outSpectraL, *outSpectraR;
	float *inBufferL, *inBufferR, *outBufferL, *outBufferR;
	float *inFFTL, *inFFTR;
	float *analysisWindow, *synthesisWindow;
#if FFT_USED_FFTW
	rfftw_plan planIn, planOut;
#elif FFT_USED_APPLEVECLIB
	FFTSetup setupReal;
	COMPLEX_SPLIT split;
#elif FFT_USED_INTEL
	IppsFFTSpec_R_32f* fftSpec[11];
	Ipp8u* mFFTSpecBuf[11];
	Ipp8u* mFFTWorkBuf[11];
	Ipp8u *mFFTInitBuf;
#endif
private:

    //==============================================================================
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (YawnGeneratorAudioProcessor)
};
