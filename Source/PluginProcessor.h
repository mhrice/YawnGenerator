/*
  ==============================================================================

    This file was auto-generated!

    It contains the basic framework code for a JUCE plugin processor.

  ==============================================================================
*/

#pragma once

#include "../JuceLibraryCode/JuceHeader.h"
#include "./OscillatorParams.h"



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
	void pan(double input, float* left, float* right, int sample);

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



private:

    //==============================================================================
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (YawnGeneratorAudioProcessor)
};
