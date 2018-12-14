/*
  ==============================================================================

    This file was auto-generated!

    It contains the basic framework code for a JUCE plugin processor.

  ==============================================================================
*/

#include "PluginProcessor.h"
#include "PluginEditor.h"
#include <windows.h>
//==============================================================================
YawnGeneratorAudioProcessor::YawnGeneratorAudioProcessor()
	:parameters(*this, nullptr),
#ifndef JucePlugin_PreferredChannelConfigurations
      AudioProcessor (BusesProperties()
                     #if ! JucePlugin_IsMidiEffect
                      #if ! JucePlugin_IsSynth
                       .withInput  ("Input",  AudioChannelSet::stereo(), true)
                      #endif
                       .withOutput ("Output", AudioChannelSet::stereo(), true)
                     #endif
                       )
#endif
{

	parameters.createAndAddParameter(std::make_unique<AudioParameterFloat>("speed", // parameter ID
		"speed", // parameter name
		NormalisableRange<float>(1, 10, 1), // range
		5.0f, // default value
		"dB"));
	parameters.createAndAddParameter(std::make_unique<AudioParameterFloat>("yawnAir", // parameter ID
		"yawnAir", // parameter name
		NormalisableRange<float>(1, 10, 1), // range
		5.0, // default value
		"dB"));

	parameters.state = ValueTree(Identifier("YawnGenerator"));
	for (int i = 0; i < numElementsInArray(currentIndices); i++) 
	{
		currentIndices[i] = 0.0;
		tableDeltas[i] = 0.0;
	}
	createWavetables();
	t = 0;
	releaseT = 0;
	trigger = false;
	cutoff = 300;
	in1 = in2 = out1 = out2 = 0.0;
	gain = 0;
	speed = 1;
	noise = 0;
	initspect(2);
	setFFTSize(1024);
}

YawnGeneratorAudioProcessor::~YawnGeneratorAudioProcessor()
{
	termspect();
}

//==============================================================================
const String YawnGeneratorAudioProcessor::getName() const
{

    return JucePlugin_Name;
}

bool YawnGeneratorAudioProcessor::acceptsMidi() const
{
   #if JucePlugin_WantsMidiInput
    return true;
   #else
    return false;
   #endif
}

bool YawnGeneratorAudioProcessor::producesMidi() const
{
   #if JucePlugin_ProducesMidiOutput
    return true;
   #else
    return false;
   #endif
}

bool YawnGeneratorAudioProcessor::isMidiEffect() const
{
   #if JucePlugin_IsMidiEffect
    return true;
   #else
    return false;
   #endif
}

double YawnGeneratorAudioProcessor::getTailLengthSeconds() const
{
    return 0.0;
}

int YawnGeneratorAudioProcessor::getNumPrograms()
{
    return 1;   // NB: some hosts don't cope very well if you tell them there are 0 programs,
                // so this should be at least 1, even if you're not really implementing programs.
}

int YawnGeneratorAudioProcessor::getCurrentProgram()
{
    return 0;
}

void YawnGeneratorAudioProcessor::setCurrentProgram (int index)
{
}

const String YawnGeneratorAudioProcessor::getProgramName (int index)
{
    return {};
}

void YawnGeneratorAudioProcessor::changeProgramName (int index, const String& newName)
{
}

//==============================================================================
void YawnGeneratorAudioProcessor::prepareToPlay (double sampleRate, int samplesPerBlock)
{
    // Use this method as the place to do any pre-playback
    // initialisation that you need..
	theSampleRate = sampleRate;
	tDelta = 1.0f / theSampleRate;
	setFrequency(START_FREQ);
	updateFilter(cutoff);
	reverb.setSampleRate(theSampleRate);
}

void YawnGeneratorAudioProcessor::releaseResources()
{
    // When playback stops, you can use this as an opportunity to free up any
    // spare memory, etc.
}

#ifndef JucePlugin_PreferredChannelConfigurations
bool YawnGeneratorAudioProcessor::isBusesLayoutSupported (const BusesLayout& layouts) const
{
  #if JucePlugin_IsMidiEffect
    ignoreUnused (layouts);
    return true;
  #else
    // This is the place where you check if the layout is supported.
    // In this template code we only support mono or stereo.
    if (layouts.getMainOutputChannelSet() != AudioChannelSet::mono()
     && layouts.getMainOutputChannelSet() != AudioChannelSet::stereo())
        return false;

    // This checks if the input layout matches the output layout
   #if ! JucePlugin_IsSynth
    if (layouts.getMainOutputChannelSet() != layouts.getMainInputChannelSet())
        return false;
   #endif

    return true;
  #endif
}
#endif

void YawnGeneratorAudioProcessor::processBlock (AudioBuffer<float>& buffer, MidiBuffer& midiMessages)
{
    ScopedNoDenormals noDenormals;
    auto totalNumInputChannels  = getTotalNumInputChannels();
    auto totalNumOutputChannels = getTotalNumOutputChannels();

    // In case we have more outputs than inputs, this code clears any output
    // channels that didn't contain input data, (because these aren't
    // guaranteed to be empty - they may contain garbage).
    // This is here to avoid people getting screaming feedback
    // when they first compile a plugin, but obviously you don't need to keep
    // this code if your algorithm always overwrites all the output channels.
	for (auto i = totalNumInputChannels; i < totalNumOutputChannels; ++i) {
		buffer.clear(i, 0, buffer.getNumSamples());
		temp.clear(i, 0, buffer.getNumSamples());
	}
	temp.setSize(1, buffer.getNumSamples());
	MidiMessage m;
	int time;

	for (MidiBuffer::Iterator i(midiMessages); i.getNextEvent(m, time);)
	{
		if (m.isNoteOn())
		{
			cutoff = 300;
			updateFilter(cutoff);
			setFrequency(m.getNoteNumber());
			gain = m.getFloatVelocity();
			trigger = true;

			t = 0;


		}
		else if (m.isNoteOff())
		{
			trigger = false;
			//gain = 0;
			if (t < 1.75) {
				t = 1.5;
			}
			else {
				gain = 0;
			}

		}
		else if (m.isAftertouch())
		{
		}
		else if (m.isPitchWheel())
		{
		}
	}



    // This is the place where you'd normally do the guts of your plugin's
    // audio processing...
    // Make sure to reset the state if your inner loop is processing
    // the samples and the outer loop is handling the channels.
    // Alternatively, you can process the samples with the channels
    // interleaved by keeping the same state.
		auto* left = buffer.getWritePointer(0);
		auto* right = buffer.getWritePointer(1);
		auto* tempW = temp.getWritePointer(0);


		updateCutoff();
		updateFilter(cutoff);

	for (auto sample = 0; sample < buffer.getNumSamples(); sample++) {


		double filteredSample = filter(getNextSample());
		double envelopedSample = envelope(filteredSample);
		//temp[sample] = envelopedSample;
		//temp.add(envelopedSample);
		tempW[sample] = envelopedSample;
		//temp.add(envelopedSample);
		//left[sample] = envelopedSample*gain*0.8;
		//right[sample] = left[sample];
		//if (phase >= TABLE_SIZE) phase = 0.0;


		//}
    }

	reverb.processMono(temp.getWritePointer(0), buffer.getNumSamples());
	auto* tempR = temp.getReadPointer(0);
	for (auto sample = 0; sample < buffer.getNumSamples(); sample++) {
		//if (sample % 10 == 0) {
			//char msgbuf[2048];
			//sprintf(msgbuf, "My variable is %f\n", tempR[sample]);
			//OutputDebugString(msgbuf);
		//}
		left[sample] = tempR[sample]*gain*0.8;
		right[sample] = left[sample];
	}
}

//==============================================================================
bool YawnGeneratorAudioProcessor::hasEditor() const
{
    return true; // (change this to false if you choose to not supply an editor)
}

AudioProcessorEditor* YawnGeneratorAudioProcessor::createEditor()
{
    return new YawnGeneratorAudioProcessorEditor (*this);
}

//==============================================================================
void YawnGeneratorAudioProcessor::getStateInformation (MemoryBlock& destData)
{
    // You should use this method to store your parameters in the memory block.
    // You could do that either as raw data, or use the XML or ValueTree classes
    // as intermediaries to make it easy to save and load complex data.
}

void YawnGeneratorAudioProcessor::setStateInformation (const void* data, int sizeInBytes)
{
    // You should use this method to restore your parameters from this memory block,
    // whose contents will have been created by the getStateInformation() call.
}

//==============================================================================
// This creates new instances of the plugin..
AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new YawnGeneratorAudioProcessor();
}

double YawnGeneratorAudioProcessor::getNextSample()
{

	double output = 0.0;
	if (trigger) {
		if (t < 0.25) {
			output = linearInterpolateTableValues(0)*logRamp(t, 0, 0.25, false) + linearInterpolateTableValues(1)*logRamp(t, 0, 0.25, true);
			double start = midiToFrequency(inputFreq - 2);
			double stop  = midiToFrequency(inputFreq);
			double dist = stop - start;
			//char msgbuf[2048];
			//sprintf(msgbuf, "My variable is %f %f %f %f\n", dist, start, tableDeltas[0], ramp(t, 0, 0.25, false)*dist + start);
			//OutputDebugString(msgbuf);
			tableDeltas[0] = (dist * ramp(t, 0, 0.25, false) + start) * tableSizeOverSampleRate;
		}
		else if (t < 0.5) {
			output = linearInterpolateTableValues(1)*logRamp(t, 0.25, 0.5, false); +linearInterpolateTableValues(2)*logRamp(t, 0.25, 0.5, true);
			double start = midiToFrequency(inputFreq - 5);
			double stop  = midiToFrequency(inputFreq - 2);
			double dist = stop - start;
			tableDeltas[1] = (dist * ramp(t, 0.25, 0.5, false) + start) * tableSizeOverSampleRate;
		}
		else if (t < 0.75) {
			output = linearInterpolateTableValues(2)*logRamp(t, 0.5, 0.75, false) +linearInterpolateTableValues(3)*expRamp(t, 0.5, 0.75, true);
			double start = midiToFrequency(inputFreq - 9);
			double stop  = midiToFrequency(inputFreq - 5);
			double dist = stop - start;
			tableDeltas[2] = (dist * ramp(t, 0.5, 0.75, false) + start) * tableSizeOverSampleRate;

		}
		else if (t < 1) {
			output = linearInterpolateTableValues(3)*logRamp(t, 0.75, 1, false) +linearInterpolateTableValues(4)*expRamp(t, 0.75, 1, true);
			double start = midiToFrequency(inputFreq - 13);
			double stop  = midiToFrequency(inputFreq - 9);
			double dist = stop - start;
			tableDeltas[3] = (dist * expRamp(t, 0.75, 1, false) + start) * tableSizeOverSampleRate;
		}
		else if (t < 1.25) {
			output = linearInterpolateTableValues(4)*logRamp(t, 1, 1.25, false) + linearInterpolateTableValues(5)*expRamp(t, 1, 1.25, true);
			double start = midiToFrequency(inputFreq - 15);
			double stop  = midiToFrequency(inputFreq - 13);
			double dist = stop - start;
			tableDeltas[4] = (dist * ramp(t, 1, 1.25, false) + start) * tableSizeOverSampleRate;
		}
		else if (t < 1.5) {
			output = linearInterpolateTableValues(5)*logRamp(t, 1.25, 1.5, false) + linearInterpolateTableValues(6)*expRamp(t, 1.25, 1.5, true);
			double start = midiToFrequency(inputFreq - 17);
			double stop  = midiToFrequency(inputFreq - 15);
			double dist = stop - start;
			tableDeltas[5] = (dist * ramp(t, 1.25, 1.5, false) + start) * tableSizeOverSampleRate;

		}
		else if (t < 1.75) {
			output = linearInterpolateTableValues(6)*logRamp(t, 1.5, 1.75, false) + linearInterpolateTableValues(7)*logRamp(t, 1.5, 1.75, true);
			double start = midiToFrequency(inputFreq - 24);
			double stop  = midiToFrequency(inputFreq - 17);
			double dist = stop - start;
			tableDeltas[6] = (dist * expRamp(t, 1.5, 1.75, false) + start) * tableSizeOverSampleRate;
			//char msgbuf[2048];
			//sprintf(msgbuf, "My variable is %f\n", pow(2.0, 11/12));
			//OutputDebugString(msgbuf);
		}
		else if (t < 2) {
			output = linearInterpolateTableValues(7)*expRamp(t, 1.75, 2.0, false);
			//char msgbuf[2048];
			//sprintf(msgbuf, "My variable is %d\n", 3);
			//OutputDebugString(msgbuf);
		}
		double noise = getNoise(t);
		output += noise;
		//else if (t < 2.25) {
		//	output = r.nextDouble() - 0.5;
		//}
		t += (tDelta*speed);
	}
	else {
		if (t < 1.75) {
			output = linearInterpolateTableValues(7)*expRamp(t, 1.5, 2, false);
			double noise = getNoise(t);
			output += noise;
			t += (tDelta);
		}

	}

	if (t > 5.5) t = 2.5;
	if (output < 0.001) output = 0;

	return output;
}


double YawnGeneratorAudioProcessor::getNoise(double time) 
{
	if (time < 1 || time > 1.75) return 0;

	double x = r.nextFloat()*2.0-1.0;
	double gain = parameters.getParameter("yawnAir")->getValue()*4;
	return x * log(time)*gain;
}

void YawnGeneratorAudioProcessor::updateCutoff()
{
	double function = t / (1 + fabs(t));
	double dist = 4000;
	double start = 300;
	cutoff = dist * function + start;
}
void YawnGeneratorAudioProcessor::setFrequency(int frequencyInMidi) 
{


	tableSizeOverSampleRate = TABLE_SIZE / theSampleRate;

	tableDeltas[0] = midiToFrequency(frequencyInMidi) * tableSizeOverSampleRate;
	tableDeltas[1] = midiToFrequency(frequencyInMidi - 2) * tableSizeOverSampleRate;
	tableDeltas[2] = midiToFrequency(frequencyInMidi - 5) * tableSizeOverSampleRate;
	tableDeltas[3] = midiToFrequency(frequencyInMidi - 9) * tableSizeOverSampleRate;
	tableDeltas[4] = midiToFrequency(frequencyInMidi - 13) * tableSizeOverSampleRate;
	tableDeltas[5] = midiToFrequency(frequencyInMidi - 15) * tableSizeOverSampleRate;
	tableDeltas[6] = midiToFrequency(frequencyInMidi - 17) * tableSizeOverSampleRate;
	tableDeltas[7] = midiToFrequency(frequencyInMidi - 24) * tableSizeOverSampleRate;

	inputFreq = frequencyInMidi;
	//char msgbuf[2048];
	//sprintf(msgbuf, "My variable is %f\n", pow(2.0, 11/12));
	//OutputDebugString(msgbuf);

}
double YawnGeneratorAudioProcessor::midiToFrequency(int m) 
{
	if (m < 0) m = 0;
	return (m > 0 && m < 128) ? pow(2.0, (m - 69) / 12.0) * 440: 440;
}
double YawnGeneratorAudioProcessor::ramp(double param, double start, double stop, bool up)
{
	double r = stop - start;
	double y = (param - start) / r;
	return up ? y : 1 - y;
}
double YawnGeneratorAudioProcessor::expRamp(double param, double start, double stop, bool up)
{

	if (start == 0) start = 0.001;
	if (stop == 0) stop = 0.001;
	//solving y=Ae^bx for y

	double newMax = 1.0;
	double newMin = 0.001;
	if (!up) {
		newMax = 0.001;
		newMin = 1.0;
	}
	double b = log(newMax / newMin) / (stop - start);
	double a = newMax / pow(MathConstants<double>::euler, stop* b);
	double y = a * pow(MathConstants<double>::euler, b*param);


	return y;

}
double YawnGeneratorAudioProcessor::envelope(double input)
{

	double function = 2.5*cos(0.80*t);

	return input * function;
}
double YawnGeneratorAudioProcessor::logRamp(double param, double start, double stop, bool up)
{

	if (start == 0) start = 0.001;
	if (stop == 0) stop = 0.001;
	//solving y=logax for y

	double newMax = 1.0;
	double newMin = 0.001;
	if (!up) {
		newMax = 0.001;
		newMin = 1.0;
	}
	//double b = log(newMax / newMin) / (stop - start);
	//double a = newMax / pow(MathConstants<double>::euler, stop* b);
	//double y = log((MathConstants<double>::euler, b*param))/log(a);
	double y = (exp(param) - 1) / (MathConstants<double>::euler - 1);

	return y;

}

double YawnGeneratorAudioProcessor::linearInterpolateTableValues(int tableNumber)
{

	auto index0 = (unsigned int)currentIndices[tableNumber];
	auto index1 = index0 + 1;
	auto* table = wavetablesA[tableNumber].getWritePointer(0);
	auto frac = currentIndices[tableNumber] - (float)index0;
	auto output = table[index0] + frac * (table[index1] - table[index0]);
	if ((currentIndices[tableNumber] += tableDeltas[tableNumber]) > TABLE_SIZE) {
		currentIndices[tableNumber] -= TABLE_SIZE;
	}


	return output;
}

void YawnGeneratorAudioProcessor::createWavetables()
{
	//char msgbuf[2048];
	//sprintf(msgbuf, "My variable is CALLEd\n");
	//OutputDebugString(msgbuf);

	int harmonics[] = { 1,2,3,4,5,6,7,8,9,10,11,12,13,14 };

	for (int i = 0; i < numElementsInArray(wavetablesA); i++)
	{
		//char msgbuf[2048];
		//sprintf(msgbuf, "My variable is %f\n", i);
		//OutputDebugString(msgbuf);
		if (i == 7) {
			bool x = true;
		}
		wavetablesA[i].setSize(1, TABLE_SIZE + 1);
		wavetablesA[i].clear();

		auto* samples = wavetablesA[i].getWritePointer(0);

		//float harmonicWeights[] = { 0.75f, 0.1f, 0.05f, 0.333f, 0.12, 0.09f, 0.1, 0.005, 0.002f, 0.001f };

		for (int harmonic = 0; harmonic < numElementsInArray(harmonics); ++harmonic)
		{
			auto angleDelta = MathConstants<double>::twoPi / (double)(TABLE_SIZE - 1) * harmonics[harmonic];
			auto currentAngle = 0.0;
			for (auto j = 0; j < TABLE_SIZE; ++j)
			{
				auto sample = std::sin(currentAngle);
				samples[j] += (double)sample * weights[i][harmonic];

				currentAngle += angleDelta;
				if (i == 7 && samples[j]==INFINITY) {
					char msgbuf[2048];
					sprintf(msgbuf, "My variable is %d %d %f %f\n", numElementsInArray(harmonics), harmonic, weights[i][harmonic], weights[i][14]);
					OutputDebugString(msgbuf);
				}
			}

		}

		samples[TABLE_SIZE] = samples[0];

	}

	//if (tableNumber == 7) {
		//auto *table = wavetablesA[7].getReadPointer(0);
		//for (int i = 0; i < TABLE_SIZE; i++) {
			//char msgbuf[2048];
			//sprintf(msgbuf, "My variable is %f\n", table[i]);
			//OutputDebugString(msgbuf);
		//}
	//}


}

void YawnGeneratorAudioProcessor::updateFilter(double cutoff) {
	//double omegaC = (MathConstants<double>::twoPi * 20.0) / theSampleRate;
	//double Q = 1.0 / 10000.0;
	//double beta = 0.5*((1 - tan(omegaC / (2 * Q))) / (1 + tan(omegaC / (2 * Q))));
	//double gamma = (0.5 + beta)*cos(omegaC);
	////double gamma = cos(omegaC) / (1 + dsp::FastMathApproximations::sin(omegaC));
	//a0 = 0.5 - beta;
	//a1 = 0.0;
	//a2 = -(0.5 - beta);
	//b1 = -2 * gamma;
	//b2 = 2 * beta;
	//From Pirckle
	double omegaC = (MathConstants<double>::twoPi * cutoff) / theSampleRate;
	double gamma = dsp::FastMathApproximations::cos(omegaC) / (1 + dsp::FastMathApproximations::sin(omegaC));
	a0 = (1 - gamma) / 2;
	a1 = a0;
	a2 = 0.0;
	b1 = -gamma;
	b2 = 0.0;
}

double YawnGeneratorAudioProcessor::filter(double input) {
	//difference equation
	double output = a0 * input + a1 * in1 + a2 * in2 - b1 * out1 - b2 * out2;
	in1 = input;
	in2 = in1;
	out1 = output;
	out2 = out1;

	return output;

}




void YawnGeneratorAudioProcessor::initspect(int32_t pNumInputs)
{
	numOutputs = 2;
	numInputs = pNumInputs;

	// zero  buffer pointers
	outBufferL = outBufferR = inBufferL = inBufferR = 0;
	inFFTL = inFFTR = inShiftL = inShiftR = outShiftL = outShiftR = 0;
	inSpectraL = inSpectraR = outSpectraL = outSpectraR = 0;
	synthesisWindow = analysisWindow = 0;

	// clear parameters
	bufferPosition = 0;
	inputTimeL = outputTimeL = 0;
	inputTimeR = outputTimeR = 0;

	// initialize fft size stuff
	overLapFFT = 2;
	sizeFFT = kSizeFFT;
	blockSizeFFT = sizeFFT >> overLapFFT;
	halfSizeFFT = sizeFFT >> 1;
	oneOverBlockSize = 1.0f / (float)blockSizeFFT;

	allocateMemory();
	clearMemory();

	// fftw setup
#if FFT_USED_FFTW
	planIn = rfftw_create_plan(sizeFFT, FFTW_REAL_TO_COMPLEX, FFTW_ESTIMATE);
	planOut = rfftw_create_plan(sizeFFT, FFTW_COMPLEX_TO_REAL, FFTW_ESTIMATE);
#elif FFT_USED_APPLEVECLIB
	log2n = (int32_t)log2f((float)kMaxSizeFFT);
	setupReal = vDSP_create_fftsetup(log2n, FFT_RADIX2);
	log2n = (int32_t)log2f((float)sizeFFT);
#elif FFT_USED_INTEL
	int points, i;
	IppStatus status;
	ippInit();
	for (i = 0, points = 4; i < 11; i++, points++)
	{
		int sizeFFTSpec, sizeFFTInitBuf, sizeFFTWorkBuf;
		status = ippsFFTGetSize_R_32f(points, IPP_FFT_DIV_INV_BY_N, ippAlgHintNone, &sizeFFTSpec, &sizeFFTInitBuf, &sizeFFTWorkBuf);

		// Alloc FFT buffers
		mFFTSpecBuf[i] = ippsMalloc_8u(sizeFFTSpec);
		mFFTInitBuf = ippsMalloc_8u(sizeFFTInitBuf);
		mFFTWorkBuf[i] = ippsMalloc_8u(sizeFFTWorkBuf);

		status = ippsFFTInit_R_32f(&(fftSpec[i]), points, IPP_FFT_DIV_INV_BY_N, ippAlgHintNone, mFFTSpecBuf[i], mFFTInitBuf);
		if (mFFTInitBuf) ippFree(mFFTInitBuf);
	}
#endif
	pi = 4.0f * atanf(1.0f);
	twoPi = 8.0f * atanf(1.0f);

	// make the windows
	initHammingWindows();
	scaleWindows();
	if (numInputs == 2)
		channelMode = kStereoMode;
	else
		channelMode = kMonoMode;
}

//-----------------------------------------------------------------------------------------
void YawnGeneratorAudioProcessor::termspect()
{
	freeMemory();
#if FFT_USED_FFTW
	rfftw_destroy_plan(planIn);
	rfftw_destroy_plan(planOut);
#elif FFT_USED_APPLEVECLIB
	vDSP_destroy_fftsetup(setupReal);
#elif FFT_USED_INTEL
	for (int i = 0; i < 11; i++)
	{
		if (mFFTWorkBuf[i]) ippFree(mFFTWorkBuf[i]);
		if (mFFTSpecBuf[i]) ippFree(mFFTSpecBuf[i]);
	}
#endif
}

bool YawnGeneratorAudioProcessor::allocateMemory()
{
#if FFT_USED_APPLEVECLIB
	split.realp = 0;
	split.realp = (float *)malloc(kHalfMaxSizeFFT * sizeof(float));
	split.imagp = 0;
	split.imagp = (float *)malloc(kHalfMaxSizeFFT * sizeof(float));
#endif
	inBufferL = 0;
	inBufferR = 0;
	inBufferL = (float *)malloc(kMaxSizeFFT * sizeof(float));
	inBufferR = (float *)malloc(kMaxSizeFFT * sizeof(float));
	inFFTL = 0;
	inFFTR = 0;
	inFFTL = (float *)malloc(kMaxSizeFFT * sizeof(float));
	inFFTR = (float *)malloc(kMaxSizeFFT * sizeof(float));
	inShiftL = 0;
	inShiftR = 0;
	inShiftL = (float *)malloc(kMaxSizeFFT * sizeof(float));
	inShiftR = (float *)malloc(kMaxSizeFFT * sizeof(float));
	inSpectraL = 0;
	inSpectraR = 0;
	inSpectraL = (float *)malloc(kMaxSizeFFT * sizeof(float));
	inSpectraR = (float *)malloc(kMaxSizeFFT * sizeof(float));

	outSpectraL = 0;
	outSpectraL = (float *)malloc(kMaxSizeFFT * sizeof(float));
	outSpectraR = 0;
	outSpectraR = (float *)malloc(kMaxSizeFFT * sizeof(float));
	outShiftL = 0;
	outShiftL = (float *)malloc(kMaxSizeFFT * sizeof(float));
	outShiftR = 0;
	outShiftR = (float *)malloc(kMaxSizeFFT * sizeof(float));
	outBufferL = 0;
	outBufferL = (float *)malloc(kMaxSizeFFT * sizeof(float));
	outBufferR = 0;
	outBufferR = (float *)malloc(kMaxSizeFFT * sizeof(float));
	synthesisWindow = 0;
	synthesisWindow = (float *)malloc(kMaxSizeFFT * sizeof(float));
	analysisWindow = 0;
	analysisWindow = (float *)malloc(kMaxSizeFFT * sizeof(float));

	for (int i = 0; i<kMaxSizeFFT; i++)
	{
		inBufferL[i] = inBufferR[i] = outBufferL[i] = outBufferR[i] = 0.0f;
		inShiftL[i] = inShiftR[i] = outShiftL[i] = outShiftR[i] = 0.0f;
		inSpectraL[i] = inSpectraR[i] = 0.0f;
		outSpectraL[i] = outSpectraR[i] = 0.0f;
	}
	return(true);
}

void YawnGeneratorAudioProcessor::freeMemory()
{
	if (inBufferL != 0) { free(inBufferL); inBufferL = 0; }
	if (inBufferR != 0) { free(inBufferR); inBufferR = 0; }
	if (inFFTL != 0) { free(inFFTL); inFFTL = 0; }
	if (inFFTR != 0) { free(inFFTR); inFFTR = 0; }
	if (outBufferL != 0) { free(outBufferL); outBufferL = 0; }
	if (outBufferR != 0) { free(outBufferR); outBufferR = 0; }
	if (inShiftL != 0) { free(inShiftL); inShiftL = 0; }
	if (inShiftR != 0) { free(inShiftR); inShiftR = 0; }
	if (outShiftL != 0) { free(outShiftL); outShiftL = 0; }
	if (outShiftR != 0) { free(outShiftR); outShiftR = 0; }
	if (inSpectraL != 0) { free(inSpectraL); inSpectraL = 0; }
	if (inSpectraR != 0) { free(inSpectraR); inSpectraR = 0; }
	if (outSpectraL != 0) { free(outSpectraL); outSpectraL = 0; }
	if (outSpectraR != 0) { free(outSpectraR); outSpectraR = 0; }
	if (analysisWindow != 0) { free(analysisWindow); analysisWindow = 0; }
	if (synthesisWindow != 0) { free(synthesisWindow); synthesisWindow = 0; }
#if FFT_USED_APPLEVECLIB
	if (split.realp != 0) { free(split.realp); split.realp = 0; }
	if (split.imagp != 0) { free(split.imagp); split.imagp = 0; }
#endif
}

void YawnGeneratorAudioProcessor::clearMemory()
{
	memset(inBufferL, 0, kMaxSizeFFT * sizeof(float));
	memset(inBufferR, 0, kMaxSizeFFT * sizeof(float));
	memset(outBufferL, 0, kMaxSizeFFT * sizeof(float));
	memset(outBufferR, 0, kMaxSizeFFT * sizeof(float));
	memset(inShiftL, 0, kMaxSizeFFT * sizeof(float));
	memset(inShiftR, 0, kMaxSizeFFT * sizeof(float));
	memset(outShiftL, 0, kMaxSizeFFT * sizeof(float));
	memset(outShiftR, 0, kMaxSizeFFT * sizeof(float));
	memset(inSpectraL, 0, kMaxSizeFFT * sizeof(float));
	memset(inSpectraR, 0, kMaxSizeFFT * sizeof(float));
	memset(outSpectraL, 0, kMaxSizeFFT * sizeof(float));
	memset(outSpectraR, 0, kMaxSizeFFT * sizeof(float));
}

void YawnGeneratorAudioProcessor::setFFTSize(int32_t newSize)
{
	log2n = (int32_t)log2f((float)newSize);
	newSize = (int32_t)(powf(2.0f, log2n));
	sizeFFT = newSize;
	blockSizeFFT = sizeFFT >> overLapFFT;
	halfSizeFFT = sizeFFT >> 1;
	oneOverBlockSize = 1.0f / (float)blockSizeFFT;
	bufferPosition = 0;
#if FFT_USED_FFTW
	planIn = rfftw_create_plan(sizeFFT, FFTW_REAL_TO_COMPLEX, FFTW_ESTIMATE);
	planOut = rfftw_create_plan(sizeFFT, FFTW_COMPLEX_TO_REAL, FFTW_ESTIMATE);
#elif FFT_USED_APPLEVECLIB
	//    setupReal = create_fftsetup(log2n, FFT_RADIX2); // no need to do anything if set up for max size
#elif FFT_USED_INTEL
#endif
	clearMemory();
	switch (windowSelected)
	{
	case kHamming:
		initHammingWindows();
		break;
	case kVonHann:
		initVonHannWindows();
		break;
	case kBlackman:
		initBlackmanWindows();
		break;
	case kKaiser:
		initKaiserWindows();
		break;
	}
	scaleWindows();
}
void YawnGeneratorAudioProcessor::processActual(AudioSampleBuffer& buffer)
{
	int32_t i, framesLeft, processframes;
	const float *in1;
	const float *in2;
	float *out1;
	float *out2;

	in1 = buffer.getReadPointer(0);
	out1 = buffer.getWritePointer(0);
	if (buffer.getNumChannels() == 1)
	{
		in2 = in1;
		out2 = out1;
		channelMode = kMonoMode;
	}
	else
	{
		in2 = buffer.getReadPointer(1);
		out2 = buffer.getWritePointer(1);
		channelMode = kStereoMode;
	}

	framesLeft = buffer.getNumSamples();

	while (framesLeft > 0)
	{
		// how many frames can we process now
		// with this we insure that we stop on the
		// blockSizeFFT boundary
		if (framesLeft + bufferPosition < blockSizeFFT)
			processframes = framesLeft;
		else
			processframes = blockSizeFFT - bufferPosition;
		// flush out the previous output, copy in the new input...
		memcpy(inBufferL + bufferPosition, in1, processframes * sizeof(float));
		for (i = 0; i<processframes; i++)
		{
			if (outBufferL[i + bufferPosition] > 1.5f) outBufferL[i + bufferPosition] = 1.5f;
			if (outBufferL[i + bufferPosition] < -1.5f) outBufferL[i + bufferPosition] = -1.5f;

			// copy the old output into the out buffers
			out1[i] = outBufferL[i + bufferPosition];
		}
		if (channelMode == kStereoMode)
		{
			memcpy(inBufferR + bufferPosition, in2, processframes * sizeof(float));
			for (i = 0; i<processframes; i++)
			{
				if (outBufferR[i + bufferPosition] > 1.5f) outBufferR[i + bufferPosition] = 1.5f;
				if (outBufferR[i + bufferPosition] < -1.5f) outBufferR[i + bufferPosition] = -1.5f;
				// copy the old output into the out buffers
				out2[i] = outBufferR[i + bufferPosition];
			}
			in2 += processframes;
			out2 += processframes;
		}

		bufferPosition += processframes;
		// if filled a buffer, we process a new block
		if (bufferPosition >= blockSizeFFT)
		{
			bufferPosition = 0;
			processFFTBlock();
		}
		in1 += processframes;
		out1 += processframes;
		framesLeft -= processframes;
	}
}
// this will generally be overridden to allow FFT size switching from GUI parameter
// called by processFFTBlock
void YawnGeneratorAudioProcessor::updateFFT()
{
	;   // no-op
}

void YawnGeneratorAudioProcessor::processFFTBlock()
{
	int32_t    i, j;
	float    outTemp;
	int32_t    maskFFT;

	updateFFT();
	maskFFT = sizeFFT - 1;
	//
	inputTimeL += blockSizeFFT;
	inputTimeR += blockSizeFFT;
	outputTimeL += blockSizeFFT;
	outputTimeR += blockSizeFFT;
	inputTimeL = inputTimeL & maskFFT;
	inputTimeR = inputTimeR & maskFFT;
	outputTimeL = outputTimeL & maskFFT;
	outputTimeR = outputTimeR & maskFFT;
	//
	// a - shift output buffer and zero out new location
	memcpy(outBufferL, outBufferL + blockSizeFFT, (sizeFFT - blockSizeFFT) * sizeof(float));
	memset(outBufferL + (sizeFFT - blockSizeFFT), 0, blockSizeFFT * sizeof(float));
	if (channelMode == kStereoMode)
	{
		// a - shift output buffer and zero out new location
		memcpy(outBufferR, outBufferR + blockSizeFFT, (sizeFFT - blockSizeFFT) * sizeof(float));
		memset(outBufferR + (sizeFFT - blockSizeFFT), 0, blockSizeFFT * sizeof(float));
	}
	// a - shift current samples in inShift
	memcpy(inShiftL, inShiftL + blockSizeFFT, (sizeFFT - blockSizeFFT) * sizeof(float));
	// a1 - copy the new stuff in
	memcpy(inShiftL + (sizeFFT - blockSizeFFT), inBufferL, blockSizeFFT * sizeof(float));
	// b - window the block in
	for (i = 0; i < sizeFFT; i++)
	{
		*(inFFTL + inputTimeL) = *(inShiftL + i) * *(analysisWindow + i);
		++inputTimeL;
		inputTimeL = inputTimeL & maskFFT;
	}
#if FFT_USED_FFTW
	rfftw_one(planIn, inFFTL, inSpectraL);
#elif FFT_USED_APPLEVECLIB
	vDSP_ctoz((COMPLEX *)inFFTL, 2, &split, 1, halfSizeFFT);
	vDSP_fft_zrip(setupReal, &split, 1, log2n, FFT_FORWARD);
	memcpy(inSpectraL, split.realp, halfSizeFFT * sizeof(float));
	*(inSpectraL + halfSizeFFT) = *(split.imagp);
	for (i = halfSizeFFT + 1, j = halfSizeFFT - 1; i < sizeFFT; i++, j--)
		*(inSpectraL + i) = *(split.imagp + j);
#elif FFT_USED_INTEL
	ippsFFTFwd_RToPerm_32f_I(inFFTL, fftSpec[log2n - 4], mFFTWorkBuf[log2n - 4]);
	*(inSpectraL + 0) = *(inFFTL + 0);
	*(inSpectraL + halfSizeFFT) = *(inFFTL + 1);
	for (i = 1, j = 2; j < sizeFFT; j += 2, i++)
		*(inSpectraL + i) = *(inFFTL + j);
	for (i = halfSizeFFT + 1, j = sizeFFT - 1; i < sizeFFT; i++, j -= 2)
		*(inSpectraL + i) = *(inFFTL + j);
#endif
	if (channelMode == kStereoMode)
	{
		// a - shift current samples in inShift
		memcpy(inShiftR, inShiftR + blockSizeFFT, (sizeFFT - blockSizeFFT) * sizeof(float));
		// a1 - copy the new stuff in
		memcpy(inShiftR + (sizeFFT - blockSizeFFT), inBufferR, blockSizeFFT * sizeof(float));
		// b - window the block in
		for (i = 0; i < sizeFFT; i++)
		{
			*(inFFTR + inputTimeR) = *(inShiftR + i) * *(analysisWindow + i);
			++inputTimeR;
			inputTimeR = inputTimeR & maskFFT;
		}
#if FFT_USED_FFTW
		rfftw_one(planIn, inFFTR, inSpectraR);
#elif FFT_USED_APPLEVECLIB
		vDSP_ctoz((COMPLEX *)inFFTR, 2, &split, 1, halfSizeFFT);
		vDSP_fft_zrip(setupReal, &split, 1, log2n, FFT_FORWARD);
		memcpy(inSpectraR, split.realp, halfSizeFFT * sizeof(float));
		*(inSpectraR + halfSizeFFT) = *(split.imagp);
		for (i = halfSizeFFT + 1, j = halfSizeFFT - 1; i < sizeFFT; i++, j--)
			*(inSpectraR + i) = *(split.imagp + j);
#elif FFT_USED_INTEL
		ippsFFTFwd_RToPerm_32f_I(inFFTR, fftSpec[log2n - 4], mFFTWorkBuf[log2n - 4]);
		*(inSpectraR + 0) = *(inFFTR + 0);
		*(inSpectraR + halfSizeFFT) = *(inFFTR + 1);
		for (i = 1, j = 2; j < sizeFFT; j += 2, i++)
			*(inSpectraR + i) = *(inFFTR + j);
		for (i = halfSizeFFT + 1, j = sizeFFT - 1; i < sizeFFT; i++, j -= 2)
			*(inSpectraR + i) = *(inFFTR + j);
#endif
	}

	processSpect();

	// d - IFFT
#if FFT_USED_FFTW
	rfftw_one(planOut, outSpectraL, outShiftL);
#elif FFT_USED_APPLEVECLIB
	memcpy(split.realp, outSpectraL, halfSizeFFT * sizeof(float));
	*(split.imagp) = *(outSpectraL + halfSizeFFT);
	for (i = halfSizeFFT + 1, j = halfSizeFFT - 1; i < sizeFFT; i++, j--)
		*(split.imagp + j) = *(outSpectraL + i);
	vDSP_fft_zrip(setupReal, &split, 1, log2n, FFT_INVERSE);
	vDSP_ztoc(&split, 1, (COMPLEX *)outShiftL, 2, halfSizeFFT);
#elif FFT_USED_INTEL
	*(outShiftL + 0) = *(outSpectraL + 0);
	*(outShiftL + 1) = *(outSpectraL + halfSizeFFT);
	for (i = 1, j = 2; j < sizeFFT; j += 2, i++)
		*(outShiftL + j) = *(outSpectraL + i);
	for (i = halfSizeFFT + 1, j = sizeFFT - 1; i < sizeFFT; i++, j -= 2)
		*(outShiftL + j) = *(outSpectraL + i);
	ippsFFTInv_PermToR_32f_I(outShiftL, fftSpec[log2n - 4], mFFTWorkBuf[log2n - 4]);
#endif
	// e - overlap add
	for (i = 0; i < sizeFFT; i++)
	{
		if ((*(outShiftL + outputTimeL) == *(outShiftL + outputTimeL)) != 1)
			*(outShiftL + outputTimeL) = 0.0f;
		outTemp = *(outShiftL + outputTimeL) * *(synthesisWindow + i);
		*(outBufferL + i) += outTemp;
		++outputTimeL;
		outputTimeL = outputTimeL & maskFFT;
	}
	if (channelMode == kStereoMode)
	{
		// d - IFFT
#if FFT_USED_FFTW
		rfftw_one(planOut, outSpectraR, outShiftR);
#elif FFT_USED_APPLEVECLIB
		memcpy(split.realp, outSpectraR, halfSizeFFT * sizeof(float));
		*(split.imagp) = *(outSpectraR + halfSizeFFT);
		for (i = halfSizeFFT + 1, j = halfSizeFFT - 1; i < sizeFFT; i++, j--)
			*(split.imagp + j) = *(outSpectraR + i);
		vDSP_fft_zrip(setupReal, &split, 1, log2n, FFT_INVERSE);
		vDSP_ztoc(&split, 1, (COMPLEX *)outShiftR, 2, halfSizeFFT);
#elif FFT_USED_INTEL
		*(outShiftR + 0) = *(outSpectraR + 0);
		*(outShiftR + 1) = *(outSpectraR + halfSizeFFT);
		for (i = 1, j = 2; j < sizeFFT; j += 2, i++)
			*(outShiftR + j) = *(outSpectraR + i);
		for (i = halfSizeFFT + 1, j = sizeFFT - 1; i < sizeFFT; i++, j -= 2)
			*(outShiftR + j) = *(outSpectraR + i);
		ippsFFTInv_PermToR_32f_I(outShiftR, fftSpec[log2n - 4], mFFTWorkBuf[log2n - 4]);
#endif
		// e - overlap add
		for (i = 0; i < sizeFFT; i++)
		{
			if ((*(outShiftR + outputTimeR) == *(outShiftR + outputTimeR)) != 1)
				*(outShiftR + outputTimeR) = 0.0f;
			outTemp = *(outShiftR + outputTimeR) * *(synthesisWindow + i);
			*(outBufferR + i) += outTemp;
			++outputTimeR;
			outputTimeR = outputTimeR & maskFFT;
		}
	}
}

void    YawnGeneratorAudioProcessor::processSpect()
{
	int i;

	// c - copy the spectra over - this is where we would typically do something
	outSpectraL[0] = inSpectraL[0];    // DC Component
	outSpectraL[halfSizeFFT] = inSpectraL[halfSizeFFT];    // Nyquist Frequency
	for (i = 1; i < 100; ++i)
	{
		outSpectraL[i] = inSpectraL[i];
		outSpectraL[sizeFFT - i] = inSpectraL[sizeFFT - i];
	}
	for (; i < halfSizeFFT; ++i)
	{
		outSpectraL[i] = 0.0f;
		outSpectraL[sizeFFT - i] = 0.0f;
	}

	outSpectraR[0] = inSpectraR[0];    // DC Component
	outSpectraR[halfSizeFFT] = inSpectraR[halfSizeFFT];    // Nyquist Frequency
	for (i = 1; i < 100; ++i)
	{
		outSpectraR[i] = inSpectraR[i];
		outSpectraR[sizeFFT - i] = inSpectraR[sizeFFT - i];
	}
	for (; i < halfSizeFFT; ++i)
	{
		outSpectraR[i] = 0.0f;
		outSpectraR[sizeFFT - i] = 0.0f;
	}

}
void    YawnGeneratorAudioProcessor::initHammingWindows(void)
{
	int32_t    index;
	float    a = 0.54f, b = 0.46f;

	windowSelected = kHamming;

	// a - make two hamming windows
	for (index = 0; index < sizeFFT; index++)
		synthesisWindow[index] = analysisWindow[index] = a - b * cosf(twoPi*index / (sizeFFT - 1));
}

void    YawnGeneratorAudioProcessor::initVonHannWindows(void)
{
	int32_t    index;
	float    a = 0.50, b = 0.40;

	windowSelected = kVonHann;

	// a - make two hamming windows
	for (index = 0; index < sizeFFT; index++)
		synthesisWindow[index] = analysisWindow[index] = a - b * cosf(twoPi*index / (sizeFFT - 1));
}

void    YawnGeneratorAudioProcessor::initBlackmanWindows(void)
{
	int32_t    index;
	float    a = 0.42, b = 0.50, c = 0.08;

	windowSelected = kBlackman;

	// a - make two hamming windows
	for (index = 0; index < sizeFFT; index++)
		synthesisWindow[index] = analysisWindow[index] = a - b * cosf(twoPi*index / (sizeFFT - 1)) + c * cosf(2.0f*twoPi*index / (sizeFFT - 1));

}

void    YawnGeneratorAudioProcessor::initKaiserWindows(void)
{

	double    bes, xind, floati;
	int32_t    i;

	windowSelected = kKaiser;

	bes = kaiserIno(6.8);
	xind = (float)(sizeFFT - 1)*(sizeFFT - 1);

	for (i = 0; i < halfSizeFFT; i++)
	{
		floati = (double)i;
		floati = 4.0 * floati * floati;
		floati = sqrt(1. - floati / xind);
		synthesisWindow[i + halfSizeFFT] = kaiserIno(6.8 * floati);
		analysisWindow[halfSizeFFT - i] = synthesisWindow[halfSizeFFT - i] = analysisWindow[i + halfSizeFFT] = (synthesisWindow[i + halfSizeFFT] /= bes);
	}
	analysisWindow[sizeFFT - 1] = synthesisWindow[sizeFFT - 1] = 0.0;
	analysisWindow[0] = synthesisWindow[0] = 0.0;
}

float    YawnGeneratorAudioProcessor::kaiserIno(float x)
{
	float    y, t, e, de, sde, xi;
	int32_t i;

	y = x / 2.;
	t = 1.e-08;
	e = 1.;
	de = 1.;
	for (i = 1; i <= 25; i++)
	{
		xi = i;
		de = de * y / xi;
		sde = de * de;
		e += sde;
		if (e * t > sde)
			break;
	}
	return(e);
}

void    YawnGeneratorAudioProcessor::scaleWindows(void)
{
	int32_t    index;
	float    sum, analFactor, synthFactor;

	// b - scale the windows
	sum = 0.0f;
	for (index = 0; index < sizeFFT; index++)
		sum += analysisWindow[index];

	synthFactor = analFactor = 2.0f / sum;

	for (index = 0; index < sizeFFT; index++)
	{
		analysisWindow[index] *= analFactor;
		synthesisWindow[index] *= synthFactor;
	}
	sum = 0.0;
	for (index = 0; index < sizeFFT; index += blockSizeFFT)
		sum += synthesisWindow[index] * synthesisWindow[index];
	// i think this scaling factor is only needed for vector lib
#if FFT_USED_FFTW
	sum = 1.0f / (sum*sizeFFT);
#elif FFT_USED_APPLEVECLIB
	sum = 1.0f / (sum*sizeFFT*2.0f);
#elif FFT_USED_INTEL
	sum = 1.0f / sum;
#endif
	for (index = 0; index < sizeFFT; index++)
		synthesisWindow[index] *= sum;
}


void    YawnGeneratorAudioProcessor::cartToPolar(float *spectrum, float *polarSpectrum)
{
	int32_t    realIndex, imagIndex, ampIndex, phaseIndex;
	float    realPart, imagPart;
	int32_t    bandNumber;


	for (bandNumber = 0; bandNumber <= halfSizeFFT; bandNumber++)
	{
		realIndex = bandNumber;
		imagIndex = sizeFFT - bandNumber;
		ampIndex = bandNumber << 1;
		phaseIndex = ampIndex + 1;
		if (bandNumber == 0)
		{
			realPart = spectrum[0];
			imagPart = 0.0;
		}
		else if (bandNumber == halfSizeFFT)
		{
			realPart = spectrum[halfSizeFFT];
			imagPart = 0.0;
		}
		else
		{
			realPart = spectrum[realIndex];
			imagPart = spectrum[imagIndex];
		}
		/*
		* compute magnitude & phase value from real and imaginary parts
		*/

		polarSpectrum[ampIndex] = hypot(realPart, imagPart);
		if (polarSpectrum[ampIndex] < 0.0f)
			polarSpectrum[phaseIndex] = 0.0;
		else
			polarSpectrum[phaseIndex] = -atan2f(imagPart, realPart);
	}
}


void    YawnGeneratorAudioProcessor::polarToCart(float *polarSpectrum, float *spectrum)
{
	float    realValue, imagValue;
	int32_t    bandNumber, realIndex, imagIndex, ampIndex, phaseIndex;

	/*
	* convert to cartesian coordinates, complex pairs
	*/
	for (bandNumber = 0; bandNumber <= halfSizeFFT; bandNumber++)
	{
		realIndex = bandNumber;
		imagIndex = sizeFFT - bandNumber;
		ampIndex = bandNumber << 1;
		phaseIndex = ampIndex + 1;
		if (polarSpectrum[ampIndex] == 0.0)
		{
			realValue = 0.0;
			imagValue = 0.0;
		}
		else if (bandNumber == 0 || bandNumber == halfSizeFFT)
		{
			realValue = polarSpectrum[ampIndex] * cosf(polarSpectrum[phaseIndex]);
			imagValue = 0.0;
		}
		else
		{
			realValue = polarSpectrum[ampIndex] * cosf(polarSpectrum[phaseIndex]);
			imagValue = polarSpectrum[ampIndex] * -sinf(polarSpectrum[phaseIndex]);
		}


		if (bandNumber == halfSizeFFT)
			realIndex = halfSizeFFT;
		spectrum[realIndex] = realValue;
		if (bandNumber != halfSizeFFT && bandNumber != 0)
			spectrum[imagIndex] = imagValue;
	}
}
