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
}

YawnGeneratorAudioProcessor::~YawnGeneratorAudioProcessor()
{
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
		pan(tempR[sample] * gain*0.8, left, right, sample);
		//left[sample] = tempR[sample]*gain*0.8;
		//right[sample] = left[sample];
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
		}
		else if (t < 2) {
			output = linearInterpolateTableValues(7)*expRamp(t, 1.75, 2.0, false);
		}
		double noise = getNoise(t);
		output += noise;
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

void YawnGeneratorAudioProcessor::pan(double input, float* left, float* right, int sample) {
	double leftFunction = exp(-t);
	double rightFunction = log10(3 * t) + 0.2;
	if (t >= 2) {
		leftFunction == 0;
		rightFunction = 1;
	}
	left[sample] = leftFunction * input;
	right[sample] = rightFunction * input;
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
	int harmonics[] = { 1,2,3,4,5,6,7,8,9,10,11,12,13,14 };

	for (int i = 0; i < numElementsInArray(wavetablesA); i++)
	{
		if (i == 7) {
			bool x = true;
		}
		wavetablesA[i].setSize(1, TABLE_SIZE + 1);
		wavetablesA[i].clear();

		auto* samples = wavetablesA[i].getWritePointer(0);

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

}

void YawnGeneratorAudioProcessor::updateFilter(double cutoff) {
	//From Pirckle LPF
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





