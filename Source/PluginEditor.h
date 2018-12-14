/*
  ==============================================================================

    This file was auto-generated!

    It contains the basic framework code for a JUCE plugin editor.

  ==============================================================================
*/

#pragma once

#include "../JuceLibraryCode/JuceHeader.h"
#include "PluginProcessor.h"

//==============================================================================
/**
*/
class YawnGeneratorAudioProcessorEditor  : public AudioProcessorEditor, public Slider::Listener
{
public:
    YawnGeneratorAudioProcessorEditor (YawnGeneratorAudioProcessor&);
    ~YawnGeneratorAudioProcessorEditor();

    //==============================================================================
    void paint (Graphics&) override;
    void resized() override;
	void sliderValueChanged(Slider* sliderThatWasMoved) override;

private:
    // This reference is provided as a quick way for your editor to
    // access the processor object that created it.
    YawnGeneratorAudioProcessor& processor;
	Slider speedSlider;
	Slider yawnAirSlider;
	ScopedPointer<AudioProcessorValueTreeState::SliderAttachment> speedSliderAttachment;
	ScopedPointer<AudioProcessorValueTreeState::SliderAttachment> yawnAirSliderAttachment;
	Image yawn;
	ImageComponent imageComponent;
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (YawnGeneratorAudioProcessorEditor)

};
