/*
  ==============================================================================

    This file was auto-generated!

    It contains the basic framework code for a JUCE plugin editor.

  ==============================================================================
*/

#include "PluginProcessor.h"
#include "PluginEditor.h"

//==============================================================================
YawnGeneratorAudioProcessorEditor::YawnGeneratorAudioProcessorEditor (YawnGeneratorAudioProcessor& p)
    : AudioProcessorEditor (&p), processor (p)
{
    // Make sure that before the constructor has finished, you've set the
    // editor's size to whatever you need it to be.
    setSize (390, 150);
	getLookAndFeel().setColour(Slider::textBoxTextColourId, Colours::black);
	getLookAndFeel().setColour(Slider::rotarySliderFillColourId, Colours::red);
	getLookAndFeel().setColour(Slider::rotarySliderOutlineColourId, Colour(255, 133, 133));

	speedSlider.setSliderStyle(Slider::RotaryHorizontalVerticalDrag);
	speedSlider.setTextBoxStyle(Slider::TextBoxBelow, false, 100, 20);
	speedSlider.addListener(this);
	speedSliderAttachment = new AudioProcessorValueTreeState::SliderAttachment(processor.parameters, "speed", speedSlider);
	addAndMakeVisible(&speedSlider);

	yawnAirSlider.setSliderStyle(Slider::RotaryHorizontalVerticalDrag);
	yawnAirSlider.setTextBoxStyle(Slider::TextBoxBelow, false, 100, 20);
	yawnAirSlider.addListener(this);
	yawnAirSliderAttachment = new AudioProcessorValueTreeState::SliderAttachment(processor.parameters, "yawnAir", yawnAirSlider);
	addAndMakeVisible(&yawnAirSlider);

	//yawn = ImageFileFormat::loadFrom(File::File("C:\\Users\\matth\\JUCE\\YawnGenerator\\Assets\\yawn.jpg"));
	//if (yawn.isValid()) {
	//	imageComponent.setImage(yawn);
	//	addAndMakeVisible(&imageComponent);
	//}

}

YawnGeneratorAudioProcessorEditor::~YawnGeneratorAudioProcessorEditor()
{
	speedSliderAttachment = nullptr;
	yawnAirSliderAttachment = nullptr;

}

//==============================================================================
void YawnGeneratorAudioProcessorEditor::paint (Graphics& g)
{
    // (Our component is opaque, so we must completely fill the background with a solid colour)
    //g.fillAll (getLookAndFeel().findColour (ResizableWindow::backgroundColourId));
	g.fillAll(Colours::lightyellow);
	//getLookAndFeel().setColour(Slider::ColourIds::textBoxTextColourId, Colours::black);

    g.setColour (Colours::black);
    g.setFont (15.0f);
	g.drawText(TRANS("Speed"), 10, 126, 100, 20, Justification::centred);
	g.drawText(TRANS("Air"),130, 126, 100, 20, Justification::centred);

}

void YawnGeneratorAudioProcessorEditor::resized()
{
    // This is generally where you'll want to lay out the positions of any
	// subcomponents in your editor..
	speedSlider.setBounds(10, 0, 100, 120);
	yawnAirSlider.setBounds(130, 0, 100, 120);
	imageComponent.setBounds(225, 0, 225, 150);
}

void YawnGeneratorAudioProcessorEditor::sliderValueChanged(Slider* sliderThatWasMoved)
{
	if (sliderThatWasMoved == &speedSlider) {
		//processor.setFrequency(sliderThatWasMoved->getValue());
		processor.speed = sliderThatWasMoved->getValue() /4;
	}
	if (sliderThatWasMoved == &yawnAirSlider) {
		
		//processor.updateFilter(sliderThatWasMoved->getValue());
	}
}
