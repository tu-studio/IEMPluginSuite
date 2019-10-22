/*
 ==============================================================================
 This file is part of the IEM plug-in suite.
 Author: Daniel Rudrich
 Copyright (c) 2017 - Institute of Electronic Music and Acoustics (IEM)
 https://iem.at

 The IEM plug-in suite is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 The IEM plug-in suite is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this software.  If not, see <https://www.gnu.org/licenses/>.
 ==============================================================================
 */

#pragma once

#include "../JuceLibraryCode/JuceHeader.h"
#include "PluginProcessor.h"

//Plugin Design Essentials
#include "../../resources/lookAndFeel/IEM_LaF.h"
#include "../../resources/customComponents/TitleBar.h"

//Custom Components
#include "../../resources/customComponents/ReverseSlider.h"
#include "../../resources/customComponents/SimpleLabel.h"
#include "../../resources/customComponents/FilterVisualizer.h"
#include "../../resources/customComponents/DecoderInfoBox.h"


typedef ReverseSlider::SliderAttachment SliderAttachment; // all ReverseSliders will make use of the parameters' valueToText() function
typedef AudioProcessorValueTreeState::ComboBoxAttachment ComboBoxAttachment;
typedef AudioProcessorValueTreeState::ButtonAttachment ButtonAttachment;

//==============================================================================
/**
*/
class SimpleDecoderAudioProcessorEditor  : public AudioProcessorEditor, private Timer, public AudioProcessorValueTreeState::Listener
{
public:
    SimpleDecoderAudioProcessorEditor (SimpleDecoderAudioProcessor&, AudioProcessorValueTreeState&);
    ~SimpleDecoderAudioProcessorEditor();

    //==============================================================================
    void paint (Graphics&) override;
    void resized() override;


    void timerCallback() override;
    void loadPresetFile();
    void parameterChanged (const String &parameterID, float newValue) override;
private:
    // ====================== beging essentials ==================
    // lookAndFeel class with the IEM plug-in suite design
    LaF globalLaF;

    // stored references to the AudioProcessor and ValueTreeState holding all the parameters
    SimpleDecoderAudioProcessor& processor;
    AudioProcessorValueTreeState& valueTreeState;


    /* title and footer component
     title component can hold different widgets for in- and output:
        - NoIOWidget (if there's no need for an input or output widget)
        - AudioChannelsIOWidget<maxNumberOfChannels, isChoosable>
        - AmbisonicIOWidget
        - DirectivitiyIOWidget
     */

    TitleBar<AmbisonicIOWidget<>, AudioChannelsIOWidget<0,false>> title;
    OSCFooter footer;
    // =============== end essentials ============

    // Attachments to create a connection between IOWidgets comboboxes
    // and the associated parameters
    std::unique_ptr<ComboBoxAttachment> cbOrderSettingAttachment;
    std::unique_ptr<ComboBoxAttachment> cbNormalizationSettingAttachment;
    //std::unique_ptr<ComboBoxAttachment> cbOutputChannelsSettingAttachment;

    bool updateChannelsInWidget = false;
    bool enableSubwooferChannelControls;
    bool changeEnablement = false;

    GroupComponent gcFilter, gcSw, gcConfiguration, gcWeights;

    // Filter slider
    ReverseSlider slLowPassFrequency, slHighPassFrequency, slLowPassGain;
    std::unique_ptr<SliderAttachment> slLowPassFrequencyAttachment, slLowPassGainAttachment, slHighPassFrequencyAttachment;
    SimpleLabel lbLowPassFrequency, lbLowPassGain, lbHighPassFrequency;

    // Subwoofer mode
    ComboBox cbSwMode;
    std::unique_ptr<ComboBoxAttachment> cbSwModeAttachment;
    SimpleLabel lbSwMode, lbSwChannel, lbAlreadyUsed;
    ReverseSlider slSwChannel;
    std::unique_ptr<SliderAttachment> slSwChannelAttachment;
    //
    TextButton btLoadFile;
    DecoderInfoBox dcInfoBox;

    // weights
    ComboBox cbWeights;
    std::unique_ptr<ComboBoxAttachment> cbWeightsAttachment;


    ReferenceCountedDecoder::Ptr lastDecoder = nullptr;

    FilterVisualizer<double> fv;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (SimpleDecoderAudioProcessorEditor)
};
