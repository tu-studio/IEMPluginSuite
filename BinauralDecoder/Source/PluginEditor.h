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


typedef ReverseSlider::SliderAttachment SliderAttachment; // all ReverseSliders will make use of the parameters' valueToText() function
typedef AudioProcessorValueTreeState::ComboBoxAttachment ComboBoxAttachment;
typedef AudioProcessorValueTreeState::ButtonAttachment ButtonAttachment;

//==============================================================================
/**
*/
class BinauralDecoderAudioProcessorEditor  : public AudioProcessorEditor, private Timer
{
public:
    BinauralDecoderAudioProcessorEditor (BinauralDecoderAudioProcessor&, AudioProcessorValueTreeState&);
    ~BinauralDecoderAudioProcessorEditor();

    //==============================================================================
    void paint (Graphics&) override;
    void resized() override;
    
    
    void timerCallback() override;
    
private:
    // ====================== begin essentials ==================
    // lookAndFeel class with the IEM plug-in suite design
    LaF globalLaF;
    
    // stored references to the AudioProcessor and ValueTreeState holding all the parameters
    BinauralDecoderAudioProcessor& processor;
    AudioProcessorValueTreeState& valueTreeState;

    
    /* title and footer component
     title component can hold different widgets for in- and output:
        - NoIOWidget (if there's no need for an input or output widget)
        - AudioChannelsIOWidget<maxNumberOfChannels, isChoosable>
        - AmbisonicIOWidget<maxOrder>
        - DirectivitiyIOWidget
     */
    TitleBar<AmbisonicIOWidget<>, BinauralIOWidget> title;
    Footer footer;
    // =============== end essentials ============
    
    // Attachments to create a connection between IOWidgets comboboxes
    // and the associated parameters
    ScopedPointer<ComboBoxAttachment> cbOrderSettingAttachment;
    ScopedPointer<ComboBoxAttachment> cbNormalizationSettingAttachment;
    
    SimpleLabel lbEq;
    ComboBox cbEq;
    ScopedPointer<ComboBoxAttachment> cbEqAttachment;
    
    

    
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (BinauralDecoderAudioProcessorEditor)
};