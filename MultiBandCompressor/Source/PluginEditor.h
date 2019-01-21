/*
 ==============================================================================
 This file is part of the IEM plug-in suite.
 Author: Markus Huber
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
#include <unordered_map>

//Plugin Design Essentials
#include "../../resources/lookAndFeel/IEM_LaF.h"
#include "../../resources/customComponents/TitleBar.h"

//Custom Components
#include "../../resources/customComponents/ReverseSlider.h"
#include "../../resources/customComponents/SimpleLabel.h"
#include "../../resources/customComponents/FilterVisualizer.h"
#include "../../resources/customComponents/CompressorVisualizer.h"
#include "../../resources/customComponents/LevelMeter.h"
#include "../../resources/customComponents/OnOffButton.h"



typedef ReverseSlider::SliderAttachment SliderAttachment; // all ReverseSliders will make use of the parameters' valueToText() function
typedef AudioProcessorValueTreeState::ComboBoxAttachment ComboBoxAttachment;
typedef AudioProcessorValueTreeState::ButtonAttachment ButtonAttachment;

extern const int numFilterBands;

//==============================================================================
/**
*/
class MultiBandCompressorAudioProcessorEditor  : public AudioProcessorEditor, private Timer, public Slider::Listener
{
public:
    MultiBandCompressorAudioProcessorEditor (MultiBandCompressorAudioProcessor&, AudioProcessorValueTreeState&);
    ~MultiBandCompressorAudioProcessorEditor();

    //==============================================================================
    void paint (Graphics&) override;
    void resized() override;
  
    void sliderValueChanged(Slider *slider) override;

    void timerCallback() override;

private:
    // ====================== begin essentials ==================
    // lookAndFeel class with the IEM plug-in suite design
    LaF globalLaF;

    // stored references to the AudioProcessor and ValueTreeState holding all the parameters
    MultiBandCompressorAudioProcessor& processor;
    AudioProcessorValueTreeState& valueTreeState;
  
    void updateFilterVisualizer();
    void drawCompressorArea();

    enum class FilterIndex
    {
        LowIndex, MidIndex, HighIndex
    };
    /* title and footer component
     title component can hold different widgets for in- and output:
        - NoIOWidget (if there's no need for an input or output widget)
        - AudioChannelsIOWidget<maxNumberOfChannels, isChoosable>
        - AmbisonicIOWidget<maxOrder>
        - DirectivitiyIOWidget
     */
    TitleBar<AudioChannelsIOWidget<64, true>, NoIOWidget> title;
    OSCFooter footer;
    // =============== end essentials ============
  
    std::unique_ptr<ComboBoxAttachment> cbInputChannelsSettingAttachment;
  
    FilterVisualizer<double> filterVisualizer;
    TooltipWindow tooltips;
  
    // Filter Cutoffs
    ReverseSlider slFilterFrequency[numFilterBands-1];
    std::unique_ptr<SliderAttachment> slFilterFrequencyAttachment[numFilterBands-1];
  
    // Solo and Bypass Buttons
    TextButton tbSoloEnabled[numFilterBands], tbBypass[numFilterBands];
    std::unique_ptr<AudioProcessorValueTreeState::ButtonAttachment> soloAttachment[numFilterBands], bypassAttachment[numFilterBands];
  
    // Compressor Parameters
    ReverseSlider slKnee[numFilterBands], slThreshold[numFilterBands], slRatio[numFilterBands], slAttackTime[numFilterBands], slReleaseTime[numFilterBands], slMakeUpGain[numFilterBands];
    std::unique_ptr<SliderAttachment> slKneeAttachment[numFilterBands], slThresholdAttachment[numFilterBands], slRatioAttachment[numFilterBands], slAttackTimeAttachment[numFilterBands], slReleaseTimeAttachment[numFilterBands], slMakeUpGainAttachment[numFilterBands];

    // Master parameters
    GroupComponent gcMasterControls;
    ReverseSlider slMasterThreshold, slMasterMakeUpGain, slMasterKnee, slMasterRatio, slMasterAttackTime, slMasterReleaseTime;
    std::unordered_map<String, double> masterSliderPrevValueMap;

    // Compressor Visualization
    OwnedArray<CompressorVisualizer> compressorVisualizers;
  
    // Meters
    LevelMeter omniFilteredMeter[numFilterBands], GRmeter[numFilterBands], omniInputMeter, omniOutputMeter;
  
    // Labels
    SimpleLabel lbKnee[numFilterBands+1], lbThreshold[numFilterBands+1], lbMakeUpGain[numFilterBands+1], lbRatio[numFilterBands+1], lbAttack[numFilterBands+1], lbRelease[numFilterBands+1], lbInput, lbOutput;
  
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (MultiBandCompressorAudioProcessorEditor)
};
