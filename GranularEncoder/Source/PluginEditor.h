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

#include "../JuceLibraryCode/JuceHeader.h"
#include "PluginProcessor.h"

#include "../../resources/customComponents/ReverseSlider.h"
#include "../../resources/lookAndFeel/IEM_LaF.h"
#include "../../resources/customComponents/TitleBar.h"
#include "../../resources/customComponents/SpherePanner.h"
#include "../../resources/customComponents/SimpleLabel.h"

typedef ReverseSlider::SliderAttachment SliderAttachment;
typedef juce::AudioProcessorValueTreeState::ComboBoxAttachment ComboBoxAttachment;
typedef juce::AudioProcessorValueTreeState::ButtonAttachment ButtonAttachment;

//==============================================================================
/**
 */
class StereoEncoderAudioProcessorEditor : public juce::AudioProcessorEditor,
                                          private juce::Timer,
                                          public SpherePanner::Listener,
                                          private juce::KeyListener
{
public:
    StereoEncoderAudioProcessorEditor(StereoEncoderAudioProcessor &, juce::AudioProcessorValueTreeState &);
    ~StereoEncoderAudioProcessorEditor();

    //==============================================================================
    void paint(juce::Graphics &) override;
    void resized() override;

    void mouseWheelOnSpherePannerMoved(SpherePanner *sphere, const juce::MouseEvent &event, const juce::MouseWheelDetails &wheel) override;

    bool keyPressed(const juce::KeyPress &key, juce::Component *originatingComponent) override;

private:
    LaF globalLaF;

    TitleBar<AudioChannelsIOWidget<2, false>, AmbisonicIOWidget<>> title;
    OSCFooter footer;

    void timerCallback() override;

    // This reference is provided as a quick way for your editor to
    // access the processor object that created it.
    StereoEncoderAudioProcessor &processor;
    juce::AudioProcessorValueTreeState &valueTreeState;

    juce::GroupComponent quatGroup, ypGroup, grainGroup, settingsGroup;
    ReverseSlider azimuthSlider, elevationSlider, shapeSlider, sizeSlider, qwSlider, qxSlider, qySlider, qzSlider;
    ReverseSlider rollSlider, widthSlider;
    ReverseSlider deltaTimeSlider, deltaTimeModSlider, grainLengthSlider, grainLengthModSlider;
    ReverseSlider positionSlider, positionModSlider, pitchSlider, pitchModSlider;
    juce::ComboBox inputChooser;

    SpherePanner sphere;
    SpherePanner::AzimuthElevationParameterElement centerElement;
    SpherePanner::RollWidthParameterElement leftElement;
    SpherePanner::RollWidthParameterElement rightElement;

    std::unique_ptr<SliderAttachment> qwAttachment;
    std::unique_ptr<SliderAttachment> qxAttachment;
    std::unique_ptr<SliderAttachment> qyAttachment;
    std::unique_ptr<SliderAttachment> qzAttachment;
    std::unique_ptr<SliderAttachment> azimuthAttachment;
    std::unique_ptr<SliderAttachment> elevationAttachment;

    std::unique_ptr<SliderAttachment> shapeAttachment;
    std::unique_ptr<SliderAttachment> sizeAttachment;

    std::unique_ptr<SliderAttachment> rollAttachment;
    std::unique_ptr<SliderAttachment> widthAttachment;

    std::unique_ptr<SliderAttachment> deltaTimeAttachment;
    std::unique_ptr<SliderAttachment> deltaTimeModAttachment;

    std::unique_ptr<SliderAttachment> grainLengthAttachment;
    std::unique_ptr<SliderAttachment> grainLengthModAttachment;

    std::unique_ptr<SliderAttachment> positionAttachment;
    std::unique_ptr<SliderAttachment> positionModAttachment;

    std::unique_ptr<SliderAttachment> pitchAttachment;
    std::unique_ptr<SliderAttachment> pitchModAttachment;

    std::unique_ptr<ComboBoxAttachment> cbNormalizationAtachement;
    std::unique_ptr<ComboBoxAttachment> cbOrderAtachement;

    juce::TooltipWindow toolTipWin;

    // labels
    SimpleLabel lbAzimuth, lbElevation, lbShape, lbSize, lbW, lbX, lbY, lbZ;
    SimpleLabel lbRoll, lblWidth; // legacy
    SimpleLabel lbDeltaTime, lbDeltaTimeMod, lbGrainLength, lbGrainLengthMod;
    SimpleLabel lbPosition, lbPositionMod, lbPitch, lbPitchMod;

    juce::ToggleButton tbFreeze;
    SimpleLabel lbFreeze;
    std::unique_ptr<ButtonAttachment> tbFreezeAttachment;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(StereoEncoderAudioProcessorEditor)
};
