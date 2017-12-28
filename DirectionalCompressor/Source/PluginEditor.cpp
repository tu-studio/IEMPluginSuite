/*
 ==============================================================================
 This file is part of the IEM plug-in suite.
 Author: Daniel Rudrich
 Copyright (c) 2017 - Institute of Electronic Music and Acoustics (IEM)
 http://www.iem.at
 
 The IEM plug-in suite is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 The IEM plug-in suite is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this software.  If not, see <http://www.gnu.org/licenses/>.
 ==============================================================================
 */

#include "PluginProcessor.h"
#include "PluginEditor.h"


//==============================================================================
AmbisonicCompressorAudioProcessorEditor::AmbisonicCompressorAudioProcessorEditor (AmbisonicCompressorAudioProcessor& p,AudioProcessorValueTreeState& vts)
    : AudioProcessorEditor (&p), processor (p), valueTreeState(vts)
{
    // Make sure that before the constructor has finished, you've set the
    // editor's size to whatever you need it to be.
    setSize (550, 445);
    setLookAndFeel (&globalLaF);
    
    addAndMakeVisible(&title);
    title.setTitle(String("Directional"),String("Compressor"));
    title.setFont(globalLaF.robotoBold,globalLaF.robotoLight);
    addAndMakeVisible(&footer);
    
    addAndMakeVisible(&sphere);
    sphere.addElement(&sphereElem);
    sphere.addListener(this);
    
    
    cbNormalizationAtachement = new ComboBoxAttachment(valueTreeState,"useSN3D", *title.getInputWidgetPtr()->getNormCbPointer());
    cbOrderAtachement = new ComboBoxAttachment(valueTreeState,"orderSetting", *title.getInputWidgetPtr()->getOrderCbPointer());
    
    
    
    // ======== general components ===========
    addAndMakeVisible(&gcMask);
    gcMask.setText("Spatial mask properties");
    gcMask.setTextLabelPosition (Justification::centredLeft);
    gcMask.setColour (GroupComponent::outlineColourId, globalLaF.ClSeperator);
    gcMask.setColour (GroupComponent::textColourId, Colours::white);
    gcMask.setVisible(true);
    
    addAndMakeVisible(&gcSettings);
    gcSettings.setText("General settings");
    gcSettings.setTextLabelPosition (Justification::centredLeft);
    gcSettings.setColour (GroupComponent::outlineColourId, globalLaF.ClSeperator);
    gcSettings.setColour (GroupComponent::textColourId, Colours::white);
    gcSettings.setVisible(true);
    
    addAndMakeVisible(&slPreGain);
    slPreGainAttachment = new SliderAttachment(valueTreeState,"preGain", slPreGain);
    slPreGain.setSliderStyle (Slider::RotaryHorizontalVerticalDrag);
    slPreGain.setTextBoxStyle (Slider::TextBoxBelow, false, 50, 15);
    slPreGain.setColour (Slider::rotarySliderOutlineColourId, globalLaF.ClWidgetColours[1]);
    slPreGain.setTooltip("PreGain");
    slPreGain.setTextValueSuffix(" dB");
    
    addAndMakeVisible(&slYaw);
    slYawAttachment = new SliderAttachment(valueTreeState,"yaw", slYaw);
    slYaw.setSliderStyle (Slider::Rotary);
    slYaw.setTextBoxStyle (Slider::TextBoxBelow, false, 50, 15);
    slYaw.setReverse(true);
    slYaw.setColour (Slider::rotarySliderOutlineColourId, globalLaF.ClWidgetColours[0]);
    slYaw.setRotaryParameters(M_PI, 3*M_PI, false);
    slYaw.setTooltip("Yaw angle");
    slYaw.setTextValueSuffix(CharPointer_UTF8 ("\xc2\xb0"));
    
    addAndMakeVisible(&slPitch);
    slPitchAttachment = new SliderAttachment(valueTreeState,"pitch", slPitch);
    slPitch.setSliderStyle (Slider::Rotary);
    slPitch.setTextBoxStyle (Slider::TextBoxBelow, false, 50, 15);
    slPitch.setColour (Slider::rotarySliderOutlineColourId, globalLaF.ClWidgetColours[1]);
    slPitch.setReverse(true);
    slPitch.setRotaryParameters(0.5*M_PI, 2.5*M_PI, false);
    slPitch.setTooltip("Pitch angle");
    slPitch.setTextValueSuffix(CharPointer_UTF8 ("\xc2\xb0"));
    
    addAndMakeVisible(&slWidth);
    slWidthAttachment = new SliderAttachment(valueTreeState,"width", slWidth);
    slWidth.setSliderStyle (Slider::Rotary);
    slWidth.setTextBoxStyle (Slider::TextBoxBelow, false, 50, 15);
    slWidth.setColour (Slider::rotarySliderOutlineColourId, globalLaF.ClWidgetColours[2]);
    slWidth.setTooltip("Mask width");
    slWidth.setTextValueSuffix(CharPointer_UTF8 ("\xc2\xb0"));
    
    addAndMakeVisible(&cbListen);
    cbListenAttachment = new ComboBoxAttachment(valueTreeState,"listen", cbListen);
    cbListen.setJustificationType(Justification::centred);
    cbListen.addSectionHeading("Listen to");
    cbListen.addItem("Full", 1);
    cbListen.addItem("Masked", 2);
    cbListen.addItem("Unmasked", 3);
    cbListen.setSelectedId(*valueTreeState.getRawParameterValue("listen")+1);
    
    // ======== compressor 1 components ===========
    bool isOn = *valueTreeState.getRawParameterValue("c1Enabled");
    
    addAndMakeVisible(&gcC1);
    gcC1.setText("Compressor 1");
    gcC1.setTextLabelPosition (Justification::centredLeft);
    gcC1.setColour (GroupComponent::outlineColourId, globalLaF.ClSeperator);
    gcC1.setColour (GroupComponent::textColourId, Colours::white);
    gcC1.setVisible(true);
    
    addAndMakeVisible(&tbC1);
    tbC1Attachment = new ButtonAttachment(valueTreeState,"c1Enabled",tbC1);
    tbC1.setColour(ToggleButton::tickColourId, globalLaF.ClWidgetColours[0]);
    tbC1.setButtonText("ON/OFF");
    tbC1.setName("C1");
    tbC1.addListener(this);
    
    addAndMakeVisible(&cbC1Driving);
    cbC1DrivingAttachment = new ComboBoxAttachment(valueTreeState,"c1DrivingSignal", cbC1Driving);
    cbC1Driving.setJustificationType(Justification::centred);
    cbC1Driving.addSectionHeading("Driving signa");
    cbC1Driving.addItem("Full", 1);
    cbC1Driving.addItem("Masked", 2);
    cbC1Driving.addItem("Unmasked", 3);
    cbC1Driving.setSelectedId(*valueTreeState.getRawParameterValue("c1DrivingSignal")+1);
    cbC1Driving.setEnabled(isOn);
    
    addAndMakeVisible(&cbC1Apply);
    cbC1ApplyAttachment = new ComboBoxAttachment(valueTreeState,"c1Apply", cbC1Apply);
    cbC1Apply.setJustificationType(Justification::centred);
    cbC1Apply.addSectionHeading("Apply to");
    cbC1Apply.addItem("Full", 1);
    cbC1Apply.addItem("Masked", 2);
    cbC1Apply.addItem("Unmasked", 3);
    cbC1Apply.setSelectedId(*valueTreeState.getRawParameterValue("c1Apply")+1);
    cbC1Apply.setEnabled(isOn);
    
    addAndMakeVisible(&slC1Threshold);
    slC1ThresholdAttachment = new SliderAttachment(valueTreeState,"c1Threshold", slC1Threshold);
    slC1Threshold.setSliderStyle (Slider::RotaryHorizontalVerticalDrag);
    slC1Threshold.setTextBoxStyle (Slider::TextBoxBelow, false, 50, 15);
    slC1Threshold.setColour (Slider::rotarySliderOutlineColourId, globalLaF.ClWidgetColours[2]);
    slC1Threshold.setTextValueSuffix(" dB");
    slC1Threshold.setEnabled(isOn);
    
    addAndMakeVisible(&slC1Ratio);
    slC1RatioAttachment = new SliderAttachment(valueTreeState,"c1Ratio", slC1Ratio);
    slC1Ratio.setSliderStyle (Slider::RotaryHorizontalVerticalDrag);
    slC1Ratio.setTextBoxStyle (Slider::TextBoxBelow, false, 50, 15);
    slC1Ratio.setColour (Slider::rotarySliderOutlineColourId, globalLaF.ClWidgetColours[3]);
    slC1Ratio.setTextValueSuffix(" : 1");
    slC1Ratio.setEnabled(isOn);
    
    addAndMakeVisible(&slC1Attack);
    slC1AttackAttachment = new SliderAttachment(valueTreeState,"c1Attack", slC1Attack);
    slC1Attack.setSliderStyle (Slider::RotaryHorizontalVerticalDrag);
    slC1Attack.setTextBoxStyle (Slider::TextBoxBelow, false, 50, 15);
    slC1Attack.setColour (Slider::rotarySliderOutlineColourId, globalLaF.ClWidgetColours[0]);
    slC1Attack.setTextValueSuffix(" ms");
    slC1Attack.setEnabled(isOn);
    
    addAndMakeVisible(&slC1Release);
    slC1ReleaseAttachment = new SliderAttachment(valueTreeState,"c1Release", slC1Release);
    slC1Release.setSliderStyle (Slider::RotaryHorizontalVerticalDrag);
    slC1Release.setTextBoxStyle (Slider::TextBoxBelow, false, 50, 15);
    slC1Release.setColour (Slider::rotarySliderOutlineColourId, globalLaF.ClWidgetColours[0]);
    slC1Release.setTextValueSuffix(" ms");
    slC1Release.setEnabled(isOn);
    
    addAndMakeVisible(&slC1Makeup);
    slC1MakeupAttachment = new SliderAttachment(valueTreeState,"c1Makeup", slC1Makeup);
    slC1Makeup.setSliderStyle (Slider::RotaryHorizontalVerticalDrag);
    slC1Makeup.setTextBoxStyle (Slider::TextBoxBelow, false, 50, 15);
    slC1Makeup.setColour (Slider::rotarySliderOutlineColourId, globalLaF.ClWidgetColours[1]);
    slC1Makeup.setTextValueSuffix(" dB");
    slC1Makeup.setEnabled(isOn);
    
    addAndMakeVisible(&dbC1GRmeter);
    dbC1GRmeter.setMinLevel(-25.0f);
    dbC1GRmeter.setColour(Colours::red.withMultipliedAlpha(0.8f));
    dbC1GRmeter.setGainReductionMeter(true);
    dbC1GRmeter.setEnabled(isOn);
    
    addAndMakeVisible(&dbC1RMSmeter);
    dbC1RMSmeter.setMinLevel(-60.0f);
    dbC1RMSmeter.setColour(Colours::green.withMultipliedAlpha(0.8f));
    dbC1RMSmeter.setGainReductionMeter(false);
    dbC1RMSmeter.setEnabled(isOn);
    
    // ======== compressor 2 components ===========
    isOn = *valueTreeState.getRawParameterValue("c2Enabled");
    
    addAndMakeVisible(&gcC2);
    gcC2.setText("Compressor 2");
    gcC2.setTextLabelPosition (Justification::centredLeft);
    gcC2.setColour (GroupComponent::outlineColourId, globalLaF.ClSeperator);
    gcC2.setColour (GroupComponent::textColourId, Colours::white);
    gcC2.setVisible(true);
    
    addAndMakeVisible(&tbC2);
    tbC2Attachment = new ButtonAttachment(valueTreeState,"c2Enabled",tbC2);
    tbC2.setColour(ToggleButton::tickColourId, globalLaF.ClWidgetColours[0]);
    tbC2.setButtonText("ON/OFF");
    tbC2.setName("C2");
    tbC2.addListener(this);
    
    addAndMakeVisible(&cbC2Driving);
    cbC2DrivingAttachment = new ComboBoxAttachment(valueTreeState,"c2DrivingSignal", cbC2Driving);
    cbC2Driving.setJustificationType(Justification::centred);
    cbC2Driving.addSectionHeading("Driving signal");
    cbC2Driving.addItem("Full", 1);
    cbC2Driving.addItem("Masked", 2);
    cbC2Driving.addItem("Unmasked", 3);
    cbC2Driving.setSelectedId(*valueTreeState.getRawParameterValue("c2DrivingSignal")+1);
    cbC2Driving.setEnabled(isOn);
    
    addAndMakeVisible(&cbC2Apply);
    cbC2ApplyAttachment = new ComboBoxAttachment(valueTreeState,"c2Apply", cbC2Apply);
    cbC2Apply.setJustificationType(Justification::centred);
    cbC2Apply.addSectionHeading("Apply to");
    cbC2Apply.addItem("Full", 1);
    cbC2Apply.addItem("Masked", 2);
    cbC2Apply.addItem("Unmasked", 3);
    cbC2Apply.setSelectedId(*valueTreeState.getRawParameterValue("c2Apply")+1);
    cbC2Apply.setEnabled(isOn);
    
    addAndMakeVisible(&slC2Threshold);
    slC2ThresholdAttachment = new SliderAttachment(valueTreeState,"c2Threshold", slC2Threshold);
    slC2Threshold.setSliderStyle (Slider::RotaryHorizontalVerticalDrag);
    slC2Threshold.setTextBoxStyle (Slider::TextBoxBelow, false, 50, 15);
    slC2Threshold.setColour (Slider::rotarySliderOutlineColourId, globalLaF.ClWidgetColours[2]);
    slC2Threshold.setTextValueSuffix(" dB");
    slC2Threshold.setEnabled(isOn);
    
    addAndMakeVisible(&slC2Ratio);
    slC2RatioAttachment = new SliderAttachment(valueTreeState,"c2Ratio", slC2Ratio);
    slC2Ratio.setSliderStyle (Slider::RotaryHorizontalVerticalDrag);
    slC2Ratio.setTextBoxStyle (Slider::TextBoxBelow, false, 50, 15);
    slC2Ratio.setColour (Slider::rotarySliderOutlineColourId, globalLaF.ClWidgetColours[3]);
    slC2Ratio.setTextValueSuffix(" : 1");
    slC2Ratio.setEnabled(isOn);
    
    addAndMakeVisible(&slC2Attack);
    slC2AttackAttachment = new SliderAttachment(valueTreeState,"c2Attack", slC2Attack);
    slC2Attack.setSliderStyle (Slider::RotaryHorizontalVerticalDrag);
    slC2Attack.setTextBoxStyle (Slider::TextBoxBelow, false, 50, 15);
    slC2Attack.setColour (Slider::rotarySliderOutlineColourId, globalLaF.ClWidgetColours[0]);
    slC2Attack.setTextValueSuffix(" ms");
    slC2Attack.setEnabled(isOn);
    
    addAndMakeVisible(&slC2Release);
    slC2ReleaseAttachment = new SliderAttachment(valueTreeState,"c2Release", slC2Release);
    slC2Release.setSliderStyle (Slider::RotaryHorizontalVerticalDrag);
    slC2Release.setTextBoxStyle (Slider::TextBoxBelow, false, 50, 15);
    slC2Release.setColour (Slider::rotarySliderOutlineColourId, globalLaF.ClWidgetColours[0]);
    slC2Release.setTextValueSuffix(" ms");
    slC2Release.setEnabled(isOn);
    
    addAndMakeVisible(&slC2Makeup);
    slC2MakeupAttachment = new SliderAttachment(valueTreeState,"c2Makeup", slC2Makeup);
    slC2Makeup.setSliderStyle (Slider::RotaryHorizontalVerticalDrag);
    slC2Makeup.setTextBoxStyle (Slider::TextBoxBelow, false, 50, 15);
    slC2Makeup.setColour (Slider::rotarySliderOutlineColourId, globalLaF.ClWidgetColours[1]);
    slC2Makeup.setTextValueSuffix(" dB");
    slC2Makeup.setEnabled(isOn);
    
    addAndMakeVisible(&dbC2GRmeter);
    dbC2GRmeter.setMinLevel(-25.0f);
    dbC2GRmeter.setColour(Colours::red.withMultipliedAlpha(0.8f));
    dbC2GRmeter.setGainReductionMeter(true);
    dbC2GRmeter.setEnabled(isOn);
    
    addAndMakeVisible(&dbC2RMSmeter);
    dbC2RMSmeter.setMinLevel(-60.0f);
    dbC2RMSmeter.setColour(Colours::green.withMultipliedAlpha(0.8f));
    dbC2RMSmeter.setGainReductionMeter(false);
    dbC2RMSmeter.setEnabled(isOn);
    
    // ===== LABELS =====
    addAndMakeVisible(&lbPreGain);
    lbPreGain.setText("Pre Gain");
    addAndMakeVisible(&lbYaw);
    lbYaw.setText("Yaw");
    addAndMakeVisible(&lbPitch);
    lbPitch.setText("Pitch");
    addAndMakeVisible(&lbWidth);
    lbWidth.setText("width");
    
    addAndMakeVisible(&lbC1Threshold);
    lbC1Threshold.setText("Threshold");
    addAndMakeVisible(&lbC1Ratio);
    lbC1Ratio.setText("Ratio");
    addAndMakeVisible(&lbC1Attack);
    lbC1Attack.setText("Attack");
    addAndMakeVisible(&lbC1Release);
    lbC1Release.setText("Release");
    addAndMakeVisible(&lbC1Makeup);
    lbC1Makeup.setText("MakeUp");
    
    addAndMakeVisible(&lbC2Threshold);
    lbC2Threshold.setText("Threshold");
    addAndMakeVisible(&lbC2Ratio);
    lbC2Ratio.setText("Ratio");
    addAndMakeVisible(&lbC2Attack);
    lbC2Attack.setText("Attack");
    addAndMakeVisible(&lbC2Release);
    lbC2Release.setText("Release");
    addAndMakeVisible(&lbC2Makeup);
    lbC2Makeup.setText("MakeUp");
    
    startTimer(50);
}

AmbisonicCompressorAudioProcessorEditor::~AmbisonicCompressorAudioProcessorEditor()
{
    setLookAndFeel(nullptr);
}

void AmbisonicCompressorAudioProcessorEditor::IEMSphereElementChanged (IEMSphere* sphere, IEMSphereElement* element) {
    
    Vector3D<float> pos = element->getPosition();
    float hypxy = sqrt(pos.x*pos.x+pos.y*pos.y);
    
    
    float yaw = atan2f(pos.y,pos.x);
    float pitch = atan2f(hypxy,pos.z)-M_PI/2;
    
    valueTreeState.getParameter("yaw")->setValue(valueTreeState.getParameterRange("yaw").convertTo0to1(yaw/M_PI*180.0f));
    valueTreeState.getParameter("pitch")->setValue(valueTreeState.getParameterRange("pitch").convertTo0to1(pitch/M_PI*180.0f));
    
    //DBG("yaw: " << yaw/M_PI*180 << " pitch: " << pitch/M_PI*180);
    
};

void AmbisonicCompressorAudioProcessorEditor::buttonStateChanged (Button* button)
{
    bool isOn = button->getToggleState();
    if (button->getName() == "C1")
    {
        slC1Threshold.setEnabled(isOn);
        slC1Ratio.setEnabled(isOn);
        slC1Attack.setEnabled(isOn);
        slC1Release.setEnabled(isOn);
        slC1Makeup.setEnabled(isOn);
     
        lbC1Threshold.setEnabled(isOn);
        lbC1Ratio.setEnabled(isOn);
        lbC1Attack.setEnabled(isOn);
        lbC1Release.setEnabled(isOn);
        lbC1Makeup.setEnabled(isOn);
        
        cbC1Driving.setEnabled(isOn);
        cbC1Apply.setEnabled(isOn);;
    }
    else if (button->getName() == "C2")
    {
        slC2Threshold.setEnabled(isOn);
        slC2Ratio.setEnabled(isOn);
        slC2Attack.setEnabled(isOn);
        slC2Release.setEnabled(isOn);
        slC2Makeup.setEnabled(isOn);
        
        lbC2Threshold.setEnabled(isOn);
        lbC2Ratio.setEnabled(isOn);
        lbC2Attack.setEnabled(isOn);
        lbC2Release.setEnabled(isOn);
        lbC2Makeup.setEnabled(isOn);
        
        cbC2Driving.setEnabled(isOn);
        cbC2Apply.setEnabled(isOn);;
    }
};

//==============================================================================
void AmbisonicCompressorAudioProcessorEditor::paint (Graphics& g)
{
    // (Our component is opaque, so we must completely fill the background with a solid colour)
    g.fillAll (globalLaF.ClBackground);


}

void AmbisonicCompressorAudioProcessorEditor::timerCallback()
{
    // === update titleBar widgets according to available input/output channel counts
    int maxInSize, maxOutSize;
    processor.getMaxSize(maxInSize, maxOutSize);
    maxOutSize = jmin(maxInSize, maxOutSize);
    maxInSize = maxOutSize;
    title.setMaxSize(maxInSize, maxOutSize);
    // ==========================================
    
    if (sphereElem.setPosition(Vector3D<float>(processor.xyz[0], processor.xyz[1], processor.xyz[2])))
        sphere.repaint();
    
    dbC1RMSmeter.setLevel(processor.c1MaxRMS);
    dbC1GRmeter.setLevel(processor.c1MaxGR);
    dbC2RMSmeter.setLevel(processor.c2MaxRMS);
    dbC2GRmeter.setLevel(processor.c2MaxGR);
}


void AmbisonicCompressorAudioProcessorEditor::resized()
{
    const int leftRightMargin = 30;
    const int headerHeight = 60;
    const int footerHeight = 25;
    const int sliderWidth = 45;
//    const int labelHeight = 15;
//    const int sliderHeight = 60;
    const int sliderSpacing = 15;
    
    Rectangle<int> area (getLocalBounds());
    
    Rectangle<int> footerArea (area.removeFromBottom (footerHeight));
    footer.setBounds(footerArea);
    
    area.removeFromLeft(leftRightMargin);
    area.removeFromRight(leftRightMargin);
    Rectangle<int> headerArea = area.removeFromTop    (headerHeight);
    title.setBounds (headerArea);
    area.removeFromTop(10);
    
    Rectangle<int> temp; //work area
    Rectangle<int> sliderRow;
    Rectangle<int> comprRow;
    comprRow = area.removeFromBottom(200);
    
    // ======== compressor 1
    {
        temp = comprRow.removeFromLeft(225);
        gcC1.setBounds(temp);
        sliderRow = temp.removeFromTop(15);
        sliderRow.removeFromRight(60);
        tbC1.setBounds(sliderRow.removeFromRight(40));
        temp.removeFromTop(10);
        
        // dbMeters
        dbC1RMSmeter.setBounds(temp.removeFromLeft(20));
        temp.removeFromLeft(10);
        dbC1GRmeter.setBounds(temp.removeFromRight(20));
        temp.removeFromRight(10);
        
        // comboBoxes
        sliderRow = temp.removeFromTop(15);
        cbC1Driving.setBounds(sliderRow.removeFromLeft(75));
        cbC1Apply.setBounds(sliderRow.removeFromRight(75));
        
        temp.removeFromTop(5);//spacing
        
        //first Sliders
        sliderRow = temp.removeFromTop(60);
        sliderRow.removeFromLeft(25);
        slC1Threshold.setBounds(sliderRow.removeFromLeft(45));
        sliderRow.removeFromRight(25);
        slC1Makeup.setBounds(sliderRow.removeFromRight(45));
        
        sliderRow = temp.removeFromTop(15);
        sliderRow.removeFromLeft(20);
        lbC1Threshold.setBounds(sliderRow.removeFromLeft(50));
        sliderRow.removeFromRight(25);
        lbC1Makeup.setBounds(sliderRow.removeFromRight(45));
        
        temp.removeFromTop(5);//spacing
        
        // next Sliders
        sliderRow = temp.removeFromTop(60);
        slC1Ratio.setBounds(sliderRow.removeFromLeft(45));
        sliderRow.removeFromLeft(sliderSpacing);
        slC1Attack.setBounds(sliderRow.removeFromLeft(45));
        sliderRow.removeFromLeft(sliderSpacing);
        slC1Release.setBounds(sliderRow.removeFromLeft(45));
        
        sliderRow = temp.removeFromTop(15);
        lbC1Ratio.setBounds(sliderRow.removeFromLeft(45));
        sliderRow.removeFromLeft(sliderSpacing);
        lbC1Attack.setBounds(sliderRow.removeFromLeft(45));
        sliderRow.removeFromLeft(sliderSpacing);
        lbC1Release.setBounds(sliderRow.removeFromLeft(45));
    }
    
    // ======== compressor 2
    {
        temp = comprRow.removeFromRight(225);
        gcC2.setBounds(temp);
        sliderRow = temp.removeFromTop(15);
        sliderRow.removeFromRight(60);
        tbC2.setBounds(sliderRow.removeFromRight(40));
        temp.removeFromTop(10);
        
        // dbMeters
        dbC2RMSmeter.setBounds(temp.removeFromLeft(20));
        temp.removeFromLeft(10);
        dbC2GRmeter.setBounds(temp.removeFromRight(20));
        temp.removeFromRight(10);
        
        // comboBoxes
        sliderRow = temp.removeFromTop(15);
        cbC2Driving.setBounds(sliderRow.removeFromLeft(75));
        cbC2Apply.setBounds(sliderRow.removeFromRight(75));
        
        temp.removeFromTop(5);//spacing
        
        //first Sliders
        sliderRow = temp.removeFromTop(60);
        sliderRow.removeFromLeft(25);
        slC2Threshold.setBounds(sliderRow.removeFromLeft(45));
        sliderRow.removeFromRight(25);
        slC2Makeup.setBounds(sliderRow.removeFromRight(45));
        
        sliderRow = temp.removeFromTop(15);
        sliderRow.removeFromLeft(20);
        lbC2Threshold.setBounds(sliderRow.removeFromLeft(50));
        sliderRow.removeFromRight(25);
        lbC2Makeup.setBounds(sliderRow.removeFromRight(45));
        
        temp.removeFromTop(5);//spacing
        
        // next Sliders
        sliderRow = temp.removeFromTop(60);
        slC2Ratio.setBounds(sliderRow.removeFromLeft(45));
        sliderRow.removeFromLeft(sliderSpacing);
        slC2Attack.setBounds(sliderRow.removeFromLeft(45));
        sliderRow.removeFromLeft(sliderSpacing);
        slC2Release.setBounds(sliderRow.removeFromLeft(45));
        
        sliderRow = temp.removeFromTop(15);
        lbC2Ratio.setBounds(sliderRow.removeFromLeft(45));
        sliderRow.removeFromLeft(sliderSpacing);
        lbC2Attack.setBounds(sliderRow.removeFromLeft(45));
        sliderRow.removeFromLeft(sliderSpacing);
        lbC2Release.setBounds(sliderRow.removeFromLeft(45));
    }
    
    area.removeFromBottom(10); //spacing
    
    int height = area.getHeight();
    sphere.setBounds(area.removeFromLeft(height));

    area.removeFromLeft(10); //spacing
    temp = area.removeFromLeft(165);
    
    // MASK PROPERTIES
    gcMask.setBounds(temp);
    temp.removeFromTop(25); //spacing
    
    sliderRow = temp.removeFromTop(60);
    slYaw.setBounds(sliderRow.removeFromLeft(sliderWidth));
    sliderRow.removeFromLeft(sliderSpacing);
    slPitch.setBounds(sliderRow.removeFromLeft(sliderWidth));
    sliderRow.removeFromLeft(sliderSpacing);
    slWidth.setBounds(sliderRow.removeFromLeft(sliderWidth));
    
    sliderRow = temp.removeFromTop(15);
    lbYaw.setBounds(sliderRow.removeFromLeft(sliderWidth));
    sliderRow.removeFromLeft(sliderSpacing);
    lbPitch.setBounds(sliderRow.removeFromLeft(sliderWidth));
    sliderRow.removeFromLeft(sliderSpacing);
    lbWidth.setBounds(sliderRow.removeFromLeft(sliderWidth));
    
    area.removeFromLeft(30); //spacing
    
    // GENERAL SETTINGS
    
    gcSettings.setBounds(area);
    area.removeFromTop(25); //spacing
    
    temp = area.removeFromLeft(sliderWidth);
    sliderRow = temp.removeFromTop(60);
    slPreGain.setBounds(sliderRow.removeFromLeft(sliderWidth));
    
    sliderRow = temp.removeFromTop(15);
    lbPreGain.setBounds(sliderRow.removeFromLeft(sliderWidth));
    
    area.removeFromLeft(15); //spacing
    cbListen.setBounds(area.removeFromTop(15));
}
