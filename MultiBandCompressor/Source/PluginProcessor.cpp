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

#include "PluginProcessor.h"
#include "PluginEditor.h"

//==============================================================================
MultiBandCompressorAudioProcessor::MultiBandCompressorAudioProcessor()
     : AudioProcessorBase (
#ifndef JucePlugin_PreferredChannelConfigurations
                           BusesProperties()
#if ! JucePlugin_IsMidiEffect
#if ! JucePlugin_IsSynth
                           .withInput  ("Input",  juce::AudioChannelSet::discreteChannels (64), true)
#endif
                           .withOutput ("Output", juce::AudioChannelSet::discreteChannels (64), true)
#endif
                           ,
#endif
                           createParameterLayout()),
       maxNumFilters (ceil (64 / IIRfloat_elements))
{
    const juce::String inputSettingID = "orderSetting";
    orderSetting = parameters.getRawParameterValue (inputSettingID);
    parameters.addParameterListener (inputSettingID, this);


    for (int i = 0; i < numFilterBands-1; ++i)
    {
        const juce::String crossoverID ("crossover" + juce::String (i));

        crossovers[i] = parameters.getRawParameterValue (crossoverID);

        lowPassLRCoeffs[i] = IIR::Coefficients<double>::makeLowPass (lastSampleRate, *crossovers[i]);
        highPassLRCoeffs[i] = IIR::Coefficients<double>::makeHighPass (lastSampleRate, *crossovers[i]);

        calculateCoefficients (i);

        iirLPCoefficients[i] = IIR::Coefficients<float>::makeLowPass (lastSampleRate, *crossovers[i]);
        iirHPCoefficients[i] = IIR::Coefficients<float>::makeHighPass (lastSampleRate, *crossovers[i]);
        iirAPCoefficients[i] = IIR::Coefficients<float>::makeAllPass (lastSampleRate, *crossovers[i]);

        parameters.addParameterListener (crossoverID, this);

        iirLP[i].clear();
        iirLP2[i].clear();
        iirHP[i].clear();
        iirHP2[i].clear();
        iirAP[i].clear();

        for (int ch = 0; ch < maxNumFilters; ++ch)
        {
            iirLP[i].add (new IIR::Filter<IIRfloat> (iirLPCoefficients[i]));
            iirLP2[i].add (new IIR::Filter<IIRfloat> (iirLPCoefficients[i]));
            iirHP[i].add (new IIR::Filter<IIRfloat> (iirHPCoefficients[i]));
            iirHP2[i].add (new IIR::Filter<IIRfloat> (iirHPCoefficients[i]));
            iirAP[i].add (new IIR::Filter<IIRfloat> (iirAPCoefficients[i]));
        }
    }

    for (int i = 0; i < numFilterBands; ++i)
    {
        for (int ch = 0; ch < maxNumFilters; ++ch)
        {
            freqBandsBlocks[i].push_back (juce::HeapBlock<char> ());
        }

        const juce::String thresholdID ("threshold" + juce::String (i));
        const juce::String kneeID ("knee" + juce::String (i));
        const juce::String attackID ("attack" + juce::String (i));
        const juce::String releaseID ("release" + juce::String (i));
        const juce::String ratioID ("ratio" + juce::String (i));
        const juce::String makeUpGainID ("makeUpGain" + juce::String (i));
        const juce::String bypassID ("bypass" + juce::String (i));
        const juce::String soloID ("solo" + juce::String (i));

        threshold[i] = parameters.getRawParameterValue (thresholdID);
        knee[i] = parameters.getRawParameterValue (kneeID);
        attack[i] = parameters.getRawParameterValue (attackID);
        release[i] = parameters.getRawParameterValue (releaseID);
        ratio[i] = parameters.getRawParameterValue (ratioID);
        makeUpGain[i] = parameters.getRawParameterValue (makeUpGainID);
        bypass[i] = parameters.getRawParameterValue (bypassID);

        parameters.addParameterListener (thresholdID, this);
        parameters.addParameterListener (kneeID, this);
        parameters.addParameterListener (attackID, this);
        parameters.addParameterListener (releaseID, this);
        parameters.addParameterListener (ratioID, this);
        parameters.addParameterListener (makeUpGainID, this);
        parameters.addParameterListener (bypassID, this);
        parameters.addParameterListener (soloID, this);
    }

    soloArray.clear();

    copyCoeffsToProcessor();
}

MultiBandCompressorAudioProcessor::~MultiBandCompressorAudioProcessor()
{
}

std::vector<std::unique_ptr<juce::RangedAudioParameter>> MultiBandCompressorAudioProcessor::createParameterLayout()
{
    std::vector<std::unique_ptr<juce::RangedAudioParameter>> params;
    const float crossoverPresets [numFilterBands-1] = { 80.0f, 440.0f, 2200.0f };

    auto floatParam = std::make_unique<juce::AudioParameterFloat> ("orderSetting",
                                                        "Ambisonics Order",
                                                        juce::NormalisableRange<float>(0.0f, 8.0f, 1.0f), 0.0f,
                                                        "",
                                                        juce::AudioProcessorParameter::genericParameter,
                                                        [](float value, int maximumStringLength)
                                                        {
                                                            if (value >= 0.5f && value < 1.5f) return "0th";
                                                            else if (value >= 1.5f && value < 2.5f) return "1st";
                                                            else if (value >= 2.5f && value < 3.5f) return "2nd";
                                                            else if (value >= 3.5f && value < 4.5f) return "3rd";
                                                            else if (value >= 4.5f && value < 5.5f) return "4th";
                                                            else if (value >= 5.5f && value < 6.5f) return "5th";
                                                            else if (value >= 6.5f && value < 7.5f) return "6th";
                                                            else if (value >= 7.5f) return "7th";
                                                            else return "Auto";
                                                       }, nullptr);
    params.push_back (std::move (floatParam));

    floatParam = std::make_unique<juce::AudioParameterFloat> ("useSN3D", "Normalization",
                                                        juce::NormalisableRange<float>(0.0f, 1.0f, 1.0f), 1.0f,
                                                        "",
                                                        juce::AudioProcessorParameter::genericParameter,
                                                        [](float value, int maximumStringLength)
                                                        {
                                                            if (value >= 0.5f) return "SN3D";
                                                            else return "N3D";
                                                        },
                                                        nullptr);
    params.push_back (std::move (floatParam));


    // Crossovers
    for (int i = 0; i < numFilterBands-1; ++i)
    {
        floatParam = std::make_unique<juce::AudioParameterFloat> ("crossover" + juce::String (i),
                                                            "Crossover " + juce::String (i),
                                                            juce::NormalisableRange<float> (20.0f, 20000.0f, 0.1f, 0.4f),
                                                            crossoverPresets[i], "Hz",
                                                            juce::AudioProcessorParameter::genericParameter,
                                                            std::function <juce::String (float value, int maximumStringLength)> ([](float v, int m) {return juce::String (v, m);}),
                                                            std::function< float (const juce::String &text)> ([](const juce::String &t){return t.getFloatValue();}));
        params.push_back (std::move (floatParam));
    }


    for (int i = 0; i < numFilterBands; ++i)
    {
        // compressor threshold
        floatParam = std::make_unique<juce::AudioParameterFloat>("threshold" + juce::String (i),
                                                           "Threshold " + juce::String (i),
                                                           juce::NormalisableRange<float> (-50.0f, 10.0f, 0.1f),
                                                           -10.0f, "dB",
                                                           juce::AudioProcessorParameter::genericParameter,
                                                           std::function <juce::String (float value, int maximumStringLength)> ([](float v, int m){return juce::String (v, 1);}),
                                                           std::function< float (const juce::String &text)> ([](const juce::String &t){return t.getFloatValue();})
                                                          );
        params.push_back (std::move (floatParam));

        // knee
        floatParam = std::make_unique<juce::AudioParameterFloat>("knee" + juce::String (i),
                                                           "Knee Width " + juce::String (i),
                                                           juce::NormalisableRange<float> (0.0f, 30.0f, 0.1f),
                                                           0.0f, "dB",
                                                           juce::AudioProcessorParameter::genericParameter,
                                                           std::function <juce::String (float value, int maximumStringLength)> ([](float v, int m){return juce::String (v, 1);}),
                                                           std::function< float (const juce::String &text)> ([](const juce::String &t){return t.getFloatValue();})
                                                          );
        params.push_back (std::move (floatParam));

        // attack
        floatParam = std::make_unique<juce::AudioParameterFloat>("attack" + juce::String (i),
                                                           "Attack Time " + juce::String (i),
                                                           juce::NormalisableRange<float> (0.0f, 100.0f, 0.1f),
                                                           30.0f, "ms",
                                                           juce::AudioProcessorParameter::genericParameter,
                                                           std::function <juce::String (float value, int maximumStringLength)> ([](float v, int m){return juce::String (v, 1);}),
                                                           std::function< float (const juce::String &text)> ([](const juce::String &t){return t.getFloatValue();})
                                                          );
        params.push_back (std::move (floatParam));

        // release
        floatParam = std::make_unique<juce::AudioParameterFloat>("release" + juce::String (i),
                                                           "Release Time " + juce::String (i),
                                                           juce::NormalisableRange<float> (0.0f, 500.0f, 0.1f),
                                                           150.0f, "ms",
                                                           juce::AudioProcessorParameter::genericParameter,
                                                           std::function <juce::String (float value, int maximumStringLength)> ([](float v, int m){return juce::String (v, 1);}),
                                                           std::function< float (const juce::String &text)> ([](const juce::String &t){return t.getFloatValue();})
                                                          );
        params.push_back (std::move (floatParam));

        // ratio
        floatParam = std::make_unique<juce::AudioParameterFloat>("ratio" + juce::String (i),
                                                           "Ratio " + juce::String (i),
                                                           juce::NormalisableRange<float> (1.0f, 16.0f, 0.1f),
                                                           4.0f, " : 1",
                                                           juce::AudioProcessorParameter::genericParameter,
                                                           std::function <juce::String (float value, int maximumStringLength)> ([](float v, int m){
                                                             return (v > 15.9f) ? juce::String ("inf") : juce::String (v, 1);}),
                                                           std::function< float (const juce::String &text)> ([](const juce::String &t){return t.getFloatValue();})
                                                          );
        params.push_back (std::move (floatParam));

        // makeUpGain
        floatParam = std::make_unique<juce::AudioParameterFloat>("makeUpGain" + juce::String (i),
                                                           "MakUp Gain " + juce::String (i),
                                                           juce::NormalisableRange<float> (-10.0f, 20.0f, 0.1f),
                                                           0.0f, "dB",
                                                           juce::AudioProcessorParameter::genericParameter,
                                                           std::function <juce::String (float value, int maximumStringLength)> ([](float v, int m){return juce::String (v, 1);}),
                                                           std::function< float (const juce::String &text)> ([](const juce::String &t){return t.getFloatValue();})
                                                          );
        params.push_back (std::move (floatParam));


        auto boolParam = std::make_unique<juce::AudioParameterBool>("bypass" + juce::String (i),
                                                           "Bypass compression on band " + juce::String (i),
                                                           false);
        params.push_back (std::move (boolParam));


        boolParam = std::make_unique<juce::AudioParameterBool>("solo" + juce::String (i),
                                                           "Solo band " + juce::String (i),
                                                           false);
        params.push_back (std::move (boolParam));
    }

    return params;
}

void MultiBandCompressorAudioProcessor::calculateCoefficients (const int i)
{
    jassert (lastSampleRate > 0.0);

    const float crossoverFrequency = juce::jmin (static_cast<float> (0.5 * lastSampleRate), crossovers[i]->load());

    double b0, b1, b2, a0, a1, a2;
    double K = std::tan (juce::MathConstants<double>::pi * (crossoverFrequency) / lastSampleRate);
    double den = 1 + juce::MathConstants<double>::sqrt2 * K + pow (K, double (2.0));

    // calculate coeffs for 2nd order Butterworth
    a0 = 1.0;
    a1 = (2*(pow (K,2.0) - 1)) / den;
    a2 = (1 - juce::MathConstants<double>::sqrt2*K + pow (K,2.0)) / den;

    // HP
    b0 = 1.0 / den;
    b1 = -2.0 * b0;
    b2 = b0;
    iirTempHPCoefficients[i] = new IIR::Coefficients<float>(b0, b1, b2, a0, a1, a2);

    // also calculate 4th order Linkwitz-Riley for GUI
    IIR::Coefficients<double>::Ptr coeffs (new IIR::Coefficients<double>(b0, b1, b2, a0, a1, a2));
    coeffs->coefficients = FilterVisualizerHelper<double>::cascadeSecondOrderCoefficients (coeffs->coefficients, coeffs->coefficients);
    *highPassLRCoeffs[i] = *coeffs;

    // LP
    b0 = pow (K,2.0) /   den;
    b1 = 2.0 * b0;
    b2 = b0;
    iirTempLPCoefficients[i] = new IIR::Coefficients<float>(b0, b1, b2, a0, a1, a2);

    coeffs.reset();
    coeffs = (new IIR::Coefficients<double>(b0, b1, b2, a0, a1, a2));
    coeffs->coefficients = FilterVisualizerHelper<double>::cascadeSecondOrderCoefficients (coeffs->coefficients, coeffs->coefficients);
    *lowPassLRCoeffs[i] = *coeffs;

    // Allpass equivalent to 4th order Linkwitz-Riley crossover
    iirTempAPCoefficients[i] = new IIR::Coefficients<float>(a2, a1, a0, a0, a1, a2);

}

void MultiBandCompressorAudioProcessor::copyCoeffsToProcessor()
{
    for (int b = 0; b < numFilterBands-1; ++b)
    {
        *iirLPCoefficients[b] = *iirTempLPCoefficients[b]; // LP
        *iirHPCoefficients[b] = *iirTempHPCoefficients[b]; // HP
        *iirAPCoefficients[b] = *iirTempAPCoefficients[b]; // AP
    }

    userChangedFilterSettings = false;
}

//==============================================================================
int MultiBandCompressorAudioProcessor::getNumPrograms()
{
    return 1;   // NB: some hosts don't cope very well if you tell them there are 0 programs,
                // so this should be at least 1, even if you're not really implementing programs.
}

int MultiBandCompressorAudioProcessor::getCurrentProgram()
{
    return 0;
}

void MultiBandCompressorAudioProcessor::setCurrentProgram (int index)
{
}

const juce::String MultiBandCompressorAudioProcessor::getProgramName (int index)
{
    return {};
}

void MultiBandCompressorAudioProcessor::changeProgramName (int index, const juce::String& newName)
{
}

//==============================================================================
void MultiBandCompressorAudioProcessor::prepareToPlay (double sampleRate, int samplesPerBlock)
{
    checkInputAndOutput (this, *orderSetting, *orderSetting, true);

    lastSampleRate = sampleRate;

    juce::dsp::ProcessSpec monoSpec;
    monoSpec.sampleRate = sampleRate;
    monoSpec.maximumBlockSize = samplesPerBlock;
    monoSpec.numChannels = 1;

    inputPeak = juce::Decibels::gainToDecibels (-INFINITY);
    outputPeak = juce::Decibels::gainToDecibels (-INFINITY);

    for (int i = 0; i < numFilterBands-1; ++i)
    {
        calculateCoefficients (i);
    }

    copyCoeffsToProcessor();

    interleavedData.clear();
    for (int i = 0; i < maxNumFilters; ++i)
    {
        interleavedData.add (new juce::dsp::AudioBlock<IIRfloat> (interleavedBlockData[i], 1, samplesPerBlock));
        
        clear (*interleavedData.getLast());
    }

    for (int i = 0; i < numFilterBands-1; ++i)
    {
        for (int ch = 0; ch < maxNumFilters; ++ch)
        {
            iirLP[i][ch]->reset (IIRfloat (0.0f));
            iirLP2[i][ch]->reset (IIRfloat (0.0f));
            iirHP[i][ch]->reset (IIRfloat (0.0f));
            iirHP2[i][ch]->reset (IIRfloat (0.0f));
            iirAP[i][ch]->reset (IIRfloat (0.0f));
            iirLP[i][ch]->prepare (monoSpec);
            iirLP2[i][ch]->prepare (monoSpec);
            iirHP[i][ch]->prepare (monoSpec);
            iirHP2[i][ch]->prepare (monoSpec);
            iirAP[i][ch]->prepare (monoSpec);
        }
    }

    for (int i = 0; i < numFilterBands; ++i)
    {
        compressors[i].prepare (monoSpec);
        compressors[i].setThreshold (*threshold[i]);
        compressors[i].setKnee (*knee[i]);
        compressors[i].setAttackTime (*attack[i] * 0.001f);
        compressors[i].setReleaseTime (*release[i] * 0.001f);
        compressors[i].setRatio (*ratio[i] > 15.9f ? INFINITY :  ratio[i]->load());
        compressors[i].setMakeUpGain (*makeUpGain[i]);

        freqBands[i].clear();
        for (int ch = 0; ch < maxNumFilters; ++ch)
        {
            freqBands[i].add (new juce::dsp::AudioBlock<IIRfloat> (freqBandsBlocks[i][ch], 1, samplesPerBlock));
        }
    }

    zero = juce::dsp::AudioBlock<float> (zeroData, IIRfloat_elements, samplesPerBlock);
    zero.clear();

    gains = juce::dsp::AudioBlock<float> (gainData, 1, samplesPerBlock);
    gains.clear();

    tempBuffer.setSize (64, samplesPerBlock);

    repaintFilterVisualization = true;
}

void MultiBandCompressorAudioProcessor::releaseResources()
{
    // When playback stops, you can use this as an opportunity to free up any
    // spare memory, etc.
}

#ifndef JucePlugin_PreferredChannelConfigurations
bool MultiBandCompressorAudioProcessor::isBusesLayoutSupported (const BusesLayout& layouts) const
{
    return true;
}
#endif

void MultiBandCompressorAudioProcessor::processBlock (juce::AudioSampleBuffer& buffer, juce::MidiBuffer& midiMessages)
{
    checkInputAndOutput (this, *orderSetting, *orderSetting, false);
    juce::ScopedNoDenormals noDenormals;

    const int maxNChIn = juce::jmin (buffer.getNumChannels(), input.getNumberOfChannels());
    if (maxNChIn < 1)
        return;
    
    

    /*for (int i = maxNChIn; i < getTotalNumOutputChannels(); ++i)
    {
        buffer.clear (i, 0, buffer.getNumSamples());
    }*/

    const int L = buffer.getNumSamples();
    const int nSIMDFilters =  1 + (maxNChIn - 1) / IIRfloat_elements;
    gainChannelPointer = gains.getChannelPointer (0);

    //tempBuffer.clear();
    gains.clear();
    zero.clear();

    // update iir filter coefficients
    if (userChangedFilterSettings.get())
        copyCoeffsToProcessor();

    inputPeak = juce::Decibels::gainToDecibels (buffer.getMagnitude (0, 0, L));
    
    using Format = juce::AudioData::Format<juce::AudioData::Float32, juce::AudioData::NativeEndian>;

    //interleave input data
    int partial = maxNChIn % IIRfloat_elements;
    if (partial == 0)
    {
        for (int i = 0; i<nSIMDFilters; ++i)
        {
            juce::AudioData::interleaveSamples (juce::AudioData::NonInterleavedSource<Format> {buffer.getArrayOfReadPointers(), IIRfloat_elements},
                                                juce::AudioData::InterleavedDest<Format> {reinterpret_cast<float*>(interleavedData[i]->getChannelPointer (0)), IIRfloat_elements},
                                                L);
        }
    }
    else
    {
        int i;
        for (i = 0; i<nSIMDFilters-1; ++i)
        {
            juce::AudioData::interleaveSamples (juce::AudioData::NonInterleavedSource<Format> {buffer.getArrayOfReadPointers(), IIRfloat_elements},
                                                juce::AudioData::InterleavedDest<Format> {reinterpret_cast<float*>(interleavedData[i]->getChannelPointer (0)), IIRfloat_elements},
                                                L);
        }

        const float* addr[IIRfloat_elements];
        int ch;
        for (ch = 0; ch < partial; ++ch)
        {
            addr[ch] = buffer.getReadPointer (i * IIRfloat_elements + ch);
        }
        for (; ch < IIRfloat_elements; ++ch)
        {
            addr[ch] = zero.getChannelPointer(ch);
        }
        juce::AudioData::interleaveSamples (juce::AudioData::NonInterleavedSource<Format> {addr, IIRfloat_elements},
                                            juce::AudioData::InterleavedDest<Format> {reinterpret_cast<float*>(interleavedData[i]->getChannelPointer (0)), IIRfloat_elements},
                                            L);
    }


    //  filter block diagram
    //                                | ---> HP3 ---> |
    //        | ---> HP2 ---> AP1 --->|               + ---> |
    //        |                       | ---> LP3 ---> |      |
    //     -->|                                              + --->
    //        |                       | ---> HP1 ---> |      |
    //        | ---> LP2 ---> AP3 --->|               + ---> |
    //                                | ---> LP1 ---> |
    for (int i = 0; i < nSIMDFilters; ++i)
    {
        const IIRfloat* chPtrinterleavedData[1] = {interleavedData[i]->getChannelPointer (0)};
        juce::dsp::AudioBlock<IIRfloat> abInterleaved (const_cast<IIRfloat**> (chPtrinterleavedData), 1, L);

        const IIRfloat* chPtrLow[1] = {freqBands[0][i]->getChannelPointer (0)};
        juce::dsp::AudioBlock<IIRfloat> abLow (const_cast<IIRfloat**> (chPtrLow), 1, L);

        const IIRfloat* chPtrMidLow[1] = {freqBands[1][i]->getChannelPointer (0)};
        juce::dsp::AudioBlock<IIRfloat> abMidLow (const_cast<IIRfloat**> (chPtrMidLow), 1, L);

        const IIRfloat* chPtrMidHigh[1] = {freqBands[2][i]->getChannelPointer (0)};
        juce::dsp::AudioBlock<IIRfloat> abMidHigh (const_cast<IIRfloat**> (chPtrMidHigh), 1, L);

        const IIRfloat* chPtrHigh[1] = {freqBands[3][i]->getChannelPointer (0)};
        juce::dsp::AudioBlock<IIRfloat> abHigh (const_cast<IIRfloat**> (chPtrHigh), 1, L);


        iirLP[1][i]->process (juce::dsp::ProcessContextNonReplacing<IIRfloat> (abInterleaved, abLow));
        iirHP[1][i]->process (juce::dsp::ProcessContextNonReplacing<IIRfloat> (abInterleaved, abHigh));

        iirLP2[1][i]->process (juce::dsp::ProcessContextReplacing<IIRfloat> (abLow));
        iirHP2[1][i]->process (juce::dsp::ProcessContextReplacing<IIRfloat> (abHigh));

        iirAP[2][i]->process (juce::dsp::ProcessContextReplacing<IIRfloat> (abLow));
        iirAP[0][i]->process (juce::dsp::ProcessContextReplacing<IIRfloat> (abHigh));

        iirHP[0][i]->process (juce::dsp::ProcessContextNonReplacing<IIRfloat> (abLow, abMidLow));
        iirHP2[0][i]->process (juce::dsp::ProcessContextReplacing<IIRfloat> (abMidLow));

        iirLP[0][i]->process (juce::dsp::ProcessContextReplacing<IIRfloat> (abLow));
        iirLP2[0][i]->process (juce::dsp::ProcessContextReplacing<IIRfloat> (abLow));

        iirLP[2][i]->process (juce::dsp::ProcessContextNonReplacing<IIRfloat> (abHigh, abMidHigh));
        iirLP2[2][i]->process (juce::dsp::ProcessContextReplacing<IIRfloat> (abMidHigh));

        iirHP[2][i]->process (juce::dsp::ProcessContextReplacing<IIRfloat> (abHigh));
        iirHP2[2][i]->process (juce::dsp::ProcessContextReplacing<IIRfloat> (abHigh));
    }

    buffer.clear();

    for (int i = 0; i < numFilterBands; ++i)
    {
        if (! soloArray.isZero())
        {
            if (! soloArray[i])
            {
                maxGR[i] = 0.0f;
                maxPeak[i] = -INFINITY;
                continue;
            }
        }

        // Deinterleave
        if (partial == 0)
        {
            for (int n = 0; n < nSIMDFilters; ++n)
            {
                juce::AudioData::deinterleaveSamples (juce::AudioData::InterleavedSource<Format> {reinterpret_cast<float*>(freqBands[i][n]->getChannelPointer (0)), IIRfloat_elements},
                                                      juce::AudioData::NonInterleavedDest<Format> {tempBuffer.getArrayOfWritePointers(), IIRfloat_elements},
                                                      L);
            }
        }
        else
        {
            int n;
            for (n = 0; n < nSIMDFilters-1; ++n)
            {
                juce::AudioData::deinterleaveSamples (juce::AudioData::InterleavedSource<Format> {reinterpret_cast<float*>(freqBands[i][n]->getChannelPointer (0)), IIRfloat_elements},
                                                      juce::AudioData::NonInterleavedDest<Format> {tempBuffer.getArrayOfWritePointers(), IIRfloat_elements},
                                                      L);
            }

            float* addr[IIRfloat_elements];
            int ch;
            for (ch = 0; ch < partial; ++ch)
            {
                addr[ch] = tempBuffer.getWritePointer (n * IIRfloat_elements + ch);
            }
            for (; ch < IIRfloat_elements; ++ch)
            {
                addr[ch] = zero.getChannelPointer (ch);
            }
            juce::AudioData::deinterleaveSamples (juce::AudioData::InterleavedSource<Format> {reinterpret_cast<float*>(freqBands[i][n]->getChannelPointer (0)), IIRfloat_elements},
                                                  juce::AudioData::NonInterleavedDest<Format> {addr, IIRfloat_elements},
                                                  L);
            zero.clear();
        }

        // Compress
        if (*bypass[i] < 0.5f)
        {
            compressors[i].getGainFromSidechainSignal (tempBuffer.getReadPointer (0), gainChannelPointer, L);
            maxGR[i] = juce::Decibels::gainToDecibels (juce::FloatVectorOperations::findMinimum (gainChannelPointer, L)) - *makeUpGain[i];
            maxPeak[i] = compressors[i].getMaxLevelInDecibels();

            for (int ch = 0; ch < maxNChIn; ++ch)
            {
                juce::FloatVectorOperations::addWithMultiply (buffer.getWritePointer (ch), tempBuffer.getReadPointer (ch), gainChannelPointer, L);
            }
        }
        else
        {
            for (int ch = 0; ch < maxNChIn; ++ch)
            {
                juce::FloatVectorOperations::add (buffer.getWritePointer (ch), tempBuffer.getReadPointer (ch), L);
            }
            maxGR[i] = 0.0f;
            maxPeak[i] = juce::Decibels::gainToDecibels (-INFINITY);
        }
        
        tempBuffer.clear();
    }

    outputPeak = juce::Decibels::gainToDecibels (buffer.getMagnitude (0, 0, L));
}

//==============================================================================
bool MultiBandCompressorAudioProcessor::hasEditor() const
{
    return true; // (change this to false if you choose to not supply an editor)
}

juce::AudioProcessorEditor* MultiBandCompressorAudioProcessor::createEditor()
{
    return new MultiBandCompressorAudioProcessorEditor (*this, parameters);
}

//==============================================================================
void MultiBandCompressorAudioProcessor::getStateInformation (juce::MemoryBlock& destData)
{
  auto state = parameters.copyState();

  auto oscConfig = state.getOrCreateChildWithName ("OSCConfig", nullptr);
  oscConfig.copyPropertiesFrom (oscParameterInterface.getConfig(), nullptr);

  std::unique_ptr<juce::XmlElement> xml (state.createXml());
  copyXmlToBinary (*xml, destData);
}


void MultiBandCompressorAudioProcessor::setStateInformation (const void* data, int sizeInBytes)
{
    std::unique_ptr<juce::XmlElement> xmlState (getXmlFromBinary (data, sizeInBytes));
    if (xmlState.get() != nullptr)
        if (xmlState->hasTagName (parameters.state.getType()))
        {
            parameters.replaceState (juce::ValueTree::fromXml (*xmlState));
            if (parameters.state.hasProperty ("OSCPort")) // legacy
            {
                oscParameterInterface.getOSCReceiver().connect (parameters.state.getProperty ("OSCPort", juce::var (-1)));
                parameters.state.removeProperty ("OSCPort", nullptr);
            }

            auto oscConfig = parameters.state.getChildWithName ("OSCConfig");
            if (oscConfig.isValid())
                oscParameterInterface.setConfig (oscConfig);
        }
}

//==============================================================================
void MultiBandCompressorAudioProcessor::parameterChanged (const juce::String &parameterID, float newValue)
{
    DBG ("Parameter with ID " << parameterID << " has changed. New value: " << newValue);

    if (parameterID.startsWith ("crossover"))
    {
        calculateCoefficients (parameterID.getLastCharacters (1).getIntValue());
        userChangedFilterSettings = true;
        repaintFilterVisualization = true;
    }
    else if (parameterID.startsWith ("threshold"))
    {
        const int compId = parameterID.getLastCharacters (1).getIntValue();
        compressors[compId].setThreshold (newValue);
        characteristicHasChanged[compId] = true;
    }
    else if (parameterID.startsWith ("knee"))
    {
        const int compId = parameterID.getLastCharacters (1).getIntValue();
        compressors[compId].setKnee (newValue);
        characteristicHasChanged[compId] = true;
    }
    else if (parameterID.startsWith ("attack"))
    {
        compressors[parameterID.getLastCharacters (1).getIntValue()].setAttackTime (newValue * 0.001f);
    }
    else if (parameterID.startsWith ("release"))
    {
        compressors[parameterID.getLastCharacters (1).getIntValue()].setReleaseTime (newValue * 0.001f);
    }
    else if (parameterID.startsWith ("ratio"))
    {
        const int compId = parameterID.getLastCharacters (1).getIntValue();
        if (newValue > 15.9f)
            compressors[compId].setRatio (INFINITY);
        else
            compressors[compId].setRatio (newValue);

        characteristicHasChanged[compId] = true;
    }
    else if (parameterID.startsWith ("makeUpGain"))
    {
        const int compId = parameterID.getLastCharacters (1).getIntValue();
        compressors[compId].setMakeUpGain (newValue);
        characteristicHasChanged[compId] = true;
    }
    else if (parameterID.startsWith ("solo"))
    {
        if (newValue >= 0.5f)
            soloArray.setBit (parameterID.getLastCharacters (1).getIntValue());
        else
            soloArray.clearBit (parameterID.getLastCharacters (1).getIntValue());
    }
    else if (parameterID == "orderSetting")
    {
        userChangedIOSettings = true;
    }

}

void MultiBandCompressorAudioProcessor::updateBuffers()
{
    DBG ("IOHelper:  input size: " << input.getSize());
    DBG ("IOHelper: output size: " << output.getSize());
}

//==============================================================================
// This creates new instances of the plugin..
juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new MultiBandCompressorAudioProcessor();
}

inline void MultiBandCompressorAudioProcessor::clear (juce::dsp::AudioBlock<IIRfloat>& ab)
{
    const int N = static_cast<int> (ab.getNumSamples()) * IIRfloat_elements;
    const int nCh = static_cast<int> (ab.getNumChannels());

    for (int ch = 0; ch < nCh; ++ch)
        juce::FloatVectorOperations::clear (reinterpret_cast<float*> (ab.getChannelPointer (ch)), N);
}
