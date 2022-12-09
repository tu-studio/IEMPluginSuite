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
#include "../../resources/AudioProcessorBase.h"

#include "../../resources/FilterVisualizerHelper.h"
#include "../../resources/Compressor.h"

#define ProcessorClass MultiBandCompressorAudioProcessor

using namespace juce::dsp;
using ParameterLayout = juce::AudioProcessorValueTreeState::ParameterLayout;


class MultiBandCompressorAudioProcessor  : public AudioProcessorBase<IOTypes::Ambisonics<>, IOTypes:: Ambisonics<>>
{
public:
    constexpr static int numberOfInputChannels = 64;
    constexpr static int numberOfOutputChannels = 64;
    //==============================================================================
    MultiBandCompressorAudioProcessor();
    ~MultiBandCompressorAudioProcessor();

    //==============================================================================
    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override;

   #ifndef JucePlugin_PreferredChannelConfigurations
    bool isBusesLayoutSupported (const BusesLayout& layouts) const override;
   #endif

    void processBlock (juce::AudioSampleBuffer&, juce::MidiBuffer&) override;

    //==============================================================================
    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override;

    //==============================================================================
    int getNumPrograms() override;
    int getCurrentProgram() override;
    void setCurrentProgram (int index) override;
    const juce::String getProgramName (int index) override;
    void changeProgramName (int index, const juce::String& newName) override;

    //==============================================================================
    void getStateInformation (juce::MemoryBlock& destData) override;
    void setStateInformation (const void* data, int sizeInBytes) override;

    //==============================================================================
    void parameterChanged (const juce::String &parameterID, float newValue) override;
    void updateBuffers() override;


    //======= Parameters ===========================================================
    std::vector<std::unique_ptr<juce::RangedAudioParameter>> createParameterLayout();


    #if JUCE_USE_SIMD
      using filterFloatType = SIMDRegister<float>;
      static constexpr int IIRfloat_elements = SIMDRegister<float>::size();
    #else /* !JUCE_USE_SIMD */
      using filterFloatType = float;
      static constexpr int IIRfloat_elements = 1;
    #endif

    static constexpr int numFreqBands {4};

    enum FrequencyBands
    {
        Low, MidLow, MidHigh, High
    };


    // Interface for gui
    double& getSampleRate() { return lastSampleRate; };
    IIR::Coefficients<double>::Ptr lowPassLRCoeffs[numFreqBands-1];
    IIR::Coefficients<double>::Ptr highPassLRCoeffs[numFreqBands-1];

    juce::Atomic<bool> repaintFilterVisualization = false;
    juce::Atomic<float> inputPeak = juce::Decibels::gainToDecibels (-INFINITY), outputPeak = juce::Decibels::gainToDecibels (-INFINITY);
    juce::Atomic<float> maxGR[numFreqBands], maxPeak[numFreqBands];

    juce::Atomic<bool> characteristicHasChanged[numFreqBands];


    iem::Compressor* getCompressor (const int i) {return &compressors[i];};

private:
    void calculateCoefficients (const int index);
    void copyCoeffsToProcessor();

    inline void clear (juce::dsp::AudioBlock<filterFloatType>& ab);

    double lastSampleRate {48000};
    const int maxNumFilters;


    // list of used audio parameters
    std::atomic<float>* orderSetting;
    std::atomic<float>* crossovers[numFreqBands-1];
    std::atomic<float>* threshold[numFreqBands];
    std::atomic<float>* knee[numFreqBands];
    std::atomic<float>* makeUpGain[numFreqBands];
    std::atomic<float>* ratio[numFreqBands];
    std::atomic<float>* attack[numFreqBands];
    std::atomic<float>* release[numFreqBands];
    std::atomic<float>* bypass[numFreqBands];

    juce::BigInteger soloArray;

    iem::Compressor compressors[numFreqBands];

    // filter coefficients
    juce::dsp::IIR::Coefficients<float>::Ptr iirLPCoefficients[numFreqBands-1], iirHPCoefficients[numFreqBands-1],
                                  iirAPCoefficients[numFreqBands-1],
                                  iirTempLPCoefficients[numFreqBands-1],
                                  iirTempHPCoefficients[numFreqBands-1], iirTempAPCoefficients[numFreqBands-1];

    // filters (cascaded butterworth/linkwitz-riley filters + allpass)
    juce::OwnedArray<IIR::Filter<filterFloatType>> iirLP[numFreqBands-1], iirHP[numFreqBands-1],
                                             iirLP2[numFreqBands-1], iirHP2[numFreqBands-1],
                                             iirAP[numFreqBands-1];

    juce::OwnedArray<juce::dsp::AudioBlock<filterFloatType>> interleavedData, freqBands[numFreqBands];
    juce::dsp::AudioBlock<float> zero, temp, gains;
    juce::AudioBuffer<float> tempBuffer;
    float* gainChannelPointer;

    std::vector<juce::HeapBlock<char>> interleavedBlockData,  freqBandsBlocks[numFreqBands];
    juce::HeapBlock<char> zeroData, tempData, gainData;
    juce::HeapBlock<const float*> channelPointers { 64 };

    juce::Atomic<bool> userChangedFilterSettings = true;

    //==============================================================================
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (MultiBandCompressorAudioProcessor)
};
