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

#define JUCE_USE_VDSP_FRAMEWORK 1

#pragma once

#include "../JuceLibraryCode/JuceHeader.h"
#include "../../resources/interpLagrangeWeights.h"
#include "../../resources/efficientSHvanilla.h"
#include "reflections.h"
#include "../../resources/ambisonicTools.h"
#include "../../resources/AudioProcessorBase.h"
#include "../../resources/customComponents/FilterVisualizer.h"


#ifdef JUCE_MAC
#define VIMAGE_H // avoid namespace clashes
#include <Accelerate/Accelerate.h>
#endif

#if JUCE_USE_SIMD
# define IIRfloat juce::dsp::SIMDRegister<float>
# define IIRfloat_elements() IIRfloat::size()
#else /* !JUCE_USE_SIMD */
# define IIRfloat float
# define IIRfloat_elements() 1
#endif /* JUCE_USE_SIMD */

const int mSig[] = {1,-1};
using namespace juce::dsp;

struct RoomParams {
    bool validRoomData = false;
    bool validListenerData = false;
    bool validReflectionData = false;
    float roomX, roomY, roomZ;
    float listenerX, listenerY, listenerZ;
    float reflCoeff, lowShelfFreq, lowShelfGain, highShelfFreq, highShelfGain, numRefl;
};

struct SharedParams {
    SharedParams()
    {
        rooms.add(RoomParams());
        rooms.add(RoomParams());
        rooms.add(RoomParams());
        rooms.add(RoomParams());
    }
    Array<RoomParams> rooms;
};

struct ReflectionProperty
{
    const int x; // coordinate in image source space
    const int y; // coordinate in image source space
    const int z; // coordinate in image source space
    const int order; // image source order
    const int xPlusReflections; // number of reflections at front wall
    const int xMinusReflections; // number of reflections at back wall
    const int yPlusReflections; // number of reflections at left wall
    const int yMinusReflections; // number of reflections at right wall
    const int zPlusReflections; // number of reflections at ceiling
    const int zMinusReflections; // number of reflections at floor
};

//==============================================================================
/**
*/
class RoomEncoderAudioProcessor  :  public AudioProcessorBase<IOTypes::Ambisonics<>, IOTypes::Ambisonics<>>,
                                    private Timer
{
public:
    //==============================================================================
    RoomEncoderAudioProcessor();
    ~RoomEncoderAudioProcessor();

    //==============================================================================

    void initializeReflectionList();
    
    //==============================================================================
    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override;
    
    void processBlock (AudioSampleBuffer&, MidiBuffer&) override;

    //==============================================================================
    AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override;

    //==============================================================================
    int getNumPrograms() override;
    int getCurrentProgram() override;
    void setCurrentProgram (int index) override;
    const String getProgramName (int index) override;
    void changeProgramName (int index, const String& newName) override;

    //==============================================================================
    void getStateInformation (MemoryBlock& destData) override;
    void setStateInformation (const void* data, int sizeInBytes) override;

    void parameterChanged (const String &parameterID, float newValue) override;


    //======= Parameters ===========================================================
    std::vector<std::unique_ptr<RangedAudioParameter>> createParameterLayout();

    //==============================================================================
    double oldDelay[nImgSrc];
    float allGains[nImgSrc];

    //filter coefficients
    IIR::Coefficients<float>::Ptr lowShelfCoefficients;
    IIR::Coefficients<float>::Ptr highShelfCoefficients;

    bool userChangedFilterSettings = true;
    bool updateFv = false;

    void timerCallback() override;

    void updateFilterCoefficients (double sampleRate);
    void calculateImageSourcePositions (const float t, const float b, const float h);

    float* numRefl;
    float mRadius[nImgSrc];

    void updateBuffers() override;

    Atomic<bool> repaintPositionPlanes = true;

private:
    //==============================================================================
    
    bool readingSharedParams = false;;

    double phi;
    double theta;

    // Parameters
    float *directivityOrderSetting;
    float *inputIsSN3D;
    float *orderSetting;
    float *useSN3D;

    float* roomX;
    float* roomY;
    float* roomZ;

    float* sourceX;
    float* sourceY;
    float* sourceZ;

    float* listenerX;
    float* listenerY;
    float* listenerZ;

    float* reflCoeff;

    float* lowShelfFreq;
    float* lowShelfGain;
    float* highShelfFreq;
    float* highShelfGain;

    float* syncChannel;
    float* syncRoomSize;
    float* syncReflection;
    float* syncListener;

    float* renderDirectPath;
    float* directPathZeroDelay;
    float* directPathUnityGain;

    float* wallAttenuationFront;
    float* wallAttenuationBack;
    float* wallAttenuationLeft;
    float* wallAttenuationRight;
    float* wallAttenuationCeiling;
    float* wallAttenuationFloor;

    int _numRefl;

    SharedResourcePointer<SharedParams> sharedParams;

    //SIMD IIR Filter
    OwnedArray<IIR::Filter<IIRfloat>> lowShelfArray;
    OwnedArray<IIR::Filter<IIRfloat>> highShelfArray;
    OwnedArray<IIR::Filter<IIRfloat>> lowShelfArray2;
    OwnedArray<IIR::Filter<IIRfloat>> highShelfArray2;
    HeapBlock<char> interleavedBlockData[16], zeroData; //todo: dynamically?
    OwnedArray<AudioBlock<IIRfloat>> interleavedData;
    AudioBlock<float> zero;


    Vector3D<float> sourcePos, listenerPos;
    float h,b,t;

    float mx[nImgSrc];
    float my[nImgSrc];
    float mz[nImgSrc];
    float smx[nImgSrc];
    float smy[nImgSrc];
    float smz[nImgSrc];


    float hypxy;
    int bufferSize;
    int bufferReadIdx;
    int overflow;
    int temp;


    int readOffset;

    float powReflCoeff[maxOrderImgSrc+1];
    double dist2smpls;

    float *tempAddr;
    float SHcoeffsOld[nImgSrc][64];
    IIRfloat SHsampleOld[nImgSrc][16]; //TODO: can be smaller: (N+1)^2/IIRfloat_elements()
    float weightedSample;

    AudioBuffer<float> delayBuffer;
    AudioBuffer<float> monoBuffer;

    OwnedArray<ReflectionProperty> reflectionList;
    
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (RoomEncoderAudioProcessor)
};
