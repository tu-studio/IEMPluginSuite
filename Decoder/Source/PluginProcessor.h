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
#include "../../resources/IOHelper.h"
#include "../../resources/NewtonApple/NewtonApple_hull3D.h"
#include "tDesign5200.h"
#include "../../resources/efficientSHvanilla.h"
#include "../../resources/ReferenceCountedDecoder.h"
#include "../../resources/AmbisonicDecoder.h"

//==============================================================================
/**
 Use the IOHelper to detect which amount of channels or which Ambisonic order is possible with the current bus layout.
 The available IOTypes are:
    - AudioChannels<maxChannelCount>
    - Ambisonics<maxOrder> (can also be used for directivity signals)
 You can leave `maxChannelCount` and `maxOrder` empty for default values (64 channels and 7th order)
*/

using namespace dsp;
class PluginTemplateAudioProcessor  : public AudioProcessor,
                                        public AudioProcessorValueTreeState::Listener,
                                        public IOHelper<IOTypes::Ambisonics<7>, IOTypes::AudioChannels<64>>,
                                        public ValueTree::Listener
{
public:
    //==============================================================================
    PluginTemplateAudioProcessor();
    ~PluginTemplateAudioProcessor();

    //==============================================================================
    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override;

   #ifndef JucePlugin_PreferredChannelConfigurations
    bool isBusesLayoutSupported (const BusesLayout& layouts) const override;
   #endif

    void processBlock (AudioSampleBuffer&, MidiBuffer&) override;

    //==============================================================================
    AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override;

    //==============================================================================
    const String getName() const override;

    bool acceptsMidi() const override;
    bool producesMidi() const override;
    bool isMidiEffect () const override;
    double getTailLengthSeconds() const override;

    //==============================================================================
    int getNumPrograms() override;
    int getCurrentProgram() override;
    void setCurrentProgram (int index) override;
    const String getProgramName (int index) override;
    void changeProgramName (int index, const String& newName) override;

    //==============================================================================
    void getStateInformation (MemoryBlock& destData) override;
    void setStateInformation (const void* data, int sizeInBytes) override;

    //==============================================================================
    void parameterChanged (const String &parameterID, float newValue) override;
    void updateBuffers() override; // use this to implement a buffer update method
    
    //==============================================================================
    void valueTreePropertyChanged (ValueTree &treeWhosePropertyHasChanged, const Identifier &property) override;
    void valueTreeChildAdded (ValueTree &parentTree, ValueTree &childWhichHasBeenAdded) override;
    void valueTreeChildRemoved (ValueTree &parentTree, ValueTree &childWhichHasBeenRemoved, int indexFromWhichChildWasRemoved) override;
    void valueTreeChildOrderChanged (ValueTree &parentTreeWhoseChildrenHaveMoved, int oldIndex, int newIndex) override;
    void valueTreeParentChanged (ValueTree &treeWhoseParentHasChanged) override;
    
    void addRandomPoint();
    
    void undo() { undoManager.undo(); }
    void redo() { undoManager.redo(); }
    
    ValueTree& getLoudspeakersValueTree() { return loudspeakers; }
    
    var lsps;
    Atomic<bool> updateLoudspeakerVisualization = false;
    Atomic<bool> updateTable = true;
    
    std::vector<R3> points;
    std::vector<Tri> triangles;
    std::vector<Vector3D<float>> normals;
    
    BigInteger imaginaryFlags;
    UndoManager undoManager;
    
    Result calculateDecoder();
    
private:
    // ====== parameters
    AudioProcessorValueTreeState parameters;
    // list of used audio parameters
    float* inputOrderSetting;
    float* useSN3D;
    
    ValueTree loudspeakers {"Loudspeakers"};
    
    AmbisonicDecoder decoder;
    ReferenceCountedDecoder::Ptr decoderConfig {nullptr};
    
    bool isLayoutReady = false;
    
    // ========== METHODS
    void prepareLayout();
    Result checkLayout();
    Result verifyLoudspeakers();
    Result calculateTris();
    void convertLoudspeakersToArray();
    
    float getKappa(float gIm, float gRe1, float gRe2, int N);
    Matrix<float> getInverse(Matrix<float> A);
    
    ValueTree createLoudspeakerFromCartesian (Vector3D<float> cartesianCoordinates, int channel, bool isVirtual = false, float gain = 1.0f);
    ValueTree createLoudspeakerFromSpherical (Vector3D<float> sphericalCoordinates, int channel, bool isVirtual = false, float gain = 1.0f);
    Vector3D<float> cartesianToSpherical (Vector3D<float> cartvect);
    Vector3D<float> sphericalToCartesian (Vector3D<float> sphervect);
    //==============================================================================
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (PluginTemplateAudioProcessor)
};
