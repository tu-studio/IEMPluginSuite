#include "JuceHeader.h"

class Grain 
{
public:
	//Grain(int startIndexCircularBuffer, int grainLengthSamples);
	Grain();

	void setBlockSize(int numSampOutBuffer);
	void startGrain(int startIndexCircularBuffer, int grainLengthSamples, int startOffset, float* channelWeights);
	void processBlock(juce::AudioBuffer<float>& buffer, juce::AudioBuffer<float>& circularBuffer, int numSampOutBuffer, int numSampCircBuffer, float *channelWeights, float mix, float gainFactor);
	void processSample(juce::AudioBuffer<float>& buffer, const float* circularLeftChannel, const float* circularRightChannel, int numSampCircBuffer, float *channelWeights, float mix, float gainFactor, int bufferIndex);

	bool isActive() const;

private:
	int _startIndexCircularBuffer;
	int _grainLengthSamples;
	int _currentIndex;
	int _startOffset;
	bool _isActive;
	int _blockCounter;
	juce::AudioBuffer<float> _outputBuffer;
	float* _channelWeights;
};