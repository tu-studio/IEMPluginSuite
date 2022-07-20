#include "JuceHeader.h"

class Grain 
{
public:
	//Grain(int startIndexCircularBuffer, int grainLengthSamples);
	Grain();

	void startGrain(int startIndexCircularBuffer, int grainLengthSamples, int startOffset);
	void processBlock(juce::AudioBuffer<float>& buffer, juce::AudioBuffer<float>& circularBuffer, int numSampOutBuffer, int numSampCircBuffer, float *channelWeights, float mix);

	bool isActive() const;

private:
	int _startIndexCircularBuffer;
	int _grainLengthSamples;
	int _currentIndex;
	int _startOffset;
	bool _isActive;
	int _blockCounter;
};