#include "JuceHeader.h"

class Grain
{
public:
	struct GrainJobParameters
	{
		int startPositionCircBuffer = 0;
		int startOffsetInBlock = 0;
		int grainLengthSamples = 0;
		float pitchReadFactor = 1.0f;
		std::array<float, 64> channelWeights;
		float gainFactor = 1.0f;
		float mix = 1.0f;
		juce::AudioBuffer<float> windowBuffer;
	};

	Grain();

	void setBlockSize(int numSampOutBuffer);
	void startGrain(const GrainJobParameters &grainParameters);
	void processBlock(juce::AudioBuffer<float> &buffer, juce::AudioBuffer<float> &circularBuffer);
	// void processSample(juce::AudioBuffer<float>& buffer, const float* circularLeftChannel, const float* circularRightChannel, int numSampCircBuffer, float *channelWeights, float mix, float gainFactor, int bufferIndex);

	bool isActive() const;

private:
	GrainJobParameters _params;
	int _currentIndex;
	bool _isActive;
	int _blockCounter;
	juce::AudioBuffer<float> _outputBuffer;
};