#include "JuceHeader.h"
#include "Type.h"

class Grain 
{
public:
	struct GrainJobParameters{
		int startPositionCircBuffer = 0;
		int startOffsetBlock = 0;
		int grainLengthSamples = 0;
		int pitchSemitones = 0;
		WindowType windowType;
		std::array<float,64> channelWeights;
		float gainFactor = 1.0f;
		float mix = 1.0f;
		//juce::Vector3D<float> grainDirection;
		//int tDesignDirectionIndex = 0;
	};

	//Grain(references to data for processing);
	Grain();

	void setBlockSize(int numSampOutBuffer);
	void startGrain(int startIndexCircularBuffer, int grainLengthSamples, int startOffset, float* channelWeights, float* windowFunction);
	void startGrain(const GrainJobParameters& grainParameters);
	void processBlock(juce::AudioBuffer<float>& buffer, juce::AudioBuffer<float>& circularBuffer, float mix, float gainFactor);
	void processBlock(juce::AudioBuffer<float>& buffer, juce::AudioBuffer<float>& circularBuffer);
	void processSample(juce::AudioBuffer<float>& buffer, const float* circularLeftChannel, const float* circularRightChannel, int numSampCircBuffer, float *channelWeights, float mix, float gainFactor, int bufferIndex);

	bool isActive() const;



private:
	int _startIndexCircularBuffer;
	int _grainLengthSamples;
	int _currentIndex;
	int _startOffset;
	float* _channelWeights;
	float* _window;

	GrainJobParameters _params;

	bool _isActive;
	int _blockCounter;
	juce::AudioBuffer<float> _outputBuffer;

	//WindowType _windowType;

};