#include "Grain.h"

Grain::Grain()
{

	_isActive = false;
	_currentIndex = 0;
	_blockCounter = 0;


	//_startOffset = 0;
	//_channelWeights = nullptr;
	//_window = nullptr;
}

void Grain::setBlockSize(int numSampOutBuffer) {
	_outputBuffer.setSize(1, numSampOutBuffer);
	_outputBuffer.clear();
}

void Grain::startGrain(const GrainJobParameters& grainParameters) // Type envelopeType)
{
	_params = grainParameters;

	_isActive = true;
	_currentIndex = 0;
	_blockCounter = 0;

	_outputBuffer.clear();
	// preRenderEnvelope(grainLengthSamples, Type envelopeType);
}

void Grain::processBlock(juce::AudioBuffer<float>& buffer, juce::AudioBuffer<float>& circularBuffer)
{
	if (!_isActive)
		return;

	int numSampCircBuffer = circularBuffer.getNumSamples();
	int numSampOutBuffer = buffer.getNumSamples();
	jassert(_outputBuffer.getNumSamples() == numSampOutBuffer);
	//jassert(_channelWeights != nullptr);
	//jassert(_window != nullptr);

	_outputBuffer.clear();
	int nChOut = buffer.getNumChannels();

	const float* circularLeftChannel = circularBuffer.getReadPointer(0);
	const float* circularRightChannel = circularBuffer.getReadPointer(1);

	const float* window_ptr = _params.windowBuffer->getReadPointer(0);
	const int windowNumSamples = _params.windowBuffer->getNumSamples();

	float* outputBufferWritePtr = _outputBuffer.getWritePointer(0);

	int outStart;
	if(_blockCounter == 0)
	{
		outStart = _params.startOffsetBlock;
	}
	else
	{
		outStart = 0;
	}

	int readIndex;
	// Write samples to internal mono buffer
	for (int i = outStart; i < numSampOutBuffer; i++)
	{
		if (_currentIndex < _params.grainLengthSamples) // grain still needs samples
		{
			readIndex = (_params.startPositionCircBuffer + _currentIndex) % numSampCircBuffer;
			outputBufferWritePtr[i] = circularLeftChannel[readIndex] * window_ptr[static_cast<int>((float)_currentIndex / _params.grainLengthSamples * windowNumSamples)];
			//_outputBuffer.setSample(0, i, circularLeftChannel[readIndex] * window_ptr[static_cast<int>((float) _currentIndex / _params.grainLengthSamples * window_size)]);
			_currentIndex++;
		}
		else
		{
			_isActive = false;
			return;
		}
	}

	const float* outputBufferReadPtr = _outputBuffer.getReadPointer(0);
	for (int ch = 0; ch < nChOut; ++ch)
	{
		buffer.addFrom(ch, 0, outputBufferReadPtr, buffer.getNumSamples(), _params.channelWeights[ch] * _params.mix * _params.gainFactor);
	}

	_blockCounter++;
};


bool Grain::isActive() const
{
	return _isActive;
};

/*void Grain::processSample(juce::AudioBuffer<float>& buffer, const float* circularLeftChannel, const float* circularRightChannel, int numSampCircBuffer, float *channelWeights, float mix, float gainFactor, int bufferIndex)
{
	if (!_isActive)
		return;

	int nChOut = buffer.getNumChannels();

	if (_currentIndex < _grainLengthSamples) // grain still needs a sample
	{
		int readIndex = (_startIndexCircularBuffer + _currentIndex) % numSampCircBuffer;
		
		for (int ch = 0; ch < nChOut; ++ch)
		{
			buffer.addSample(ch, bufferIndex, 1.0f*circularLeftChannel[readIndex] * channelWeights[ch] * mix * gainFactor);
		}
		_currentIndex++;
	}
	else
	{
		_isActive = false;
		return;
	}
};*/

