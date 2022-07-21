#include "Grain.h"

Grain::Grain()
{
	_startIndexCircularBuffer = 0;
	_grainLengthSamples = 0;
	_isActive = false;
	_currentIndex = 0;
	_startOffset = 0;
	_blockCounter = 0;
}

void Grain::setBlockSize(int numSampOutBuffer) {
	_outputBuffer.setSize(1, numSampOutBuffer);
	_outputBuffer.clear();
}

void Grain::startGrain(int startIndexCircularBuffer, int grainLengthSamples, int startOffset, float* channelWeights) // Type envelopeType)
{
	_startIndexCircularBuffer = startIndexCircularBuffer;
	_grainLengthSamples = grainLengthSamples;
	_isActive = true;
	_startOffset = startOffset;
	_currentIndex = 0;
	_blockCounter = 0;
	_outputBuffer.clear();
	_channelWeights = channelWeights;
	// preRenderEnvelope(grainLengthSamples, Type envelopeType);
}

void Grain::processBlock(juce::AudioBuffer<float>& buffer, juce::AudioBuffer<float>& circularBuffer, int numSampOutBuffer, int numSampCircBuffer, float *channelWeights, float mix, float gainFactor)
{
	if (!_isActive)
		return;

	jassert(_outputBuffer.getNumSamples() == numSampOutBuffer);
	_outputBuffer.clear();

	const float* circularLeftChannel = circularBuffer.getReadPointer(0);
	const float* circularRightChannel = circularBuffer.getReadPointer(1);

	int nChOut = buffer.getNumChannels();

	int outStart;
	if(_blockCounter == 0)
	{
		outStart = _startOffset;
	}
	else
	{
		outStart = 0;
	}

	int readIndex;
	// Write samples to internal mono buffer
	for (int i = outStart; i < numSampOutBuffer; i++)
	{
		if (_currentIndex < _grainLengthSamples) // grain still needs samples
		{
			readIndex = (_startIndexCircularBuffer + _currentIndex) % numSampCircBuffer;
			_outputBuffer.setSample(0, i, circularLeftChannel[readIndex]);
			_currentIndex++;
		}
		else
		{
			_isActive = false;
			return;
		}
	}

	const float* grain_samples = _outputBuffer.getReadPointer(0);
	// SIMD Ambisonics encoding to current grain direction
	for (int ch = 0; ch < nChOut; ++ch)
	{
		buffer.addFrom(ch, 0, grain_samples, buffer.getNumSamples(), _channelWeights[ch] * mix * gainFactor);
	}

	_blockCounter++;
};

void Grain::processSample(juce::AudioBuffer<float>& buffer, const float* circularLeftChannel, const float* circularRightChannel, int numSampCircBuffer, float *channelWeights, float mix, float gainFactor, int bufferIndex)
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
};

bool Grain::isActive() const {
	return _isActive;
};