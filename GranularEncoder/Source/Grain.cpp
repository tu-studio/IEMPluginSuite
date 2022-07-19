#include "Grain.h"

/*
Grain::Grain(int startIndexCircularBuffer, int grainLengthSamples)
{
	_startIndexCircularBuffer = startIndexCircularBuffer;
	_grainLengthSamples = grainLengthSamples;
	_isActive = false;
	_currentIndex = 0;
};*/

Grain::Grain()
{
	_startIndexCircularBuffer = 0;
	_grainLengthSamples = 0;
	_isActive = false;
	_currentIndex = 0;
	_startOffset = 0;
}

void Grain::startGrain(int startIndexCircularBuffer, int grainLengthSamples, int startOffset) // Type envelopeType)
{
	_startIndexCircularBuffer = startIndexCircularBuffer;
	_grainLengthSamples = grainLengthSamples;
	_isActive = true;
	_startOffset = startOffset;
	_currentIndex = 0;
	// preRenderEnvelope(grainLengthSamples, Type envelopeType);
}

void Grain::processBlock(juce::AudioBuffer<float>& buffer, juce::AudioBuffer<float>& circularBuffer, int numSampOutBuffer, int numSampCircBuffer, float *channelWeights, float mix)
{
	if (!_isActive)
		return;
	const float* circularLeftChannel = circularBuffer.getReadPointer(0);
	const float* circularRightChannel = circularBuffer.getReadPointer(1);

	int nChOut = buffer.getNumChannels();

	int outStart = (_currentIndex > (numSampOutBuffer - _startOffset)) ? 0 : _startOffset;
	for (int i = outStart; i < numSampOutBuffer; i++)
	{
		if (_currentIndex < _grainLengthSamples) // grain still needs samples
		{
			for (int ch = 0; ch < nChOut; ++ch) 
			{
				int readIndex = (_startIndexCircularBuffer + _currentIndex) % numSampCircBuffer;
				buffer.addSample(ch, i, circularLeftChannel[readIndex] * channelWeights[ch] * mix);
			}
			_currentIndex++;
		}
		else
		{
			_isActive = false;
			_currentIndex = 0;
			return;
		}
	}
};

bool Grain::isActive() const {
	return _isActive;
};