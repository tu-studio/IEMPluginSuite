#include "Grain.h"

Grain::Grain()
{
	_isActive = false;
	_currentIndex = 0;
	_blockCounter = 0;
}

void Grain::setBlockSize(int numSampOutBuffer)
{
	_outputBuffer.setSize(1, numSampOutBuffer);
	_outputBuffer.clear();
}

void Grain::startGrain(const GrainJobParameters &grainParameters) // Type envelopeType)
{
	_params = grainParameters;

	_isActive = true;
	_currentIndex = 0;
	_blockCounter = 0;

	_outputBuffer.clear();
}

void Grain::processBlock(juce::AudioBuffer<float> &buffer, juce::AudioBuffer<float> &circularBuffer)
{
	if (!_isActive)
		return;

	int numSampCircBuffer = circularBuffer.getNumSamples();
	int numSampOutBuffer = buffer.getNumSamples();
	jassert(_outputBuffer.getNumSamples() == numSampOutBuffer);

	_outputBuffer.clear();
	int nChOut = buffer.getNumChannels();

	const float *circularLeftChannel = circularBuffer.getReadPointer(0);
	const float *circularRightChannel = circularBuffer.getReadPointer(1);

	const float *window_ptr = _params.windowBuffer->getReadPointer(0);
	const int windowNumSamples = _params.windowBuffer->getNumSamples();

	float *outputBufferWritePtr = _outputBuffer.getWritePointer(0);

	int outStart;
	if (_blockCounter == 0)
	{
		outStart = _params.startOffsetInBlock;
	}
	else
	{
		outStart = 0;
	}

	// Write samples to internal mono buffer
	for (int i = outStart; i < numSampOutBuffer; i++)
	{
		if (_currentIndex < _params.grainLengthSamples) // grain still needs samples
		{
			// Linear interpolation of buffer samples 
			float readIndex = _params.startPositionCircBuffer + (_currentIndex * _params.pitchReadFactor);
			int readIndexInt = static_cast<int>(readIndex);
			int readIndexIntNext = readIndexInt + 1; 
			float sampleFracWeight = readIndex - readIndexInt;
			if (readIndexInt >= numSampCircBuffer)
				readIndexInt = readIndexInt - numSampCircBuffer;
			if (readIndexIntNext >= numSampCircBuffer)
				readIndexIntNext = readIndexIntNext - numSampCircBuffer;
			float sampleIntPart = circularLeftChannel[readIndexInt];
			float sampleFracPart = circularLeftChannel[readIndexIntNext] - sampleIntPart;
			float sampleValue = sampleIntPart + sampleFracWeight * sampleFracPart;

			// Linear interpolation for grain window function
			float windowIndex = static_cast<float>(_currentIndex) / static_cast<float>(_params.grainLengthSamples) * (windowNumSamples-1);
			int windowIndexInt = static_cast<int>(windowIndex);
			jassert(windowIndexInt >= 0 && windowIndexInt < (windowNumSamples-1));

			int windowIndexIntNext = windowIndexInt + 1;//windowIndexInt < 1023 ? (windowIndexInt + 1) : windowIndexInt;
			float windowFracWeight = windowIndex - windowIndexInt;
			float windowIntPart = window_ptr[windowIndexInt];
			float windFracPart = window_ptr[windowIndexIntNext] - windowIntPart;
			float windowValue = windowIntPart + windowFracWeight * windFracPart;
		    jassert(windowValue >= 0.0f && windowValue <= 1.0f);
			outputBufferWritePtr[i] = sampleValue * windowValue;

			_currentIndex++;
		}
		else
		{
			_isActive = false;
			break;
		}
	}

	const float *outputBufferReadPtr = _outputBuffer.getReadPointer(0);
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
