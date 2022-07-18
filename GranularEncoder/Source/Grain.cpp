#include "Grain.h"

Grain::Grain(int startIndexCircularBuffer, int grainLengthSamples)
{
	_startIndexCircularBuffer = startIndexCircularBuffer;
	_grainLengthSamples = grainLengthSamples;
	_isActive = true;
	_currentIndex = 0;
};