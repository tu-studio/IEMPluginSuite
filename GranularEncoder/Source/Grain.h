class Grain 
{
public:
	Grain(int startIndexCircularBuffer, int grainLengthSamples);

private:
	int _startIndexCircularBuffer;
	int _grainLengthSamples;
	int _currentIndex;
	bool _isActive;
};