/*
 ==============================================================================
 This file is part of the IEM plug-in suite.
 Author: Daniel Rudrich / Markus Huber
 Copyright (c) 2017 - Institute of Electronic Music and Acoustics (IEM)
 https://iem.at

 The IEM plug-in suite is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 The IEM plug-in suite is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this software.  If not, see <https://www.gnu.org/licenses/>.
 ==============================================================================
 */


#pragma once
//#include "../JuceLibraryCode/JuceHeader.h"

template <typename coefficientsType>
class FilterBankVisualizer    : public Component
{
public:
    FilterBankVisualizer() : overallGainInDb (0.0f), sampleRate (48000.0)
    {
        init();
    };

    FilterBankVisualizer (float fMin, float fMax, float dbMin, float dbMax, float gridDiv) : overallGainInDb (0.0f), sampleRate (48000.0), s {fMin, fMax, dbMin, dbMax, gridDiv}
    {
        init();
    };

    ~FilterBankVisualizer() {};
  
    void init ()
    {
        dyn = s.dbMax - s.dbMin;
        zero = 2.0f * s.dbMax / dyn;
        scale = 1.0f / (zero + std::tanh (s.dbMin / dyn * -2.0f));
    }

    void paint (Graphics& g) override
    {
        g.setColour (Colours::steelblue.withMultipliedAlpha (0.01f));
        g.fillAll();

        g.setFont (getLookAndFeel().getTypefaceForFont (Font (12.0f, 2)));
        g.setFont (12.0f);

        // db labels
        float dyn = s.dbMax - s.dbMin;
        int numgridlines = dyn / s.gridDiv + 1;

        //g.setFont(Font(getLookAndFeel().getTypefaceForFont (Font(10.0f, 1)));
        g.setColour (Colours::white);
        int lastTextDrawPos = -1;
        for (int i = 0; i < numgridlines; ++i)
        {
            float db_val = s.dbMax - i * s.gridDiv;
            lastTextDrawPos = drawLevelMark (g, 0, 20, db_val, String (db_val, 0), lastTextDrawPos);
        }


        // frequency labels
        for (float f = s.fMin; f <= s.fMax; f += pow (10, floor (log10 (f)))) {
            int xpos = hzToX (f);

            String axislabel;
            bool drawText = false;

            if ((f == 20) || (f == 50) || (f == 100) || (f == 500))
            {
                axislabel = String (f, 0);
                drawText = true;
            }
            else if ((f == 1000) || (f == 5000) || (f == 10000) || (f == 20000))
            {
                axislabel = String(f / 1000, 0);
                axislabel << "k";
                drawText = true;
            }
            else
                continue;

            g.drawText (axislabel, xpos - 10, dbToY (s.dbMin) + OH + 0.0f, 20, 12, Justification::centred, false);
        }


        g.setColour (Colours::steelblue.withMultipliedAlpha (0.8f));
        g.strokePath (dbGridPath, PathStrokeType (0.5f));

        g.setColour (Colours::steelblue.withMultipliedAlpha (0.9f));
        g.strokePath (hzGridPathBold, PathStrokeType (1.0f));

        g.setColour (Colours::steelblue.withMultipliedAlpha (0.8f));
        g.strokePath (hzGridPath, PathStrokeType (0.5f));


        // draw filter magnitude responses
        Path magnitude;
        allMagnitudesInDb.fill (overallGainInDb);
        const int size = elements.size();

        const int xMin = hzToX (s.fMin);
        const int xMax = hzToX (s.fMax);
        const int yMin = jmax (dbToY (s.dbMax), 0);
        const int yMax = jmax (dbToY (s.dbMin), yMin);
        const int yZero = dbToY (0.0f);
      
        g.excludeClipRegion (Rectangle<int> (0.0f, yMax + OH, getWidth(), getHeight() - yMax - OH));


        // get values for first split (mid crossover), since we need them more than once
        const int firstLowSplitIndex = (size / 2) - 1;
        const int firstHighSplitIndex = firstLowSplitIndex + 1;
      
        auto firstLowSplitCoeffs = elements[firstLowSplitIndex]->coefficients;
        auto firstHighSplitCoeffs = elements[firstHighSplitIndex]->coefficients;
      
        Array<double> firstLowSplitMagnitude;
        Array<double> firstHighSplitMagnitude;
        firstLowSplitMagnitude.resize (numPixels);
        firstHighSplitMagnitude.resize (numPixels);
      
        if (firstLowSplitCoeffs != nullptr)
            firstLowSplitCoeffs->getMagnitudeForFrequencyArray (frequencies.getRawDataPointer(), firstLowSplitMagnitude.getRawDataPointer(), numPixels, sampleRate);
        if (firstHighSplitCoeffs != nullptr)
            firstHighSplitCoeffs->getMagnitudeForFrequencyArray (frequencies.getRawDataPointer(), firstHighSplitMagnitude.getRawDataPointer(), numPixels, sampleRate);


        // get x-axis values in-between crossovers, in order to display the filtered bands properly.
        // the magnitude responses of the filter are only drawn one-sided, starting (LoPass) or ending (HiPass) in between crossovers
        Array<int> bandStartStop;
        bandStartStop.add (xMin);
        for (int i = 1; i < numFreqBands - 1; ++i)
        {
            int xStart = elements[2*i-2]->frequencySlider == nullptr ? xMin : hzToX (elements[2*i-2]->frequencySlider->getValue ());
            int xStop = elements[2*i]->frequencySlider == nullptr ? xMin : hzToX (elements[2*i]->frequencySlider->getValue ());
            bandStartStop.add (xStart + ((xStop - xStart)/2));
        }
        bandStartStop.add (xMax);


        for (int i = 0; i < size; ++i)
        {
            magnitude.clear();
            auto handle (elements[i]);
          
            // get magnitude response of filter
            if (i != firstLowSplitIndex && i != firstHighSplitIndex)
            {
                typename dsp::IIR::Coefficients<coefficientsType>::Ptr coeffs = handle->coefficients;
                if (coeffs != nullptr)
                    coeffs->getMagnitudeForFrequencyArray (frequencies.getRawDataPointer(), magnitudes.getRawDataPointer(), numPixels, sampleRate);
            }
            else
            {
                magnitudes = i == firstLowSplitIndex ? firstLowSplitMagnitude : firstHighSplitMagnitude;
            }
          
            // get additional gain
            float additiveDB = 0.0f;
            if (handle->gainSlider != nullptr)
                additiveDB = handle->gainSlider->getValue();
          
            int xStart = xMin;
            int xStop = numPixels;

            if (!(i == elements.size()-1 || i == 0)) // lowest and highest freq band do not have to be split up, only go through a single filter
            {
                if ((i % 2) == 0) // Lowpass
                {
                    xStart = bandStartStop[i/2];
                }
                else // Hipass
                {
                    xStop = bandStartStop[(i/2) + 1] - xMin;
                }
            }

            // draw responses
            float db = Decibels::gainToDecibels (magnitudes[xStart - xMin]) + additiveDB;
            magnitude.startNewSubPath (xStart, jlimit (static_cast<float> (yMin), static_cast<float> (yMax) + OH + 1.0f, dbToYFloat (db)));
      
            for (int i = xStart + 1; i < xStart + xStop + 1; ++i)
            {
                db = Decibels::gainToDecibels (magnitudes[i - xMin]) + additiveDB;
                float y = jlimit (static_cast<float> (yMin), static_cast<float> (yMax) + OH + 1.0f, dbToYFloat (db));
                magnitude.lineTo (i, y);
            }
      
            g.setColour (handle->colour.withMultipliedAlpha (0.7f));
            g.strokePath (magnitude, PathStrokeType (1.0f));
            magnitude.lineTo (xStop + xMin, yMax + OH + 1.0f);
            magnitude.lineTo (xStart, yMax + OH + 1.0f);
            magnitude.closeSubPath();
            g.setColour (handle->colour.withMultipliedAlpha (0.3f));
            g.fillPath (magnitude);
          
          
            // accumulate responses
            if (i < firstLowSplitIndex)
            {
                FloatVectorOperations::multiply (magnitudes.getRawDataPointer(), firstLowSplitMagnitude.getRawDataPointer(), numPixels);
            }
            else if (i > firstHighSplitIndex)
            {
                FloatVectorOperations::multiply (magnitudes.getRawDataPointer(), firstHighSplitMagnitude.getRawDataPointer(), numPixels);
            }
            else
            {
                continue;
            }
          
            for (int n = 0; n < numPixels; ++n)
            {
                db = Decibels::gainToDecibels (magnitudes[n]) + additiveDB;
                allMagnitudesInDb.setUnchecked (n, Decibels::gainToDecibels (Decibels::decibelsToGain (allMagnitudesInDb[n]) + Decibels::decibelsToGain (db)));
            }
        }
      
        // draw summed magnitude
        magnitude.clear();
        magnitude.startNewSubPath (xMin, jlimit (static_cast<float> (yMin), static_cast<float> (yMax) + OH + 1.0f, dbToYFloat (allMagnitudesInDb[0])));
        for (int x = xMin + 1; x<=xMax; ++x)
        {
            magnitude.lineTo (x, jlimit (static_cast<float> (yMin), static_cast<float> (yMax) + OH + 1.0f, dbToYFloat (allMagnitudesInDb[x-xMin])));
        }

        g.setColour (Colours::white);
        g.strokePath (magnitude, PathStrokeType (1.5f));
        magnitude.lineTo (xMax, yZero);
        magnitude.lineTo (xMin, yZero);
        magnitude.closeSubPath();
        g.setColour (Colours::white.withMultipliedAlpha (0.1f));
        g.fillPath (magnitude);
      
      
        // draw filter knobs
        float prevXPos = xMin;
        for (int i = 0; i < numFreqBands-1; ++i)
        {
            auto handle (elements[2*i]);
            float xPos = handle->frequencySlider == nullptr ? xMin : hzToX (handle->frequencySlider->getValue());
          
            // we must keep some distance between the freq bands (they must not overlap).
            // otherwise filling the rectangle will fail
            float xWidth = static_cast<int>(xPos - prevXPos) >= 1.0f ? xPos - prevXPos : 1.0f;
            g.setColour (handle->colour.withMultipliedAlpha (0.3f));
            g.fillRoundedRectangle (prevXPos, yMin, xWidth, yMax, 4.0);
          
            // draw separation lines between frequency bands
            Path filterBandSeparator;
            filterBandSeparator.startNewSubPath (xPos, yMin);
            filterBandSeparator.lineTo (xPos, yMax + yMin);
            g.setColour (Colour (0xFF979797));
            g.strokePath (filterBandSeparator, PathStrokeType (2.0f));
            prevXPos = xPos;

            float yPos = dbToYFloat (allMagnitudesInDb[xPos - xMin]);
          
            g.setColour (Colour (0xFF191919));
            g.drawEllipse (xPos - 5.0f, yPos - 5.0f , 10.0f, 10.0f, 3.0f);

            Colour ellipseColour = Colour (0xFF979797).brighter();
            g.setColour (ellipseColour);
            g.drawEllipse (xPos - 5.0f, yPos - 5.0f , 10.0f, 10.0f, 2.0f);
            g.setColour (activeElem == 2*i ? ellipseColour : ellipseColour.withSaturation (0.2));
            g.fillEllipse (xPos - 5.0f, yPos - 5.0f , 10.0f, 10.0f);
        }
      
        // fill last rectangle
        auto handle = elements.getLast();
        float xWidth = static_cast<int>(xMax - prevXPos) >= 1.0f ? xMax - prevXPos : 1.0f;
        g.setColour (handle->colour.withMultipliedAlpha (0.3f));
        g.fillRoundedRectangle (prevXPos, yMin, xWidth, yMax, 4.0);
    }


    void resized() override
    {
        int xMin = hzToX(s.fMin);
        int xMax = hzToX(s.fMax);
        numPixels = xMax - xMin + 1;

        frequencies.resize (numPixels);
        for (int i = 0; i < numPixels; ++i)
            frequencies.set (i, xToHz (xMin + i));

        allMagnitudesInDb.resize (numPixels);
        magnitudes.resize (numPixels);
        magnitudes.fill (1.0f);

        const float width = getWidth() - mL - mR;
        dbGridPath.clear();

        float dyn = s.dbMax - s.dbMin;
        int numgridlines = dyn / s.gridDiv + 1;

        for (int i = 0; i < numgridlines; ++i)
        {
            float db_val = s.dbMax - i * s.gridDiv;

            int ypos = dbToY (db_val);

            dbGridPath.startNewSubPath (mL - OH, ypos);
            dbGridPath.lineTo (mL + width + OH, ypos);
        }

        hzGridPath.clear();
        hzGridPathBold.clear();

        for (float f = s.fMin; f <= s.fMax; f += powf(10, floor (log10 (f))))
        {
            int xpos = hzToX (f);

            if ((f == 20) || (f == 50) || (f == 100) || (f == 500) || (f == 1000) || (f == 5000) || (f == 10000) || (f == 20000))
            {
                hzGridPathBold.startNewSubPath (xpos, dbToY (s.dbMax) - OH);
                hzGridPathBold.lineTo (xpos, dbToY (s.dbMin) + OH);

            } else
            {
                hzGridPath.startNewSubPath (xpos, dbToY (s.dbMax) - OH);
                hzGridPath.lineTo (xpos, dbToY (s.dbMin) + OH);
            }
        }
    }

    int inline drawLevelMark (Graphics& g, int x, int width, const int level, const String& label, int lastTextDrawPos = -1)
    {
        float yPos = dbToYFloat (level);
        x = x + 1.0f;
        width = width - 2.0f;

        if (yPos - 4 > lastTextDrawPos)
        {
            g.drawText (label, x + 2, yPos - 4, width - 4, 9, Justification::right, false);
            return yPos + 5;
        }
        return lastTextDrawPos;
    }

    int dbToY (const float dB)
    {
        int ypos = dbToYFloat(dB);
        return ypos;
    }

    float dbToYFloat (const float dB)
    {
        const float height = static_cast<float> (getHeight()) - mB - mT;
        if (height <= 0.0f) return 0.0f;
        float temp;
        if (dB < 0.0f)
            temp = zero + std::tanh(dB / dyn * -2.0f);
        else
            temp = zero - 2.0f * dB / dyn;

        return mT + scale * height * temp;
    }

    float yToDb (const float y)
    {
        float height = static_cast<float> (getHeight()) - mB - mT;

        float temp = (y - mT) / height / scale - zero;
        float dB;
        if (temp > 0.0f)
            dB =  std::atanh (temp) * dyn * -0.5f;
        else
            dB = - 0.5f * temp * dyn;
        return std::isnan (dB) ? s.dbMin : dB;
    }

    int hzToX (float hz)
    {
        float width = static_cast<float> (getWidth()) - mL - mR;
        int xpos = mL + width * (log (hz / s.fMin) / log (s.fMax / s.fMin));
        return xpos;
    }

    float xToHz (int x)
    {
        float width = static_cast<float> (getWidth()) - mL - mR;
        return s.fMin * pow ((s.fMax / s.fMin), ((x - mL) / width));
    }

    void setSampleRate (const double newSampleRate)
    {
        sampleRate = newSampleRate;
    }

    void setOverallGain (float newGain)
    {
        float gainInDb = Decibels::gainToDecibels (newGain);
        if (overallGainInDb != gainInDb)
        {
            overallGainInDb = gainInDb;
            repaint();
        }
    }

    void setOverallGainInDecibels (float newGainInDecibels)
    {
        if (overallGainInDb != newGainInDecibels)
        {
            overallGainInDb = newGainInDecibels;
            repaint();
        }
    }

    void mouseDrag (const MouseEvent &event) override
    {
        Point<int> pos = event.getPosition();
        float frequency = xToHz (pos.x);

        if (activeElem != -1)
        {
            auto handle (elements[activeElem]);
            if (handle->frequencySlider != nullptr)
                handle->frequencySlider->setValue (frequency);
        }
    }

    void mouseMove (const MouseEvent &event) override
    {
        Point<int> pos = event.getPosition();
        int oldActiveElem = activeElem;
        activeElem = -1;
        jassert (elements.size() >= 2* (numFreqBands - 1));
        for (int i = 0; i < numFreqBands - 1; ++i)
        {
            auto handle (elements[2*i]);
          
            int x = handle->frequencySlider == nullptr ? hzToX (s.fMin) : hzToX (handle->frequencySlider->getValue());
            float y = dbToYFloat (allMagnitudesInDb[x - hzToX (s.fMin)]);

            Point<int> filterPos (x, y);
            if (pos.getDistanceSquaredFrom (filterPos) < 48)
            {
                activeElem = 2*i;
                break;
            }
        }

        if (oldActiveElem != activeElem)
            repaint();
    }
  
    void addCoefficients (typename dsp::IIR::Coefficients<coefficientsType>::Ptr newCoeffs, Colour newColourForCoeffs, Slider* frequencySlider = nullptr, Slider* gainSlider = nullptr)
    {
        elements.add (new FilterWithSlidersAndColour<coefficientsType> {newCoeffs, newColourForCoeffs, frequencySlider, gainSlider});
    }
  
    void setNumFreqBands (const int value)
    {
        numFreqBands = value;
    }
  
    void setFilterToSyncWith (const int filterIdx, const int filterToSyncWith)
    {
        if (filterIdx < elements.size() && filterToSyncWith < elements.size())
        {
            elements[filterIdx]->syncFilterIndex = filterToSyncWith;
            repaint();
        }
    }

private:

    struct Settings
    {
        float fMin = 20.0f;    // minimum displayed frequency
        float fMax = 20000.0f; // maximum displayed frequency
        float dbMin = -15.0f;  // min displayed dB
        float dbMax = 15.0f;   // max displayed dB
        float gridDiv = 5.0f;  // how many dB per divisions (between grid lines)
    };

    template <typename T>
    struct FilterWithSlidersAndColour
    {
        typename dsp::IIR::Coefficients<T>::Ptr coefficients;
        Colour colour;
        Slider* frequencySlider = nullptr;
        Slider* gainSlider = nullptr;
        int syncFilterIndex = std::numeric_limits<int>::lowest(); // cutoff synced with other filter. synced filters are assumed to be hipass
    };

    const float mL = 23.0f;
    const float mR = 10.0f;
    const float mT = 7.0f;
    const float mB = 15.0f;
    const float OH = 3.0f;

    float overallGainInDb {0.0};

    double sampleRate;

    int activeElem = -1;

    float dyn, zero, scale;

    Settings s;
    Path dbGridPath;
    Path hzGridPath;
    Path hzGridPathBold;

    Array<double> frequencies;
    Array<double> magnitudes;
    int numPixels, numFreqBands;

    Array<float> allMagnitudesInDb;

    OwnedArray<FilterWithSlidersAndColour<coefficientsType>> elements;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (FilterBankVisualizer)
};
