/*
 ==============================================================================
 This file is part of the IEM plug-in suite.
 Author: Daniel Rudrich, Felix Holzmueller
 Copyright (c) 2022 - Institute of Electronic Music and Acoustics (IEM)
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

/*
 The following code taken from JUCE and modified.
 */

/*
==============================================================================

   This file is part of the JUCE library.
   Copyright (c) 2022 - Raw Material Software Limited

   JUCE is an open source library subject to commercial or open-source
   licensing.

   By using JUCE, you agree to the terms of both the JUCE 7 End-User
License
   Agreement and JUCE Privacy Policy.

   End User License Agreement: www.juce.com/juce-7-licence
   Privacy Policy: www.juce.com/juce-privacy-policy

   Or: You may also use this code under the terms of the GPL v3 (see
   www.gnu.org/licenses).

   JUCE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY, AND ALL WARRANTIES,
WHETHER
   EXPRESSED OR IMPLIED, INCLUDING MERCHANTABILITY AND FITNESS FOR
PURPOSE, ARE
   DISCLAIMED.

  
==============================================================================
*/

#include <JuceHeader.h>
#include <PluginProcessor.h>

#include "MyStandaloneFilterWindow.h"

namespace juce
{

//==============================================================================
class StandaloneApp : public JUCEApplication
{
public:
    StandaloneApp()
    {
        PluginHostType::jucePlugInClientCurrentWrapperType = AudioProcessor::wrapperType_Standalone;

        juce::PropertiesFile::Options options;

        options.applicationName = getApplicationName();
        options.filenameSuffix = ".settings";
        options.osxLibrarySubFolder = "Application Support/IEMAudioPlugins";
#if JUCE_LINUX || JUCE_BSD
        options.folderName =
            File::getSpecialLocation (File::userApplicationDataDirectory).getFullPathName()
            + File::getSeparatorString() + "IEMAudioPlugins";
#else
        options.folderName = "IEMAudioPlugins";
#endif

        appProperties.setStorageParameters (options);
    }

    const juce::String getApplicationName() override { return CharPointer_UTF8 (JucePlugin_Name); }
    const juce::String getApplicationVersion() override { return JucePlugin_VersionString; }
    bool moreThanOneInstanceAllowed() override { return true; }
    void anotherInstanceStarted (const juce::String&) override {}

#ifdef HEADLESS_BUILD
    virtual MyStandalonePluginHolder* createProcessor() { 
#ifdef JucePlugin_PreferredChannelConfigurations
        StandalonePluginHolder::PluginInOuts channels[] = {
            JucePlugin_PreferredChannelConfigurations
        };
#endif

        return new MyStandalonePluginHolder (appProperties.getUserSettings());
    }
#else
    virtual MyStandaloneFilterWindow* createWindow()
    {
#ifdef JucePlugin_PreferredChannelConfigurations
        StandalonePluginHolder::PluginInOuts channels[] = {
            JucePlugin_PreferredChannelConfigurations
        };
#endif

        return new MyStandaloneFilterWindow (
            getApplicationName(),
            LookAndFeel::getDefaultLookAndFeel().findColour (
                juce::ResizableWindow::backgroundColourId),
            appProperties.getUserSettings(),
            false,
            {},
            nullptr
#ifdef JucePlugin_PreferredChannelConfigurations
            ,
            juce::Array<StandalonePluginHolder::PluginInOuts> (channels,
                                                               juce::numElementsInArray (channels))
#else
            ,
            {}
#endif
#if JUCE_DONT_AUTO_OPEN_MIDI_DEVICES_ON_MOBILE
                ,
            false
#endif
        );
    }
#endif

    //==============================================================================
    void initialise (const juce::String&) override
    {
#ifdef HEADLESS_BUILD
        mainProcessor.reset (createProcessor());
#else
        mainWindow.reset (createWindow());

#if JUCE_STANDALONE_FILTER_WINDOW_USE_KIOSK_MODE
        Desktop::getInstance().setKioskModeComponent (mainWindow.get(), false);
#endif

        mainWindow->setVisible (true);
#endif
    }

    void shutdown() override
    {
#ifdef HEADLESS_BUILD
        mainProcessor.reset();
#else
        mainWindow.reset();
#endif
        appProperties.saveIfNeeded();
    }

    //==============================================================================
    void systemRequestedQuit() override
    {
#ifdef HEADLESS_BUILD
        if (mainProcessor.get() != nullptr)
            mainProcessor->savePluginState();
#else
        if (mainWindow.get() != nullptr)
            mainWindow->pluginHolder->savePluginState();
#endif

        if (ModalComponentManager::getInstance()->cancelAllModalComponents())
        {
            juce::Timer::callAfterDelay (100,
                                         []()
                                         {
                                             if (auto app = JUCEApplicationBase::getInstance())
                                                 app->systemRequestedQuit();
                                         });
        }
        else
        {
            quit();
        }
    }

protected:
    ApplicationProperties appProperties;
#ifdef HEADLESS_BUILD
    std::unique_ptr<MyStandalonePluginHolder> mainProcessor;
#else
    std::unique_ptr<MyStandaloneFilterWindow> mainWindow;
#endif
};

} // namespace juce

#if JucePlugin_Build_Standalone && JUCE_IOS

JUCE_BEGIN_IGNORE_WARNINGS_GCC_LIKE ("-Wmissing-prototypes")

using namespace juce;

bool JUCE_CALLTYPE juce_isInterAppAudioConnected()
{
    if (auto holder = StandalonePluginHolder::getInstance())
        return holder->isInterAppAudioConnected();

    return false;
}

void JUCE_CALLTYPE juce_switchToHostApplication()
{
    if (auto holder = StandalonePluginHolder::getInstance())
        holder->switchToHostApplication();
}

Image JUCE_CALLTYPE juce_getIAAHostIcon (int size)
{
    if (auto holder = StandalonePluginHolder::getInstance())
        return holder->getIAAHostIcon (size);

    return Image();
}

JUCE_END_IGNORE_WARNINGS_GCC_LIKE
#endif

START_JUCE_APPLICATION (juce::StandaloneApp)
