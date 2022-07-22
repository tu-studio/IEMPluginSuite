
juce_add_plugin (GranularEncoder
    PLUGIN_MANUFACTURER_CODE IEMA
    PLUGIN_CODE GrEn
    ICON_BIG "../resources/Standalone/IEMPluginSuiteSmall.png"
    ICON_SMALL "../resources/Standalone/IEMPluginSuiteSmall.png"
    COMPANY_NAME "IEM"
    PRODUCT_NAME "GranularEncoder"
    FORMATS ${IEM_FORMATS}
    VERSION "0.0.1")


juce_generate_juce_header (GranularEncoder)

target_sources (GranularEncoder PRIVATE
    Source/PluginEditor.cpp
    Source/PluginEditor.h
    Source/PluginProcessor.cpp
    Source/PluginProcessor.h
    Source/Grain.cpp
    Source/Grain.h
    Source/Type.h

    ../resources/OSC/OSCInputStream.h
    ../resources/OSC/OSCParameterInterface.cpp
    ../resources/OSC/OSCParameterInterface.h
    ../resources/OSC/OSCStatus.cpp
    ../resources/OSC/OSCStatus.h
    ../resources/OSC/OSCUtilities.h

    ../resources/efficientSHvanilla.cpp
    )

target_compile_definitions (GranularEncoder PRIVATE
    JUCE_USE_CURL=0
    JUCE_WEB_BROWSER=0
    JUCE_USE_CUSTOM_PLUGIN_STANDALONE_APP=1
    JUCE_DISPLAY_SPLASH_SCREEN=0
    JUCE_VST3_CAN_REPLACE_VST2=0)

target_include_directories (GranularEncoder PRIVATE Source)

target_link_libraries (GranularEncoder PRIVATE
    LAF_fonts
    juce::juce_audio_utils
    juce::juce_audio_plugin_client
    juce::juce_osc
    juce::juce_opengl
    juce::juce_recommended_config_flags
    juce::juce_recommended_lto_flags)
