#include <stdio.h>
#include "Gamma/AudioIO.h"
#include "Gamma/Domain.h"
#include "../libsofa/src/SOFA.h"


using namespace gam;

int frameCount    = 512;
int samplingRate  = 44100;
int channelsIn      = 0;
int channelsOut     = 2;

sofa::SimpleFreeFieldHRIR HRIR("/home/pradeep/Q4/SpatialAudio/Project/CPP/subject_020.sofa");
std::vector< double > values;
double set[500000];



/************************************************************************************/
/*!
 *  @brief          Example for displaying all informations about a NetCDFFile file,
 *                  in a fashion similar to matlab 'ncdisp' command
 *
 */
/************************************************************************************/
static void DisplayInformations(const std::string & filename,
                                std::ostream & output = std::cout)
{
    ///@n this doesnt check whether the file corresponds to SOFA conventions...
    const sofa::NetCDFFile file( filename );
    
    const std::string tabSeparator = "\t";
    
    //==============================================================================
    // global attributes
    //==============================================================================
    {
        std::vector< std::string > attributeNames;
        file.GetAllAttributesNames( attributeNames );
        
        output << std::endl;
        output << "Global Attributes:" << std::endl;
        
        for( std::size_t i = 0; i < attributeNames.size(); i++ )
        {
            const std::string name = attributeNames[i];
            const std::string value= file.GetAttributeValueAsString( name );
            
            output << tabSeparator << sofa::String::PadWith( attributeNames[i] ) << " = " << value << std::endl;
        }
    }
    
    //==============================================================================
    // dimensions
    //==============================================================================
    {
        std::vector< std::string > dimensionNames;
        file.GetAllDimensionsNames( dimensionNames );
        
        output << std::endl;
        output << "Dimensions:" << std::endl;
        
        for( std::size_t i = 0; i < dimensionNames.size(); i++ )
        {
            const std::string name = dimensionNames[i];
            const std::size_t dim  = file.GetDimension( name );
            output << tabSeparator << name << " = " << dim << std::endl;
        }
    }
    
    //==============================================================================
    // variables
    //==============================================================================
    {
        std::vector< std::string > variableNames;
        file.GetAllVariablesNames( variableNames );
        
        output << std::endl;
        output << "Variables:" << std::endl;
        
        for( std::size_t i = 0; i < variableNames.size(); i++ )
        {
            const std::string name      = variableNames[i];
            const std::string typeName  = file.GetVariableTypeName( name );
            
            const std::string dimsNames = file.GetVariableDimensionsNamesAsString( name );
            const std::string dims      = file.GetVariableDimensionsAsString( name );
            
            output << tabSeparator << name << std::endl;
            output << tabSeparator << tabSeparator << sofa::String::PadWith( "Datatype: " ) << typeName << std::endl;
            output << tabSeparator << tabSeparator << sofa::String::PadWith( "Dimensions: ") << dimsNames << std::endl;;
            output << tabSeparator << tabSeparator << sofa::String::PadWith( "Size: ") << dims << std::endl;;
            
            std::vector< std::string > attributeNames;
            std::vector< std::string > attributeValues;
            file.GetVariablesAttributes( attributeNames, attributeValues, name );
            
            SOFA_ASSERT( attributeNames.size() == attributeValues.size() );
            
            if( attributeNames.size() > 0 )
            {
                output << tabSeparator << tabSeparator << sofa::String::PadWith( "Attributes: ") << dims << std::endl;;
            }
            
            for( std::size_t j = 0; j < attributeNames.size(); j++ )
            {
                output << tabSeparator << tabSeparator << tabSeparator;
                output << sofa::String::PadWith( attributeNames[j] ) << " = " << attributeValues[j] << std::endl;
            }
        }
        
    }
}


void audioCallBack(AudioIOData& io)
{
    // Your Code: Executes once every frameCount samples
    
    while(io())
    {
        // Your Code: Executes on every sample
        
        for (int i = 0; i < channelsOut; i++)
        {
            io.out(i) = 0;
        }
    }
}

int main()
{
    
    const std::string filename = "/home/pradeep/Q4/SpatialAudio/Project/CPP/subject_020.sofa";
    
        
    DisplayInformations( filename );
    
    HRIR.GetDataIR(values);
    
    std::cout << "Data Size: " << values.size() << std::endl;
    
    AudioIO audioIO(frameCount, samplingRate, audioCallBack, NULL, channelsOut, channelsIn);
    Sync::master().spu(audioIO.framesPerSecond());
    audioIO.start();
    printf("Press 'enter' to quit...\n");
    getchar();
    return 0;
}
