/************************************************************************************/
/*  FILE DESCRIPTION                                                                */
/*----------------------------------------------------------------------------------*/
/*!
 *   @file       sofawrite.cpp
 *   @brief      Various code examples... Just adapt these pieces of code to your needs...
 *   @author     Thibaut Carpentier, UMR STMS 9912 - Ircam-Centre Pompidou / CNRS / UPMC
 *
 *   @date       15/10/2014
 *
 */
/************************************************************************************/
#include "libsofa/src/SOFA.h"
//#include "libsofa/dependencies/include/ncDim.h"
//#include "libsofa/dependencies/include/ncVar.h"


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


/************************************************************************************/
/*!
 *  @brief          Main entry point
 *
 */
/************************************************************************************/
int main(int argc, char *argv[])
{
    ///@todo : change this !
    const std::string filename = "/home/pradeep/Q4/SpatialAudio/CPP_Project/subject_020.sofa";
    //"/Users/tcarpent/Desktop/sofa_files/subject_003.sofa";
    //"/Users/tcarpent/Downloads/ClubFritz_SH_BM_01.sofa";
    //"/Users/tcarpent/Desktop/sofa_files/hpir_nh2.sofa";
    ///"/Users/tcarpent/Desktop/sofa_files/BTDEI-hp_H010-subj_S115-Set02_BEC-RAW.sofa";
    
//    TestFileConvention( filename );
    
    DisplayInformations( filename );
    
    /// example for creating a SimpleFreeFieldHRIR file
    //CreateSimpleFreeFieldHRIRFile();
    
    return 0;
}
