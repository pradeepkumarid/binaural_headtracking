/*
 Copyright (c) 2013-2014, UMR STMS 9912 - Ircam-Centre Pompidou / CNRS / UPMC
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the <organization> nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 
 Spatial acoustic data file format - AES69-2015 - Standard for File Exchange - Spatial Acoustic Data File Format
 http://www.aes.org
 
 SOFA (Spatially Oriented Format for Acoustics)
 http://www.sofaconventions.org
 
 */


/************************************************************************************/
/*  FILE DESCRIPTION                                                                */
/*----------------------------------------------------------------------------------*/
/*!
 *   @file       SOFAGeneralFIR.cpp
 *   @brief      Class for SOFA files with GeneralFIR convention
 *   @author     Thibaut Carpentier, UMR STMS 9912 - Ircam-Centre Pompidou / CNRS / UPMC
 *
 *   @date       10/05/2013
 *
 */
/************************************************************************************/
#include "../src/SOFAGeneralFIR.h"
#include "../src/SOFAUtils.h"
#include "../src/SOFANcUtils.h"
#include "../src/SOFAString.h"
#include "../src/SOFAPoint3.h"
#include "../src/SOFAListener.h"

using namespace sofa;

const unsigned int GeneralFIR::ConventionVersionMajor  =   1;
const unsigned int GeneralFIR::ConventionVersionMinor  =   0;

const std::string GeneralFIR::GetConventionVersion()
{
    return sofa::String::Int2String( GeneralFIR::ConventionVersionMajor ) + std::string(".") + sofa::String::Int2String( GeneralFIR::ConventionVersionMinor );
}

/************************************************************************************/
/*!
 *  @brief          Class constructor
 *  @param[in]      path : the file path
 *  @param[in]      mode : opening mode
 *
 */
/************************************************************************************/
GeneralFIR::GeneralFIR(const std::string &path,
                       const netCDF::NcFile::FileMode &mode)
: sofa::File( path, mode )
{
}

/************************************************************************************/
/*!
 *  @brief          Class destructor
 *
 */
/************************************************************************************/
GeneralFIR::~GeneralFIR()
{
}

const bool GeneralFIR::checkGlobalAttributes() const
{
    sofa::Attributes attributes;
    GetGlobalAttributes( attributes );
    
    if( attributes.Get( sofa::Attributes::kSOFAConventions ) != "GeneralFIR" )
    {
        SOFA_THROW( "Not a 'GeneralFIR' SOFAConvention" );
        return false;
    }
    
    /// the value of DataType shall be 'FIR'
    if( attributes.Get( sofa::Attributes::kDataType ) != "FIR" )
    {
        SOFA_THROW( "invalid 'DataType'" );
        return false;
    }
    
    return true;
}

/************************************************************************************/
/*!
 *  @brief          Returns true if this is a valid SOFA file with GeneralFIR convention
 *
 */
/************************************************************************************/
const bool GeneralFIR::IsValid() const
{
    if( sofa::File::IsValid() == false )
    {
        return false;
    }
    
    if( IsFIRDataType() == false )
    {
        SOFA_THROW( "'DataType' shall be FIR" );
        return false;
    }
    
    if( checkGlobalAttributes() == false )
    {
        return false;
    }
    
    SOFA_ASSERT( GetDimension( "I" ) == 1 );
    SOFA_ASSERT( GetDimension( "C" ) == 3 );
    
    return true;
}

