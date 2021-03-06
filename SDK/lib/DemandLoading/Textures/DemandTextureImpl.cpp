//
// Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

#include "Textures/DemandTextureImpl.h"
#include "DemandLoaderImpl.h"
#include "Memory/TilePool.h"
#include "PageTableManager.h"
#include "Textures/TextureRequestHandler.h"
#include "Util/Math.h"
#include "Util/Stopwatch.h"

#include <DemandLoading/TileIndexing.h>
#include <ImageReader/ImageReader.h>

#include <cuda.h>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace demandLoading {

DemandTextureImpl::DemandTextureImpl( unsigned int                              id,
                                      unsigned int                              maxNumDevices,
                                      const TextureDescriptor&                  descriptor,
                                      std::shared_ptr<imageReader::ImageReader> image,
                                      DemandLoaderImpl*                         loader )

    : m_id( id )
    , m_descriptor( descriptor )
    , m_image( image )
    , m_loader( loader )
{
    // Construct per-device sparse and dense textures.  These are just empty shells until they are initialized.  
    // Note that the vectors do not grow after construction, which is important for thread safety.
    m_sparseTextures.reserve( maxNumDevices );
    m_denseTextures.reserve( maxNumDevices );
    for( unsigned int i = 0; i < maxNumDevices; ++i )
    {
        m_sparseTextures.emplace_back( i );
        m_denseTextures.emplace_back( i );
    }

    m_sampler = {0};
}

unsigned int DemandTextureImpl::getId() const
{
    return m_id;
}

bool DemandTextureImpl::init( unsigned int deviceIndex )
{
    std::unique_lock<std::mutex> lock( m_initMutex );

    // Open the image if necessary, fetching the dimensions and other info.
    if( !m_isInitialized )
    {
        if( !m_image->open( &m_info ) )
            return false;
    }

    // Initialize the sparse or dense texture for the specified device.
    if( useSparseTexture() )
    {
        DEMAND_ASSERT( deviceIndex < m_sparseTextures.size() );
        SparseTexture& sparseTexture = m_sparseTextures[deviceIndex];
        sparseTexture.init( m_descriptor, m_info );

        if( !m_isInitialized )
        {
            m_isInitialized = true;

            // Retain various properties for subsequent use.  (They're the same on all devices.)
            m_tileWidth         = sparseTexture.getTileWidth();
            m_tileHeight        = sparseTexture.getTileHeight();
            m_mipTailFirstLevel = sparseTexture.getMipTailFirstLevel();
            m_mipTailSize       = m_mipTailFirstLevel < m_info.numMipLevels ? sparseTexture.getMipTailSize() : 0;

            // Verify that the tile size agrees with TilePool.
            DEMAND_ASSERT( m_tileWidth * m_tileHeight * imageReader::getBytesPerChannel( getInfo().format ) <= sizeof( TileBuffer ) );

            // Record the dimensions of each miplevel.
            const unsigned int numMipLevels = getInfo().numMipLevels;
            m_mipLevelDims.resize( numMipLevels );
            for( unsigned int i = 0; i < numMipLevels; ++i )
            {
                m_mipLevelDims[i] = sparseTexture.getMipLevelDims( i );
            }

            initSampler();
        }
    }
    else // dense texture
    {
        DEMAND_ASSERT( deviceIndex < m_denseTextures.size() );
        DenseTexture& denseTexture = m_denseTextures[deviceIndex];
        denseTexture.init( m_descriptor, m_info );

        if( !m_isInitialized )
        {
            m_isInitialized = true;

            // Set dummy properties (not used for dense textures)
            m_tileWidth         = 64;
            m_tileHeight        = 64;
            m_mipTailFirstLevel = 0;
            m_mipTailSize       = 0;

            // Record the dimensions of each miplevel.
            const unsigned int numMipLevels = getInfo().numMipLevels;
            m_mipLevelDims.resize( numMipLevels );
            for( unsigned int i = 0; i < numMipLevels; ++i )
            {
                m_mipLevelDims[i] = denseTexture.getMipLevelDims( i );
            }

            initSampler();
        }
    }
    return true;
}

void DemandTextureImpl::initSampler()
{
    // Construct the canonical sampler for this texture, excluding the CUDA texture object, which
    // differs for each device (see getTextureObject).
    
    // Note: m_sampler zeroed out in constructor, so that the udim fields can be initialized before calling initSampler.

    // Descriptions
    m_sampler.desc.numMipLevels     = getInfo().numMipLevels;
    m_sampler.desc.logTileWidth     = static_cast<unsigned int>( log2f( static_cast<float>( getTileWidth() ) ) );
    m_sampler.desc.logTileHeight    = static_cast<unsigned int>( log2f( static_cast<float>( getTileHeight() ) ) );
    m_sampler.desc.isSparseTexture  = useSparseTexture() ? 1 : 0;
    m_sampler.desc.wrapMode0        = static_cast<int>( getDescriptor().addressMode[0] );
    m_sampler.desc.wrapMode1        = static_cast<int>( getDescriptor().addressMode[1] );
    m_sampler.desc.mipmapFilterMode = getDescriptor().mipmapFilterMode;
    m_sampler.desc.maxAnisotropy    = getDescriptor().maxAnisotropy;

    // Dimensions
    m_sampler.width             = getInfo().width;
    m_sampler.height            = getInfo().height;
    m_sampler.mipTailFirstLevel = getMipTailFirstLevel();

    // Initialize mipLevelSizes
    demandLoading::TextureSampler::MipLevelSizes* mls = m_sampler.mipLevelSizes;
    memset( mls, 0, MAX_TILE_LEVELS * sizeof( demandLoading::TextureSampler::MipLevelSizes ) );

    if( m_sampler.desc.isSparseTexture )
    {
        // Calculate number of tiles for sparse texture
        for( int mipLevel = static_cast<int>( m_sampler.mipTailFirstLevel ); mipLevel >= 0; --mipLevel )
        {
            if( mipLevel < static_cast<int>( m_sampler.mipTailFirstLevel ) )
                mls[mipLevel].mipLevelStart = mls[mipLevel + 1].mipLevelStart + getNumTilesInLevel( mipLevel + 1 );
            else
                mls[mipLevel].mipLevelStart = 0;

            mls[mipLevel].levelWidthInTiles = static_cast<unsigned short>(
                getLevelDimInTiles( m_sampler.width, static_cast<unsigned int>( mipLevel ), getTileWidth() ) );
            mls[mipLevel].levelHeightInTiles = static_cast<unsigned short>(
                getLevelDimInTiles( m_sampler.height, static_cast<unsigned int>( mipLevel ), getTileHeight() ) );
        }
        m_sampler.numPages = mls[0].mipLevelStart + getNumTilesInLevel( 0 );

        // Reserve a range of page table entries, one per tile, associated with the page request
        // handler for this texture.
        m_requestHandler.reset( new TextureRequestHandler( this, m_loader ) );
        m_sampler.startPage = m_loader->getPageTableManager()->reserve( m_sampler.numPages, m_requestHandler.get() );
    }
    else // Dense texture 
    {
        // Dense textures do not need extra page table entries
        m_sampler.numPages = 0;
        m_sampler.startPage = m_id;
    }
}

const imageReader::TextureInfo& DemandTextureImpl::getInfo() const
{
    DEMAND_ASSERT( m_isInitialized );
    return m_info;
}

const TextureSampler& DemandTextureImpl::getSampler() const
{
    DEMAND_ASSERT( m_isInitialized );
    return m_sampler;
}

const TextureDescriptor& DemandTextureImpl::getDescriptor() const
{
    return m_descriptor;
}

uint2 DemandTextureImpl::getMipLevelDims( unsigned int mipLevel ) const
{
    DEMAND_ASSERT( m_isInitialized );
    DEMAND_ASSERT( mipLevel < m_mipLevelDims.size() );
    return m_mipLevelDims[mipLevel];
}

unsigned int DemandTextureImpl::getTileWidth() const
{
    DEMAND_ASSERT( m_isInitialized );
    return m_tileWidth;
}

unsigned int DemandTextureImpl::getTileHeight() const
{
    DEMAND_ASSERT( m_isInitialized );
    return m_tileHeight;
}

unsigned int DemandTextureImpl::getMipTailFirstLevel() const
{
    DEMAND_ASSERT( m_isInitialized );
    return m_mipTailFirstLevel;
}

CUtexObject DemandTextureImpl::getTextureObject( unsigned int deviceIndex ) const
{
    DEMAND_ASSERT( m_isInitialized );
    if( useSparseTexture() )
    {
        DEMAND_ASSERT( deviceIndex < m_sparseTextures.size() );
        return m_sparseTextures[deviceIndex].getTextureObject();
    }
    else 
    {
        DEMAND_ASSERT( deviceIndex < m_denseTextures.size() );
        return m_denseTextures[deviceIndex].getTextureObject();
    }
}

unsigned int DemandTextureImpl::getNumTilesInLevel( unsigned int mipLevel ) const
{
    if( mipLevel > getMipTailFirstLevel() || mipLevel >= getInfo().numMipLevels )
        return 0;

    unsigned int levelWidthInTiles  = getLevelDimInTiles( m_mipLevelDims[0].x, mipLevel, m_tileWidth );
    unsigned int levelHeightInTiles = getLevelDimInTiles( m_mipLevelDims[0].y, mipLevel, m_tileHeight );

    return calculateNumTilesInLevel( levelWidthInTiles, levelHeightInTiles );
}

// Tiles can be read concurrently.  The EXRReader currently locks, however, because the OpenEXR 2.x
// tile reading API is stateful.  That should be fixed in OpenEXR 3.0.
bool DemandTextureImpl::readTile( unsigned int mipLevel, unsigned int tileX, unsigned int tileY, char* tileBuffer, size_t tileBufferSize ) const
{
    DEMAND_ASSERT( m_isInitialized );
    DEMAND_ASSERT( mipLevel < m_info.numMipLevels );

    // Resize buffer if necessary.
    const unsigned int bytesPerPixel = imageReader::getBytesPerChannel( getInfo().format ) * getInfo().numChannels;
    const unsigned int bytesPerTile  = getTileWidth() * getTileHeight() * bytesPerPixel;
    DEMAND_ASSERT_MSG( bytesPerTile <= tileBufferSize, "Maximum tile size exceeded" );

    return m_image->readTile( tileBuffer, mipLevel, tileX, tileY, getTileWidth(), getTileHeight() );
}

// Tiles can be filled concurrently.
void DemandTextureImpl::fillTile( unsigned int                 deviceIndex,
                                  CUstream                     stream,
                                  unsigned int                 mipLevel,
                                  unsigned int                 tileX,
                                  unsigned int                 tileY,
                                  const char*                  tileData,
                                  size_t                       tileSize,
                                  CUmemGenericAllocationHandle handle,
                                  size_t                       offset ) const
{
    DEMAND_ASSERT( deviceIndex < m_sparseTextures.size() );
    DEMAND_ASSERT( mipLevel < m_info.numMipLevels );
    DEMAND_ASSERT( tileSize <= sizeof( TileBuffer ) );

    m_sparseTextures[deviceIndex].fillTile( stream, mipLevel, tileX, tileY, tileData, tileSize, handle, offset );
}

// Tiles can be unmapped concurrently.
void DemandTextureImpl::unmapTile( unsigned int deviceIndex, CUstream stream, unsigned int mipLevel, unsigned int tileX, unsigned int tileY ) const
{
    DEMAND_ASSERT( deviceIndex < m_sparseTextures.size() );
    DEMAND_ASSERT( mipLevel < m_info.numMipLevels );
    m_sparseTextures[deviceIndex].unmapTile( stream, mipLevel, tileX, tileY );
}

// Request deduplication will ensure that concurrent calls to readMipTail do not occur.  Note that
// the EXRReader currently locks because the OpenEXR 2.x tile reading API is stateful.  That should
// be fixed in OpenEXR 3.0.
bool DemandTextureImpl::readMipTail( char* buffer, size_t bufferSize ) const
{
    DEMAND_ASSERT( m_isInitialized );
    DEMAND_ASSERT( getMipTailFirstLevel() < m_info.numMipLevels );
    DEMAND_ASSERT_MSG( m_mipTailSize <= bufferSize, "Maximum mip tail size exceeded" );

    const unsigned int pixelSize = getInfo().numChannels * imageReader::getBytesPerChannel( getInfo().format );
    return m_image->readMipTail( buffer, getMipTailFirstLevel(), getInfo().numMipLevels, m_mipLevelDims.data(), pixelSize );
}

// Request deduplication will ensure that concurrent calls to readMipTail do not occur.  Note that
// the EXRReader currently locks because the OpenEXR 2.x tile reading API is stateful.  That should
// be fixed in OpenEXR 3.0.
void DemandTextureImpl::fillMipTail( unsigned int                 deviceIndex,
                                     CUstream                     stream,
                                     const char*                  mipTailData,
                                     size_t                       mipTailSize,
                                     CUmemGenericAllocationHandle handle,
                                     size_t                       offset ) const
{
    DEMAND_ASSERT( deviceIndex < m_sparseTextures.size() );
    DEMAND_ASSERT( getMipTailFirstLevel() < m_info.numMipLevels );

    m_sparseTextures[deviceIndex].fillMipTail( stream, mipTailData, mipTailSize, handle, offset );
}

void DemandTextureImpl::unmapMipTail( unsigned int deviceIndex, CUstream stream ) const
{
    DEMAND_ASSERT( deviceIndex < m_sparseTextures.size() );
    m_sparseTextures[deviceIndex].unmapMipTail( stream );
}

// Fill the dense texture on the given device.
void DemandTextureImpl::fillDenseTexture( unsigned int deviceIndex, CUstream stream, const char* textureData, unsigned int width, unsigned int height )
{
    DEMAND_ASSERT( deviceIndex < m_denseTextures.size() );
    m_denseTextures[deviceIndex].fillTexture( stream, textureData, width, height );
}

// Set this texture as an entry point for a udim texture array.
void DemandTextureImpl::setUdimTexture( unsigned int udimStartPage, unsigned int udim, unsigned int vdim, bool isBaseTexture )
{
    m_sampler.desc.isUdimBaseTexture = isBaseTexture ? 1 : 0;
    m_sampler.udimStartPage          = udimStartPage;
    m_sampler.udim                   = udim;
    m_sampler.vdim                   = vdim;
}

size_t DemandTextureImpl::getMipTailSize() 
{ 
    DEMAND_ASSERT( m_isInitialized );
    return m_mipTailSize; 
}

}  // namespace demandLoading
