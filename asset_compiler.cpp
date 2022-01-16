#include "core_types.h"

#include "meshoptimizer/meshoptimizer.h"
#define CGLTF_IMPLEMENTATION
#include "cgltf.h"
#include "spng.h"
#include "lz4.h"


#include "r_data_structs.h"

#include <iostream>
#include <cmath>
#include <vector>
#include <span>
#include <string_view>		
#include <assert.h>
#include <unordered_map>
#include <memory>
#include <numeric>

#include <functional>
#include <thread>
#include <atomic>

#define _XM_NO_XMVECTOR_OVERLOADS_
#include <DirectXMath.h>
#include <DirectXCollision.h>

#include "bcn_compressor.h"

#include "job_system.h"


#define BYTE_COUNT( buffer ) (size_t) std::size( buffer ) * sizeof( buffer[ 0 ] )

using namespace DirectX;

using float2 = DirectX::XMFLOAT2;
using float3 = DirectX::XMFLOAT3;
using float4 = DirectX::XMFLOAT4;


enum texture_format : u8
{
	TEXTURE_FORMAT_UNDEFINED,
	TEXTURE_FORMAT_RBGA8_SRGB,
	TEXTURE_FORMAT_RBGA8_UNORM,
	TEXTURE_FORMAT_BC1_RGB_SRGB,
	TEXTURE_FORMAT_BC5_UNORM,
	TEXTURE_FORMAT_COUNT
};
enum texture_type : u8
{
	TEXTURE_TYPE_1D,
	TEXTURE_TYPE_2D,
	TEXTURE_TYPE_3D,
	//TEXTURE_TYPE_CUBE,
	TEXTURE_TYPE_COUNT
};
enum pbr_type : u8
{
	PBR_TYPE_BASECOLOR,
	PBR_TYPE_NORMAL,
	PBR_TYPE_METALLICROUGHNESS,
	PBR_TYPE_COUNT
};
enum gltf_sampler_filter : u8
{
	GLTF_SAMPLER_FILTER_NEAREST = 0,
	GLTF_SAMPLER_FILTER_LINEAR = 1,
	GLTF_SAMPLER_FILTER_NEAREST_MIPMAP_NEAREST = 2,
	GLTF_SAMPLER_FILTER_LINEAR_MIPMAP_NEAREST = 3,
	GLTF_SAMPLER_FILTER_NEAREST_MIPMAP_LINEAR = 4,
	GLTF_SAMPLER_FILTER_LINEAR_MIPMAP_LINEAR
};
enum gltf_sampler_address_mode : u8
{
	GLTF_SAMPLER_ADDRESS_MODE_REPEAT = 0,
	GLTF_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE = 1,
	GLTF_SAMPLER_ADDRESS_MODE_MIRRORED_REPEAT = 2
};


struct sampler_config
{
	gltf_sampler_filter			min;
	gltf_sampler_filter			mag;
	gltf_sampler_address_mode	addrU;
	gltf_sampler_address_mode	addrV;
	//gltf_sampler_address_mode	addrW;
};

// TODO: remove range and nameHash ?
struct image_metadata
{
	u64				nameHash;
	range			texBinRange;
	u16				width;
	u16				height;
	texture_format	format;
	texture_type	type;
	u8				mipCount = 1;
	u8				layerCount = 1;
};

// TODO: count bytes or elements
struct drak_file_footer
{
	range meshesByteRange;
	range mtrlsByteRange;
	range imgsByteRange;
	range vtxByteRange;
	range idxByteRange;
	range texBinByteRange;
	range mletsByteRange;
	range mletsDataByteRange;

	u32	compressedSize;
	u32	originalSize;

	char magik[ 4 ] = "DRK";
	u32 drakVer = 0;
	u32 contentVer = 0;
};



// TODO: use DirectXMath ?
// TODO: fast ?
inline float SignNonZero( float e )
{
	return ( e >= 0.0f ) ? 1.0f : -1.0f;
}
inline float2 OctaNormalEncode( float3 n )
{
	// NOTE: Project the sphere onto the octahedron, and then onto the xy plane
	float absLen = std::fabs( n.x ) + std::fabs( n.y ) + std::fabs( n.z );
	float absNorm = ( absLen == 0.0f ) ? 0.0f : 1.0f / absLen;
	float nx = n.x * absNorm;
	float ny = n.y * absNorm;

	// NOTE: Reflect the folds of the lower hemisphere over the diagonals
	float octaX = ( n.z < 0.f ) ? ( 1.0f - std::fabs( ny ) ) * SignNonZero( nx ): nx;
	float octaY = ( n.z < 0.f ) ? ( 1.0f - std::fabs( nx ) ) * SignNonZero( ny ): ny;
	
	return { octaX, octaY };
}
// TODO: use angle between normals ?
inline float EncodeTanToAngle( float3 n, float3 t )
{
	using namespace DirectX;

	// NOTE: inspired by Doom Eternal
	float3 tanRef = ( std::abs( n.x ) > std::abs( n.z ) ) ?
		float3{ -n.y, n.x, 0.0f } :
		float3{ 0.0f, -n.z, n.y };

	float tanRefAngle = XMVectorGetX( 
		XMVector3AngleBetweenVectors( XMLoadFloat3( &t ), XMLoadFloat3( &tanRef ) ) );
	return XMScalarModAngle( tanRefAngle ) * XM_1DIVPI;
}

struct png_decoder
{
	spng_ctx* ctx;

	png_decoder( const u8* pngData, u64 pngSize ) : ctx{ spng_ctx_new( 0 ) }
	{
		// NOTE: ignore chunk CRC's 
		assert( !spng_set_crc_action( ctx, SPNG_CRC_USE, SPNG_CRC_USE ) );
		assert( !spng_set_png_buffer( ctx, pngData, pngSize ) );
	}
	~png_decoder() { spng_ctx_free( ctx ); }

};
inline u64 PngGetDecodedImageByteCount( const png_decoder& dcd )
{
	u64 outSize = 0;
	assert( !spng_decoded_image_size( dcd.ctx, SPNG_FMT_RGBA8, &outSize ) );

	return outSize;
}
inline u64 PngGetDecodedImageSize( const png_decoder& dcd )
{
	spng_ihdr ihdr = {};
	assert( !spng_get_ihdr( dcd.ctx, &ihdr ) );
	
	return u64( ihdr.width ) | ( u64( ihdr.height ) << 32 );
}
inline void PngDecodeImageFromMem( const png_decoder& dcd, u8* txBinOut, u64 txSize )
{
	assert( !spng_decode_image( dcd.ctx, txBinOut, txSize, SPNG_FMT_RGBA8, 0 ) );
}


inline gltf_sampler_filter GetSamplerFilter( i32 code )
{
	switch( code )
	{
	case 9728: return GLTF_SAMPLER_FILTER_NEAREST;
	case 9729: return GLTF_SAMPLER_FILTER_LINEAR;
	case 9984: return GLTF_SAMPLER_FILTER_NEAREST_MIPMAP_NEAREST;
	case 9985: return GLTF_SAMPLER_FILTER_LINEAR_MIPMAP_NEAREST;
	case 9986: return GLTF_SAMPLER_FILTER_NEAREST_MIPMAP_LINEAR;
	case 9987: return GLTF_SAMPLER_FILTER_LINEAR_MIPMAP_LINEAR;
	}
}
inline gltf_sampler_address_mode GetSamplerAddrMode( i32 code )
{
	switch( code )
	{
	case 10497: return GLTF_SAMPLER_ADDRESS_MODE_REPEAT;
	case 33071: return GLTF_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
	case 33648: return GLTF_SAMPLER_ADDRESS_MODE_MIRRORED_REPEAT;
	}
}

// NOTE: will assume: pos,normal,tan are vec3 and uv is vec2
struct _mesh
{
	range	posStreamRange;
	range	normalStreamRange;
	range	tanStreamRange;
	range	uvsStreamRange;
	range	idxRange;
	float	aabbMin[ 3 ];
	float	aabbMax[ 3 ];
	u16		mtlIdx;
};


inline DirectX::XMMATRIX CgltfNodeGetTransf( const cgltf_node* node )
{
	using namespace DirectX;

	XMMATRIX t = {};
	if( node->has_rotation || node->has_translation || node->has_scale )
	{
		XMVECTOR move = XMLoadFloat3( (const XMFLOAT3*) node->translation );
		XMVECTOR rot = XMLoadFloat4( (const XMFLOAT4*) node->rotation );
		XMVECTOR scale = XMLoadFloat3( (const XMFLOAT3*) node->scale );
		t = XMMatrixAffineTransformation( scale, XMVectorSet( 0, 0, 0, 1 ), rot, move );
	}
	else if( node->has_matrix )
	{
		// NOTE: gltf matrices are stored in col maj
		t = XMMatrixTranspose( XMLoadFloat4x4( (const XMFLOAT4X4*) node->matrix ) );
	}

	return t;
}
inline u64 CgltfCompTypeByteCount( cgltf_component_type compType )
{
	switch( compType )
	{
	case cgltf_component_type_r_8: case cgltf_component_type_r_8u: return 1;
	case cgltf_component_type_r_16: case cgltf_component_type_r_16u: return 2;
	case cgltf_component_type_r_32u: case cgltf_component_type_r_32f: return 4;
	case cgltf_component_type_invalid: return -1;
	}
}
inline float CgltfReadFloat( const u8* data, cgltf_component_type compType )
{
	switch( compType )
	{
	case cgltf_component_type_invalid: default: return NAN;
	case cgltf_component_type_r_8: return float( *(const i8*) data );
	case cgltf_component_type_r_8u: return float( *(const u8*) data );
	case cgltf_component_type_r_16: return float( *(const i16*) data );
	case cgltf_component_type_r_16u: return float( *(const u16*) data );
	case cgltf_component_type_r_32u: return float( *(const u32*) data );
	case cgltf_component_type_r_32f: return *(const float*) data;
	}
}

struct raw_texture_extents
{
	u16				width;
	u16				height;
	//texture_type	type;
};

inline raw_texture_extents CgltfDecodeTexture( const cgltf_texture& t, const u8* pBin, std::vector<u8>& data )
{
	using namespace std;
	u64 imgOffset = t.image->buffer_view->offset;
	u64 imgSize = t.image->buffer_view->size;

	std::string_view mimeType = { t.image->mime_type };
	if( mimeType == "image/png"sv )
	{
		png_decoder dcd( pBin + imgOffset, imgSize );
		u64 imageByteCount = PngGetDecodedImageByteCount( dcd );
		u64 widthHeight = PngGetDecodedImageSize( dcd );

		data.resize( imageByteCount );
		PngDecodeImageFromMem( dcd, std::data( data ), imageByteCount );

		return { .width = u16( widthHeight & u32( -1 ) ), .height = u16( widthHeight >> 32 ) };
	}
	assert( 0 );
}

struct exported_texture
{
	image_metadata metadata;
	std::vector<u8> data;
};

struct material
{
	u16 baseColIdx;
	u16 occRoughMetalIdx;
	u16 normalMapIdx;
	XMFLOAT3 baseColFactor;
	float metallicFactor;
	float roughnessFactor;
};

struct imported_mesh
{
	std::vector<XMFLOAT3> positions;
	std::vector<XMFLOAT3> normals;
	std::vector<XMFLOAT3> tangents;
	std::vector<XMFLOAT2> texCoords;
	std::vector<u32> indexBuffer;

	XMFLOAT3 aabbMin;
	XMFLOAT3 aabbMax;

	cgltf_component_type indexType;
};

struct node
{
	std::vector<u16> meshIndices;
	std::vector<u16> materialIndices;
	XMFLOAT4X4A transformation;
	u16 parent;
};

// TODO: rethink samplers
// TODO: more ?
static void
LoadGlbFile(
	const std::span<u8>			glbData,
	std::vector<float>&				meshAttrs,
	std::vector<u32>&				indices,
	std::vector<u8>&				texBin,
	std::vector<image_metadata>&	imgDescs, 
	std::vector<material_data>&		mtrlDescs,
	std::vector<_mesh>&		meshDescs
){
	using namespace DirectX;
	using texture_key = const cgltf_texture const*;
	using material_key = const cgltf_material const*;
	using mesh_key = const cgltf_primitive const*;


	cgltf_options options = { .type = cgltf_file_type_glb };
	cgltf_data* data = 0;
	assert( !cgltf_parse( &options, std::data( glbData ), std::size( glbData ), &data ) );
	assert( !cgltf_validate( data ) );
	assert( data->scenes_count == 1 );
	const u8* pBin = (const u8*) data->bin;

	std::unordered_map<texture_key, u16> textureLookup;
	std::vector<exported_texture*> textures;

	std::unordered_map<material_key, u16> materialLookup;
	std::vector<material> materials;

	std::unordered_map<mesh_key, u16> meshPrimitivesLookup;
	std::vector<imported_mesh*> importredMeshes;

	std::vector<std::function<void()>> jobs;

	for( u64 mi = 0; mi < data->materials_count; ++mi )
	{
		// TODO: assert pbrMaterial has at least one of the textures  ?
		const cgltf_material& mtrl = data->materials[ mi ];
		if( !mtrl.has_pbr_metallic_roughness ) continue;


		u16 thisMaterialIdx = std::size( materials );
		materialLookup.insert( { &mtrl, thisMaterialIdx } );
		materials.push_back( {} );
		
		material& thisMaterial = materials[ thisMaterialIdx ];
		thisMaterial.baseColIdx = -1;
		thisMaterial.occRoughMetalIdx = -1;
		thisMaterial.normalMapIdx = -1;

		const cgltf_pbr_metallic_roughness& pbrMetallicRoughness = mtrl.pbr_metallic_roughness;
		thisMaterial.baseColFactor = ( const XMFLOAT3& ) pbrMetallicRoughness.base_color_factor;
		thisMaterial.metallicFactor = pbrMetallicRoughness.metallic_factor;
		thisMaterial.roughnessFactor = pbrMetallicRoughness.roughness_factor;

		if( const cgltf_texture* pbrBaseCol = pbrMetallicRoughness.base_color_texture.texture )
		{
			if( textureLookup.find( pbrBaseCol ) == std::cend( textureLookup ) )
			{
				u16 thisTextureIdx = std::size( textures );
				textureLookup.insert( { pbrBaseCol,thisTextureIdx } );
				textures.push_back( new exported_texture );

				thisMaterial.baseColIdx = thisTextureIdx;
				auto& thisTexture = *textures[ thisTextureIdx ];
				jobs.push_back(
					[pbrBaseCol, pBin, &thisTexture]()
					{
						std::vector<u8>& outData = thisTexture.data;
						std::vector<u8> rawData;

						raw_texture_extents raw = CgltfDecodeTexture( *pbrBaseCol, pBin, rawData );
						outData.resize( GetBCTexByteCount( raw.width, raw.height, bc1BytesPerBlock ) );
						CompressToBc1_SIMD( std::data( rawData ), raw.width, raw.height, std::data( outData ) );
						
						thisTexture.metadata = {
							.width = raw.width,
							.height = raw.height,
							.format = TEXTURE_FORMAT_BC1_RGB_SRGB,
							.type = TEXTURE_TYPE_2D,
							.mipCount = 1,
							.layerCount = 1
						};
					}
				);
			}
		}

		if( const cgltf_texture* metalRoughMap = pbrMetallicRoughness.metallic_roughness_texture.texture )
		{
			if( textureLookup.find( metalRoughMap ) == std::cend( textureLookup ) )
			{
				u16 thisTextureIdx = std::size( textures );
				textureLookup.insert( { metalRoughMap,thisTextureIdx } );
				textures.push_back( new exported_texture );

				thisMaterial.occRoughMetalIdx = thisTextureIdx;
				auto& thisTexture = *textures[ thisTextureIdx ];
				jobs.push_back(
					[metalRoughMap, pBin, &thisTexture]()
					{
						std::vector<u8>& outData = thisTexture.data;
						std::vector<u8> rawData;

						raw_texture_extents raw = CgltfDecodeTexture( *metalRoughMap, pBin, rawData );
						outData.resize( GetBCTexByteCount( raw.width, raw.height, bc5BytesPerBlock ) );
						CompressMetalRoughMapToBc5_SIMD( std::data( rawData ), raw.width, raw.height, std::data( outData ) );
						
						thisTexture.metadata = {
							.width = raw.width,
							.height = raw.height,
							.format = TEXTURE_FORMAT_BC5_UNORM,
							.type = TEXTURE_TYPE_2D,
							.mipCount = 1,
							.layerCount = 1
						};
					}
				);
			}
		}

		if( const cgltf_texture* normalMap = mtrl.normal_texture.texture )
		{
			if( textureLookup.find( normalMap ) == std::cend( textureLookup ) )
			{
				u16 thisTextureIdx = std::size( textures );
				textureLookup.insert( { normalMap,thisTextureIdx } );
				textures.push_back( new exported_texture );

				thisMaterial.normalMapIdx = thisTextureIdx;
				auto& thisTexture = *textures[ thisTextureIdx ];
				jobs.push_back(
					[normalMap, pBin, &thisTexture]()
					{
						std::vector<u8>& outData = thisTexture.data;
						std::vector<u8> rawData;

						raw_texture_extents raw = CgltfDecodeTexture( *normalMap, pBin, rawData );
						outData.resize( GetBCTexByteCount( raw.width, raw.height, bc5BytesPerBlock ) );
						CompressMetalRoughMapToBc5_SIMD( std::data( rawData ), raw.width, raw.height, std::data( outData ) );
						
						thisTexture.metadata = {
							.width = raw.width,
							.height = raw.height,
							.format = TEXTURE_FORMAT_BC5_UNORM,
							.type = TEXTURE_TYPE_2D,
							.mipCount = 1,
							.layerCount = 1
						};
					}
				);
			}
		}
	}

	for( u64 i = 0; i < std::size( jobs ); ++i )
	{
		pJobSystem->Execute( jobs[ i ] );
	}
	

	for( u64 mi = 0; mi < data->meshes_count; ++mi )
	{
		const cgltf_mesh& thisMesh = data->meshes[ mi ];
		for( u64 pi = 0; pi < thisMesh.primitives_count; ++pi )
		{
			const cgltf_primitive& prim = thisMesh.primitives[ pi ];

			u16 thisMeshPrimitiveIdx = std::size( importredMeshes );
			importredMeshes.push_back( new imported_mesh );

			// NOTE: will assume all these prims are unique
			meshPrimitivesLookup.insert( { &prim,thisMeshPrimitiveIdx } );

			imported_mesh& m = *importredMeshes[ thisMeshPrimitiveIdx ];

			// NOTE: attrs must have the same count 
			u64 primVtxCount = prim.attributes[ 0 ].data->count;
			for( u64 ai = 0; ai < prim.attributes_count; ++ai ) assert( primVtxCount == prim.attributes[ ai ].data->count );

			for( u64 ai = 0; ai < prim.attributes_count; ++ai )
			{
				const cgltf_attribute& vtxAttr = prim.attributes[ ai ];

				u64 compByteCount = CgltfCompTypeByteCount( vtxAttr.data->component_type );
				u64 attrCompCount = cgltf_num_components( vtxAttr.data->type );
				u64 attrStride = vtxAttr.data->stride;
				u64 attrOffset = vtxAttr.data->offset;
				u64 attrSrcOffset = vtxAttr.data->buffer_view->offset;
				u64 attrDataCount = vtxAttr.data->count;

				if( vtxAttr.type == cgltf_attribute_type_position )
				{
					assert( sizeof( decltype( m.positions )::value_type ) == ( compByteCount * attrCompCount ) );

					m.positions.resize( attrDataCount );
					const u8* pFirstPosData = pBin + attrSrcOffset + attrOffset;
					std::memcpy( std::data( m.positions ), pFirstPosData, attrDataCount* attrStride );

					if( vtxAttr.data->has_min && vtxAttr.data->has_max )
					{
						m.aabbMin = ( const XMFLOAT3& ) vtxAttr.data->min;
						m.aabbMax = ( const XMFLOAT3& ) vtxAttr.data->max;
					}
					else
					{
						constexpr float floatMin = std::numeric_limits<float>::min();
						constexpr float floatMax = std::numeric_limits<float>::max();

						XMFLOAT3 min = { floatMin, floatMin, floatMin };
						XMFLOAT3 max = { floatMax, floatMax, floatMax };
						for( XMFLOAT3& v : m.positions )
						{
							if( v.x > max.x ) { max.x = v.x; }
							if( v.x < min.x ) { min.x = v.x; }

							if( v.y > max.y ) { max.y = v.y; }
							if( v.y < min.y ) { min.y = v.y; }

							if( v.z > max.z ) { max.z = v.z; }
							if( v.z < min.z ) { min.z = v.z; }
						}
					}
				}

				else if( vtxAttr.type == cgltf_attribute_type_normal )
				{
					assert( sizeof( decltype( m.normals )::value_type ) == ( compByteCount * attrCompCount ) );

					m.normals.resize( attrDataCount );
					const u8* pFirstNormData = pBin + attrSrcOffset + attrOffset;
					std::memcpy( std::data( m.normals ), pFirstNormData, attrDataCount* attrStride );
				}

				else if( vtxAttr.type == cgltf_attribute_type_tangent )
				{
					assert( attrCompCount == 4 );
					assert( sizeof( decltype( m.tangents )::value_type ) == ( compByteCount * ( attrCompCount - 1 ) ) );

					m.tangents.resize( attrDataCount );
					const XMFLOAT4* pFirstTanData = ( const XMFLOAT4* ) ( pBin + attrSrcOffset + attrOffset );
					for( u64 ti = 0; ti < attrDataCount; ++ti )
					{
						XMFLOAT4 thisTan = pFirstTanData[ ti ];
						m.tangents[ ti ] = {
							thisTan.x * thisTan.w,
							thisTan.y * thisTan.w,
							thisTan.z * thisTan.w
						};
					}
				}

				else if( vtxAttr.type == cgltf_attribute_type_texcoord )
				{
					assert( sizeof( decltype( m.texCoords )::value_type ) == ( compByteCount * attrCompCount ) );

					m.texCoords.resize( attrDataCount );
					const u8* pFirstTexCoordData = pBin + attrSrcOffset + attrOffset;
					std::memcpy( std::data( m.texCoords ), pFirstTexCoordData, attrDataCount * attrStride );
				}
			}

			if( prim.indices )
			{
				m.indexBuffer.resize( prim.indices->count );
				m.indexType = prim.indices->component_type;
				const u8* idxSrc = pBin + prim.indices->buffer_view->offset;
				u64 idxStride = prim.indices->stride;
				for( u64 i = 0; i < prim.indices->count; ++i )
				{
					u64 idx = cgltf_component_read_index( idxSrc + idxStride * i, prim.indices->component_type );
					m.indexBuffer[ i ] = u32( idx );
				}
			}
		}
	}


	std::vector<node> nodeTransf( data->nodes_count );

	const cgltf_scene& scene = *data->scene;
	for( u64 n = 0; n < data->nodes_count; ++n )
	{
		const cgltf_node* node = data->nodes + n;

		XMMATRIX t = CgltfNodeGetTransf( node );

		for( const cgltf_node* parent = node->parent;
			 parent;
			 parent = parent->parent )
		{
			t = XMMatrixMultiply( t, CgltfNodeGetTransf( parent ) );
		}
		//XMStoreFloat4x4( &nodeTransf[ n ], t );
	}


	pJobSystem->Wait();

	cgltf_free( data );
}

// TODO: avoid template madness ?
// TODO: mesh triangulate ?
template<typename T>
u64 MeshoptReindexMesh( std::span<T> vtxSpan, std::span<u32> idxSpan )
{
	T*      vertices = std::data( vtxSpan );
	u32*	indices = std::data( idxSpan );
	u64		vtxCount = std::size( vtxSpan );
	u64		idxCount = std::size( idxSpan );

	std::vector<u32> remap( vtxCount );
	u64 newVtxCount = meshopt_generateVertexRemap( 
		std::data( remap ), indices, idxCount, vertices, vtxCount, sizeof( vertices[ 0 ] ) );

	assert( newVtxCount <= vtxCount );
	if( newVtxCount == vtxCount ) return newVtxCount;

	meshopt_remapIndexBuffer( indices, indices, idxCount, std::data( remap ) );
	meshopt_remapVertexBuffer( vertices, vertices, vtxCount, sizeof( vertices[ 0 ] ), std::data( remap ) );
	return newVtxCount;
}

template<typename T> float* GetCompX( T* );
template<> inline float* GetCompX( vertex* v ){ return &( v->px ); }
template<> inline float* GetCompX( DirectX::XMFLOAT3* v ){ return &( v->x ); }

template<typename T>
void MeshoptOptimizeMesh( std::span<T> vtxSpan, std::span<u32> idxSpan )
{
	T*      vertices = std::data( vtxSpan );
	u32*    indices = std::data( idxSpan );
	u64		vtxCount = std::size( vtxSpan );
	u64		idxCount = std::size( idxSpan );

	meshopt_optimizeVertexCache( indices, indices, idxCount, vtxCount );
	//meshopt_optimizeOverdraw( indices, indices, idxCount, &vertices[ 0 ].px, vtxCount, sizeof( vertices[ 0 ] ), 1.05f );
	meshopt_optimizeOverdraw( indices, indices, idxCount, GetCompX( &vertices[ 0 ] ), vtxCount, sizeof( vertices[ 0 ] ), 1.05f );
	meshopt_optimizeVertexFetch( vertices, indices, idxCount, vertices, vtxCount, sizeof( vertices[ 0 ] ) );
}

template u64 MeshoptReindexMesh( std::span<DirectX::XMFLOAT3> vtxSpan, std::span<u32> idxSpan );
template void MeshoptOptimizeMesh( std::span<DirectX::XMFLOAT3> vtxSpan, std::span<u32> idxSpan );

constexpr u64 MAX_VTX = 128;
constexpr u64 MAX_TRIS = 256;
constexpr float CONE_WEIGHT = 0.8f;
// TODO: fix C++ constness thing
// TODO: data offseting for more meshes
static void MeshoptMakeMeshlets(
	const std::span<vertex>	    vtxSpan,
	const std::vector<u32>& lodIndices,
	const std::vector<range>& lods,
	std::vector<meshlet>& mletsDesc,
	std::vector<u32>& mletsData,
	std::vector<range>& outMeshletLods
){
	using namespace DirectX;

	std::vector<u32> meshletData;

	std::vector<meshopt_Meshlet> meshlets;
	std::vector<u32> mletVtx;
	std::vector<u8> mletIdx;

	for( range lodRange : lods )
	{
		const std::span<u32> lodIdx = { const_cast< u32* >( std::data( lodIndices ) ) + lodRange.offset,lodRange.size };

		u64 maxMeshletCount = meshopt_buildMeshletsBound( std::size( lodIdx ), MAX_VTX, MAX_TRIS );
		meshlets.resize( maxMeshletCount );
		mletVtx.resize( maxMeshletCount * MAX_VTX );
		mletIdx.resize( maxMeshletCount * MAX_TRIS * 3 );

		u64 meshletCount = meshopt_buildMeshlets( std::data( meshlets ), std::data( mletVtx ), std::data( mletIdx ),
												  std::data( lodIdx ), std::size( lodIdx ),
												  &vtxSpan[ 0 ].px, std::size( vtxSpan ), sizeof( vtxSpan[ 0 ] ),
												  MAX_VTX, MAX_TRIS, CONE_WEIGHT );

		const meshopt_Meshlet& last = meshlets[ meshletCount - 1 ];

		meshlets.resize( meshletCount );
		mletVtx.resize( last.vertex_offset + last.vertex_count );
		mletIdx.resize( last.triangle_offset + ( ( last.triangle_count * 3 + 3 ) & ~3 ) );
		meshletData.reserve( std::size( meshletData ) + std::size( mletVtx ) + std::size( mletVtx ) );

		u64 mletOffset = std::size( mletsDesc );
		outMeshletLods.push_back( { mletOffset,meshletCount } );

		for( const meshopt_Meshlet& m : meshlets )
		{
			u64 dataOffset = std::size( meshletData );

			for( u64 i = 0; i < m.vertex_count; ++i )
			{
				meshletData.push_back( mletVtx[ i + m.vertex_offset ] );
			}

			const u32* indexGroups = ( const u32* ) ( std::data( mletIdx ) + m.triangle_offset );
			u64 indexGroupCount = ( m.triangle_count * 3 + 3 ) / 4;
			for( u64 i = 0; i < indexGroupCount; ++i )
			{
				meshletData.push_back( indexGroups[ i ] );
			}

			meshopt_Bounds bounds = meshopt_computeMeshletBounds(
				std::data( mletVtx ), std::data( mletIdx ), m.triangle_count,
				&vtxSpan[ 0 ].px, std::size( vtxSpan ), sizeof( vtxSpan[ 0 ] ) );

			// TODO: don't copy ?
			XMFLOAT3 mletVertices[ MAX_VTX ] = {};
			for( u64 vi = 0; vi < m.vertex_count; ++vi )
			{
				const vertex& v = vtxSpan[ mletVtx[ vi + m.vertex_offset ] ];
				mletVertices[ vi ] = { v.px,v.py,v.pz };
			}

			BoundingBox aabb = {};
			BoundingBox::CreateFromPoints( aabb, m.vertex_count, mletVertices, sizeof( mletVertices[ 0 ] ) );

			meshlet data = {};
			data.center = aabb.Center;
			data.extent = aabb.Extents;
			data.coneAxis = *( const XMFLOAT3* ) bounds.cone_axis;
			data.coneApex = *( const XMFLOAT3* ) bounds.cone_apex;
			data.coneX = bounds.cone_axis_s8[ 0 ];
			data.coneY = bounds.cone_axis_s8[ 1 ];
			data.coneZ = bounds.cone_axis_s8[ 2 ];
			data.coneCutoff = bounds.cone_cutoff_s8;
			data.vertexCount = m.vertex_count;
			data.triangleCount = m.triangle_count;
			data.dataOffset = dataOffset;
			mletsDesc.push_back( data );
		}

		mletsData.insert( std::end( mletsData ), std::begin( meshletData ), std::end( meshletData ) );
	}
}

constexpr u64 lodMaxCount = 1;
// TODO: indicesOut offset ?
inline u64 MeshoptMakeMeshLods(
	const std::span<vertex> verticesView,
	const std::span<u32>	indicesView,
	u32*					indicesOut,
	std::vector<range>&	    meshLods
){
	constexpr float ERROR_THRESHOLD = 1e-2f;
	constexpr float reductionFactor = 0.85f;

	assert( meshLods[ 0 ].size );

	u64 totalIndexCount = meshLods[ 0 ].size;
	u64 meshLodsCount = 1;
	for( ; meshLodsCount < std::size( meshLods ); ++meshLodsCount )
	{
		const range& prevLod = meshLods[ meshLodsCount - 1 ];
		const u32* prevIndices = indicesOut + prevLod.offset;
		u32 nextIndicesOffset = prevLod.offset + prevLod.size;
		u32* nextIndices = indicesOut + nextIndicesOffset;

		u64 nextIndicesCount = meshopt_simplify( nextIndices,
												 prevIndices,
												 prevLod.size,
												 &verticesView[ 0 ].px,
												 std::size( verticesView ),
												 sizeof( verticesView[ 0 ] ),
												 float( prevLod.size ) * reductionFactor,
												 ERROR_THRESHOLD );

		assert( nextIndicesCount <= prevLod.size );

		meshopt_optimizeVertexCache( nextIndices, nextIndices, nextIndicesCount, std::size( verticesView ) );
		// NOTE: reached the error bound
		if( nextIndicesCount == prevLod.size ) break;

		meshLods[ meshLodsCount ].size = nextIndicesCount;
		meshLods[ meshLodsCount ].offset = nextIndicesOffset;

		totalIndexCount += nextIndicesCount;
	}

	meshLods.resize( meshLodsCount );
	
	return totalIndexCount;
}

// TODO: world handedness
// TODO: use u16 idx
// TODO: quantize pos + uvs
// TODO: revisit index offsets and stuff
// TODO: expose mem ops ?
static std::pair<range, range> AssembleAndOptimizeMesh(
	const std::vector<float>& attrStreams,
	const std::vector<u32>&   importedIndices,
	const _mesh&      rawMesh,
	std::vector<vertex>&      vertices,
	std::vector<u32>&         indices
){
	u64 vtxAttrCount = rawMesh.posStreamRange.size / 3;
	u64 vtxOffset = std::size( vertices );
	vertices.resize( vtxOffset + vtxAttrCount );

	vertex* firstVertex = &vertices[ vtxOffset ];
	for( u64 i = 0; i < vtxAttrCount; ++i )
	{
		const float* posStream = std::data( attrStreams ) + rawMesh.posStreamRange.offset;

		firstVertex[ i ].px = -posStream[ i * 3 + 0 ];
		firstVertex[ i ].py = posStream[ i * 3 + 1 ];
		firstVertex[ i ].pz = posStream[ i * 3 + 2 ];
	}
	for( u64 i = 0; i < vtxAttrCount; ++i )
	{
		const float* normalStream = std::data( attrStreams ) + rawMesh.normalStreamRange.offset;
		const float* tanStream = std::data( attrStreams ) + rawMesh.tanStreamRange.offset;

		float nx = -normalStream[ i * 3 + 0 ];
		float ny = normalStream[ i * 3 + 1 ];
		float nz = normalStream[ i * 3 + 2 ];
		float tx = tanStream[ i * 3 + 0 ];
		float ty = tanStream[ i * 3 + 1 ];
		float tz = tanStream[ i * 3 + 2 ];

		DirectX::XMFLOAT2 octaNormal = OctaNormalEncode( { nx,ny,nz } );
		float tanAngle = EncodeTanToAngle( { nx,ny,nz }, { tx,ty,tz } );

		i8 snormNx = meshopt_quantizeSnorm( octaNormal.x, 8 );
		i8 snormNy = meshopt_quantizeSnorm( octaNormal.y, 8 );
		i8 snormTanAngle = meshopt_quantizeSnorm( tanAngle, 8 );

		u32 bitsSnormNx = *( u8* ) &snormNx;
		u32 bitsSnormNy = *( u8* ) &snormNy;
		u32 bitsSnormTanAngle = *( u8* ) &snormTanAngle;

		u32 packedTanFrame = bitsSnormNx | ( bitsSnormNy << 8 ) | ( bitsSnormTanAngle << 16 );

		firstVertex[ i ].snorm8octTanFrame = packedTanFrame;
		
	}
	for( u64 i = 0; i < vtxAttrCount; ++i )
	{
		const float* uvsStream = std::data( attrStreams ) + rawMesh.uvsStreamRange.offset;

		firstVertex[ i ].tu = uvsStream[ i * 2 + 0 ];
		firstVertex[ i ].tv = uvsStream[ i * 2 + 1 ];
	}

	u64 idxOffset = std::size( indices );
	indices.resize( idxOffset + rawMesh.idxRange.size * lodMaxCount );

	for( u64 i = 0; i < rawMesh.idxRange.size; ++i )
	{
		indices[ idxOffset + i ] = importedIndices[ rawMesh.idxRange.offset + i ] + vtxOffset;
	}

	// NOTE: optimize and lod
	u64 newVtxCount = MeshoptReindexMesh( 
		std::span<vertex>{ std::data( vertices ) + vtxOffset,vtxAttrCount }, 
		{ std::data( indices ) + idxOffset, rawMesh.idxRange.size } );
	vertices.resize( vtxOffset + newVtxCount );
	MeshoptOptimizeMesh( std::span<vertex>{ firstVertex,vtxAttrCount }, { std::data( indices ) + idxOffset, rawMesh.idxRange.size } );

	return{ { vtxOffset, u32( std::size( vertices ) - vtxOffset ) }, { idxOffset, rawMesh.idxRange.size } };
}


// TODO: more efficient copy
// TODO: better binary file design ?
void CompileGlbAssetToBinary( 
	const std::span<u8>	glbData, 
	std::vector<u8>&		drakAsset
){
	using namespace DirectX;

	std::vector<float>				meshAttrs;
	std::vector<u32>				rawIndices;
	std::vector<_mesh>		rawMeshDescs;

	std::vector<vertex>				vertices;
	std::vector<u32>				indices;
	std::vector<u8>					texBin;
	std::vector<mesh_desc>	        meshDescs;
	std::vector<material_data>		mtrlDescs;
	std::vector<image_metadata>		imgDescs;

	std::vector<meshlet>			mlets;
	std::vector<u32>				mletsData;

	LoadGlbFile( glbData, meshAttrs, rawIndices, texBin, imgDescs, mtrlDescs, rawMeshDescs );

	meshDescs.reserve( std::size( rawMeshDescs ) );
	// TODO: expose lod loop ?
	for( const _mesh& m : rawMeshDescs )
	{
		{
			u32 posAttrCount = m.posStreamRange.size / 3;
			u32 normalAttrCount = m.normalStreamRange.size / 3;
			u32 tanAttrCount = m.tanStreamRange.size / 3;
			u32 uvsAttrCount = m.uvsStreamRange.size / 2;
			assert( ( posAttrCount == normalAttrCount ) && ( posAttrCount == tanAttrCount ) && ( posAttrCount == uvsAttrCount ) );

			assert( sizeof( rawIndices[ 0 ] ) == sizeof( indices[ 0 ] ) );
		}

		auto[ vtxRange, idxRange ] = AssembleAndOptimizeMesh( meshAttrs, rawIndices, m, vertices, indices );

		std::vector<range> idxLods( lodMaxCount );
		idxLods[ 0 ] = idxRange;
		u64 totalIndexCount = MeshoptMakeMeshLods(
			{ std::data( vertices ) + vtxRange.offset,vtxRange.size }, { std::data( indices ) + idxRange.offset, idxRange.size },
			std::data( indices ),
			idxLods );
		indices.resize( idxRange.offset + totalIndexCount );

		const std::span<vertex> vtxSpan = { std::data( vertices ) + vtxRange.offset,vtxRange.size };
		std::vector<range> meshletLods;
		MeshoptMakeMeshlets( vtxSpan, indices, idxLods, mlets, mletsData, meshletLods );

		assert( std::size( idxLods ) == std::size( meshletLods ) );

		meshDescs.push_back( {} );

		mesh_desc& meshOut = meshDescs[ std::size( meshDescs ) - 1 ];
		meshOut.vertexCount = vtxRange.size;
		meshOut.vertexOffset = vtxRange.offset;
		meshOut.lodCount = std::size( idxLods );
		for( u64 l = 0; l < std::size( idxLods ); ++l )
		{
			meshOut.lods[ l ].indexCount = idxLods[ l ].size;
			meshOut.lods[ l ].indexOffset = idxLods[ l ].offset;
			meshOut.lods[ l ].meshletCount = meshletLods[ l ].size;
			meshOut.lods[ l ].meshletOffset = meshletLods[ l ].offset;
		}

		meshOut.aabbMin = { -m.aabbMin[ 0 ], m.aabbMin[ 1 ], m.aabbMin[ 2 ] };
		meshOut.aabbMax = { -m.aabbMax[ 0 ], m.aabbMax[ 1 ], m.aabbMax[ 2 ] };
		
		{
			XMVECTOR xmm0 = XMLoadFloat3( &meshOut.aabbMin );
			XMVECTOR xmm1 = XMLoadFloat3( &meshOut.aabbMax );
			XMVECTOR center = XMVectorScale( XMVectorAdd( xmm1, xmm0 ), 0.5f );
			XMVECTOR extent = XMVectorAbs( XMVectorScale( XMVectorSubtract( xmm1, xmm0 ), 0.5f ) );
			XMStoreFloat3( &meshOut.center, center );
			XMStoreFloat3( &meshOut.extent, extent );
			//XMStoreFloat3( &out.aabbMin, XMVectorAdd( center, extent ) );
			//XMStoreFloat3( &out.aabbMax, XMVectorSubtract( center, extent ) );
		}

		//meshOut.materialIndex = rawMesh.mtlIdx;
	}

	// TODO: assert that none of these overflow 2gbs
	u64 totalDataSize = 
		BYTE_COUNT( meshDescs ) + BYTE_COUNT( mtrlDescs ) + BYTE_COUNT( imgDescs ) + 
		BYTE_COUNT( vertices ) + BYTE_COUNT( indices ) + BYTE_COUNT( texBin ) +
		BYTE_COUNT( mlets ) + BYTE_COUNT( mletsData );

	drak_file_footer fileFooter = {};
	fileFooter.compressedSize = totalDataSize;
	fileFooter.originalSize = totalDataSize;
	
	std::vector<u8> outData( fileFooter.originalSize + +sizeof( fileFooter ) );
	u8* pOutData = std::data( outData );
	const u8* pDataBegin = std::data( outData );

	fileFooter.meshesByteRange = { u32( pOutData - pDataBegin ),BYTE_COUNT( meshDescs ) };
	pOutData = ( u8* ) std::memcpy( pOutData, std::data( meshDescs ), BYTE_COUNT( meshDescs ) ) + BYTE_COUNT( meshDescs );

	fileFooter.mtrlsByteRange = { u32( pOutData - pDataBegin ),BYTE_COUNT( mtrlDescs ) };
	pOutData = ( u8* ) std::memcpy( pOutData, std::data( mtrlDescs ), BYTE_COUNT( mtrlDescs ) ) + BYTE_COUNT( mtrlDescs );

	fileFooter.imgsByteRange = { u32( pOutData - pDataBegin ),BYTE_COUNT( imgDescs ) };
	pOutData = ( u8* ) std::memcpy( pOutData, std::data( imgDescs ), BYTE_COUNT( imgDescs ) ) + BYTE_COUNT( imgDescs );

	fileFooter.vtxByteRange = { u32( pOutData - pDataBegin ),BYTE_COUNT( vertices ) };
	pOutData = ( u8* ) std::memcpy( pOutData, std::data( vertices ), BYTE_COUNT( vertices ) ) + BYTE_COUNT( vertices );
	
	fileFooter.idxByteRange = { u32( pOutData - pDataBegin ),BYTE_COUNT( indices ) };
	pOutData = ( u8* ) std::memcpy( pOutData, std::data( indices ), BYTE_COUNT( indices ) ) + BYTE_COUNT( indices );
	
	fileFooter.mletsByteRange = { u32( pOutData - pDataBegin ),BYTE_COUNT( mlets ) };
	pOutData = ( u8* ) std::memcpy( pOutData, std::data( mlets ), BYTE_COUNT( mlets ) ) + BYTE_COUNT( mlets );
	
	fileFooter.mletsDataByteRange = { u32( pOutData - pDataBegin ),BYTE_COUNT( mletsData ) };
	pOutData = ( u8* ) std::memcpy( pOutData, std::data( mletsData ), BYTE_COUNT( mletsData ) ) + BYTE_COUNT( mletsData );
	
	fileFooter.texBinByteRange = { u32( pOutData - pDataBegin ),BYTE_COUNT( texBin ) };
	pOutData = ( u8* ) std::memcpy( pOutData, std::data( texBin ), BYTE_COUNT( texBin ) ) + BYTE_COUNT( texBin );
	
	*( drak_file_footer*) ( std::data( outData ) + totalDataSize ) = fileFooter;

	drakAsset = std::move( outData );
}