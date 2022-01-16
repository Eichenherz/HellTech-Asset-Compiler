#include "core_types.h"

#include <iostream>

#include <vector>
#include <assert.h>
#include <span>

#include <memory>



#define NOMINMAX
#include <Windows.h>

struct win_file
{
	HANDLE hndl = INVALID_HANDLE_VALUE;
	HANDLE mapping = INVALID_HANDLE_VALUE;
	UINT8* pMemView = 0;
	UINT64 size = -1;
	bool readOnly;
};

inline win_file WinMountMemMappedFile( const char* path )
{
	win_file dataFile = {};

	WIN32_FILE_ATTRIBUTE_DATA fileInfo = {};
	assert( GetFileAttributesEx( path, GetFileExInfoStandard, &fileInfo ) );
	assert( !( fileInfo.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY ) );

	LARGE_INTEGER fileSize = {};
	fileSize.LowPart = fileInfo.nFileSizeLow;
	fileSize.HighPart = fileInfo.nFileSizeHigh;
	assert( fileSize.QuadPart );

	dataFile.readOnly = fileInfo.dwFileAttributes & FILE_ATTRIBUTE_READONLY;
	dataFile.size = fileSize.QuadPart;

	DWORD accessMode = GENERIC_READ;
	DWORD flagsAndAttrs = FILE_ATTRIBUTE_READONLY;
	dataFile.hndl = CreateFile( path, accessMode, 0, 0, OPEN_EXISTING, flagsAndAttrs, 0 );
	dataFile.mapping = CreateFileMapping( dataFile.hndl, 0, PAGE_READONLY, 0, 0, 0 );

	dataFile.pMemView = ( UINT8* ) MapViewOfFile( dataFile.mapping, FILE_MAP_READ, 0, 0, dataFile.size );

	return dataFile;
}




void CompileGlbAssetToBinary( const std::span<u8> glbData, std::vector<u8>& drakAsset );

#include "job_system.h";

static std::unique_ptr<job_system> pJobSystem;

int main()
{
    pJobSystem = std::make_unique<job_system>();
    pJobSystem->Init();

    constexpr char path[] = "D:\\3d models\\Elemental.glb";
	win_file glbFile = WinMountMemMappedFile( path );

	std::vector<u8> out;
	CompileGlbAssetToBinary( std::span<u8>{ glbFile.pMemView, glbFile.size }, out );

	return 0;
}