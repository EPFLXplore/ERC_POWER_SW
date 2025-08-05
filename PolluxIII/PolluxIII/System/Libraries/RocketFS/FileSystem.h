/*
 * filesystem.h
 *
 *  Created on: 14 Oct 2019
 *      Author: Arion
 */

#ifndef INC_FILESYSTEM_H_
#define INC_FILESYSTEM_H_

#include "File.h"
#include "IO.h"
#include "Stream.h"

#include <stdbool.h>
#include <stdint.h>


/*
 * FS-specific defines
 */
#define NUM_BLOCKS 4080
#define NUM_FILES 16
#define PROTECTED_BLOCKS 8



/*
 * Since stm32f446 only has 128KB memory, we cannot afford more than 16-bytes data-blocks...
 */
typedef struct DataBlock {
	uint16_t successor;
} DataBlock;


typedef enum StreamMode { OVERWRITE, APPEND } StreamMode;


class FileSystem : public IO {
public:
	FileSystem(const char *id, uint32_t capacity, uint32_t block_size);

	void setLogger(void (*logger)(const char*));

	void configure(const char *id, uint32_t capacity, uint32_t block_size);

	void bind(
		void (*read)(uint32_t, uint8_t*, uint32_t),
		void (*write)(uint32_t, uint8_t*, uint32_t),
		void (*erase_block)(uint32_t)
	);

	void mount();
	void unmount();
	void format();
	void flush(); // Flushes the partition table
	File* newFile(const char* name, FileType type);
	void delFile(File* file);
	File* getFile(const char* name);
	bool touch(File* file);
	Stream openStream(File* file, StreamMode mode);

	void (*read_func)(uint32_t address, uint8_t* buffer, uint32_t length);
	void (*write_func)(uint32_t address, uint8_t* buffer, uint32_t length);
	void (*erase_block_func)(uint32_t address);
	void (*erase_sector_func)(uint32_t address);

	void read(uint32_t address, uint8_t* buffer, uint32_t length);
	void write(uint32_t address, uint8_t* buffer, uint32_t length);

	uint32_t getTotalUsedBlocks();

	void (*log)(const char*);

private:
	bool device_configured;
	bool io_bound;
	bool mounted;
	bool debug;

	const char *id;
	uint32_t addressable_space;
	uint32_t block_size;

	bool allow_unsafe_access;

	uint32_t total_used_blocks;
	uint8_t partition_table[NUM_BLOCKS];
    uint8_t reverse_partition_table[NUM_BLOCKS];
	bool partition_table_modified;
	DataBlock data_blocks[NUM_BLOCKS];
	File files[NUM_FILES];

	void checkMounted();

	void initBlockManagement();

	uint16_t alloc(FileType type);
	void free(uint16_t block_id);
	int32_t access(uint32_t* address, uint32_t length, AccessType access_type);
	void protect(uint16_t block_id);

	void writeHeader(uint16_t block_id, uint16_t file_id, uint16_t predecessor);
	bool checkHeader(uint16_t block_id);

	uint16_t loadFileMeta(File* file);
	uint32_t computeLength(uint16_t block_id);
	uint32_t getBaseAddress(uint16_t block_id);


	void updateUsageTable(uint32_t write_begin, uint32_t write_end);

	void updateRelativeTime();
	void decreaseRelativeTime();

	uint32_t computeLengthFromUsage(uint64_t usage_table);
};


#endif /* INC_FILESYSTEM_H_ */
