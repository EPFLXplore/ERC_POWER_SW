/*
 * filesystem.c
 *
 *  Created on: 14 Oct 2019
 *      Author: Arion
 */

#include "FileSystem.h"
#include "File.h"
#include "Stream.h"

#include <stddef.h>

/*
 * FileSystem structure
 *
 * Each 4KB subsector is called a 'block'.
 *
 * Block 0: Core block
 * 		2KB: RocketFS heuristic magic number
 * 		2KB: Metadata
 * Block 1: Master partition (bit 0...3: FileType, 4...7: relative initialisation time)
 * Block 2: Recovery partition
 * Block 3: Backup slot 1
 * Block 4: Backup slot 2
 * Block 5: Backup slot 3
 * Block 6: Backup slot 4
 * Block 7: Journal
 *
 * Block 8: Data
 * ...
 * Block 4091: Data
 *
 * Block 4092: Reserved
 * Block 4093: Reserved
 * Block 4094: Reserved (INVASIVE_TEST for flash memory)
 * Block 4095: Reserved (GENTLE_TEST for flash memory)
 */

#define CORRUPTION_THRESHOLD 4
#define MAGIC_PERIOD 7
#define BACKUP_MAGIC 0xC0FFEE

/*
 * Utility functions
 */
static uint8_t __clamp(uint8_t input, uint8_t start, uint8_t end);
static uint64_t __signed_shift(int64_t input, int8_t amount);
static uint64_t __generate_periodic(uint8_t period);
static bool __periodic_magic_match(uint8_t period, uint64_t testable_magic);
static void __no_log(const char* _);

/*
 * FileSystem functions
 */
FileSystem::FileSystem(const char *id, uint32_t capacity, uint32_t block_size) : IO() {

	this->id = id;
	this->addressable_space = capacity;
	this->block_size = block_size;
	this->total_used_blocks = 0;
	this->partition_table_modified = false;

	if(block_size < NUM_BLOCKS) {
		//log("Fatal: Device's sub-sector granularity is too high. Consider using using a device with higher block_size.");
		this->device_configured = false;
	} else if(block_size * NUM_BLOCKS > capacity) {
		//log("Fatal: Device's sub-sector granularity is too low. Consider using using a device with lower block_size.");
		this->device_configured = false;
	} else {
		this->device_configured = true;
	}

	this->io_bound = false;
	this->mounted = false;
	this->debug = false;

	this->allow_unsafe_access = false;

	this->log = &__no_log;
	this->read_func = nullptr;
	this->write_func = nullptr;
	this->erase_block_func = nullptr;
	this->erase_sector_func = nullptr;

	for(uint16_t i = 0; i < NUM_BLOCKS; i++) {
		reverse_partition_table[i] = 0;
		partition_table[i] = 0;
		data_blocks[i].successor = 0;
	}

	for(uint16_t i = 0; i < NUM_FILES; i++) {
		files[i].first_block = 0;
		files[i].last_block = 0;
		files[i].hash = 0;
		files[i].length = 0;
		files[i].used_blocks = 0;
	}
}

void FileSystem::setLogger(void (*logger)(const char*)) {
	this->log = logger;
	this->debug = true;

	log("FileSystem log initialised.");
}

void FileSystem::bind(
	void (*read)(uint32_t, uint8_t*, uint32_t),
	void (*write)(uint32_t, uint8_t*, uint32_t),
	void (*erase_block)(uint32_t)
) {
	this->read_func = read;
	this->write_func = write;
	this->erase_block_func = erase_block;
	this->io_bound = true;

   if(!this->debug) {
	   this->log = &__no_log;
   }
}

void FileSystem::mount() {
	log("Mounting filesystem...");

	if(this->mounted) {
		log("Error: FileSystem already mounted.");
		return;
	}

	uint32_t core_base = getBaseAddress(0);
	uint32_t master_base = getBaseAddress(1);
	uint32_t aux_base = getBaseAddress(2);

	Stream core_stream(this, core_base, RAW);

	uint64_t magic = core_stream.read64();

	if(__periodic_magic_match(MAGIC_PERIOD, magic)) {
		log("Reading partition table...");

		if(checkHeader(1)) {
			Stream master_stream(this, master_base, RAW);
			master_stream.read(reverse_partition_table, NUM_BLOCKS);
			master_stream.close();

			for(uint32_t i = 0; i < NUM_BLOCKS; i++) {
				// Reverse bits to increase the lifetime of NOR flash memories (do not do this if the targeted device is a NAND flash).
				partition_table[i] = ~reverse_partition_table[i];
			}
		} else if(checkHeader(2)) {
			log("Primary partition table corrupted. Falling back to auxiliary partition table");

			Stream aux_stream(this, aux_base, RAW);
			aux_stream.read(reverse_partition_table, NUM_BLOCKS);
			aux_stream.close();

			for(uint32_t i = 0; i < NUM_BLOCKS; i++) {
				// Reverse bits to increase the lifetime of NOR flash memories (do not do this if the targeted device is a NAND flash).
				partition_table[i] = ~reverse_partition_table[i];
			}

			partition_table_modified = true;
			flush();

		} else {
			log("Partition tables corrupted. Formatting.");
			format();
		}


		initBlockManagement();

		mounted = true;

		log("Filesystem mounted.");
	} else {
		// TODO (a) Check redundant magic number.
		// TODO (b) Write corrupted partition to a free backup slot.
		log("Mounting filesystem for the first time or filesystem corrupted.");

		format();

		mount();
	}

	core_stream.close();
}

void FileSystem::unmount() {
	log("Unmounting FileSystem...");

	checkMounted();

	flush();

	mounted = false;

	log("FileSystem unmounted.");
}


void FileSystem::format() {
	log("Formatting FileSystem...");

	uint32_t core_base = getBaseAddress(0);
	uint32_t master_base = getBaseAddress(1);

	erase_block_func(core_base);   // Core block
	erase_block_func(master_base); // Master partition block

	Stream core_stream(this, core_base, RAW);
	Stream master_stream(this, master_base, RAW);

	/*
	 * Blocks 0 to 7 are reserved anyways
	 */

	master_stream.write8(~0b00001110); // Core block (used as internal relative clock)
	master_stream.write8(~0b00001111); // Master partition block
	master_stream.write8(~0b00001111); // Recovery partition block
	master_stream.write8(~0b00001111); // Backup partition block 1
	master_stream.write8(~0b00001111); // Backup partition block 2
	master_stream.write8(~0b00001111); // Backup partition block 3
	master_stream.write8(~0b00001111); // Backup partition block 4
	master_stream.write8(~0b00001111); // Journal block

	master_stream.close();

	writeHeader(0, 0, 0);
	writeHeader(1, 0, 0);
	writeHeader(2, 0, 0);
	writeHeader(3, 0, 0);
	writeHeader(4, 0, 0);
	writeHeader(5, 0, 0);
	writeHeader(6, 0, 0);
	writeHeader(7, 0, 0);

	uint64_t magic = __generate_periodic(MAGIC_PERIOD);
	core_stream.write64(magic);

	/*
	 * ... write heuristic magic number and metadata
	 */

	core_stream.close();

	total_used_blocks = 8;

	log("FileSystem formatted.");
}

/*
 * Flushes the partition table
 */
void FileSystem::flush() {
	if(mounted && partition_table_modified) {
		log("Flushing partition table...");

		partition_table_modified = false;

		for(uint32_t i = 0; i < NUM_BLOCKS; i++) {
			// Reverse bits to increase the lifetime of NOR flash memories (do not do this if the targeted device is a NAND flash).
		   reverse_partition_table[i] = ~partition_table[i];
		}

		uint32_t master_base = getBaseAddress(1);
		uint32_t aux_base = getBaseAddress(2);

		Stream master_stream(this, master_base, RAW);
		Stream aux_stream(this, aux_base, RAW);

		erase_block_func(block_size); // Erase the master partition block
		master_stream.write(reverse_partition_table, NUM_BLOCKS);
		master_stream.close();
		writeHeader(1, 0, 0);

		erase_block_func(2*block_size); // Erase the master partition block
		aux_stream.write(reverse_partition_table, NUM_BLOCKS);
		aux_stream.close();
		writeHeader(2, 0, 0);

		log("Partition table flushed.");
	}
}

/*
 * Names at most 15 characters long.
 * Storing file names in a hashtable.
 */
File* FileSystem::newFile(const char* name, FileType type) {
	char filename[16];

	checkMounted();

	log("Creating new file...");

	filename_copy(name, filename);

	File* file;
	uint32_t hash = hash_filename(filename);
	uint8_t bucket = hash % NUM_FILES;

	for(uint8_t file_id = bucket; file_id < bucket + NUM_FILES; file_id++) {
		file = &(files[file_id % NUM_FILES]);

		if(filename_equals(file->filename, name)) {
			log("File with the given filename already exists:");
			log(name);
			return 0;
		}

		if(file->first_block == 0) {
			// Yey! We found an available file identifier

			uint16_t first_block_id = alloc(type);
			writeHeader(first_block_id, file_id, 0);
			protect(first_block_id);

			uint32_t address = getBaseAddress(first_block_id);

			write(address, (uint8_t*) filename, 16); // Write the filename

			filename_copy(filename, file->filename);
			file->hash = hash;
			file->first_block = first_block_id;
			file->last_block = first_block_id;
			file->used_blocks = 1;
			file->length = 0;

			flush();

			log("File created.");

			return file;
		}
	}

	log("Maximal number of files reached.");
	return 0;
}

void FileSystem::delFile(File* file) {
	checkMounted();

	log("Deleting file...");

	uint16_t block_id = file->first_block;
	DataBlock* block;

	if(block_id) {
		do {
			free(block_id);

			block = &(data_blocks[block_id]);

			block_id = block->successor;
			block->successor = 0;
		} while(block_id);

		file->hash = 0;
		file->first_block = 0;
		file->last_block = 0;
		file->length = 0;
		file->used_blocks = 0;

		flush();

		log("File deleted.");
	} else {
		log("File does not exist.");
	}
}

File* FileSystem::getFile(const char* name) {
	checkMounted();

	char filename[16];
	filename_copy(name, filename);

	File* file;
	uint32_t hash = hash_filename(filename);
	uint8_t bucket = hash % NUM_FILES;

	for(uint8_t file_id = bucket; file_id < bucket + NUM_FILES; file_id++) {
		file = &(files[file_id % NUM_FILES]);

		if(file->first_block && filename_equals(file->filename, filename)) {
		   //rfs_load_file_meta(fs, file);
			return file;
		}
	}

	log("File with the given filename was not found in the filesystem:");
	log(filename);

	return 0;
}

bool FileSystem::touch(File* file) {
	checkMounted();

	loadFileMeta(file);

	return true;
}


Stream FileSystem::openStream(File* file, StreamMode mode) {
	checkMounted();

	switch(mode) {
	case OVERWRITE: {
		uint16_t first_block = file->first_block;
		uint32_t base_address = getBaseAddress(first_block) + 16; // Do not overwrite the 16-characters long identifier

		FileType type = static_cast<FileType>(partition_table[first_block] >> 4);

		return Stream(this, base_address, type);
	}

	case APPEND: {
		uint16_t last_block = file->last_block;
		uint32_t base_address = last_block * block_size + computeLength(last_block);

		FileType type = static_cast<FileType>(partition_table[last_block] >> 4);

		return Stream(this, base_address, type);
	}

	default:
		log("Unsupported stream mode");
		return Stream(this, 0, EMPTY);
	}
}


void FileSystem::checkMounted() {
	if(!mounted) {
		log("Error: FileSystem not mounted");
	}
}


/*
 * Corruption utility functions
 */

/*
 * Use Gaussian filter on binary magic number to remove impulsional noise.
 * Decompose magic signal in wave form and compare it with the given periodicity
 * (we could use Fourier transforms but that would probably be overkill for embedded systems).
 * Sigma: 1.5
 */

static uint8_t __clamp(uint8_t input, uint8_t start, uint8_t end) {
	if(input <= start) {
		return start;
	} else if(input >= end) {
		return end;
	} else {
		return input;
	}
}

static uint64_t __signed_shift(int64_t input, int8_t amount) {
	if(amount > 0) {
		return input >> amount;
	} else if(amount < 0) {
		return input << (-amount);
	} else {
		return input;
	}
}

static uint64_t __generate_periodic(uint8_t period) {
	uint64_t periodic = 0;
	uint64_t period_generator = (-1ULL) >> (64 - period / 2);

	for(uint8_t i = 0; i < 64; i += period) {
		periodic <<= period;
		periodic |= period_generator;
	}

	return periodic;
}

static bool __periodic_magic_match(uint8_t period, uint64_t testable_magic) {
	static const uint16_t __gaussian_kernel[] = { 614, 2447, 3877, 2447, 614 };
	static const uint16_t __gaussian_divider[] = { 3470, 4693, 5000 };

	uint8_t i, j;
	uint64_t weighted = 0;
	uint64_t hard_coded_magic = __generate_periodic(period);

	uint64_t filtered = 0;

	for(i = 0; i < 64; i++) {
		weighted = 0;
		filtered <<= 1;

		for(j = 0; j < 5; j++) {
			weighted += __gaussian_kernel[j] * (__signed_shift(testable_magic, 65 - i - j) & 0b1);
		}

		if(i < 32) {
			filtered |=  weighted / __gaussian_divider[__clamp(i, 0, 2)];
		} else {
			filtered |= weighted / __gaussian_divider[__clamp(64 - i, 0, 2)];
		}
	}

	uint8_t delta = 0;

	filtered ^= hard_coded_magic;

	for(uint8_t i = 0; i < 64; i++) {
		delta += (filtered >> i) & 0b1;
	}

	return delta < CORRUPTION_THRESHOLD;
}

void FileSystem::read(uint32_t address, uint8_t* buffer, uint32_t length) {
	read_func(address, buffer, length);
}

void FileSystem::write(uint32_t address, uint8_t* buffer, uint32_t length) {
	write_func(address, buffer, length);
}

uint32_t FileSystem::getTotalUsedBlocks() {
	return total_used_blocks;
}

static void __no_log(const char* _) {}
