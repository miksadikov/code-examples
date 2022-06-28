
/**@file Запуск/останов функционала flashcache.

  Mikhail Sadikov, @miksadikov

**/
#include "flcache.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <libcsm/log.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include "config.h"
#include "usbgm.h"
#include "usbgm_shared.h"

#define PRELIMINARY_CHUNK_BLOCKS 240
#define PRELIMINARY_CHUNK_SIZE   (PRELIMINARY_CHUNK_BLOCKS * DEF_BLOCK_SIZE)
#define READ_REPORTS ((uint32_t)((100*1024*1024/512)/PRELIMINARY_CHUNK_BLOCKS)*PRELIMINARY_CHUNK_BLOCKS)
#define READ_ADDITIONAL (20 * 1024 * 1024 / DEF_BLOCK_SIZE)
#define SECTOR_NUM_LEN sizeof(uint32_t)
#define MASS_STORAGE_DEACTIVATE "<NULL>"
#define MASS_STORAGE_CONFIG_PATH "/sys/kernel/config/usb_gadget/gadget_dev/configs/config.1/mass_storage.0"
#define MASS_STORAGE_PATH MASS_STORAGE_CONFIG_PATH "/lun.%d/%s"
#define MASS_STORAGE_FILE "file"
#define MASS_STORAGE_READWRITE_MODE "ro"
#define MASS_STORAGE_READONLY "1"

#define FLASHCACHE_REMOVE "dmsetup remove cachedev"
#define FLASHCACHE_DEV "/dev/mapper/cachedev"
#define SHUFFLE_DEV "/dev/mapper/shuffledev"
#define FLASHCACHE_PROC_FILES_PATH "/proc/flashcache/"
#define FLASHCACHE_PROC_SECTORS_FILE "/flashcache_sectors"
#define FLASHCACHE_PROC_SECTORS_NUM_FILE "/flashcache_sectors_num"

#define CACHED_DATA_NOT_DELETED_FILE "/usr/share/dev/tmp/flcache"

void del_cached_data_if_necessary(const char *partition);
int delete_sectors(uint32_t* sectors, uint32_t s_quantity, const char *partition);

/* ----- Static ----- */

static int cachedev_exists(const char *path) {
  if (access(path, F_OK) != -1) {
    return 1;
  }
  return 0;
}

static int get_storage_file(char *buffer, size_t len, const char *file) {
  int i = 0;

  for (i = 1; i >= 0; i--) {
    snprintf(buffer, len, MASS_STORAGE_PATH, i, file);
    if (!access(buffer, F_OK)) {
      return 0;
    }
  }

  return -1;
}

static int mass_storage_check() {
  DIR* dir = opendir(MASS_STORAGE_CONFIG_PATH);

  if (!dir) {
    return -1;
  }

  closedir(dir);
  return 0;
}

static int mass_storage_activate(const char* disk_path) {
  int writed = 0;
  char buffer[128];

  if (get_storage_file(buffer, sizeof(buffer), MASS_STORAGE_FILE) < 0) {
    log_error("failed to get storage file '%s'", MASS_STORAGE_FILE);
    return -1;
  }

  FILE* fp = fopen(buffer, "we");
  if (fp == NULL) {
    log_error("failed to open %s file - %s", buffer,
              strerror(errno));
    return -1;
  }

  log_debug("set %s to %s", disk_path, buffer);
  writed = fwrite(disk_path, sizeof(char), strlen(disk_path), fp);
  if (writed <= 0) {
    log_error("failed to write %s - %s", disk_path, strerror(errno));
    fclose(fp);
    return -1;
  }

  fclose(fp);
  return 0;
}

static int mass_storage_deactivate(void) {
  int writed = 0;
  char buffer[128];

  if (get_storage_file(buffer, sizeof(buffer), MASS_STORAGE_FILE) < 0) {
    log_error("failed to get storage file '%s'", MASS_STORAGE_FILE);
    return -1;
  }

  FILE* fp = fopen(buffer, "we");
  if (fp == NULL) {
    log_error("failed to open %s file - %s", buffer,
              strerror(errno));
    return -1;
  }

  log_debug("set %s to %s", MASS_STORAGE_DEACTIVATE, buffer);
  writed = fwrite(MASS_STORAGE_DEACTIVATE, sizeof(char),
                  strlen(MASS_STORAGE_DEACTIVATE), fp);
  if (writed <= 0) {
    log_error("failed to write %s - %s", MASS_STORAGE_DEACTIVATE,
              strerror(errno));
    fclose(fp);
    return -1;
  }

  fclose(fp);
  return 0;
}

static int mass_storage_set_readwrite_mode(const char *mode) {
  int writed = 0;
  char buffer[128];

  if (get_storage_file(buffer, sizeof(buffer), MASS_STORAGE_READWRITE_MODE) < 0) {
    log_error("failed to get storage file '%s'", MASS_STORAGE_READWRITE_MODE);
    return -1;
  }

  FILE* fp = fopen(buffer, "we");
  if (fp == NULL) {
    log_error("failed to open %s file - %s", buffer,
              strerror(errno));
    return -1;
  }

  log_debug("set %s to %s", MASS_STORAGE_READONLY, buffer);
  writed = fwrite(MASS_STORAGE_READONLY, sizeof(char),
                  strlen(MASS_STORAGE_READONLY), fp);
  if (writed <= 0) {
    log_error("failed to write %s - %s", MASS_STORAGE_READONLY,
              strerror(errno));
    fclose(fp);
    return -1;
  }

  fclose(fp);
  return 0;
}

static void make_path_to_cache_inf_files(const char* disk_path, const char* proc_path,
                                         const char* inf_file_name,
                                         char* inf_file_path,
                                         const char* data_file_name,
                                         char* data_file_path) {
  inf_file_path[0] = '\0';
  data_file_path[0] = '\0';

  strncat(inf_file_path, proc_path, PATH_MAX_LEN - 1);
  strncat(inf_file_path, inf_file_name, PATH_MAX_LEN - strlen(inf_file_path) - 1);

  strncat(data_file_path, proc_path, PATH_MAX_LEN - 1);
  strncat(data_file_path, data_file_name,
          PATH_MAX_LEN - strlen(data_file_path) - 1);
}

static uint32_t char4_to_uint32(const char* data) {
  uint32_t val32 = 0;

  val32 = (((uint64_t)*data << 0) | ((uint64_t) * (data + 1) << 8) |
           ((uint64_t) * (data + 2) << 16) | ((uint64_t) * (data + 3) << 24));

  return val32;
}

static int delete_all_cached_data(const char *cached_disk, const char *flcache_partition) {
  char i_file_path[PATH_MAX_LEN];
  char d_file_path[PATH_MAX_LEN];
  char sectors_total[SECTOR_NUM_LEN];
  uint32_t sectors_total32 = 0;
  uint32_t* sectors = NULL;
  int readed = 0;
  int ret = 0;
  FILE* fp = NULL;

  make_path_to_cache_inf_files(cached_disk, FLASHCACHE_PROC_FILES_PATH,
                               FLASHCACHE_PROC_SECTORS_NUM_FILE, i_file_path,
                               FLASHCACHE_PROC_SECTORS_FILE, d_file_path);
  fp = fopen(i_file_path, "rbe");
  if (fp == NULL) {
    log_error("failed to open %s file - %s", i_file_path, strerror(errno));
    return -1;
  }

  readed = fread(sectors_total, sizeof(char), SECTOR_NUM_LEN, fp);
  if (readed != sizeof(sectors_total)) {
    log_error("error when read %s file", i_file_path);
    fclose(fp);
    return -1;
  }

  sectors_total32 = char4_to_uint32(sectors_total);
  if (sectors_total32 == 0) {
    fclose(fp);
    return -1;
  }
  log_debug("flashcache sectors to delete = %lu", sectors_total32);
  fclose(fp);

  fp = fopen(d_file_path, "rbe");
  if (fp == NULL) {
    log_error("failed to open %s file - %s", d_file_path, strerror(errno));
    return -1;
  }

  sectors = (uint32_t*)malloc(sectors_total32 * sizeof(uint32_t));
  if (sectors == NULL) {
    log_error("malloc error when delete cached data");
    fclose(fp);
    return -1;
  }

  readed = fread(sectors, sizeof(uint32_t), sectors_total32, fp);
  if (readed <= 0) {
    log_error("error when read %s file", d_file_path);
    free(sectors);
    fclose(fp);
    return -1;
  }

  ret = delete_sectors(sectors, sectors_total32, flcache_partition);
  if (ret == 0) {
    log_debug("all cached data are successfully deleted");
  }
  fclose(fp);
  free(sectors);
  return 0;
}

static void set_flcache_session_flag(void) {
  FILE* fp = fopen(CACHED_DATA_NOT_DELETED_FILE, "w+e");
  if (fp == NULL) {
    log_error("failed to create %s file - %s", CACHED_DATA_NOT_DELETED_FILE,
              strerror(errno));
  } else {
    fclose(fp);
  }
}

static void clear_flcache_session_flag(void) {
  if (remove(CACHED_DATA_NOT_DELETED_FILE) != 0) {
    log_error("failed to create %s file - %s", CACHED_DATA_NOT_DELETED_FILE,
              strerror(errno));
  }
}

#ifdef PRELIM_CACHE
static uint64_t get_flcache_count(void) {
  int fd;
  uint32_t sectors;

  if ((fd = open(FLASHCACHE_PROC_FILES_PATH FLASHCACHE_PROC_SECTORS_NUM_FILE, O_RDONLY)) < 0) {
    log_error("failed to open %s - %s\n", FLASHCACHE_PROC_SECTORS_NUM_FILE, strerror(errno));
    return 0;
  }

  if (read(fd, &sectors, sizeof(sectors)) < 0) {
    log_error("failed to read %s - %s\n", FLASHCACHE_PROC_SECTORS_NUM_FILE, strerror(errno));
    close(fd);
    return 0;
  }

  close(fd);
  return sectors * (DEF_CACHE_SECTOR_SIZE / DEF_BLOCK_SIZE);
}

static int cache_preliminary_filling(const char *disk_path, const char *cachedev, rndgen *rg) {
  int64_t max_sector_num = 0;
  int64_t skip = 0;
  off64_t offset = 0;
  off64_t r = 0;
  uint64_t disk_sz = 0;
  uint64_t reads = 0;
  uint64_t reads_report = 0;
  uint64_t cfp_sz = 0, cfp_sz_orig = 0;  // cache filling pool size
  uint64_t sectors;
  int readed = 0;
  uint8_t* wrk_buff = NULL;
  int cache_fd = 0;
  int error = 0;

  struct timespec start;
  struct timespec end;
  struct timespec temp;  // for time profiling

  if ((disk_sz = util_disk_size_blocks(disk_path)) == 0) {
    return -1;
  }

  log_debug("disk size = %" PRId64, disk_sz);

  if (posix_memalign((void**)&wrk_buff, 4*DEF_BLOCK_SIZE, PRELIMINARY_CHUNK_SIZE)) {
    log_error("posix_memalign() failed -%s", strerror(errno));
    return -1;
  }

  cfp_sz_orig = cfp_sz = param_get_cahce_size(disk_sz);
  reads_report = READ_REPORTS;

  log_debug("start of cache preliminary filling, usb flash size = %llu, cache pool size = %llu", disk_sz, cfp_sz);
  max_sector_num = disk_sz - PRELIMINARY_CHUNK_BLOCKS - 1;

  if ((cache_fd = open(cachedev, O_RDONLY | O_CLOEXEC)) < 0) {
    log_error("failed to open %s - %s\n", cachedev, strerror(errno));
    return -1;
  }

  clock_gettime(CLOCK_MONOTONIC, &start);
again:
  while (!error && reads < cfp_sz) {
    int ret = 0;

    ret = rndgen_get_rnd_values(rg, &skip, NULL);
    if (ret < 0) {
      log_error("couldn't get random values\n");
      usleep(10 * 1000);
      continue;
    }

    skip = skip % max_sector_num;
    offset = skip * DEF_BLOCK_SIZE;
    r = offset % PRELIMINARY_CHUNK_SIZE;
    offset = r ? offset + (PRELIMINARY_CHUNK_SIZE - r) : offset;

    if (lseek64(cache_fd, offset, SEEK_SET) < 0) {
      error = 1;
      log_error("couldn't skip to required position (%" PRId64 ") - %s\n", offset, strerror(errno));
      continue;
    }

    readed = read(cache_fd, (void *)wrk_buff, PRELIMINARY_CHUNK_SIZE);
    if (readed < 0) {
      error = 1;
      log_error("failed to read - %s\n", strerror(errno));
      continue;
    }
    reads += PRELIMINARY_CHUNK_BLOCKS;

    if ((reads % reads_report) == 0 || reads >= cfp_sz) {
      clock_gettime(CLOCK_MONOTONIC, &end);
      temp = util_time_diff(&start, &end);
      log_debug("Filled %llu/%llu(%llu%%) blocks in %4ld.%03lds", reads, cfp_sz,
        100ULL*reads/cfp_sz, temp.tv_sec, temp.tv_nsec / 1000000);
    }
  }

  if (!error) {
    sectors = get_flcache_count();
    if (!sectors) {
      error = 1;
    } else if (sectors < cfp_sz_orig) {
      cfp_sz += READ_ADDITIONAL;
      goto again;
    }
  }

  close(cache_fd);
  free(wrk_buff);

  if (error) {
    log_debug("cache preliminary filling failed");
    return -1;
  }
  log_debug("cache preliminary filling finished successfully");
  return 0;
}
#endif

static int prev_flcache_session_not_correctly_ended(void) {
  if (access(CACHED_DATA_NOT_DELETED_FILE, F_OK) != -1) {
    // file exists, it means what prev session was finished incorrectly
    return 1;
  }
  return 0;
}

/* ----- Exports ----- */

int flashcache_start(const char *disk_path, rndgen *rg) {
  int status = 0;
  int ret = 0;
  pid_t child_pid = 0;

  if (mass_storage_check() < 0) {
    log_error("failed to check mass_storage");
    return -1;
  }

  log_debug("run flashcache_create utility");
  child_pid = fork();
  if (child_pid >= 0) { // fork succeeded
    if (child_pid == 0) {
      /* run flashcache_create utility
       * block size = 4 KB
       * use "writearound" caching mode - all disk reads are cached.
       * https://github.com/facebookarchive/flashcache/blob/master/doc/
       * flashcache-sa-guide.txt
      */
      status =
          execl("/usr/bin/flashcache_create", "flashcache_create", "-v", "-f", "-b",
                "4k", "-s", FLASHCACHE_PARTITION_SIZE, "-p", "around",
                "cachedev", FLASHCACHE_PARTITION, disk_path, NULL);
      return status;
    }
    wait(&status);
    if (status == -1) {
      log_error("failed to run flashcache_create - %s", strerror(errno));
      return status;
    }

#ifdef PRELIM_CACHE
    ret = cache_preliminary_filling(disk_path, FLASHCACHE_DEV, rg);
    if (ret != 0) {
      flashcache_stop(disk_path);
      return ret;
    }
#endif

    ret = mass_storage_set_readwrite_mode(MASS_STORAGE_READONLY);
    if (ret != 0) {
      return ret;
    }
    ret = mass_storage_activate(FLASHCACHE_DEV);

    set_flcache_session_flag();

  } else {
    log_error("failed to run flashcache_create");
    return -1;
  }

  return ret;
}

int flashcache_stop(const char *cached_disk) {
  int status = 0;
  int ret = 0;
  pid_t child_pid = 0;

  log_debug("flashcache stop and remove");

  usleep(10000);

  ret = mass_storage_deactivate();
  if (ret != 0) {
    log_error("failed to deactivate mass storage");
  }

  usleep(10000);

  ret = delete_all_cached_data(cached_disk, FLASHCACHE_PARTITION);
  if (ret != 0) {
    log_error("failed to delete cached data");
  }

  clear_flcache_session_flag();


  log_debug("cachedev remove...");

  if (cachedev_exists(FLASHCACHE_DEV)) {
    usleep(500000);
    child_pid = fork();
    if (child_pid >= 0) { // fork succeeded
      if (child_pid == 0) {
        /* run dmsetup utility, this removes the flashcache volume
         * https://github.com/facebookarchive/flashcache/blob/master/
         * /doc/flashcache-sa-guide.txt
         */
        status = execl("/usr/sbin/dmsetup", "dmsetup", "remove",
                       "cachedev", NULL);
        return status;
      }
      wait(&status);
      if (status == -1) {
        log_error("failed to run dmsetup remove - %s", strerror(errno));
      }
    } else {
      log_error("failed to run dmsetup remove");
    }
  }

  log_debug("cachedev remove OK");
  log_debug("shuffledev remove...");

  if (cachedev_exists(SHUFFLE_DEV)) {
    usleep(500000);
    child_pid = fork();
    if (child_pid >= 0) { // fork succeeded
      if (child_pid == 0) {
        // run dmsetup utility, this removes the shuffledev volume
        status = execl("/usr/sbin/dmsetup", "dmsetup", "remove",
                       "shuffledev", NULL);
        return status;
      }
      wait(&status);
      if (status == -1) {
        log_error("failed to run dmsetup remove - %s", strerror(errno));
      }
    } else {
      log_error("failed to run dmsetup remove");
    }
  }

  log_debug("shuffledev remove OK");
  return 0;
}

void flashcache_check_session() {
  if (prev_flcache_session_not_correctly_ended()) {
    del_cached_data_if_necessary(FLASHCACHE_PARTITION);
    clear_flcache_session_flag();
  }
}
