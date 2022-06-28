/**@file Удаление кешированных данных.

  Mikhail Sadikov, @miksadikov

**/
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <libcsm/log.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "config.h"
#include "flcache.h"

/* Default output blocksize. */
#define DEFAULT_BLOCKSIZE 512
#define BLOCKS_PER_WRITE 8  // page size
#define FLASHCACHE_MAX_SECTOR_NUMBER ((1024 * 1024 * 1024 / 512) - 1)

char zero_buf[DEFAULT_BLOCKSIZE * BLOCKS_PER_WRITE];

static void del_dirty_page(int fd, uint32_t off);
static int check_on_zero(const char* buf, size_t sz);

int delete_sectors(uint32_t* sectors, uint32_t s_quantity, const char *partition) {
  int fd = 0;
  long offset = 0;
  int error = 0;
  uint32_t s_count = 0;
  ssize_t written = 0;
  uint32_t* psectors = sectors;
  size_t size = 0;
  size_t total_written = 0;

  if ((fd = open(partition, O_RDWR | O_CLOEXEC)) < 0) {
    log_error("can't open %s - %s", partition, strerror(errno));
    return -1;
  }

  size = sizeof(zero_buf);

  do {
    offset = lseek(fd, (long)(*psectors * DEFAULT_BLOCKSIZE), SEEK_SET);
    if (offset < 0) {
      log_error("can't seek to %lu in %s - %s", (long)*psectors, partition,
                strerror(errno));
      error = -1;
      break;
    }
    memset(zero_buf, 0, sizeof(zero_buf));
    do {
      written = write(fd, zero_buf + total_written, size - total_written);
      if (written < 0) {
        if (errno != EINTR) {
          log_error("can't write to %s - %s (ret = %i)", partition,
                    strerror(errno), written);
          error = -1;
          break;
        }
        written = 0;

      } else if (written == 0) {
        if (errno != ECHILD) {
          log_error("can't write to %s - %s (ret = %i)", partition,
                    strerror(errno), written);
          error = -1;
          break;
        }
      }
      total_written += written;
    } while ((total_written < size) && (error == 0));

    total_written = 0;
    psectors++;
    s_count++;
  } while ((s_count < s_quantity) && (error == 0));

  close(fd);
  return error;
}

void del_cached_data_if_necessary(const char *partition) {
  int fd = 0;
  long offset = 0;
  int error = 0;
  uint32_t s_count = 0;
  ssize_t readed = 0;
  ssize_t size = sizeof(zero_buf);

  if ((fd = open(partition, O_RDWR | O_CLOEXEC)) < 0) {
    log_error("can't open %s - %s", partition, strerror(errno));
    return;
  }

  log_debug(
      "prev flcache session was not correctly ended, we must check cache...");
  do {
    offset = lseek(fd, (long)(s_count * DEFAULT_BLOCKSIZE), SEEK_SET);
    if (offset < 0) {
      log_error("can't seek to %lu sector in %s - %s", s_count, partition,
                strerror(errno));
      error = -1;
      break;
    }

    readed = read(fd, (void*)zero_buf, size);
    if (readed <= 0) {
      log_error("read error on %lu of %s - %s", s_count, partition,
                strerror(errno));
    } else {
      if (check_on_zero(zero_buf, size) != 0) {
        del_dirty_page(fd, s_count);
      }
    }
    s_count += BLOCKS_PER_WRITE;
  } while ((s_count < FLASHCACHE_MAX_SECTOR_NUMBER) && (error == 0));
  log_debug("cache check is ended");
  close(fd);
}

static void del_dirty_page(int fd, uint32_t off) {
  long ret = 0;
  int error = 0;
  ssize_t written = 0;
  size_t total_written = 0;
  size_t sz = sizeof(zero_buf);

  ret = lseek(fd, (long)(off * DEFAULT_BLOCKSIZE), SEEK_SET);
  if (ret < 0) {
    log_error("can't seek to %lu sector - %s", off, strerror(errno));
    return;
  }

  memset(zero_buf, 0, sz);

  do {
    written = write(fd, zero_buf + total_written, sz - total_written);
    if (written < 0) {
      if (errno != EINTR) {
        log_error("write error - %s (ret = %i)", strerror(errno), written);
        error = -1;
        break;
      }
      written = 0;

    } else if (written == 0) {
      if (errno != ECHILD) {
        log_error("write error - %s (ret = %i)", strerror(errno), written);
        error = -1;
        break;
      }
    }
    total_written += written;
  } while ((total_written < sz) && (error == 0));
}

static int check_on_zero(const char* buf, size_t sz) {
  size_t i = 0;

  for (i = 0; i < sz; i++) {
    if (*(buf + i) != 0) {
      return -1;
    }
  }
  return 0;
}
