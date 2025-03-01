#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include "mtp_bootloader_idt9320.h"
#include <sys/ioctl.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#define I2C_ADDR 0x3B
// starting address of firmware section to flash by bootloader
#define MTP_BOOTLOADER_FW_SECTION_START_ADDR 0x0400
#define MTP_VERIFIER_START_ADDR 0x0402
#define MTP_VERIFIER_DATA_SIZE_ADDR 0x0404
#define MTP_VERIFIER_CRC_ADDR 0x0406

#define UNLOCK_SYS_REGISTERS 0x5A
#define HS_CLOCKS 0
#define AHB_CLOCKS 9
#define PULSE_1US 5
#define PULSE_500NS 0x1D
#define HALT_PROCESSOR 0x10
#define ENABLE_MTP 1
#define INIT_PROGRAMMING_STRUCTURE 0
#define RESET_PROCESSOR 0x80
#define SEND_SIZE 16
#define MTP_BOOTLOADER_SECTION_SIZE 128
#define MTP_MEM_SIZE_LIMIT 1 << 14
#define START_PROG_CYCLE 1
#define START_VERIFY_CYCLE 0x11

#define STATUS_CRC_BUSY 1
#define STATUS_CRC_ERROR 8
#define STATUS_OK 2
#define STATUS_MTP_WRITE_ERR 4
#define STATUS_CHECK_SUM_ERR 8
#define STATUS_MTP_REPAIR_ERR 64

// this should be provided by vendor with the firmware file, or if
// firmware is taken from vendor kernel, can be found in driver.
#define MTP_VERIFY_ADDR			0x0000
#define MTP_VERIFY_SIZE			0x4680
#define MTP_VERIFY_CHKSUM		0x0274

struct downloader_section {
  unsigned short status;
  unsigned short start_addr;
  unsigned short code_length;
  unsigned short data_chk_sum;
  unsigned char data_buf[MTP_BOOTLOADER_SECTION_SIZE];
};

static void help(void) {
  fprintf(stderr,
      "Usage: idt9320_mtp_flasher I2CBUS ACTION FILEPATH\n"
      "  I2CBUS is an integer or an I2C bus name\n"
      "  ACTION is test, flash, check or repair\n"
      "  FILEPATH firmware file to flash\n"
      "Example (bus 0, flash mfc_fw.bin file):\n"
      "  # idt9320_mtp_flasher 0 flash mfc_fw.bin\n"
      );
}

/* msleep(): Sleep for the requested number of milliseconds. */
int msleep(long msec)
{
  struct timespec ts;
  int res;

  if (msec < 0)
  {
    errno = EINVAL;
    return -1;
  }

  ts.tv_sec = msec / 1000;
  ts.tv_nsec = (msec % 1000) * 1000000;

  do {
    res = nanosleep(&ts, &ts);
  } while (res && errno == EINTR);

  return res;
}

static void i2c_write_data(int file, unsigned short addr, void *data, int size) {
  int ret, sendsz = size + 2;
  unsigned char buf[sendsz];

  buf[0] = addr >> 8;
  buf[1] = addr & 0xff;
  memcpy(buf + 2, data, size);

  if ((ret = write(file, buf, sendsz)) != sendsz) {
    printf("i2c message sent failed: %d\n", ret);
    exit(1);
  }
}


static int i2c_write_byte_data_safe(int file, unsigned short addr, unsigned char data) {
  unsigned char buf[3] = { addr >> 8, addr & 0xff, data };
  int ret;

  return write(file, buf, 3) != 3;
}

static void i2c_write_byte_data(int file, unsigned short addr, unsigned char data) {
  unsigned char buf[3] = { addr >> 8, addr & 0xff, data };
  int ret;

  if ((ret = write(file, buf, 3)) != 3) {
    printf("i2c message sent failed: %d\n", ret);
    exit(1);
  }
}

static void i2c_read_byte_data(int file, unsigned short addr, unsigned char *data) {
  struct i2c_rdwr_ioctl_data rdwr;
  struct i2c_msg msgs[2];
  int nmsgs = 2;
  int nmsgs_sent = 0;
  unsigned char wbuf[2] = { addr >> 8, addr & 0xff };

  msgs[0].addr = I2C_ADDR;
  msgs[0].flags = 0;
  msgs[0].len = 2;
  msgs[0].buf = wbuf;


  msgs[1].addr = I2C_ADDR;
  msgs[1].flags = I2C_M_RD;
  msgs[1].len = 1;
  msgs[1].buf = data;

  rdwr.msgs = msgs;
  rdwr.nmsgs = nmsgs;
  nmsgs_sent = ioctl(file, I2C_RDWR, &rdwr);

  if (nmsgs_sent != 2) {
    printf("failed to send 2 i2c messages: %d;\n", nmsgs_sent);
    exit(1);
  }
}

static int i2c_read_data(int file, unsigned short addr, unsigned char *data, int size) {
  struct i2c_rdwr_ioctl_data rdwr;
  struct i2c_msg msgs[2];
  int nmsgs = 2;
  int nmsgs_sent = 0;
  unsigned char wbuf[2] = { addr >> 8, addr & 0xff };

  msgs[0].addr = I2C_ADDR;
  msgs[0].flags = 0;
  msgs[0].len = 2;
  msgs[0].buf = wbuf;


  msgs[1].addr = I2C_ADDR;
  msgs[1].flags = I2C_M_RD;
  msgs[1].len = size;
  msgs[1].buf = data;

  rdwr.msgs = msgs;
  rdwr.nmsgs = nmsgs;
  nmsgs_sent = ioctl(file, I2C_RDWR, &rdwr);

  if (nmsgs_sent != 2) {
    printf("i2c read failed: %d;\n", nmsgs_sent);
  }

  return nmsgs_sent;
}

static void verify_bootloader(int file, unsigned short addr, const unsigned char *bootloader, int size) {
  unsigned char rbuf[size];

  printf("verify...");
  i2c_read_data(file, addr, rbuf, size);
  if (!memcmp(rbuf, bootloader, size)) {
    printf("ok\n");
  } else {

    for (int i = 0; i < size; i++) {
      if (bootloader[i] != rbuf[i])
        printf("byte %d\n: expect: 0x%x, actual: 0x%x\n", i, bootloader[i] & 0xFF, rbuf[i] & 0xff);
    }
    printf("not ok\n");
    exit(1);
  }
}

static void load_bootloader(int file, unsigned short addr, unsigned char *bootloader, int size) {
  struct i2c_msg msg;
  int nmsgs = 2;
  int nmsgs_sent = 0;
  int ret = 0;
  int i = 0;
  int sendsz = SEND_SIZE;
  int cnt = 0;
  int retry_cnt = 0;
  unsigned char *data;
  unsigned char rdata[sendsz];

  printf("loading bootloader");

  while (size > 0) {
    if (size < sendsz)
      sendsz = size;

    data = bootloader + i++ * SEND_SIZE;

    i2c_write_data(file, addr, data, sendsz);

    i2c_read_data(file, addr, rdata, sendsz);

    size -= sendsz;
    addr += sendsz;

    if (memcmp(rdata, data, sendsz)) {
      printf("0x%x verification failed\n", addr);
    } else {
      printf(".");
    }
  }
  printf("ok\n");
}

void do_repair_cycle(int file, unsigned short section_addr) {
  int i = 1000;
  unsigned char status = START_PROG_CYCLE;

  i2c_write_byte_data(file, section_addr, START_PROG_CYCLE);

  while (status & START_PROG_CYCLE) {
    i2c_read_byte_data(file, section_addr, &status);
    msleep(20);

    if (!i--) {
      printf("timeout trying to repair. Status: 0x%hu\n", status);
      exit(1);
    }
  }

  switch (status) {
    case (STATUS_OK):
      printf("repair ok\n");
      break;
    case (STATUS_MTP_REPAIR_ERR):
      printf("repair failed: %d, i: %d\n", status, i);
      exit(1);
    default:
      printf("unknown status: 0x%x\n", status & 0xff);
      i2c_read_byte_data(file, 0x0400, &status);
      printf("0x0400: 0x%x\n", status & 0xff);
      exit(1);
  }
}

void do_verify_cycle(int file, unsigned short section_addr) {
  int i = 1000;
  unsigned short status = START_VERIFY_CYCLE;

  i2c_write_byte_data(file, section_addr, START_VERIFY_CYCLE);

  while (status & STATUS_CRC_BUSY) {
    i2c_read_data(file, section_addr, (char *) &status, 2);
    msleep(20);

    if (!i--) {
      printf("timeout trying to verify. Status: 0x%hu\n", status);
      exit(1);
    }
  }

  printf("0x%x", status);

  switch (status >> 8) {
    case (STATUS_OK):
      printf("CRC verify ok\n");
      break;
    case (STATUS_CRC_BUSY):
      printf("failed: CRC busy\n");
      exit(1);
    case (STATUS_CRC_ERROR):
      printf("failed: CRC error\n");
      exit(1);
    default:
      printf("unknown status: 0x%x\n", status);
      exit(1);
  }
}


void do_prog_cycle(int file, unsigned short section_addr, unsigned short start_addr) {
  int i = 250;
  unsigned char status = START_PROG_CYCLE;

  i2c_write_byte_data(file, section_addr, START_PROG_CYCLE);

  while (status & START_PROG_CYCLE) {
    i2c_read_byte_data(file, section_addr, &status);
    msleep(20);

    if (!i--) {
      printf("timeout trying to flash section. Status: 0x%hu\n", status);
      exit(1);
    }
  }

  switch (status) {
    case (STATUS_OK):
      printf("0x%hx ok\n", start_addr);
      break;
    case (STATUS_MTP_WRITE_ERR):
      printf("0x%hx mtp write error\n", start_addr);
      exit(1);
    case (STATUS_CHECK_SUM_ERR):
      printf("0x%hx checksum error\n", start_addr);
      exit(1);
    default:
      printf("0x%hx unknown status: 0x%x\n", start_addr, status & 0xff);
      i2c_read_byte_data(file, 0x0401, &status);
      printf("0x0401: 0x%x\n", status & 0xff);
      exit(1);
  }
}

void load_section(int file, unsigned short section_addr, struct downloader_section *section) {
  struct i2c_msg msg;
  int sendsz = sizeof(struct downloader_section) + 2;
  //int sendsz = 144;
  unsigned char data[144] = { 0 };
  int nmsgs_sent = 0;

  data[0] = section_addr >> 8;
  data[1] = section_addr & 0xFF;
  memcpy(data + 2, section, sizeof(struct downloader_section));

  if (write(file, data, sendsz) != sendsz) {
    printf("i2c write failed\n");
    exit(1);
  }
}

void load_fw(int file, int file_fw) {
  int ret = 0, i = 0, nbytes;
  unsigned short fw_addr = 0, buf_sum;
  struct downloader_section *section;

  unsigned char buf[MTP_BOOTLOADER_SECTION_SIZE];
  section = malloc(sizeof(struct downloader_section));

  while ((nbytes = read(file_fw, section->data_buf, MTP_BOOTLOADER_SECTION_SIZE)) > 0
      && (i * MTP_BOOTLOADER_SECTION_SIZE) < (1 << 14)) {
    for (int j = 0; j < nbytes; j++)
      buf_sum += section->data_buf[j];

    section->status = 0;
    section->start_addr = fw_addr;
    section->code_length = nbytes;
    section->data_chk_sum = section->start_addr + section->code_length + buf_sum;
    load_section(file, MTP_BOOTLOADER_FW_SECTION_START_ADDR, section);
    do_prog_cycle(file, MTP_BOOTLOADER_FW_SECTION_START_ADDR, section->start_addr);

    i++;
    fw_addr += MTP_BOOTLOADER_SECTION_SIZE;
    buf_sum = 0;
  }

  free(section);
}

void prepare_system(int file) {
  i2c_write_byte_data(file, 0x3000, UNLOCK_SYS_REGISTERS);
  i2c_write_byte_data(file, 0x3004, HS_CLOCKS);
  i2c_write_byte_data(file, 0x3008, AHB_CLOCKS);
  i2c_write_byte_data(file, 0x300C, PULSE_1US);
  i2c_write_byte_data(file, 0x300D, PULSE_500NS);

  i2c_write_byte_data(file, 0x3040, HALT_PROCESSOR | ENABLE_MTP);
  msleep(10);
  i2c_write_byte_data(file, 0x3040, HALT_PROCESSOR);
  msleep(10);
}

void mtp_programming_procedure(int file, int fw_file) {
  int ret = 0;

  prepare_system(file);
  load_bootloader(file, MTP_BOOTLOADER_START_ADDR, MTPBootloader9320, sizeof(MTPBootloader9320));
  verify_bootloader(file, MTP_BOOTLOADER_START_ADDR, MTPBootloader9320, sizeof(MTPBootloader9320));

  i2c_write_byte_data(file, MTP_BOOTLOADER_FW_SECTION_START_ADDR, INIT_PROGRAMMING_STRUCTURE);
  i2c_write_byte_data(file, 0x3048, REMAP_RAM_TO_MTP);
  ret = i2c_write_byte_data_safe(file, 0x3040, RESET_PROCESSOR);
  if (ret)
    printf("cpu successfully reset\n");

  msleep(100);
  printf("loading...\n");

  load_fw(file, fw_file);

  printf("you should reset chip power\n");
}

void mtp_verify_procedure(int file) {
  int ret = 0;
  struct downloader_section *section;

  section = malloc(sizeof(struct downloader_section));
  section->status = 0;
  section->start_addr = 0;
  section->code_length = MTP_VERIFY_SIZE;
  section->data_chk_sum = MTP_VERIFY_CHKSUM;

  prepare_system(file);
  load_bootloader(file, MTP_BOOTLOADER_START_ADDR, MTPVerifier9320, sizeof(MTPVerifier9320));
  verify_bootloader(file, MTP_BOOTLOADER_START_ADDR, MTPVerifier9320, sizeof(MTPVerifier9320));

  i2c_write_byte_data(file, MTP_BOOTLOADER_FW_SECTION_START_ADDR, INIT_PROGRAMMING_STRUCTURE);
  i2c_write_byte_data(file, 0x3048, REMAP_RAM_TO_MTP);
  ret = i2c_write_byte_data_safe(file, 0x3040, RESET_PROCESSOR);
  if (ret)
    printf("cpu successfully reset\n");

  msleep(100);
  printf("verifying...\n");

  load_section(file, MTP_BOOTLOADER_FW_SECTION_START_ADDR, section);
  do_verify_cycle(file, MTP_BOOTLOADER_FW_SECTION_START_ADDR);

  printf("you should reset chip power\n");
}

void mtp_repair_procedure(int file) {
  int ret = 0;
  struct downloader_section *section;

  section = malloc(sizeof(struct downloader_section));
  section->status = 0;
  section->start_addr = 0;
  section->code_length = MTP_VERIFY_SIZE;
  section->data_chk_sum = MTP_VERIFY_CHKSUM;

  prepare_system(file);
  load_bootloader(file, MTP_BOOTLOADER_START_ADDR, MTPRepair9320, sizeof(MTPRepair9320));
  verify_bootloader(file, MTP_BOOTLOADER_START_ADDR, MTPRepair9320, sizeof(MTPRepair9320));

  i2c_write_byte_data(file, MTP_BOOTLOADER_FW_SECTION_START_ADDR, INIT_PROGRAMMING_STRUCTURE);
  i2c_write_byte_data(file, 0x3048, REMAP_RAM_TO_MTP);
  ret = i2c_write_byte_data_safe(file, 0x3040, RESET_PROCESSOR);
  if (ret)
    printf("cpu successfully reset\n");

  msleep(100);
  printf("repairing...\n");

  do_repair_cycle(file, MTP_BOOTLOADER_FW_SECTION_START_ADDR);

  printf("you should reset chip power\n");
}

void test_bootloader(int file) {
  int ret;
  unsigned short status, val = 0xDEAD;

  prepare_system(file);
  load_bootloader(file, MTP_BOOTLOADER_START_ADDR, MTPBootloader9320, sizeof(MTPBootloader9320));
  verify_bootloader(file, MTP_BOOTLOADER_START_ADDR, MTPBootloader9320, sizeof(MTPBootloader9320));

  i2c_write_data(file, MTP_BOOTLOADER_FW_SECTION_START_ADDR, &val, 2);
  i2c_write_byte_data(file, 0x3048, REMAP_RAM_TO_MTP);
  ret = i2c_write_byte_data_safe(file, 0x3040, RESET_PROCESSOR);
  if (ret)
    printf("cpu successfully reset\n");

  msleep(100);

  i2c_read_data(file, MTP_BOOTLOADER_FW_SECTION_START_ADDR, (unsigned char *) &status, 2);
  if (status != val)
    printf("bootloader successfully started!\n");
  else
    printf("bootloader fail to start\n");

  printf("you should reset chip power\n");
}

void main(int argc, char *argv[]) {
  int file, fw_file, i2cbus, adapter_nr;
  struct stat st_fw_file;
  char filename[20];
  char *action;
  char *filepath;

  if (argc < 2) {
    help();
    exit(1);
  }

  adapter_nr = (int) strtol(argv[optind++], (char **)NULL, 10);
  action = argv[optind++];

  snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);
  file = open(filename, O_RDWR);
  if (file < 0) {
    printf("Failed to open i2c device\n");
    exit(file);
  }

  if (ioctl(file, I2C_SLAVE, I2C_ADDR) < 0) {
    printf("Failed to set i2c slave address\n");
    exit(1);
  }

  if (!action) {
    help();
    exit(1);
  }

  if (!strcmp(action, "test")) {
    test_bootloader(file);
  }

  if (!strcmp(action, "flash")) {
    filepath = argv[optind++];

    if (!filepath) {
      help();
      exit(1);
    }

    fw_file = open(filepath, O_RDWR);
    if (fw_file < 0) {
      printf("failed to find firmware file at: %s\n", filepath);
      exit(1);
    }

    fstat(fw_file, &st_fw_file);
    if (st_fw_file.st_size > MTP_MEM_SIZE_LIMIT)
      exit(ENOMEM);

    mtp_programming_procedure(file, fw_file);
  }

  if (!strcmp(action, "check")) {
    mtp_verify_procedure(file);
  }

  if (!strcmp(action, "repair")) {
    mtp_repair_procedure(file);
  }
}
