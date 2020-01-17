/*
 Copyright (c) 2016 Mathieu Laurendeau <mat.lau@laposte.net>
 License: GPLv3
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <limits.h>
#include <string.h>
#include <getopt.h>

#include <gimxusb/include/gusb.h>
#include <gimxpoll/include/gpoll.h>
#include <gimxtimer/include/gtimer.h>
#include <gimxtime/include/gtime.h>
#include <gimxlog/include/glog.h>
#include <gimxprio/include/gprio.h>

#include <gimxcommon/test/common.h>
#include <gimxcommon/test/handlers.c>
#include <gimxcommon/test/timer.c>

#define PERIOD 5000 //microseconds
#define RUMBLE_PERIOD 1000000 //microseconds
#define FF_PERIOD 80000 //microseconds

typedef struct {
  unsigned short length;
  unsigned char data[64];
} s_usb_packet;

static struct {
  unsigned short vid;
  unsigned short pid;
  struct {
      unsigned char in;
      unsigned char out;
  } endpoints;
  s_usb_packet start;
  s_usb_packet stop;
} rumble_cmds[] = {
    {
        .vid = 0x046d,
        .pid = 0xc218,
        .endpoints = { .in = 0x81, .out = 0x01 },
        .start = { 7, { 0x51, 0x00, 0x7f, 0x00, 0x7f, 0x00, 0x00 } },
        .stop =  { 1, { 0xf3 } },
    },
    { // Dualshock 4 v1
        .vid = 0x054c,
        .pid = 0x05c4,
        .endpoints = { .in = 0x84, .out = 0x03 },
        .start = { 9, { 0x05, 0xff, 0x00, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00 } },
        .stop =  { 9, { 0x05, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00 } },
    },
    { // X-Box pad v2 (US)
        .vid = 0x045e,
        .pid = 0x0289,
        .endpoints = { .in = 0x00, .out = 0x00 },
        .start = { 16, { 0x21, 0x09, 0x00, 0x02, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0xff, 0x00, 0xff } },
        .stop =  { 16, { 0x21, 0x09, 0x00, 0x02, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00 } },
    }
};

static int rumble_index = -1;

static struct {
  unsigned short vid;
  unsigned short pid;
  struct {
      unsigned char in;
      unsigned char out;
  } endpoints;
  s_usb_packet stop;
  s_usb_packet left;
  s_usb_packet right;
} ff_cmds[] = {
    {
        .vid = 0x046d,
        .pid = 0xc29a,
        .endpoints = { .in = 0x81, .out = 0x01 },
        .stop = { 7, { 0xf3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } }, // stop all forces
        .left  = { 7, { 0x11, 0x08, 0x60, 0x80, 0x00, 0x00, 0x00 } },
        .right = { 7, { 0x11, 0x08, 0xa0, 0x80, 0x00, 0x00, 0x00 } },
    },
};

static unsigned int periods = 0;
static int quiet = 0;
static int debug = 0;
static int prio = 0;

static int ff_index = -1;

static struct gusb_device * device = NULL;

static char * usb_select() {

  char * path = NULL;

  struct gusb_device_info * usb_devs = gusb_enumerate(0x0000, 0x0000);
  if (usb_devs == NULL) {
    fprintf(stderr, "No usb device detected!\n");
    return NULL;
  }
  printf("Available usb devices:\n");
  unsigned int index = 0;
  struct gusb_device_info * current;
  for (current = usb_devs; current != NULL; current = current->next) {
    printf("%d VID 0x%04x PID 0x%04x PATH %s\n", index++, current->vendor_id, current->product_id, current->path);
  }

  printf("Select the usb device number: ");
  fflush(stdout);
  unsigned int choice = UINT_MAX;
  if (scanf("%d", &choice) == 1 && choice < index) {
    current = usb_devs;
    while(choice > 0) {
        current = current->next;
        --choice;
    }
    unsigned int i;
    for (i = 0; i < sizeof(rumble_cmds) / sizeof(*rumble_cmds); ++i) {
      if(rumble_cmds[i].vid == current->vendor_id && rumble_cmds[i].pid == current->product_id) {
        rumble_index = i;
      }
    }
    for (i = 0; i < sizeof(ff_cmds) / sizeof(*ff_cmds); ++i) {
      if(ff_cmds[i].vid == current->vendor_id && ff_cmds[i].pid == current->product_id) {
        ff_index = i;
      }
    }
    if (rumble_index == -1 && ff_index == -1) {
      fprintf(stderr, "Selected usb device is not supported!\n");
    } else {
      path = strdup(current->path);
      if(path == NULL) {
        fprintf(stderr, "can't duplicate path.\n");
      }
    }
  } else {
    fprintf(stderr, "Invalid choice.\n");
  }

  gusb_free_enumeration(usb_devs);

  return path;
}

static void dump(const unsigned char * packet, unsigned char length) {

  int i;
  for (i = 0; i < length; ++i) {
    if (i && !(i % 8)) {
      printf("\n");
    }
    printf("0x%02x ", packet[i]);
  }
  printf("\n");
}

int usb_read(void * user __attribute__((unused)), unsigned char endpoint, const void * buf, int status) {

  if (status < 0) {
    set_done();
    return 1;
  }

  int ret = gusb_poll(device, endpoint);
  if (ret < 0) {
    set_done();
    return 1;
  }

  if (status > 0 && !quiet) {
    gtime now = gtime_gettime();
    printf("%lu.%06lu ", GTIME_SECPART(now), GTIME_USECPART(now));
    printf("%s\n", __func__);
    dump((unsigned char *) buf, status);
    fflush(stdout);
  }

  return 0;
}

static int usb_busy = 0;

static int counter = 0;

void rumble_task() {

  if(rumble_index < 0) {
    return;
  }

  static int rumble = 0;

  if (!usb_busy) {

    if (counter >= RUMBLE_PERIOD / PERIOD) {
      if(rumble) {
        printf("Stop rumble\n");
      } else {
        printf("Start rumble\n");
      }
      fflush(stdout);
      rumble = !rumble;
      counter = 0;
    }

    if(rumble) {
      gusb_write(device, rumble_cmds[rumble_index].endpoints.out, rumble_cmds[rumble_index].start.data, rumble_cmds[rumble_index].start.length);
    } else  {
      gusb_write(device, rumble_cmds[rumble_index].endpoints.out, rumble_cmds[rumble_index].stop.data, rumble_cmds[rumble_index].stop.length);
    }
    usb_busy = 1;
  }
}

void ff_task() {

  if(ff_index < 0) {
    return;
  }

  static int ff_play = 0;
  static int ff_dir = 0;

  if (!usb_busy) {

    if (counter >= RUMBLE_PERIOD / PERIOD) {
      if(ff_play) {
        printf("Stop rumble\n");
      } else {
        printf("Start rumble\n");
      }
      fflush(stdout);
      ff_play = !ff_play;
      counter = 0;
    }

    if(ff_play) {
      static int cpt = 0;
      if(ff_dir) {
        gusb_write(device, ff_cmds[ff_index].endpoints.out, ff_cmds[ff_index].left.data, ff_cmds[ff_index].left.length);
      } else {
        gusb_write(device, ff_cmds[ff_index].endpoints.out, ff_cmds[ff_index].right.data, ff_cmds[ff_index].right.length);
      }
      ++cpt;
      if(cpt == FF_PERIOD / PERIOD) {
        ff_dir = !ff_dir;
        cpt = 0;
      }
    } else  {
      gusb_write(device, ff_cmds[ff_index].endpoints.out, ff_cmds[ff_index].stop.data, ff_cmds[ff_index].stop.length);
    }
    usb_busy = 1;
  }
}

void usb_task() {

  if(is_done()) {
    return;
  }
  rumble_task();
  ff_task();
}

int usb_write(void * user __attribute__((unused)), unsigned char endpoint __attribute__((unused)), int transfered __attribute__((unused))) {

  if (endpoint == 0x00 && rumble_cmds[rumble_index].endpoints.in != 0) {

      unsigned char endpoint = 0;
      if(rumble_index >= 0) {
        endpoint = rumble_cmds[rumble_index].endpoints.in;
      }
      if(ff_index >= 0) {
        endpoint = ff_cmds[ff_index].endpoints.in;
      }
      int ret = gusb_poll(device, endpoint);
      if (ret < 0) {
        set_done();
      }
  }

  usb_busy = 0;
  usb_task();
  return 0;
}

int usb_close(void * user __attribute__((unused))) {
  set_done();
  return 0;
}

static void usage() {
  fprintf(stderr, "Usage: ./gusb_test [-n period_count] [-p] [-q] [-d]\n");
  exit(EXIT_FAILURE);
}

/*
 * Reads command-line arguments.
 */
static int read_args(int argc, char* argv[]) {

  int opt;
  while ((opt = getopt(argc, argv, "dn:pq")) != -1) {
    switch (opt) {
    case 'd':
      debug = 1;
      break;
    case 'n':
      periods = atoi(optarg);
      break;
    case 'p':
      prio = 1;
      break;
    case 'q':
      quiet = 1;
      break;
    default: /* '?' */
      usage();
      break;
    }
  }
  return 0;
}

int main(int argc, char* argv[]) {

  setup_handlers();

  read_args(argc, argv);

  if (debug) {
    glog_set_all_levels(E_GLOG_LEVEL_TRACE);
  }

  char * path = usb_select();

  if(path == NULL) {
    fprintf(stderr, "No usb device selected!\n");
    exit(-1);
  }

  device = gusb_open_path(path);

  if (device != NULL) {

    const s_usb_descriptors * usb_desc = gusb_get_usb_descriptors(device);

    printf("Opened device: VID 0x%04x PID 0x%04x PATH %s\n", usb_desc->device.idVendor, usb_desc->device.idProduct, path);

    GUSB_CALLBACKS gusb_callbacks = {
            .fp_read = usb_read,
            .fp_write = usb_write,
            .fp_close = usb_close,
            .fp_register = REGISTER_FUNCTION,
            .fp_remove = REMOVE_FUNCTION,
    };
    if (gusb_register(device, NULL, &gusb_callbacks) != -1) {

      GTIMER_CALLBACKS timer_callbacks = {
              .fp_read = timer_read,
              .fp_close = timer_close,
              .fp_register = REGISTER_FUNCTION,
              .fp_remove = REMOVE_FUNCTION,
      };
      struct gtimer * timer = gtimer_start(NULL, PERIOD, &timer_callbacks);
      if (timer == NULL) {
        set_done();
      }

      if (!is_done()) {
          struct {
              struct usb_ctrlrequest req;
              unsigned char data[64];
          } transfer = {
                  .req = {
                          .bRequestType = USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
                          .bRequest = 0x01, // GET REPORT
                          .wValue = 0x0100, // report type = input (1), report id = 0
                          .wIndex = 0x0000, // interface 0
                          .wLength = sizeof(transfer.data),
                  },
                  .data = { }
          };

          int ret = gusb_write(device, 0x00, &transfer, sizeof(transfer.req));
          if (ret < 0) {
              set_done();
          } else {
              usb_busy = 1;
          }
      }

      if (prio && gprio_init() < 0) {
          return -1;
      }

      while (!is_done() || usb_busy) {

        gpoll();

        ++counter;

        if (periods > 0) {
            --periods;
            if (periods == 0) {
                set_done();
            }
        }
      }

      if (prio)
      {
        gprio_clean();
      }

      if (timer != NULL) {
        gtimer_close(timer);
      }

      if(rumble_index >= 0) {
        gusb_write_timeout(device, rumble_cmds[rumble_index].endpoints.out, rumble_cmds[rumble_index].stop.data, rumble_cmds[rumble_index].stop.length, 1000);
      }

      if(ff_index >= 0) {
        gusb_write_timeout(device, ff_cmds[ff_index].endpoints.out, ff_cmds[ff_index].stop.data, ff_cmds[ff_index].stop.length, 1000);
      }

      gusb_close(device);
    }
  }

  free(path);

  printf("Exiting\n");

  return 0;
}
