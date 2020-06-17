/*
 Copyright (c) 2019 Mathieu Laurendeau <mat.lau@laposte.net>
 License: GPLv3
 */

#include <gusb.h>
#include <gimxpoll/include/gpoll.h>
#include <gimxcommon/include/gerror.h>
#include <gimxcommon/include/glist.h>
#ifdef WIN32
#include <gimxtimer/include/gtimer.h>
#endif
#include <gimxlog/include/glog.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <libusb-1.0/libusb.h>

#define POLLIN      0x001
#define POLLOUT     0x004

#define OUT_TIMEOUT 20 // milliseconds
#define CONTROL_TIMEOUT 50 // milliseconds

#define DEFAULT_TIMEOUT 1000 // milliseconds

#define IS_ENDPOINT_IN(endpoint) ((endpoint & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_IN)
#define IS_ENDPOINT_OUT(endpoint) ((endpoint & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_OUT)
#define IS_ENDPOINT_INTERRUPT(endpoint) ((endpoint & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_INTERRUPT)
#define IS_ENDPOINT_BULK(endpoint) ((endpoint & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_BULK)
#define IS_ENDPOINT_ISOCHRONOUS(endpoint) ((endpoint & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_ISOCHRONOUS)

#define INVALID_ENDPOINT_INDEX 0xff

#define DEFAULT_STRING_BUFFER_SIZE 255

GLOG_INST(GLOG_NAME)

struct gusb_device {
  char * path;
  libusb_context * ctx;
  libusb_device_handle * devh;
  s_usb_descriptors descriptors;
  struct {
    struct {
      unsigned char type;
      unsigned short size;
    } in;
    struct {
      unsigned char type;
      unsigned short size;
    } out;
  } endpoints[LIBUSB_ENDPOINT_ADDRESS_MASK];
  GUSB_CALLBACKS callbacks;
  void * user;
  struct libusb_transfer ** transfers;
  unsigned int transfers_nb;
  int closing; // do not process completed transfers when closing
  GLIST_LINK(struct gusb_device);
};

static GLIST_INST(struct gusb_device, usb_devices);

#if !defined(LIBUSB_API_VERSION) && !defined(LIBUSBX_API_VERSION)
static const char * LIBUSB_CALL libusb_strerror(enum libusb_error errcode)
{
  return libusb_error_name(errcode);
}
#endif

#define PRINT_ERROR_LIBUSB(libusbfunc,ret) \
    do { \
        if (GLOG_LEVEL(GLOG_NAME,ERROR)) { \
            fprintf(stderr, "%s:%d %s: %s failed with error: %s\n", __FILE__, __LINE__, __func__, libusbfunc, libusb_strerror(ret)); \
        } \
    } while (0)

#define PRINT_ERROR_INVALID_ENDPOINT(msg, endpoint) \
    do { \
        if (GLOG_LEVEL(GLOG_NAME,ERROR)) { \
            fprintf(stderr, "%s:%d %s: %s: 0x%02x\n", __FILE__, __LINE__, __func__, msg, endpoint); \
        } \
    } while (0)

#ifdef WIN32
static struct gtimer * usb_timer = NULL;
#endif

static int add_transfer(struct libusb_transfer * transfer) {
  struct gusb_device * device = (struct gusb_device *) transfer->user_data;
  unsigned int i;
  for (i = 0; i < device->transfers_nb; ++i) {
    if (device->transfers[i] == transfer) {
      return 0;
    }
  }
  void * ptr = realloc(device->transfers, (device->transfers_nb + 1) * sizeof(*device->transfers));
  if (ptr) {
    device->transfers = ptr;
    device->transfers[device->transfers_nb] = transfer;
    device->transfers_nb++;
    return 0;
  } else {
    PRINT_ERROR_ALLOC_FAILED("realloc");
    return -1;
  }
}

static void remove_transfer(struct libusb_transfer * transfer) {
  struct gusb_device * device = (struct gusb_device *) transfer->user_data;
  unsigned int i;
  for (i = 0; i < device->transfers_nb; ++i) {
    if (device->transfers[i] == transfer) {
      memmove(device->transfers + i, device->transfers + i + 1, (device->transfers_nb - i - 1) * sizeof(*device->transfers));
      device->transfers_nb--;
      void * ptr = realloc(device->transfers, device->transfers_nb * sizeof(*device->transfers));
      if (ptr || !device->transfers_nb) {
        device->transfers = ptr;
      } else {
        PRINT_ERROR_ALLOC_FAILED("realloc");
      }
      free(transfer->buffer);
      libusb_free_transfer(transfer);
      break;
    }
  }
}

static inline unsigned char get_endpoint(struct gusb_device * device, unsigned char endpoint, unsigned char direction, unsigned int count) {

  if ((endpoint & LIBUSB_ENDPOINT_DIR_MASK) != direction) {

    PRINT_ERROR_INVALID_ENDPOINT("wrong direction for endpoint", endpoint);
    return INVALID_ENDPOINT_INDEX;
  }
  
  unsigned char endpointIndex = (endpoint & LIBUSB_ENDPOINT_ADDRESS_MASK);
    
  if (endpointIndex == 0) {

    PRINT_ERROR_INVALID_ENDPOINT("invalid endpoint", endpoint);
    return INVALID_ENDPOINT_INDEX;
  }
  
  --endpointIndex;

  if (IS_ENDPOINT_IN(endpoint)) {

    if(device->endpoints[endpointIndex].in.type == 0) {
      PRINT_ERROR_INVALID_ENDPOINT("no such endpoint", endpoint);
      return INVALID_ENDPOINT_INDEX;
    }
    if (count > device->endpoints[endpointIndex].in.size) {

      PRINT_ERROR_OTHER("incorrect transfer size");
      return INVALID_ENDPOINT_INDEX;
    }
  } else {

    if(device->endpoints[endpointIndex].out.type == 0) {
      PRINT_ERROR_INVALID_ENDPOINT("no such endpoint", endpoint);
      return INVALID_ENDPOINT_INDEX;
    }
    if (count > device->endpoints[endpointIndex].out.size) {

      PRINT_ERROR_OTHER("incorrect transfer size");
      return INVALID_ENDPOINT_INDEX;
    }
  }
  
  return endpointIndex;
}

static char * make_path(libusb_device * dev) {
  uint8_t path[1 + 7] = { };
  int pathLen = sizeof(path) / sizeof(*path);
  static char str[sizeof(path) / sizeof(*path) * 3];
  path[0] = libusb_get_bus_number(dev);
  int ret = libusb_get_port_numbers(dev, path + 1, pathLen - 1);
  if (ret < 0) {
    PRINT_ERROR_LIBUSB("libusb_get_port_numbers", ret);
    return NULL;
  }
  int i;
  for (i = 0; i < ret + 1; ++i) {
    snprintf(str + i * 3, sizeof(str) - i * 3, "%02x:", path[i]);
  }
  str[(ret + 1) * 3 - 1] = '\0';
  return str;
}

static struct gusb_device * add_device(const char * path, int print) {

  struct gusb_device * current = GLIST_BEGIN(usb_devices);
  while (current != GLIST_END(usb_devices)) {
    if(current->path && !strcmp(current->path, path)) {
      if(print) {
        PRINT_ERROR_OTHER("device already opened");
      }
      return NULL;
    }
    current = current->next;
  }
  struct gusb_device * device = calloc(1, sizeof(*device));
  if (device == NULL) {
    PRINT_ERROR_ALLOC_FAILED("calloc");
    return NULL;
  }
  device->path = strdup(path);
  if (device->path == NULL) {
    PRINT_ERROR_OTHER("can't duplicate path");
    free(device);
    return NULL;
  }
  GLIST_ADD(usb_devices, device);
  return device;
}

static int submit_transfer(struct libusb_transfer * transfer) {
  /*
   * Don't submit the transfer if it can't be added in the 'transfers' table.
   * Otherwise it would not be possible to cleanly cancel it.
   */
  int ret = add_transfer(transfer);

  if (ret != -1) {
    ret = libusb_submit_transfer(transfer);
    if (ret != LIBUSB_SUCCESS) {
      PRINT_ERROR_LIBUSB("libusb_submit_transfer", ret);
      remove_transfer(transfer);
      return -1;
    }
  }
  return 0;
}

static void usb_callback(struct libusb_transfer* transfer) {

  struct gusb_device * device = (struct gusb_device *) transfer->user_data;

  if(device->closing == 1 || transfer->status == LIBUSB_TRANSFER_CANCELLED) {
    remove_transfer(transfer);
    return;
  }

  if (GLOG_LEVEL(GLOG_NAME,DEBUG)) {
    fprintf(stderr, "endpoint: 0x%02x, transfer status: %s\n", transfer->endpoint, libusb_error_name(transfer->status));
  }

  int status;
  switch (transfer->status) {
  case LIBUSB_TRANSFER_COMPLETED:
    status = transfer->actual_length;
    break;
  case LIBUSB_TRANSFER_TIMED_OUT:
    status = E_STATUS_TRANSFER_TIMED_OUT;
    break;
  case LIBUSB_TRANSFER_STALL:
    status = E_STATUS_TRANSFER_STALL;
    break;
  case LIBUSB_TRANSFER_NO_DEVICE:
    status = E_STATUS_NO_DEVICE;
    break;
  default:
    status = E_STATUS_TRANSFER_ERROR;
    if (GLOG_LEVEL(GLOG_NAME,ERROR)) {
      fprintf(stderr, "libusb_transfer failed with status %s (endpoint=0x%02x)\n", libusb_error_name(transfer->status), transfer->endpoint);
    }
    break;
  }
  if (transfer->type == LIBUSB_TRANSFER_TYPE_CONTROL) {
    struct libusb_control_setup * setup = libusb_control_transfer_get_setup(transfer);
    if(setup->bmRequestType & LIBUSB_ENDPOINT_IN) {
      unsigned char * data = libusb_control_transfer_get_data(transfer);
      device->callbacks.fp_read(device->user, transfer->endpoint, data, status);
    } else {
      device->callbacks.fp_write(device->user, transfer->endpoint, status);
    }
  } else {
    if (IS_ENDPOINT_OUT(transfer->endpoint)) {
      device->callbacks.fp_write(device->user, transfer->endpoint, status);
    } else {
      device->callbacks.fp_read(device->user, transfer->endpoint, transfer->buffer, status);
    }
  }

  remove_transfer(transfer);
}

int gusb_poll(struct gusb_device * device, unsigned char endpoint) {

  unsigned char endpointIndex = get_endpoint(device, endpoint, LIBUSB_ENDPOINT_IN, 0);
  if(endpointIndex == INVALID_ENDPOINT_INDEX) {
  
    return -1;
  }

  if (device->callbacks.fp_read == NULL) {

    PRINT_ERROR_OTHER("missing read callback");
    return -1;
  }
  
  unsigned int size = device->endpoints[endpointIndex].in.size;

  unsigned char * buf = calloc(size, sizeof(char));
  if (buf == NULL) {

    PRINT_ERROR_ALLOC_FAILED("calloc");
    return -1;
  }

  struct libusb_transfer * transfer = libusb_alloc_transfer(0);
  if (transfer == NULL) {

    PRINT_ERROR_ALLOC_FAILED("libusb_alloc_transfer");
    free(buf);
    return -1;
  }

  switch (device->endpoints[endpointIndex].in.type) {
  case LIBUSB_TRANSFER_TYPE_INTERRUPT:
    libusb_fill_interrupt_transfer(transfer, device->devh, endpoint, buf, size,
        (libusb_transfer_cb_fn) usb_callback, (void *) (intptr_t) device, 0);
    break;
  default:
    PRINT_ERROR_OTHER("unsupported endpoint type");
    libusb_free_transfer(transfer);
    free(buf);
    return -1;
  }

  return submit_transfer(transfer);
}

static int handle_events(void * user) {

  struct gusb_device * device = (struct gusb_device *) user;

  return libusb_handle_events_timeout_completed(device->ctx, &((struct timeval){ 0, 0 }), NULL);
}

static int transfer_timeout(struct gusb_device * device, unsigned char endpoint, void * buf, unsigned int count, unsigned int timeout) {

  int transfered = -1;
  
  uint8_t endpointIndex = (endpoint & LIBUSB_ENDPOINT_ADDRESS_MASK);

  uint8_t type;
  if (endpointIndex == 0) {
	  type = LIBUSB_TRANSFER_TYPE_CONTROL;
  } else {
    --endpointIndex;
    if (IS_ENDPOINT_IN(endpoint)) {
      type = device->endpoints[endpointIndex].in.type;
    } else {
      type = device->endpoints[endpointIndex].out.type;
    }
  }

  int ret = -1;
  switch (type) {
  case LIBUSB_TRANSFER_TYPE_CONTROL:
    {
      struct libusb_control_setup * control_setup = (struct libusb_control_setup *)buf;
      unsigned char * data = buf + sizeof(*control_setup);
      ret = libusb_control_transfer(device->devh, control_setup->bmRequestType, control_setup->bRequest, control_setup->wValue,
              control_setup->wIndex, data, control_setup->wLength, timeout);
	  if (ret < 0) {

	    PRINT_ERROR_LIBUSB("libusb_control_transfer", ret);
	    return -1;
	  } else {
	    transfered = ret;
	  }
    }
	break;
  case LIBUSB_TRANSFER_TYPE_INTERRUPT:
    ret = libusb_interrupt_transfer(device->devh, endpoint, buf, count, &transfered, timeout);
    if (ret != LIBUSB_SUCCESS && ret != LIBUSB_ERROR_TIMEOUT) {

      PRINT_ERROR_LIBUSB("libusb_interrupt_transfer", ret);
      return -1;
    }
    break;
  default:
    PRINT_ERROR_OTHER("unsupported endpoint type");
    break;
  }

  return transfered;
}

int gusb_write_timeout(struct gusb_device * device, unsigned char endpoint, void * buf, unsigned int count, unsigned int timeout) {

  if (endpoint != 0) {

    unsigned char endpointIndex = get_endpoint(device, endpoint, LIBUSB_ENDPOINT_OUT, count);
    if(endpointIndex == INVALID_ENDPOINT_INDEX) {

      return -1;
    }
  }

  return transfer_timeout(device, endpoint, buf, count, timeout);
}

int gusb_read_timeout(struct gusb_device * device, unsigned char endpoint, void * buf, unsigned int count, unsigned int timeout) {

  unsigned char endpointIndex = get_endpoint(device, endpoint, LIBUSB_ENDPOINT_IN, count);
  if(endpointIndex == INVALID_ENDPOINT_INDEX) {
  
    return -1;
  }

  return transfer_timeout(device, endpoint, buf, count, timeout);
}

static int get_configurations (struct gusb_device * device) {

  s_usb_descriptors * descriptors = &device->descriptors;

  descriptors->configurations = calloc(descriptors->device.bNumConfigurations, sizeof(*descriptors->configurations));
  if (descriptors->configurations == NULL) {
    PRINT_ERROR_ALLOC_FAILED("calloc");
    return -1;
  }
  
  unsigned char index;
  for (index = 0; index < descriptors->device.bNumConfigurations; ++index) {
  
    struct usb_config_descriptor descriptor;
    
    int ret = libusb_control_transfer(device->devh, LIBUSB_ENDPOINT_IN,
        LIBUSB_REQUEST_GET_DESCRIPTOR, (LIBUSB_DT_CONFIG << 8) | index, 0, (unsigned char *)&descriptor,
        sizeof(descriptor), DEFAULT_TIMEOUT);
    
    if (ret < 0) {
      PRINT_ERROR_LIBUSB("libusb_control_transfer", ret);
      return -1;
    }
    
    struct p_configuration * configurations = descriptors->configurations + index;

    configurations->raw = calloc(descriptor.wTotalLength, sizeof(*configurations->raw));
    if (configurations->raw == NULL) {
      PRINT_ERROR_ALLOC_FAILED("calloc");
      return -1;
    }
    
    ret = libusb_control_transfer(device->devh, LIBUSB_ENDPOINT_IN,
        LIBUSB_REQUEST_GET_DESCRIPTOR, (LIBUSB_DT_CONFIG << 8) | index, 0, configurations->raw,
        descriptor.wTotalLength, DEFAULT_TIMEOUT);
    
    if (ret < 0) {
      PRINT_ERROR_LIBUSB("libusb_control_transfer", ret);
      return -1;
    }
  }

  return 0;
}

static int add_descriptor (struct gusb_device * device, unsigned short wValue, unsigned short wIndex, unsigned short wLength, unsigned char * data) {

  s_usb_descriptors * descriptors = &device->descriptors;
  
  void * ptr = realloc(descriptors->others, (descriptors->nbOthers + 1) * sizeof(*descriptors->others));
  if (ptr == NULL) {
    PRINT_ERROR_ALLOC_FAILED("realloc");
    free(data);
    return -1;
  }

  descriptors->others = ptr;
  memset(descriptors->others + descriptors->nbOthers, 0x00, sizeof(*descriptors->others));
  descriptors->others[descriptors->nbOthers].wValue = wValue;
  descriptors->others[descriptors->nbOthers].wIndex = wIndex;
  descriptors->others[descriptors->nbOthers].wLength = wLength;
  descriptors->others[descriptors->nbOthers].data = data;
  ++descriptors->nbOthers;
  
  return 0;
}

static int get_lang_id_0 (struct gusb_device * device) {

  if (device->descriptors.langId0.bLength >= 4) {
      return 0; // already fetched
  }

  struct usb_string_descriptor * descriptor = &device->descriptors.langId0;

  int ret = libusb_control_transfer(device->devh, LIBUSB_ENDPOINT_IN,
      LIBUSB_REQUEST_GET_DESCRIPTOR, (LIBUSB_DT_STRING << 8) | 0, 0, (unsigned char *)descriptor,
      sizeof(*descriptor), DEFAULT_TIMEOUT);

  // LIBUSB_ERROR_PIPE means device sent a STALL packet
  // device can have a string index set but no descriptor
  if (ret == LIBUSB_ERROR_PIPE) {
      if (GLOG_LEVEL(GLOG_NAME,DEBUG)) {
        printf("%s:%d %s: libusb_control_transfer failed with error: %s\n", __FILE__, __LINE__, __func__, libusb_strerror(ret));
      }
      return 1;
  }

  if (ret < 0) {
    PRINT_ERROR_LIBUSB("libusb_control_transfer", ret);
    return 1;
  }

  if (ret < 4) {
    memset(&device->descriptors.langId0, 0x00, sizeof(device->descriptors.langId0));
    PRINT_ERROR_OTHER("failed to get wLANGID[0]");
    return -1;
  }

  return 0;
}

static int get_string_descriptor (struct gusb_device * device, unsigned char index) {

  int ret = get_lang_id_0(device);
  if (ret != 0) {
    return ret;
  }

  s_usb_descriptors * descriptors = &device->descriptors;

  unsigned char * data = calloc(DEFAULT_STRING_BUFFER_SIZE, sizeof(*data));
  if (data == NULL) {
    PRINT_ERROR_ALLOC_FAILED("calloc");
    return -1;
  }

  ret = libusb_control_transfer(device->devh, LIBUSB_ENDPOINT_IN,
      LIBUSB_REQUEST_GET_DESCRIPTOR, (LIBUSB_DT_STRING << 8) | index, descriptors->langId0.wData[0], data, DEFAULT_STRING_BUFFER_SIZE, DEFAULT_TIMEOUT);

  // LIBUSB_ERROR_PIPE means device sent a STALL packet
  // device can have a string index set but no descriptor
  if (ret == LIBUSB_ERROR_PIPE) {
      if (GLOG_LEVEL(GLOG_NAME,DEBUG)) {
        printf("%s:%d %s: libusb_control_transfer failed with error: %s\n", __FILE__, __LINE__, __func__, libusb_strerror(ret));
      }
      return 1;
  }

  if (ret < 0) {
    PRINT_ERROR_LIBUSB("libusb_control_transfer", ret);
    free(data);
    return -1;
  }

  struct usb_descriptor_header * descriptor = (struct usb_descriptor_header *)data;

  if (descriptor->bLength > ret) {
    void * ptr = realloc(data, descriptor->bLength * sizeof(*data));
    if (ptr == NULL) {
      PRINT_ERROR_ALLOC_FAILED("realloc");
      free(data);
      return -1;
    }

    data = ptr;

    ret = libusb_control_transfer(device->devh, LIBUSB_ENDPOINT_IN,
        LIBUSB_REQUEST_GET_DESCRIPTOR, (LIBUSB_DT_STRING << 8) | index, descriptors->langId0.wData[0], data, descriptor->bLength, DEFAULT_TIMEOUT);
    if (ret < 0) {
      PRINT_ERROR_LIBUSB("libusb_control_transfer", ret);
      free(data);
      return -1;
    }
  }
    
  return add_descriptor(device, (LIBUSB_DT_STRING << 8) | index, descriptors->langId0.wData[0], ret, data);
}

static int probe_interface (struct gusb_device * device, unsigned char configurationIndex, struct usb_interface_descriptor * interface) {

  struct p_configuration * pConfiguration = device->descriptors.configurations + configurationIndex;

  if (interface->bInterfaceNumber >= pConfiguration->descriptor->bNumInterfaces) {
    PRINT_ERROR_OTHER("bad interface number");
    return -1;
  }

  struct p_interface * pInterface = pConfiguration->interfaces + interface->bInterfaceNumber;

  void * altInterfaces = realloc(pInterface->altInterfaces, (pInterface->bNumAltInterfaces + 1) * sizeof(*pInterface->altInterfaces));
  if(altInterfaces == NULL) {
    PRINT_ERROR_ALLOC_FAILED("realloc");
    return -1;
  }

  pInterface->altInterfaces = altInterfaces;
  memset(pInterface->altInterfaces + pInterface->bNumAltInterfaces, 0x00, sizeof(*pInterface->altInterfaces));
  pInterface->altInterfaces[pInterface->bNumAltInterfaces].descriptor = interface;
  ++pInterface->bNumAltInterfaces;

  if (interface->iInterface) {
    int ret = get_string_descriptor (device, interface->iInterface);
    if (ret < 0) {
      return -1;
    }
  }

  return 0;
}

struct p_altInterface * get_interface (struct gusb_device * device, unsigned char configurationIndex, struct usb_interface_descriptor * interface) {

  if (interface == NULL) {
      PRINT_ERROR_OTHER("missing interface");
      return NULL;
    }

    struct p_configuration * pConfiguration = device->descriptors.configurations + configurationIndex;

    if (interface->bInterfaceNumber >= pConfiguration->descriptor->bNumInterfaces) {
      PRINT_ERROR_OTHER("bad interface number");
      return NULL;
    }

    if (interface->bAlternateSetting >= pConfiguration->interfaces[interface->bInterfaceNumber].bNumAltInterfaces) {
      PRINT_ERROR_OTHER("bad alternative interface number");
      return NULL;
    }

    return pConfiguration->interfaces[interface->bInterfaceNumber].altInterfaces + interface->bAlternateSetting;
}

static int probe_hid (struct gusb_device * device, unsigned char configurationIndex, struct usb_interface_descriptor * interface, struct usb_hid_descriptor * hid) {

  struct p_altInterface * pAltInterface = get_interface(device, configurationIndex, interface);
  if (pAltInterface == NULL) {
    return -1;
  }
  
  if (pAltInterface->descriptor->bInterfaceClass != LIBUSB_CLASS_HID) {
    return 0;
  }
  
  pAltInterface->hidDescriptor = hid;

  unsigned char rdescIndex;
  for (rdescIndex = 0; rdescIndex < hid->bNumDescriptors; ++ rdescIndex) {
    if (hid->rdesc[rdescIndex].wReportDescriptorLength > 0) {
      unsigned char * data = calloc(hid->rdesc[rdescIndex].wReportDescriptorLength, sizeof(*data));
      if (data == NULL) {
        PRINT_ERROR_ALLOC_FAILED("calloc");
        return -1;
      }
      int ret = libusb_control_transfer(device->devh, LIBUSB_ENDPOINT_IN | LIBUSB_RECIPIENT_INTERFACE,
          LIBUSB_REQUEST_GET_DESCRIPTOR, (hid->rdesc[rdescIndex].bReportDescriptorType << 8) | 0, pAltInterface->descriptor->bInterfaceNumber, data, hid->rdesc[rdescIndex].wReportDescriptorLength, DEFAULT_TIMEOUT);
      if (ret < 0) {
        PRINT_ERROR_LIBUSB("libusb_control_transfer", ret);
        free(data);
        return -1;
      }
      
      return add_descriptor(device, (hid->rdesc[rdescIndex].bReportDescriptorType << 8), pAltInterface->descriptor->bInterfaceNumber, ret, data);
    }
  }

  return 0;
}

static int probe_endpoint (struct gusb_device * device, unsigned char configurationIndex, struct usb_interface_descriptor * interface, struct usb_endpoint_descriptor * endpoint) {

  struct p_altInterface * pAltInterface = get_interface(device, configurationIndex, interface);
  if (pAltInterface == NULL) {
    return -1;
  }

  void * endpoints = realloc(pAltInterface->endpoints, (pAltInterface->bNumEndpoints + 1) * sizeof(*pAltInterface->endpoints));
  if(endpoints == NULL) {
    PRINT_ERROR_ALLOC_FAILED("realloc");
    return -1;
  }

  pAltInterface->endpoints = endpoints;
  memset(pAltInterface->endpoints + pAltInterface->bNumEndpoints, 0x00, sizeof(*pAltInterface->endpoints));
  pAltInterface->endpoints[pAltInterface->bNumEndpoints] = endpoint;
  ++pAltInterface->bNumEndpoints;

  uint16_t size = endpoint->wMaxPacketSize;
  uint8_t type = endpoint->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK;
  uint8_t endpointNumber = endpoint->bEndpointAddress & LIBUSB_ENDPOINT_ADDRESS_MASK;
  if (endpointNumber > 0) {
    if (IS_ENDPOINT_IN(endpoint->bEndpointAddress)) {
      device->endpoints[endpointNumber - 1].in.type = type;
      device->endpoints[endpointNumber - 1].in.size = size;
    } else {
      device->endpoints[endpointNumber - 1].out.type = type;
      device->endpoints[endpointNumber - 1].out.size = size;
    }
  }

  return 0;
}

static int probe_configurations (struct gusb_device * device) {

  s_usb_descriptors * descriptors = &device->descriptors;

  int ret;

  unsigned char index;
  for (index = 0; index < descriptors->device.bNumConfigurations; ++index) {
  
    void * ptr = descriptors->configurations[index].raw;

    descriptors->configurations[index].descriptor = ptr;
    struct usb_config_descriptor * configuration = ptr;

    descriptors->configurations[index].interfaces = calloc(configuration->bNumInterfaces, sizeof(*descriptors->configurations[index].interfaces));
    if (descriptors->configurations[index].interfaces == NULL) {
        PRINT_ERROR_ALLOC_FAILED("calloc");
        return -1;
    }
  
    if (configuration->iConfiguration) {
      ret = get_string_descriptor (device, configuration->iConfiguration);
      if (ret < 0) {
        return -1;
      }
    }
    
    ptr += configuration->bLength;

    struct usb_interface_descriptor * interface = NULL;
  
    while (ptr < (void *)configuration + configuration->wTotalLength) {
      
      struct usb_descriptor_header * header = ptr;
      
      switch (header->bDescriptorType) {
      case LIBUSB_DT_INTERFACE:
      interface = ptr;
      ret = probe_interface(device, index, ptr);
      if (ret < 0) {
        return -1;
      }
      break;
      case LIBUSB_DT_ENDPOINT:
      ret = probe_endpoint(device, index, interface, ptr);
      if (ret < 0) {
        return -1;
      }
      break;
      case LIBUSB_DT_HID:
        ret = probe_hid(device, index, interface, ptr);
        if (ret < 0) {
          return -1;
        }
      break;
      case LIBUSB_DT_CONFIG:
      case LIBUSB_DT_REPORT:
      case LIBUSB_DT_PHYSICAL:
      case LIBUSB_DT_DEVICE:
      case LIBUSB_DT_STRING:
      default:
      break;
      }
      
      ptr += header->bLength;
    }
  }
  
  return 0;
}

static int get_device (struct gusb_device * device) {

  struct usb_device_descriptor * descriptor = &device->descriptors.device;

  int ret = libusb_control_transfer(device->devh, LIBUSB_ENDPOINT_IN,
      LIBUSB_REQUEST_GET_DESCRIPTOR, (LIBUSB_DT_DEVICE << 8) | 0, 0, (unsigned char *)descriptor,
      sizeof(*descriptor), DEFAULT_TIMEOUT);
  
  if (ret < 0) {
    PRINT_ERROR_LIBUSB("libusb_control_transfer", ret);
    return -1;
  }
  
  if (ret != sizeof(*descriptor)) {
    PRINT_ERROR_OTHER("invalid device descriptor size");
    return -1;
  }

  if (descriptor->iManufacturer) {
    ret = get_string_descriptor (device, descriptor->iManufacturer);
    if (ret < 0) {
      return -1;
    }
  }
  
  if (descriptor->iProduct) {
    ret = get_string_descriptor (device, descriptor->iProduct);
    if (ret < 0) {
      return -1;
    }
  }
  
  if (descriptor->iSerialNumber) {
    ret = get_string_descriptor (device, descriptor->iSerialNumber);
    if (ret < 0) {
      return -1;
    }
  }

  return 0;
}

static int get_descriptors (struct gusb_device * device) {
  
  int ret = get_device(device);
  if (ret < 0) {
    return -1;
  }
  
  if(device->descriptors.device.bNumConfigurations == 0) {
    PRINT_ERROR_OTHER("device has no configuration");
    return -1;
  }
  
  ret = get_configurations(device);
  if (ret < 0) {
    return -1;
  }
  
  return probe_configurations(device);
}

static int handle_interfaces(struct gusb_device * device, int claim) {

  libusb_device * dev = libusb_get_device(device->devh);
  if (dev == NULL) {
    PRINT_ERROR_OTHER("libusb_get_device failed");
    return -1;
  }

  struct libusb_device_descriptor desc;
  int ret = libusb_get_device_descriptor(dev, &desc);
  if (ret != LIBUSB_SUCCESS) {
    PRINT_ERROR_LIBUSB("libusb_get_device_descriptor", ret);
    return -1;
  }

  if (desc.bNumConfigurations) {

    struct libusb_config_descriptor * configuration;
    ret = libusb_get_config_descriptor(dev, 0, &configuration);
    if (ret != LIBUSB_SUCCESS) {
      return -1;
    }
    int interfaceIndex;
    for (interfaceIndex = 0; interfaceIndex < configuration->bNumInterfaces; ++interfaceIndex) {
      const struct libusb_interface * interface = configuration->interface + interfaceIndex;
      if(claim) {
        ret = libusb_claim_interface(device->devh,  interface->altsetting->bInterfaceNumber);
        if (ret != LIBUSB_SUCCESS) {
          PRINT_ERROR_LIBUSB("libusb_claim_interface", ret);
          libusb_free_config_descriptor(configuration);
          return -1;
        }
      } else {
        ret = libusb_release_interface(device->devh, interface->altsetting->bInterfaceNumber); //warning: this is a blocking function
        if (ret != LIBUSB_SUCCESS) {
          PRINT_ERROR_LIBUSB("libusb_release_interface", ret);
          libusb_free_config_descriptor(configuration);
          return -1;
        }
      }
    }
    libusb_free_config_descriptor(configuration);
  }

  return 0;
}

static int claim_device(struct gusb_device * device, libusb_device * dev) {

  int ret = libusb_open(dev, &device->devh);
  if (ret != LIBUSB_SUCCESS) {
    PRINT_ERROR_LIBUSB("libusb_open", ret);
    return -1;
  }

#if defined(LIBUSB_API_VERSION) || defined(LIBUSBX_API_VERSION)
  libusb_set_auto_detach_kernel_driver(device->devh, 1);
#else
#ifndef WIN32
  ret = libusb_kernel_driver_active(device->devh, 0);
  if(ret == 1)
  {
    ret = libusb_detach_kernel_driver(device->devh, 0);
    if(ret != LIBUSB_SUCCESS)
    {
      PRINT_ERROR_LIBUSB("libusb_detach_kernel_driver", ret);
      return -1;
    }
  }
  else if(ret != LIBUSB_SUCCESS)
  {
    PRINT_ERROR_LIBUSB("libusb_kernel_driver_active", ret);
    return -1;
  }
#endif
#endif

  int configuration;
  
  ret = libusb_get_configuration(device->devh, &configuration);
  if (ret != LIBUSB_SUCCESS) {
    PRINT_ERROR_LIBUSB("libusb_get_configuration", ret);
    return -1;
  }

  if (configuration == 0) {
    configuration = 1;
    ret = libusb_set_configuration(device->devh, 1); //warning: this is a blocking function
    if (ret != LIBUSB_SUCCESS) {
      PRINT_ERROR_LIBUSB("libusb_set_configuration", ret);
      return -1;
    }
  }

  ret = handle_interfaces(device, 1);
  if(ret < 0) {
      return -1;
  }

  // Don't use libusb_get_config_descriptor: it squeezes out some parts of the descriptor!
  ret = get_descriptors(device);
  if(ret < 0) {
      return -1;
  }

  ret = libusb_reset_device(device->devh);
  if (ret != LIBUSB_SUCCESS) {
    PRINT_ERROR_LIBUSB("libusb_reset_device", ret);
    return -1;
  }

  return 0;
}

struct gusb_device_info * gusb_enumerate(unsigned short vendor, unsigned short product) {

  struct gusb_device_info * devs = NULL;
  struct gusb_device_info * last = NULL;

  int ret = -1;

  static libusb_device** usb_devs = NULL;
  static ssize_t cnt = 0;
  int dev_i;

  libusb_context * ctx = NULL;

  ret = libusb_init(&ctx);
  if (ret != LIBUSB_SUCCESS) {
      PRINT_ERROR_LIBUSB("libusb_init", ret);
      return NULL;
  }
#if defined(WIN32) && (LIBUSB_API_VERSION >= 0x01000106)
  ret = libusb_set_option(ctx, LIBUSB_OPTION_USE_USBDK);
  if (ret == LIBUSB_ERROR_NOT_FOUND) {
    PRINT_ERROR_OTHER("UsbDk is not installed");
    libusb_exit(ctx);
    return NULL;
  }
#endif

  cnt = libusb_get_device_list(ctx, &usb_devs);
  if (cnt < 0) {
    PRINT_ERROR_LIBUSB("libusb_get_device_list", cnt);
    libusb_exit(ctx);
    return NULL;
  }

  for (dev_i = 0; dev_i < cnt; ++dev_i) {
    struct libusb_device_descriptor desc;
    ret = libusb_get_device_descriptor(usb_devs[dev_i], &desc);
    if (!ret) {
      if (vendor) {
        if (desc.idVendor != vendor) {
          continue;
        }
        if (product) {
          if (desc.idProduct != product) {
            continue;
          }
        }
      }

      const char * spath = make_path(usb_devs[dev_i]);
      if (spath == NULL) {
        continue;
      }

      char * path = strdup(spath);
      if (path == NULL) {
        PRINT_ERROR_OTHER("strdup failed");
        continue;
      }

      void * ptr = malloc(sizeof(*devs));
      if (ptr == NULL) {
        PRINT_ERROR_ALLOC_FAILED("realloc");
        free(path);
        continue;
      }

      struct gusb_device_info * dev = ptr;

      dev->path = path;
      dev->vendor_id = desc.idVendor;
      dev->product_id = desc.idProduct;
      dev->next = NULL;

      if (devs == NULL) {
          devs = dev;
      } else {
          last->next = dev;
      }

      last = dev;
    }
  }

  libusb_free_device_list(usb_devs, 1);

  libusb_exit(ctx);

  return devs;
}

void gusb_free_enumeration(struct gusb_device_info * devs) {

  struct gusb_device_info * current = devs;
  while (current != NULL) {
    struct gusb_device_info * next = current->next;
    free(current->path);
    free(current);
    current = next;
  }
}

struct gusb_device * gusb_open_ids(unsigned short vendor, unsigned short product) {

  int ret = -1;

  static libusb_device** devs = NULL;
  static ssize_t cnt = 0;
  int dev_i;

  libusb_context * ctx = NULL;

  ret = libusb_init(&ctx);
  if (ret != LIBUSB_SUCCESS) {
      PRINT_ERROR_LIBUSB("libusb_init", ret);
      return NULL;
  }
#if defined(WIN32) && (LIBUSB_API_VERSION >= 0x01000106)
  ret = libusb_set_option(ctx, LIBUSB_OPTION_USE_USBDK);
  if (ret == LIBUSB_ERROR_NOT_FOUND) {
    PRINT_ERROR_OTHER("UsbDk is not installed");
    libusb_exit(ctx);
    return NULL;
  }
#endif

  cnt = libusb_get_device_list(ctx, &devs);
  if (cnt < 0) {
    PRINT_ERROR_LIBUSB("libusb_get_device_list", cnt);
    libusb_exit(ctx);
    return NULL;
  }

  for (dev_i = 0; dev_i < cnt; ++dev_i) {
    struct libusb_device_descriptor desc;
    ret = libusb_get_device_descriptor(devs[dev_i], &desc);
    if (!ret) {
      if (desc.idVendor == vendor && desc.idProduct == product) {

        const char * spath = make_path(devs[dev_i]);
        if (spath == NULL) {
          continue;
        }

        struct gusb_device * device = add_device(spath, 0);
        if (device == NULL) {
          continue;
        }

        if (claim_device(device, devs[dev_i]) != -1) {
          libusb_free_device_list(devs, 1);
          device->ctx = ctx;
          return device;
        } else {
          gusb_close(device);
        }
      }
    }
  }

  libusb_free_device_list(devs, 1);

  libusb_exit(ctx);

  return NULL;
}

struct gusb_device * gusb_open_path(const char * path) {

  int ret = -1;

  static libusb_device** devs = NULL;
  static ssize_t cnt = 0;
  int dev_i;

  if (path == NULL) {
    PRINT_ERROR_OTHER("path is NULL");
    return NULL;
  }

  libusb_context * ctx = NULL;

  ret = libusb_init(&ctx);
  if (ret != LIBUSB_SUCCESS) {
      PRINT_ERROR_LIBUSB("libusb_init", ret);
      return NULL;
  }
#if defined(WIN32) && (LIBUSB_API_VERSION >= 0x01000106)
  ret = libusb_set_option(ctx, LIBUSB_OPTION_USE_USBDK);
  if (ret == LIBUSB_ERROR_NOT_FOUND) {
    PRINT_ERROR_OTHER("UsbDk is not installed");
    libusb_exit(ctx);
    return NULL;
  }
#endif

  cnt = libusb_get_device_list(ctx, &devs);
  if (cnt < 0) {
    PRINT_ERROR_LIBUSB("libusb_get_device_list", cnt);
    libusb_exit(ctx);
    return NULL;
  }

  for (dev_i = 0; dev_i < cnt; ++dev_i) {
    const char * spath = make_path(devs[dev_i]);
    if (spath == NULL || strcmp(spath, path)) {
      continue;
    }
    struct libusb_device_descriptor desc;
    ret = libusb_get_device_descriptor(devs[dev_i], &desc);
    if (!ret) {

      struct gusb_device * device = add_device(path, 0);
      if (device == NULL) {
        continue;
      }

      if (claim_device(device, devs[dev_i]) != -1) {
        libusb_free_device_list(devs, 1);
        device->ctx = ctx;
        return device;
      } else {
        gusb_close(device);
      }
    }
  }

  libusb_free_device_list(devs, 1);

  libusb_exit(ctx);

  return NULL;
}

const s_usb_descriptors * gusb_get_usb_descriptors(struct gusb_device * device) {

  return &device->descriptors;
}

#ifdef WIN32
static int usb_timer_read(void * user __attribute__((unused)))
{
  struct gusb_device * current;
  for (current = GLIST_BEGIN(usb_devices); current != GLIST_END(usb_devices); current = current->next) {
      handle_events(current);
  }
  return 0;
}

static int usb_timer_close(void * user __attribute__((unused)))
{
  return 1;
}

static int usb_timer_start() {

  if (usb_timer == NULL) {
    /*
     * Create a 1ms periodic timer to handle events.
     */
    GTIMER_CALLBACKS callbacks = {
            .fp_read = usb_timer_read,
            .fp_close = usb_timer_close,
            .fp_register = gpoll_register_handle,
            .fp_remove = gpoll_remove_handle
    };
    usb_timer = gtimer_start(NULL, 1000, &callbacks);
    if (usb_timer == NULL) {
      return -1;
    }
  }
  return 0;
}
#endif

#ifndef WIN32
static int close_callback(void * user) {

  struct gusb_device * device = (struct gusb_device *) user;

  return device->callbacks.fp_close(device->user);
}

static inline int pollfd_register(struct gusb_device * device, int fd, short events) {

    GPOLL_CALLBACKS callbacks = {
            .fp_read = (events & POLLIN) ? handle_events : NULL,
            .fp_write = (events & POLLOUT) ? handle_events : NULL,
            .fp_close = close_callback
    };

    return device->callbacks.fp_register(fd, device, &callbacks);
}

static void pollfd_added_cb (int fd, short events, void * user_data) {

    struct gusb_device * device = (struct gusb_device *) user_data;

  pollfd_register(device, fd, events);
}

static inline void pollfd_remove (struct gusb_device * device, int fd) {

  device->callbacks.fp_remove(fd);
}

static void pollfd_removed_cb (int fd, void * user_data) {

  struct gusb_device * device = (struct gusb_device *) user_data;

  pollfd_remove(device, fd);
}

static int set_notifiers(struct gusb_device * device, libusb_context * ctx) {

    int ret = 0;
    libusb_set_pollfd_notifiers(ctx, pollfd_added_cb, pollfd_removed_cb, (void *)(intptr_t) device);
    const struct libusb_pollfd ** pollfds = libusb_get_pollfds(ctx);
    int i;
    for (i = 0; pollfds[i] != NULL && ret != -1; ++i) {
      ret = pollfd_register(device, pollfds[i]->fd, pollfds[i]->events);
    }
    free(pollfds);

    if (ret == -1) {
        // roll-back
        for (i = i - 1; i >= 0; --i) {
            pollfd_remove(device, pollfds[i]->fd);
        }
        libusb_set_pollfd_notifiers(ctx, NULL, NULL, NULL);
        return -1;
    }

    return 0;
}
#endif

int gusb_register(struct gusb_device * device, void * user, const GUSB_CALLBACKS * callbacks) {

  if (callbacks->fp_register == NULL) {

    PRINT_ERROR_OTHER("fp_register is NULL");
    return -1;
  }

  if (callbacks->fp_remove == NULL) {

    PRINT_ERROR_OTHER("fp_remove is NULL");
    return -1;
  }

  int ret = 0;

  device->callbacks = *callbacks;

#ifdef WIN32
  ret = usb_timer_start();
#else
  ret = set_notifiers(device, device->ctx);
#endif

  if (ret < 0) {
      memset(&device->callbacks, 0x00, sizeof(device->callbacks));
      return -1;
  }

  device->user = user;

  return ret;
}

/*
 * Cancel all pending transfers for a given device.
 */
static void cancel_transfers(struct gusb_device * device) {
  unsigned int i;
  for (i = 0; i < device->transfers_nb; ++i) {

      libusb_cancel_transfer(device->transfers[i]);
  }

  while (device->transfers_nb > 0) {

    if (libusb_handle_events(device->ctx) != LIBUSB_SUCCESS) {

      break;
    }
  }
}

int gusb_close(struct gusb_device * device) {

  device->closing = 1;

  if (device->devh) {

    cancel_transfers(device);

    handle_interfaces(device, 0); //warning: this is a blocking function
#if !defined(LIBUSB_API_VERSION) && !defined(LIBUSBX_API_VERSION)
#ifndef WIN32
        libusb_attach_kernel_driver(device->devh, 0);
#endif
#endif
    libusb_close(device->devh);
  }

  if (device->ctx != NULL) {
      libusb_exit(device->ctx);
  }

  free(device->path);
  if (device->descriptors.configurations != NULL) {
    unsigned char configurationIndex;
    for (configurationIndex = 0; configurationIndex < device->descriptors.device.bNumConfigurations; ++configurationIndex) {
      struct p_configuration * pConfiguration = device->descriptors.configurations + configurationIndex;
      if (pConfiguration->descriptor != NULL) {
        unsigned char interfaceIndex;
        for (interfaceIndex = 0; interfaceIndex < pConfiguration->descriptor->bNumInterfaces; ++interfaceIndex) {
          struct p_interface * pInterface = pConfiguration->interfaces + interfaceIndex;
          unsigned char altInterfaceIndex;
          for (altInterfaceIndex = 0; altInterfaceIndex < pInterface->bNumAltInterfaces; ++altInterfaceIndex) {
            struct p_altInterface * pAltInterface = pInterface->altInterfaces + altInterfaceIndex;
            free(pAltInterface->endpoints);
          }
          free(pInterface->altInterfaces);
        }
        free(pConfiguration->interfaces);
      }
      free(pConfiguration->raw);
    }
  }
  free(device->descriptors.configurations);
  unsigned int othersIndex;
  for (othersIndex = 0; othersIndex < device->descriptors.nbOthers; ++othersIndex) {
    free(device->descriptors.others[othersIndex].data);
  }
  free(device->descriptors.others);

  GLIST_REMOVE(usb_devices, device);

  free(device);

#ifdef WIN32
  if (GLIST_IS_EMPTY(usb_devices)) {
    if (usb_timer != NULL) {
      gtimer_close(usb_timer);
    }
  }
#endif

  return 1;
}

int gusb_write(struct gusb_device * device, unsigned char endpoint, const void * buf, unsigned int count) {

  if (endpoint != 0) {

    // TODO MLA: replace 0 with count?
    unsigned char endpointIndex = get_endpoint(device, endpoint, LIBUSB_ENDPOINT_OUT, 0);
    if(endpointIndex == INVALID_ENDPOINT_INDEX) {

      return -1;
    }
  } else {

    struct libusb_control_setup * control_setup = (struct libusb_control_setup *)buf;
    if(control_setup->bmRequestType & LIBUSB_ENDPOINT_IN) {

      count += control_setup->wLength;
    }
  }

  if (device->callbacks.fp_write == NULL) {

    PRINT_ERROR_OTHER("missing write callback");
    return -1;
  }

  unsigned char * buffer = malloc(count * sizeof(*buffer));
  if (buffer == NULL) {

    PRINT_ERROR_ALLOC_FAILED("calloc");
    return -1;
  }

  memcpy(buffer, buf, count);

  struct libusb_transfer * transfer = libusb_alloc_transfer(0);
  if (transfer == NULL) {

    PRINT_ERROR_ALLOC_FAILED("libusb_alloc_transfer");
    free(buffer);
    return -1;
  }

  if (endpoint == 0) {

    libusb_fill_control_transfer(transfer, device->devh,
        buffer, (libusb_transfer_cb_fn) usb_callback, (void *) (intptr_t) device, CONTROL_TIMEOUT);
  } else {

    libusb_fill_interrupt_transfer(transfer, device->devh, endpoint,
        buffer, count, (libusb_transfer_cb_fn) usb_callback, (void *) (intptr_t) device, OUT_TIMEOUT);
  }

  return submit_transfer(transfer);
}

const char * gusb_get_path(struct gusb_device * device) {

    return device->path;
}
