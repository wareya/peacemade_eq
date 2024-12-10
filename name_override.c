
#include <usb_names.h>

#define PRODUCT_NAME		{'P','e','a','c','e','m','a','d','e',' ','E','Q'}
#define PRODUCT_NAME_LEN	12

struct usb_string_descriptor_struct usb_string_product_name = {
    2 + PRODUCT_NAME_LEN * 2,
    3,
    PRODUCT_NAME
};

#define MIDI_PORT1_NAME {'P','e','a','c','e','m','a','d','e',' ','E','Q'}
#define MIDI_PORT1_NAME_LEN 12

struct usb_string_descriptor_struct usb_string_midi_port1 = {
        2 + MIDI_PORT1_NAME_LEN * 2,
        3,
        MIDI_PORT1_NAME
};