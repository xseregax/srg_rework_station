/**
 * @file enc_buttons.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "enc_buttons.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

void encoder_init(void);
void encoder_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data);
void encoder_handler(void);

void button_init(void);
void button_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data);
int8_t button_get_pressed_id(void);
bool button_is_pressed(uint8_t id);

/**********************
 *  STATIC VARIABLES
 **********************/
lv_indev_t * indev_encoder;
lv_indev_t * indev_button;

static int32_t encoder_diff;
static lv_indev_state_t encoder_state;

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_port_indev_init(void)
{
  static lv_indev_drv_t indev_drv;

  /*------------------
   * Encoder
   * -----------------*/

  /*Initialize your encoder if you have*/
  encoder_init();

  /*Register a encoder input device*/
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_ENCODER;
  indev_drv.read_cb = encoder_read;
  indev_encoder = lv_indev_drv_register(&indev_drv);

  /*Later you should create group(s) with `lv_group_t * group = lv_group_create()`,
   *add objects to the group with `lv_group_add_obj(group, obj)`
   *and assign this input device to group to navigate in it:
   *`lv_indev_set_group(indev_encoder, group);`*/

  /*------------------
   * Button
   * -----------------*/

  /*Initialize your button if you have*/
  button_init();

  /*Register a button input device*/
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_BUTTON;
  indev_drv.read_cb = button_read;
  indev_button = lv_indev_drv_register(&indev_drv);

  /*Assign buttons to points on the screen*/
  static const lv_point_t btn_points[2] = {
          {10, 10},   /*Button 0 -> x:10; y:10*/
          {40, 100},  /*Button 1 -> x:40; y:100*/
  };
  lv_indev_set_button_points(indev_button, btn_points);
}



/*------------------
 * Encoder
 * -----------------*/

/*Initialize your keypad*/
void encoder_init(void)
{
    /*Your code comes here*/
}

/*Will be called by the library to read the encoder*/
void encoder_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{

    data->enc_diff = encoder_diff;
    data->state = encoder_state;
}

/*Call this function in an interrupt to process encoder events (turn, press)*/
void encoder_handler(void)
{
    /*Your code comes here*/

    encoder_diff += 0;
    encoder_state = LV_INDEV_STATE_REL;
}

/*------------------
 * Button
 * -----------------*/

/*Initialize your buttons*/
void button_init(void)
{
    /*Your code comes here*/
}

/*Will be called by the library to read the button*/
void button_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{

    static uint8_t last_btn = 0;

    /*Get the pressed button's ID*/
    int8_t btn_act = button_get_pressed_id();

    if(btn_act >= 0) {
        data->state = LV_INDEV_STATE_PR;
        last_btn = btn_act;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }

    /*Save the last pressed button's ID*/
    data->btn_id = last_btn;
}

/*Get ID  (0, 1, 2 ..) of the pressed button*/
int8_t button_get_pressed_id(void)
{
    uint8_t i;

    /*Check to buttons see which is being pressed (assume there are 2 buttons)*/
    for(i = 0; i < 2; i++) {
        /*Return the pressed button's ID*/
        if(button_is_pressed(i)) {
            return i;
        }
    }

    /*No button pressed*/
    return -1;
}

/*Test if `id` button is pressed or not*/
bool button_is_pressed(uint8_t id)
{

    /*Your code comes here*/

    return false;
}

