
#include "lv_buttons.h"

LV_BUTTONS lv_buttons = {0};

void encoder_init(void);

void encoder_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data);

void encoder_handler(void);

void buttons_init(void);

void button_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data);

int8_t button_get_pressed_id(void);

bool button_is_pressed(uint8_t id);



void lv_buttons_init(TIM_HandleTypeDef *timEncoderHandle) {
    lv_buttons.timEncoderHandle = timEncoderHandle;

    static lv_indev_drv_t indev_drv;

    lv_group_t *default_group = lv_group_create();
    lv_group_set_default(default_group);

    /*------------------
     * Encoder
     * -----------------*/

    /*Initialize your encoder if you have*/
    encoder_init();

    /*Register a encoder input device*/
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_ENCODER;
    indev_drv.read_cb = encoder_read;
    lv_buttons.indev_encoder = lv_indev_drv_register(&indev_drv);

    lv_indev_set_group(lv_buttons.indev_encoder, default_group);

    /*Later you should create group(s) with `lv_group_t * group = lv_group_create()`,
     *add objects to the group with `lv_group_add_obj(group, obj)`
     *and assign this input device to group to navigate in it:
     *`lv_indev_set_group(indev_encoder, group);`*/

    /*------------------
     * Button
     * -----------------*/

    /*Initialize your button if you have*/
    buttons_init();

    /*Register a button input device*/
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_BUTTON;
    indev_drv.read_cb = button_read;
    lv_buttons.indev_button = lv_indev_drv_register(&indev_drv);

    lv_indev_set_group(lv_buttons.indev_button, default_group);

    /*Assign buttons to points on the screen*/
    static const lv_point_t btn_points[2] = {
            {10, 10},   /*Button 0 -> x:10; y:10*/
            {40, 100},  /*Button 1 -> x:40; y:100*/
    };
    lv_indev_set_button_points(lv_buttons.indev_button, btn_points);
}



/*------------------
 * Encoder
 * -----------------*/

// Initialize your encoder
void encoder_init(void) {
    lv_buttons.encoder_value_prev = __HAL_TIM_GET_COUNTER(lv_buttons.timEncoderHandle);
}

/*Will be called by the library to read the encoder*/
void encoder_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {
    data->enc_diff = lv_buttons.encoder_diff;
    data->state = lv_buttons.encoder_state;
}

/*Call this function in an interrupt to process encoder events (turn, press)*/
void lv_buttons_encoder_handler(void) {
    //encoder_value_curr = __HAL_TIM_GET_COUNTER(ENCODER_PORT);

    int32_t diff;

  if (__HAL_TIM_IS_TIM_COUNTING_DOWN(lv_buttons.timEncoderHandle))
  {
    diff = lv_buttons.encoder_value_prev - lv_buttons.encoder_value_curr;
    if (lv_buttons.encoder_value_curr >= lv_buttons.encoder_value_prev)
      diff += 65535;

  } else {
    diff = lv_buttons.encoder_value_curr - lv_buttons.encoder_value_prev;

    if (lv_buttons.encoder_value_curr <= lv_buttons.encoder_value_prev)
      diff += 65535;
  }

    lv_buttons.encoder_value_prev = lv_buttons.encoder_value_curr;

    lv_buttons.encoder_diff += diff;

    lv_buttons.encoder_state = LV_INDEV_STATE_RELEASED; //LV_INDEV_STATE_RELEASED | LV_INDEV_STATE_PRESSED
}

/*------------------
 * Button
 * -----------------*/

/*Initialize your buttons*/
void buttons_init(void) {
    /*Your code comes here*/
}

/*Will be called by the library to read the button*/
void button_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {

    static uint8_t last_btn = 0;

    /*Get the pressed button's ID*/
    int8_t btn_act = button_get_pressed_id();

    if (btn_act >= 0) {
        data->state = LV_INDEV_STATE_PR;
        last_btn = btn_act;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }

    /*Save the last pressed button's ID*/
    data->btn_id = last_btn;

    //data->continue_reading
}

/*Get ID  (0, 1, 2 ..) of the pressed button*/
int8_t button_get_pressed_id(void) {
    uint8_t i;

    /*Check to buttons see which is being pressed (assume there are 2 buttons)*/
    for (i = 0; i < 2; i++) {
        /*Return the pressed button's ID*/
        if (button_is_pressed(i)) {
            return i;
        }
    }

    /*No button pressed*/
    return -1;
}

/*Test if `id` button is pressed or not*/
bool button_is_pressed(uint8_t id) {

    /*Your code comes here*/

    return false;
}

