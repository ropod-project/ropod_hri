// Based on https://github.com/b-it-bots/led_control by Torsten Jandt

#include <Adafruit_CircuitPlayground.h>

#define BUFFER_SIZE 128

#define COMMAND_OFF 0
// command_id r g b
#define COMMAND_FULL 1
// command dir speed r g b
#define COMMAND_ROTATE 2
// command speed r g b
#define COMMAND_BLINK 3
// command speed r g b limit
#define COMMAND_BLINK_LIMIT 4
// command speed r g b
#define COMMAND_BLINK_BUZZER 5
// command speed r g b limit
#define COMMAND_BLINK_LIMIT_BUZZER 6
// command progress (0 - 100)
#define COMMAND_PROGRESS 7
// command q1 q2 q3 q4 r g b (quadrant: 1-4)
#define COMMAND_QUADRANT 8
// command speed q1 q2 q3 q4 r g b (quadrant: 1-4)
#define COMMAND_QUADRANT_BLINK 9
#define NUM_LEDS 10

#define DELAY 10
#define SOUND_DELAY_MULTIPLIER 100

uint8_t command = COMMAND_OFF;
String command_data = "";
bool received_command = true;

String buffer = "";

void setup() {
    CircuitPlayground.begin();
    CircuitPlayground.clearPixels();
    Serial.begin(9600);

    //command = COMMAND_BLINK_LIMIT;
    //command_data = "50 255 0 0 5";
}

uint32_t mul_color(uint32_t color, float f);

void parse_buffer();
void command_full();
void command_rotate();
void command_blink();
void command_blink_limit();

void loop() {
    while(Serial.available() > 0) {
        uint8_t byte = Serial.read();
        CircuitPlayground.redLED(true);
        if(byte == '\n') {
            parse_buffer();
            break;
        } else {
            buffer += (char) byte;
        }
    }
    if (CircuitPlayground.leftButton() or CircuitPlayground.rightButton())
    {
        command = COMMAND_OFF;
    }
    CircuitPlayground.redLED(false);
    switch(command) {
        case COMMAND_OFF:
            CircuitPlayground.clearPixels();
            break;
        case COMMAND_FULL:
            command_full();
            break;
        case COMMAND_BLINK:
            command_blink(false);
            break;
        case COMMAND_BLINK_BUZZER:
            command_blink(CircuitPlayground.slideSwitch());
            break;
        case COMMAND_BLINK_LIMIT:
            command_blink_limit(false);
            break;
        case COMMAND_BLINK_LIMIT_BUZZER:
            command_blink_limit(CircuitPlayground.slideSwitch());
            break;
        case COMMAND_ROTATE:
            command_rotate();
            break;
        case COMMAND_PROGRESS:
            command_progress();
            break;
        case COMMAND_QUADRANT:
            command_quadrant();
            break;
        case COMMAND_QUADRANT_BLINK:
            command_quadrant_blink();
            break;
        default:
            CircuitPlayground.clearPixels();
            break;
    }
    delay(DELAY);
}

int command_color_r = 0;
int command_color_g = 0;
int command_color_b = 0;


void command_full() {
    if(received_command) {
        int r, g, b;
        char chars[command_data.length()+1];
        command_data.toCharArray(chars, command_data.length()+1);
        sscanf(chars, "%d %d %d", &command_color_r, &command_color_g, &command_color_b);

        received_command = false;

        for (uint16_t i = 0; i < NUM_LEDS; i++)
        {
            CircuitPlayground.setPixelColor(i, command_color_r, command_color_g, command_color_b);
        }
    }
}


uint8_t command_blink_steps = 0;

uint8_t command_blink_count = 0;
uint8_t total_blink_count = 0;
bool command_blink_state = false;

void command_blink(bool play_buzzer) {
    if (total_blink_count % SOUND_DELAY_MULTIPLIER == 0)
    {
        if (play_buzzer)
        {
            buzzer();
        }
        total_blink_count = 0;
    }
    total_blink_count += 1;
    if(received_command) {
        int speed, r, g, b;
        char chars[command_data.length()+1];
        command_data.toCharArray(chars, command_data.length()+1);
        sscanf(chars, "%d %d %d %d", &speed, &command_color_r, &command_color_g, &command_color_b);
        command_blink_steps = (int)((float)speed / (float) DELAY);
        received_command = false;
        command_blink_state = false;
        total_blink_count = 0;
    }
    if(command_blink_count == 0) {
        CircuitPlayground.clearPixels();
        if(command_blink_state) {
            for (uint16_t i = 0; i < NUM_LEDS; i++) {
                CircuitPlayground.setPixelColor(i, command_color_r, command_color_g, command_color_b);
            }
        }
        command_blink_state = !command_blink_state;
    }
    command_blink_count = (command_blink_count + 1) % command_blink_steps;
}

uint8_t command_blink_limit_limit = 0;
uint8_t command_blink_limit_count = 0;

void command_blink_limit(bool play_buzzer) {
    if (total_blink_count % SOUND_DELAY_MULTIPLIER == 0)
    {
        if (play_buzzer)
        {
            buzzer();
        }
        total_blink_count = 0;
    }
    total_blink_count += 1;

    if(received_command) {
        int speed, r, g, b, limit;
        char chars[command_data.length()+1];
        command_data.toCharArray(chars, command_data.length()+1);
        sscanf(chars, "%d %d %d %d %d", &speed, &command_color_r, &command_color_g, &command_color_b, &limit);
        command_blink_steps = (int)((float)speed / (float) DELAY);
        command_blink_limit_limit = limit;
        command_blink_limit_count = 0;
        command_blink_state = false;
        received_command = false;
        total_blink_count = 0;
    }
    if(command_blink_count == 0 && (command_blink_limit_count < command_blink_limit_limit || !command_blink_state )) {
        CircuitPlayground.clearPixels();
        if(command_blink_state) {
            for (uint16_t i = 0; i < NUM_LEDS; i++) {
                CircuitPlayground.setPixelColor(i, command_color_r, command_color_g, command_color_b);
            }
        }
        command_blink_state = !command_blink_state;
        command_blink_limit_count++;
    }
    command_blink_count = (command_blink_count + 1) % command_blink_steps;
}

uint8_t command_rotate_dir = 0;
uint8_t command_rotate_steps = 0;

uint8_t command_rotate_pos = 0;
uint8_t command_rotate_count = 0;

void command_rotate() {
    if(received_command) {
        int dir, speed, r, g, b;
        char chars[command_data.length()+1];
        command_data.toCharArray(chars, command_data.length()+1);
        sscanf(chars, "%d %d %d %d %d", &dir, &speed, &command_color_r, &command_color_g, &command_color_b);
        command_rotate_dir = (bool) dir ? -1 : 1;
        command_rotate_steps = (int)((float)speed / (float) DELAY);
        received_command = false;
    }
    if(command_rotate_count == 0) {
        CircuitPlayground.clearPixels();
        for (uint16_t i = 0; i < NUM_LEDS; i++)
        {
            float sigma = 0.5f;
            float diff = min(((int)command_rotate_pos - (int)i), NUM_LEDS + (int)i - (int)command_rotate_pos);
            float f = (1.0f / sqrt(2.0f * PI * sigma)) * pow(EULER, -((diff*diff)/(2.0f*sigma)));

            CircuitPlayground.setPixelColor(i, command_color_r * f, command_color_g * f, command_color_b * f);
        }
        if (command_rotate_dir == 1)
        {
            command_rotate_pos = (command_rotate_pos + 1) % NUM_LEDS;
        }
        else
        {
            if (command_rotate_pos == 0)
            {
                command_rotate_pos = (uint8_t)(NUM_LEDS - 1);
            }
            else
            {
                command_rotate_pos = command_rotate_pos - 1;
            }
        }
    }
    command_rotate_count = (command_rotate_count + 1) % command_rotate_steps;
}

void command_progress()
{
    if (received_command)
    {
        int progress;
        char chars[command_data.length()+1];
        command_data.toCharArray(chars, command_data.length()+1);
        sscanf(chars, "%d", &progress);
        received_command = false;
        int complete = (int)((progress * NUM_LEDS) / 100);
        int partial = (progress * NUM_LEDS) - (complete * 100);
        for (uint16_t i = 0; i < NUM_LEDS; i++)
        {
            if (i < complete)
            {
                CircuitPlayground.setPixelColor(i, 0, 255, 0);
            }
            if (i > complete)
            {
                CircuitPlayground.setPixelColor(i, 255, 0, 0);
            }
            if (i == complete && partial > 0)
            {
                CircuitPlayground.setPixelColor(i, 80*((100-partial)/ 100.0f), 80*(partial / 100.0f), 0);
            }
            else if (i == complete)
            {
                CircuitPlayground.setPixelColor(i, 255, 0, 0);
            }
        }
    }

}

void command_quadrant()
{
    if (received_command)
    {
        int q1, q2, q3, q4;
        char chars[command_data.length()+1];
        command_data.toCharArray(chars, command_data.length()+1);
        sscanf(chars, "%d %d %d %d %d %d %d", &q1, &q2, &q3, &q4, &command_color_r, &command_color_g, &command_color_b);
        received_command = false;
        CircuitPlayground.clearPixels();
        if (q1 == 1)
        {
            CircuitPlayground.setPixelColor(0, command_color_r, command_color_g, command_color_b);
            CircuitPlayground.setPixelColor(1, command_color_r, command_color_g, command_color_b);
            CircuitPlayground.setPixelColor(2, command_color_r, command_color_g, command_color_b);
        }
        if (q2 == 1)
        {
            CircuitPlayground.setPixelColor(2, command_color_r, command_color_g, command_color_b);
            CircuitPlayground.setPixelColor(3, command_color_r, command_color_g, command_color_b);
            CircuitPlayground.setPixelColor(4, command_color_r, command_color_g, command_color_b);
        }
        if (q3 == 1)
        {
            CircuitPlayground.setPixelColor(5, command_color_r, command_color_g, command_color_b);
            CircuitPlayground.setPixelColor(6, command_color_r, command_color_g, command_color_b);
            CircuitPlayground.setPixelColor(7, command_color_r, command_color_g, command_color_b);
        }
        if (q4 == 1)
        {
            CircuitPlayground.setPixelColor(7, command_color_r, command_color_g, command_color_b);
            CircuitPlayground.setPixelColor(8, command_color_r, command_color_g, command_color_b);
            CircuitPlayground.setPixelColor(9, command_color_r, command_color_g, command_color_b);
        }
    }
}

uint8_t quadrant_0 = 0;
uint8_t quadrant_1 = 0;
uint8_t quadrant_2 = 0;
uint8_t quadrant_3 = 0;

void command_quadrant_blink()
{
    if (received_command)
    {
        int q1, q2, q3, q4;
        int speed;
        char chars[command_data.length()+1];
        command_data.toCharArray(chars, command_data.length()+1);
        sscanf(chars, "%d %d %d %d %d %d %d %d", &speed, &q1, &q2, &q3, &q4, &command_color_r, &command_color_g, &command_color_b);
        quadrant_0 = q1;
        quadrant_1 = q2;
        quadrant_2 = q3;
        quadrant_3 = q4;

        command_blink_steps = (int)((float)speed / (float) DELAY);
        received_command = false;
        command_blink_state = false;
        command_blink_count = 0;
    }
    if(command_blink_count == 0) {
        CircuitPlayground.clearPixels();
        if(command_blink_state) {
            if (quadrant_0 == 1)
            {
                CircuitPlayground.setPixelColor(0, command_color_r, command_color_g, command_color_b);
                CircuitPlayground.setPixelColor(1, command_color_r, command_color_g, command_color_b);
                CircuitPlayground.setPixelColor(2, command_color_r, command_color_g, command_color_b);
            }
            if (quadrant_1 == 1)
            {
                CircuitPlayground.setPixelColor(2, command_color_r, command_color_g, command_color_b);
                CircuitPlayground.setPixelColor(3, command_color_r, command_color_g, command_color_b);
                CircuitPlayground.setPixelColor(4, command_color_r, command_color_g, command_color_b);
            }
            if (quadrant_2 == 1)
            {
                CircuitPlayground.setPixelColor(5, command_color_r, command_color_g, command_color_b);
                CircuitPlayground.setPixelColor(6, command_color_r, command_color_g, command_color_b);
                CircuitPlayground.setPixelColor(7, command_color_r, command_color_g, command_color_b);
            }
            if (quadrant_3 == 1)
            {
                CircuitPlayground.setPixelColor(7, command_color_r, command_color_g, command_color_b);
                CircuitPlayground.setPixelColor(8, command_color_r, command_color_g, command_color_b);
                CircuitPlayground.setPixelColor(9, command_color_r, command_color_g, command_color_b);
            }
        }
        command_blink_state = !command_blink_state;
    }
    command_blink_count = (command_blink_count + 1) % command_blink_steps;
}


void buzzer()
{
    CircuitPlayground.playTone(100, 500, false);
}

void parse_buffer() {
    buffer.trim();
    command = buffer.toInt();
    int index = buffer.indexOf(' ');
    if(index > 0) {
        command_data = buffer.substring(index + 1, buffer.length());
    } else {
        command_data = "";
    }
    
    buffer = "";
    received_command = true;
}
