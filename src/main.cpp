#include <mbed.h>
#include <vector>
#include "drivers/LCD_DISCO_F429ZI.h"
#include "drivers/TS_DISCO_F429ZI.h"
#include <array>
#include <limits>
#include <cmath>
#include <math.h>
// #include "gyro.h"


// Register addresses

#define CTRL_REG_1 0x20 // control register 1
#define CTRL_REG_3 0x22 // control register 3
#define CTRL_REG_4 0x23 // control register 4

#define OUT_X_L 0x28 // X-axis angular rate data Low

#define INT1_SRC 0x31 // interrupt 1 source register
// Output data rate selections and cutoff frequencies

#define ODR_200_CUTOFF_50 0x60

// Interrupt configurations
#define INT2_DRDY 0x08 // Data ready on DRDY/INT2 pin

// Fullscale selections
#define FULL_SCALE_245 0x00      // full scale 245 dps
#define FULL_SCALE_500 0x10      // full scale 500 dps
#define FULL_SCALE_2000 0x20     // full scale 2000 dps
#define FULL_SCALE_2000_ALT 0x30 // full scale 2000 dps

// Sensitivities in dps/digit
#define SENSITIVITY_245 0.00875f // 245 dps typical sensitivity
#define SENSITIVITY_500 0.0175f  // 500 dps typical sensitivity
#define SENSITIVITY_2000 0.07f   // 2000 dps typical sensitivity

// Convert constants
#define MY_LEG 1              // put board on left leg 0.8m above ground
#define DEGREE_TO_RAD 0.0175f // rad = dgree * (pi / 180)

#define POWERON 0x0f  // turn gyroscope
#define POWEROFF 0x00 // turnoff gyroscope


// Initialization parameters
typedef struct
{
    uint8_t conf1;       // output data rate
    uint8_t conf3;       // interrupt configuration
    uint8_t conf4;       // full sacle selection
} Gyroscope_Init_Parameters;

// Raw data
typedef struct
{
    int16_t x_raw; // X-axis raw data
    int16_t y_raw; // Y-axis raw data
    int16_t z_raw; // Z-axis raw data
} Gyroscope_RawData;


//initialize the SPI interface for communication with GYRO
SPI gyroscope(PF_9, PF_8, PF_7); // mosi, miso, sclk
DigitalOut cs(PC_1);

int16_t x_threshold; // X-axis calibration threshold
int16_t y_threshold; // Y-axis calibration threshold
int16_t z_threshold; // Z-axis calibration threshold

int16_t x_sample; // X-axis zero-rate level sample
int16_t y_sample; // Y-axis zero-rate level sample
int16_t z_sample; // Z-axis zero-rate level sample

float sensitivity = 0.0f;

Gyroscope_RawData *gyro_raw;

// Write I/O
void WriteByte(uint8_t address, uint8_t data)
{
  cs = 0;
  gyroscope.write(address);
  gyroscope.write(data);
  cs = 1;
}

// Get raw data from gyroscope
void GetGyroValue(Gyroscope_RawData *rawdata)
{
  cs = 0;
  gyroscope.write(OUT_X_L | 0x80 | 0x40); // auto-incremented read
  rawdata->x_raw = gyroscope.write(0xff) | gyroscope.write(0xff) << 8;
  rawdata->y_raw = gyroscope.write(0xff) | gyroscope.write(0xff) << 8;
  rawdata->z_raw = gyroscope.write(0xff) | gyroscope.write(0xff) << 8;
  cs = 1;
}

// Calibrate gyroscope before recording
// Find the "turn-on" zero rate level
// Set up thresholds for three axes
// Data below the corresponding threshold will be treated as zero to offset random vibrations when walking
void CalibrateGyroscope(Gyroscope_RawData *rawdata)
{
  int16_t sumX = 0;
  int16_t sumY = 0;
  int16_t sumZ = 0;
  for (int i = 0; i < 128; i++)
  {
    GetGyroValue(rawdata);
    sumX += rawdata->x_raw;
    sumY += rawdata->y_raw;
    sumZ += rawdata->z_raw;
    x_threshold = max(x_threshold, rawdata->x_raw);
    y_threshold = max(y_threshold, rawdata->y_raw);
    z_threshold = max(z_threshold, rawdata->z_raw);
    wait_us(10000);
  }

  x_sample = sumX >> 7; // 128 is 2^7
  y_sample = sumY >> 7;
  z_sample = sumZ >> 7;
}

// Initiate gyroscope, set up control registers
void InitiateGyroscope(Gyroscope_Init_Parameters *init_parameters, Gyroscope_RawData *init_raw_data)
{
  gyro_raw = init_raw_data;
  cs = 1;
  // set up gyroscope
  gyroscope.format(8, 3);       // 8 bits per SPI frame; polarity 1, phase 0
  gyroscope.frequency(1000000); // clock frequency deafult 1 MHz max:10MHz

  WriteByte(CTRL_REG_1, init_parameters->conf1 | POWERON); // set ODR Bandwidth and enable all 3 axises
  WriteByte(CTRL_REG_3, init_parameters->conf3);           // DRDY enable
  WriteByte(CTRL_REG_4, init_parameters->conf4);           // LSB, full sacle selection: 500dps

  switch (init_parameters->conf4)
  {
  case FULL_SCALE_245:
    sensitivity = SENSITIVITY_245;
    break;

  case FULL_SCALE_500:
    sensitivity = SENSITIVITY_500;
    break;

  case FULL_SCALE_2000:
    sensitivity = SENSITIVITY_2000;
    break;

  case FULL_SCALE_2000_ALT:
    sensitivity = SENSITIVITY_2000;
    break;
  }

  CalibrateGyroscope(gyro_raw); // calibrate the gyroscope and find the threshold for x, y, and z.
}

// convert raw data to dps
float ConvertToDPS(int16_t axis_data)
{
  float dps = axis_data * sensitivity;
  return dps;
}

// convert raw data to calibrated data directly
void GetCalibratedRawData()
{
  GetGyroValue(gyro_raw);

  // offset the zero rate level
  gyro_raw->x_raw -= x_sample;
  gyro_raw->y_raw -= y_sample;
  gyro_raw->z_raw -= z_sample;

  // put data below threshold to zero
  if (abs(gyro_raw->x_raw) < abs(x_threshold))
    gyro_raw->x_raw = 0;
  if (abs(gyro_raw->y_raw) < abs(y_threshold))
    gyro_raw->y_raw = 0;
  if (abs(gyro_raw->z_raw) < abs(z_threshold))
    gyro_raw->z_raw = 0;
}


vector<array<float, 3>> gesture_key; // the gesture key
vector<array<float, 3>> unlocking_record; // the unlocking record

const int ButtonX = 10; //button for screen
const int ButtonY = 100;
const int Button1Width = 100;
const int Button1Height = 50;
const char *Button1Label = "UNLOCK";

int err = 0; // for error checking

// tollerance
#define EPSILON 0.45f

//Text size
#define TEXT_SIZE 24

// Events
#define IS_GUESTURE_SAVED 1
#define IS_OPEN 2
#define DELETE 4
#define READY 8

InterruptIn gyro_int2(PA_2, PullDown);
InterruptIn ResetButton(USER_BUTTON, PullDown);

DigitalOut green_led(LED1);
DigitalOut red_led(LED2);

// LCD object
LCD_DISCO_F429ZI lcd; 

// Touch screen object
TS_DISCO_F429ZI ts; 

EventFlags flags; // Event flags

Timer timer; // Timer

const int Button2X = 121; //button for screen
const int Button2Y = 100;
const int Button2Width = 100;
const int Button2Height = 50;
const char *Button2Label = "RECORD";

void button_press() // button press ISR
{
    flags.set(DELETE);
}
void onGyroDataReady() // Gyrscope data ready ISR
{
    flags.set(READY);
}

const int message_x = 5;
const int message_y = 30;
const char *message = "ECEGY 6483: Group 27";
const int text_x = 5;
const int text_y = 270;
const char *text_0 = "NO GUESTURE RECORDED";
const char *text_1 = "LOCKED";

// Check if the touch is inside a button
// @param touch_x: the x coordinate of the touch
// @param touch_y: the y coordinate of the touch
// @param button_x: the x coordinate of the button
// @param button_y: the y coordinate of the button
// @param button_width: the width of the button
// @param button_height: the height of the button
// @return: true if the touch is inside the button, false otherwise
bool detect_touch(int touch_x, int touch_y, int button_x, int button_y, int button_width, int button_height) {
    return (touch_x >= button_x && touch_x <= button_x + button_width &&
            touch_y >= button_y && touch_y <= button_y + button_height);
}

// Display the buttons on the LCD
void display()
{
    // Add your touch screen initialization and handling code here
    TS_StateTypeDef ts_state;

    if (ts.Init(lcd.GetXSize(), lcd.GetYSize()) != TS_OK)
    {
        printf("Failed to initialize the touch screen!\r\n");
        return;
    }

    // initialize a string display_buffer that can be draw on the LCD to dispaly the status
    char display_buffer[50];

    while (1)
    {
        ts.GetState(&ts_state);
        if (ts_state.TouchDetected)
        {
            int touch_x = ts_state.X;
            int touch_y = ts_state.Y - 50;
            printf("Touch values: x = %d, y = %d\n", touch_x, touch_y);
            // Check if the touch is inside record button
            if (detect_touch(touch_x, touch_y, Button2X, Button2Y, Button1Width, Button1Height))
            {
                sprintf(display_buffer, "Recording");
                lcd.SetTextColor(LCD_COLOR_BLACK);                  
                lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
                lcd.SetTextColor(LCD_COLOR_RED);                   
                lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
                ThisThread::sleep_for(1s);
                flags.set(IS_GUESTURE_SAVED);
            }

            // Check if the touch is inside unlock button
            if (detect_touch(touch_x, touch_y, ButtonX, ButtonY, Button2Width, Button2Height))
            {
                sprintf(display_buffer, "Wait");
                lcd.SetTextColor(LCD_COLOR_BLACK);                  
                lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
                lcd.SetTextColor(LCD_COLOR_RED);                   
                lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
                ThisThread::sleep_for(1s);
                flags.set(IS_OPEN);
            }
        }
        ThisThread::sleep_for(10ms);
    }
}

// Save the gesture key to memory
// @param gesture: the gesture key to be saved
// @param address: the address of the flash to save the gesture key
// @return: true if the gesture key is saved successfully, false otherwise
bool store_gyro_Data(vector<array<float, 3>> &gesture, uint32_t address)
{
    FlashIAP flash;
    flash.init();

    // Calculate the total size of the data to be stored in bytes
    uint32_t data_size = gesture.size() * sizeof(array<float, 3>);

    // Erase the flash sector
    flash.erase(address, data_size);

    // Write the data to flash
    int write_result = flash.program(gesture.data(), address, data_size);

    flash.deinit();

    return write_result == 0;
}

// Read the gesture key from memory
// @param address: the address of the flash to read the gesture key
// @param length: the length of the gesture key to be read
// @return the gesture key read from the flash
vector<array<float, 3>> read_gyro_data(uint32_t address, size_t length)
{
    vector<array<float, 3>> gesture_key(length);

    FlashIAP flash;
    flash.init();

    // Read the data from flash
    flash.read(gesture_key.data(), address, length * sizeof(array<float, 3>));

    flash.deinit();

    return gesture_key;
}

// create a button on the LCD
void create_button(int x, int y, int width, int height, const char *label)
{
    lcd.SetTextColor(LCD_COLOR_RED);
    lcd.FillRect(x, y, width, height);
    lcd.DisplayStringAt(x + width / 2 - strlen(label) * 19, y + height / 2 - 8, (uint8_t *)label, CENTER_MODE);
}

// Process the data from the gyroscope
// data: the data from the gyroscope
void process_gyro_data(vector<array<float, 3>> &data)
{
    float threshold = 0.00001;
    auto ptr = data.begin();
    // find the first element where data from any
    // one direction is larger than the threshold
    while (abs((*ptr)[0]) <= threshold && abs((*ptr)[1]) <= threshold && abs((*ptr)[2]) <= threshold)
    {
        ptr++;
    }
    if (ptr == data.end())
        return;      // all data less than threshold
    auto lptr = ptr; // record the left bound
    // start searching from end to front
    ptr = data.end() - 1;
    while (abs((*ptr)[0]) <= threshold && abs((*ptr)[1]) <= threshold && abs((*ptr)[2]) <= threshold)
    {
        ptr--;
    }
    auto rptr = ptr; // record the right bound
    // start moving elements to the front
    auto replace_ptr = data.begin();
    for (; replace_ptr != lptr && lptr <= rptr; replace_ptr++, lptr++)
    {
        *replace_ptr = *lptr;
    }
    // trim the end
    if (lptr > rptr)
    {
        data.erase(replace_ptr, data.end());
    }
    else
    {
        data.erase(rptr + 1, data.end());
    }
}


// Calculate the correlation between two vectors
// a: the first vector
// b: the second vector
// return: the correlation between the two vectors
float correlation(const vector<float> &a, const vector<float> &b)
{
    // check if the size of the two vectors are the same
    if (a.size() != b.size())
    {
        err = -1;
        return 0.0f;
    }

    float sum_a = 0, sum_b = 0, sum_ab = 0, sq_sum_a = 0, sq_sum_b = 0;

//calculate running sum and running square of a and b vectors.
    for (size_t i = 0; i < a.size(); ++i)
    {
        sum_a += a[i];
        sum_b += b[i];
        sum_ab += a[i] * b[i];
        sq_sum_a += a[i] * a[i];
        sq_sum_b += b[i] * b[i];
    }

    size_t n = a.size(); // number of elements

    float numerator = n * sum_ab - sum_a * sum_b; // Covariance for numerator
    
    float denominator = sqrt((n * sq_sum_a - sum_a * sum_a) * (n * sq_sum_b - sum_b * sum_b)); // Standard deviation for denominator

    return numerator / denominator;
}

// Calculate the similarity between two vectors for v1 and v2
array<float, 3> calculate_similarity(vector<array<float, 3>>& vec1, vector<array<float, 3>>& vec2) {
    array<float, 3> result;

    // Calculate the correlation for each coordinate
    for (int i = 0; i < 3; i++) {
        vector<float> a;
        vector<float> b;

        // Populate 'a' and 'b' with the ith coordinates of vec1 and vec2
        for (const auto& arr : vec1) {
            a.push_back(arr[i]);
        }
        for (const auto& arr : vec2) {
            b.push_back(arr[i]);
        }

        // Resize 'a' to match the size of 'b', if necessary
        if (a.size() > b.size()) {
            a.resize(b.size(), 0);
        } else if (b.size() > a.size()) {
            b.resize(a.size(), 0);
        }

        // Calculate the correlation and store the result
        result[i] = correlation(a, b);
    }

    return result;
}


// record the gesture key
void record_gyro()
{

    // intiiate the gyroscope
    Gyroscope_Init_Parameters params;
    params.conf1 = ODR_200_CUTOFF_50;
    params.conf3 = INT2_DRDY;
    params.conf4 = FULL_SCALE_500;

    // Set up gyroscope's raw data
    Gyroscope_RawData raw_data;

    // initialize LCD display buffer
    char display_buffer[50];

    // check the signal and set the flag
    // for the first sample.
    if (!(flags.get() & READY) && (gyro_int2.read() == 1))
    {
        flags.set(READY);
    }

    while (1)
    {
        vector<array<float, 3>> temp_key; 

        auto flag_check = flags.wait_any(IS_GUESTURE_SAVED | IS_OPEN | DELETE);

        if (flag_check & DELETE)
        {
            // Erase the gesture key
            sprintf(display_buffer, "Erasing");
            lcd.SetTextColor(LCD_COLOR_BLACK);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
            lcd.SetTextColor(LCD_COLOR_RED);                  
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
            gesture_key.clear();
            
            // Erase the unlocking record
            lcd.SetTextColor(LCD_COLOR_BLACK);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
            lcd.SetTextColor(LCD_COLOR_RED);                   
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
            unlocking_record.clear();

            // Reset the LED and print the message
            green_led = 1;
            red_led = 0;
            sprintf(display_buffer, "All Erasing finish.");
            lcd.SetTextColor(LCD_COLOR_BLACK);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
            lcd.SetTextColor(LCD_COLOR_RED);                  
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
        }

        if (flag_check & (IS_GUESTURE_SAVED | IS_OPEN))
        {
            sprintf(display_buffer, "Wait");
            lcd.SetTextColor(LCD_COLOR_BLACK);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
            lcd.SetTextColor(LCD_COLOR_RED);                   
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);

            ThisThread::sleep_for(1s);

            lcd.SetTextColor(LCD_COLOR_BLACK);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
            lcd.SetTextColor(LCD_COLOR_RED);                   
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);

            // Initiate gyroscope
            InitiateGyroscope(&params, &raw_data);

            // start recording gesture
            lcd.SetTextColor(LCD_COLOR_BLACK);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE);
            lcd.SetTextColor(LCD_COLOR_RED);                   
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
            ThisThread::sleep_for(1s);
            
            lcd.SetTextColor(LCD_COLOR_BLACK);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
            lcd.SetTextColor(LCD_COLOR_RED);                   
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
            ThisThread::sleep_for(1s);

            sprintf(display_buffer, "Recording in 1sec ");

            lcd.SetTextColor(LCD_COLOR_BLACK);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
            lcd.SetTextColor(LCD_COLOR_RED);                  
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
            ThisThread::sleep_for(1s);

            sprintf(display_buffer, "Recording");
            lcd.SetTextColor(LCD_COLOR_BLACK);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
            lcd.SetTextColor(LCD_COLOR_RED);                   
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
            
            // gyro data recording loop
            timer.start();
            while (timer.elapsed_time() < 5s)
            {
                // Wait for the gyroscope data to be ready
                flags.wait_all(READY);
                // Read the data from the gyroscope
                GetCalibratedRawData();
                // Add the converted data to the gesture_key vector
                temp_key.push_back({ConvertToDPS(raw_data.x_raw), ConvertToDPS(raw_data.y_raw), ConvertToDPS(raw_data.z_raw)});
                ThisThread::sleep_for(50ms); // 20Hz
            }
            timer.stop();  // Stop timer
            timer.reset(); // Reset timer

            // trim zeros
            process_gyro_data(temp_key);

            sprintf(display_buffer, "Done");
            lcd.SetTextColor(LCD_COLOR_BLACK);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
            lcd.SetTextColor(LCD_COLOR_RED);                   
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
        }

        // check the flag see if it is recording or unlocking
        if (flag_check & IS_GUESTURE_SAVED)
        {
            if (gesture_key.empty())
            {
                lcd.SetTextColor(LCD_COLOR_BLACK);                  
                lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
                lcd.SetTextColor(LCD_COLOR_RED);                   
                lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);

                // save the key
                gesture_key = temp_key;

                // clear temp_key
                temp_key.clear();

                // toggle led
                red_led = 1;
                green_led = 0;

                sprintf(display_buffer, "Guesture saved");
                lcd.SetTextColor(LCD_COLOR_BLACK);                  
                lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
                lcd.SetTextColor(LCD_COLOR_RED);                   
                lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
            }
            else
            {
                lcd.SetTextColor(LCD_COLOR_BLACK);                  
                lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
                lcd.SetTextColor(LCD_COLOR_RED);                   
                lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);

                ThisThread::sleep_for(1s);
                
                // clear old key
                gesture_key.clear();

                // save new key
                gesture_key = temp_key;

                sprintf(display_buffer, "New guesture saved.");
                lcd.SetTextColor(LCD_COLOR_BLACK);                  
                lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
                lcd.SetTextColor(LCD_COLOR_RED);                   
                lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);

                // clear temp_key
                temp_key.clear();

                // toggle led
                red_led = 1;
                green_led = 0;
            }
        }
        else if (flag_check & IS_OPEN)
        {
            flags.clear(IS_OPEN);
            sprintf(display_buffer, "Wait");
            lcd.SetTextColor(LCD_COLOR_BLACK);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
            lcd.SetTextColor(LCD_COLOR_RED);                   
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);

            unlocking_record = temp_key; // save the unlocking record
            temp_key.clear(); // clear temp_key

            // check if the gesture key is empty
            if (gesture_key.empty())
            {
                sprintf(display_buffer, "NO GEUSTURE SAVED.");
                lcd.SetTextColor(LCD_COLOR_BLACK);                  
                lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
                lcd.SetTextColor(LCD_COLOR_RED);                   
                lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);

                unlocking_record.clear(); // clear unlocking record

                // toggle led
                green_led = 1;
                red_led = 0;
            }
            else // compare the unlocking record with the gesture key
            {
                
                int unlock = 0; // counter for the coordinates that are above threshold

                array<float, 3> correlationResult = calculate_similarity(gesture_key, unlocking_record); // calculate correlation

                if (err != 0)
                {
                    printf("Error in correlation: different sizes!\n");
                }
                else
                {
                    printf("Correlation values: x = %f, y = %f, z = %f\n", correlationResult[0], correlationResult[1], correlationResult[2]);
                    
                    // check for thresholding
                    for (size_t i = 0; i < correlationResult.size(); i++)
                    {
                        if (correlationResult[i] > EPSILON)
                        {
                            unlock++;
                        }
                    }
                }

                if (unlock==3) // need to find a better threshold
                {
                    sprintf(display_buffer, "OPENED");
                    lcd.SetTextColor(LCD_COLOR_BLACK);                  
                    lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
                    lcd.SetTextColor(LCD_COLOR_RED);                   
                    lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
                    
                    // toggle led
                    green_led = 1;
                    red_led = 0;

                    // clear unlocking record
                    unlocking_record.clear();
                    unlock = 0;
                }
                else
                {
                    sprintf(display_buffer, "LOCKED");
                    lcd.SetTextColor(LCD_COLOR_BLACK);                  
                    lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
                    lcd.SetTextColor(LCD_COLOR_RED);                   
                    lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);

                    // toggle led
                    green_led = 0;
                    red_led = 1;

                    // clear unlocking record
                    unlocking_record.clear();
                    unlock = 0;
                }
            }
        }
        ThisThread::sleep_for(100ms);
    }
}


int main()
{
    lcd.Clear(LCD_COLOR_BLACK);
    // Draw buttons on the screen
create_button(ButtonX, ButtonY, Button1Width, Button1Height, Button1Label);
create_button(Button2X, Button2Y, Button2Width, Button2Height, Button2Label);

// Display a message on the LCD screen
lcd.DisplayStringAt(message_x, message_y, (uint8_t *)message, CENTER_MODE);

// Initialize interrupts for user button and gyroscope
ResetButton.rise(&button_press);
gyro_int2.rise(&onGyroDataReady);

// Check if the gesture key is empty
if (gesture_key.empty())
{
    // Turn on the green LED and display a message on the LCD screen
    red_led = 0;
    green_led = 1;
    lcd.DisplayStringAt(text_x, text_y, (uint8_t *)text_0, CENTER_MODE);
}
else
{
    // Turn on the red LED and display a message on the LCD screen
    red_led = 1;
    green_led = 0;
    lcd.DisplayStringAt(text_x, text_y, (uint8_t *)text_1, CENTER_MODE);
}

// Initialize gyroscope
Thread key_saving;
key_saving.start(callback(record_gyro));

// Initialize touch screen
Thread touch_thread;
touch_thread.start(callback(display));

// Run the main loop
while (1)
{
    // Sleep for 100ms before repeating the loop
    ThisThread::sleep_for(100ms);
}
}