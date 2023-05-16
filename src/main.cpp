#include <mbed.h>
#include <vector>
#include "drivers/LCD_DISCO_F429ZI.h"
#include "drivers/TS_DISCO_F429ZI.h"
#include <array>
#include <limits>
#include <cmath>
#include <math.h>
#include "gyro.h"

vector<array<float, 3>> gesture_key; // the gesture key
vector<array<float, 3>> unlocking_record; // the unlocking record

const int button1_x = 10;
const int button1_y = 100;
const int button1_width = 100;
const int button1_height = 50;
const char *button1_label = "UNLOCK";

int err = 0; // for error checking

// error tolerance for gesture recognition
#define EPSILON 0.3000000119F

//Text size
#define TEXT_SIZE 24

// Events
#define IS_GUESTURE_SAVED 1
#define IS_OPEN 2
#define DELETE 4
#define READY 8

InterruptIn gyro_int2(PA_2, PullDown);
InterruptIn user_button(USER_BUTTON, PullDown);

DigitalOut green_led(LED1);
DigitalOut red_led(LED2);

// LCD object
LCD_DISCO_F429ZI lcd; 

// Touch screen object
TS_DISCO_F429ZI ts; 

EventFlags flags; // Event flags

Timer timer; // Timer

const int button2_x = 121;
const int button2_y = 100;
const int button2_width = 100;
const int button2_height = 50;
const char *button2_label = "RECORD";

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


// Detect if the touch is inside a button
// touch_x: the x coordinate of the touch
// touch_y: the y coordinate of the touch
// button_x: the x coordinate of the button
// button_y: the y coordinate of the button
// button_width: the width of the button
// button_height: the height of the button
// return: true if the touch is inside the button, false otherwise
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
            if (detect_touch(touch_x, touch_y, button2_x, button2_y, button1_width, button1_height))
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
            if (detect_touch(touch_x, touch_y, button1_x, button1_y, button2_width, button2_height))
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
// gesture: the gesture key to be saved
// address: the address of the flash to save the gesture key
// return: true if the gesture key is saved successfully, false otherwise
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
// address: the address of the flash to read the gesture key
// length: the length of the gesture key to be read
// return: the gesture key read from the flash
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

    for (size_t i = 0; i < a.size(); ++i)
    {
        sum_a += a[i];
        sum_b += b[i];
        sum_ab += a[i] * b[i];
        sq_sum_a += a[i] * a[i];
        sq_sum_b += b[i] * b[i];
    }

    size_t n = a.size(); // number of elements

    float numerator = n * sum_ab - sum_a * sum_b; // Covariance
    
    float denominator = sqrt((n * sq_sum_a - sum_a * sum_a) * (n * sq_sum_b - sum_b * sum_b)); // Standard deviation

    return numerator / denominator;
}

// Calculate the similarity between two vectors
// vec1: the first vector
// vec2: the second vector
// return: the similarity between the two vectors
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
                    printf("Error calculating correlation: vectors have different sizes\n");
                }
                else
                {
                    printf("Correlation values: x = %f, y = %f, z = %f\n", correlationResult[0], correlationResult[1], correlationResult[2]);
                    
                    // iterate through correlationResult to check if all values are above threshold
                    for (size_t i = 0; i < correlationResult.size(); i++)
                    {
                        if (correlationResult[i] > EPSILON)
                        {
                            unlock++;
                        }
                    }
                }

                if (unlock==3) // TODO: need to find a better threshold
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
    // Draw buttons
    create_button(button1_x, button1_y, button1_width, button1_height, button1_label);
    create_button(button2_x, button2_y, button2_width, button2_height, button2_label);

    lcd.DisplayStringAt(message_x, message_y, (uint8_t *)message, CENTER_MODE);

    // initialize all interrupts
    user_button.rise(&button_press);
    gyro_int2.rise(&onGyroDataReady);

    if (gesture_key.empty())
    {
        red_led = 0;
        green_led = 1;
        lcd.DisplayStringAt(text_x, text_y, (uint8_t *)text_0, CENTER_MODE);
    }
    else
    {
        red_led = 1;
        green_led = 0;
        lcd.DisplayStringAt(text_x, text_y, (uint8_t *)text_1, CENTER_MODE);
    }

    // Gyroscope initialization
    Thread key_saving;
    key_saving.start(callback(record_gyro));

    // Touch screen initialization
    Thread touch_thread;
    touch_thread.start(callback(display));

    // Run the main loop
    while (1)
    {
        ThisThread::sleep_for(100ms);
    }
}
