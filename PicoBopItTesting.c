#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "pico/multicore.h"
#include "hardware/irq.h"  // interrupts
#include "hardware/sync.h" // wait for interrupt 
#include "hardware/clocks.h"
#include "pico/rand.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include <math.h>

#include "haw/MPU6050.h"

//#include "ring.h"
//#include "sample.h"
//#include "vibes.h"
#include "startupsound.h"
//#include "startupsound22k16bit.h"
#include "barrelroll.h"
#include "scream.h"
#include "pokesuccess.h"
#include "simonclick1.h"
#include "simonoutput1.h"
#include "shake.h"

// CONFIG !!!!!!!!!!!!!!!!!!!!!!!
const int introDurationMs = 4000;
const int simonInputVisualDurationMs = 250;
// CONFIG !!!!!!!!!!!!!!!!!!!!!!!
 
const int AUDIO_PIN = 15;  // you can change this to whatever you like

const int redOutputPin = 19;
const int greenOutputPin = 17;
const int blueOutputPin = 18;
const int yellowOutputPin = 16;

const int redInputPin = 22;
const int greenInputPin = 26;
const int blueInputPin = 21;
const int yellowInputPin = 20;
const int gyroSda = 2;
const int gyroScl = 3;
const int soundModuleInputPin = 12;
bool redInputPressed = false;
bool greenInputPressed = false;
bool blueInputPressed = false;
bool yellowInputPressed = false;
bool anyInputPressed = false;

uint32_t redInputPressedMs = 0;
uint32_t greenInputPressedMs = 0;
uint32_t blueInputPressedMs = 0;
uint32_t yellowInputPressedMs = 0;

int gameplayState = 0; // 0 = intro, 1 = start menu, 2 = ingame
int difficultyLevel = 0;

bool simonModuleActive = false;
bool gyroModuleActive = false;
bool shakeModuleActive = false;
bool soundModuleActive = false;

uint32_t simonModuleActivatedMs = 0;
uint32_t gyroModuleActivatedMs = 0;
uint32_t shakeModuleActivatedMs = 0;
uint32_t soundModuleActivatedMs = 0;

uint32_t primaryModuleWonMs = 0;

int soundModuleCounter = 0;
int totalAccelerationCounter = 0;

// int wav_position = 0;
// const uint8_t *soundSamplesPointer = startUpSound;
// int currentSampleLength = startUpSoundLength;

// 0 = startupsound, 1 = scream, 2 = pokesuccess, 3 = barrelroll, 4 = simonclick1, 5 = simonoutput1, 6 = shake
const int numberOfSounds = 7;
int wav_positions[] = {0, 0, 0, 0, 0, 0, 0};
bool soundsPlaying[] = {true, false, false, false, false, false, false};
const uint8_t *soundSamplesPointers[] = {startUpSound, screamSound, pokeSuccessSound, barrelRollSound, simonClickSound1, simonOutputSound1, shakeSound};
const int soundSampleLengths[] = {startUpSoundLength, screamSoundLength, pokeSuccessSoundLength, barrelRollSoundLength, simonClickSound1Length, simonOutputSound1Length, shakeSoundLength};

uint32_t serialYieldCompensationMs = 0;
uint32_t msSinceGameEnd = 0;

const long long int thirtyTwoBitMagnitude = 4294967296;

// GLOBAL FUNCTION DECLARATIONS
void simon_module_cleanup();
bool is_simon_module_input_active();
mpu6050_t mpu6050;
// GLOBAL FUNCTION DECLARATIONS

#define FLAG_VALUE 123
void core1_entry() {
    sleep_ms(500);

    multicore_fifo_push_blocking(FLAG_VALUE);

    uint32_t g = multicore_fifo_pop_blocking();

    if (g != FLAG_VALUE)
        printf("Hmm, that's not right on core 1!\n");
    else
        printf("It's all gone well on core 1!\n");

    while (1)
        tight_loop_contents();
}

void serial_init() {
    stdio_usb_init();
    while (!stdio_usb_connected()) {}

    sleep_ms(1000);

    printf("Hello, multicore!\n");

    /// \tag::setup_multicore[]

    multicore_launch_core1(core1_entry);

    // Wait for it to start up

    uint32_t g = multicore_fifo_pop_blocking();

    if (g != FLAG_VALUE)
        printf("Hmm, that's not right on core 0!\n");
    else {
        multicore_fifo_push_blocking(FLAG_VALUE);
        printf("It's all gone well on core 0!\n");
    }
    /// \end::setup_multicore[]
}

static int addr = 0x68; // MPU6050 address
static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_blocking(((&i2c1_inst)), addr, buf, 2, false);
    sleep_ms(100); // Allow device to reset and stabilize

    // Clear sleep mode (0x6B register, 0x00 value)
    buf[1] = 0x00;  // Clear sleep mode by writing 0x00 to the 0x6B register
    i2c_write_blocking(((&i2c1_inst)), addr, buf, 2, false); 
    sleep_ms(10); // Allow stabilization after waking up
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(((&i2c1_inst)), addr, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(((&i2c1_inst)), addr, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(((&i2c1_inst)), addr, &val, 1, true);
    i2c_read_blocking(((&i2c1_inst)), addr, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(((&i2c1_inst)), addr, &val, 1, true);
    i2c_read_blocking(((&i2c1_inst)), addr, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

// void play_sound(const uint8_t newSoundSamples[], int newSoundSampleLength, bool overrideCurrentSound) {
//     if (!overrideCurrentSound && wav_position < currentSampleLength) {return;}

//     soundSamplesPointer = newSoundSamples;
//     currentSampleLength = newSoundSampleLength;
//     wav_position = 0;
// }

// 0 = startupsound, 1 = scream, 2 = pokesuccess, 3 = barrelroll, 4 = simonclick1, 5 = simonoutput1, 6 = shake
void play_sound(int soundIndex) {
    soundsPlaying[soundIndex] = true;
    wav_positions[soundIndex] = 0;
}

void disable_simon_outputs() {
    gpio_put(redOutputPin, false);
    gpio_put(greenOutputPin, false);
    gpio_put(blueOutputPin, false);
    gpio_put(yellowOutputPin, false);
}

void lose_game() {
    gameplayState = 1;
    soundModuleCounter = 0;
    totalAccelerationCounter = 0;
    msSinceGameEnd = to_ms_since_boot(get_absolute_time());

    simonModuleActive = false;
    gyroModuleActive = false;
    soundModuleActive = false;

    simon_module_cleanup();
}

uint32_t blink_led_ms = 0;
void blink_led_loop(uint32_t msSinceBootNew) {
    if (msSinceBootNew - blink_led_ms > 1000) {
        blink_led_ms = msSinceBootNew;
        // printf("gameplayState: %f", (float) gameplayState);
        // printf("simonModuleActive: %f", (float) simonModuleActive);
        // printf("gyroModuleActive: %f", (float) gyroModuleActive);
        // printf("soundModuleActive: %f", (float) soundModuleActive);
        //printf("difficultyLevel: %i\n", difficultyLevel);
        gpio_put(PICO_DEFAULT_LED_PIN, !gpio_get(PICO_DEFAULT_LED_PIN));
    }
}

uint32_t intro_loop_ms = 0;
void intro_loop(uint32_t msSinceBootNew) {
    if (gameplayState != 0) {return;}

    if (msSinceBootNew - serialYieldCompensationMs > introDurationMs) {
        disable_simon_outputs();

        gameplayState = 1;
        return;
    }

    if (msSinceBootNew - intro_loop_ms > 250) {
        intro_loop_ms = msSinceBootNew;

        if (gpio_get(redOutputPin)) {
            gpio_put(blueOutputPin, true);
            gpio_put(redOutputPin, false);
        } else if (gpio_get(blueOutputPin)) {
            gpio_put(yellowOutputPin, true);
            gpio_put(blueOutputPin, false);
        } else if (gpio_get(yellowOutputPin)) {
            gpio_put(greenOutputPin, true);
            gpio_put(yellowOutputPin, false);
        } else if (gpio_get(greenOutputPin)) {
            gpio_put(redOutputPin, true);
            gpio_put(greenOutputPin, false);
        } else {
            gpio_put(redOutputPin, true);
        }
    }
}

void push(int array[], int *size, int value) {
    array[*size] = value;
    (*size)++;
}

int get_random_simon_output() {
    uint32_t random = get_rand_32();

    if (random < thirtyTwoBitMagnitude / 4) {
        return redOutputPin;
    } else if (random < thirtyTwoBitMagnitude / 2) {
        return greenOutputPin;
    } else if (random < (double) thirtyTwoBitMagnitude / 1.3333) {
        return blueOutputPin;
    } else {
        return yellowOutputPin;
    }
}

bool oldSimonModuleActive = false;
int simonColors[100] = {};
int simonColorsSize = 0;
int simonColorsInputted = 0;
int colorsPerIteration = 1;
uint32_t msSinceColorOutput = 0;
void simon_module_loop(uint32_t msSinceBootNew) {
    if (!simonModuleActive && !oldSimonModuleActive) {return;}

    if (!simonModuleActive && oldSimonModuleActive && gameplayState == 2) { // handle cleanup + module reward here
        primaryModuleWonMs = msSinceBootNew;
        oldSimonModuleActive = simonModuleActive;
        simon_module_cleanup();
        play_sound(2);
        return;
    }

    int maxColors = difficultyLevel < 30 ? 4 : 5;
    colorsPerIteration = (difficultyLevel * 0.3) + 1;
    colorsPerIteration = colorsPerIteration > maxColors ? maxColors : colorsPerIteration;
    //int iterations = 1;
    uint32_t delayBetweenColors = 900 - (difficultyLevel * 15);
    uint32_t allowedInputTime = (colorsPerIteration * 1000) + 4000;

    uint32_t timeToCompleteMs = 0;
    //for (int i = 0; i < iterations; i++) {
        timeToCompleteMs += (delayBetweenColors * colorsPerIteration) + allowedInputTime;
    //}

    if (msSinceBootNew - simonModuleActivatedMs > timeToCompleteMs) {
        lose_game();
        return;
    }

    if (msSinceBootNew - msSinceColorOutput > delayBetweenColors - 100 && simonColorsSize < colorsPerIteration) {
        gpio_put(simonColors[simonColorsSize - 1], false);
    }

    if (msSinceBootNew - msSinceColorOutput > delayBetweenColors && simonColorsSize < colorsPerIteration) {
        msSinceColorOutput = msSinceBootNew;

        int colorOutput = get_random_simon_output();
        //gpio_put(simonColors[simonColorsSize - 1], false);
        gpio_put(colorOutput, true);
        push(simonColors, &simonColorsSize, colorOutput);
        play_sound(5);
    }

    
    oldSimonModuleActive = simonModuleActive;
}
void simon_input_visual(uint32_t msSinceBootNew, uint32_t inputPressedMs, uint outputPin) {
    if (!is_simon_module_input_active(msSinceBootNew)) {return;}

    if (msSinceBootNew - inputPressedMs > simonInputVisualDurationMs) {
        gpio_put(outputPin, false);
    } else {
        gpio_put(outputPin, true);
    }
}

void simon_module_cleanup() {
    disable_simon_outputs();
    
    for (int i = 0; i < simonColorsSize; i++) {
        simonColors[i] = 0; // Set each element to 0
    }
    simonColorsSize = 0;
    simonColorsInputted = 0;
    oldSimonModuleActive = false;
}

bool is_simon_module_input_active(uint32_t msSinceBootNew) {
    return simonColorsSize == colorsPerIteration && simonModuleActive && msSinceBootNew - msSinceColorOutput > 500;
}

void on_simon_button_press(int outputPin, uint32_t msSinceBootNew) {
    if (!is_simon_module_input_active(msSinceBootNew)) {return;}

    int currentSimonColor = simonColors[simonColorsInputted];
    if (currentSimonColor != outputPin) {
        lose_game();
        return;
    }

    simonColorsInputted++;
    play_sound(4);

    if (simonColorsInputted >= simonColorsSize) {
        simonModuleActive = false;
    }
}

uint32_t msSinceGyroCheck = 0;
int16_t initialAcceleration[3], initialGyro[3], initialTemp;
void gyro_module_loop(uint32_t msSinceBootNew) {
    if (!gyroModuleActive) {return;}

    uint32_t gyroCheckDelay = 50;
    uint32_t timeToCompleteMs = 4000;
    int16_t yTiltGoal = 10000;

    if (msSinceBootNew - gyroModuleActivatedMs > timeToCompleteMs) {
        lose_game();
        return;
    }

    if (msSinceBootNew - msSinceGyroCheck > gyroCheckDelay) {
        msSinceGyroCheck = msSinceBootNew;

        int16_t acceleration[3], gyro[3], temp;

        mpu6050_read_raw(gyro, acceleration, &temp); // -16k through 16k
        int difference = abs(gyro[1] - initialGyro[1]);

        //printf("Acc. X = %d, Y = %d, Z = %d\n", acceleration[0], acceleration[1], acceleration[2]);
        //printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
        // Temperature is simple so use the datasheet calculation to get deg C.
        // Note this is chip temperature.
        //printf("Temp. = %f\n", (temp / 340.0) + 36.53);

        bool wins = difference < 0 ? difference < -yTiltGoal : difference > yTiltGoal;
        //printf("%i\n", difference);
        if (difference > yTiltGoal) {
            gyroModuleActive = false;
            play_sound(2);
        }
    }
}

uint32_t msSinceShakeCheck = 0;
void shake_module_loop(uint32_t msSinceBootNew) {
    if (!shakeModuleActive) {return;}

    uint32_t shakeCheckDelay = 50;
    uint32_t timeToCompleteMs = 40000;
    int totalAccelerationNeeded = 300000 + (difficultyLevel * 15000);

    if (msSinceBootNew - shakeModuleActivatedMs > timeToCompleteMs) {
        lose_game();
        return;
    }

    if (msSinceBootNew - msSinceShakeCheck > shakeCheckDelay) {
        msSinceShakeCheck = msSinceBootNew;

        int16_t acceleration[3], gyro[3], temp;

        mpu6050_read_raw(gyro, acceleration, &temp); // -16k through 16k
        int totalAcceleration = abs(acceleration[0]) + abs(acceleration[1]) + abs(acceleration[2]);
        totalAccelerationCounter += totalAcceleration;

        //printf("totalAccelerationCounter: %i\n", totalAccelerationCounter);
        if (totalAccelerationCounter > totalAccelerationNeeded) {
            shakeModuleActive = false;
            totalAccelerationCounter = 0;
            play_sound(2);
        }
    }
}

uint32_t msSinceSoundCheck = 0;
void sound_module_loop(uint32_t msSinceBootNew) {
    if (!soundModuleActive) {return;}

    uint32_t soundCheckDelay = 50;
    uint32_t soundsNeeded = (difficultyLevel * 0.1) + 3;
    uint32_t timeToCompleteMs = (soundsNeeded * soundCheckDelay) + 5000;

    if (msSinceBootNew - soundModuleActivatedMs > timeToCompleteMs) {
        lose_game();
        return;
    }

    if (msSinceBootNew - msSinceSoundCheck > soundCheckDelay) {
        msSinceSoundCheck = msSinceBootNew;

        bool soundInputState = gpio_get(soundModuleInputPin);
        soundModuleCounter += soundInputState;

        if (soundModuleCounter > soundsNeeded) { // Module is completed
            soundModuleCounter = 0;
            soundModuleActive = false;
            play_sound(2);
        }
    }
}

int oldGameplayState = 0;
uint32_t primaryModuleActivatedMs = 0;
uint32_t secondaryModuleActivatedMs = 0;
void gameplay_handler_loop(uint32_t msSinceBootNew) {
    if (gameplayState != 2) {return;}

    //int maxPrimaries = (difficultyLevel / 10) + 1;
    int maxPrimaries = 1; // only 1 primary currently
    int delayBetweenPrimariesMs = 4000.0f / (((float) difficultyLevel * 0.05f) + 1.0f);

    // int maxSecondaries = (difficultyLevel / 10) + 1;
    int maxSecondaries = 1;
    int delayBetweenSecondariesMs = 15000.0f / (((float) difficultyLevel * 0.05f) + 1.0f);

    int activePrimaries = simonModuleActive;
    int activeSecondaries = gyroModuleActive + soundModuleActive;

    if (activePrimaries < maxPrimaries && msSinceBootNew - primaryModuleWonMs > delayBetweenPrimariesMs) {
        primaryModuleActivatedMs = msSinceBootNew;
        difficultyLevel++;

        simonModuleActive = true;
        simonModuleActivatedMs = msSinceBootNew;
    }

    if (activeSecondaries < maxSecondaries && msSinceBootNew - secondaryModuleActivatedMs > delayBetweenSecondariesMs) {
        secondaryModuleActivatedMs = msSinceBootNew;
        uint32_t random = get_rand_32();

        if (random < thirtyTwoBitMagnitude / 3) {
            gyroModuleActivatedMs = msSinceBootNew;
            gyroModuleActive = true;
            mpu6050_read_raw(initialGyro, initialAcceleration, &initialTemp);
            play_sound(3);
        } else if (random < thirtyTwoBitMagnitude / 1.5) {
            soundModuleActivatedMs = msSinceBootNew;
            soundModuleActive = true;
            play_sound(1);
        } else {
            shakeModuleActivatedMs = msSinceBootNew;
            shakeModuleActive = true;
            //mpu6050_read_raw(initialGyro, initialAcceleration, &initialTemp);
            play_sound(6);
        }
    }

    oldGameplayState = gameplayState;
}

uint32_t main_loop_ms = 0;
void main_loop(uint32_t msSinceBootNew) {
    if (msSinceBootNew - main_loop_ms > 1) {
        main_loop_ms = msSinceBootNew;

        if (gameplayState == 0) {return;}

        if (gameplayState == 1) {
            gpio_put(redOutputPin, true);
            gpio_put(greenOutputPin, true);
            gpio_put(blueOutputPin, true);
            gpio_put(yellowOutputPin, true);
        } else if (gpio_get(redOutputPin) && gpio_get(greenOutputPin) && gpio_get(blueOutputPin) && gpio_get(yellowOutputPin)) {
            gpio_put(redOutputPin, false);
            gpio_put(greenOutputPin, false);
            gpio_put(blueOutputPin, false);
            gpio_put(yellowOutputPin, false);
        }

        bool newRedInputPressed = gpio_get(redInputPin);
        bool newGreenInputPressed = gpio_get(greenInputPin);
        bool newBlueInputPressed = gpio_get(blueInputPin);
        bool newYellowInputPressed = gpio_get(yellowInputPin);
        bool newAnyInputPressed = newRedInputPressed || newGreenInputPressed || newBlueInputPressed || newYellowInputPressed;
        //printf("newRedInputPressed: %f\n", (float) newRedInputPressed);

        if ((redInputPressed != newRedInputPressed) && newRedInputPressed) {
            redInputPressedMs = msSinceBootNew;
            on_simon_button_press(redOutputPin, msSinceBootNew);

            if (gameplayState == 1 && msSinceBootNew - msSinceGameEnd > 500) {
                gameplayState = 2;
                difficultyLevel = 1;
            }
        }

        if ((greenInputPressed != newGreenInputPressed) && newGreenInputPressed) {
            greenInputPressedMs = msSinceBootNew;
            on_simon_button_press(greenOutputPin, msSinceBootNew);

            if (gameplayState == 1 && msSinceBootNew - msSinceGameEnd > 500) {
                gameplayState = 2;
                difficultyLevel = 1;
            }
        }

        if ((blueInputPressed != newBlueInputPressed) && newBlueInputPressed) {
            blueInputPressedMs = msSinceBootNew;
            on_simon_button_press(blueOutputPin, msSinceBootNew);

            if (gameplayState == 1 && msSinceBootNew - msSinceGameEnd > 500) {
                gameplayState = 2;
                difficultyLevel = 15;
            }
        }

        if ((yellowInputPressed != newYellowInputPressed) && newYellowInputPressed) {
            yellowInputPressedMs = msSinceBootNew;
            on_simon_button_press(yellowOutputPin, msSinceBootNew);

            if (gameplayState == 1 && msSinceBootNew - msSinceGameEnd > 500) {
                gameplayState = 2;
                difficultyLevel = 1;
            }
        }

        if ((anyInputPressed != newAnyInputPressed) && newAnyInputPressed) {
            //printf("Simon button pressed!\n");

        }

        simon_input_visual(msSinceBootNew, redInputPressedMs, redOutputPin);
        simon_input_visual(msSinceBootNew, greenInputPressedMs, greenOutputPin);
        simon_input_visual(msSinceBootNew, blueInputPressedMs, blueOutputPin);
        simon_input_visual(msSinceBootNew, yellowInputPressedMs, yellowOutputPin);

        sound_module_loop(msSinceBootNew);
        gyro_module_loop(msSinceBootNew);
        simon_module_loop(msSinceBootNew);
        shake_module_loop(msSinceBootNew);
        gameplay_handler_loop(msSinceBootNew);

        redInputPressed = newRedInputPressed;
        greenInputPressed = newGreenInputPressed;
        blueInputPressed = newBlueInputPressed;
        yellowInputPressed = newYellowInputPressed;
        anyInputPressed = newAnyInputPressed;
    }
}

/*
 * PWM Interrupt Handler which outputs PWM level and advances the
 * current sample.
 * 
 * We repeat the same value for 8 cycles this means sample rate etc
 * adjust by factor of 8   (this is what bitshifting <<3 is doing)
 * 
 */
void pwm_interrupt_handler() {
    uint32_t msSinceBootNew = to_ms_since_boot(get_absolute_time());
    blink_led_loop(msSinceBootNew);
    main_loop(msSinceBootNew);
    intro_loop(msSinceBootNew);

    pwm_clear_irq(pwm_gpio_to_slice_num(AUDIO_PIN));
    int result = 0;
    int numberOfSoundsPlaying = 0;

    for (int i = 0; i < numberOfSounds; i++) {
        bool soundPlaying = soundsPlaying[i];
        if (!soundPlaying) {continue;}
        numberOfSoundsPlaying++;

        int wav_position = wav_positions[i];
        const uint8_t *soundSamplesPointer = soundSamplesPointers[i];
        int soundSampleLength = soundSampleLengths[i];

        if (wav_position < (soundSampleLength<<3) - 1) {
            // set pwm level 
            // allow the pwm value to repeat for 8 cycles this is >>3
            // uint8_t result = soundSamplesPointer[wav_position>>3];
            result += soundSamplesPointer[wav_position>>3];

            // uint8_t msByte = soundSamplesPointer[wav_position>>3]; // TODO: check if this is correct
            // uint8_t lsByte = soundSamplesPointer[(wav_position>>3) + 1];
            // //uint16_t resultUnsigned = (msByte << 8) | lsByte;
            // uint16_t resultUnsigned = ((uint16_t)msByte << 8) | lsByte;
            // int16_t resultSigned = *(int16_t*)&resultUnsigned; // samples are originally stored in signed 16 bit PCM
            // uint16_t result = resultSigned + 32768;

            //pwm_set_gpio_level(AUDIO_PIN, result);
            wav_positions[i] = wav_position + 1;
        } else {
            // reset to start
            //wav_position = 0;
            //pwm_set_gpio_level(AUDIO_PIN, 0);
            soundsPlaying[i] = false;
        }
    }

    if (numberOfSoundsPlaying > 0) {
        result = result / numberOfSoundsPlaying;
    }

    pwm_set_gpio_level(AUDIO_PIN, result);
}

void pwm_audio_init() {
    /* Overclocking for fun but then also so the system clock is a 
     * multiple of typical audio sampling rates.
     */
    set_sys_clock_khz(176000, true); 
    gpio_set_function(AUDIO_PIN, GPIO_FUNC_PWM);

    int audio_pin_slice = pwm_gpio_to_slice_num(AUDIO_PIN);

    // Setup PWM interrupt to fire when PWM cycle is complete
    pwm_clear_irq(audio_pin_slice);
    pwm_set_irq_enabled(audio_pin_slice, true);
    // set the handle function above
    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_interrupt_handler); 
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // Setup PWM for audio output
    pwm_config config = pwm_get_default_config();
    /* Base clock 176,000,000 Hz divide by wrap 250 then the clock divider further divides
     * to set the interrupt rate. 
     * 
     * 11 KHz is fine for speech. Phone lines generally sample at 8 KHz
     * 
     * 
     * So clkdiv should be as follows for given sample rate
     *  8.0f for 11 KHz
     *  4.0f for 22 KHz
     *  2.0f for 44 KHz etc
     */
    pwm_config_set_clkdiv(&config, 4.0f);
    pwm_config_set_wrap(&config, 250);
    pwm_init(audio_pin_slice, &config, true);

    pwm_set_gpio_level(AUDIO_PIN, 0);

    while(1) {
        __wfi(); // Wait for Interrupt
    }
}

void simon_module_init() {
    gpio_init(redOutputPin);
    gpio_set_dir(redOutputPin, GPIO_OUT);
    
    gpio_init(greenOutputPin);
    gpio_set_dir(greenOutputPin, GPIO_OUT);

    gpio_init(blueOutputPin);
    gpio_set_dir(blueOutputPin, GPIO_OUT);

    gpio_init(yellowOutputPin);
    gpio_set_dir(yellowOutputPin, GPIO_OUT);

    disable_simon_outputs();

    gpio_init(redInputPin);
    gpio_pull_down(redInputPin);

    gpio_init(greenInputPin);
    gpio_pull_down(greenInputPin);

    gpio_init(blueInputPin);
    gpio_pull_down(blueInputPin);

    gpio_init(yellowInputPin);
    gpio_pull_down(yellowInputPin);
}

void gyro_module_init() {
    i2c_init(((&i2c1_inst)), 400 * 1000);
    gpio_set_function(gyroSda, GPIO_FUNC_I2C);
    gpio_set_function(gyroScl, GPIO_FUNC_I2C);
    gpio_pull_up(gyroSda);
    gpio_pull_up(gyroScl);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(gyroSda, gyroScl, GPIO_FUNC_I2C));

    mpu6050_reset();
}

void sound_module_init() {
    gpio_init(soundModuleInputPin);
    gpio_pull_down(soundModuleInputPin);
}

// we start the sound processing
void main() {
  stdio_init_all();

  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  gpio_put(PICO_DEFAULT_LED_PIN, true);

  //serial_init();
  serialYieldCompensationMs = to_ms_since_boot(get_absolute_time());
  //printf("MAIN 1\n");
  gyro_module_init();
  sound_module_init();
  simon_module_init();
  pwm_audio_init();
  //printf("MAIN 2\n");
//   while (true) {
//     //printf("I'm alive!!\n");
//     gpio_put(PICO_DEFAULT_LED_PIN, true);
//     sleep_ms(500);
//     gpio_put(PICO_DEFAULT_LED_PIN, false);
//     sleep_ms(500);
//   }
}