// By default the teensy audio library has ~3ms of added latency from using 128-sample
// buffers/transactions. You can reduce this by editing AudioStream.h in the platform headers (see
// AUDIO_BLOCK_SAMPLES). Know that using a define here will not work, because it won't affect the
// teensy audio library's .c files. Warning: using a buffer size that isn't 128 will disable USB
// audio support.

#include <Arduino.h>
#include <Audio.h>
#include <Wire.h>
#include <EEPROM.h>

bool clipping = false;
int32_t clipping_time = 0;  // milliseconds
int32_t max_seen = 0;
float rms = 0.0;

void led_on() { digitalWrite(13, HIGH); }
void led_off() { digitalWrite(13, LOW); }

int clamp(int a, int min, int max) { return a < min ? min : (a > max ? max : a); }

float unlerp(float a, float b, float t) { return (t - a) / (b - a); }
float lerp(float a, float b, float t) { return a * (1.0f - t) + b * t; }

#if F_CPU == 24000000
#define NUM_FILTERS (4)
#else
#define NUM_FILTERS (10)
#endif

#define LIMITER_ATTACK 8 // measured in samples

class AudioBadLimiter
{
public:
    AudioBadLimiter()
    {
        // limit                    = 0.2f;
        limit = 1.0f;
        gain = 1.0f;
        const float release_secs = 0.04f;
        release = 1.0f - powf(0.5f, 1.0f / AUDIO_SAMPLE_RATE * (2.0f * release_secs));
        sustain = (1.0f / AUDIO_SAMPLE_RATE) * 0.04f;  // 10ms
        amp_mem = 1.0f;
        sustain_time = sustain;

        attack_i = 0;
        for (int i = 0; i < LIMITER_ATTACK; i++)
        {
            memory[i] = 0.0f;
            susmem[i] = 32768;
        }
        box_blur = 32768 * LIMITER_ATTACK;
    }
    void apply(float* data)
    {
        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
        {
            float amplitude = abs(data[i]) * gain;

            if (amplitude > limit)
            {
                gain = limit / abs(data[i]);
                sustain_time = 0;
                //amp_mem = amplitude * 1.1f;
                amp_mem = amplitude * 0.9f;
            }
            else if (amplitude > amp_mem)
                sustain_time = 0;

            if (sustain_time >= sustain && gain != 1.0f)
                gain = lerp(gain, 1.0f, release);
            else
                sustain_time += 1;

            box_blur -= susmem[attack_i];
            susmem[attack_i] = gain * 32768.0f;
            box_blur += susmem[attack_i];

            float out = memory[attack_i];
            memory[attack_i] = data[i];

            attack_i = (attack_i + 1) % LIMITER_ATTACK;

            data[i] = out * ((float) box_blur * (1.0f / 32768.0f / (float) LIMITER_ATTACK));
        }
    }

private:
    float amp_mem;
    float limit;
    float gain;
    float release;
    float level;
    int sustain;
    int sustain_time;

    int box_blur;

    int attack_i;
    int susmem[LIMITER_ATTACK];
    float memory[LIMITER_ATTACK];
};


int filters_ready = 0;
struct BiquadData
{
    BiquadData() { clearFilter(); }

    void clearFilter()
    {
        b0 = 1.0f;
        b1 = 0.0f;
        b2 = 0.0f;

        a1 = 0.0f;
        a2 = 0.0f;

        y1 = 0.0f;
        y2 = 0.0f;

        x1 = 0.0f;
        x2 = 0.0f;
    }

    bool isNullFilter()
    {
        return b0 == 1.0f && b1 == 0.0f && b2 == 0.0f && a1 == 0.0f && a2 == 0.0f;
    }

    void apply(float* data)
    {
        if (isNullFilter() || !filters_ready)
            return;

        int i = 0;
        for (; i < AUDIO_BLOCK_SAMPLES; i++)
        {
            float x0 = data[i];

            float y0 = b0 * x0 + b1 * x1 + b2 * x2 + a1 * y1 + a2 * y2;

            data[i] = y0;

            y2 = y1;
            y1 = y0;

            x2 = x1;
            x1 = x0;
        }

        // fix any NaNs (almost certainly not going to happen, but better safe than sorry)
        if (y2 != y2)
            y2 = 0.0f;
        if (y1 != y1)
            y1 = 0.0f;
        if (x2 != x2)
            x2 = 0.0f;
        if (x1 != x1)
            x1 = 0.0f;
    }

    float a1, a2, b0, b1, b2;
    float x1, x2;
    float y1, y2;
};

class AudioCustomBiquad : public AudioStream
{
public:
    AudioCustomBiquad()
    : AudioStream(1, inputQueueArray)
    {
    }
    void update()
    {
        audio_block_t* block;
        block = receiveWritable();
        if (!block)
            return;

        rms = 0.0f;
        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
        {
            if (block->data[i] == 32767 || block->data[i] < -32768)
            {
                clipping = true;
                clipping_time = 50;
            }
            max_seen = max(max_seen, abs(block->data[i]));

            scratch[i] = float(block->data[i]) / 32768.0f;
            rms += scratch[i] * scratch[i];
        }
        rms = sqrtf(rms / AUDIO_BLOCK_SAMPLES);

        for (int i = 0; i < NUM_FILTERS; i++)
            stages[i].apply(scratch);
        limiter.apply(scratch);

        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
            block->data[i] = max(-32768.0, min(32767.0, scratch[i] * 32768.0));

        transmit(block);
        release(block);
    }

    void setCoefficients(int stage, double* coeffs)
    {
        stage = stage % NUM_FILTERS;

        stages[stage].b0 = coeffs[0];
        stages[stage].b1 = coeffs[1];
        stages[stage].b2 = coeffs[2];
        stages[stage].a1 = -coeffs[3];
        stages[stage].a2 = -coeffs[4];
    }

private:
    float scratch[AUDIO_BLOCK_SAMPLES];
    BiquadData stages[NUM_FILTERS];
    AudioBadLimiter limiter;
    audio_block_t* inputQueueArray[1];
};

AudioControlSGTL5000 analog_control;

#if AUDIO_BLOCK_SAMPLES == 128
AudioInputUSB usb_in;
AudioOutputUSB usb_out;
#endif
AudioInputI2S analog_in;
AudioAmplifier amp1;
AudioAmplifier amp2;
AudioCustomBiquad biquad_left;
AudioCustomBiquad biquad_right;
AudioOutputI2S analog_out;

#if AUDIO_BLOCK_SAMPLES == 128
AudioConnection patchCord1(usb_in, 0, amp1, 0);
AudioConnection patchCord2(usb_in, 1, amp2, 0);
AudioConnection patchCord7(biquad_left, 0, usb_out, 0);
AudioConnection patchCord8(biquad_right, 0, usb_out, 1);
#endif
AudioConnection patchCord1b(analog_in, 0, biquad_left, 0);
AudioConnection patchCord2b(analog_in, 1, biquad_right, 0);
AudioConnection patchCord3(amp1, biquad_left);
AudioConnection patchCord4(amp2, biquad_right);
AudioConnection patchCord9(biquad_left, 0, analog_out, 0);
AudioConnection patchCord10(biquad_right, 0, analog_out, 1);

enum filter_type
{
    T_NULL,
    T_LOWPASS,
    T_HIGHPASS,
    T_LOWSHELF,
    T_HIGHSHELF,
    T_BANDPASS,
    T_NOTCH,
    T_PEAK,
    T_END,
};

struct FilterConfig
{
    uint32_t type = T_NULL;
    float freq_ratio = 0.5;
    float gain = 0.0;
    float q = 1.0;
};

FilterConfig filters[NUM_FILTERS];

void setNullFilter(AudioCustomBiquad* target, int stage);
void setNullFilter(AudioCustomBiquad* target, int stage)
{
    double null_filter[5] = { 1.0, 0.0, 0.0, 0.0, 0.0 };

    target->setCoefficients(stage, null_filter);
}

void setFilter(AudioCustomBiquad* target, int stage, FilterConfig* config);
void setFilter(AudioCustomBiquad* target, int stage, FilterConfig* config)
{
    const float pi = 3.14159265358979323846264338327950288419;

    int type = config->type;
    float fraction = config->freq_ratio;
    float db = config->gain;
    float q = config->q;

    float w = pi * 2.0 * fraction;
    float cos_w = cos(w);
    float sin_w = sin(w);
    float alpha = sin_w / (2.0 * q);

    float a = pow(10.0, db / 40.0);
    float ap = a + 1.0;
    float am = a - 1.0;
    float beta = sqrt(a + a);

    float a0, a1, a2, b0, b1, b2;

    if (type == T_LOWPASS)
    {
        a0 = 1.0 + alpha;
        a1 = -2.0 * cos_w;
        a2 = 1.0 - alpha;
        b0 = (1.0 - cos_w) / 2.0;
        b1 = (1.0 - cos_w);
        b2 = (1.0 - cos_w) / 2.0;
    }
    else if (type == T_HIGHPASS)
    {
        a0 = 1.0 + alpha;
        a1 = -2.0 * cos_w;
        a2 = 1.0 - alpha;
        b0 = (1.0 + cos_w) / 2.0;
        b1 = -(1.0 + cos_w);
        b2 = (1.0 + cos_w) / 2.0;
    }
    else if (type == T_LOWSHELF)
    {
        a0 = ap + am * cos_w + beta * sin_w;
        a1 = (am + ap * cos_w) * -2.0;
        a2 = ap + am * cos_w - beta * sin_w;
        b0 = a * (ap - am * cos_w + beta * sin_w);
        b1 = a * (am - ap * cos_w) * 2.0;
        b2 = a * (ap - am * cos_w - beta * sin_w);
    }
    else if (type == T_HIGHSHELF)
    {
        a0 = ap - am * cos_w + beta * sin_w;
        a1 = (am - ap * cos_w) * 2.0;
        a2 = ap - am * cos_w - beta * sin_w;
        b0 = a * (ap + am * cos_w + beta * sin_w);
        b1 = a * (am + ap * cos_w) * -2.0;
        b2 = a * (ap + am * cos_w - beta * sin_w);
    }
    else if (type == T_BANDPASS)
    {
        a0 = 1.0 + alpha;
        a1 = -2.0 * cos_w;
        a2 = 1.0 - alpha;
        b0 = alpha;
        b1 = 0.0;
        b2 = -alpha;
    }
    else if (type == T_NOTCH)
    {
        a0 = 1.0 + alpha;
        a1 = -2.0 * cos_w;
        a2 = 1.0 - alpha;
        b0 = 1.0;
        b1 = -2.0 * cos_w;
        b2 = 1.0;
    }
    else if (type == T_PEAK)
    {
        a0 = 1.0 + alpha / a;
        a1 = -2.0 * cos_w;
        a2 = 1.0 - alpha / a;
        b0 = 1.0 + alpha * a;
        b1 = -2.0 * cos_w;
        b2 = 1.0 - alpha * a;
    }
    else  // unsupported, null
    {
        setNullFilter(target, stage);
        return;
    }

    double filter[5];

    filter[0] = b0 / a0;
    filter[1] = b1 / a0;
    filter[2] = b2 / a0;

    filter[3] = a1 / a0;
    filter[4] = a2 / a0;

    Serial.printf(
        "built filter: %f %f %f %f %f\n", filter[0], filter[1], filter[2], filter[3], filter[4]);

    target->setCoefficients(stage, filter);
}

elapsedMillis timer;
unsigned int last_timer;

enum
{
    MODE_DIGITAL,
    MODE_ANALOG,
};

template<typename T>
T read_any(size_t addr)
{
    T ret;
    uint8_t* loc = (uint8_t*) (&ret);
    memset(loc, 0, sizeof(ret));

    for (size_t i = 0; i < sizeof(ret); i++)
        loc[i] |= EEPROM.read(addr + i);

    return ret;
}

template<typename T>
void write_any(size_t addr, T val)
{
    uint8_t* loc = (uint8_t*) (&val);

    for (size_t i = 0; i < sizeof(val); i++)
        EEPROM.write(addr + i, loc[i]);
}

template<typename T>
size_t read_array(size_t addr, T* val, size_t size)
{
    for (size_t i = 0; i < size; i++)
    {
        *val = read_any<T>(addr);
        addr += sizeof(*val);
        val += 1;
    }
    return addr;
}

template<typename T>
size_t write_array(size_t addr, T* val, size_t size)
{
    for (size_t i = 0; i < size; i++)
    {
        write_any(addr, *val);
        addr += sizeof(*val);
        val += 1;
    }
    return addr;
}

uint32_t mode = MODE_ANALOG;
float volume = 0.35;
float in_level = 0;

char magic[16] = "PEACEMADEEQMEM3";
void setup_storage()
{
    magic[15] = NUM_FILTERS;
    char found_magic[16];
    size_t addr = 0;
    addr = read_array(0, found_magic, 16);
    if (memcmp(magic, found_magic, 16))
    {
        Serial.printf("EEPROM unintialized! Initializing...\n");
        write_array(0, magic, 16);
        addr = write_array(addr, filters, NUM_FILTERS);
        addr = write_array(addr, &mode, 1);
        addr = write_array(addr, &volume, 1);
        addr = write_array(addr, &in_level, 1);
    }
    else
    {
        Serial.printf("EEPROM initialized already!\n");
        addr = read_array(addr, filters, NUM_FILTERS);
        addr = read_array(addr, &mode, 1);
        addr = read_array(addr, &volume, 1);
        addr = read_array(addr, &in_level, 1);
    }
}

void setup()
{
    pinMode(13, OUTPUT);

    Serial.printf("len %d\n", sizeof(filters));

    analog_control.enable();

    analog_control.audioProcessorDisable();
    analog_control.adcHighPassFilterDisable();

    analog_control.inputSelect(AUDIO_INPUT_LINEIN);
    analog_control.unmuteHeadphone();

    last_timer = timer;

    Serial.begin(9600);

    AudioMemory(80);

    for (size_t i = 0; i < 16; i++)
        Serial.printf("%02X ", EEPROM.read(i));
    Serial.println("");

#if NUM_FILTERS >= 4
    filters[0] = { T_HIGHPASS, 40.0 / AUDIO_SAMPLE_RATE, 0.0, 0.7071 };
    filters[1] = { T_PEAK, 220.0 / AUDIO_SAMPLE_RATE, -4.5, 2.0 };
    filters[2] = { T_PEAK, 4250.0 / AUDIO_SAMPLE_RATE, -9.0, 6.0 };
    filters[3] = { T_HIGHSHELF, 7500.0 / AUDIO_SAMPLE_RATE, -4.5, 0.7071 };
    filters[4] = { T_PEAK, 125.0 / AUDIO_SAMPLE_RATE, 7.5, 0.5 };
#endif

    setup_storage();

    analog_control.volume(volume);
    analog_control.lineInLevel(in_level);

#if AUDIO_BLOCK_SAMPLES == 128
    if (mode == MODE_DIGITAL)
    {
        // analog input connectors
        patchCord1b.disconnect();
        patchCord2b.disconnect();
        // digital input connectors
        patchCord3.connect();
        patchCord4.connect();
    }
    else if (mode == MODE_ANALOG)
    {
        // digital input connectors
        patchCord3.disconnect();
        patchCord4.disconnect();
        // analog input connectors
        patchCord1b.connect();
        patchCord2b.connect();
    }
#endif

    for (int i = 0; i < NUM_FILTERS; i++)
    {
        setFilter(&biquad_right, i, &filters[i]);
        setFilter(&biquad_left, i, &filters[i]);
    }

    usbMIDI.setHandleControlChange(acceptControlChange);

    Serial.println("eq loaded");

    filters_ready = 1;
}

bool wants_update = false;
bool filters_dirty = false;
void acceptControlChange(byte channel, byte control, byte value)
{
    if (channel == 1)
    {
        if (control == 94)
        {
            uint32_t new_mode = value % 2;
            if (mode != new_mode)
            {
#if AUDIO_BLOCK_SAMPLES == 128
                if (new_mode == MODE_DIGITAL)
                {
                    // analog input connectors
                    patchCord1b.disconnect();
                    patchCord2b.disconnect();
                    // digital input connectors
                    patchCord3.connect();
                    patchCord4.connect();
                }
                else if (new_mode == MODE_ANALOG)
                {
                    // digital input connectors
                    patchCord3.disconnect();
                    patchCord4.disconnect();
                    // analog input connectors
                    patchCord1b.connect();
                    patchCord2b.connect();
                }
#endif
            }
            mode = new_mode;
        }
        else if (control == 95)
        {
            in_level = value % 16;
            analog_control.lineInLevel(in_level);
        }
        else if (control == 31)
        {
            volume = float(value) / 127.0;
            analog_control.volume(volume);
        }
        else if (control == 63)
        {
            if (value == 1)
                wants_update = true;
        }
        else if (control < 94)
        {
            if (control >= 64)
            {
                int i = control - 64;
                if (i < NUM_FILTERS)
                {
                    filters[i].type = value;

                    setFilter(&biquad_right, i, &filters[i]);
                    setFilter(&biquad_left, i, &filters[i]);
                }
            }
            else
            {
                int filter = (control % 32) / 3;
                int prop = (control % 32) % 3;
                bool is_msb = control < 32;

                int i = filter;
                if (i < NUM_FILTERS)
                {
                    uint16_t data = 0;

                    if (prop == 0)
                        data = freq_to_control(filters[i].freq_ratio * AUDIO_SAMPLE_RATE);
                    else if (prop == 1)
                        data = db_to_control(filters[i].gain);
                    else if (prop == 2)
                        data = q_to_control(filters[i].q);

                    if (is_msb == 1)
                    {
                        data &= ~(0x7F << 7);
                        data |= value << 7;
                    }
                    else
                    {
                        data &= ~0x7F;
                        data |= value;
                    }

                    if (prop == 0)
                        filters[i].freq_ratio = control_to_freq(data) / AUDIO_SAMPLE_RATE;
                    else if (prop == 1)
                        filters[i].gain = control_to_db(data);
                    else if (prop == 2)
                        filters[i].q = control_to_q(data);

                    setFilter(&biquad_right, i, &filters[i]);
                    setFilter(&biquad_left, i, &filters[i]);
                }
            }
        }
        filters_dirty = true;
    }
}

uint16_t f_to_control(float f)
{
    f = max(0.0, min(1.0, f));
    return round(f * 16383.0);
}
float control_to_f(uint16_t f) { return float(f) / 16383.0; }

uint16_t freq_to_control(float freq)
{
    float low = log(10.0);
    float high = log(24000.0);
    float mid = log(freq);

    float f = min(1.0, max(0.0, unlerp(low, high, mid)));
    return f_to_control(f);
}
float control_to_freq(uint16_t control)
{
    float f = control_to_f(control);

    float low = log(10.0);
    float high = log(24000.0);
    float mid = lerp(low, high, f);

    return exp(mid);
}

uint16_t db_to_control(float db)
{
    float f = min(1.0, max(0.0, unlerp(-18.0, 18.0, db)));
    return f_to_control(f);
}
float control_to_db(uint16_t control)
{
    float f = control_to_f(control);
    return lerp(-18.0, 18.0, f);
}

uint16_t q_to_control(float q)
{
    float low = log(0.333333);
    float high = log(33.333333);
    float mid = log(q);

    float f = min(1.0, max(0.0, unlerp(low, high, mid)));
    return f_to_control(f);
}
float control_to_q(uint16_t control)
{
    float f = control_to_f(control);

    float low = log(0.333333);
    float high = log(33.333333);
    float mid = lerp(low, high, f);

    return exp(mid);
}

int prev_volume = 0;  // for usb
void loop()
{
#if AUDIO_BLOCK_SAMPLES == 128
    int new_volume = usb_in.volume();
    if (prev_volume != new_volume)
    {
        amp1.gain(new_volume);
        amp2.gain(new_volume);
        prev_volume = new_volume;
    }
#endif

    if (last_timer / 4000 != timer / 4000 && filters_dirty)
    {
        filters_dirty = false;
        size_t addr;
        addr = write_array(16, filters, NUM_FILTERS);
        addr = write_array(addr, &mode, 1);
        addr = write_array(addr, &volume, 1);
        addr = write_array(addr, &in_level, 1);
        Serial.println("wrote filters to EEPROM");
    }
    if (last_timer / 2000 != timer / 2000)
    {
        Serial.print("Audio processor usage: ");
        Serial.println(AudioProcessorUsage());
        Serial.print("Max seen input value: ");
        Serial.println(max_seen);
        max_seen = 0;
        Serial.print("RMS input value (db): ");
        Serial.println(logf(rms) / logf(10.0f) * 20.0f);
    }

    if (clipping)
    {
        clipping_time = clamp(clipping_time - (timer - last_timer), 0, 1000);
        if (clipping_time == 0)
            clipping = false;
    }
    if (last_timer / 8 != timer / 8)
    {
        if (clipping)
            led_on();
        else
            led_off();

        while (usbMIDI.read()) {}

        if (wants_update)
        {
            wants_update = false;
            int channel = 1;

            usbMIDI.sendControlChange(94, mode, channel);
            usbMIDI.sendControlChange(95, in_level, channel);
            usbMIDI.sendControlChange(62, NUM_FILTERS, channel);
            usbMIDI.sendControlChange(31, uint8_t(volume * 127.0), channel);

            for (int i = 0; i < NUM_FILTERS; i++)
            {
                usbMIDI.sendControlChange(i + 64, filters[i].type & 0x7F, channel);

                uint16_t data = 0;

                data = freq_to_control(filters[i].freq_ratio * AUDIO_SAMPLE_RATE);

                // msb goes in 0..=30 and must be sent first (I think)
                usbMIDI.sendControlChange(i * 3 + 0 + 0, (data >> 7) & 0x7F, channel);
                // lsb goes in 32..=62
                usbMIDI.sendControlChange(i * 3 + 0 + 32, data & 0x7F, channel);

                data = db_to_control(filters[i].gain);
                usbMIDI.sendControlChange(i * 3 + 1 + 0, (data >> 7) & 0x7F, channel);
                usbMIDI.sendControlChange(i * 3 + 1 + 32, data & 0x7F, channel);

                data = q_to_control(filters[i].q);
                usbMIDI.sendControlChange(i * 3 + 2 + 0, (data >> 7) & 0x7F, channel);
                usbMIDI.sendControlChange(i * 3 + 2 + 32, data & 0x7F, channel);
            }
        }
    }
    last_timer = timer;
}