
// lowering causes bugs in teensy audio library
//#define AUDIO_BLOCK_SAMPLES 128

// setting doesn't do anything (each update is still 2.9ms apart, i.e. 128 44100hz samples)
//#define AUDIO_SAMPLE_RATE_EXACT 96000.0f

#include <Arduino.h>

#include <Audio.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <EEPROM.h>

bool clipping = false;
int32_t clipping_time = 0; // milliseconds
void led_on()
{
    digitalWrite(13, HIGH);
}
void led_off()
{
    digitalWrite(13, LOW);
}

int clamp(int a, int min, int max)
{
    return a < min ? min : (a > max ? max : a);
}

float unlerp(float a, float b, float t)
{
    return (t-a)/(b-a);
}
float lerp(float a, float b, float t)
{
    return a*(1.0-t) + b*t;
}

class AudioBadLimiter
{
public:
	AudioBadLimiter()
    {
        limit = 1.0;
		gain = 1.0;
        release = 0.020*AUDIO_SAMPLE_RATE;
        
        hit_progress = 0;
        hit_gain = 0.0;
        hit_gain_start = 0.0;
	}
    void apply(float * data)
    {
        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
        {
            float amplitude = abs(data[i]) * gain;
            
            if (release > 0)
            {
                float release_f = unlerp(0.0, float(release), float(hit_progress));
                
                if (amplitude > limit)
                {
                    clipping = true;
                    clipping_time = 1000;
                    
                    float next_gain = gain * limit/amplitude;
                    hit_gain_start = gain;
                    hit_gain = next_gain;
                    hit_progress = 0;
                }
                else if (release_f < 1.0)
                {
                    gain = lerp(hit_gain, 1.0, release_f);
                    hit_progress += 1;
                }
                else
                    gain = 1.0;
            }
            else
            {
                if (amplitude > limit)
                {
                    clipping = true;
                    clipping_time = 1000;
                    
                    gain = limit/amplitude;
                }
                else
                    gain = 1.0;
            }
            
            data[i] *= gain;
        }
    }

private:
    float limit;
	float gain;
    int release;
    int hit_progress;
    float hit_gain_start;
    float hit_gain;
};

struct BiquadData
{
    BiquadData()
    {
        clearFilter();
	}
    
    void clearFilter()
    {
        b0 = 1.0;
        b1 = 0.0;
        b2 = 0.0;
        
        a1 = 0.0;
        a2 = 0.0;
        
        y1 = 0.0;
        y2 = 0.0;
        
        x1 = 0.0;
        x2 = 0.0;
    }
    
    bool isNullFilter()
    {
        return b0 == 1.0 && b1 == 0.0 && b2 == 0.0 && a1 == 0.0 && a2 == 0.0;
    }
    
    void apply(float * data)
    {
        if (isNullFilter())
            return;
        
        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
        {
            float x0 = data[i];
            float y0 =
                b0 * x0 +
                b1 * x1 +
                b2 * x2 +
                a1 * y1 +
                a2 * y2
            ;
            
            data[i] = y0;
            
            y2 = y1;
            y1 = y0;
            
            x2 = x1;
            x1 = x0;
        }
        // fix any NaNs
        if (y2 != y2) y2 = 0.0;
        if (y1 != y1) y1 = 0.0;
        if (x2 != x2) x2 = 0.0;
        if (x1 != x1) x1 = 0.0;
    }
    
    float a1, a2, b0, b1, b2;
    float x1, x2;
    float y1, y2;
};

class AudioCustomBiquad : public AudioStream
{
public:
	AudioCustomBiquad() : AudioStream(1, inputQueueArray) {}
	void update()
    {
        audio_block_t * block;
        block = receiveWritable();
        if (!block)
            return;
        
        float scratch[AUDIO_BLOCK_SAMPLES];
        
        for(int i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
            scratch[i] = float(block->data[i]) / 32768.0;
        
        stage_0.apply(scratch);
        stage_1.apply(scratch);
        stage_2.apply(scratch);
        stage_3.apply(scratch);
        limiter.apply(scratch);
        
        for(int i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
            block->data[i] = max(-32768.0, min(32767.0, scratch[i] * 32768.0));
        
        transmit(block);
        release(block);
    }
    
    void setCoefficients(int stage, double * coeffs)
    {
        BiquadData * target = nullptr;
        
        if(stage == 0)
            target = &stage_0;
        else if(stage == 1)
            target = &stage_1;
        else if(stage == 2)
            target = &stage_2;
        else if(stage == 3)
            target = &stage_3;
        else
            return;
        
        target->b0 = coeffs[0];
        target->b1 = coeffs[1];
        target->b2 = coeffs[2];
        target->a1 = -coeffs[3];
        target->a2 = -coeffs[4];
    }

private:
    BiquadData stage_0;
    BiquadData stage_1;
    BiquadData stage_2;
    BiquadData stage_3;
    AudioBadLimiter limiter;
	audio_block_t * inputQueueArray[1];
};

AudioControlSGTL5000     analog_control;

AudioInputUSB            usb_in;
AudioInputI2S            analog_in;
AudioAmplifier           amp1;
AudioAmplifier           amp2;
AudioAmplifier           amp_a_1;
AudioAmplifier           amp_a_2;
AudioCustomBiquad        biquad_left;
AudioCustomBiquad        biquad_right;
AudioOutputI2S           analog_out;
AudioOutputUSB           usb_out;

AudioConnection          patchCord1 (usb_in, 0, amp1, 0);
AudioConnection          patchCord2 (usb_in, 1, amp2, 0);
AudioConnection          patchCord3 (amp1, biquad_left);
AudioConnection          patchCord4 (amp2, biquad_right);
AudioConnection          patchCord7 (biquad_left, 0, usb_out, 0);
AudioConnection          patchCord8 (biquad_right, 0, usb_out, 1);
AudioConnection          patchCord9 (biquad_left, 0, analog_out, 0);
AudioConnection          patchCord10(biquad_right, 0, analog_out, 1);

AudioConnection          patchCord1b(analog_in, 0, biquad_left, 0);
AudioConnection          patchCord2b(analog_in, 1, biquad_right, 0);

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

void setNullFilter(AudioCustomBiquad * target, int stage);
void setNullFilter(AudioCustomBiquad * target, int stage)
{
    double null_filter[5] = { 1.0, 0.0, 0.0, 0.0, 0.0 };
    
    target->setCoefficients(stage, null_filter);
}

void setFilter(AudioCustomBiquad * target, int stage, FilterConfig * config);
void setFilter(AudioCustomBiquad * target, int stage, FilterConfig * config)
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
    
    float a = pow(10.0, db/40.0);
    float ap = a + 1.0;
    float am = a - 1.0;
    float beta = sqrt(a + a);
    
    float a0, a1, a2, b0, b1, b2;
    
    if(type == T_LOWPASS)
    {
        a0 =   1.0 + alpha;
        a1 =  -2.0 * cos_w;
        a2 =   1.0 - alpha;
        b0 =  (1.0 - cos_w) / 2.0;
        b1 =  (1.0 - cos_w);
        b2 =  (1.0 - cos_w) / 2.0;
    }
    else if(type == T_HIGHPASS)
    {
        a0 =   1.0 + alpha;
        a1 =  -2.0 * cos_w;
        a2 =   1.0 - alpha;
        b0 =  (1.0 + cos_w) / 2.0;
        b1 = -(1.0 + cos_w);
        b2 =  (1.0 + cos_w) / 2.0;
    }
    else if(type == T_LOWSHELF)
    {
        a0 =      ap + am * cos_w + beta * sin_w;
        a1 =     (am + ap * cos_w) * -2.0;
        a2 =      ap + am * cos_w - beta * sin_w;
        b0 = a * (ap - am * cos_w + beta * sin_w);
        b1 = a * (am - ap * cos_w) * 2.0;
        b2 = a * (ap - am * cos_w - beta * sin_w);
    }
    else if(type == T_HIGHSHELF)
    {
        a0 =       ap - am * cos_w + beta * sin_w;
        a1 =      (am - ap * cos_w) * 2.0;
        a2 =       ap - am * cos_w - beta * sin_w;
        b0 =  a * (ap + am * cos_w + beta * sin_w);
        b1 =  a * (am + ap * cos_w) * -2.0;
        b2 =  a * (ap + am * cos_w - beta * sin_w);
    }
    else if(type == T_BANDPASS)
    {
        a0 =  1.0 + alpha;
        a1 = -2.0 * cos_w;
        a2 =  1.0 - alpha;
        b0 =  alpha;
        b1 =  0.0;
        b2 = -alpha;
    }
    else if(type == T_NOTCH)
    {
        a0 =  1.0 + alpha;
        a1 = -2.0 * cos_w;
        a2 =  1.0 - alpha;
        b0 =  1.0;
        b1 = -2.0 * cos_w;
        b2 =  1.0;
    }
    else if(type == T_PEAK)
    {
        a0 =  1.0 + alpha / a;
        a1 = -2.0 * cos_w;
        a2 =  1.0 - alpha / a;
        b0 =  1.0 + alpha * a;
        b1 = -2.0 * cos_w;
        b2 =  1.0 - alpha * a;
    }
    else // unsupported, null
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
    
    Serial.printf("built filter: %f %f %f %f %f\n", filter[0], filter[1], filter[2], filter[3], filter[4]);
    
    target->setCoefficients(stage, filter);
}

const int num_filters = 4;
FilterConfig filters[num_filters];

elapsedMillis timer;
unsigned int last_timer;

enum {
    MODE_DIGITAL,
    MODE_ANALOG,
};

template <typename T>
T read_any(size_t addr)
{
    T ret;
    uint8_t * loc = (uint8_t *)(&ret);
    memset(loc, 0, sizeof(ret));
    
    for (size_t i = 0; i < sizeof(ret); i++)
        loc[i] |= EEPROM.read(addr + i);
    
    return ret;
}

template <typename T>
void write_any(size_t addr, T val)
{
    uint8_t * loc = (uint8_t *)(&val);
    
    for (size_t i = 0; i < sizeof(val); i++)
        EEPROM.write(addr + i, loc[i]);
}

template <typename T>
size_t read_array(size_t addr, T * val, size_t size)
{
    for (size_t i = 0; i < size; i++)
    {
        *val = read_any<T>(addr);
        addr += sizeof(*val);
        val += 1;
    }
    return addr;
}

template <typename T>
size_t write_array(size_t addr, T * val, size_t size)
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

char magic[16] = "PEACEMADEEQ_MEM";
void setup_storage()
{
    char found_magic[16];
    size_t addr = 0;
    addr = read_array(0, found_magic, 16);
    if (memcmp(magic, found_magic, 16))
    {
        Serial.printf("EEPROM unintialized! Initializing...\n");
        write_array(0, magic, 16);
        addr = write_array(addr, filters, num_filters);
        addr = write_array(addr, &mode, 1);
        addr = write_array(addr, &volume, 1);
        addr = write_array(addr, &in_level, 1);
    }
    else
    {
        Serial.printf("EEPROM initialized already!\n");
        addr = read_array(addr, filters, num_filters);
        addr = read_array(addr, &mode, 1);
        addr = read_array(addr, &volume, 1);
        addr = read_array(addr, &in_level, 1);
    }
}

void setup()
{
    Serial.printf("len %d\n", sizeof(filters));
    
    analog_control.enable();
    
    analog_control.audioProcessorDisable();
    analog_control.adcHighPassFilterDisable();
    
    analog_control.inputSelect(AUDIO_INPUT_LINEIN);
    analog_control.unmuteHeadphone();
    
    last_timer = timer;
    
    Serial.begin(9600);
    delay(500);
    
    AudioMemory(80);
    
    for(size_t i = 0; i < 16; i++)
        Serial.printf("%02X ", EEPROM.read(i));
    Serial.println("");
    
    filters[0] = { T_HIGHPASS,    40.0/AUDIO_SAMPLE_RATE,  0.0, 0.7071 };
    filters[1] = { T_PEAK,       220.0/AUDIO_SAMPLE_RATE, -4.5, 2.0 };
    filters[2] = { T_PEAK,      4250.0/AUDIO_SAMPLE_RATE, -9.0, 6.0 };
    filters[3] = { T_HIGHSHELF, 7500.0/AUDIO_SAMPLE_RATE, -4.5, 0.7071 };
    
    setup_storage();
    
    analog_control.volume(volume);
    analog_control.lineInLevel(in_level);
    
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
    
    for(int i = 0; i < num_filters; i++)
    {
        setFilter(&biquad_right, i, &filters[i]);
        setFilter(&biquad_left, i, &filters[i]);
    }
    
    usbMIDI.setHandleControlChange(acceptControlChange);
    
    Serial.println("eq loaded");
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
            volume = float(value)/127.0;
            analog_control.volume(volume);
        }
        else if (control == 63)
        {
            if (value == 1)
                wants_update = true;
        }
        else if (control < 94)
        {
            if(control >= 64)
            {
                int i = control - 64;
                i = i % num_filters;
                filters[i].type = value;
                
                setFilter(&biquad_right, i, &filters[i]);
                setFilter(&biquad_left, i, &filters[i]);
            }
            else
            {
                int filter = (control % 32) / 3;
                int prop = (control % 32) % 3;
                bool is_msb = control < 32;
                
                int i = filter;
                
                uint16_t data = 0;
                
                if(prop == 0)
                    data = freq_to_control(filters[i].freq_ratio * AUDIO_SAMPLE_RATE);
                else if(prop == 1)
                    data = db_to_control(filters[i].gain);
                else if(prop == 2)
                    data = q_to_control(filters[i].q);
                
                if(is_msb == 1)
                {
                    data &= ~(0x7F << 7);
                    data |= value << 7;
                }
                else
                {
                    data &= ~0x7F;
                    data |= value;
                }
                
                if(prop == 0)
                    filters[i].freq_ratio = control_to_freq(data) / AUDIO_SAMPLE_RATE;
                else if(prop == 1)
                    filters[i].gain = control_to_db(data);
                else if(prop == 2)
                    filters[i].q = control_to_q(data);
                
                setFilter(&biquad_right, i, &filters[i]);
                setFilter(&biquad_left, i, &filters[i]);
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
float control_to_f(uint16_t f)
{
    return float(f) / 16383.0;
}

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

int prev_volume = 0;
void loop()
{
    int new_volume = usb_in.volume();
    if (prev_volume != new_volume)
    {
        amp1.gain(new_volume);
        amp2.gain(new_volume);
        prev_volume = new_volume;
    }
    
    if (last_timer/4000 != timer/4000 && filters_dirty)
    {
        filters_dirty = false;
        size_t addr;
        addr = write_array(16, filters, num_filters);
        addr = write_array(addr, &mode, 1);
        addr = write_array(addr, &volume, 1);
        addr = write_array(addr, &in_level, 1);
        Serial.println("wrote filters to EEPROM");
    }
    if (last_timer/2000 != timer/2000)
    {
        Serial.print("Audio processor usage: ");
        Serial.println(AudioProcessorUsage());
    }
    
    if (clipping)
    {
        clipping_time = clamp(clipping_time - (timer - last_timer), 0, 1000);
        if (clipping_time == 0)
            clipping = false;
    }
    if (last_timer/8 != timer/8)
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
            usbMIDI.sendControlChange(31, uint8_t(volume*127.0), channel);
            
            for(int i = 0; i < num_filters; i++)
            {
                usbMIDI.sendControlChange(i + 64, filters[i].type & 0x7F, channel);
                
                uint16_t data = 0;
                
                data = freq_to_control(filters[i].freq_ratio * AUDIO_SAMPLE_RATE);
                
                // msb goes in 0..=30 and must be sent first (I think)
                usbMIDI.sendControlChange(i*3 + 0 + 0, (data>>7) & 0x7F, channel);
                // lsb goes in 32..=62
                usbMIDI.sendControlChange(i*3 + 0 + 32, data & 0x7F, channel);
                
                data = db_to_control(filters[i].gain);
                usbMIDI.sendControlChange(i*3 + 1 + 0, (data>>7) & 0x7F, channel);
                usbMIDI.sendControlChange(i*3 + 1 + 32, data & 0x7F, channel);
                
                data = q_to_control(filters[i].q);
                usbMIDI.sendControlChange(i*3 + 2 + 0, (data>>7) & 0x7F, channel);
                usbMIDI.sendControlChange(i*3 + 2 + 32, data & 0x7F, channel);
            }
        }
    }
    last_timer = timer;
}