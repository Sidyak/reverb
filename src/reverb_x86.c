/*
Author: Kim Radmacher

Date: 15.09.2022

Description:
  Schroeders reverb
  Project is deployed on x86
*/

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

//#define DEBUG

#if defined(_MSC_VER)
#include <getopt.h>
#else
#include <unistd.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif
#include "wavreader.h"
#include "wavwriter.h"
#ifdef __cplusplus
}
#endif

const int iMAX_BUFFER_SIZE = (2*48000); // 2 seconds max reverb

int iFFCF1_BUFFER_SIZE = 1687;
int iFFCF2_BUFFER_SIZE = 1601;
int iFFCF3_BUFFER_SIZE = 2053;
int iFFCF4_BUFFER_SIZE = 2251;
const float fFFCF1_GAIN = 0.773;
const float fFFCF2_GAIN = 0.802f;
const float fFFCF3_GAIN = 0.753f;
const float fFFCF4_GAIN = 0.733f;

int iAP1_BUFFER_SIZE = 347;
int iAP2_BUFFER_SIZE = 113;
int iAP3_BUFFER_SIZE = 37;
const float fAP1_GAIN = 0.7f;
const float fAP2_GAIN = 0.7f;
const float fAP3_GAIN = 0.7f;

int32_t inL, inR, outL, outR;

// Buffer
float *fFFCF1;
float *fFFCF2;
float *fFFCF3;
float *fFFCF4;
float *fAP1;
float *fAP2;
float *fAP3;

// Index to access the buffer
int iFFCF1 = 0;
int iFFCF2 = 0;
int iFFCF3 = 0;
int iFFCF4 = 0;
int iAP1 = 0;
int iAP2 = 0;
int iAP3 = 0;

float dryWet = 0;
float modReverb = 0;

void cleanup(void);

bool setup(void)
{

    fFFCF1 = (float*)malloc(iMAX_BUFFER_SIZE*sizeof(float));
    if(!fFFCF1)
        return false;

    fFFCF2 = (float*)malloc(iMAX_BUFFER_SIZE*sizeof(float));
    if(!fFFCF2)
        return false;

    fFFCF3 = (float*)malloc(iMAX_BUFFER_SIZE*sizeof(float));
    if(!fFFCF3)
        return false;

    fFFCF4 = (float*)malloc(iMAX_BUFFER_SIZE*sizeof(float));
    if(!fFFCF4)
        return false;

    fAP1 = (float*)malloc(iMAX_BUFFER_SIZE*sizeof(float));
    if(!fAP1)
        return false;

    fAP2 = (float*)malloc(iMAX_BUFFER_SIZE*sizeof(float));
    if(!fAP2)
        return false;

    fAP3 = (float*)malloc(iMAX_BUFFER_SIZE*sizeof(float));
    if(!fAP3)
        return false;

    return true;

}

void usage(const char* name)
{
    fprintf(stderr, "%s in.wav out.wav <dry/wet in a range of 0...100 percent>\n", name);
}

// NOTE: and TODO: currently only wav 16 bit is supported 
#define MAX_SMP_VAL (1.f * 32767.f)
#define MIN_SMP_VAL (-1.f * 32767.f)

inline float hardClip(float x)
{
    return (x > MAX_SMP_VAL) ? MAX_SMP_VAL : (x < MIN_SMP_VAL) ? MIN_SMP_VAL : x;
}

// Process a all pass
float processAP(float x, float g, float* state, int* i, int iBufsize)
{
    float y;
    int index = *i;

    y = -g * x + state[index];
    y *= (1 - g*g); // Added due to high gain -> clipping

    state[index] = g * state[(index-1+iBufsize)%iBufsize] + g * x;//x;//g * x;//

    if(++index > iBufsize)
        index = 0;

    *i = index;

    return hardClip(y);
}

// Process a feed backwards comb filer
float processFBCF(float x, float g, float* state, int* i, int iBufsize)
{
    float y;
    int index = *i;

    y = x + g * state[index];
    state[index] = y;

    if(++index > iBufsize)
        index = 0;

    *i = index;

    return hardClip(y);
}

// Process a feed forward comb filer
float processFFCF(float x, float g, float* state, int* i, int iBufsize)
{
    float y;
    int index = *i;

    y = g * x + g * state[index];
    
    state[index] = x;

    if(++index > iBufsize)
        index = 0;

    *i = index;

    return hardClip(y);
}

// Process mixing matrix
float processMM(float x1, float x2, float x3, float x4)
{
    float s1, s2;
    s1 = x1 + x3;
    s2 = x2 + x4;

    float OutA, OutB, OutC, OutD;

    // Different reverb combinations
    OutA = s1 + s2;
    OutB = -OutA;
    OutD = s1 - s2;
    OutC = -OutD;

    return hardClip(OutA);
}

int main(int argc, char *argv[])
{
    const char *infile, *outfile;
    FILE *out;
    void *wavIn;
    void *wavOut;
    int format, sample_rate, channels, bits_per_sample;
    uint32_t data_length;
    int input_size;
    uint8_t* input_buf;
    int16_t* convert_buf;

    if(!setup())
    {
        fprintf(stderr, "setup failed\n");
    }
    
    if (argc - optind < 2)
    {
        fprintf(stderr, "Error: not enough parameter provided\n");
        usage(argv[0]);
        return 1;
    }
    
    infile = argv[optind];
    outfile = argv[optind + 1];

    if (argc - optind > 2)
    {
        dryWet = atoi(argv[optind + 2]);
    }

    if (argc - optind > 3)
    {
        modReverb = atoi(argv[optind + 3]);
    }

    wavIn = wav_read_open(infile);
    if (!wavIn)
    {
        fprintf(stderr, "Unable to open wav file %s\n", infile);
        return 1;
    }
    if (!wav_get_header(wavIn, &format, &channels, &sample_rate, &bits_per_sample, &data_length))
    {
        fprintf(stderr, "Bad wav file %s\n", infile);
        return 1;
    }
    if (format != 1)
    {
        fprintf(stderr, "Unsupported WAV format %d\n", format);
        return 1;
    }

    wavOut = wav_write_open(outfile, sample_rate, bits_per_sample, channels);

    if (!wavOut)
    {
        fprintf(stderr, "Unable to open wav file for writing %s\n", infile);
        return 1;
    }

    input_size = data_length;
    input_buf = (uint8_t*) malloc(input_size);
    convert_buf = (int16_t*) malloc(input_size);

    if (input_buf == NULL || convert_buf == NULL)
    {
        fprintf(stderr, "Unable to allocate memory for buffer\n");
        return 1;
    }

    int read = wav_read_data(wavIn, input_buf, input_size);

    if(dryWet > 100)
    {
        printf("WARNING: dryWet > 100 saturating to 100\n");
        dryWet = 100;
    }
    if(dryWet < 0.f)
    {
        printf("WARNING: dryWet < 0 saturating to 0\n");
        dryWet = 0;
    }

    printf("using dryWet = %f percent \n", dryWet);


    if(modReverb)
    {
        if(modReverb > 100)
        {
            printf("WARNING: modReverb > 100 saturating to 100\n");
            modReverb = 100;
        }
        if(modReverb < 0.f)
        {
            printf("WARNING: modReverb < 0 saturating to 0\n");
            modReverb = 0;
        }
        printf("using modReverb = %f percent \n", modReverb);
        modReverb = modReverb/100.f;

        iFFCF1_BUFFER_SIZE = (int)(expf(2.9 * modReverb) * iFFCF1_BUFFER_SIZE);
        iFFCF2_BUFFER_SIZE = (int)(expf(2.9 * modReverb) * iFFCF2_BUFFER_SIZE);
        iFFCF3_BUFFER_SIZE = (int)(expf(2.9 * modReverb) * iFFCF3_BUFFER_SIZE);
        iFFCF4_BUFFER_SIZE = (int)(expf(2.9 * modReverb) * iFFCF4_BUFFER_SIZE);
        iAP1_BUFFER_SIZE = (int)(expf(2.9 * modReverb) * iAP1_BUFFER_SIZE);
        iAP2_BUFFER_SIZE = (int)(expf(2.9 * modReverb) * iAP2_BUFFER_SIZE);
        iAP3_BUFFER_SIZE = (int)(expf(2.9 * modReverb) * iAP3_BUFFER_SIZE);

        printf("using iFFCF1_BUFFER_SIZE = %d\n", iFFCF1_BUFFER_SIZE);
        printf("using iFFCF2_BUFFER_SIZE = %d\n", iFFCF2_BUFFER_SIZE);
        printf("using iFFCF3_BUFFER_SIZE = %d\n", iFFCF3_BUFFER_SIZE);
        printf("using iFFCF4_BUFFER_SIZE = %d\n", iFFCF4_BUFFER_SIZE);

        printf("\nusing iAP1_BUFFER_SIZE = %d\n", iAP1_BUFFER_SIZE);
        printf("using iAP2_BUFFER_SIZE = %d\n", iAP2_BUFFER_SIZE);
        printf("using iAP3_BUFFER_SIZE = %d\n", iAP3_BUFFER_SIZE);
    }

    printf("data_length = %d\tread = %d\tinput_size = %d \n", data_length, read, input_size);
    printf("sample_rate = %d\tbits_per_sample = %d\tchannels = %d \n", sample_rate, bits_per_sample, channels);

    int numSamples = read/2;
    for(unsigned int n = 0; n < numSamples; n++)
    {
        const uint8_t* in = &input_buf[2*n];
        convert_buf[n] = in[0] | (in[1] << 8);
    }
    int pitchOff = 0;
    // iterate over the audio frames and create three oscillators, seperated in phase by PI/2
    for(unsigned int n = 0; n < numSamples; n+=channels)
    {
        // Read audio inputs
        if(channels == 1)
        {
            inL = (int32_t)convert_buf[n];///(1<<15);
            inR = inL;
        }
        else if(channels == 2)
        {
            // interleaved left right channel
            inL = (int32_t)convert_buf[n];///(1<<15);
            inR = (int32_t)convert_buf[n+1];///(1<<15);
        }
        else
        {
            fprintf(stderr, "channel = %d\n", channels);
            return -1;
        }

        float fInput = (inL + inR) * 0.5f;

        // Process
        float ap = fInput;

        ap = processAP(ap, fAP1_GAIN, fAP1, &iAP1, iAP1_BUFFER_SIZE);
        ap = processAP(ap, fAP2_GAIN, fAP2, &iAP2, iAP2_BUFFER_SIZE);
        ap = processAP(ap, fAP3_GAIN, fAP3, &iAP3, iAP3_BUFFER_SIZE);
        
        float fOutput = processFFCF(ap, fFFCF1_GAIN, fFFCF1, &iFFCF1, iFFCF1_BUFFER_SIZE);
        fOutput = hardClip(fOutput + processFFCF(ap, fFFCF2_GAIN, fFFCF2, &iFFCF2, iFFCF2_BUFFER_SIZE));
        fOutput = hardClip(fOutput + processFFCF(ap, fFFCF3_GAIN, fFFCF3, &iFFCF3, iFFCF3_BUFFER_SIZE));
        fOutput = hardClip(fOutput + processFFCF(ap, fFFCF4_GAIN, fFFCF4, &iFFCF4, iFFCF4_BUFFER_SIZE));

        fOutput *= dryWet/100.f;
        fOutput += (1.f - dryWet/100.f) * fInput;


        int16_t oL = (int16_t)fOutput;
        int16_t oR = (int16_t)fOutput;
        
        wav_write_data(wavOut, (unsigned char*)&oL, 2);

        if(channels > 1)
        {
            wav_write_data(wavOut, (unsigned char*)&oR, 2);
        }
    }    

    free(convert_buf);
    free(input_buf);
    
    cleanup();

    wav_write_close(wavOut);
    wav_read_close(wavIn);

    return 0;
}
// cleanup_render() is called once at the end, after the audio has stopped.
// Release any resources that were allocated in initialise_render().

void cleanup()
{
    if(fFFCF1)
        free(fFFCF1);
    if(fFFCF2)
        free(fFFCF2);
    if(fFFCF3)
        free(fFFCF3);
    if(fFFCF4)
        free(fFFCF4);

    if(fAP1)
        free(fAP1);
    if(fAP2)
        free(fAP2);
    if(fAP3)
        free(fAP3);
}
