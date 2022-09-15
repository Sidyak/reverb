/*
 ____  _____ _        _
| __ )| ____| |      / \
|  _ \|  _| | |     / _ \
| |_) | |___| |___ / ___ \
|____/|_____|_____/_/   \_\
http://bela.io

Author: Kim Radmacher
Description: Schroeders Reverb 
Date: 15.09.2022

*/

#include <Bela.h>
#include <math.h>

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

// Set the analog channels to read from
int gAudioFramesPerAnalogFrame = 0;

// cpu cycle read
static inline uint32_t ccnt_read (void)
{
  uint32_t cc = 0;
  __asm__ volatile ("mrc p15, 0, %0, c9, c13, 0":"=r" (cc));
  return cc;
}

bool setup(BelaContext *context, void *userData)
{
    // Useful calculations
    if(context->analogFrames)
        gAudioFramesPerAnalogFrame = context->audioFrames / context->analogFrames;

    printf("go setup\n");
    printf("context->audioFrames = %d\n", context->audioFrames);
    printf("context->audioSampleRate = %f\n", context->audioSampleRate);
    printf("context->audioInChannels = %d\n", context->audioInChannels);
    printf("context->audioOutChannels = %d\n", context->audioOutChannels);

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

    // Check that we have the same number of inputs and outputs.
    if(context->audioInChannels != context->audioOutChannels ||
            context->analogInChannels != context-> analogOutChannels){
        printf("Error: for this project, you need the same number of input and output channels.\n");
        return false;
    }
    return true;
}

static float dryWet = 0;
static float modDryWet = 0;
static float roomSize = 0;
static float modRoomSize = 0;

int modParamInt(int param, int* pParam, int range)
{
    if(param < 0)
    {
        if(param <= *pParam-range)
        {
            *pParam -= range;
        }
        else if(param >= *pParam+range)
        {
            *pParam += range;
        }
    }
    else
    {
        if(param >= *pParam+range)
        {
            *pParam += range;
        }
        else if(param < *pParam)
        {
            *pParam -= range;
        }
    }

    param = *pParam;
    return param;
}

float modParamFloat(float param, float* pParam, float range)
{
    if(param < 0)
    {
        if(param <= *pParam-range)
        {
            *pParam -= range;
        }
        else if(param >= *pParam+range)
        {
            *pParam += range;
        }
    }
    else
    {
        if(param >= *pParam+range)
        {
            *pParam += range;
        }
        else if(param < *pParam)
        {
            *pParam -= range;
        }
    }

    param = *pParam;
    return param;
}


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

void render(BelaContext *context, void *userData)
{
    for(unsigned int n = 0; n < context->audioFrames; n++){

        uint32_t t0 = ccnt_read();
        uint32_t t1 = t0;//ccnt_read();       
        //rt_printf("%u\n", t1-t0);

        if(gAudioFramesPerAnalogFrame && !(n % gAudioFramesPerAnalogFrame)) {
            if(n==0)
            {
                dryWet = (float)map(analogRead(context, n/gAudioFramesPerAnalogFrame, 0), 0, 1, 0, 105)/100.f;
                dryWet = modParamFloat(dryWet, &modDryWet, 0.1f);

                roomSize = (float)map(analogRead(context, n/gAudioFramesPerAnalogFrame, 1), 0, 1, 0, 105)/100.f;
                roomSize = modParamFloat(roomSize, &modRoomSize, 0.1f);

                iFFCF1_BUFFER_SIZE = (int)(expf(2.9 * roomSize) * iFFCF1_BUFFER_SIZE);
                iFFCF2_BUFFER_SIZE = (int)(expf(2.9 * roomSize) * iFFCF2_BUFFER_SIZE);
                iFFCF3_BUFFER_SIZE = (int)(expf(2.9 * roomSize) * iFFCF3_BUFFER_SIZE);
                iFFCF4_BUFFER_SIZE = (int)(expf(2.9 * roomSize) * iFFCF4_BUFFER_SIZE);
                iAP1_BUFFER_SIZE = (int)(expf(2.9 * roomSize) * iAP1_BUFFER_SIZE);
                iAP2_BUFFER_SIZE = (int)(expf(2.9 * roomSize) * iAP2_BUFFER_SIZE);
                iAP3_BUFFER_SIZE = (int)(expf(2.9 * roomSize) * iAP3_BUFFER_SIZE);
            }
        }

        float fInL = 0;
        float fInR = 0;

        // Read audio inputs
        fInL = audioRead(context,n,0);
        fInR = audioRead(context,n,1);

        float fInput = (fInL + fInR) * 0.5f;

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

        t1 = ccnt_read();
        //rt_printf("\r\r\rdryWet = %f, roomSize = %f ####  %u cycles process", roomSize, t1-t0);    
//rt_printf("\rfOutput = %f", fOutput);

        // Write the output sample
        audioWrite(context, n, 0, fOutput);
        audioWrite(context, n, 1, fOutput);
    }

}

void cleanup(BelaContext *context, void *userData)
{

}
