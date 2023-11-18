#ifndef WAVEFOLDER_H
#define WAVEFOLDER_H

#include <stdint.h>
#include "daisy.h"

using namespace daisy;

void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size);


#endif // WAVEFOLDER_H
