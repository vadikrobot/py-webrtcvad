/*
 *  Copyright (c) 2014 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef COMMON_AUDIO_WAV_FILE_H_
#define COMMON_AUDIO_WAV_FILE_H_

#include <memory>

namespace webrtc {

// Interface to provide access to WAV file parameters.
class WavFile {
 public:
  virtual ~WavFile() {}

  virtual int sample_rate() const = 0;
  virtual size_t num_channels() const = 0;
  virtual size_t num_samples() const = 0;
};

class WavWriterInterface : public WavFile {
 public:
  // Write additional samples to the file. Each sample is in the range
  // [-32768,32767], and there must be the previously specified number of
  // interleaved channels.
  virtual void WriteSamples(const float* samples, size_t num_samples) = 0;
  virtual void WriteSamples(const int16_t* samples, size_t num_samples) = 0;
};

class WavReaderInterface : public WavFile {
 public:
  // Resets position to the beginning of the file.
  virtual void Reset() = 0;

  // Returns the number of samples read. If this is less than requested,
  // verifies that the end of the file was reached.
  virtual size_t ReadSamples(size_t num_samples, float* samples) = 0;
  virtual size_t ReadSamples(size_t num_samples, int16_t* samples) = 0;
};

std::unique_ptr<WavWriterInterface> CreateWavWriter(const std::string& filename, int sample_rate, size_t num_channels);
std::unique_ptr<WavReaderInterface> CreateWavReader(const std::string& filename);

}  // namespace webrtc

#endif  // COMMON_AUDIO_WAV_FILE_H_
