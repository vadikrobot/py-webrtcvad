/*
 *  Copyright (c) 2014 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "webrtc/common_audio/wav_file.h"

#include <errno.h>

#include <algorithm>
#include <cstdio>
#include <type_traits>
#include <utility>

#include <stdint.h>

#include <cstddef>
#include <string>

#include "webrtc/rtc_base/constructor_magic.h"
#include "webrtc/rtc_base/system/file_wrapper.h"

#include "webrtc/common_audio/include/audio_util.h"
#include "webrtc/common_audio/wav_header.h"
#include "webrtc/rtc_base/checks.h"
// #include "webrtc/rtc_base/logging.h"
#include "webrtc/rtc_base/system/arch.h"

namespace webrtc {
namespace {

// We write 16-bit PCM WAV files.
constexpr WavFormat kWavFormat = kWavFormatPcm;
static_assert(std::is_trivially_destructible<WavFormat>::value, "");
constexpr size_t kBytesPerSample = 2;

// Doesn't take ownership of the file handle and won't close it.
class ReadableWavFile : public ReadableWav {
 public:
  explicit ReadableWavFile(FileWrapper* file) : file_(file) {}
  ReadableWavFile(const ReadableWavFile&) = delete;
  ReadableWavFile& operator=(const ReadableWavFile&) = delete;
  size_t Read(void* buf, size_t num_bytes) override {
    size_t count = file_->Read(buf, num_bytes);
    pos_ += count;
    return count;
  }
  bool SeekForward(uint32_t num_bytes) override {
    bool success = file_->SeekRelative(num_bytes);
    if (success) {
      pos_ += num_bytes;
    }
    return success;
  }
  int64_t GetPosition() { return pos_; }

 private:
  FileWrapper* file_;
  int64_t pos_ = 0;
};

}  // namespace

// Simple C++ class for writing 16-bit PCM WAV files. All error handling is
// by calls to RTC_CHECK(), making it unsuitable for anything but debug code.
class WavWriter final : public WavWriterInterface {
 public:
  // Open a new WAV file for writing.
  WavWriter(const std::string& filename, int sample_rate, size_t num_channels);

  // Open a new WAV file for writing.
  WavWriter(FileWrapper file, int sample_rate, size_t num_channels);

  // Close the WAV file, after writing its header.
  ~WavWriter() override;

  // Write additional samples to the file. Each sample is in the range
  // [-32768,32767], and there must be the previously specified number of
  // interleaved channels.
  void WriteSamples(const float* samples, size_t num_samples) override;
  void WriteSamples(const int16_t* samples, size_t num_samples) override;

  int sample_rate() const override;
  size_t num_channels() const override;
  size_t num_samples() const override;

 private:
  void Close();
  const int sample_rate_;
  const size_t num_channels_;
  size_t num_samples_;  // Total number of samples written to file.
  FileWrapper file_;    // Output file, owned by this class

  RTC_DISALLOW_COPY_AND_ASSIGN(WavWriter);
};

// Follows the conventions of WavWriter.
class WavReader final : public WavReaderInterface {
 public:
  // Opens an existing WAV file for reading.
  explicit WavReader(const std::string& filename);

  // Use an existing WAV file for reading.
  explicit WavReader(FileWrapper file);

  // Close the WAV file.
  ~WavReader() override;

  // Resets position to the beginning of the file.
  void Reset() override;

  // Returns the number of samples read. If this is less than requested,
  // verifies that the end of the file was reached.
  size_t ReadSamples(size_t num_samples, float* samples) override;
  size_t ReadSamples(size_t num_samples, int16_t* samples) override;

  int sample_rate() const override;
  size_t num_channels() const override;
  size_t num_samples() const override;

 private:
  void Close();
  int sample_rate_;
  size_t num_channels_;
  size_t num_samples_;  // Total number of samples in the file.
  size_t num_samples_remaining_;
  FileWrapper file_;  // Input file, owned by this class.
  int64_t
      data_start_pos_;  // Position in the file immediately after WAV header.

  RTC_DISALLOW_COPY_AND_ASSIGN(WavReader);
};

WavReader::WavReader(const std::string& filename)
    : WavReader(FileWrapper::OpenReadOnly(filename)) {}

WavReader::WavReader(FileWrapper file) : file_(std::move(file)) {
  RTC_CHECK(file_.is_open())
      << "Invalid file. Could not create file handle for wav file.";

  ReadableWavFile readable(&file_);
  WavFormat format;
  size_t bytes_per_sample;
  RTC_CHECK(ReadWavHeader(&readable, &num_channels_, &sample_rate_, &format,
                          &bytes_per_sample, &num_samples_));
  num_samples_remaining_ = num_samples_;
  RTC_CHECK_EQ(kWavFormat, format);
  RTC_CHECK_EQ(kBytesPerSample, bytes_per_sample);
  data_start_pos_ = readable.GetPosition();
}

WavReader::~WavReader() {
  Close();
}

void WavReader::Reset() {
  RTC_CHECK(file_.SeekTo(data_start_pos_))
      << "Failed to set position in the file to WAV data start position";
  num_samples_remaining_ = num_samples_;
}

int WavReader::sample_rate() const {
  return sample_rate_;
}

size_t WavReader::num_channels() const {
  return num_channels_;
}

size_t WavReader::num_samples() const {
  return num_samples_;
}

size_t WavReader::ReadSamples(size_t num_samples, int16_t* samples) {
#ifndef WEBRTC_ARCH_LITTLE_ENDIAN
#error "Need to convert samples to big-endian when reading from WAV file"
#endif
  // There could be metadata after the audio; ensure we don't read it.
  num_samples = std::min(num_samples, num_samples_remaining_);
  const size_t num_bytes = num_samples * sizeof(*samples);
  const size_t read_bytes = file_.Read(samples, num_bytes);
  // If we didn't read what was requested, ensure we've reached the EOF.
  RTC_CHECK(read_bytes == num_bytes || file_.ReadEof());
  RTC_CHECK_EQ(read_bytes % 2, 0)
      << "End of file in the middle of a 16-bit sample";
  const size_t read_samples = read_bytes / 2;
  RTC_CHECK_LE(read_samples, num_samples_remaining_);
  num_samples_remaining_ -= read_samples;
  return read_samples;
}

size_t WavReader::ReadSamples(size_t num_samples, float* samples) {
  static const size_t kChunksize = 4096 / sizeof(uint16_t);
  size_t read = 0;
  for (size_t i = 0; i < num_samples; i += kChunksize) {
    int16_t isamples[kChunksize];
    size_t chunk = std::min(kChunksize, num_samples - i);
    chunk = ReadSamples(chunk, isamples);
    for (size_t j = 0; j < chunk; ++j)
      samples[i + j] = isamples[j];
    read += chunk;
  }
  return read;
}

void WavReader::Close() {
  file_.Close();
}

WavWriter::WavWriter(const std::string& filename,
                     int sample_rate,
                     size_t num_channels)
    // Unlike plain fopen, OpenWriteOnly takes care of filename utf8 ->
    // wchar conversion on windows.
    : WavWriter(FileWrapper::OpenWriteOnly(filename),
                sample_rate,
                num_channels) {}

WavWriter::WavWriter(FileWrapper file, int sample_rate, size_t num_channels)
    : sample_rate_(sample_rate),
      num_channels_(num_channels),
      num_samples_(0),
      file_(std::move(file)) {
  // Handle errors from the OpenWriteOnly call in above constructor.
  RTC_CHECK(file_.is_open()) << "Invalid file. Could not create wav file.";

  RTC_CHECK(CheckWavParameters(num_channels_, sample_rate_, kWavFormat,
                               kBytesPerSample, num_samples_));

  // Write a blank placeholder header, since we need to know the total number
  // of samples before we can fill in the real data.
  static const uint8_t blank_header[kWavHeaderSize] = {0};
  RTC_CHECK(file_.Write(blank_header, kWavHeaderSize));
}

WavWriter::~WavWriter() {
  Close();
}

int WavWriter::sample_rate() const {
  return sample_rate_;
}

size_t WavWriter::num_channels() const {
  return num_channels_;
}

size_t WavWriter::num_samples() const {
  return num_samples_;
}

void WavWriter::WriteSamples(const int16_t* samples, size_t num_samples) {
#ifndef WEBRTC_ARCH_LITTLE_ENDIAN
#error "Need to convert samples to little-endian when writing to WAV file"
#endif
  RTC_CHECK(file_.Write(samples, sizeof(*samples) * num_samples));
  num_samples_ += num_samples;
  RTC_CHECK(num_samples_ >= num_samples);  // detect size_t overflow
}

void WavWriter::WriteSamples(const float* samples, size_t num_samples) {
  static const size_t kChunksize = 4096 / sizeof(uint16_t);
  for (size_t i = 0; i < num_samples; i += kChunksize) {
    int16_t isamples[kChunksize];
    const size_t chunk = std::min(kChunksize, num_samples - i);
    FloatS16ToS16(samples + i, chunk, isamples);
    WriteSamples(isamples, chunk);
  }
}

void WavWriter::Close() {
  RTC_CHECK(file_.Rewind());
  uint8_t header[kWavHeaderSize];
  WriteWavHeader(header, num_channels_, sample_rate_, kWavFormat,
                 kBytesPerSample, num_samples_);
  RTC_CHECK(file_.Write(header, kWavHeaderSize));
  RTC_CHECK(file_.Close());
}

std::unique_ptr<WavWriterInterface> CreateWavWriter(const std::string& filename, int sample_rate, size_t num_channels) {
  return std::make_unique<WavWriter>(filename, sample_rate, num_channels);
}

std::unique_ptr<WavReaderInterface> CreateWavReader(const std::string& filename) {
  return std::make_unique<WavReader>(filename);
}

}  // namespace webrtc
