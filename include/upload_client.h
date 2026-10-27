#pragma once
#include <Arduino.h>

// Pushes a file from SD to the configured upload backend (device_config.h:
// upload_host/upload_port/upload_api_key). The installed WalterModem
// release caps a single HTTP request body at 65535 bytes, so larger files
// are split into sequential chunk POSTs:
//
//   POST /upload/<name>?part=<n>&total=<N>&final=<0|1>
//   Authorization: Basic base64(<api_key>:)
//   Body: raw chunk bytes
//
// The backend (a small server the user runs themselves, out of scope for
// this firmware) is expected to reassemble parts by name and finalize on
// final=1, then forward the complete file to S3-compatible storage.
//
// Only the daily positions CSV is uploaded (see main.cpp): the MiniSEED and
// GNSS raw files are far too large for LTE-M (~230 MB/day of MiniSEED
// alone) and stay on the SD card for physical retrieval.

// Queues the upload as a modem job (see modem.h) and returns immediately -
// false if one is already in flight or it couldn't be queued. A missing
// local file (no rows that day) counts as success; an unavailable SD card
// does not.
bool uploadFileAsync(const char *localPath, const char *remoteFilename);
bool uploadBusy();

enum UploadResult : uint8_t { UPLOAD_NONE, UPLOAD_OK, UPLOAD_FAILED };
// Consumes: reports the finished upload's result once, UPLOAD_NONE otherwise.
UploadResult uploadTakeResult();
