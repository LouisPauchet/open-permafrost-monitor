#pragma once
#include <Arduino.h>

// Pushes a file from SD to the configured upload backend (device_config.h:
// upload_host/upload_port/upload_api_key). The installed WalterModem
// release caps a single HTTP request body at 65535 bytes, so files larger
// than that (routine for the daily MiniSEED/GNSS-raw/NTRIP logs) are split
// into sequential chunk POSTs:
//
//   POST /upload/<name>?part=<n>&total=<N>&final=<0|1>
//   Authorization: Basic base64(<api_key>:)
//   Body: raw chunk bytes
//
// The backend (a small server the user runs themselves, out of scope for
// this firmware) is expected to reassemble parts by name and finalize on
// final=1, then forward the complete file to S3-compatible storage.
bool uploadFile(const char *localPath, const char *remoteFilename);
