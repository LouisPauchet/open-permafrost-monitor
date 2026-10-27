#pragma once
#include <Arduino.h>

// Thin wrapper around the quickspot/WalterModem library (Sequans GM02SP
// LTE-M/NB-IoT modem on the Walter ESP32-S3 board). Centralizes the
// AT-command modem's async, event-driven API behind simple blocking-style
// calls so the rest of the firmware doesn't need to know about it directly.

bool modemInit();        // one-time: attach to the modem chip + PDP context
bool modemConnect();     // ensure network-registered; cheap if already attached
bool modemIsConnected();

// Deregisters from the LTE-M network (NO_RF) between measurement cycles -
// call once queued modem jobs have actually finished (modemJobsBusy()
// false), not right after kicking them off. modemConnect() brings it back
// (setOpState(FULL) + wait for registration) the next time it's needed, same
// as gnss.h's gnssSleep()/gnssWakeup() pair.
void modemSleep();

// Raw TCP socket (used by ntrip_client.cpp for the RTCM stream). Only one
// socket is used at a time.
//
// Command mode (AT+SQNSSEND/AT+SQNSRECV), not the Sequans modem's
// "online"/transparent socket mode - that was tried (AT+SQNSD connMode=0,
// a direct UART<->socket pipe with no AT framing at all) specifically to
// avoid this async command-queue machinery, but live hardware testing
// showed the caster never receiving anything at all once dialed that way,
// across multiple otherwise-correct variations (matching WalterModem's own
// proven +SQNSD parameter list exactly, disabling hardware flow control,
// waiting up to 20s for a reply) - this Sequans firmware revision doesn't
// seem to support it reliably regardless of what the client does right.
// Reverted to command mode, which is proven to work (sub-200ms caster
// response, 85KB+/cycle of verified-real RTCM).
bool modemSocketOpen(const char *host, uint16_t port);
bool modemSocketSend(const uint8_t *data, size_t len);
size_t modemSocketAvailable();
size_t modemSocketRead(uint8_t *buf, size_t maxLen);
void modemSocketClose();

// True while an async socket receive is outstanding. A send submitted at
// the same moment can collide with it on WalterModem's command channel
// (see modem.cpp's async receive design notes) - callers with a send that
// can wait a moment (e.g. ntripSendGGA's periodic GGA relay) should check
// this and defer rather than risk it.
bool modemReceiveBusy();

// Holds off any *new* async receive kickoff for the given duration, so a
// priority send (GGA) gets one guaranteed-clear shot at the AT channel
// instead of racing the (often near-continuous) receive side. Does not
// affect a receive that's already in flight when called.
void modemSuppressReceiveFor(unsigned long ms);

// True if the most recent async send (since the last call to this
// function) failed - e.g. it was successfully queued but the modem later
// answered with an error. Distinct from modemSocketSend()'s own return
// value, which only reflects whether the command was queued, not whether
// it actually succeeded (that's only known later, asynchronously).
// Consumes the flag: only reports failures new since the last check.
bool modemLastSendFailed();

// Modem worker: runs blocking AT sequences (MQTT batch, HTTP upload) one at
// a time on a dedicated task, so they never block loopTask (and the task
// watchdog) and never overlap each other. fn runs on the worker task; ctx
// must stay valid until it has run. Returns false if the modem isn't
// initialized or the queue is full.
typedef void (*ModemJobFn)(void *ctx);
bool modemRunJob(ModemJobFn fn, void *ctx);
bool modemJobsBusy(); // true while any job is queued or running

// MQTT (used by mqtt_client.cpp for batched telemetry) - stages the batch
// and queues config->connect->publish->disconnect as one modem job.
// Returns true if it was queued, not that the publish itself succeeded -
// check modemMqttBatchBusy()/modemMqttLastBatchOk() later for that.
bool modemMqttPublishBatchAsync(const char *host, uint16_t port, const char *clientId,
                                 const char *user, const char *pass, const char *caCert,
                                 const char *topic, const uint8_t *payload, size_t len);
bool modemMqttBatchBusy();
bool modemMqttLastBatchOk(); // consumes: true only once, right when a batch succeeds

// HTTP (used by upload_client.cpp for the daily upload POST) - blocking,
// call only from a modem job (see modemRunJob()). Auth is HTTP
// Basic (api key as the username, empty password) rather than a bearer
// header - the installed WalterModem release (1.5.0) doesn't expose a way
// to set a custom Authorization header on httpSend, only httpConfigProfile's
// built-in basic-auth support.
bool modemHttpPost(const char *host, uint16_t port, const char *path,
                    const char *apiKey, const uint8_t *body, size_t bodyLen);
